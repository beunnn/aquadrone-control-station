// Control box (Waveshare ESP32-P4 dev kit): joystick and buttons to MAVLink RC overrides and commands over Ethernet/UDP
// to mavlink-router, for the vehicle selected on the shared MQTT topic.
#include <inttypes.h>
#include <string.h>
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"
#include "buttons.h"
#include "joystick.h"
#include "mavlink_headers_cfg.h"
#include "numericTypes.h"
#include "sdkconfig.h"
#include "selection.h"

// Waveshare ESP32-P4-Module-DEV-KIT: IP101GRI PHY over RMII with an external 50 MHz clock (pins from the schematic).
#define ETH_MDC_GPIO      31
#define ETH_MDIO_GPIO     52
#define ETH_RST_GPIO      51
#define ETH_CLK_GPIO      50
#define ETH_PHY_ADDR_AUTO ( -1 )
#define GOT_IP_BIT        BIT0

// Fixed local port so mavlink-router keeps the same learned endpoint across reboots.
#define LOCAL_UDP_PORT 14561U
#define RX_BUFFER_LEN  512U

#define RC_IGNORE           UINT16_MAX
#define RC_LOW              1000U
#define RC_NEUTRAL          1500U
#define RC_HIGH             2000U
#define DEADBAND            0.08f
#define EXPO                0.30f
#define NEUTRAL_BURST_COUNT 5

// ArduPilot Rover custom modes.
#define ROVER_MODE_MANUAL 0
#define ROVER_MODE_HOLD   4
#define ROVER_MODE_AUTO   10
#define ROVER_MODE_RTL    11

#define CONTROL_PERIOD_MS  50U
#define BOOT_HOLDOFF_US    5000000
#define ARM_HOLD_TICKS     16
#define ESTOP_HOLD_TICKS   6
#define ESTOP_PULSE_US     3000000
#define ESTOP_REPEAT_US    200000
#define FORCE_DISARM_MAGIC 21196.0f
#define VEHICLE_LOST_US    3000000
#define OVERRIDE_LOG_EVERY 20U
#define CMD_RETRY_US       1000000
#define CMD_MAX_TRIES      3

// A discrete command (mode, arm, disarm) waiting for its COMMAND_ACK; resent until acknowledged or out of tries.
typedef struct
{
    bool active;
    uint16_t command;
    float32_t param1;
    float32_t param2;
    int32_t tries;
    int64_t sentUs;
    const char* name;
} pendingCommand_t;

static const char* logTag = "ctrlbox";
static EventGroupHandle_t networkEvents;
static int udpSocket = -1;
static struct sockaddr_in hubAddress;

// Selected vehicle, 0 = none (nothing is sent). Only controlTask changes it.
static volatile int32_t targetSysid = 0;

// Vehicle state, written by rxTask and read by controlTask. heartbeatSysid tags lastHeartbeatUs so that a late heartbeat
// from the previously selected vehicle is not trusted for the new one.
static volatile int32_t heartbeatSysid  = 0;
static volatile int64_t lastHeartbeatUs = 0;
static volatile bool vehicleArmed       = false;
static volatile uint32_t vehicleMode    = 0U;

static pendingCommand_t pendingCommand;
static volatile int32_t ackCommand = -1;
static volatile int32_t ackResult  = 0;

// IP_EVENT_ETH_GOT_IP handler (ESP-IDF default event loop); releases app_main.
static void onGotIp( void* arg, esp_event_base_t base, int32_t eventId, void* eventData )
{
    const ip_event_got_ip_t* event = ( const ip_event_got_ip_t* )eventData;

    ( void )arg;
    ( void )base;
    ( void )eventId;
    ESP_LOGI( logTag, "IP " IPSTR " gw " IPSTR, IP2STR( &event->ip_info.ip ), IP2STR( &event->ip_info.gw ) );
    ( void )xEventGroupSetBits( networkEvents, GOT_IP_BIT );
}

// ETH_EVENT handler (ESP-IDF default event loop); link state is only logged.
static void onEthernetEvent( void* arg, esp_event_base_t base, int32_t eventId, void* eventData )
{
    const char* state = "other";

    ( void )arg;
    ( void )base;
    ( void )eventData;

    if ( eventId == ETHERNET_EVENT_CONNECTED ) { state = "link up"; }
    else if ( eventId == ETHERNET_EVENT_DISCONNECTED ) { state = "link down"; }
    else { state = "other"; }

    ESP_LOGI( logTag, "ETH event %" PRId32 " (%s)", eventId, state );
}

// Brings up the on-board Ethernet with DHCP; the IP arrives later in onGotIp.
static void ethernetStart( void )
{
    esp_netif_config_t netifConfig = ESP_NETIF_DEFAULT_ETH();
    eth_mac_config_t macConfig     = ETH_MAC_DEFAULT_CONFIG();

    eth_esp32_emac_config_t emacConfig = ETH_ESP32_EMAC_DEFAULT_CONFIG();

    eth_phy_config_t phyConfig = ETH_PHY_DEFAULT_CONFIG();
    esp_eth_handle_t ethHandle = NULL;

    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    esp_netif_t* netif = esp_netif_new( &netifConfig );

    emacConfig.smi_gpio.mdc_num  = ETH_MDC_GPIO;
    emacConfig.smi_gpio.mdio_num = ETH_MDIO_GPIO;

    emacConfig.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emacConfig.clock_config.rmii.clock_gpio = ETH_CLK_GPIO;

    esp_eth_mac_t* mac = esp_eth_mac_new_esp32( &emacConfig, &macConfig );

    phyConfig.phy_addr       = ETH_PHY_ADDR_AUTO;
    phyConfig.reset_gpio_num = ETH_RST_GPIO;
    esp_eth_phy_t* phy       = esp_eth_phy_new_ip101( &phyConfig );

    const esp_eth_config_t ethConfig = ETH_DEFAULT_CONFIG( mac, phy );
    ESP_ERROR_CHECK( esp_eth_driver_install( &ethConfig, &ethHandle ) );
    ESP_ERROR_CHECK( esp_netif_attach( netif, esp_eth_new_netif_glue( ethHandle ) ) );
    ESP_ERROR_CHECK( esp_event_handler_register( ETH_EVENT, ESP_EVENT_ANY_ID, onEthernetEvent, NULL ) );
    ESP_ERROR_CHECK( esp_event_handler_register( IP_EVENT, IP_EVENT_ETH_GOT_IP, onGotIp, NULL ) );
    ESP_ERROR_CHECK( esp_eth_start( ethHandle ) );
}

static void sendMessage( const mavlink_message_t* message )
{
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer( buffer, message );

    ( void )sendto( udpSocket, buffer, length, 0, ( const struct sockaddr* )&hubAddress, sizeof( hubAddress ) );
}

// Sends (or resends) the pending command to the selected vehicle; the confirmation field counts the attempts.
static void commandTransmit( pendingCommand_t* command )
{
    mavlink_message_t message;

    ( void )mavlink_msg_command_long_pack( CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &message, ( uint8_t )targetSysid, MAV_COMP_ID_AUTOPILOT1, command->command, ( uint8_t )command->tries, command->param1, command->param2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
    sendMessage( &message );
    command->sentUs = esp_timer_get_time();
    command->tries++;
}

// Starts a new discrete command; replaces any command still waiting for its ACK.
static void commandStart( const char* name, uint16_t command, float32_t param1, float32_t param2 )
{
    if ( targetSysid == 0 )
    {
        ESP_LOGW( logTag, "cmd %s ignored: no vehicle selected", name );
        return;
    }

    pendingCommand = ( pendingCommand_t ){ .active = true, .command = command, .param1 = param1, .param2 = param2, .tries = 0, .sentUs = 0, .name = name };
    ackCommand     = -1;
    commandTransmit( &pendingCommand );
    ESP_LOGI( logTag, "cmd %s sent", name );
}

// Called every control cycle: finishes the pending command on its ACK, otherwise retries every second.
static void commandService( void )
{
    if ( !pendingCommand.active ) { return; }

    if ( ackCommand == ( int32_t )pendingCommand.command )
    {
        ESP_LOGI( logTag, "cmd %s ACK result %" PRId32 " (%s)", pendingCommand.name, ackResult, ( ackResult == 0 ) ? "accepted" : "REJECTED" );
        pendingCommand.active = false;
    }
    else if ( ( esp_timer_get_time() - pendingCommand.sentUs ) > CMD_RETRY_US )
    {
        if ( pendingCommand.tries >= CMD_MAX_TRIES )
        {
            ESP_LOGW( logTag, "cmd %s: no ACK after %" PRId32 " tries", pendingCommand.name, pendingCommand.tries );
            pendingCommand.active = false;
        }
        else { commandTransmit( &pendingCommand ); }
    }
    else
    {
        // Still waiting for the ACK.
    }
}

// ArduPilot expects a 1 Hz heartbeat from a control source; it also lets the router learn this endpoint.
static void heartbeatTask( void* arg )
{
    ( void )arg;

    for ( ;; )
    {
        mavlink_message_t message;

        ( void )mavlink_msg_heartbeat_pack( CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &message, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE );
        sendMessage( &message );
        vTaskDelay( pdMS_TO_TICKS( 1000U ) );
    }
}

// Maps a raw axis to -1..1 with deadband and expo; the deadband also hides the ~7 % cross-talk between X and Y.
static float32_t axisNormalise( uint16_t raw )
{
    float32_t value  = ( float32_t )( ( int32_t )raw - 32768 ) / 32767.0f;
    float32_t shaped = 0.0f;

    if ( value > 1.0f ) { value = 1.0f; }

    if ( value < -1.0f ) { value = -1.0f; }

    const float32_t magnitude = ( value < 0.0f ) ? -value : value;

    if ( magnitude >= DEADBAND )
    {
        const float32_t scaled = ( magnitude - DEADBAND ) / ( 1.0f - DEADBAND );

        shaped = ( ( 1.0f - EXPO ) * scaled ) + ( EXPO * scaled * scaled * scaled );
    }

    return ( value < 0.0f ) ? -shaped : shaped;
}

// The boat's companion computer rejects RC values outside 1000..2000 on the channels it checks (1, 3, 6, 7, 10).
static uint16_t rcClamp( int32_t value )
{
    uint16_t clamped = RC_NEUTRAL;

    if ( value < ( int32_t )RC_LOW ) { clamped = RC_LOW; }
    else if ( value > ( int32_t )RC_HIGH ) { clamped = RC_HIGH; }
    else { clamped = ( uint16_t )value; }

    return clamped;
}

static uint16_t toPwm( float32_t normalised ) { return rcClamp( ( int32_t )( ( float32_t )RC_NEUTRAL + ( 500.0f * normalised ) + 0.5f ) ); }

// The slider is descriptor axis Z (verified on the bench, docs/joystick.md), inverted on request: bottom = 2000 us, top = 1000 us.
static uint16_t speedScaleRaw( const joystickState_t* joystick ) { return ( uint16_t )( UINT16_MAX - joystick->z ); }

// RC10 throttle speed scale for the companion computer: 0..65535 maps linearly onto 1000..2000 us.
static uint16_t speedScalePwm( uint16_t raw ) { return rcClamp( ( int32_t )RC_LOW + ( int32_t )( ( ( uint32_t )raw * 1000UL ) / 65535UL ) ); }

// Unused channels are UINT16_MAX, which ArduPilot treats as "ignore this channel".
static void sendOverride( int32_t target, uint16_t steer, uint16_t throttle, uint16_t motor, uint16_t trim, uint16_t speed )
{
    mavlink_message_t message;

    ( void )mavlink_msg_rc_channels_override_pack( CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &message, ( uint8_t )target, MAV_COMP_ID_AUTOPILOT1, steer, RC_IGNORE, throttle, RC_IGNORE, RC_IGNORE, motor, trim, RC_IGNORE, RC_IGNORE, speed, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE,
                                                   RC_IGNORE, RC_IGNORE );
    sendMessage( &message );
}

// A few neutral, motor-off overrides so a vehicle we stop controlling is left stopped rather than on its last stick values.
static void sendNeutralBurst( int32_t target, uint16_t speed )
{
    for ( int32_t burst = 0; burst < NEUTRAL_BURST_COUNT; burst++ )
    {
        sendOverride( target, RC_NEUTRAL, RC_NEUTRAL, RC_LOW, RC_NEUTRAL, speed );
    }
}

static bool inMask( uint32_t mask, buttonId_t button ) { return ( mask & BTN_BIT( button ) ) != 0U; }

// 20 Hz control loop: selection changes, e-stop, override enable, motor latch, discrete commands and the RC override stream.
static void controlTask( void* arg )
{
    bool overrideEnabled = false;

    bool motorOn      = false;
    bool estopActive  = false;
    bool estopRearmed = true;

    bool vehicleOkPrevious = false;
    int64_t estopEndUs     = 0;

    int64_t lastEstopCommandUs = 0;

    int32_t estopTicks     = 0;
    int32_t armTicks       = 0;
    uint32_t overrideCount = 0U;
    uint16_t lastSpeed     = RC_LOW;
    TickType_t lastWake    = xTaskGetTickCount();

    ( void )arg;

    for ( ;; )
    {
        vTaskDelayUntil( &lastWake, pdMS_TO_TICKS( CONTROL_PERIOD_MS ) );
        const uint32_t held   = buttonsHeld();
        const uint32_t events = buttonsPressedEvents();
        const int64_t now     = esp_timer_get_time();

        // Boot hold-off: all button activity is ignored for the first 5 s.
        if ( now < BOOT_HOLDOFF_US ) { continue; }

        // Shared vehicle selection; deferred during an e-stop pulse so the stop completes on the boat it was meant for.
        const int32_t requested = selectionGet();

        if ( ( requested != targetSysid ) && !estopActive )
        {
            const int32_t previous = targetSysid;

            if ( overrideEnabled && ( previous != 0 ) ) { sendNeutralBurst( previous, lastSpeed ); }

            motorOn  = false;
            armTicks = 0;

            pendingCommand.active = false;

            vehicleArmed = false;
            vehicleMode  = 0U;
            targetSysid  = requested;
            ESP_LOGW( logTag, "target vehicle %" PRId32 " -> %" PRId32 " (motor off)", previous, requested );
        }

        const bool vehicleOk = ( targetSysid != 0 ) && ( heartbeatSysid == targetSysid ) && ( lastHeartbeatUs != 0 ) && ( ( now - lastHeartbeatUs ) < VEHICLE_LOST_US );

        if ( vehicleOk != vehicleOkPrevious )
        {
            ESP_LOGW( logTag, "vehicle %s", vehicleOk ? "heartbeat OK" : "heartbeat LOST" );
            vehicleOkPrevious = vehicleOk;
        }

        if ( !vehicleOk && motorOn )
        {
            motorOn = false;
            ESP_LOGW( logTag, "motor OFF (vehicle lost)" );
        }

        // Emergency shutdown: both buttons held for 300 ms give a 3 s stop pulse, which then clears itself.
        // Both buttons must be fully released before another stop can fire.
        const bool bothEstopHeld = inMask( held, buttonEstopA ) && inMask( held, buttonEstopB );

        if ( !inMask( held, buttonEstopA ) && !inMask( held, buttonEstopB ) ) { estopRearmed = true; }

        estopTicks = bothEstopHeld ? ( estopTicks + 1 ) : 0;

        if ( !estopActive && estopRearmed && ( estopTicks >= ESTOP_HOLD_TICKS ) )
        {
            estopActive  = true;
            estopRearmed = false;
            estopEndUs   = now + ESTOP_PULSE_US;
            motorOn      = false;

            pendingCommand.active = false;
            ESP_LOGE( logTag, "EMERGENCY SHUTDOWN: stop pulse for %" PRId32 " s", ( int32_t )( ESTOP_PULSE_US / 1000000 ) );
        }

        if ( estopActive && ( now >= estopEndUs ) )
        {
            estopActive = false;
            ESP_LOGW( logTag, "emergency shutdown pulse ended, controls released (motor stays off)" );
        }

        // Force disarm (param2 magic value), repeated until the vehicle reports disarmed.
        if ( estopActive && vehicleArmed && ( targetSysid != 0 ) && ( ( now - lastEstopCommandUs ) > ESTOP_REPEAT_US ) )
        {
            mavlink_message_t message;

            ( void )mavlink_msg_command_long_pack( CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &message, ( uint8_t )targetSysid, MAV_COMP_ID_AUTOPILOT1, MAV_CMD_COMPONENT_ARM_DISARM, 0U, 0.0f, FORCE_DISARM_MAGIC, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f );
            sendMessage( &message );
            lastEstopCommandUs = now;
        }

        if ( inMask( events, buttonOverrideToggle ) )
        {
            if ( overrideEnabled )
            {
                overrideEnabled = false;

                motorOn = false;

                if ( targetSysid != 0 ) { sendNeutralBurst( targetSysid, lastSpeed ); }

                ESP_LOGW( logTag, "RC override DISABLED (no override messages are sent)" );
            }
            else
            {
                overrideEnabled = true;

                motorOn = false;
                ESP_LOGW( logTag, "RC override ENABLED (motor off)" );
            }
        }

        // Motor on is only accepted with the override enabled, outside an e-stop and with a live vehicle.
        if ( inMask( events, buttonMotorOff ) )
        {
            motorOn = false;
            ESP_LOGI( logTag, "motor OFF" );
        }

        if ( inMask( events, buttonMotorOn ) )
        {
            if ( overrideEnabled && !estopActive && vehicleOk )
            {
                motorOn = true;
                ESP_LOGI( logTag, "motor ON" );
            }
            else { ESP_LOGW( logTag, "motor ON refused: override %s, e-stop pulse %s, vehicle heartbeat %s", overrideEnabled ? "enabled" : "DISABLED (press the override button)", estopActive ? "ACTIVE" : "inactive", vehicleOk ? "ok" : "LOST" ); }
        }

        // Discrete commands are blocked during the e-stop pulse; disarm is always allowed.
        if ( !estopActive )
        {
            const float32_t customMode = ( float32_t )MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

            if ( inMask( events, buttonMode1 ) ) { commandStart( "mode Manual", MAV_CMD_DO_SET_MODE, customMode, ( float32_t )ROVER_MODE_MANUAL ); }

            if ( inMask( events, buttonMode2 ) ) { commandStart( "mode Hold", MAV_CMD_DO_SET_MODE, customMode, ( float32_t )ROVER_MODE_HOLD ); }

            if ( inMask( events, buttonMode3 ) ) { commandStart( "mode Auto", MAV_CMD_DO_SET_MODE, customMode, ( float32_t )ROVER_MODE_AUTO ); }

            if ( inMask( events, buttonMode4 ) ) { commandStart( "mode RTL", MAV_CMD_DO_SET_MODE, customMode, ( float32_t )ROVER_MODE_RTL ); }

            // Arm sits on the trigger: only a deliberate 0.8 s hold arms.
            armTicks = inMask( held, buttonArm ) ? ( armTicks + 1 ) : 0;

            if ( armTicks == ARM_HOLD_TICKS ) { commandStart( "arm", MAV_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f ); }
        }

        if ( inMask( events, buttonDisarm ) ) { commandStart( "disarm", MAV_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f ); }

        commandService();

        joystickState_t joystick;

        if ( overrideEnabled && ( targetSysid != 0 ) && joystickGet( &joystick ) )
        {
            const bool driving   = motorOn && !estopActive;
            const bool trimUp    = inMask( held, buttonTrimUp );
            const bool trimDown  = inMask( held, buttonTrimDown );
            const uint16_t speed = speedScalePwm( speedScaleRaw( &joystick ) );
            const uint16_t motor = driving ? RC_HIGH : RC_LOW;
            uint16_t steer       = toPwm( axisNormalise( joystick.x ) );

            uint16_t trim = RC_NEUTRAL;

            // Forward is Y low on this stick, hence the sign flip.
            const uint16_t throttle = driving ? toPwm( -axisNormalise( joystick.y ) ) : RC_NEUTRAL;

            // Both trim buttons held counts as neither.
            if ( trimUp != trimDown ) { trim = trimUp ? RC_HIGH : RC_LOW; }

            if ( estopActive )
            {
                steer = RC_NEUTRAL;
                trim  = RC_NEUTRAL;
            }

            lastSpeed = speed;
            sendOverride( targetSysid, steer, throttle, motor, trim, speed );
            overrideCount++;

            if ( ( overrideCount % OVERRIDE_LOG_EVERY ) == 0U ) { ESP_LOGI( logTag, "RC override -> sys %" PRId32 ": ch1 %u ch3 %u ch6 %u ch7 %u ch10 %u | veh %s mode %" PRIu32, targetSysid, steer, throttle, motor, trim, speed, vehicleArmed ? "ARMED" : "disarmed", vehicleMode ); }
        }
    }
}

// Receives MAVLink from the hub and keeps the selected vehicle's heartbeat state and command ACKs.
static void rxTask( void* arg )
{
    uint8_t buffer[RX_BUFFER_LEN];
    mavlink_message_t message;
    mavlink_status_t status;

    ( void )arg;

    for ( ;; )
    {
        const int32_t received = ( int32_t )recv( udpSocket, buffer, sizeof( buffer ), 0 );

        for ( int32_t index = 0; index < received; index++ )
        {
            const int32_t target = targetSysid;

            if ( ( mavlink_parse_char( MAVLINK_COMM_0, buffer[index], &message, &status ) != 0U ) && ( target != 0 ) && ( ( int32_t )message.sysid == target ) )
            {
                if ( ( message.msgid == MAVLINK_MSG_ID_HEARTBEAT ) && ( message.compid == MAV_COMP_ID_AUTOPILOT1 ) )
                {
                    mavlink_heartbeat_t heartbeat;

                    mavlink_msg_heartbeat_decode( &message, &heartbeat );
                    vehicleArmed    = ( heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED ) != 0U;
                    vehicleMode     = heartbeat.custom_mode;
                    heartbeatSysid  = ( int32_t )message.sysid;
                    lastHeartbeatUs = esp_timer_get_time();
                }
                else if ( message.msgid == MAVLINK_MSG_ID_COMMAND_ACK )
                {
                    mavlink_command_ack_t ack;

                    mavlink_msg_command_ack_decode( &message, &ack );
                    ackResult  = ( int32_t )ack.result;
                    ackCommand = ( int32_t )ack.command;
                }
                else
                {
                    // Other messages from the vehicle are not used here.
                }
            }
        }
    }
}

void app_main( void )
{
    const struct sockaddr_in localAddress = { .sin_family = AF_INET, .sin_port = htons( LOCAL_UDP_PORT ), .sin_addr.s_addr = htonl( INADDR_ANY ) };

    ESP_ERROR_CHECK( nvs_flash_init() );
    buttonsStart();
    joystickStart();
    networkEvents = xEventGroupCreate();
    ethernetStart();
    ( void )xEventGroupWaitBits( networkEvents, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY );

    udpSocket = socket( AF_INET, SOCK_DGRAM, IPPROTO_IP );
    ( void )memset( &hubAddress, 0, sizeof( hubAddress ) );
    hubAddress.sin_family = AF_INET;
    hubAddress.sin_port   = htons( CONFIG_HUB_PORT );
    ( void )inet_pton( AF_INET, CONFIG_HUB_IP, &hubAddress.sin_addr );
    ( void )bind( udpSocket, ( const struct sockaddr* )&localAddress, sizeof( localAddress ) );
    selectionStart();

    ( void )xTaskCreate( heartbeatTask, "hb", 4096, NULL, 5, NULL );
    ( void )xTaskCreate( rxTask, "rx", 8192, NULL, 5, NULL );
    ( void )xTaskCreate( controlTask, "ctrl", 6144, NULL, 6, NULL );
}
