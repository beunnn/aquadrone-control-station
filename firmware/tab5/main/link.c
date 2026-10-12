// MAVLink link to mavlink-router: discovers vehicles from their heartbeats and tracks their status.
// The Tab5 never sends commands or overrides; the heartbeat only lets the router learn our address.
#include "link.h"

#include <inttypes.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "mavlink_headers_cfg.h"
#include "sdkconfig.h"

#define RX_BUFFER_LEN       512U
#define RX_ERROR_DELAY_MS   100U
#define HEARTBEAT_PERIOD_MS 1000U

static const char* logTag = "link";
static int udpSocket      = -1;
static struct sockaddr_in hubAddress;
static portMUX_TYPE tableLock    = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t lastRxUs = 0;

// Kept sorted by sysid so the UI shows a stable order.
static vehicle_t vehicles[LINK_MAX_VEHICLES];
static int32_t vehicleCount = 0;

// Returns the entry for sysid, inserting it in sysid order if create is set; NULL if absent or the table is full.
// Call with tableLock held.
static vehicle_t* findVehicle( uint8_t sysid, bool create )
{
    int32_t index      = 0;
    vehicle_t* vehicle = NULL;

    while ( ( index < vehicleCount ) && ( vehicles[index].sysid < sysid ) )
    {
        index++;
    }

    if ( ( index < vehicleCount ) && ( vehicles[index].sysid == sysid ) ) { vehicle = &vehicles[index]; }
    else if ( create && ( vehicleCount < LINK_MAX_VEHICLES ) )
    {
        ( void )memmove( &vehicles[index + 1], &vehicles[index], ( size_t )( vehicleCount - index ) * sizeof( vehicle_t ) );
        vehicleCount++;
        vehicles[index] = ( vehicle_t ){ .sysid = sysid, .batteryPercent = LINK_BATT_UNKNOWN, .gpsSats = LINK_SATS_UNKNOWN };

        vehicle = &vehicles[index];
    }
    else { vehicle = NULL; }

    return vehicle;
}

// Updates the vehicle table from one parsed message; only the autopilot component of each system is tracked.
static void handleMessage( const mavlink_message_t* message )
{
    const int64_t now = esp_timer_get_time();

    if ( message->compid != MAV_COMP_ID_AUTOPILOT1 ) { return; }

    if ( message->msgid == MAVLINK_MSG_ID_HEARTBEAT )
    {
        mavlink_heartbeat_t heartbeat;

        mavlink_msg_heartbeat_decode( message, &heartbeat );

        // GCS heartbeats (control box, Mission Planner, other Tab5s) are not vehicles.
        if ( ( heartbeat.type == MAV_TYPE_GCS ) || ( heartbeat.autopilot == MAV_AUTOPILOT_INVALID ) ) { return; }

        taskENTER_CRITICAL( &tableLock );
        vehicle_t* vehicle = findVehicle( message->sysid, true );

        if ( vehicle != NULL )
        {
            vehicle->type  = heartbeat.type;
            vehicle->armed = ( heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED ) != 0U;
            vehicle->mode  = heartbeat.custom_mode;

            vehicle->lastHeartbeatUs = now;
        }

        taskEXIT_CRITICAL( &tableLock );
    }
    else if ( message->msgid == MAVLINK_MSG_ID_SYS_STATUS )
    {
        mavlink_sys_status_t status;

        mavlink_msg_sys_status_decode( message, &status );
        taskENTER_CRITICAL( &tableLock );
        vehicle_t* vehicle = findVehicle( message->sysid, false );

        if ( vehicle != NULL )
        {
            vehicle->hasBattery     = status.voltage_battery != UINT16_MAX;
            vehicle->batteryMv      = status.voltage_battery;
            vehicle->batteryPercent = status.battery_remaining;
        }

        taskEXIT_CRITICAL( &tableLock );
    }
    else if ( message->msgid == MAVLINK_MSG_ID_GPS_RAW_INT )
    {
        mavlink_gps_raw_int_t gps;

        mavlink_msg_gps_raw_int_decode( message, &gps );
        taskENTER_CRITICAL( &tableLock );
        vehicle_t* vehicle = findVehicle( message->sysid, false );

        if ( vehicle != NULL )
        {
            vehicle->hasGps  = true;
            vehicle->gpsFix  = gps.fix_type;
            vehicle->gpsSats = gps.satellites_visible;
        }

        taskEXIT_CRITICAL( &tableLock );
    }
    else
    {
        // Other messages are not shown on the selector.
    }
}

static void rxTask( void* arg )
{
    uint8_t buffer[RX_BUFFER_LEN];
    mavlink_message_t message;
    mavlink_status_t status;

    ( void )arg;

    for ( ;; )
    {
        const int32_t received = ( int32_t )recv( udpSocket, buffer, sizeof( buffer ), 0 );

        // Socket errors (e.g. while Wi-Fi is down) would otherwise spin this task.
        if ( received <= 0 )
        {
            vTaskDelay( pdMS_TO_TICKS( RX_ERROR_DELAY_MS ) );
            continue;
        }

        lastRxUs = esp_timer_get_time();

        for ( int32_t index = 0; index < received; index++ )
        {
            if ( mavlink_parse_char( MAVLINK_COMM_0, buffer[index], &message, &status ) != 0U ) { handleMessage( &message ); }
        }
    }
}

static void heartbeatTask( void* arg )
{
    ( void )arg;

    for ( ;; )
    {
        mavlink_message_t message;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        ( void )mavlink_msg_heartbeat_pack( CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &message, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0U, 0U, MAV_STATE_ACTIVE );
        const uint16_t length = mavlink_msg_to_send_buffer( buffer, &message );

        // Fails harmlessly while Wi-Fi is down.
        ( void )sendto( udpSocket, buffer, length, 0, ( const struct sockaddr* )&hubAddress, sizeof( hubAddress ) );
        vTaskDelay( pdMS_TO_TICKS( HEARTBEAT_PERIOD_MS ) );
    }
}

void linkStart( void )
{
    // A fixed source port keeps the router's learned endpoint stable across reboots.
    const struct sockaddr_in localAddress = { .sin_family = AF_INET, .sin_port = htons( CONFIG_LOCAL_PORT ), .sin_addr.s_addr = htonl( INADDR_ANY ) };

    udpSocket = socket( AF_INET, SOCK_DGRAM, IPPROTO_IP );
    ( void )memset( &hubAddress, 0, sizeof( hubAddress ) );
    hubAddress.sin_family = AF_INET;
    hubAddress.sin_port   = htons( CONFIG_HUB_PORT );
    ( void )inet_pton( AF_INET, CONFIG_HUB_IP, &hubAddress.sin_addr );

    if ( bind( udpSocket, ( const struct sockaddr* )&localAddress, sizeof( localAddress ) ) != 0 ) { ESP_LOGE( logTag, "bind to port %d failed", CONFIG_LOCAL_PORT ); }

    ( void )xTaskCreate( rxTask, "mav_rx", 8192, NULL, 5, NULL );
    ( void )xTaskCreate( heartbeatTask, "mav_hb", 4096, NULL, 4, NULL );
}

int32_t linkVehicles( vehicle_t* out, int32_t maxCount )
{
    int32_t count = 0;

    taskENTER_CRITICAL( &tableLock );
    count = ( vehicleCount < maxCount ) ? vehicleCount : maxCount;
    ( void )memcpy( out, vehicles, ( size_t )count * sizeof( vehicle_t ) );
    taskEXIT_CRITICAL( &tableLock );

    return count;
}

int64_t linkLastRxUs( void ) { return lastRxUs; }
