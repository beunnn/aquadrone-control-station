// USB HID host reader for the Thrustmaster Solaris Base (report layout in docs/joystick.md).
// Uses the ESP-IDF USB host library: a library task services the bus, a client task receives attach/detach events,
// and input reports arrive in the interrupt-IN transfer callback.
#include "joystick.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "usb/usb_host.h"

#define JS_VID             0x044FU
#define JS_PID             0x0422U
#define JS_IFACE           0U
#define JS_EP_IN           0x81U
#define JS_REPORT_LEN      64U
#define JS_REPORT_ID       1U
#define FEATURE_REPORT_LEN 64U
#define CONTROL_TIMEOUT_MS 1000U
#define CONTROL_WAIT_MS    2000U
#define SETUP_DELAY_MS     1500U

static const char* logTag = "joystick";

static usb_host_client_handle_t usbClient;
static usb_device_handle_t usbDevice;
static usb_transfer_t* inTransfer;
static volatile bool deviceConnected;
static joystickState_t latestState;
static portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t controlDone;

static uint16_t readLe16( const uint8_t* bytes ) { return ( uint16_t )( ( uint16_t )bytes[0] | ( uint16_t )( ( uint16_t )bytes[1] << 8U ) ); }

// Decodes one input report into latestState; called from the interrupt-IN transfer callback.
static void parseReport( const uint8_t* report, uint32_t length )
{
    joystickState_t state;
    uint32_t vendorLength = 0U;

    if ( ( length < JOYSTICK_VENDOR_OFFSET ) || ( report[0] != JS_REPORT_ID ) ) { return; }

    ( void )memset( &state, 0, sizeof( state ) );

    for ( uint32_t byte = 0U; byte < 6U; byte++ )
    {
        state.buttons |= ( uint64_t )report[1U + byte] << ( 8U * byte );
    }

    state.buttons &= ( 1ULL << JOYSTICK_BUTTON_COUNT ) - 1ULL;
    state.hat    = ( uint8_t )( report[12] & 0x0FU );
    state.x      = readLe16( &report[13] );
    state.y      = readLe16( &report[15] );
    state.z      = readLe16( &report[17] );
    state.rz     = readLe16( &report[19] );
    state.ry     = readLe16( &report[21] );
    state.rx     = readLe16( &report[23] );
    state.slider = readLe16( &report[25] );
    state.dial   = readLe16( &report[27] );

    vendorLength = ( length >= JS_REPORT_LEN ) ? JOYSTICK_VENDOR_LEN : ( length - JOYSTICK_VENDOR_OFFSET );
    ( void )memcpy( state.vendor, &report[JOYSTICK_VENDOR_OFFSET], vendorLength );

    portENTER_CRITICAL( &stateLock );
    state.seq   = latestState.seq + 1U;
    latestState = state;
    portEXIT_CRITICAL( &stateLock );
}

// Interrupt-IN completion callback (USB host library); resubmits so reports keep streaming.
static void onInTransfer( usb_transfer_t* transfer )
{
    if ( transfer->status == USB_TRANSFER_STATUS_COMPLETED )
    {
        parseReport( transfer->data_buffer, ( uint32_t )transfer->actual_num_bytes );
        ( void )usb_host_transfer_submit( transfer );
    }
    else if ( ( transfer->status != USB_TRANSFER_STATUS_NO_DEVICE ) && ( transfer->status != USB_TRANSFER_STATUS_CANCELED ) )
    {
        ESP_LOGW( logTag, "transfer status %d, resubmitting", ( int )transfer->status );
        ( void )usb_host_transfer_submit( transfer );
    }
    else
    {
        // Device gone or transfer cancelled: the detach handler frees the transfer.
    }
}

// Control-transfer completion callback (USB host library); wakes hidSetupTask.
static void onControlDone( usb_transfer_t* transfer )
{
    ( void )transfer;
    ( void )xSemaphoreGive( controlDone );
}

// Submits the control request already prepared in transfer and waits for it to complete. Used by hidSetupTask only.
static bool submitControl( usb_transfer_t* transfer, uint32_t length )
{
    transfer->device_handle    = usbDevice;
    transfer->bEndpointAddress = 0U;

    transfer->num_bytes  = ( int )length;
    transfer->callback   = onControlDone;
    transfer->timeout_ms = CONTROL_TIMEOUT_MS;

    return ( usb_host_transfer_submit_control( usbClient, transfer ) == ESP_OK ) && ( xSemaphoreTake( controlDone, pdMS_TO_TICKS( CONTROL_WAIT_MS ) ) == pdTRUE );
}

// One-shot task started at attach. The slider only reports after the standard HID SET_PROTOCOL + SET_IDLE requests that
// a Linux/Windows host sends (docs/joystick.md). The vendor feature reports are only read and logged, for diagnostics.
static void hidSetupTask( void* arg )
{
    typedef struct
    {
        uint8_t request;
        uint16_t value;
        const char* name;
    } classRequest_t;

    static const classRequest_t classRequests[] = { { 0x0BU, 1U, "SET_PROTOCOL(report)" }, { 0x0AU, 0U, "SET_IDLE(0)" } };

    static const uint8_t featureIds[] = { 0xF1U, 0xF2U, 0xF3U };

    usb_transfer_t* transfer = NULL;

    ( void )arg;
    vTaskDelay( pdMS_TO_TICKS( SETUP_DELAY_MS ) );

    if ( usbDevice == NULL ) { vTaskDelete( NULL ); }

    controlDone = xSemaphoreCreateBinary();
    ( void )usb_host_transfer_alloc( sizeof( usb_setup_packet_t ) + FEATURE_REPORT_LEN, 0, &transfer );

    for ( uint32_t index = 0U; ( index < ( sizeof( classRequests ) / sizeof( classRequests[0] ) ) ) && ( usbDevice != NULL ); index++ )
    {
        usb_setup_packet_t* setup = ( usb_setup_packet_t* )transfer->data_buffer;

        // Host-to-device, class, interface.
        setup->bmRequestType = 0x21U;
        setup->bRequest      = classRequests[index].request;

        setup->wValue  = classRequests[index].value;
        setup->wIndex  = JS_IFACE;
        setup->wLength = 0U;

        const bool done = submitControl( transfer, sizeof( usb_setup_packet_t ) );
        ESP_LOGI( logTag, "%s: %s (status %d)", classRequests[index].name, done ? "done" : "request failed", done ? ( int )transfer->status : -1 );
    }

    for ( uint32_t index = 0U; ( index < sizeof( featureIds ) ) && ( usbDevice != NULL ); index++ )
    {
        usb_setup_packet_t* setup = ( usb_setup_packet_t* )transfer->data_buffer;

        // Device-to-host, class, interface; GET_REPORT of a feature report (type 3) with the given id.
        setup->bmRequestType = 0xA1U;
        setup->bRequest      = 0x01U;

        setup->wValue  = ( uint16_t )( ( 3U << 8U ) | featureIds[index] );
        setup->wIndex  = JS_IFACE;
        setup->wLength = FEATURE_REPORT_LEN;

        if ( !submitControl( transfer, sizeof( usb_setup_packet_t ) + FEATURE_REPORT_LEN ) ) { ESP_LOGW( logTag, "feature 0x%02X: request failed", featureIds[index] ); }
        else
        {
            const int32_t received = ( int32_t )transfer->actual_num_bytes - ( int32_t )sizeof( usb_setup_packet_t );
            char line[( 3U * FEATURE_REPORT_LEN ) + 8U];
            int32_t offset = 0;

            line[0] = '\0';

            for ( int32_t byte = 0; ( byte < received ) && ( byte < ( int32_t )FEATURE_REPORT_LEN ); byte++ )
            {
                offset += snprintf( &line[offset], sizeof( line ) - ( size_t )offset, "%02x ", transfer->data_buffer[sizeof( usb_setup_packet_t ) + ( size_t )byte] );
            }

            ESP_LOGI( logTag, "feature 0x%02X status %d len %" PRId32 ": %s", featureIds[index], ( int )transfer->status, received, line );
        }
    }

    ( void )usb_host_transfer_free( transfer );
    vTaskDelete( NULL );
}

// Opens a newly attached USB device and, if it is the Solaris Base, claims its HID interface and starts reading.
static void openDevice( uint8_t address )
{
    usb_device_handle_t device = NULL;

    const usb_device_desc_t* descriptor = NULL;

    esp_err_t err = ESP_OK;

    if ( usb_host_device_open( usbClient, address, &device ) != ESP_OK ) { return; }

    ( void )usb_host_get_device_descriptor( device, &descriptor );
    ESP_LOGI( logTag, "USB device %04x:%04x", descriptor->idVendor, descriptor->idProduct );

    if ( ( descriptor->idVendor != JS_VID ) || ( descriptor->idProduct != JS_PID ) )
    {
        ( void )usb_host_device_close( usbClient, device );
        return;
    }

    err = usb_host_interface_claim( usbClient, device, JS_IFACE, 0U );

    if ( err != ESP_OK )
    {
        ESP_LOGE( logTag, "claim failed: %s", esp_err_to_name( err ) );
        ( void )usb_host_device_close( usbClient, device );
        return;
    }

    ( void )usb_host_transfer_alloc( JS_REPORT_LEN, 0, &inTransfer );
    inTransfer->device_handle    = device;
    inTransfer->bEndpointAddress = JS_EP_IN;

    inTransfer->num_bytes  = ( int )JS_REPORT_LEN;
    inTransfer->callback   = onInTransfer;
    inTransfer->timeout_ms = 0U;

    usbDevice = device;

    // seq stays 0 until the first real report, so a button held at attach does not look like a fresh press.
    portENTER_CRITICAL( &stateLock );
    ( void )memset( &latestState, 0, sizeof( latestState ) );
    portEXIT_CRITICAL( &stateLock );

    deviceConnected = true;
    ESP_LOGI( logTag, "Solaris Base attached, reading reports" );
    ( void )xTaskCreate( hidSetupTask, "hidsetup", 4096, NULL, 4, NULL );
    ( void )usb_host_transfer_submit( inTransfer );
}

// Releases everything openDevice acquired; called when the Solaris Base is unplugged.
static void closeDevice( void )
{
    ESP_LOGW( logTag, "Solaris Base removed" );
    deviceConnected = false;
    ( void )usb_host_endpoint_halt( usbDevice, JS_EP_IN );
    ( void )usb_host_endpoint_flush( usbDevice, JS_EP_IN );
    ( void )usb_host_interface_release( usbClient, usbDevice, JS_IFACE );
    ( void )usb_host_device_close( usbClient, usbDevice );
    ( void )usb_host_transfer_free( inTransfer );
    inTransfer = NULL;
    usbDevice  = NULL;
}

// Attach/detach events from the USB host library, delivered in clientTask.
static void onClientEvent( const usb_host_client_event_msg_t* message, void* arg )
{
    ( void )arg;

    if ( message->event == USB_HOST_CLIENT_EVENT_NEW_DEV ) { openDevice( message->new_dev.address ); }
    else if ( ( message->event == USB_HOST_CLIENT_EVENT_DEV_GONE ) && ( usbDevice != NULL ) && ( message->dev_gone.dev_hdl == usbDevice ) ) { closeDevice(); }
    else
    {
        // Other devices and events are ignored.
    }
}

// Services the USB host library (bus events, enumeration).
static void hostTask( void* arg )
{
    ( void )arg;

    for ( ;; )
    {
        uint32_t eventFlags = 0U;
        ( void )usb_host_lib_handle_events( portMAX_DELAY, &eventFlags );
    }
}

// Delivers client events to onClientEvent.
static void clientTask( void* arg )
{
    ( void )arg;

    for ( ;; )
    {
        ( void )usb_host_client_handle_events( usbClient, portMAX_DELAY );
    }
}

void joystickStart( void )
{
    const usb_host_config_t hostConfig = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };

    const usb_host_client_config_t clientConfig = { .is_synchronous = false, .max_num_event_msg = 5, .async = { .client_event_callback = onClientEvent, .callback_arg = NULL } };

    ESP_ERROR_CHECK( usb_host_install( &hostConfig ) );
    ESP_ERROR_CHECK( usb_host_client_register( &clientConfig, &usbClient ) );
    ( void )xTaskCreate( hostTask, "usb_host", 4096, NULL, 10, NULL );
    ( void )xTaskCreate( clientTask, "usb_client", 4096, NULL, 9, NULL );
}

bool joystickGet( joystickState_t* out )
{
    bool valid = false;

    if ( deviceConnected && ( latestState.seq != 0U ) )
    {
        portENTER_CRITICAL( &stateLock );
        *out = latestState;
        portEXIT_CRITICAL( &stateLock );
        valid = true;
    }

    return valid;
}
