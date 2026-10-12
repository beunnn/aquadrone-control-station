// Follows the retained MQTT topic aquadrone/target_sysid (published by the Tab5) and reports presence.
// Uses the ESP-IDF esp-mqtt client, which runs its own task and delivers events to onMqttEvent.
#include "selection.h"

#include <inttypes.h>
#include <string.h>
#include "cJSON.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "numericTypes.h"
#include "sdkconfig.h"

#define TOPIC_TARGET  "aquadrone/target_sysid"
#define TOPIC_STATUS  "aquadrone/ctrl/ctrlbox/status"
#define SYSID_INVALID ( -1 )
#define SYSID_MIN     1
#define SYSID_MAX     254
#define KEEPALIVE_S   5

static const char* logTag = "selection";

static volatile int32_t selectedSysid = 0;

// Returns the sysid from {"sysid": N, ...}, 0 for an empty (cleared) payload, SYSID_INVALID otherwise.
static int32_t parseSysid( const char* data, int32_t length )
{
    int32_t sysid = SYSID_INVALID;

    if ( length == 0 ) { sysid = 0; }
    else
    {
        // cJSON returns NULL for a parse error or missing key, and every call below accepts NULL.
        cJSON* root = cJSON_ParseWithLength( data, ( size_t )length );

        const cJSON* value = cJSON_GetObjectItemCaseSensitive( root, "sysid" );

        // valuedouble == valueint rejects fractional numbers such as 2.5.
        if ( ( cJSON_IsNumber( value ) != 0 ) && ( value->valuedouble == ( float64_t )value->valueint ) && ( value->valueint >= SYSID_MIN ) && ( value->valueint <= SYSID_MAX ) && ( value->valueint != CONFIG_MAV_SYSID ) ) { sysid = ( int32_t )value->valueint; }

        cJSON_Delete( root );
    }

    return sysid;
}

// Applies a message received on the subscribed topics; only TOPIC_TARGET is subscribed.
static void onMessage( const esp_mqtt_event_t* event )
{
    const bool isTarget = ( event->topic_len == ( int )strlen( TOPIC_TARGET ) ) && ( memcmp( event->topic, TOPIC_TARGET, ( size_t )event->topic_len ) == 0 );

    if ( !isTarget ) { return; }

    if ( event->data_len != event->total_data_len )
    {
        ESP_LOGW( logTag, "fragmented selection message ignored" );
        return;
    }

    const int32_t sysid = parseSysid( event->data, ( int32_t )event->data_len );

    if ( sysid < 0 )
    {
        ESP_LOGW( logTag, "invalid selection payload '%.*s' ignored", event->data_len, event->data );
        return;
    }

    if ( sysid != selectedSysid ) { ESP_LOGI( logTag, "selection %" PRId32 " -> %" PRId32, selectedSysid, sysid ); }

    selectedSysid = sysid;
}

// esp-mqtt event handler, runs in the MQTT client task.
static void onMqttEvent( void* handlerArg, esp_event_base_t base, int32_t eventId, void* eventData )
{
    const esp_mqtt_event_handle_t event = ( esp_mqtt_event_handle_t )eventData;

    ( void )handlerArg;
    ( void )base;

    switch ( ( esp_mqtt_event_id_t )eventId )
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI( logTag, "broker connected" );
            ( void )esp_mqtt_client_publish( event->client, TOPIC_STATUS, "online", 0, 1, 1 );
            ( void )esp_mqtt_client_subscribe( event->client, TOPIC_TARGET, 1 );
            break;

        case MQTT_EVENT_DISCONNECTED:
            // A broker outage must not drop control of the boat, so the last selection is kept.
            ESP_LOGW( logTag, "broker disconnected, keeping target %" PRId32, selectedSysid );
            break;

        case MQTT_EVENT_DATA:
            onMessage( event );
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGW( logTag, "MQTT error" );
            break;

        default:
            break;
    }
}

void selectionStart( void )
{
    const esp_mqtt_client_config_t config = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .credentials.username = CONFIG_MQTT_USER,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,
        .session.keepalive = KEEPALIVE_S,
        .session.last_will = { .topic = TOPIC_STATUS, .msg = "offline", .qos = 1, .retain = 1 },
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init( &config );

    ESP_ERROR_CHECK( esp_mqtt_client_register_event( client, ESP_EVENT_ANY_ID, onMqttEvent, NULL ) );
    ESP_ERROR_CHECK( esp_mqtt_client_start( client ) );
}

int32_t selectionGet( void ) { return selectedSysid; }
