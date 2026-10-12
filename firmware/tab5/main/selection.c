// Shared vehicle selection over MQTT: publishes taps to the retained topic aquadrone/target_sysid and shows only what
// the broker echoes back, so the display always matches what the control box follows.
// Uses the ESP-IDF esp-mqtt client, which runs its own task and delivers events to onMqttEvent.
#include "selection.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "cJSON.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h"

#define TOPIC_TARGET  "aquadrone/target_sysid"
#define TOPIC_STATUS  "aquadrone/ctrl/+/status"
#define TOPIC_OWN     "aquadrone/ctrl/tab5/status"
#define TOPIC_CTRLBOX "aquadrone/ctrl/ctrlbox/status"
#define SYSID_INVALID ( -1 )
#define SYSID_MIN     1
#define SYSID_MAX     254
#define KEEPALIVE_S   5
#define PAYLOAD_LEN   112U

// Before this time (2023-11) the clock has not been set by SNTP yet, so no timestamp is published.
#define TIME_VALID_S 1700000000

static const char* logTag = "selection";
static esp_mqtt_client_handle_t mqttClient;
static volatile bool brokerConnected = false;
static volatile int32_t currentSysid = 0;
static volatile int32_t pendingSysid = 0;
static volatile int32_t lastSeq      = 0;

static volatile devState_t ctrlboxState = devUnknown;

static bool topicIs( const esp_mqtt_event_t* event, const char* topic ) { return ( event->topic_len == ( int )strlen( topic ) ) && ( memcmp( event->topic, topic, ( size_t )event->topic_len ) == 0 ); }

// Applies a retained or live selection message; an empty payload means the selection was cleared.
// The highest seq seen is remembered so our next publication continues the sequence of all publishers.
static void onTargetMessage( const char* data, int32_t length )
{
    int32_t sysid = 0;

    if ( length > 0 )
    {
        // cJSON returns NULL for a parse error or missing key, and every call below accepts NULL.
        cJSON* root = cJSON_ParseWithLength( data, ( size_t )length );

        const cJSON* value   = cJSON_GetObjectItemCaseSensitive( root, "sysid" );
        const cJSON* seqItem = cJSON_GetObjectItemCaseSensitive( root, "seq" );

        sysid = ( ( cJSON_IsNumber( value ) != 0 ) && ( value->valueint >= SYSID_MIN ) && ( value->valueint <= SYSID_MAX ) ) ? ( int32_t )value->valueint : SYSID_INVALID;

        if ( ( cJSON_IsNumber( seqItem ) != 0 ) && ( ( int32_t )seqItem->valueint > lastSeq ) ) { lastSeq = ( int32_t )seqItem->valueint; }

        cJSON_Delete( root );
    }

    if ( sysid < 0 )
    {
        ESP_LOGW( logTag, "invalid selection payload '%.*s' ignored", ( int )length, data );
        return;
    }

    ESP_LOGI( logTag, "selection confirmed: %" PRId32, sysid );
    currentSysid = sysid;
    pendingSysid = 0;
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
            brokerConnected = true;
            ( void )esp_mqtt_client_publish( event->client, TOPIC_OWN, "online", 0, 1, 1 );
            ( void )esp_mqtt_client_subscribe( event->client, TOPIC_TARGET, 1 );
            ( void )esp_mqtt_client_subscribe( event->client, TOPIC_STATUS, 1 );
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW( logTag, "broker disconnected" );
            brokerConnected = false;
            pendingSysid    = 0;
            ctrlboxState    = devUnknown;
            break;

        case MQTT_EVENT_DATA:
            // Fragments only occur for payloads larger than the client buffer; selection and status messages are tiny.
            if ( event->data_len != event->total_data_len ) { break; }

            if ( topicIs( event, TOPIC_TARGET ) ) { onTargetMessage( event->data, ( int32_t )event->data_len ); }
            else if ( topicIs( event, TOPIC_CTRLBOX ) ) { ctrlboxState = ( ( event->data_len == 6 ) && ( memcmp( event->data, "online", 6U ) == 0 ) ) ? devOnline : devOffline; }
            else
            {
                // Other devices' status topics are not shown yet.
            }

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
        .session.last_will = { .topic = TOPIC_OWN, .msg = "offline", .qos = 1, .retain = 1 },
    };

    mqttClient = esp_mqtt_client_init( &config );
    ESP_ERROR_CHECK( esp_mqtt_client_register_event( mqttClient, ESP_EVENT_ANY_ID, onMqttEvent, NULL ) );
    ESP_ERROR_CHECK( esp_mqtt_client_start( mqttClient ) );
}

bool selectionRequest( int32_t sysid )
{
    char payload[PAYLOAD_LEN];
    const int32_t seq = lastSeq + 1;
    const time_t now  = time( NULL );

    if ( !brokerConnected ) { return false; }

    if ( now > TIME_VALID_S ) { ( void )snprintf( payload, sizeof( payload ), "{\"sysid\":%" PRId32 ",\"src\":\"tab5\",\"seq\":%" PRId32 ",\"ts\":%lld}", sysid, seq, ( long long )now ); }
    else { ( void )snprintf( payload, sizeof( payload ), "{\"sysid\":%" PRId32 ",\"src\":\"tab5\",\"seq\":%" PRId32 "}", sysid, seq ); }

    if ( esp_mqtt_client_publish( mqttClient, TOPIC_TARGET, payload, 0, 1, 1 ) < 0 ) { return false; }

    lastSeq      = seq;
    pendingSysid = sysid;
    ESP_LOGI( logTag, "selection requested: %s", payload );

    return true;
}

int32_t selectionCurrent( void ) { return currentSysid; }

int32_t selectionPending( void ) { return pendingSysid; }

bool selectionBrokerConnected( void ) { return brokerConnected; }

devState_t selectionCtrlboxState( void ) { return ctrlboxState; }
