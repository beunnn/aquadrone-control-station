// Wi-Fi station. On the P4 the esp_wifi API is provided by esp_wifi_remote, which forwards every call to the on-board
// ESP32-C6 over esp_hosted (SDIO); events come back through the normal ESP-IDF event loop.
#include "wifi.h"

#include <stdio.h>
#include <string.h>
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define INIT_RETRY_MS 5000U
#define IP_TEXT_LEN   16U

static const char* logTag  = "wifi";
static portMUX_TYPE ipLock = portMUX_INITIALIZER_UNLOCKED;
static char ipText[IP_TEXT_LEN];
static const char* volatile stateText = "starting";

// WIFI_EVENT / IP_EVENT handler (default event loop): connects at start, reconnects after every disconnect.
static void onWifiEvent( void* arg, esp_event_base_t base, int32_t eventId, void* eventData )
{
    ( void )arg;

    if ( ( base == WIFI_EVENT ) && ( eventId == WIFI_EVENT_STA_START ) ) { ( void )esp_wifi_connect(); }
    else if ( ( base == WIFI_EVENT ) && ( eventId == WIFI_EVENT_STA_DISCONNECTED ) )
    {
        const wifi_event_sta_disconnected_t* event = ( const wifi_event_sta_disconnected_t* )eventData;

        taskENTER_CRITICAL( &ipLock );
        ipText[0] = '\0';
        taskEXIT_CRITICAL( &ipLock );
        stateText = "connecting";
        ESP_LOGW( logTag, "disconnected (reason %d), retrying", ( int )event->reason );
        ( void )esp_wifi_connect();
    }
    else if ( ( base == IP_EVENT ) && ( eventId == IP_EVENT_STA_GOT_IP ) )
    {
        const ip_event_got_ip_t* event = ( const ip_event_got_ip_t* )eventData;
        char text[IP_TEXT_LEN];

        ( void )snprintf( text, sizeof( text ), IPSTR, IP2STR( &event->ip_info.ip ) );
        taskENTER_CRITICAL( &ipLock );
        ( void )memcpy( ipText, text, sizeof( ipText ) );
        taskEXIT_CRITICAL( &ipLock );
        ESP_LOGI( logTag, "IP %s", text );
    }
    else
    {
        // Other Wi-Fi events are not used.
    }
}

// One-shot task: esp_wifi_init() fails while the C6 does not answer on SDIO, and esp_hosted resets the C6 on every attempt.
static void initTask( void* arg )
{
    const wifi_init_config_t initConfig = WIFI_INIT_CONFIG_DEFAULT();
    wifi_config_t config;

    ( void )arg;

    while ( esp_wifi_init( &initConfig ) != ESP_OK )
    {
        stateText = "C6 not responding";
        ESP_LOGE( logTag, "Wi-Fi module (ESP32-C6) not responding, retrying in %u s", INIT_RETRY_MS / 1000U );
        vTaskDelay( pdMS_TO_TICKS( INIT_RETRY_MS ) );
    }

    ( void )memset( &config, 0, sizeof( config ) );
    ( void )strlcpy( ( char* )config.sta.ssid, CONFIG_WIFI_SSID, sizeof( config.sta.ssid ) );
    ( void )strlcpy( ( char* )config.sta.password, CONFIG_WIFI_PASSWORD, sizeof( config.sta.password ) );
    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
    ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &config ) );
    stateText = "connecting";
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_LOGI( logTag, "connecting to '%s'", CONFIG_WIFI_SSID );
    vTaskDelete( NULL );
}

void wifiStart( void )
{
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    ( void )esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK( esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, onWifiEvent, NULL ) );
    ESP_ERROR_CHECK( esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, onWifiEvent, NULL ) );
    ( void )xTaskCreate( initTask, "wifi_init", 4096, NULL, 5, NULL );
}

const char* wifiStateText( void ) { return stateText; }

bool wifiIp( char* buffer, size_t length )
{
    taskENTER_CRITICAL( &ipLock );
    ( void )strlcpy( buffer, ipText, length );
    taskEXIT_CRITICAL( &ipLock );

    return buffer[0] != '\0';
}
