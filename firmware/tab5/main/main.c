// M5Stack Tab5: vehicle selector and status display for the AquaDrone control station.
#include "bsp/esp-bsp.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"
#include "link.h"
#include "sdkconfig.h"
#include "selection.h"
#include "ui.h"
#include "wifi.h"

static const char* logTag = "tab5";

void app_main( void )
{
    esp_err_t err = nvs_flash_init();

    if ( ( err == ESP_ERR_NVS_NO_FREE_PAGES ) || ( err == ESP_ERR_NVS_NEW_VERSION_FOUND ) )
    {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK( err );

    // The BSP owns the LVGL task; all LVGL calls outside it need the display lock.
    ( void )bsp_display_start();
    ( void )bsp_display_backlight_on();
    ( void )bsp_display_lock( 0U );
    uiStart();
    bsp_display_unlock();

    // WLAN_PWR_EN on the second IO expander powers the ESP32-C6.
    ESP_ERROR_CHECK( bsp_feature_enable( BSP_FEATURE_WIFI, true ) );
    wifiStart();

    // Only used for the selection timestamp; selections are still published before the clock is set.
    const esp_sntp_config_t sntpConfig = ESP_NETIF_SNTP_DEFAULT_CONFIG( CONFIG_NTP_SERVER );
    ( void )esp_netif_sntp_init( &sntpConfig );

    selectionStart();
    linkStart();
    ESP_LOGI( logTag, "started" );
}
