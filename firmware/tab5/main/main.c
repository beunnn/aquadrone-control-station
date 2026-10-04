// M5Stack Tab5: vehicle selector and status display for the AquaDrone control station.
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "nvs_flash.h"
#include "bsp/esp-bsp.h"
#include "link.h"
#include "selection.h"
#include "ui.h"
#include "wifi.h"
#include "sdkconfig.h"

static const char *TAG = "tab5";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    bsp_display_start();
    bsp_display_backlight_on();
    bsp_display_lock(0);
    ui_start();
    bsp_display_unlock();

    // WLAN_PWR_EN on the IO expander powers the ESP32-C6.
    ESP_ERROR_CHECK(bsp_feature_enable(BSP_FEATURE_WIFI, true));
    wifi_start();

    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_NTP_SERVER);
    esp_netif_sntp_init(&sntp);

    selection_start();
    link_start();
    ESP_LOGI(TAG, "started");
}
