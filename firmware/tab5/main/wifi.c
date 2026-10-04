// Wi-Fi station. On the P4 the esp_wifi API is provided by esp_wifi_remote, which forwards it to the C6 over esp_hosted SDIO.
#include "wifi.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define INIT_RETRY_MS 5000

static const char *TAG = "wifi";
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_ip[16];
static const char *volatile s_state = "starting";

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        taskENTER_CRITICAL(&s_lock);
        s_ip[0] = '\0';
        taskEXIT_CRITICAL(&s_lock);
        s_state = "connecting";
        ESP_LOGW(TAG, "disconnected (reason %d), retrying", ((wifi_event_sta_disconnected_t *)data)->reason);
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&e->ip_info.ip));
        taskENTER_CRITICAL(&s_lock);
        memcpy(s_ip, ip, sizeof(s_ip));
        taskEXIT_CRITICAL(&s_lock);
        ESP_LOGI(TAG, "IP %s", ip);
    }
}

// esp_wifi_init() fails when the C6 does not answer on SDIO; esp_hosted resets the C6 again on every attempt.
static void init_task(void *arg)
{
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    while (esp_wifi_init(&init) != ESP_OK) {
        s_state = "C6 not responding";
        ESP_LOGE(TAG, "Wi-Fi module (ESP32-C6) not responding, retrying in %d s", INIT_RETRY_MS / 1000);
        vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_MS));
    }

    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, CONFIG_WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, CONFIG_WIFI_PASSWORD, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
    s_state = "connecting";
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "connecting to '%s'", CONFIG_WIFI_SSID);
    vTaskDelete(NULL);
}

void wifi_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));
    xTaskCreate(init_task, "wifi_init", 4096, NULL, 5, NULL);
}

const char *wifi_state_text(void) { return s_state; }

bool wifi_ip(char *buf, size_t len)
{
    taskENTER_CRITICAL(&s_lock);
    strlcpy(buf, s_ip, len);
    taskEXIT_CRITICAL(&s_lock);
    return buf[0] != '\0';
}
