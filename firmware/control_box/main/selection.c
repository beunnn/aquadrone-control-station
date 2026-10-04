// Follows the retained MQTT topic aquadrone/target_sysid (published by the Tab5) and reports presence.
#include "selection.h"
#include <string.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "sdkconfig.h"

#define TOPIC_TARGET "aquadrone/target_sysid"
#define TOPIC_STATUS "aquadrone/ctrl/ctrlbox/status"

static const char *TAG = "selection";
static volatile int s_sysid;

// Returns the sysid from {"sysid": N, ...}, 0 for an empty (cleared) payload, -1 if invalid.
static int parse_sysid(const char *data, int len)
{
    if (len == 0) return 0;
    cJSON *root = cJSON_ParseWithLength(data, len);
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "sysid");
    int id = -1;
    if (cJSON_IsNumber(v) && v->valuedouble == (double)v->valueint && v->valueint >= 1 && v->valueint <= 254 && v->valueint != CONFIG_MAV_SYSID) id = v->valueint;
    cJSON_Delete(root);
    return id;
}

static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "broker connected");
        esp_mqtt_client_publish(e->client, TOPIC_STATUS, "online", 0, 1, 1);
        esp_mqtt_client_subscribe(e->client, TOPIC_TARGET, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        // A broker outage must not drop control of the boat, so the last selection is kept.
        ESP_LOGW(TAG, "broker disconnected, keeping target %d", s_sysid);
        break;
    case MQTT_EVENT_DATA:
        if (e->topic_len != (int)strlen(TOPIC_TARGET) || memcmp(e->topic, TOPIC_TARGET, e->topic_len) != 0) break;
        if (e->data_len != e->total_data_len) { ESP_LOGW(TAG, "fragmented selection message ignored"); break; }
        int sysid = parse_sysid(e->data, e->data_len);
        if (sysid < 0) { ESP_LOGW(TAG, "invalid selection payload '%.*s' ignored", e->data_len, e->data); break; }
        if (sysid != s_sysid) ESP_LOGI(TAG, "selection %d -> %d", s_sysid, sysid);
        s_sysid = sysid;
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGW(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

void selection_start(void)
{
    const esp_mqtt_client_config_t cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .credentials.username = CONFIG_MQTT_USER,
        .credentials.authentication.password = CONFIG_MQTT_PASSWORD,
        .session.keepalive = 5,
        .session.last_will = { .topic = TOPIC_STATUS, .msg = "offline", .qos = 1, .retain = 1 },
    };
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, on_mqtt, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
}

int selection_get(void) { return s_sysid; }
