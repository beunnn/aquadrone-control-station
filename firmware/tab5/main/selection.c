// Shared vehicle selection over MQTT: publishes taps to the retained topic aquadrone/target_sysid and
// shows only what the broker echoes back, so the display always matches what the control box follows.
#include "selection.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "sdkconfig.h"

#define TOPIC_TARGET   "aquadrone/target_sysid"
#define TOPIC_STATUS   "aquadrone/ctrl/+/status"
#define TOPIC_OWN      "aquadrone/ctrl/tab5/status"
#define TOPIC_CTRLBOX  "aquadrone/ctrl/ctrlbox/status"
#define TIME_VALID_S   1700000000   // before this the clock has not been set by SNTP

static const char *TAG = "selection";
static esp_mqtt_client_handle_t s_client;
static volatile bool s_connected;
static volatile int s_current;
static volatile int s_pending;
static volatile int s_seq;
static volatile dev_state_t s_ctrlbox = DEV_UNKNOWN;

static bool topic_is(const esp_mqtt_event_handle_t e, const char *topic)
{
    return e->topic_len == (int)strlen(topic) && memcmp(e->topic, topic, e->topic_len) == 0;
}

static void on_target(const char *data, int len)
{
    int sysid = 0;
    if (len > 0) {
        cJSON *root = cJSON_ParseWithLength(data, len);
        const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, "sysid");
        const cJSON *seq = cJSON_GetObjectItemCaseSensitive(root, "seq");
        sysid = (cJSON_IsNumber(v) && v->valueint >= 1 && v->valueint <= 254) ? v->valueint : -1;
        if (cJSON_IsNumber(seq) && seq->valueint > s_seq) s_seq = seq->valueint;
        cJSON_Delete(root);
    }
    if (sysid < 0) { ESP_LOGW(TAG, "invalid selection payload '%.*s' ignored", len, data); return; }
    ESP_LOGI(TAG, "selection confirmed: %d", sysid);
    s_current = sysid;
    s_pending = 0;
}

static void on_mqtt(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "broker connected");
        s_connected = true;
        esp_mqtt_client_publish(e->client, TOPIC_OWN, "online", 0, 1, 1);
        esp_mqtt_client_subscribe(e->client, TOPIC_TARGET, 1);
        esp_mqtt_client_subscribe(e->client, TOPIC_STATUS, 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "broker disconnected");
        s_connected = false;
        s_pending = 0;
        s_ctrlbox = DEV_UNKNOWN;
        break;
    case MQTT_EVENT_DATA:
        if (e->data_len != e->total_data_len) break;
        if (topic_is(e, TOPIC_TARGET)) {
            on_target(e->data, e->data_len);
        } else if (topic_is(e, TOPIC_CTRLBOX)) {
            s_ctrlbox = (e->data_len == 6 && memcmp(e->data, "online", 6) == 0) ? DEV_ONLINE : DEV_OFFLINE;
        }
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
        .session.last_will = { .topic = TOPIC_OWN, .msg = "offline", .qos = 1, .retain = 1 },
    };
    s_client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, on_mqtt, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_client));
}

bool selection_request(int sysid)
{
    if (!s_connected) return false;
    char buf[112];
    int seq = s_seq + 1;
    time_t now = time(NULL);
    if (now > TIME_VALID_S) snprintf(buf, sizeof(buf), "{\"sysid\":%d,\"src\":\"tab5\",\"seq\":%d,\"ts\":%lld}", sysid, seq, (long long)now);
    else snprintf(buf, sizeof(buf), "{\"sysid\":%d,\"src\":\"tab5\",\"seq\":%d}", sysid, seq);

    if (esp_mqtt_client_publish(s_client, TOPIC_TARGET, buf, 0, 1, 1) < 0) return false;
    s_seq = seq;
    s_pending = sysid;
    ESP_LOGI(TAG, "selection requested: %s", buf);
    return true;
}

int selection_current(void) { return s_current; }
int selection_pending(void) { return s_pending; }
bool selection_broker_connected(void) { return s_connected; }
dev_state_t selection_ctrlbox_state(void) { return s_ctrlbox; }
