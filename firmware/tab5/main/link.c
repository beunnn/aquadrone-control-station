// MAVLink link to mavlink-router: discovers vehicles from their heartbeats and tracks their status.
// The Tab5 never sends commands or overrides; the heartbeat only lets the router learn our address.
#include "link.h"
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "mavlink_headers_cfg.h"
#include "sdkconfig.h"

static const char *TAG = "link";
static int s_sock = -1;
static struct sockaddr_in s_hub;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static vehicle_t s_veh[LINK_MAX_VEHICLES];   // sorted by sysid
static int s_nveh;
static volatile int64_t s_last_rx_us;

// Returns the entry for sysid, inserting it in sysid order if create is set. Call with s_lock held.
static vehicle_t *find(uint8_t sysid, bool create)
{
    int i = 0;
    while (i < s_nveh && s_veh[i].sysid < sysid) i++;
    if (i < s_nveh && s_veh[i].sysid == sysid) return &s_veh[i];
    if (!create || s_nveh == LINK_MAX_VEHICLES) return NULL;
    memmove(&s_veh[i + 1], &s_veh[i], (s_nveh - i) * sizeof(vehicle_t));
    s_nveh++;
    s_veh[i] = (vehicle_t){ .sysid = sysid, .batt_pct = -1, .gps_sats = 255 };
    return &s_veh[i];
}

static void handle(const mavlink_message_t *msg)
{
    if (msg->compid != MAV_COMP_ID_AUTOPILOT1) return;
    int64_t now = esp_timer_get_time();

    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        if (hb.type == MAV_TYPE_GCS || hb.autopilot == MAV_AUTOPILOT_INVALID) return;
        taskENTER_CRITICAL(&s_lock);
        vehicle_t *v = find(msg->sysid, true);
        if (v) {
            v->type = hb.type;
            v->armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
            v->mode = hb.custom_mode;
            v->last_hb_us = now;
        }
        taskEXIT_CRITICAL(&s_lock);
    } else if (msg->msgid == MAVLINK_MSG_ID_SYS_STATUS) {
        mavlink_sys_status_t st;
        mavlink_msg_sys_status_decode(msg, &st);
        taskENTER_CRITICAL(&s_lock);
        vehicle_t *v = find(msg->sysid, false);
        if (v) {
            v->has_batt = st.voltage_battery != UINT16_MAX;
            v->batt_mv = st.voltage_battery;
            v->batt_pct = st.battery_remaining;
        }
        taskEXIT_CRITICAL(&s_lock);
    } else if (msg->msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
        mavlink_gps_raw_int_t gps;
        mavlink_msg_gps_raw_int_decode(msg, &gps);
        taskENTER_CRITICAL(&s_lock);
        vehicle_t *v = find(msg->sysid, false);
        if (v) {
            v->has_gps = true;
            v->gps_fix = gps.fix_type;
            v->gps_sats = gps.satellites_visible;
        }
        taskEXIT_CRITICAL(&s_lock);
    }
}

static void rx_task(void *arg)
{
    uint8_t buf[512];
    mavlink_message_t msg;
    mavlink_status_t st;
    for (;;) {
        int n = recv(s_sock, buf, sizeof(buf), 0);
        if (n <= 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        s_last_rx_us = esp_timer_get_time();
        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st)) handle(&msg);
        }
    }
}

static void heartbeat_task(void *arg)
{
    for (;;) {
        mavlink_message_t m;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_heartbeat_pack(CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &m, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        int len = mavlink_msg_to_send_buffer(buf, &m);
        sendto(s_sock, buf, len, 0, (struct sockaddr *)&s_hub, sizeof(s_hub));   // fails harmlessly while Wi-Fi is down
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void link_start(void)
{
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    memset(&s_hub, 0, sizeof(s_hub));
    s_hub.sin_family = AF_INET;
    s_hub.sin_port = htons(CONFIG_HUB_PORT);
    inet_pton(AF_INET, CONFIG_HUB_IP, &s_hub.sin_addr);
    // A fixed source port keeps the router's learned endpoint stable across reboots.
    struct sockaddr_in local = { .sin_family = AF_INET, .sin_port = htons(CONFIG_LOCAL_PORT), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (bind(s_sock, (struct sockaddr *)&local, sizeof(local)) != 0) ESP_LOGE(TAG, "bind to port %d failed", CONFIG_LOCAL_PORT);

    xTaskCreate(rx_task, "mav_rx", 8192, NULL, 5, NULL);
    xTaskCreate(heartbeat_task, "mav_hb", 4096, NULL, 4, NULL);
}

int link_vehicles(vehicle_t *out, int max)
{
    taskENTER_CRITICAL(&s_lock);
    int n = s_nveh < max ? s_nveh : max;
    memcpy(out, s_veh, n * sizeof(vehicle_t));
    taskEXIT_CRITICAL(&s_lock);
    return n;
}

int64_t link_last_rx_us(void) { return s_last_rx_us; }
