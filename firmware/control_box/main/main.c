// Control box: Ethernet bring-up + MAVLink heartbeat over UDP to mavlink-router.
#include <string.h>
#include "esp_log.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "mavlink_headers_cfg.h"
#include "joystick.h"
#include "buttons.h"
#include "sdkconfig.h"

static const char *TAG = "ctrlbox";

// Waveshare ESP32-P4-Module-DEV-KIT: IP101GRI PHY, RMII, external 50 MHz clock on GPIO50
#define ETH_MDC_GPIO   31
#define ETH_MDIO_GPIO  52
#define ETH_RST_GPIO   51
#define ETH_PHY_ADDR   (-1)  // autodetect

static EventGroupHandle_t s_ev;
#define GOT_IP_BIT BIT0

static void on_got_ip(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    const ip_event_got_ip_t *e = data;
    ESP_LOGI(TAG, "IP " IPSTR " gw " IPSTR, IP2STR(&e->ip_info.ip), IP2STR(&e->ip_info.gw));
    xEventGroupSetBits(s_ev, GOT_IP_BIT);
}

static void on_eth(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    ESP_LOGI(TAG, "ETH event %ld (%s)", (long)id,
             id == ETHERNET_EVENT_CONNECTED ? "link up" :
             id == ETHERNET_EVENT_DISCONNECTED ? "link down" : "other");
}

static void eth_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_config_t cfg = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t *netif = esp_netif_new(&cfg);

    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    eth_esp32_emac_config_t emac = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac.smi_gpio.mdc_num = ETH_MDC_GPIO;
    emac.smi_gpio.mdio_num = ETH_MDIO_GPIO;
    emac.clock_config.rmii.clock_mode = EMAC_CLK_EXT_IN;
    emac.clock_config.rmii.clock_gpio = 50;
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac, &mac_cfg);

    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr = ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = ETH_RST_GPIO;
    esp_eth_phy_t *phy = esp_eth_phy_new_ip101(&phy_cfg);

    esp_eth_handle_t eth = NULL;
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &eth));
    ESP_ERROR_CHECK(esp_netif_attach(netif, esp_eth_new_netif_glue(eth)));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, on_eth, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, on_got_ip, NULL));
    ESP_ERROR_CHECK(esp_eth_start(eth));
}

static int s_sock = -1;
static struct sockaddr_in s_hub;

static void send_msg(const mavlink_message_t *m)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int n = mavlink_msg_to_send_buffer(buf, m);
    sendto(s_sock, buf, n, 0, (struct sockaddr *)&s_hub, sizeof(s_hub));
}

// ---- vehicle state, written by rx_task, read by control_task ------------------
static volatile int64_t s_last_hb_us;      // last HEARTBEAT from the target vehicle (component 1)
static volatile bool s_veh_armed;
static volatile uint32_t s_veh_mode;

// ---- command sending with ACK / retry ------------------------------------------
typedef struct {
    bool active;
    uint16_t cmd;
    float p1, p2;
    int tries;
    int64_t sent_us;
    const char *name;
} pending_cmd_t;
static pending_cmd_t s_pend;
static volatile int s_ack_cmd = -1, s_ack_result;

#define CMD_RETRY_US 1000000
#define CMD_MAX_TRIES 3

static void cmd_transmit(pending_cmd_t *c)
{
    mavlink_message_t m;
    mavlink_msg_command_long_pack(CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &m, CONFIG_TARGET_SYSID, 1,
                                  c->cmd, c->tries, c->p1, c->p2, 0, 0, 0, 0, 0);
    send_msg(&m);
    c->sent_us = esp_timer_get_time();
    c->tries++;
}

static void cmd_start(const char *name, uint16_t cmd, float p1, float p2)
{
    s_pend = (pending_cmd_t){ .active = true, .cmd = cmd, .p1 = p1, .p2 = p2, .name = name };
    s_ack_cmd = -1;
    cmd_transmit(&s_pend);
    ESP_LOGI(TAG, "cmd %s sent", name);
}

static void cmd_service(void)
{
    if (!s_pend.active) return;
    if (s_ack_cmd == s_pend.cmd) {
        ESP_LOGI(TAG, "cmd %s ACK result %d (%s)", s_pend.name, s_ack_result, s_ack_result == 0 ? "accepted" : "REJECTED");
        s_pend.active = false;
        return;
    }
    if (esp_timer_get_time() - s_pend.sent_us > CMD_RETRY_US) {
        if (s_pend.tries >= CMD_MAX_TRIES) {
            ESP_LOGW(TAG, "cmd %s: no ACK after %d tries", s_pend.name, s_pend.tries);
            s_pend.active = false;
        } else {
            cmd_transmit(&s_pend);
        }
    }
}

static void heartbeat_task(void *arg)
{
    for (;;) {
        mavlink_message_t m;
        mavlink_msg_heartbeat_pack(CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &m,
                                   MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
        send_msg(&m);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ---- joystick -> RC override ----------------------------------------------------
#define RC_IGNORE UINT16_MAX
#define DEADBAND  0.08f
#define EXPO      0.30f

static float axis_norm(uint16_t raw)  // -1..1
{
    float v = ((int)raw - 32768) / 32767.0f;
    if (v > 1) v = 1;
    if (v < -1) v = -1;
    float a = v < 0 ? -v : v;
    if (a < DEADBAND) return 0;
    a = (a - DEADBAND) / (1.0f - DEADBAND);
    a = (1 - EXPO) * a + EXPO * a * a * a;
    return v < 0 ? -a : a;
}

// The boat's companion computer rejects RC values outside 1000..2000 on the channels it checks
// (1, 3, 6, 7), so everything sent on those channels is clamped into range.
static uint16_t rc_clamp(int v) { return v < 1000 ? 1000 : (v > 2000 ? 2000 : v); }
static uint16_t to_pwm(float n) { return rc_clamp((int)(1500.0f + 500.0f * n + 0.5f)); }

// RC10: throttle speed scale used by the companion computer, driven by a joystick axis (0..65535 -> 1000..2000 us).
// Which physical axis is the slider is set here (verified on the bench; see docs/joystick.md).
#define SPEED_SCALE_RAW(js) ((uint16_t)(65535 - (js).z))   // inverted: slider bottom = 2000 us, top = 1000 us
static uint16_t speed_scale_pwm(uint16_t raw) { return rc_clamp(1000 + (int)(raw * 1000UL / 65535UL)); }

static void send_override(uint16_t steer, uint16_t thr, uint16_t motor, uint16_t trim, uint16_t speed)
{
    mavlink_message_t m;
    mavlink_msg_rc_channels_override_pack(CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &m, CONFIG_TARGET_SYSID, 1,
        steer, RC_IGNORE, thr, RC_IGNORE, RC_IGNORE, motor, trim, RC_IGNORE,
        RC_IGNORE, speed, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE, RC_IGNORE);
    send_msg(&m);
}

// Rover custom modes
#define ROVER_MODE_MANUAL 0
#define ROVER_MODE_HOLD   4
#define ROVER_MODE_AUTO   10
#define ROVER_MODE_RTL    11
#define ARM_HOLD_TICKS    16     // 16 x 50 ms = 0.8 s
#define ESTOP_PULSE_US    3000000   // emergency-stop output lasts 3 s
#define ESTOP_HOLD_TICKS  6      // 6 x 50 ms = 300 ms with both buttons held
#define VEH_LOST_US       3000000

static void control_task(void *arg)
{
    bool ovr_enabled = false;     // RC override output, OFF at boot
    bool motor_on = false;
    bool estop = false;           // true during the emergency-stop pulse
    int64_t estop_end_us = 0;
    bool estop_rearm = true;      // both stop buttons must be released before another stop can fire
    int estop_ticks = 0;
    int arm_ticks = 0;
    int64_t last_estop_cmd = 0;
    bool veh_ok_prev = false;
    uint32_t n = 0;
    uint16_t last_speed = 1000;
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));  // 20 Hz
        uint32_t held = buttons_held();
        uint32_t ev = buttons_pressed_events();
        int64_t now = esp_timer_get_time();
        if (now < 5000000) continue;   // boot hold-off: ignore all button activity for the first 5 s
        bool veh_ok = (now - s_last_hb_us) < VEH_LOST_US && s_last_hb_us != 0;

        if (veh_ok != veh_ok_prev) {
            ESP_LOGW(TAG, "vehicle %s", veh_ok ? "heartbeat OK" : "heartbeat LOST");
            veh_ok_prev = veh_ok;
        }
        if (!veh_ok && motor_on) { motor_on = false; ESP_LOGW(TAG, "motor OFF (vehicle lost)"); }

        // --- emergency shutdown: both buttons held 300 ms -> a 3 s stop pulse, then it clears itself ---
        bool both = (held & BTN_BIT(BTN_ESTOP_A)) && (held & BTN_BIT(BTN_ESTOP_B));
        if (!(held & (BTN_BIT(BTN_ESTOP_A) | BTN_BIT(BTN_ESTOP_B)))) estop_rearm = true;   // fully released
        estop_ticks = both ? estop_ticks + 1 : 0;
        if (!estop && estop_rearm && estop_ticks >= ESTOP_HOLD_TICKS) {
            estop = true;
            estop_rearm = false;
            estop_end_us = now + ESTOP_PULSE_US;
            motor_on = false;
            s_pend.active = false;
            ESP_LOGE(TAG, "EMERGENCY SHUTDOWN: stop pulse for %d s", ESTOP_PULSE_US / 1000000);
        }
        if (estop && now >= estop_end_us) {
            estop = false;
            ESP_LOGW(TAG, "emergency shutdown pulse ended, controls released (motor stays off)");
        }
        if (estop && s_veh_armed && now - last_estop_cmd > 200000) {
            mavlink_message_t m;   // force disarm, repeated until the vehicle reports disarmed
            mavlink_msg_command_long_pack(CONFIG_MAV_SYSID, CONFIG_MAV_COMPID, &m, CONFIG_TARGET_SYSID, 1,
                                          MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 21196, 0, 0, 0, 0, 0);
            send_msg(&m);
            last_estop_cmd = now;
        }

        // --- override enable toggle ---
        if (ev & BTN_BIT(BTN_OVERRIDE_TOGGLE)) {
            if (ovr_enabled) {
                ovr_enabled = false;
                motor_on = false;
                for (int i = 0; i < 5; i++) send_override(1500, 1500, 1000, 1500, last_speed);  // neutral burst, then stop
                ESP_LOGW(TAG, "RC override DISABLED (no override messages are sent)");
            } else {
                ovr_enabled = true;
                motor_on = false;
                ESP_LOGW(TAG, "RC override ENABLED (motor off)");
            }
        }

        // --- motor on/off (only with override enabled, not during e-stop) ---
        if (ev & BTN_BIT(BTN_MOTOR_OFF)) { motor_on = false; ESP_LOGI(TAG, "motor OFF"); }
        if (ev & BTN_BIT(BTN_MOTOR_ON)) {
            if (ovr_enabled && !estop && veh_ok) { motor_on = true; ESP_LOGI(TAG, "motor ON"); }
            else ESP_LOGW(TAG, "motor ON refused: override %s, e-stop pulse %s, vehicle heartbeat %s",
                          ovr_enabled ? "enabled" : "DISABLED (press the override button)", estop ? "ACTIVE" : "inactive", veh_ok ? "ok" : "LOST");
        }

        // --- discrete commands (blocked during e-stop) ---
        if (!estop) {
            if (ev & BTN_BIT(BTN_MODE1)) cmd_start("mode Manual", MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ROVER_MODE_MANUAL);
            if (ev & BTN_BIT(BTN_MODE2)) cmd_start("mode Hold",   MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ROVER_MODE_HOLD);
            if (ev & BTN_BIT(BTN_MODE3)) cmd_start("mode Auto",   MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ROVER_MODE_AUTO);
            if (ev & BTN_BIT(BTN_MODE4)) cmd_start("mode RTL",    MAV_CMD_DO_SET_MODE, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, ROVER_MODE_RTL);
            // Arm sits on the trigger: only a deliberate ~0.8 s hold arms.
            arm_ticks = (held & BTN_BIT(BTN_ARM)) ? arm_ticks + 1 : 0;
            if (arm_ticks == ARM_HOLD_TICKS) cmd_start("arm", MAV_CMD_COMPONENT_ARM_DISARM, 1, 0);
        }
        if (ev & BTN_BIT(BTN_DISARM)) cmd_start("disarm", MAV_CMD_COMPONENT_ARM_DISARM, 0, 0);
        cmd_service();

        // --- RC override stream ---
        joystick_state_t js;
        if (ovr_enabled && joystick_get(&js)) {
            uint16_t steer = to_pwm(axis_norm(js.x));
            uint16_t thr = (motor_on && !estop) ? to_pwm(-axis_norm(js.y)) : 1500;   // forward = Y low
            uint16_t motor = (motor_on && !estop) ? 2000 : 1000;
            bool up = held & BTN_BIT(BTN_TRIM_UP), dn = held & BTN_BIT(BTN_TRIM_DOWN);
            uint16_t trim = (up == dn) ? 1500 : (up ? 2000 : 1000);
            if (estop) { steer = 1500; trim = 1500; }
            uint16_t speed = speed_scale_pwm(SPEED_SCALE_RAW(js));
            last_speed = speed;
            send_override(steer, thr, motor, trim, speed);
            if (++n % 20 == 0)
                ESP_LOGI(TAG, "RC override -> sys %d: ch1 %u ch3 %u ch6 %u ch7 %u ch10 %u | veh %s mode %lu",
                         CONFIG_TARGET_SYSID, steer, thr, motor, trim, speed, s_veh_armed ? "ARMED" : "disarmed", (unsigned long)s_veh_mode);
        }
    }
}

static void rx_task(void *arg)
{
    uint8_t buf[512];
    mavlink_message_t msg;
    mavlink_status_t st;
    for (;;) {
        int n = recv(s_sock, buf, sizeof(buf), 0);
        for (int i = 0; i < n; i++) {
            if (!mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &st)) continue;
            if (msg.sysid != CONFIG_TARGET_SYSID) continue;
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT && msg.compid == 1) {
                mavlink_heartbeat_t hb;
                mavlink_msg_heartbeat_decode(&msg, &hb);
                s_veh_armed = hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED;
                s_veh_mode = hb.custom_mode;
                s_last_hb_us = esp_timer_get_time();
            } else if (msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
                mavlink_command_ack_t ack;
                mavlink_msg_command_ack_decode(&msg, &ack);
                s_ack_result = ack.result;
                s_ack_cmd = ack.command;
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    buttons_start();
    joystick_start();
    s_ev = xEventGroupCreate();
    eth_start();
    xEventGroupWaitBits(s_ev, GOT_IP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    memset(&s_hub, 0, sizeof(s_hub));
    s_hub.sin_family = AF_INET;
    s_hub.sin_port = htons(CONFIG_HUB_PORT);
    inet_pton(AF_INET, CONFIG_HUB_IP, &s_hub.sin_addr);
    // Bind so the router learns a stable source port.
    struct sockaddr_in local = { .sin_family = AF_INET, .sin_port = htons(14561), .sin_addr.s_addr = htonl(INADDR_ANY) };
    bind(s_sock, (struct sockaddr *)&local, sizeof(local));

    xTaskCreate(heartbeat_task, "hb", 4096, NULL, 5, NULL);
    xTaskCreate(rx_task, "rx", 8192, NULL, 5, NULL);
    xTaskCreate(control_task, "ctrl", 6144, NULL, 6, NULL);
}
