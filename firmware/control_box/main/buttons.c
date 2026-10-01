// Logical control buttons. One is a physical GPIO button (RC override enable); all others come
// from joystick buttons through k_js_map. Everything is debounced and edge-detected here.
#include "buttons.h"
#include "joystick.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "buttons";

// Physical GPIO per logical button, -1 = not a GPIO button.
static const int k_gpio[BTN_COUNT] = {
    [0 ... BTN_COUNT - 1] = -1,
    [BTN_OVERRIDE_TOGGLE] = GPIO_NUM_3,   // wired between GPIO3 and GND
};

// Joystick button bit (bit n = button n+1) per logical button, -1 = unassigned.
// Filled in after the learning session; unassigned functions do nothing.
static const int k_js_map[BTN_COUNT] = {
    [0 ... BTN_COUNT - 1] = -1,
    [BTN_MODE1] = 4, [BTN_MODE2] = 5, [BTN_MODE3] = 6, [BTN_MODE4] = 7,   // joystick buttons 5..8
    [BTN_ARM] = 0,                                                          // button 1 (trigger): needs a 0.8 s hold
    [BTN_DISARM] = 1, [BTN_MOTOR_ON] = 2, [BTN_MOTOR_OFF] = 3,             // buttons 2, 3, 4
    [BTN_TRIM_UP] = 26, [BTN_TRIM_DOWN] = 25,                               // buttons 27, 26
    [BTN_ESTOP_A] = 14, [BTN_ESTOP_B] = 12,                                 // buttons 15, 13 (both must be held)
};

#define POLL_MS 5
#define STABLE_SAMPLES 3  // 15 ms debounce

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_held;
static volatile uint32_t s_events;

static bool raw_level(int i, bool js_ok, uint64_t js_buttons)
{
    if (k_gpio[i] >= 0) return gpio_get_level(k_gpio[i]) == 0;
    if (k_js_map[i] >= 0 && js_ok) return (js_buttons >> k_js_map[i]) & 1;
    return false;
}

static void button_task(void *arg)
{
    uint8_t cnt[BTN_COUNT] = {0};
    bool js_was_ok = false;
    uint64_t js_prev = 0;
    for (;;) {
        joystick_state_t js = {0};
        bool js_ok = joystick_get(&js);

        // Log every joystick button edge so buttons can be identified (learning session).
        if (js_ok && js_was_ok) {
            uint64_t changed = js.buttons ^ js_prev;
            for (int b = 0; b < 44; b++)
                if ((changed >> b) & 1)
                    ESP_LOGI(TAG, "joystick button %d (bit %d) %s", b + 1, b, ((js.buttons >> b) & 1) ? "PRESSED" : "released");
        }
        // Log movements of the spare axes (identifies the slider etc.), at most when they moved noticeably.
        if (js_ok) {
            const uint16_t ax[6] = { js.z, js.rx, js.ry, js.rz, js.slider, js.dial };
            static const char *const nm[6] = { "Z", "Rx", "Ry", "Rz", "Slider", "Dial" };
            static uint16_t logged[6];
            static bool have_logged;
            if (!have_logged) { for (int a = 0; a < 6; a++) logged[a] = ax[a]; have_logged = true; }
            for (int a = 0; a < 6; a++) {
                if (ax[a] > logged[a] + 4000 || ax[a] + 4000 < logged[a]) {
                    ESP_LOGI(TAG, "axis %s = %u", nm[a], ax[a]);
                    logged[a] = ax[a];
                }
            }
        }
        {   // diagnostics: report rate every 5 s
            static int tick; static uint32_t seq0;
            if (++tick >= 1000) {
                tick = 0;
                ESP_LOGI(TAG, "joystick %s, %lu reports in last 5 s", js_ok ? "ok" : "NOT connected", (unsigned long)(js.seq - seq0));
                seq0 = js.seq;
            }
        }
        if (js_ok) {   // diagnostics: vendor bytes (report bytes 29..63) that change
            static uint8_t vprev[35]; static bool vhave; static int64_t vlast;
            if (vhave) {
                char buf[160]; int o = 0;
                for (int k = 0; k < 35 && o < 140; k++)
                    if (js.vendor[k] != vprev[k]) o += snprintf(buf + o, sizeof(buf) - o, " [%d]=%u", 29 + k, js.vendor[k]);
                if (o && esp_timer_get_time() - vlast > 200000) { ESP_LOGI(TAG, "vendor bytes changed:%s", buf); vlast = esp_timer_get_time(); }
            }
            memcpy(vprev, js.vendor, 35); vhave = true;
        }
        js_prev = js_ok ? js.buttons : 0;

        // On joystick (re)connect, whatever is already pressed counts as held, without events.
        if (js_ok && !js_was_ok) {
            taskENTER_CRITICAL(&s_mux);
            for (int i = 0; i < BTN_COUNT; i++) {
                if (k_js_map[i] < 0) continue;
                if (raw_level(i, true, js.buttons)) s_held |= BTN_BIT(i); else s_held &= ~BTN_BIT(i);
                cnt[i] = 0;
            }
            taskEXIT_CRITICAL(&s_mux);
            ESP_LOGI(TAG, "joystick connected, buttons held at connect: 0x%011llx", (unsigned long long)js.buttons);
        }
        js_was_ok = js_ok;

        for (int i = 0; i < BTN_COUNT; i++) {
            bool raw = raw_level(i, js_ok, js.buttons);
            bool cur = (s_held >> i) & 1;
            if (raw == cur) { cnt[i] = 0; continue; }
            if (!raw && k_js_map[i] >= 0 && !js_ok) {   // joystick gone: release immediately, no event
                taskENTER_CRITICAL(&s_mux); s_held &= ~BTN_BIT(i); taskEXIT_CRITICAL(&s_mux);
                cnt[i] = 0;
                continue;
            }
            if (++cnt[i] >= STABLE_SAMPLES) {
                cnt[i] = 0;
                taskENTER_CRITICAL(&s_mux);
                if (raw) { s_held |= BTN_BIT(i); s_events |= BTN_BIT(i); }
                else s_held &= ~BTN_BIT(i);
                taskEXIT_CRITICAL(&s_mux);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

void buttons_start(void)
{
    for (int i = 0; i < BTN_COUNT; i++) {
        if (k_gpio[i] < 0) continue;
        gpio_config_t c = {
            .pin_bit_mask = 1ULL << k_gpio[i], .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&c));
    }
    // Let the pull-ups settle, then seed the state: a GPIO already low at boot counts as
    // "held since boot" and cannot produce a press event until it has been released.
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t boot_mask = 0;
    for (int i = 0; i < BTN_COUNT; i++)
        if (k_gpio[i] >= 0 && gpio_get_level(k_gpio[i]) == 0) boot_mask |= BTN_BIT(i);
    s_held = boot_mask;
    s_events = 0;
    if (boot_mask) ESP_LOGW(TAG, "pins LOW at boot (ignored until released): mask 0x%04lx", (unsigned long)boot_mask);
    xTaskCreate(button_task, "buttons", 4096, NULL, 6, NULL);
}

uint32_t buttons_held(void) { return s_held; }

uint32_t buttons_pressed_events(void)
{
    taskENTER_CRITICAL(&s_mux);
    uint32_t e = s_events;
    s_events = 0;
    taskEXIT_CRITICAL(&s_mux);
    return e;
}
