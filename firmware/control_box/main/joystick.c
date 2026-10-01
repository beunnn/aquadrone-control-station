// USB HID host reader for the Thrustmaster Solaris Base (report layout in docs/joystick.md).
#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "joystick.h"

#define JS_VID 0x044f
#define JS_PID 0x0422
#define JS_IFACE 0
#define JS_EP_IN 0x81
#define JS_REPORT_LEN 64

static const char *TAG = "joystick";

static usb_host_client_handle_t s_client;
static usb_device_handle_t s_dev;
static usb_transfer_t *s_xfer;
static volatile bool s_connected;
static joystick_state_t s_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static inline uint16_t le16(const uint8_t *p) { return p[0] | (p[1] << 8); }

static void parse(const uint8_t *r, int len)
{
    if (len < 29 || r[0] != 1) return;
    joystick_state_t s = {0};
    for (int i = 0; i < 6; i++) s.buttons |= (uint64_t)r[1 + i] << (8 * i);
    s.buttons &= (1ULL << 44) - 1;
    s.hat = r[12] & 0x0f;
    s.x = le16(r + 13);  s.y = le16(r + 15);  s.z = le16(r + 17);  s.rz = le16(r + 19);
    s.ry = le16(r + 21); s.rx = le16(r + 23); s.slider = le16(r + 25); s.dial = le16(r + 27);
    memcpy(s.vendor, r + 29, len >= 64 ? 35 : (len > 29 ? len - 29 : 0));
    portENTER_CRITICAL(&s_lock);
    s.seq = s_state.seq + 1;
    s_state = s;
    portEXIT_CRITICAL(&s_lock);
}

static void xfer_cb(usb_transfer_t *t)
{
    if (t->status == USB_TRANSFER_STATUS_COMPLETED) {
        parse(t->data_buffer, t->actual_num_bytes);
        usb_host_transfer_submit(t);
    } else if (t->status != USB_TRANSFER_STATUS_NO_DEVICE && t->status != USB_TRANSFER_STATUS_CANCELED) {
        ESP_LOGW(TAG, "transfer status %d, resubmitting", t->status);
        usb_host_transfer_submit(t);
    }
}

static void feature_diag_task(void *arg);
static void open_device(uint8_t addr)
{
    usb_device_handle_t dev;
    if (usb_host_device_open(s_client, addr, &dev) != ESP_OK) return;
    const usb_device_desc_t *dd;
    usb_host_get_device_descriptor(dev, &dd);
    ESP_LOGI(TAG, "USB device %04x:%04x", dd->idVendor, dd->idProduct);
    if (dd->idVendor != JS_VID || dd->idProduct != JS_PID) {
        usb_host_device_close(s_client, dev);
        return;
    }
    esp_err_t e = usb_host_interface_claim(s_client, dev, JS_IFACE, 0);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "claim failed: %s", esp_err_to_name(e));
        usb_host_device_close(s_client, dev);
        return;
    }
    usb_host_transfer_alloc(JS_REPORT_LEN, 0, &s_xfer);
    s_xfer->device_handle = dev;
    s_xfer->bEndpointAddress = JS_EP_IN;
    s_xfer->num_bytes = JS_REPORT_LEN;
    s_xfer->callback = xfer_cb;
    s_xfer->timeout_ms = 0;
    s_dev = dev;
    portENTER_CRITICAL(&s_lock);
    memset(&s_state, 0, sizeof(s_state));   // seq = 0 until the first real report
    portEXIT_CRITICAL(&s_lock);
    s_connected = true;
    ESP_LOGI(TAG, "Solaris Base attached, reading reports");
    xTaskCreate(feature_diag_task, "featdiag", 4096, NULL, 4, NULL);
    usb_host_transfer_submit(s_xfer);
}

// ---- read-only diagnostic: GET_REPORT(feature) for the vendor feature reports -----------------
#include "freertos/semphr.h"
static SemaphoreHandle_t s_ctrl_done;
static void ctrl_cb(usb_transfer_t *t) { xSemaphoreGive(s_ctrl_done); }

static void feature_diag_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(1500));
    if (!s_dev) { vTaskDelete(NULL); }
    s_ctrl_done = xSemaphoreCreateBinary();
    usb_transfer_t *t;
    usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + 64, 0, &t);

    // Standard HID class requests that the Linux usbhid driver issues at attach.
    // 1) SET_PROTOCOL(report protocol = 1), 2) SET_IDLE(duration 0, all reports)
    const struct { uint8_t req; uint16_t val; const char *name; } std[] = {
        { 0x0B, 1, "SET_PROTOCOL(report)" }, { 0x0A, 0, "SET_IDLE(0)" },
    };
    for (int k = 0; k < 2 && s_dev; k++) {
        usb_setup_packet_t *sp = (usb_setup_packet_t *)t->data_buffer;
        sp->bmRequestType = 0x21;               // host-to-device, class, interface
        sp->bRequest = std[k].req;
        sp->wValue = std[k].val;
        sp->wIndex = JS_IFACE;
        sp->wLength = 0;
        t->device_handle = s_dev;
        t->bEndpointAddress = 0;
        t->num_bytes = sizeof(usb_setup_packet_t);
        t->callback = ctrl_cb;
        t->timeout_ms = 1000;
        bool ok = usb_host_transfer_submit_control(s_client, t) == ESP_OK &&
                  xSemaphoreTake(s_ctrl_done, pdMS_TO_TICKS(2000)) == pdTRUE;
        ESP_LOGI(TAG, "%s: %s (status %d)", std[k].name, ok ? "done" : "request failed", ok ? t->status : -1);
    }
    const uint8_t ids[] = { 0xF1, 0xF2, 0xF3 };
    for (int k = 0; k < 3 && s_dev; k++) {
        usb_setup_packet_t *sp = (usb_setup_packet_t *)t->data_buffer;
        sp->bmRequestType = 0xA1;               // device-to-host, class, interface
        sp->bRequest = 0x01;                    // GET_REPORT
        sp->wValue = (3 << 8) | ids[k];         // feature report, id
        sp->wIndex = JS_IFACE;
        sp->wLength = 64;
        t->device_handle = s_dev;
        t->bEndpointAddress = 0;
        t->num_bytes = sizeof(usb_setup_packet_t) + 64;
        t->callback = ctrl_cb;
        t->timeout_ms = 1000;
        if (usb_host_transfer_submit_control(s_client, t) != ESP_OK ||
            xSemaphoreTake(s_ctrl_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "feature 0x%02X: request failed", ids[k]);
            continue;
        }
        int n = t->actual_num_bytes - sizeof(usb_setup_packet_t);
        char line[3 * 64 + 8]; int o = 0;
        for (int i = 0; i < n && i < 64; i++) o += snprintf(line + o, sizeof(line) - o, "%02x ", t->data_buffer[sizeof(usb_setup_packet_t) + i]);
        ESP_LOGI(TAG, "feature 0x%02X status %d len %d: %s", ids[k], t->status, n, line);
    }
    usb_host_transfer_free(t);
    vTaskDelete(NULL);
}

static void client_event_cb(const usb_host_client_event_msg_t *msg, void *arg)
{
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        open_device(msg->new_dev.address);
    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        if (s_dev && msg->dev_gone.dev_hdl == s_dev) {
            ESP_LOGW(TAG, "Solaris Base removed");
            s_connected = false;
            usb_host_endpoint_halt(s_dev, JS_EP_IN);
            usb_host_endpoint_flush(s_dev, JS_EP_IN);
            usb_host_interface_release(s_client, s_dev, JS_IFACE);
            usb_host_device_close(s_client, s_dev);
            usb_host_transfer_free(s_xfer);
            s_xfer = NULL;
            s_dev = NULL;
        }
    }
}

static void host_task(void *arg)
{
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

static void client_task(void *arg)
{
    for (;;) usb_host_client_handle_events(s_client, portMAX_DELAY);
}

void joystick_start(void)
{
    usb_host_config_t cfg = { .skip_phy_setup = false, .intr_flags = ESP_INTR_FLAG_LEVEL1 };
    ESP_ERROR_CHECK(usb_host_install(&cfg));
    usb_host_client_config_t ccfg = {
        .is_synchronous = false, .max_num_event_msg = 5,
        .async = { .client_event_callback = client_event_cb, .callback_arg = NULL },
    };
    ESP_ERROR_CHECK(usb_host_client_register(&ccfg, &s_client));
    xTaskCreate(host_task, "usb_host", 4096, NULL, 10, NULL);
    xTaskCreate(client_task, "usb_client", 4096, NULL, 9, NULL);
}

bool joystick_get(joystick_state_t *out)
{
    if (!s_connected || s_state.seq == 0) return false;   // not valid until a real report has arrived
    portENTER_CRITICAL(&s_lock);
    *out = s_state;
    portEXIT_CRITICAL(&s_lock);
    return true;
}
