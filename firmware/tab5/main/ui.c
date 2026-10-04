// Selector screen: status bar, current selection and one card per discovered vehicle; tapping a card selects it.
// Everything below runs in the LVGL task (timer and event callbacks), so no extra locking is needed.
#include "ui.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "lvgl.h"
#include "link.h"
#include "selection.h"
#include "wifi.h"

#define REFRESH_MS      250
#define HB_STALE_US     3000000
#define COLOR_SELECTED  0x2E7D32
#define COLOR_PENDING   0xF9A825
#define COLOR_IDLE      0x9E9E9E
#define COLOR_WARN      0xC62828

typedef struct {
    lv_obj_t *btn;
    lv_obj_t *title;
    lv_obj_t *detail;
    uint8_t sysid;
} card_t;

static const char *TAG = "ui";
static lv_obj_t *s_status;
static lv_obj_t *s_selected;
static lv_obj_t *s_list;
static lv_obj_t *s_empty;
static card_t s_cards[LINK_MAX_VEHICLES];
static int s_ncards;

static const char *rover_mode_name(uint32_t mode)
{
    switch (mode) {
    case 0:  return "MANUAL";
    case 1:  return "ACRO";
    case 3:  return "STEERING";
    case 4:  return "HOLD";
    case 5:  return "LOITER";
    case 6:  return "FOLLOW";
    case 7:  return "SIMPLE";
    case 8:  return "DOCK";
    case 9:  return "CIRCLE";
    case 10: return "AUTO";
    case 11: return "RTL";
    case 12: return "SMART_RTL";
    case 15: return "GUIDED";
    case 16: return "INITIALISING";
    default: return "?";
    }
}

static void on_card_clicked(lv_event_t *e)
{
    int sysid = (int)(intptr_t)lv_event_get_user_data(e);
    if (sysid == selection_current()) return;
    if (!selection_request(sysid)) ESP_LOGW(TAG, "select %d failed: broker not connected", sysid);
}

static card_t *card_for(uint8_t sysid)
{
    for (int i = 0; i < s_ncards; i++) {
        if (s_cards[i].sysid == sysid) return &s_cards[i];
    }
    if (s_ncards == LINK_MAX_VEHICLES) return NULL;

    card_t *c = &s_cards[s_ncards++];
    c->sysid = sysid;
    c->btn = lv_button_create(s_list);
    lv_obj_set_size(c->btn, LV_PCT(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(c->btn, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_all(c->btn, 20, 0);
    lv_obj_set_style_pad_row(c->btn, 8, 0);
    lv_obj_set_style_bg_color(c->btn, lv_color_hex(0x263238), 0);
    lv_obj_set_style_border_width(c->btn, 6, 0);
    lv_obj_add_event_cb(c->btn, on_card_clicked, LV_EVENT_CLICKED, (void *)(intptr_t)sysid);
    c->title = lv_label_create(c->btn);
    lv_obj_set_style_text_font(c->title, &lv_font_montserrat_28, 0);
    c->detail = lv_label_create(c->btn);
    lv_obj_set_style_text_font(c->detail, &lv_font_montserrat_20, 0);
    return c;
}

static void update_card( const vehicle_t *v, int64_t now, int selected, int pending )
{
    card_t *c = card_for( v->sysid );

    if ( !c ) return;
    
    bool stale = ( now - v->last_hb_us ) > HB_STALE_US;
    float age  = ( now - v->last_hb_us ) / 1e6f;

    lv_label_set_text_fmt( c->title, "Boat %2u   %10s   %10s", v->sysid, v->armed ? "Armed" : "Disarmed", rover_mode_name( v->mode ) );

    char batt[32] = "Batt --";
    char gps[32]  = "GPS --";

    if (v->has_batt) {
        if (v->batt_pct >= 0) snprintf( batt, sizeof(batt), "Batt %.1f V %d%%", v->batt_mv / 1000.0f, v->batt_pct);
        else                  snprintf( batt, sizeof(batt), "Batt %.1f V",      v->batt_mv / 1000.0f );
    }

    if (v->has_gps) snprintf( gps, sizeof( gps ), "GPS Fix %2u, %2u sats", v->gps_fix, v->gps_sats == 255 ? 0 : v->gps_sats );
    
    lv_label_set_text_fmt( c->detail,  "%s    %s     Link %s %.1f s", batt, gps, stale ? "LOST" : "ok", age );

    uint32_t border = v->sysid == selected ? COLOR_SELECTED : (v->sysid == pending ? COLOR_PENDING : COLOR_IDLE );

    lv_obj_set_style_border_color( c->btn,  lv_color_hex( border ), 0 );
    lv_obj_set_style_text_color( c->title,  lv_color_hex( stale ? COLOR_IDLE : ( v->armed ? COLOR_WARN : 0xFFFFFF ) ), 0 );
    lv_obj_set_style_text_color( c->detail, lv_color_hex( stale ? COLOR_IDLE : 0xFFFFFF ), 0 );
}

static void refresh( lv_timer_t *t )
{
    int64_t now = esp_timer_get_time();
    char ip[16];
    bool wifi_ok = wifi_ip( ip, sizeof( ip ) );
    int64_t rx = link_last_rx_us();
    dev_state_t box = selection_ctrlbox_state();

    char hub[24];
    
    if ( rx == 0 ) snprintf( hub, sizeof( hub ), "No Data" );
    else           snprintf( hub, sizeof( hub ), "%.1f s", ( now - rx) / 1e6f );

    lv_label_set_text_fmt( s_status, " Wi-Fi %s   |   MQTT %s   |   Datahub %s   |   Control Box %s",
                           wifi_ok ? ip : wifi_state_text(), selection_broker_connected() ? "OK" : "Down", hub,
                           box == DEV_ONLINE ? "Online" : (box == DEV_OFFLINE ? "Offline" : "?" ) );

    int selected = selection_current();
    int pending  = selection_pending();

    /**/ if ( pending  ) lv_label_set_text_fmt( s_selected, "Selecting boat %d ...", pending );
    else if ( selected ) lv_label_set_text_fmt( s_selected, "Selected: boat %d", selected );
    else                 lv_label_set_text(     s_selected, "No boat selected" );

    lv_obj_set_style_text_color( s_selected, lv_color_hex( pending ? COLOR_PENDING : ( selected ? COLOR_SELECTED : COLOR_WARN ) ), 0 );

    vehicle_t veh[LINK_MAX_VEHICLES];
    int n = link_vehicles(veh, LINK_MAX_VEHICLES);

    for ( int i = 0; i < n; i++ ) 
    {
        update_card( &veh[i], now, selected, pending );
    }

    if ( n > 0 ) lv_obj_set_hidden( s_empty, true );
}

void ui_start( void )
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color( scr, lv_color_hex( 0x101418 ), 0 );
    lv_obj_set_style_text_color( scr, lv_color_hex( 0xFFFFFF ), 0 );
    lv_obj_set_flex_flow( scr, LV_FLEX_FLOW_COLUMN );
    lv_obj_set_style_pad_all( scr, 20, 0 );
    lv_obj_set_style_pad_row( scr, 16, 0 );

    s_status = lv_label_create( scr );
    lv_obj_set_width( s_status, LV_PCT( 100 ) );
    lv_label_set_long_mode( s_status, LV_LABEL_LONG_WRAP );
    lv_obj_set_style_text_font( s_status, &lv_font_montserrat_20, 0 );

    s_selected = lv_label_create( scr );
    lv_obj_set_style_text_font( s_selected, &lv_font_montserrat_40, 0 );

    s_list = lv_obj_create( scr );
    lv_obj_set_width( s_list, LV_PCT(100) );
    lv_obj_set_flex_grow( s_list, 1 );
    lv_obj_set_flex_flow( s_list, LV_FLEX_FLOW_COLUMN );
    lv_obj_set_style_pad_row( s_list, 16, 0 );
    lv_obj_set_style_bg_opa( s_list, LV_OPA_TRANSP, 0 );
    lv_obj_set_style_border_width( s_list, 0, 0 );

    s_empty = lv_label_create( s_list );
    lv_obj_set_style_text_font( s_empty, &lv_font_montserrat_28, 0 );
    lv_label_set_text( s_empty, "Waiting for vehicle heartbeats ..." );

    refresh( NULL );
    lv_timer_create( refresh, REFRESH_MS, NULL );
}
