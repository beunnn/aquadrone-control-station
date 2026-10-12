// Selector screen: status bar, current selection and one card per discovered vehicle; tapping a card selects it.
// Everything below runs in the LVGL task (timer and event callbacks), so no extra locking is needed.
#include "ui.h"

#include <inttypes.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "link.h"
#include "lvgl.h"
#include "numericTypes.h"
#include "selection.h"
#include "wifi.h"

#define REFRESH_MS       250U
#define HB_STALE_US      3000000
#define COLOR_SELECTED   0x2E7D32U
#define COLOR_PENDING    0xF9A825U
#define COLOR_IDLE       0x9E9E9EU
#define COLOR_WARN       0xC62828U
#define COLOR_TEXT       0xFFFFFFU
#define COLOR_CARD       0x263238U
#define COLOR_BACKGROUND 0x101418U
#define TEXT_LEN         32U
#define IP_TEXT_LEN      16U
#define HUB_TEXT_LEN     24U

// One vehicle card; created on first sight of a vehicle and kept for the lifetime of the screen.
typedef struct
{
    lv_obj_t* button;
    lv_obj_t* title;
    lv_obj_t* detail;
    uint8_t sysid;
} card_t;

static const char* logTag = "ui";
static lv_obj_t* statusLabel;
static lv_obj_t* selectedLabel;
static lv_obj_t* cardList;
static lv_obj_t* emptyLabel;
static card_t cards[LINK_MAX_VEHICLES];
static int32_t cardCount = 0;

static const char* roverModeName( uint32_t mode )
{
    const char* name = "?";

    switch ( mode )
    {
        case 0U:
            name = "MANUAL";
            break;

        case 1U:
            name = "ACRO";
            break;

        case 3U:
            name = "STEERING";
            break;

        case 4U:
            name = "HOLD";
            break;

        case 5U:
            name = "LOITER";
            break;

        case 6U:
            name = "FOLLOW";
            break;

        case 7U:
            name = "SIMPLE";
            break;

        case 8U:
            name = "DOCK";
            break;

        case 9U:
            name = "CIRCLE";
            break;

        case 10U:
            name = "AUTO";
            break;

        case 11U:
            name = "RTL";
            break;

        case 12U:
            name = "SMART_RTL";
            break;

        case 15U:
            name = "GUIDED";
            break;

        case 16U:
            name = "INITIALISING";
            break;

        default:
            name = "?";
            break;
    }

    return name;
}

// LV_EVENT_CLICKED on a card; the sysid travels in the event user data.
static void onCardClicked( lv_event_t* event )
{
    const int32_t sysid = ( int32_t )( intptr_t )lv_event_get_user_data( event );

    if ( sysid == selectionCurrent() ) { return; }

    if ( !selectionRequest( sysid ) ) { ESP_LOGW( logTag, "select %" PRId32 " failed: broker not connected", sysid ); }
}

// Returns the card for sysid, creating it on first use; NULL when all cards are in use.
static card_t* cardFor( uint8_t sysid )
{
    for ( int32_t index = 0; index < cardCount; index++ )
    {
        if ( cards[index].sysid == sysid ) { return &cards[index]; }
    }

    if ( cardCount == LINK_MAX_VEHICLES ) { return NULL; }

    card_t* card = &cards[cardCount];
    cardCount++;
    card->sysid  = sysid;
    card->button = lv_button_create( cardList );
    lv_obj_set_size( card->button, LV_PCT( 100 ), LV_SIZE_CONTENT );
    lv_obj_set_flex_flow( card->button, LV_FLEX_FLOW_COLUMN );
    lv_obj_set_style_pad_all( card->button, 20, 0 );
    lv_obj_set_style_pad_row( card->button, 8, 0 );
    lv_obj_set_style_bg_color( card->button, lv_color_hex( COLOR_CARD ), 0 );
    lv_obj_set_style_border_width( card->button, 6, 0 );
    ( void )lv_obj_add_event_cb( card->button, onCardClicked, LV_EVENT_CLICKED, ( void* )( intptr_t )sysid );
    card->title = lv_label_create( card->button );
    lv_obj_set_style_text_font( card->title, &lv_font_montserrat_28, 0 );
    card->detail = lv_label_create( card->button );
    lv_obj_set_style_text_font( card->detail, &lv_font_montserrat_20, 0 );

    return card;
}

static void updateCard( const vehicle_t* vehicle, int64_t now, int32_t selected, int32_t pending )
{
    card_t* card = cardFor( vehicle->sysid );

    if ( card == NULL ) { return; }

    const int64_t ageUs    = now - vehicle->lastHeartbeatUs;
    const bool stale       = ageUs > HB_STALE_US;
    const float32_t age    = ( float32_t )ageUs / 1e6f;
    char battery[TEXT_LEN] = "Batt --";
    char gps[TEXT_LEN]     = "GPS --";

    uint32_t border = COLOR_IDLE;

    lv_label_set_text_fmt( card->title, "Boat %2u   %10s   %10s", vehicle->sysid, vehicle->armed ? "Armed" : "Disarmed", roverModeName( vehicle->mode ) );

    if ( vehicle->hasBattery )
    {
        if ( vehicle->batteryPercent >= 0 ) { ( void )snprintf( battery, sizeof( battery ), "Batt %.1f V %d%%", ( float64_t )vehicle->batteryMv / 1000.0, vehicle->batteryPercent ); }
        else { ( void )snprintf( battery, sizeof( battery ), "Batt %.1f V", ( float64_t )vehicle->batteryMv / 1000.0 ); }
    }

    if ( vehicle->hasGps ) { ( void )snprintf( gps, sizeof( gps ), "GPS Fix %2u, %2u sats", vehicle->gpsFix, ( vehicle->gpsSats == LINK_SATS_UNKNOWN ) ? 0U : vehicle->gpsSats ); }

    lv_label_set_text_fmt( card->detail, "%s    %s     Link %s %.1f s", battery, gps, stale ? "LOST" : "ok", ( float64_t )age );

    if ( ( int32_t )vehicle->sysid == selected ) { border = COLOR_SELECTED; }
    else if ( ( int32_t )vehicle->sysid == pending ) { border = COLOR_PENDING; }
    else { border = COLOR_IDLE; }

    lv_obj_set_style_border_color( card->button, lv_color_hex( border ), 0 );
    lv_obj_set_style_text_color( card->title, lv_color_hex( stale ? COLOR_IDLE : ( vehicle->armed ? COLOR_WARN : COLOR_TEXT ) ), 0 );
    lv_obj_set_style_text_color( card->detail, lv_color_hex( stale ? COLOR_IDLE : COLOR_TEXT ), 0 );
}

static const char* ctrlboxText( devState_t state )
{
    const char* text = "?";

    if ( state == devOnline ) { text = "Online"; }
    else if ( state == devOffline ) { text = "Offline"; }
    else { text = "?"; }

    return text;
}

// LVGL timer callback, every REFRESH_MS: copies the shared state and updates all labels and cards.
static void refresh( lv_timer_t* timer )
{
    const int64_t now      = esp_timer_get_time();
    const int64_t rxUs     = linkLastRxUs();
    const int32_t selected = selectionCurrent();
    const int32_t pending  = selectionPending();
    char ip[IP_TEXT_LEN];
    char hub[HUB_TEXT_LEN];
    vehicle_t vehicleTable[LINK_MAX_VEHICLES];
    uint32_t selectedColor = COLOR_WARN;

    ( void )timer;
    const bool wifiOk = wifiIp( ip, sizeof( ip ) );

    if ( rxUs == 0 ) { ( void )snprintf( hub, sizeof( hub ), "No Data" ); }
    else { ( void )snprintf( hub, sizeof( hub ), "%.1f s", ( float64_t )( now - rxUs ) / 1e6 ); }

    lv_label_set_text_fmt( statusLabel, " Wi-Fi %s   |   MQTT %s   |   Datahub %s   |   Control Box %s", wifiOk ? ip : wifiStateText(), selectionBrokerConnected() ? "OK" : "Down", hub, ctrlboxText( selectionCtrlboxState() ) );

    if ( pending != 0 )
    {
        lv_label_set_text_fmt( selectedLabel, "Selecting boat %" PRId32 " ...", pending );
        selectedColor = COLOR_PENDING;
    }
    else if ( selected != 0 )
    {
        lv_label_set_text_fmt( selectedLabel, "Selected: boat %" PRId32, selected );
        selectedColor = COLOR_SELECTED;
    }
    else
    {
        lv_label_set_text( selectedLabel, "No boat selected" );
        selectedColor = COLOR_WARN;
    }

    lv_obj_set_style_text_color( selectedLabel, lv_color_hex( selectedColor ), 0 );

    const int32_t vehicleCount = linkVehicles( vehicleTable, LINK_MAX_VEHICLES );

    for ( int32_t index = 0; index < vehicleCount; index++ )
    {
        updateCard( &vehicleTable[index], now, selected, pending );
    }

    if ( vehicleCount > 0 ) { lv_obj_set_hidden( emptyLabel, true ); }
}

void uiStart( void )
{
    lv_obj_t* screen = lv_screen_active();

    lv_obj_set_style_bg_color( screen, lv_color_hex( COLOR_BACKGROUND ), 0 );
    lv_obj_set_style_text_color( screen, lv_color_hex( COLOR_TEXT ), 0 );
    lv_obj_set_flex_flow( screen, LV_FLEX_FLOW_COLUMN );
    lv_obj_set_style_pad_all( screen, 20, 0 );
    lv_obj_set_style_pad_row( screen, 16, 0 );

    statusLabel = lv_label_create( screen );
    lv_obj_set_width( statusLabel, LV_PCT( 100 ) );
    lv_label_set_long_mode( statusLabel, LV_LABEL_LONG_WRAP );
    lv_obj_set_style_text_font( statusLabel, &lv_font_montserrat_20, 0 );

    selectedLabel = lv_label_create( screen );
    lv_obj_set_style_text_font( selectedLabel, &lv_font_montserrat_40, 0 );

    cardList = lv_obj_create( screen );
    lv_obj_set_width( cardList, LV_PCT( 100 ) );
    lv_obj_set_flex_grow( cardList, 1 );
    lv_obj_set_flex_flow( cardList, LV_FLEX_FLOW_COLUMN );
    lv_obj_set_style_pad_row( cardList, 16, 0 );
    lv_obj_set_style_bg_opa( cardList, LV_OPA_TRANSP, 0 );
    lv_obj_set_style_border_width( cardList, 0, 0 );

    emptyLabel = lv_label_create( cardList );
    lv_obj_set_style_text_font( emptyLabel, &lv_font_montserrat_28, 0 );
    lv_label_set_text( emptyLabel, "Waiting for vehicle heartbeats ..." );

    refresh( NULL );
    ( void )lv_timer_create( refresh, REFRESH_MS, NULL );
}
