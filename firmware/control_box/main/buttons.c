// Logical control buttons. One is a physical GPIO button (RC override enable); all others come from joystick buttons
// through joystickMap. Everything is debounced and edge-detected here, in buttonTask.
#include "buttons.h"

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "joystick.h"

#define NO_SOURCE              ( -1 )
#define POLL_MS                5U
#define STABLE_SAMPLES         3U
#define PULLUP_SETTLE_MS       100U
#define AXIS_COUNT             6U
#define AXIS_LOG_STEP          4000U
#define RATE_LOG_POLLS         1000U
#define VENDOR_LOG_INTERVAL_US 200000
#define VENDOR_TEXT_LIMIT      140

static const char* logTag = "buttons";

// Physical GPIO per logical button. Only the override enable is a GPIO, wired between GPIO3 and GND (internal pull-up).
static const int32_t gpioMap[buttonCount] = {
    [buttonMode1] = NO_SOURCE,
    [buttonMode2] = NO_SOURCE,
    [buttonMode3] = NO_SOURCE,
    [buttonMode4] = NO_SOURCE,
    [buttonArm] = NO_SOURCE,
    [buttonDisarm] = NO_SOURCE,
    [buttonMotorOn] = NO_SOURCE,
    [buttonMotorOff] = NO_SOURCE,
    [buttonTrimUp] = NO_SOURCE,
    [buttonTrimDown] = NO_SOURCE,
    [buttonEstopA] = NO_SOURCE,
    [buttonEstopB] = NO_SOURCE,
    [buttonOverrideToggle] = ( int32_t )GPIO_NUM_3,
};

// Joystick button bit (bit n = joystick button n + 1) per logical button, from the learning session (docs/pinout.md).
// Arm is the trigger and needs a 0.8 s hold (control task); the e-stop needs buttons 15 and 13 held together.
static const int32_t joystickMap[buttonCount] = {
    [buttonMode1] = 4, [buttonMode2] = 5, [buttonMode3] = 6, [buttonMode4] = 7, [buttonArm] = 0, [buttonDisarm] = 1, [buttonMotorOn] = 2, [buttonMotorOff] = 3, [buttonTrimUp] = 26, [buttonTrimDown] = 25, [buttonEstopA] = 14, [buttonEstopB] = 12, [buttonOverrideToggle] = NO_SOURCE,
};

static portMUX_TYPE stateLock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t heldMask;
static volatile uint32_t pressEvents;

// Undebounced level of one logical button; true = pressed.
static bool rawLevel( uint32_t button, bool joystickOk, uint64_t joystickButtons )
{
    bool pressed = false;

    if ( gpioMap[button] >= 0 ) { pressed = ( gpio_get_level( ( gpio_num_t )gpioMap[button] ) == 0 ); }
    else if ( ( joystickMap[button] >= 0 ) && joystickOk ) { pressed = ( ( joystickButtons >> ( uint32_t )joystickMap[button] ) & 1ULL ) != 0ULL; }
    else { pressed = false; }

    return pressed;
}

// Learning-session aid: logs every joystick button edge so physical buttons can be identified.
static void logButtonEdges( uint64_t buttons, uint64_t previousButtons )
{
    const uint64_t changed = buttons ^ previousButtons;

    for ( uint32_t bit = 0U; bit < JOYSTICK_BUTTON_COUNT; bit++ )
    {
        if ( ( ( changed >> bit ) & 1ULL ) != 0ULL ) { ESP_LOGI( logTag, "joystick button %" PRIu32 " (bit %" PRIu32 ") %s", bit + 1U, bit, ( ( ( buttons >> bit ) & 1ULL ) != 0ULL ) ? "PRESSED" : "released" ); }
    }
}

// Identifies spare axes (e.g. the slider) by logging those that moved noticeably since they were last logged.
static void logAxisMoves( const joystickState_t* joystick )
{
    static const char* const names[AXIS_COUNT] = { "Z", "Rx", "Ry", "Rz", "Slider", "Dial" };
    static uint16_t logged[AXIS_COUNT];
    static bool haveLogged = false;

    const uint16_t axes[AXIS_COUNT] = { joystick->z, joystick->rx, joystick->ry, joystick->rz, joystick->slider, joystick->dial };

    if ( !haveLogged )
    {
        ( void )memcpy( logged, axes, sizeof( logged ) );
        haveLogged = true;
    }

    for ( uint32_t axis = 0U; axis < AXIS_COUNT; axis++ )
    {
        if ( ( ( uint32_t )axes[axis] > ( ( uint32_t )logged[axis] + AXIS_LOG_STEP ) ) || ( ( ( uint32_t )axes[axis] + AXIS_LOG_STEP ) < ( uint32_t )logged[axis] ) )
        {
            ESP_LOGI( logTag, "axis %s = %u", names[axis], axes[axis] );
            logged[axis] = axes[axis];
        }
    }
}

// Logs the report rate every 5 s (1000 polls); the stick only reports on change, so a low idle rate is normal.
static void logReportRate( bool joystickOk, uint32_t seq )
{
    static uint32_t pollCount    = 0U;
    static uint32_t seqAtLastLog = 0U;

    pollCount++;

    if ( pollCount >= RATE_LOG_POLLS )
    {
        pollCount = 0U;
        ESP_LOGI( logTag, "joystick %s, %" PRIu32 " reports in last 5 s", joystickOk ? "ok" : "NOT connected", seq - seqAtLastLog );
        seqAtLastLog = seq;
    }
}

// Diagnostics from the slider investigation: logs vendor report bytes 29..63 that change, at most every 200 ms.
static void logVendorChanges( const joystickState_t* joystick )
{
    static uint8_t previous[JOYSTICK_VENDOR_LEN];
    static bool havePrevious = false;
    static int64_t lastLogUs = 0;

    if ( havePrevious )
    {
        char text[160];
        int32_t offset = 0;

        text[0] = '\0';

        for ( uint32_t index = 0U; ( index < JOYSTICK_VENDOR_LEN ) && ( offset < VENDOR_TEXT_LIMIT ); index++ )
        {
            if ( joystick->vendor[index] != previous[index] ) { offset += snprintf( &text[offset], sizeof( text ) - ( size_t )offset, " [%" PRIu32 "]=%u", JOYSTICK_VENDOR_OFFSET + index, joystick->vendor[index] ); }
        }

        if ( ( offset > 0 ) && ( ( esp_timer_get_time() - lastLogUs ) > VENDOR_LOG_INTERVAL_US ) )
        {
            ESP_LOGI( logTag, "vendor bytes changed:%s", text );
            lastLogUs = esp_timer_get_time();
        }
    }

    ( void )memcpy( previous, joystick->vendor, JOYSTICK_VENDOR_LEN );
    havePrevious = true;
}

// On joystick (re)connect, whatever is already pressed counts as held, without events.
static void seedJoystickButtons( uint64_t joystickButtons, uint8_t* stableCount )
{
    taskENTER_CRITICAL( &stateLock );

    for ( uint32_t button = 0U; button < ( uint32_t )buttonCount; button++ )
    {
        if ( joystickMap[button] >= 0 )
        {
            if ( rawLevel( button, true, joystickButtons ) ) { heldMask |= BTN_BIT( button ); }
            else { heldMask &= ~BTN_BIT( button ); }

            stableCount[button] = 0U;
        }
    }

    taskEXIT_CRITICAL( &stateLock );
    ESP_LOGI( logTag, "joystick connected, buttons held at connect: 0x%011llx", ( unsigned long long )joystickButtons );
}

// Polls all sources every 5 ms; a level must be stable for 3 samples (15 ms) before it changes the held state.
static void buttonTask( void* arg )
{
    uint8_t stableCount[buttonCount];
    bool joystickWasOk       = false;
    uint64_t previousButtons = 0U;

    ( void )arg;
    ( void )memset( stableCount, 0, sizeof( stableCount ) );

    for ( ;; )
    {
        joystickState_t joystick;

        ( void )memset( &joystick, 0, sizeof( joystick ) );
        const bool joystickOk = joystickGet( &joystick );

        if ( joystickOk && joystickWasOk ) { logButtonEdges( joystick.buttons, previousButtons ); }

        if ( joystickOk ) { logAxisMoves( &joystick ); }

        logReportRate( joystickOk, joystick.seq );

        if ( joystickOk ) { logVendorChanges( &joystick ); }

        previousButtons = joystickOk ? joystick.buttons : 0U;

        if ( joystickOk && !joystickWasOk ) { seedJoystickButtons( joystick.buttons, stableCount ); }

        joystickWasOk = joystickOk;

        for ( uint32_t button = 0U; button < ( uint32_t )buttonCount; button++ )
        {
            const bool raw     = rawLevel( button, joystickOk, joystick.buttons );
            const bool current = ( ( heldMask >> button ) & 1U ) != 0U;

            if ( raw == current ) { stableCount[button] = 0U; }
            else if ( !raw && ( joystickMap[button] >= 0 ) && !joystickOk )
            {
                // Joystick gone: release immediately, without an event.
                taskENTER_CRITICAL( &stateLock );
                heldMask &= ~BTN_BIT( button );
                taskEXIT_CRITICAL( &stateLock );
                stableCount[button] = 0U;
            }
            else
            {
                stableCount[button]++;

                if ( stableCount[button] >= STABLE_SAMPLES )
                {
                    stableCount[button] = 0U;
                    taskENTER_CRITICAL( &stateLock );

                    if ( raw )
                    {
                        heldMask |= BTN_BIT( button );
                        pressEvents |= BTN_BIT( button );
                    }
                    else { heldMask &= ~BTN_BIT( button ); }

                    taskEXIT_CRITICAL( &stateLock );
                }
            }
        }

        vTaskDelay( pdMS_TO_TICKS( POLL_MS ) );
    }
}

void buttonsStart( void )
{
    uint32_t bootMask = 0U;

    for ( uint32_t button = 0U; button < ( uint32_t )buttonCount; button++ )
    {
        if ( gpioMap[button] >= 0 )
        {
            const gpio_config_t config = { .pin_bit_mask = 1ULL << ( uint32_t )gpioMap[button], .mode = GPIO_MODE_INPUT, .pull_up_en = GPIO_PULLUP_ENABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE };
            ESP_ERROR_CHECK( gpio_config( &config ) );
        }
    }

    // A pin already low at boot counts as held since boot and cannot produce a press event until released:
    // a board pin pulled low at boot once sent an arm command (PLAN.md, incident 2026-10-01).
    vTaskDelay( pdMS_TO_TICKS( PULLUP_SETTLE_MS ) );

    for ( uint32_t button = 0U; button < ( uint32_t )buttonCount; button++ )
    {
        if ( ( gpioMap[button] >= 0 ) && ( gpio_get_level( ( gpio_num_t )gpioMap[button] ) == 0 ) ) { bootMask |= BTN_BIT( button ); }
    }

    heldMask    = bootMask;
    pressEvents = 0U;

    if ( bootMask != 0U ) { ESP_LOGW( logTag, "pins LOW at boot (ignored until released): mask 0x%04" PRIx32, bootMask ); }

    ( void )xTaskCreate( buttonTask, "buttons", 4096, NULL, 6, NULL );
}

uint32_t buttonsHeld( void ) { return heldMask; }

uint32_t buttonsPressedEvents( void )
{
    uint32_t events = 0U;

    taskENTER_CRITICAL( &stateLock );
    events      = pressEvents;
    pressEvents = 0U;
    taskEXIT_CRITICAL( &stateLock );

    return events;
}
