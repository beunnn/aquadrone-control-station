#pragma once
#include <stdint.h>
#include <stdbool.h>

// Logical buttons, wired between the GPIO and GND (internal pull-up, active low).
typedef enum {
    BTN_MODE1, BTN_MODE2, BTN_MODE3, BTN_MODE4,  // Manual, Hold, Auto, RTL
    BTN_ARM, BTN_DISARM,
    BTN_MOTOR_ON, BTN_MOTOR_OFF,
    BTN_TRIM_UP, BTN_TRIM_DOWN,
    BTN_ESTOP_A, BTN_ESTOP_B,
    BTN_OVERRIDE_TOGGLE,
    BTN_COUNT
} button_t;

void buttons_start(void);
// Debounced "currently held" bitmask (bit n = button n).
uint32_t buttons_held(void);
// Press events since the last call (rising edges, never lost between polls).
uint32_t buttons_pressed_events(void);
#define BTN_BIT(b) (1u << (b))
