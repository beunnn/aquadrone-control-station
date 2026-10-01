#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t x, y, z, rz, ry, rx, slider, dial;  // raw 0..65535, centre ~32768
    uint8_t hat;                                  // 0..7, 8+ = centred
    uint64_t buttons;                             // bit n = button n+1 (44 used)
    uint8_t vendor[35];                           // report bytes 29..63 (descriptor says constant; may hold slider data)
    uint32_t seq;                                 // increments per report
} joystick_state_t;

// Start USB host and read a Thrustmaster Solaris Base (044f:0422).
void joystick_start(void);
// Copy the latest state. Returns false if no joystick is connected.
bool joystick_get(joystick_state_t *out);
