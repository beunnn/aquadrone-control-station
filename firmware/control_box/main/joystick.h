#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <stdbool.h>
#include <stdint.h>

#define JOYSTICK_BUTTON_COUNT  44U
#define JOYSTICK_VENDOR_OFFSET 29U
#define JOYSTICK_VENDOR_LEN    35U

// One decoded Solaris Base input report. Axes are raw 0..65535 with the centre near 32768.
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t rz;
    uint16_t ry;
    uint16_t rx;
    uint16_t slider;
    uint16_t dial;

    // 0..7 = direction, 8 and above = centred.
    uint8_t hat;

    // Bit n = joystick button n + 1.
    uint64_t buttons;

    // Report bytes 29..63; the descriptor declares them constant, kept for diagnostics.
    uint8_t vendor[JOYSTICK_VENDOR_LEN];

    // Increments per received report; 0 means no report has arrived since the device attached.
    uint32_t seq;
} joystickState_t;

// Installs the USB host stack and starts reading a Thrustmaster Solaris Base (044f:0422). Called once from app_main.
void joystickStart( void );

// Copies the latest report into out. Returns false (out untouched) while no joystick is attached or no report has arrived yet.
bool joystickGet( joystickState_t* out );

#endif
