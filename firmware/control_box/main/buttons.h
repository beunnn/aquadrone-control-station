#ifndef BUTTONS_H
#define BUTTONS_H

#include <stdbool.h>
#include <stdint.h>

// Logical control buttons. Which GPIO or joystick button drives each one is mapped in buttons.c.
typedef enum
{
    // Modes: Manual, Hold, Auto, RTL.
    buttonMode1,
    buttonMode2,
    buttonMode3,
    buttonMode4,
    buttonArm,
    buttonDisarm,
    buttonMotorOn,
    buttonMotorOff,
    buttonTrimUp,
    buttonTrimDown,
    buttonEstopA,
    buttonEstopB,
    buttonOverrideToggle,
    buttonCount
} buttonId_t;

#define BTN_BIT( button ) ( ( uint32_t )1U << ( uint32_t )( button ) )

// Configures the GPIO buttons and starts the 5 ms debounce task. Called once from app_main before the joystick starts.
void buttonsStart( void );

// Debounced "currently held" bitmask (BTN_BIT per button); polled by the control task.
uint32_t buttonsHeld( void );

// Press events (rising edges) since the previous call, so none are lost between control-task polls.
uint32_t buttonsPressedEvents( void );

#endif
