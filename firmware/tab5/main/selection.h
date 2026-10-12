#ifndef SELECTION_H
#define SELECTION_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    devUnknown = -1,
    devOffline = 0,
    devOnline = 1
} devState_t;

// Starts the MQTT client: follows the retained selection and the control-box presence, publishes our own presence.
// Called once from app_main; esp-mqtt keeps reconnecting by itself.
void selectionStart( void );

// Publishes a new selection (retained). Returns false if the broker is not connected. Called from the UI on a card tap.
bool selectionRequest( int32_t sysid );

// Selection as confirmed by the broker (0 = none).
int32_t selectionCurrent( void );

// Selection requested from this Tab5 that the broker has not echoed yet (0 = none pending).
int32_t selectionPending( void );

bool selectionBrokerConnected( void );

devState_t selectionCtrlboxState( void );

#endif
