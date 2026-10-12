#ifndef SELECTION_H
#define SELECTION_H

#include <stdint.h>

// Starts the MQTT client that follows the shared vehicle selection. Called once from app_main after the network has an IP.
void selectionStart( void );

// Latest selected vehicle sysid from the broker, 0 = nothing selected (safe default: the control task sends nothing).
int32_t selectionGet( void );

#endif
