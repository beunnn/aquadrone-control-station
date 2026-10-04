#pragma once
#include <stdbool.h>

typedef enum { DEV_UNKNOWN = -1, DEV_OFFLINE = 0, DEV_ONLINE = 1 } dev_state_t;

// Starts the MQTT client: follows the retained selection and the control-box presence, publishes our own presence.
void selection_start(void);

// Publishes a new selection (retained). Returns false if the broker is not connected.
bool selection_request(int sysid);

// Selection as confirmed by the broker (0 = none).
int selection_current(void);

// Last selection requested from this Tab5 that the broker has not echoed yet (0 = none pending).
int selection_pending(void);

bool selection_broker_connected(void);
dev_state_t selection_ctrlbox_state(void);
