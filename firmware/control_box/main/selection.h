#pragma once

// Starts the MQTT client that follows the shared vehicle selection; call once the network has an IP.
void selection_start(void);

// Latest selected vehicle sysid from the broker, 0 = nothing selected (safe default: no output).
int selection_get(void);
