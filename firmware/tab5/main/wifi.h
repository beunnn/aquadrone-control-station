#pragma once
#include <stdbool.h>
#include <stddef.h>

// Starts the Wi-Fi station (via the on-board ESP32-C6) and reconnects forever. Also creates the default event loop.
// Never aborts: if the C6 does not respond, initialisation is retried in the background.
void wifi_start(void);

// Copies the current IP into buf; returns false (and an empty string) while not connected.
bool wifi_ip(char *buf, size_t len);

// Short state text for the status bar while not connected.
const char *wifi_state_text(void);
