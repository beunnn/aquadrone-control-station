#ifndef WIFI_H
#define WIFI_H

#include <stdbool.h>
#include <stddef.h>

// Starts the Wi-Fi station through the on-board ESP32-C6 and reconnects forever; also creates the default event loop.
// Never aborts: if the C6 does not respond, initialisation is retried in the background. Called once from app_main.
void wifiStart( void );

// Copies the current IP into buffer; returns false (and an empty string) while not connected.
bool wifiIp( char* buffer, size_t length );

// Short state text for the status bar while not connected.
const char* wifiStateText( void );

#endif
