#pragma once
#include <stdbool.h>
#include <stdint.h>

#define LINK_MAX_VEHICLES 16

// Vehicle seen on the hub, keyed by sysid (autopilot component only).
typedef struct {
    uint8_t sysid;
    uint8_t type;           // MAV_TYPE
    bool armed;
    uint32_t mode;          // ArduPilot custom_mode
    int64_t last_hb_us;
    bool has_batt;
    uint16_t batt_mv;
    int8_t batt_pct;        // -1 = unknown
    bool has_gps;
    uint8_t gps_fix;        // GPS_FIX_TYPE
    uint8_t gps_sats;       // 255 = unknown
} vehicle_t;

// Starts the MAVLink UDP link to the hub (receive-only apart from a 1 Hz heartbeat).
void link_start(void);

// Copies the vehicle table sorted by sysid; returns the number of entries.
int link_vehicles(vehicle_t *out, int max);

// Time of the last MAVLink message from the hub, 0 if none yet.
int64_t link_last_rx_us(void);
