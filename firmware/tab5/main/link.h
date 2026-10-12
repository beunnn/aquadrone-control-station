#ifndef LINK_H
#define LINK_H

#include <stdbool.h>
#include <stdint.h>

#define LINK_MAX_VEHICLES 16
#define LINK_BATT_UNKNOWN ( -1 )
#define LINK_SATS_UNKNOWN 255U

// Vehicle seen on the hub, keyed by sysid (autopilot component only).
typedef struct
{
    uint8_t sysid;

    // MAV_TYPE from the heartbeat.
    uint8_t type;
    bool armed;

    // ArduPilot custom_mode.
    uint32_t mode;
    int64_t lastHeartbeatUs;
    bool hasBattery;
    uint16_t batteryMv;

    // LINK_BATT_UNKNOWN when the autopilot does not estimate it.
    int8_t batteryPercent;
    bool hasGps;

    // GPS_FIX_TYPE.
    uint8_t gpsFix;

    // LINK_SATS_UNKNOWN when not reported.
    uint8_t gpsSats;
} vehicle_t;

// Starts the MAVLink UDP link to the hub (receive-only apart from a 1 Hz heartbeat). Called once from app_main.
void linkStart( void );

// Copies up to maxCount vehicles, sorted by sysid, into out; returns the number copied. Used by the UI refresh timer.
int32_t linkVehicles( vehicle_t* out, int32_t maxCount );

// Time of the last MAVLink message from the hub, 0 if none yet.
int64_t linkLastRxUs( void );

#endif
