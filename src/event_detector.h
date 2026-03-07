#pragma once

#include <stdint.h>
#include "altimeter_rx.h"
#include "gps_handler.h"
#include "config.h"

// --- Event Snapshot ---
// Captured at the instant an event is detected.
// Contains the altimeter values at the moment of the event.
typedef struct {
    uint8_t  code;          // Event code (EVT_* from config.h)
    uint32_t flight_time_s; // Flight time in seconds when event occurred
    int32_t  alt_m;         // Altitude in meters at event
    int32_t  vel_mps;       // Velocity in m/s at event (integer, signed)
    // GPS position — included for LANDED and REPORT events
    bool     has_gps;
    double   lat;
    double   lng;
    // Flight report extras (only for EVT_REPORT)
    uint8_t  error_flags;   // FLERR_* bitfield
} FlightEvent;

// Initialize the event detector. Call once in setup().
void event_detector_init();

// Check for state transitions in the latest altimeter data.
// Call every loop iteration after altimeter_rx_update().
// Detected events are pushed to the internal queue.
void event_detector_update(const AltimeterData* alt, const GpsFix* fix);

// Returns true if there are queued events waiting to be transmitted.
bool event_detector_has_events();

// Peek at the next event without removing it.
// Returns nullptr if queue is empty.
const FlightEvent* event_detector_peek();

// Remove the front event from the queue (after all repeats are sent).
void event_detector_pop();

// Reset detector state (e.g. when switching modes or on power-up).
void event_detector_reset();
