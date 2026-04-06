#pragma once

#include <stdint.h>

// All fields from the $PYRO NMEA sentence.
// Values are integer, metric, matching the serial protocol exactly.
//
// The flags field is 16-bit to support extended event flags from the
// altimeter's deployment/failure detection logic. Bits 0-5 are the
// original flags; bits 6-11 are extended flags (EFLAG_*).
// If the altimeter sends 2-char hex (old firmware), only bits 0-7 are
// populated. If it sends 4-char hex (new firmware), all 16 bits are used.
typedef struct {
    uint16_t seq;           // Sequence number (0–65535)
    uint8_t  state;         // 0=PAD_IDLE, 1=ASCENT, 2=DESCENT, 3=LANDED
    uint8_t  thrust;        // 1=thrust detected, 0=no thrust
    int32_t  alt_cm;        // Altitude in centimeters (relative to ground)
    int32_t  vel_cms;       // Vertical velocity in cm/s (positive=up)
    int32_t  max_alt_cm;    // Max altitude this flight, cm
    int32_t  press_pa;      // Filtered pressure in pascals
    uint32_t flight_time_ms;// Flight time in ms since launch (0 on pad)
    uint16_t flags;         // Bitfield: bits 0-5 original, bits 6-11 extended (EFLAG_*)
    uint16_t p1_adc;        // Pyro 1 raw ADC (0–4095)
    uint16_t p2_adc;        // Pyro 2 raw ADC (0–4095)
    uint16_t batt_adc;      // Battery voltage raw ADC (0 until implemented)
    int16_t  temp_deci_c;   // Temperature in tenths of °C (0 until implemented)
    bool     valid;         // True once at least one good $PYRO sentence received
    uint32_t last_rx_ms;    // millis() of last successful parse

    // --- Event sentences from altimeter ---
    // These flags are set when the corresponding $PYRO_* event sentence
    // is received. The event_detector consumes them and clears via
    // altimeter_rx_clear_events().

    bool     apogee_event;          // $PYRO_APO received
    int32_t  apogee_max_alt_cm;     // max_alt_cm from $PYRO_APO
    uint32_t apogee_time_ms;        // flight_time_ms from $PYRO_APO

    bool     pyro1_fire_event;      // $PYRO_FIRE with channel=1 received
    int32_t  pyro1_fire_alt_cm;     // alt_cm from $PYRO_FIRE ch=1
    uint32_t pyro1_fire_time_ms;    // flight_time_ms from $PYRO_FIRE ch=1

    bool     pyro2_fire_event;      // $PYRO_FIRE with channel=2 received
    int32_t  pyro2_fire_alt_cm;     // alt_cm from $PYRO_FIRE ch=2
    uint32_t pyro2_fire_time_ms;    // flight_time_ms from $PYRO_FIRE ch=2

    bool     landing_event;         // $PYRO_LAND received
    int32_t  landing_max_alt_cm;    // max_alt_cm from $PYRO_LAND
    uint32_t landing_time_ms;       // flight_time_ms from $PYRO_LAND
} AltimeterData;

void altimeter_rx_init();
void altimeter_rx_update();
const AltimeterData* altimeter_rx_get();
void altimeter_rx_clear_events();
