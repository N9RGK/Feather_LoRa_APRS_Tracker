#pragma once

#include <stdint.h>

// All fields from the $PYRO NMEA sentence.
// Values are integer, metric, matching the serial protocol exactly.
typedef struct {
    uint16_t seq;           // Sequence number (0–65535)
    uint8_t  state;         // 0=PAD_IDLE, 1=ASCENT, 2=DESCENT, 3=LANDED
    uint8_t  thrust;        // 1=thrust detected, 0=no thrust
    int32_t  alt_cm;        // Altitude in centimeters (relative to ground)
    int32_t  vel_cms;       // Vertical velocity in cm/s (positive=up)
    int32_t  max_alt_cm;    // Max altitude this flight, cm
    int32_t  press_pa;      // Filtered pressure in pascals
    uint32_t flight_time_ms;// Flight time in ms since launch (0 on pad)
    uint8_t  flags;         // Bitfield: p1_cont, p2_cont, p1_fired, p2_fired, armed, apogee
    uint16_t p1_adc;        // Pyro 1 raw ADC (0–4095)
    uint16_t p2_adc;        // Pyro 2 raw ADC (0–4095)
    uint16_t batt_adc;      // Battery voltage raw ADC (0 until implemented)
    int16_t  temp_deci_c;   // Temperature in tenths of °C (0 until implemented)
    bool     valid;         // True once at least one good $PYRO sentence received
    uint32_t last_rx_ms;    // millis() of last successful parse
} AltimeterData;

void altimeter_rx_init();
void altimeter_rx_update();
const AltimeterData* altimeter_rx_get();
