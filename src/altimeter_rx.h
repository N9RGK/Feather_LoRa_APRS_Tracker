#pragma once

#include <stdint.h>

typedef struct {
    float pressure_hpa;
    float altitude_m;
    uint8_t flight_state;
    bool valid;
} AltimeterData;

void altimeter_rx_init();
void altimeter_rx_update();
const AltimeterData* altimeter_rx_get();
