#pragma once

#include <stdint.h>

typedef struct {
    double lat;
    double lng;
    float  altitude_m;
    float  speed_kmh;
    float  course_deg;
    uint8_t satellites;
    bool   valid;
} GpsFix;

void gps_handler_init();
void gps_handler_update();
const GpsFix* gps_handler_get_fix();
