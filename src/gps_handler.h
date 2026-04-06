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

// GPS date/time accessors for session ID generation
bool gps_datetime_valid();
uint16_t gps_date_year();
uint8_t  gps_date_month();
uint8_t  gps_date_day();
uint8_t  gps_time_hour();
uint8_t  gps_time_minute();
uint8_t  gps_time_second();
