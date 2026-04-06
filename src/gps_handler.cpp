#include "gps_handler.h"
#include <TinyGPSPlus.h>
#include <Arduino.h>

static TinyGPSPlus gps;
static GpsFix fix = {};

void gps_handler_init() {
    Serial1.begin(9600);  // GPS FeatherWing on Serial1
}

void gps_handler_update() {
    while (Serial1.available()) {
        gps.encode(Serial1.read());
    }
    if (gps.location.isValid()) {
        fix.lat         = gps.location.lat();
        fix.lng         = gps.location.lng();
        fix.altitude_m  = gps.altitude.meters();
        fix.speed_kmh   = gps.speed.kmph();
        fix.course_deg  = gps.course.deg();
        fix.satellites  = gps.satellites.value();
        fix.valid       = true;
    }
}

const GpsFix* gps_handler_get_fix() {
    return &fix;
}

bool gps_datetime_valid() {
    return gps.date.isValid() && gps.time.isValid();
}
uint16_t gps_date_year()    { return gps.date.year(); }
uint8_t  gps_date_month()   { return gps.date.month(); }
uint8_t  gps_date_day()     { return gps.date.day(); }
uint8_t  gps_time_hour()    { return gps.time.hour(); }
uint8_t  gps_time_minute()  { return gps.time.minute(); }
uint8_t  gps_time_second()  { return gps.time.second(); }
