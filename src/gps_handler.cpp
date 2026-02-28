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
