#include "test_data.h"
#include "config.h"
#include <Arduino.h>

static GpsFix test_gps = {
    .lat        = 41.88267,
    .lng        = -87.62333,
    .altitude_m = 200.0f,
    .speed_kmh  = 0.0f,
    .course_deg = 0.0f,
    .satellites = 8,
    .valid      = true
};

static AltimeterData test_alt = {
    .seq            = 0,
    .state          = STATE_PAD_IDLE,
    .thrust         = 0,
    .alt_cm          = 0,
    .vel_cms         = 0,
    .max_alt_cm      = 0,
    .press_pa        = 101325,
    .flight_time_ms  = 0,
    .flags           = FLAG_P1_CONT | FLAG_P2_CONT,
    .p1_adc          = 350,
    .p2_adc          = 348,
    .batt_adc        = 3200,
    .temp_deci_c     = 210,
    .valid           = true,
    .last_rx_ms      = 0
};

const GpsFix* test_data_get_gps() {
    return &test_gps;
}

const AltimeterData* test_data_get_alt() {
    test_alt.last_rx_ms = millis();
    test_alt.seq++;
    return &test_alt;
}
