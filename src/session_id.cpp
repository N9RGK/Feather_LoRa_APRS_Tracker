#include "session_id.h"
#include "gps_handler.h"
#include <Arduino.h>
#include <stdio.h>

static uint32_t s_session_id = 0;
static bool     s_valid = false;

static uint32_t gps_to_epoch(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t minute, uint8_t second) {
    uint32_t days = 0;
    for (uint16_t y = 1970; y < year; y++) {
        days += (y % 4 == 0 && (y % 100 != 0 || y % 400 == 0)) ? 366 : 365;
    }
    static const uint16_t mdays[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    days += mdays[month - 1];
    if (month > 2 && (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)))
        days++;
    days += day - 1;
    return days * 86400UL + hour * 3600UL + minute * 60UL + second;
}

void session_id_init() {
    s_session_id = 0;
    s_valid = false;
}

void session_id_update() {
    if (s_valid) return;

    if (!gps_datetime_valid()) return;

    uint16_t year = gps_date_year();
    if (year < 2024) return;  // sanity check — reject clearly invalid GPS data

    uint8_t month  = gps_date_month();
    uint8_t day    = gps_date_day();
    uint8_t hour   = gps_time_hour();
    uint8_t minute = gps_time_minute();
    uint8_t second = gps_time_second();

    s_session_id = gps_to_epoch(year, month, day, hour, minute, second);
    s_valid = true;

    char log_buf[64];
    snprintf(log_buf, sizeof(log_buf),
             "Session ID: %08lX (GPS: %04u-%02u-%02u %02u:%02u:%02u UTC)",
             (unsigned long)s_session_id,
             year, month, day, hour, minute, second);
    Serial.println(log_buf);
}

bool session_id_valid() {
    return s_valid;
}

uint32_t session_id_get() {
    return s_session_id;
}

bool session_id_hex(char* buf, size_t buf_size) {
    if (!s_valid) return false;
    snprintf(buf, buf_size, "%08lX", (unsigned long)s_session_id);
    return true;
}
