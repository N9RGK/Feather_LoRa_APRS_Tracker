#include <unity.h>
#include "telemetry.h"

void test_aprs_packet_contains_callsign() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {.altitude_m = 500.0f, .flight_state = 1, .valid = true};
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.startsWith("N9RGK-1>APRS"));
}

void test_aprs_packet_contains_position() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {.altitude_m = 0.0f, .flight_state = 0, .valid = true};
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    // Should contain uncompressed position with N and W
    TEST_ASSERT_TRUE(pkt.indexOf("N/") > 0);
    TEST_ASSERT_TRUE(pkt.indexOf("W") > 0);
}

void test_aprs_packet_contains_altitude() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {.altitude_m = 5500.0f, .flight_state = 3, .valid = true};
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("Alt:5500m") > 0);
}

void test_aprs_packet_contains_state_name() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {.altitude_m = 5500.0f, .flight_state = 3, .valid = true};
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("St:APOGEE") > 0);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_aprs_packet_contains_callsign);
    RUN_TEST(test_aprs_packet_contains_position);
    RUN_TEST(test_aprs_packet_contains_altitude);
    RUN_TEST(test_aprs_packet_contains_state_name);
    return UNITY_END();
}
