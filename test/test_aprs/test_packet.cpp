#include <unity.h>
#include "telemetry.h"

// --- APRS Packet Tests ---

void test_aprs_packet_contains_callsign() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {};
    alt.alt_cm = 50000;
    alt.state = STATE_ASCENT;
    alt.thrust = 1;
    alt.valid = true;
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.startsWith("N9RGK-1>APRS"));
}

void test_aprs_packet_contains_position() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {};
    alt.alt_cm = 0;
    alt.state = STATE_PAD_IDLE;
    alt.valid = true;
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("N/") > 0);
    TEST_ASSERT_TRUE(pkt.indexOf("W") > 0);
}

void test_aprs_comment_has_altitude() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {};
    alt.alt_cm = 167700;
    alt.vel_cms = -1200;
    alt.state = STATE_DESCENT;
    alt.valid = true;
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("Alt:1677m") > 0);
}

void test_aprs_comment_has_phase() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {};
    alt.alt_cm = 50000;
    alt.state = STATE_ASCENT;
    alt.thrust = 0;
    alt.valid = true;
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("Phase:COAST") > 0);
}

void test_aprs_comment_has_velocity() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {};
    alt.alt_cm = 50000;
    alt.vel_cms = 15000;
    alt.state = STATE_ASCENT;
    alt.thrust = 1;
    alt.valid = true;
    String pkt = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("V:+150m/s") > 0);
}

void test_aprs_boost_vs_coast() {
    GpsFix fix = {.valid = false};
    AltimeterData alt = {};
    alt.alt_cm = 10000;
    alt.state = STATE_ASCENT;
    alt.valid = true;

    alt.thrust = 1;
    String boost = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(boost.indexOf("Phase:BOOST") > 0);

    alt.thrust = 0;
    String coast = telemetry_build_aprs_packet(&fix, &alt);
    TEST_ASSERT_TRUE(coast.indexOf("Phase:COAST") > 0);
}

// --- Dense Packet Tests ---

void test_dense_packet_has_markers() {
    GpsFix fix = {.lat = 41.8827, .lng = -87.6233, .valid = true};
    AltimeterData alt = {};
    alt.alt_cm = 167700;
    alt.vel_cms = -1200;
    alt.max_alt_cm = 167700;
    alt.state = STATE_DESCENT;
    alt.flags = FLAG_P1_CONT | FLAG_P2_CONT | FLAG_P1_FIRED | FLAG_ARMED | FLAG_APOGEE;
    alt.p1_adc = 350;
    alt.p2_adc = 348;
    alt.seq = 1847;
    alt.valid = true;
    String pkt = telemetry_build_dense_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("{{T2:") > 0);
    TEST_ASSERT_TRUE(pkt.indexOf("}}") > 0);
}

void test_dense_packet_has_altitude() {
    GpsFix fix = {.valid = false};
    AltimeterData alt = {};
    alt.alt_cm = 167700;
    alt.state = STATE_DESCENT;
    alt.valid = true;
    String pkt = telemetry_build_dense_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("alt1677") > 0);
}

void test_dense_packet_has_sequence() {
    GpsFix fix = {.valid = false};
    AltimeterData alt = {};
    alt.alt_cm = 0;
    alt.seq = 42;
    alt.valid = true;
    String pkt = telemetry_build_dense_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("sq42") > 0);
}

void test_dense_no_altimeter() {
    GpsFix fix = {.valid = false};
    AltimeterData alt = {};
    alt.valid = false;
    String pkt = telemetry_build_dense_packet(&fix, &alt);
    TEST_ASSERT_TRUE(pkt.indexOf("noalt") > 0);
}

// --- Phase Name Tests ---

void test_phase_names() {
    TEST_ASSERT_EQUAL_STRING("PAD",     telemetry_phase_name(STATE_PAD_IDLE, 0));
    TEST_ASSERT_EQUAL_STRING("BOOST",   telemetry_phase_name(STATE_ASCENT, 1));
    TEST_ASSERT_EQUAL_STRING("COAST",   telemetry_phase_name(STATE_ASCENT, 0));
    TEST_ASSERT_EQUAL_STRING("DESCENT", telemetry_phase_name(STATE_DESCENT, 0));
    TEST_ASSERT_EQUAL_STRING("LANDED",  telemetry_phase_name(STATE_LANDED, 0));
}

int main() {
    UNITY_BEGIN();
    // APRS
    RUN_TEST(test_aprs_packet_contains_callsign);
    RUN_TEST(test_aprs_packet_contains_position);
    RUN_TEST(test_aprs_comment_has_altitude);
    RUN_TEST(test_aprs_comment_has_phase);
    RUN_TEST(test_aprs_comment_has_velocity);
    RUN_TEST(test_aprs_boost_vs_coast);
    // Dense
    RUN_TEST(test_dense_packet_has_markers);
    RUN_TEST(test_dense_packet_has_altitude);
    RUN_TEST(test_dense_packet_has_sequence);
    RUN_TEST(test_dense_no_altimeter);
    // Phase names
    RUN_TEST(test_phase_names);
    return UNITY_END();
}
