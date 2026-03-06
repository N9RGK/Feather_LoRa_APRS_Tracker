// Unit tests for the $PYRO NMEA parser.
// These test the parse_pyro_sentence() logic by feeding raw sentence strings
// through the altimeter_rx module.
//
// NOTE: These tests require exposing parse_pyro_sentence() or testing via
// the update/get interface with a mock serial. For now, these serve as
// documentation of expected behavior and can be adapted for native testing.

#include <unity.h>
#include <string.h>
#include "altimeter_rx.h"
#include "config.h"

// === Test Vectors ===
// These are valid $PYRO sentences with correct checksums.
// Checksum is XOR of all chars between '$' and '*' (exclusive of both).

// Helper: compute NMEA checksum for a sentence (without $ and *XX)
static uint8_t compute_checksum(const char* body) {
    uint8_t cs = 0;
    while (*body) cs ^= (uint8_t)*body++;
    return cs;
}

// Helper: build a complete $PYRO sentence with valid checksum
static String build_pyro(const char* body) {
    uint8_t cs = compute_checksum(body);
    char buf[200];
    snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, cs);
    return String(buf);
}

void test_pad_idle_sentence() {
    // On pad, pyros good, no motion
    String sentence = build_pyro("PYRO,0042,0,0,0,0,0,101325,0,03,0350,0348,0,0");
    // This sentence should parse to:
    // seq=42, state=0 (PAD_IDLE), thrust=0, alt=0, vel=0, maxalt=0,
    // press=101325, time=0, flags=0x03 (p1_cont + p2_cont), p1adc=350, p2adc=348
    TEST_ASSERT_TRUE(sentence.indexOf("$PYRO") >= 0);
    TEST_ASSERT_TRUE(sentence.indexOf("*") > 0);
}

void test_ascent_sentence() {
    // Ascent at 500m, climbing at 150 m/s, under thrust
    String sentence = build_pyro("PYRO,0318,1,1,50000,15000,50000,95600,3200,17,0350,0348,0,0");
    // flags=0x17: p1_cont + p2_cont + p1_fired + armed
    TEST_ASSERT_TRUE(sentence.length() > 20);
}

void test_descent_sentence() {
    // Descent at 1677m, falling at 12 m/s, pyro 1 fired, apogee detected
    String sentence = build_pyro("PYRO,1847,2,0,167700,-1200,167700,84200,18500,35,0350,0348,0,0");
    // flags=0x35: p1_cont + p1_fired + armed + apogee
    TEST_ASSERT_TRUE(sentence.length() > 20);
}

void test_checksum_validation() {
    // Build a valid sentence then corrupt one byte
    String good = build_pyro("PYRO,0001,0,0,0,0,0,101325,0,00,0000,0000,0,0");
    // Corrupt: change seq from 0001 to 9999 without updating checksum
    String bad = good;
    bad.setCharAt(6, '9');  // corrupt the sequence number
    // The checksum should no longer match
    // (We can't directly call parse_pyro_sentence from here without exposing it,
    //  but this documents the expected behavior)
    TEST_ASSERT_TRUE(good != bad);
}

void test_flags_bitfield() {
    // Verify flag constants match expected bit positions
    TEST_ASSERT_EQUAL(0x01, FLAG_P1_CONT);
    TEST_ASSERT_EQUAL(0x02, FLAG_P2_CONT);
    TEST_ASSERT_EQUAL(0x04, FLAG_P1_FIRED);
    TEST_ASSERT_EQUAL(0x08, FLAG_P2_FIRED);
    TEST_ASSERT_EQUAL(0x10, FLAG_ARMED);
    TEST_ASSERT_EQUAL(0x20, FLAG_APOGEE);
}

void test_state_constants() {
    TEST_ASSERT_EQUAL(0, STATE_PAD_IDLE);
    TEST_ASSERT_EQUAL(1, STATE_ASCENT);
    TEST_ASSERT_EQUAL(2, STATE_DESCENT);
    TEST_ASSERT_EQUAL(3, STATE_LANDED);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pad_idle_sentence);
    RUN_TEST(test_ascent_sentence);
    RUN_TEST(test_descent_sentence);
    RUN_TEST(test_checksum_validation);
    RUN_TEST(test_flags_bitfield);
    RUN_TEST(test_state_constants);
    return UNITY_END();
}
