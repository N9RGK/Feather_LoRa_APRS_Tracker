#include "altimeter_rx.h"
#include "config.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "wiring_private.h"  // pinPeripheral()

// SERCOM1 hardware UART for altimeter data ($PYRO protocol at 115200 baud)
// RX: pin 11 (PA16, SERCOM1 PAD 0)
// TX: pin 10 (PA18, SERCOM1 PAD 2) — unused, satisfies SERCOM constraint
static Uart altSerial(&sercom1, ALT_RX_PIN, ALT_TX_PIN,
                      SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler() {
    altSerial.IrqHandler();
}

static AltimeterData data = {};

// NMEA line buffer — $PYRO sentences are ~80-120 chars max
#define LINE_BUF_SIZE 160
static char line_buf[LINE_BUF_SIZE];
static uint8_t line_idx = 0;

// Parse a hex string of variable length (e.g. "1B" -> 0x1B, "031F" -> 0x031F).
// Handles both 2-char (old altimeter, 8-bit flags) and 4-char (new altimeter,
// 16-bit flags with extended event detection bits).
static uint16_t parse_hex_word(const char* s) {
    uint16_t val = 0;
    for (int i = 0; i < 4 && s[i]; i++) {
        char c = s[i];
        uint8_t nibble;
        if (c >= '0' && c <= '9')      nibble = (c - '0');
        else if (c >= 'A' && c <= 'F') nibble = (c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') nibble = (c - 'a' + 10);
        else break;  // stop on non-hex char
        val = (val << 4) | nibble;
    }
    return val;
}

// Compute NMEA XOR checksum over characters between '$' and '*' (exclusive)
static uint8_t nmea_checksum(const char* sentence) {
    uint8_t cs = 0;
    const char* p = sentence + 1;  // skip '$'
    while (*p && *p != '*') {
        cs ^= (uint8_t)*p;
        p++;
    }
    return cs;
}

// Parse a complete $PYRO sentence into the data struct.
// Format: $PYRO,seq,state,thrust,alt,vel,maxalt,press,time,flags,p1adc,p2adc,batt,temp*XX
// The flags field accepts both 2-char hex (8-bit) and 4-char hex (16-bit).
// Returns true if parse succeeded and checksum matched.
static bool parse_pyro_sentence(const char* sentence) {
    // Verify prefix
    if (strncmp(sentence, "$PYRO,", 6) != 0) return false;

    // Find checksum delimiter
    const char* star = strchr(sentence, '*');
    if (!star) return false;

    // Verify checksum
    uint16_t expected_cs = parse_hex_word(star + 1);
    uint8_t actual_cs = nmea_checksum(sentence);
    if ((uint8_t)expected_cs != actual_cs) return false;

    // Parse CSV fields after "$PYRO,"
    // 13 fields: seq,state,thrust,alt,vel,maxalt,press,time,flags,p1adc,p2adc,batt,temp
    const char* p = sentence + 6;  // skip "$PYRO,"
    char field_buf[16];
    int field_idx = 0;

    uint16_t t_seq = 0;
    uint8_t  t_state = 0;
    uint8_t  t_thrust = 0;
    int32_t  t_alt = 0;
    int32_t  t_vel = 0;
    int32_t  t_maxalt = 0;
    int32_t  t_press = 0;
    uint32_t t_time = 0;
    uint16_t t_flags = 0;
    uint16_t t_p1adc = 0;
    uint16_t t_p2adc = 0;
    uint16_t t_batt = 0;
    int16_t  t_temp = 0;

    while (field_idx < 13 && *p && *p != '*') {
        // Extract field up to next comma or '*'
        int fi = 0;
        while (*p && *p != ',' && *p != '*' && fi < 15) {
            field_buf[fi++] = *p++;
        }
        field_buf[fi] = '\0';
        if (*p == ',') p++;  // skip comma

        switch (field_idx) {
            case 0:  t_seq    = (uint16_t)atol(field_buf); break;
            case 1:  t_state  = (uint8_t)atoi(field_buf);  break;
            case 2:  t_thrust = (uint8_t)atoi(field_buf);  break;
            case 3:  t_alt    = atol(field_buf);            break;
            case 4:  t_vel    = atol(field_buf);            break;
            case 5:  t_maxalt = atol(field_buf);            break;
            case 6:  t_press  = atol(field_buf);            break;
            case 7:  t_time   = (uint32_t)atol(field_buf);  break;
            case 8:  t_flags  = parse_hex_word(field_buf);  break;
            case 9:  t_p1adc  = (uint16_t)atol(field_buf); break;
            case 10: t_p2adc  = (uint16_t)atol(field_buf); break;
            case 11: t_batt   = (uint16_t)atol(field_buf); break;
            case 12: t_temp   = (int16_t)atol(field_buf);  break;
        }
        field_idx++;
    }

    if (field_idx < 13) return false;  // Not enough fields

    // Commit parsed data atomically
    data.seq            = t_seq;
    data.state          = t_state;
    data.thrust         = t_thrust;
    data.alt_cm         = t_alt;
    data.vel_cms        = t_vel;
    data.max_alt_cm     = t_maxalt;
    data.press_pa       = t_press;
    data.flight_time_ms = t_time;
    data.flags          = t_flags;
    data.p1_adc         = t_p1adc;
    data.p2_adc         = t_p2adc;
    data.batt_adc       = t_batt;
    data.temp_deci_c    = t_temp;
    data.valid          = true;
    data.last_rx_ms     = millis();

    return true;
}

void altimeter_rx_init() {
    altSerial.begin(ALTIMETER_BAUD);
    pinPeripheral(ALT_TX_PIN, PIO_SERCOM);  // pin 10 -> SERCOM1 PAD 2
    pinPeripheral(ALT_RX_PIN, PIO_SERCOM);  // pin 11 -> SERCOM1 PAD 0
}

void altimeter_rx_update() {
    while (altSerial.available()) {
        char c = altSerial.read();

        if (c == '$') {
            // Start of a new sentence — reset buffer
            line_idx = 0;
            line_buf[line_idx++] = c;
        } else if (c == '\n' || c == '\r') {
            // End of sentence — attempt parse
            if (line_idx > 0) {
                line_buf[line_idx] = '\0';
                parse_pyro_sentence(line_buf);
                line_idx = 0;
            }
        } else {
            // Accumulate character
            if (line_idx < LINE_BUF_SIZE - 1) {
                line_buf[line_idx++] = c;
            } else {
                // Overflow — discard this line
                line_idx = 0;
            }
        }
    }
}

const AltimeterData* altimeter_rx_get() {
    return &data;
}
