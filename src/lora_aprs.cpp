#include "lora_aprs.h"
#include "gps_handler.h"
#include "altimeter_rx.h"
#include "telemetry.h"
#include "config.h"
#include <RadioLib.h>
#include <Arduino.h>

static RFM96 radio = new Module(LORA_CS_PIN, LORA_DIO0_PIN, LORA_RST_PIN, LORA_DIO1_PIN);

// 3-byte LoRa APRS header (CA2RXU / OE-style)
// Without this, receivers consume the first 3 bytes of the callsign as header
static const uint8_t OE_HEADER[] = {0x3C, 0xFF, 0x01};

// Transmit a raw packet string with the OE header prepended
static bool lora_transmit(const String& packet) {
    size_t pktLen = packet.length();
    uint8_t* buf = new uint8_t[3 + pktLen];
    if (!buf) return false;
    memcpy(buf, OE_HEADER, 3);
    memcpy(buf + 3, packet.c_str(), pktLen);
    int txState = radio.transmit(buf, 3 + pktLen);
    delete[] buf;
    if (txState != RADIOLIB_ERR_NONE) {
        Serial.print("  TX failed: ");
        Serial.println(txState);
        return false;
    }
    return true;
}

void lora_aprs_init() {
    int state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC_WORD, LORA_POWER_DBM);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("FAILED (");
        Serial.print(state);
        Serial.println(")");
    } else {
        Serial.println("ok");
        Serial.print("  Freq: ");
        Serial.print(LORA_FREQ_MHZ);
        Serial.print(" MHz, SF");
        Serial.print(LORA_SF);
        Serial.print(", BW ");
        Serial.print(LORA_BW_KHZ, 0);
        Serial.print(" kHz, ");
        Serial.print(LORA_POWER_DBM);
        Serial.println(" dBm");
    }
}

void lora_aprs_send() {
    const GpsFix* fix = gps_handler_get_fix();
    const AltimeterData* alt = altimeter_rx_get();

    String packet = telemetry_build_aprs_packet(fix, alt);

    Serial.print("[APRS TX] ");
    Serial.println(packet);
    Serial.print("  GPS: ");
    if (fix->valid) {
        Serial.print(fix->lat, 4);
        Serial.print(", ");
        Serial.print(fix->lng, 4);
        Serial.print(" (");
        Serial.print(fix->satellites);
        Serial.println(" sats)");
    } else {
        Serial.println("no fix");
    }
    Serial.print("  Alt: ");
    if (alt->valid) {
        Serial.print(alt->alt_cm / 100);
        Serial.println("m");
    } else {
        Serial.println("no data");
    }

    lora_transmit(packet);
}

void lora_dense_send() {
    const GpsFix* fix = gps_handler_get_fix();
    const AltimeterData* alt = altimeter_rx_get();

    String packet = telemetry_build_dense_packet(fix, alt);

    Serial.print("[DENSE TX] ");
    Serial.println(packet);

    lora_transmit(packet);
}
