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

static bool configure_radio(const TrackerConfig* cfg) {
    int state = radio.begin(cfg->lora_freq_mhz, cfg->lora_bw_khz,
                            cfg->lora_sf, cfg->lora_cr,
                            cfg->lora_sync_word, cfg->lora_power_dbm);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("FAILED (");
        Serial.print(state);
        Serial.println(")");
        return false;
    }
    Serial.println("ok");
    Serial.print("  Freq: ");
    Serial.print(cfg->lora_freq_mhz);
    Serial.print(" MHz, SF");
    Serial.print(cfg->lora_sf);
    Serial.print(", BW ");
    Serial.print(cfg->lora_bw_khz, 0);
    Serial.print(" kHz, ");
    Serial.print(cfg->lora_power_dbm);
    Serial.println(" dBm");
    return true;
}

bool lora_aprs_init(const TrackerConfig* cfg) {
    return configure_radio(cfg);
}

bool lora_aprs_reconfigure(const TrackerConfig* cfg) {
    Serial.print("LoRa reconfigure... ");
    return configure_radio(cfg);
}

void lora_aprs_send() {
    const GpsFix* fix = gps_handler_get_fix();
    const AltimeterData* alt = altimeter_rx_get();
    const TrackerConfig* cfg = tracker_config_get();

    String packet = telemetry_build_aprs_packet(fix, alt, cfg->callsign);

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

void lora_compact_aprs_send() {
    const GpsFix* fix = gps_handler_get_fix();
    const AltimeterData* alt = altimeter_rx_get();
    const TrackerConfig* cfg = tracker_config_get();

    String packet = telemetry_build_compact_aprs_packet(fix, alt, cfg->callsign);

    Serial.print("[APRS-C TX] ");
    Serial.println(packet);

    lora_transmit(packet);
}

void lora_dense_send() {
    const GpsFix* fix = gps_handler_get_fix();
    const AltimeterData* alt = altimeter_rx_get();
    const TrackerConfig* cfg = tracker_config_get();

    String packet = telemetry_build_dense_packet(fix, alt, cfg->callsign);

    Serial.print("[DENSE TX] ");
    Serial.println(packet);

    lora_transmit(packet);
}

void lora_event_send(const FlightEvent* evt) {
    const TrackerConfig* cfg = tracker_config_get();

    String packet = telemetry_build_event_packet(evt, cfg->callsign);

    Serial.print("[EVENT TX] ");
    Serial.println(packet);

    lora_transmit(packet);
}

void lora_curve_send() {
    const AltimeterData* alt = altimeter_rx_get();
    const TrackerConfig* cfg = tracker_config_get();

    String packet = telemetry_build_curve_packet(alt, cfg->callsign);

    Serial.print("[CURVE TX] ");
    Serial.println(packet);

    lora_transmit(packet);
}

// Estimated packet sizes (payload bytes including OE header).
// Used for airtime calculations. These are approximate — the actual
// size varies slightly with field values, but the estimates are
// conservative (slightly over) so timing is safe.
uint16_t lora_packet_size_estimate(uint8_t mode) {
    // 3-byte OE header is always added
    switch (mode) {
        case MODE_APRS:   return 3 + 77;   // APRS position + full comment
        case MODE_FULL:   return 3 + 163;  // Dense telemetry with GPS
        case MODE_HYBRID: return 3 + 163;  // Dense is the larger packet
        case MODE_EVENT:  return 3 + 60;   // Compact APRS (event packets handled separately)
        case MODE_CURVE:  return 3 + 65;   // Curve packet (minimal)
        default:          return 3 + 80;
    }
}
