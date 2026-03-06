#include <Arduino.h>
#include "config.h"
#include "gps_handler.h"
#include "lora_aprs.h"
#include "altimeter_rx.h"
#include "tracker_config.h"
#include "serial_cmd.h"

// Get the phase-dependent transmit interval for the current flight state.
// Returns interval in ms. Falls back to pad rate if altimeter data invalid.
static uint32_t get_phase_interval() {
    const TrackerConfig* cfg = tracker_config_get();
    const AltimeterData* alt = altimeter_rx_get();
    if (!alt->valid) return cfg->aprs_rate_pad_ms;

    switch (alt->state) {
        case STATE_PAD_IDLE: return cfg->aprs_rate_pad_ms;
        case STATE_ASCENT:   return cfg->aprs_rate_ascent_ms;
        case STATE_DESCENT:  return cfg->aprs_rate_descent_ms;
        case STATE_LANDED:   return cfg->aprs_rate_landed_ms;
        default:             return cfg->aprs_rate_pad_ms;
    }
}

static const char* mode_name(uint8_t mode) {
    switch (mode) {
        case MODE_APRS:   return "APRS";
        case MODE_FULL:   return "Full";
        case MODE_HYBRID: return "Hybrid";
        default:          return "Unknown";
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // wait up to 3s for USB serial
    Serial.println();
    Serial.println("=== Feather LoRa APRS Tracker ===");

    // Load config from SPI flash (falls back to compiled defaults)
    tracker_config_init();
    const TrackerConfig* cfg = tracker_config_get();

    Serial.print("Callsign: ");
    Serial.println(cfg->callsign);
    Serial.print("Telemetry mode: ");
    Serial.println(mode_name(cfg->telemetry_mode));
    Serial.print("LoRa: SF");
    Serial.print(cfg->lora_sf);
    Serial.print(", ");
    Serial.print(cfg->lora_freq_mhz);
    Serial.println(" MHz");

    Serial.print("GPS init... ");
    gps_handler_init();
    Serial.println("ok");

    Serial.print("Altimeter SERCOM1 init (");
    Serial.print(ALTIMETER_BAUD);
    Serial.print(" baud)... ");
    altimeter_rx_init();
    Serial.println("ok");

    Serial.print("LoRa radio init... ");
    lora_aprs_init(cfg);

    // Initialize CDC serial command handler
    serial_cmd_init();
    Serial.println("CDC serial command handler ready");
}

// LED flash patterns to distinguish packet types
static void flash_aprs() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
}

static void flash_dense() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(30);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // Feed GPS and altimeter parsers
    gps_handler_update();
    altimeter_rx_update();

    // Process incoming CDC serial commands
    serial_cmd_update();

    // Heartbeat: brief blink every 2s to show the board is alive
    static uint32_t last_blink = 0;
    if (millis() - last_blink >= 2000) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        last_blink = millis();
    }

    const TrackerConfig* cfg = tracker_config_get();
    static uint32_t last_aprs_tx = 0;
    static uint32_t last_dense_tx = 0;
    uint32_t now = millis();
    uint32_t interval = get_phase_interval();

    switch (cfg->telemetry_mode) {

    // ---------------------------------------------------------------
    // MODE_APRS: Only APRS position packets at phase-dependent rate.
    // 1 Hz effective during ascent. CA2RXU-compatible.
    // ---------------------------------------------------------------
    case MODE_APRS:
        if (now - last_aprs_tx >= interval) {
            flash_aprs();
            lora_aprs_send();
            last_aprs_tx = now;

            Serial.print("  [APRS] every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        break;

    // ---------------------------------------------------------------
    // MODE_FULL: Only dense telemetry packets at phase-dependent rate.
    // 1 Hz effective during ascent. NOT CA2RXU-compatible.
    // GPS position is embedded in the dense packet payload.
    // ---------------------------------------------------------------
    case MODE_FULL:
        if (now - last_dense_tx >= interval) {
            flash_dense();
            lora_dense_send();
            last_dense_tx = now;

            Serial.print("  [FULL] every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        break;

    // ---------------------------------------------------------------
    // MODE_HYBRID: Alternating APRS + dense packets (2 Hz effective).
    // APRS at phase-dependent intervals, dense fills the gaps.
    // Best of both worlds: CA2RXU compatibility + full data.
    // ---------------------------------------------------------------
    case MODE_HYBRID:
    default:
        // Time for an APRS packet?
        if (now - last_aprs_tx >= interval) {
            flash_aprs();
            lora_aprs_send();
            last_aprs_tx = now;
            last_dense_tx = now;  // Reset dense timer to avoid double-send

            Serial.print("  [HYBRID] APRS every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        // Otherwise, is there room for a dense packet?
        else if (now - last_dense_tx >= cfg->dense_min_interval_ms) {
            // Only send dense if we have enough time before next APRS.
            // Leave at least 200ms margin for radio airtime + processing.
            uint32_t time_to_next_aprs = (last_aprs_tx + interval) - now;
            if (time_to_next_aprs > 200) {
                flash_dense();
                lora_dense_send();
                last_dense_tx = now;
            }
        }
        break;
    }
}
