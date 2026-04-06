#include "serial_cmd.h"
#include "tracker_config.h"
#include "lora_aprs.h"
#include "gps_handler.h"
#include "altimeter_rx.h"
#include "telemetry.h"
#include "test_data.h"
#include "lora_airtime.h"
#include "event_detector.h"
#include "airtime.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>

// SEND RAW can carry up to ~512 hex chars (256 bytes payload)
#define CMD_BUF_SIZE 600
static char cmd_buf[CMD_BUF_SIZE];
static uint16_t cmd_idx = 0;

// Transmission stats (tracked across SEND commands)
static uint32_t tx_count = 0;
static uint32_t last_tx_ms = 0;

// Forward declarations
static void process_command(const char* cmd);
static void cmd_ping();
static void cmd_get_config();
static void cmd_get_status();
static void cmd_get_radio();
static void cmd_set(const char* args);
static void cmd_send(const char* args);
static void cmd_apply();
static void cmd_save();
static void cmd_reload();

static const char* mode_str(uint8_t mode) {
    switch (mode) {
        case MODE_APRS:   return "aprs";
        case MODE_FULL:   return "full";
        case MODE_HYBRID: return "hybrid";
        case MODE_EVENT:  return "event";
        case MODE_CURVE:  return "curve";
        case MODE_TEST:   return "test";
        default:          return "hybrid";
    }
}

static uint8_t mode_from_str(const char* s) {
    if (strcmp(s, "aprs") == 0) return MODE_APRS;
    if (strcmp(s, "full") == 0) return MODE_FULL;
    if (strcmp(s, "hybrid") == 0) return MODE_HYBRID;
    if (strcmp(s, "event") == 0) return MODE_EVENT;
    if (strcmp(s, "curve") == 0) return MODE_CURVE;
    if (strcmp(s, "test") == 0) return MODE_TEST;
    return 0xFF;  // invalid
}

void serial_cmd_init() {
    cmd_idx = 0;
}

void serial_cmd_update() {
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmd_idx > 0) {
                cmd_buf[cmd_idx] = '\0';
                process_command(cmd_buf);
                cmd_idx = 0;
            }
        } else {
            if (cmd_idx < CMD_BUF_SIZE - 1) {
                cmd_buf[cmd_idx++] = c;
            } else {
                // Overflow — discard
                cmd_idx = 0;
                Serial.println("ERR command too long");
            }
        }
    }
}

static void process_command(const char* cmd) {
    // Skip leading whitespace
    while (*cmd == ' ') cmd++;

    if (strncmp(cmd, "PING", 4) == 0) {
        cmd_ping();
    } else if (strcmp(cmd, "GET config") == 0) {
        cmd_get_config();
    } else if (strcmp(cmd, "GET status") == 0) {
        cmd_get_status();
    } else if (strcmp(cmd, "GET radio") == 0) {
        cmd_get_radio();
    } else if (strncmp(cmd, "SET ", 4) == 0) {
        cmd_set(cmd + 4);
    } else if (strncmp(cmd, "SEND ", 5) == 0) {
        cmd_send(cmd + 5);
    } else if (strcmp(cmd, "APPLY") == 0) {
        cmd_apply();
    } else if (strcmp(cmd, "SAVE") == 0) {
        cmd_save();
    } else if (strcmp(cmd, "RELOAD") == 0) {
        cmd_reload();
    } else {
        Serial.print("ERR unknown command: ");
        Serial.println(cmd);
    }
}

static void cmd_ping() {
    Serial.println("PONG Feather_LoRa_Tracker");
}

static void cmd_get_config() {
    const TrackerConfig* cfg = tracker_config_get();

    StaticJsonDocument<640> doc;
    doc["callsign"]             = cfg->callsign;
    doc["tracker_id"]           = cfg->tracker_id;
    doc["telemetry_mode"]       = mode_str(cfg->telemetry_mode);
    doc["lora_freq_mhz"]        = cfg->lora_freq_mhz;
    doc["lora_bw_khz"]          = cfg->lora_bw_khz;
    doc["lora_sf"]              = cfg->lora_sf;
    doc["lora_cr"]              = cfg->lora_cr;
    doc["lora_sync_word"]       = cfg->lora_sync_word;
    doc["lora_power_dbm"]       = cfg->lora_power_dbm;
    doc["aprs_rate_pad_ms"]     = cfg->aprs_rate_pad_ms;
    doc["aprs_rate_ascent_ms"]  = cfg->aprs_rate_ascent_ms;
    doc["aprs_rate_descent_ms"] = cfg->aprs_rate_descent_ms;
    doc["aprs_rate_landed_ms"]  = cfg->aprs_rate_landed_ms;
    doc["dense_min_interval_ms"] = cfg->dense_min_interval_ms;
    doc["event_aprs_rate_pad_ms"]     = cfg->event_aprs_rate_pad_ms;
    doc["event_aprs_rate_ascent_ms"]  = cfg->event_aprs_rate_ascent_ms;
    doc["event_aprs_rate_descent_ms"] = cfg->event_aprs_rate_descent_ms;
    doc["event_aprs_rate_landed_ms"]  = cfg->event_aprs_rate_landed_ms;
    doc["event_repeat_count"]         = cfg->event_repeat_count;

    serializeJson(doc, Serial);
    Serial.println();
}

static void cmd_get_status() {
    const GpsFix* fix = gps_handler_get_fix();
    const AltimeterData* alt = altimeter_rx_get();
    const TrackerConfig* cfg = tracker_config_get();

    StaticJsonDocument<384> doc;
    doc["device"]     = "Feather_LoRa_Tracker";
    doc["callsign"]   = cfg->callsign;
    doc["tracker_id"] = cfg->tracker_id;
    doc["mode"]       = mode_str(cfg->telemetry_mode);
    doc["uptime_ms"]  = millis();

    JsonObject gps = doc.createNestedObject("gps");
    gps["valid"]      = fix->valid;
    if (fix->valid) {
        gps["lat"]        = fix->lat;
        gps["lng"]        = fix->lng;
        gps["satellites"] = fix->satellites;
        gps["speed_kmh"]  = fix->speed_kmh;
    }

    JsonObject altObj = doc.createNestedObject("altimeter");
    altObj["valid"]    = alt->valid;
    if (alt->valid) {
        altObj["alt_cm"]    = alt->alt_cm;
        altObj["vel_cms"]   = alt->vel_cms;
        altObj["state"]     = alt->state;
        altObj["thrust"]    = alt->thrust;
        altObj["seq"]       = alt->seq;
        altObj["age_ms"]    = (long)(millis() - alt->last_rx_ms);
    }

    serializeJson(doc, Serial);
    Serial.println();
}

static void cmd_set(const char* args) {
    // Parse "key value" from args
    // Find the first space separating key from value
    const char* space = strchr(args, ' ');
    if (!space) {
        Serial.println("ERR SET requires <key> <value>");
        return;
    }

    // Extract key
    char key[32];
    int keyLen = space - args;
    if (keyLen >= (int)sizeof(key)) keyLen = sizeof(key) - 1;
    memcpy(key, args, keyLen);
    key[keyLen] = '\0';

    // Extract value (skip space)
    const char* val = space + 1;
    while (*val == ' ') val++;  // skip extra spaces

    TrackerConfig* cfg = tracker_config_get();

    if (strcmp(key, "callsign") == 0) {
        strncpy(cfg->callsign, val, sizeof(cfg->callsign) - 1);
        cfg->callsign[sizeof(cfg->callsign) - 1] = '\0';
    } else if (strcmp(key, "tracker_id") == 0) {
        strncpy(cfg->tracker_id, val, sizeof(cfg->tracker_id) - 1);
        cfg->tracker_id[sizeof(cfg->tracker_id) - 1] = '\0';
    } else if (strcmp(key, "telemetry_mode") == 0) {
        uint8_t m = mode_from_str(val);
        if (m == 0xFF) {
            Serial.println("ERR invalid mode (use aprs/full/hybrid/event/curve/test)");
            return;
        }
        cfg->telemetry_mode = m;
    } else if (strcmp(key, "lora_freq_mhz") == 0) {
        cfg->lora_freq_mhz = atof(val);
    } else if (strcmp(key, "lora_bw_khz") == 0) {
        cfg->lora_bw_khz = atof(val);
    } else if (strcmp(key, "lora_sf") == 0) {
        cfg->lora_sf = (uint8_t)atoi(val);
    } else if (strcmp(key, "lora_cr") == 0) {
        cfg->lora_cr = (uint8_t)atoi(val);
    } else if (strcmp(key, "lora_sync_word") == 0) {
        cfg->lora_sync_word = (uint8_t)strtol(val, NULL, 0);  // supports 0x prefix
    } else if (strcmp(key, "lora_power_dbm") == 0) {
        cfg->lora_power_dbm = (int8_t)atoi(val);
    } else if (strcmp(key, "aprs_rate_pad_ms") == 0) {
        cfg->aprs_rate_pad_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "aprs_rate_ascent_ms") == 0) {
        cfg->aprs_rate_ascent_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "aprs_rate_descent_ms") == 0) {
        cfg->aprs_rate_descent_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "aprs_rate_landed_ms") == 0) {
        cfg->aprs_rate_landed_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "dense_min_interval_ms") == 0) {
        cfg->dense_min_interval_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "event_aprs_rate_pad_ms") == 0) {
        cfg->event_aprs_rate_pad_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "event_aprs_rate_ascent_ms") == 0) {
        cfg->event_aprs_rate_ascent_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "event_aprs_rate_descent_ms") == 0) {
        cfg->event_aprs_rate_descent_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "event_aprs_rate_landed_ms") == 0) {
        cfg->event_aprs_rate_landed_ms = (uint32_t)atol(val);
    } else if (strcmp(key, "event_repeat_count") == 0) {
        cfg->event_repeat_count = (uint8_t)atoi(val);
    } else {
        Serial.print("ERR unknown key: ");
        Serial.println(key);
        return;
    }

    // If LoRa radio settings changed, reconfigure the radio
    if (strncmp(key, "lora_", 5) == 0) {
        lora_aprs_reconfigure(cfg);
    }

    Serial.print("OK ");
    Serial.print(key);
    Serial.print("=");
    Serial.println(val);
}

static void cmd_save() {
    if (tracker_config_save(tracker_config_get())) {
        Serial.println("OK config saved to flash");
    } else {
        Serial.println("ERR failed to save config");
    }
}

static void cmd_reload() {
    TrackerConfig* cfg = tracker_config_get();
    if (tracker_config_load(cfg)) {
        // Re-apply LoRa settings
        lora_aprs_reconfigure(cfg);
        Serial.println("OK config reloaded from flash");
    } else {
        Serial.println("ERR failed to reload config (using current)");
    }
}

// --- Helper: parse a hex string into a byte buffer ---
// Returns number of bytes parsed, or -1 on error.
static int parse_hex(const char* hex, uint8_t* buf, size_t buf_size) {
    size_t hex_len = strlen(hex);
    if (hex_len % 2 != 0) return -1;
    size_t n_bytes = hex_len / 2;
    if (n_bytes > buf_size) return -1;

    for (size_t i = 0; i < n_bytes; i++) {
        char hi = hex[i * 2];
        char lo = hex[i * 2 + 1];
        uint8_t val = 0;

        if (hi >= '0' && hi <= '9')      val = (hi - '0') << 4;
        else if (hi >= 'A' && hi <= 'F') val = (hi - 'A' + 10) << 4;
        else if (hi >= 'a' && hi <= 'f') val = (hi - 'a' + 10) << 4;
        else return -1;

        if (lo >= '0' && lo <= '9')      val |= (lo - '0');
        else if (lo >= 'A' && lo <= 'F') val |= (lo - 'A' + 10);
        else if (lo >= 'a' && lo <= 'f') val |= (lo - 'a' + 10);
        else return -1;

        buf[i] = val;
    }
    return (int)n_bytes;
}

// --- SEND command ---
static void cmd_send(const char* args) {
    const TrackerConfig* cfg = tracker_config_get();
    if (cfg->telemetry_mode != MODE_TEST) {
        Serial.println("ERR SEND not in test mode");
        return;
    }

    // Skip leading whitespace
    while (*args == ' ') args++;

    if (strncmp(args, "APRS", 4) == 0) {
        const GpsFix* fix = test_data_get_gps();
        const AltimeterData* alt = test_data_get_alt();
        String packet = telemetry_build_aprs_packet(fix, alt, cfg->callsign);
        size_t total = 3 + packet.length();
        lora_transmit_packet(packet);
        tx_count++;
        last_tx_ms = millis();
        uint32_t airtime = lora_airtime_ms(total, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
        Serial.print("OK SENT APRS len=");
        Serial.print(total);
        Serial.print(" airtime_ms=");
        Serial.print(airtime);
        Serial.print(" freq=");
        Serial.println(cfg->lora_freq_mhz);

    } else if (strncmp(args, "DENSE", 5) == 0) {
        const GpsFix* fix = test_data_get_gps();
        const AltimeterData* alt = test_data_get_alt();
        String packet = telemetry_build_dense_packet(fix, alt, cfg->callsign);
        size_t total = 3 + packet.length();
        lora_transmit_packet(packet);
        tx_count++;
        last_tx_ms = millis();
        uint32_t airtime = lora_airtime_ms(total, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
        Serial.print("OK SENT DENSE len=");
        Serial.print(total);
        Serial.print(" airtime_ms=");
        Serial.print(airtime);
        Serial.print(" freq=");
        Serial.println(cfg->lora_freq_mhz);

    } else if (strncmp(args, "EVENT", 5) == 0) {
        // Parse optional event code character after "EVENT"
        const char* rest = args + 5;
        while (*rest == ' ') rest++;
        char code = (*rest != '\0') ? *rest : 'A';  // default to apogee

        const AltimeterData* alt = test_data_get_alt();
        FlightEvent evt = {};
        evt.code = (uint8_t)code;
        evt.flight_time_s = alt->flight_time_ms / 1000;
        evt.alt_m = alt->alt_cm / 100;
        evt.vel_mps = alt->vel_cms / 100;
        evt.has_gps = false;
        evt.error_flags = 0;

        // Include GPS for LANDED and REPORT events
        if (code == 'L' || code == 'R') {
            const GpsFix* fix = test_data_get_gps();
            if (fix->valid) {
                evt.has_gps = true;
                evt.lat = fix->lat;
                evt.lng = fix->lng;
            }
        }

        String packet = telemetry_build_event_packet(&evt, cfg->callsign);
        size_t total = 3 + packet.length();
        lora_transmit_packet(packet);
        tx_count++;
        last_tx_ms = millis();
        uint32_t airtime = lora_airtime_ms(total, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
        Serial.print("OK SENT EVENT len=");
        Serial.print(total);
        Serial.print(" airtime_ms=");
        Serial.print(airtime);
        Serial.print(" freq=");
        Serial.println(cfg->lora_freq_mhz);

    } else if (strncmp(args, "CURVE", 5) == 0) {
        const AltimeterData* alt = test_data_get_alt();
        String packet = telemetry_build_curve_packet(alt, cfg->callsign);
        size_t total = 3 + packet.length();
        lora_transmit_packet(packet);
        tx_count++;
        last_tx_ms = millis();
        uint32_t airtime = lora_airtime_ms(total, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
        Serial.print("OK SENT CURVE len=");
        Serial.print(total);
        Serial.print(" airtime_ms=");
        Serial.print(airtime);
        Serial.print(" freq=");
        Serial.println(cfg->lora_freq_mhz);

    } else if (strncmp(args, "RAW ", 4) == 0) {
        const char* hex = args + 4;
        while (*hex == ' ') hex++;

        uint8_t raw_buf[256];
        int n = parse_hex(hex, raw_buf, sizeof(raw_buf));
        if (n <= 0) {
            Serial.println("ERR invalid hex string");
            return;
        }
        lora_transmit_raw(raw_buf, (size_t)n);
        tx_count++;
        last_tx_ms = millis();
        Serial.print("OK SENT RAW len=");
        Serial.println(n);

    } else if (strncmp(args, "CARRIER", 7) == 0) {
        const char* rest = args + 7;
        while (*rest == ' ') rest++;
        uint32_t duration = (*rest != '\0') ? (uint32_t)atol(rest) : 5000;
        if (duration == 0) duration = 5000;

        lora_transmit_carrier(duration);
        tx_count++;
        last_tx_ms = millis();
        Serial.print("OK CARRIER duration_ms=");
        Serial.println(duration);

    } else {
        Serial.println("ERR SEND type must be APRS|DENSE|EVENT|CURVE|RAW|CARRIER");
    }
}

// --- GET radio command ---
static void cmd_get_radio() {
    const TrackerConfig* cfg = tracker_config_get();

    StaticJsonDocument<384> doc;
    doc["freq_mhz"]    = cfg->lora_freq_mhz;
    doc["sf"]           = cfg->lora_sf;
    doc["bw_khz"]       = cfg->lora_bw_khz;
    doc["cr"]           = cfg->lora_cr;
    doc["sync_word"]    = cfg->lora_sync_word;
    doc["power_dbm"]    = cfg->lora_power_dbm;
    doc["tx_count"]     = tx_count;
    doc["last_tx_ms"]   = last_tx_ms;

    // Airtime estimates for each packet type
    doc["aprs_airtime_ms"]  = lora_airtime_ms(3 + 77,  cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
    doc["dense_airtime_ms"] = lora_airtime_ms(3 + 163, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
    doc["event_airtime_ms"] = lora_airtime_ms(3 + 55,  cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);
    doc["curve_airtime_ms"] = lora_airtime_ms(3 + 65,  cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr);

    serializeJson(doc, Serial);
    Serial.println();
}

// --- APPLY command ---
static void cmd_apply() {
    const TrackerConfig* cfg = tracker_config_get();
    lora_aprs_reconfigure(cfg);
    recompute_airtime_limits();
    Serial.println("OK radio reconfigured and airtime limits recomputed");
}
