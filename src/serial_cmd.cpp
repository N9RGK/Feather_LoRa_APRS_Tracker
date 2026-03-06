#include "serial_cmd.h"
#include "tracker_config.h"
#include "lora_aprs.h"
#include "gps_handler.h"
#include "altimeter_rx.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>

#define CMD_BUF_SIZE 128
static char cmd_buf[CMD_BUF_SIZE];
static uint8_t cmd_idx = 0;

// Forward declarations
static void process_command(const char* cmd);
static void cmd_ping();
static void cmd_get_config();
static void cmd_get_status();
static void cmd_set(const char* args);
static void cmd_save();
static void cmd_reload();

static const char* mode_str(uint8_t mode) {
    switch (mode) {
        case MODE_APRS:   return "aprs";
        case MODE_FULL:   return "full";
        case MODE_HYBRID: return "hybrid";
        default:          return "hybrid";
    }
}

static uint8_t mode_from_str(const char* s) {
    if (strcmp(s, "aprs") == 0) return MODE_APRS;
    if (strcmp(s, "full") == 0) return MODE_FULL;
    if (strcmp(s, "hybrid") == 0) return MODE_HYBRID;
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
    } else if (strncmp(cmd, "SET ", 4) == 0) {
        cmd_set(cmd + 4);
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

    StaticJsonDocument<512> doc;
    doc["callsign"]             = cfg->callsign;
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
    } else if (strcmp(key, "telemetry_mode") == 0) {
        uint8_t m = mode_from_str(val);
        if (m == 0xFF) {
            Serial.println("ERR invalid mode (use aprs/full/hybrid)");
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
