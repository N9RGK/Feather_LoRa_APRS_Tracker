#pragma once

#include <stdint.h>

// Runtime configuration loaded from /tracker.json on SPIFFS.
// Fields mirror the JSON keys. Defaults come from config.h compile-time values.
typedef struct {
    char     callsign[12];        // APRS callsign (e.g. "N9RGK-1")
    uint8_t  telemetry_mode;      // 0=APRS, 1=Full, 2=Hybrid
    float    lora_freq_mhz;
    float    lora_bw_khz;
    uint8_t  lora_sf;
    uint8_t  lora_cr;
    uint8_t  lora_sync_word;
    int8_t   lora_power_dbm;
    uint32_t aprs_rate_pad_ms;
    uint32_t aprs_rate_ascent_ms;
    uint32_t aprs_rate_descent_ms;
    uint32_t aprs_rate_landed_ms;
    uint32_t dense_min_interval_ms;
} TrackerConfig;

// Load config from /tracker.json on SPIFFS.
// Returns true if file was read successfully; false uses compiled defaults.
bool tracker_config_load(TrackerConfig* cfg);

// Save current config to /tracker.json on SPIFFS.
// Returns true on success.
bool tracker_config_save(const TrackerConfig* cfg);

// Get pointer to the active config (singleton, loaded once at boot).
TrackerConfig* tracker_config_get();

// Initialize: mount SPIFFS and load config. Call once in setup().
void tracker_config_init();
