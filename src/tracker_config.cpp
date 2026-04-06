#include "tracker_config.h"
#include "config.h"
#include <Arduino.h>
#include <FlashStorage.h>

FlashStorage(flash_store, TrackerConfig);

static TrackerConfig active_config;

static void set_defaults(TrackerConfig* cfg) {
    strncpy(cfg->callsign, CALLSIGN, sizeof(cfg->callsign) - 1);
    cfg->callsign[sizeof(cfg->callsign) - 1] = '\0';
    strncpy(cfg->tracker_id, TRACKER_ID, sizeof(cfg->tracker_id) - 1);
    cfg->tracker_id[sizeof(cfg->tracker_id) - 1] = '\0';
    cfg->telemetry_mode      = TELEMETRY_MODE;
    cfg->lora_freq_mhz       = LORA_FREQ_MHZ;
    cfg->lora_bw_khz         = LORA_BW_KHZ;
    cfg->lora_sf              = LORA_SF;
    cfg->lora_cr              = LORA_CR;
    cfg->lora_sync_word       = LORA_SYNC_WORD;
    cfg->lora_power_dbm       = LORA_POWER_DBM;
    cfg->aprs_rate_pad_ms     = APRS_RATE_PAD_MS;
    cfg->aprs_rate_ascent_ms  = APRS_RATE_ASCENT_MS;
    cfg->aprs_rate_descent_ms = APRS_RATE_DESCENT_MS;
    cfg->aprs_rate_landed_ms  = APRS_RATE_LANDED_MS;
    cfg->dense_min_interval_ms = DENSE_MIN_INTERVAL_MS;
    // MODE_EVENT defaults
    cfg->event_aprs_rate_pad_ms     = EVENT_APRS_RATE_PAD_MS;
    cfg->event_aprs_rate_ascent_ms  = EVENT_APRS_RATE_ASCENT_MS;
    cfg->event_aprs_rate_descent_ms = EVENT_APRS_RATE_DESCENT_MS;
    cfg->event_aprs_rate_landed_ms  = EVENT_APRS_RATE_LANDED_MS;
    cfg->event_repeat_count         = EVENT_REPEAT_COUNT;
}

static uint8_t mode_from_string(const char* s) {
    if (strcmp(s, "aprs") == 0)   return MODE_APRS;
    if (strcmp(s, "full") == 0)   return MODE_FULL;
    if (strcmp(s, "hybrid") == 0) return MODE_HYBRID;
    if (strcmp(s, "event") == 0)  return MODE_EVENT;
    if (strcmp(s, "curve") == 0)  return MODE_CURVE;
    return MODE_HYBRID;  // default
}

static const char* mode_to_string(uint8_t mode) {
    switch (mode) {
        case MODE_APRS:   return "aprs";
        case MODE_FULL:   return "full";
        case MODE_HYBRID: return "hybrid";
        case MODE_EVENT:  return "event";
        case MODE_CURVE:  return "curve";
        default:          return "hybrid";
    }
}

// Check if flash data looks valid (not erased 0xFF)
static bool flash_data_valid(const TrackerConfig* cfg) {
    return cfg->callsign[0] >= 0x20 && cfg->callsign[0] <= 0x7E;
}

bool tracker_config_load(TrackerConfig* cfg) {
    TrackerConfig stored;
    flash_store.read(&stored);

    if (!flash_data_valid(&stored)) {
        Serial.println("  Flash uninitialized, using defaults");
        set_defaults(cfg);
        return false;
    }

    *cfg = stored;
    Serial.println("  Config loaded from flash");
    return true;
}

bool tracker_config_save(const TrackerConfig* cfg) {
    flash_store.write(*cfg);
    Serial.println("  Config saved to flash");
    return true;
}

TrackerConfig* tracker_config_get() {
    return &active_config;
}

void tracker_config_init() {
    Serial.print("FlashStorage init... ");
    Serial.println("ok");

    Serial.print("Loading config... ");
    tracker_config_load(&active_config);
}
