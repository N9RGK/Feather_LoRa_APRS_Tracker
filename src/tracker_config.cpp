#include "tracker_config.h"
#include "config.h"
#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>

#if defined(ARDUINO_ARCH_SAMD)
  #include <SdFat.h>
  #if defined(EXTERNAL_FLASH_USE_QSPI)
    Adafruit_FlashTransport_QSPI flashTransport;
  #elif defined(EXTERNAL_FLASH_USE_SPI)
    Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                               EXTERNAL_FLASH_USE_SPI);
  #else
    Adafruit_FlashTransport_SPI flashTransport(SS, SPI);
  #endif
  Adafruit_SPIFlash flash(&flashTransport);
  FatVolume fatfs;
  #define USE_SPIFATFS 1
#endif

static TrackerConfig active_config;

static void set_defaults(TrackerConfig* cfg) {
    strncpy(cfg->callsign, CALLSIGN, sizeof(cfg->callsign) - 1);
    cfg->callsign[sizeof(cfg->callsign) - 1] = '\0';
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

bool tracker_config_load(TrackerConfig* cfg) {
    set_defaults(cfg);

#ifdef USE_SPIFATFS
    File32 file = fatfs.open("tracker.json", FILE_READ);
    if (!file) {
        Serial.println("  No tracker.json found, using defaults");
        return false;
    }

    StaticJsonDocument<768> doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();

    if (err) {
        Serial.print("  JSON parse error: ");
        Serial.println(err.c_str());
        return false;
    }

    // Apply values from JSON, keeping defaults for missing keys
    if (doc.containsKey("callsign")) {
        strncpy(cfg->callsign, doc["callsign"].as<const char*>(), sizeof(cfg->callsign) - 1);
        cfg->callsign[sizeof(cfg->callsign) - 1] = '\0';
    }
    if (doc.containsKey("telemetry_mode"))
        cfg->telemetry_mode = mode_from_string(doc["telemetry_mode"].as<const char*>());
    if (doc.containsKey("lora_freq_mhz"))
        cfg->lora_freq_mhz = doc["lora_freq_mhz"].as<float>();
    if (doc.containsKey("lora_bw_khz"))
        cfg->lora_bw_khz = doc["lora_bw_khz"].as<float>();
    if (doc.containsKey("lora_sf"))
        cfg->lora_sf = doc["lora_sf"].as<uint8_t>();
    if (doc.containsKey("lora_cr"))
        cfg->lora_cr = doc["lora_cr"].as<uint8_t>();
    if (doc.containsKey("lora_sync_word"))
        cfg->lora_sync_word = doc["lora_sync_word"].as<uint8_t>();
    if (doc.containsKey("lora_power_dbm"))
        cfg->lora_power_dbm = doc["lora_power_dbm"].as<int8_t>();
    if (doc.containsKey("aprs_rate_pad_ms"))
        cfg->aprs_rate_pad_ms = doc["aprs_rate_pad_ms"].as<uint32_t>();
    if (doc.containsKey("aprs_rate_ascent_ms"))
        cfg->aprs_rate_ascent_ms = doc["aprs_rate_ascent_ms"].as<uint32_t>();
    if (doc.containsKey("aprs_rate_descent_ms"))
        cfg->aprs_rate_descent_ms = doc["aprs_rate_descent_ms"].as<uint32_t>();
    if (doc.containsKey("aprs_rate_landed_ms"))
        cfg->aprs_rate_landed_ms = doc["aprs_rate_landed_ms"].as<uint32_t>();
    if (doc.containsKey("dense_min_interval_ms"))
        cfg->dense_min_interval_ms = doc["dense_min_interval_ms"].as<uint32_t>();
    // MODE_EVENT settings
    if (doc.containsKey("event_aprs_rate_pad_ms"))
        cfg->event_aprs_rate_pad_ms = doc["event_aprs_rate_pad_ms"].as<uint32_t>();
    if (doc.containsKey("event_aprs_rate_ascent_ms"))
        cfg->event_aprs_rate_ascent_ms = doc["event_aprs_rate_ascent_ms"].as<uint32_t>();
    if (doc.containsKey("event_aprs_rate_descent_ms"))
        cfg->event_aprs_rate_descent_ms = doc["event_aprs_rate_descent_ms"].as<uint32_t>();
    if (doc.containsKey("event_aprs_rate_landed_ms"))
        cfg->event_aprs_rate_landed_ms = doc["event_aprs_rate_landed_ms"].as<uint32_t>();
    if (doc.containsKey("event_repeat_count"))
        cfg->event_repeat_count = doc["event_repeat_count"].as<uint8_t>();

    Serial.println("  Config loaded from tracker.json");
    return true;
#else
    return false;
#endif
}

bool tracker_config_save(const TrackerConfig* cfg) {
#ifdef USE_SPIFATFS
    // Remove old file first (FatFs doesn't truncate on open)
    fatfs.remove("tracker.json");

    File32 file = fatfs.open("tracker.json", FILE_WRITE);
    if (!file) {
        Serial.println("  Failed to open tracker.json for writing");
        return false;
    }

    StaticJsonDocument<768> doc;
    doc["callsign"]             = cfg->callsign;
    doc["telemetry_mode"]       = mode_to_string(cfg->telemetry_mode);
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
    // MODE_EVENT settings
    doc["event_aprs_rate_pad_ms"]     = cfg->event_aprs_rate_pad_ms;
    doc["event_aprs_rate_ascent_ms"]  = cfg->event_aprs_rate_ascent_ms;
    doc["event_aprs_rate_descent_ms"] = cfg->event_aprs_rate_descent_ms;
    doc["event_aprs_rate_landed_ms"]  = cfg->event_aprs_rate_landed_ms;
    doc["event_repeat_count"]         = cfg->event_repeat_count;

    serializeJsonPretty(doc, file);
    file.close();

    Serial.println("  Config saved to tracker.json");
    return true;
#else
    return false;
#endif
}

TrackerConfig* tracker_config_get() {
    return &active_config;
}

void tracker_config_init() {
    Serial.print("SPI flash init... ");
#ifdef USE_SPIFATFS
    if (!flash.begin()) {
        Serial.println("FAILED");
        set_defaults(&active_config);
        return;
    }
    Serial.println("ok");

    Serial.print("Mounting filesystem... ");
    if (!fatfs.begin(&flash)) {
        Serial.println("FAILED (not formatted?)");
        set_defaults(&active_config);
        return;
    }
    Serial.println("ok");
#endif

    Serial.print("Loading config... ");
    tracker_config_load(&active_config);
}
