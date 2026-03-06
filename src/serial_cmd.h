#pragma once

// CDC Serial Command Protocol
//
// The ground station communicates with the tracker over USB CDC serial
// using a simple text command protocol. Commands are newline-terminated.
//
// Commands:
//   PING                     → "PONG Feather_LoRa_Tracker\n"
//   GET config               → JSON dump of current TrackerConfig
//   GET status               → JSON with GPS, altimeter, uptime info
//   SET <key> <value>        → Update a config field in RAM
//   SAVE                     → Write current config to flash (tracker.json)
//   RELOAD                   → Re-read config from flash
//
// SET keys (match tracker.json):
//   callsign, telemetry_mode, lora_freq_mhz, lora_bw_khz, lora_sf,
//   lora_cr, lora_sync_word, lora_power_dbm, aprs_rate_pad_ms,
//   aprs_rate_ascent_ms, aprs_rate_descent_ms, aprs_rate_landed_ms,
//   dense_min_interval_ms
//
// Response format:
//   OK <detail>\n          — command succeeded
//   ERR <detail>\n         — command failed
//   {json}\n               — for GET commands

void serial_cmd_init();
void serial_cmd_update();
