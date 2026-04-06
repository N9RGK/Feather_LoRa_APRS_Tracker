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
//   GET radio                → JSON with radio params, tx stats, airtime estimates
//   SET <key> <value>        → Update a config field in RAM
//   SEND <type> [args]       → Transmit a test packet (MODE_TEST only)
//   APPLY                    → Reconfigure radio + recompute airtime limits
//   SAVE                     → Write current config to flash (tracker.json)
//   RELOAD                   → Re-read config from flash
//
// SEND types (requires telemetry_mode == test):
//   SEND APRS               → Build and transmit an APRS position packet
//   SEND DENSE              → Build and transmit a dense telemetry packet
//   SEND EVENT [code]       → Build and transmit an event packet (default 'A')
//   SEND CURVE              → Build and transmit a curve packet
//   SEND RAW <hex>          → Transmit raw bytes (no OE header)
//   SEND CARRIER [ms]       → Transmit unmodulated carrier (default 5000ms)
//
// SET keys (match tracker.json):
//   callsign, telemetry_mode, lora_freq_mhz, lora_bw_khz, lora_sf,
//   lora_cr, lora_sync_word, lora_power_dbm, aprs_rate_pad_ms,
//   aprs_rate_ascent_ms, aprs_rate_descent_ms, aprs_rate_landed_ms,
//   dense_min_interval_ms, event_aprs_rate_pad_ms,
//   event_aprs_rate_ascent_ms, event_aprs_rate_descent_ms,
//   event_aprs_rate_landed_ms, event_repeat_count
//
// Response format:
//   OK <detail>\n          — command succeeded
//   ERR <detail>\n         — command failed
//   {json}\n               — for GET commands

void serial_cmd_init();
void serial_cmd_update();
