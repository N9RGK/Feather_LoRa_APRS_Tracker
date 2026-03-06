#pragma once

#include "tracker_config.h"

// Initialize radio from runtime config. Returns true on success.
bool lora_aprs_init(const TrackerConfig* cfg);

// Re-configure radio with new settings (e.g. after CDC SET command).
// Returns true on success.
bool lora_aprs_reconfigure(const TrackerConfig* cfg);

// Send an APRS position packet (CA2RXU-compatible, with 3-byte OE header)
void lora_aprs_send();

// Send a dense telemetry packet (our proprietary format, with 3-byte OE header)
void lora_dense_send();
