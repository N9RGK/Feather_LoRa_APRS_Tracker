#pragma once

#include "tracker_config.h"
#include "event_detector.h"

// Initialize radio from runtime config. Returns true on success.
bool lora_aprs_init(const TrackerConfig* cfg);

// Re-configure radio with new settings (e.g. after CDC SET command).
// Returns true on success.
bool lora_aprs_reconfigure(const TrackerConfig* cfg);

// Send an APRS position packet (CA2RXU-compatible, with 3-byte OE header)
void lora_aprs_send();

// Send a compact APRS position packet (shorter comment for MODE_EVENT)
void lora_compact_aprs_send();

// Send a dense telemetry packet (our proprietary format, with 3-byte OE header)
void lora_dense_send();

// Send an event packet (with 3-byte OE header)
void lora_event_send(const FlightEvent* evt);

// Send a curve packet (with 3-byte OE header). Minimal data for max rate.
void lora_curve_send();

// Get the estimated packet size (bytes) for a given packet type.
// Used by the airtime calculator to compute dynamic intervals.
// Includes the 3-byte OE header.
uint16_t lora_packet_size_estimate(uint8_t mode);
