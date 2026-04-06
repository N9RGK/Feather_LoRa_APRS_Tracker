#pragma once

#include <Arduino.h>
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

// Transmit a packet string with the 3-byte OE header prepended.
// Used by SEND commands in test mode.
bool lora_transmit_packet(const String& packet);

// Transmit raw bytes directly (no OE header prepended).
// Caller controls the full payload.
bool lora_transmit_raw(const uint8_t* data, size_t len);

// Transmit an unmodulated carrier for the specified duration in ms.
// Uses RadioLib's transmitDirect(0) and standby() to bracket the carrier.
// Returns to RX-ready (standby) state after.
bool lora_transmit_carrier(uint32_t duration_ms);
