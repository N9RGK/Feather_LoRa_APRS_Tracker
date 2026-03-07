#pragma once

#include <stdint.h>

// Calculate the on-air time for a LoRa packet in milliseconds.
// Uses the standard LoRa modem timing formula from SX1276 datasheet.
//
// Parameters:
//   payload_bytes — total packet payload size (including OE header)
//   sf            — spreading factor (6-12)
//   bw_khz        — bandwidth in kHz (125, 250, 500)
//   cr            — coding rate denominator (5-8, i.e. 4/5 to 4/8)
//   preamble_len  — preamble symbol count (default 8 for LoRa APRS)
//
// Returns airtime in milliseconds.
uint32_t lora_airtime_ms(uint16_t payload_bytes, uint8_t sf, float bw_khz,
                          uint8_t cr, uint8_t preamble_len = 8);

// Calculate minimum safe interval between transmissions for a given
// packet size. Adds dead-air padding proportional to the airtime:
//
//   dead_air = max(airtime * pct / 100, floor_ms)
//   interval = airtime + dead_air
//
// The proportional padding ensures adequate dead air at all spreading
// factors. At SF12 a 3-second packet gets 600ms dead air (at 20%),
// not a fixed 200ms that would barely cover radio turnaround.
//
// Parameters:
//   pct      — dead-air as a percentage of airtime (default 20)
//   floor_ms — minimum dead-air in ms regardless of airtime (default 150)
uint32_t lora_min_interval_ms(uint16_t payload_bytes, uint8_t sf, float bw_khz,
                               uint8_t cr, uint8_t pct = 20,
                               uint16_t floor_ms = 150);
