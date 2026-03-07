#include "lora_airtime.h"
#include <math.h>

// LoRa airtime calculation based on Semtech SX1276 datasheet Section 4.1.1.
//
// Symbol time: Tsym = 2^SF / BW
// Preamble time: Tpreamble = (n_preamble + 4.25) * Tsym
// Payload symbols: max(ceil((8*PL - 4*SF + 28 + 16 - 20*H) / (4*(SF-2*DE))) * (CR+4), 0) + 8
//   where H=0 (explicit header), DE=1 if SF>=11 (low data rate optimization),
//   CRC=1 (always on), CR = coding_rate - 4 (1 for 4/5, 2 for 4/6, etc.)
// Total: Tpreamble + payload_symbols * Tsym

uint32_t lora_airtime_ms(uint16_t payload_bytes, uint8_t sf, float bw_khz,
                          uint8_t cr, uint8_t preamble_len) {
    // Symbol time in milliseconds
    float tsym_ms = (float)(1UL << sf) / (bw_khz * 1000.0f) * 1000.0f;

    // Preamble time
    float t_preamble = (preamble_len + 4.25f) * tsym_ms;

    // Low data rate optimization: enabled for SF >= 11 with BW 125 kHz
    uint8_t de = (sf >= 11 && bw_khz <= 125.0f) ? 1 : 0;

    // Header mode: 0 = explicit (always for LoRa APRS)
    uint8_t h = 0;

    // CRC: always enabled
    int16_t crc_bits = 16;

    // Coding rate offset: cr=5 means 4/5, so cr_offset = cr - 4
    uint8_t cr_offset = (cr > 4) ? (cr - 4) : 1;

    // Payload symbol count (from Semtech formula)
    float numerator = (float)(8 * payload_bytes - 4 * sf + 28 + crc_bits - 20 * h);
    float denominator = (float)(4 * (sf - 2 * de));

    int32_t payload_symbols;
    if (denominator <= 0) {
        // Safety: shouldn't happen with valid SF
        payload_symbols = 8;
    } else {
        float ratio = numerator / denominator;
        int32_t ceil_val = (int32_t)ceilf(ratio);
        if (ceil_val < 0) ceil_val = 0;
        payload_symbols = 8 + ceil_val * (cr_offset + 4);
    }

    // Total time
    float total_ms = t_preamble + payload_symbols * tsym_ms;

    return (uint32_t)(total_ms + 0.5f);  // round to nearest ms
}

uint32_t lora_min_interval_ms(uint16_t payload_bytes, uint8_t sf, float bw_khz,
                               uint8_t cr, uint8_t pct, uint16_t floor_ms) {
    uint32_t airtime = lora_airtime_ms(payload_bytes, sf, bw_khz, cr);

    // Dead air = max(airtime * pct%, floor_ms)
    // This ensures proportional padding at high SFs while keeping a
    // sane minimum at low SFs where airtime is short.
    uint32_t proportional = (airtime * pct) / 100;
    uint32_t dead_air = (proportional > floor_ms) ? proportional : floor_ms;

    return airtime + dead_air;
}
