#pragma once

// --- APRS Configuration ---
#define CALLSIGN        "N9RGK-1"

// --- Telemetry Mode ---
// Configurable via HTTP interface when USB-connected to ground station.
// MODE_APRS:   APRS position packets only (1 Hz, CA2RXU-compatible)
// MODE_FULL:   Dense telemetry packets only (1 Hz, GPS included in payload)
// MODE_HYBRID: Alternating APRS + dense packets (2 Hz effective rate)
#define MODE_APRS       0
#define MODE_FULL       1
#define MODE_HYBRID     2

// Default telemetry mode — overridden by tracker.json config
#define TELEMETRY_MODE  MODE_HYBRID

// --- LoRa Radio Settings (CA2RXU-compatible) ---
#define LORA_FREQ_MHZ       443.775  // 70cm amateur band
#define LORA_BW_KHZ         125.0    // Bandwidth: 125 kHz
#define LORA_SF             8        // Spreading Factor 8
#define LORA_CR             5        // Coding Rate 4/5
#define LORA_SYNC_WORD      0x12     // LoRa APRS sync word
#define LORA_POWER_DBM      17       // TX power in dBm (max 20 for RFM96)

// --- Phase-Dependent APRS Transmit Intervals (ms) ---
// These are the intervals between APRS position packets.
// Dense telemetry packets fill the gaps between APRS packets.
#define APRS_RATE_PAD_MS      10000   // 10s on pad
#define APRS_RATE_ASCENT_MS    1000   // 1s during ascent
#define APRS_RATE_DESCENT_MS   2000   // 2s during descent
#define APRS_RATE_LANDED_MS    5000   // 5s once landed

// Dense telemetry is sent between APRS packets at this minimum interval.
// Actual rate depends on remaining time budget after APRS + airtime.
#define DENSE_MIN_INTERVAL_MS  500    // Don't send dense packets faster than 2Hz

// --- Pin Assignments (Feather M0) ---
#define LORA_CS_PIN     8
#define LORA_RST_PIN    4
#define LORA_DIO0_PIN   3
#define LORA_DIO1_PIN   6

// --- Altimeter Serial (SERCOM1 hardware UART) ---
// RX on pin 11 (PA16, SERCOM1 PAD 0) — physical wire from altimeter board
// TX on pin 10 (PA18, SERCOM1 PAD 2) — unused, no wire needed
#define ALT_RX_PIN        11
#define ALT_TX_PIN        10
#define ALTIMETER_BAUD    115200

// --- Flight State Constants (from $PYRO protocol) ---
#define STATE_PAD_IDLE    0
#define STATE_ASCENT      1
#define STATE_DESCENT     2
#define STATE_LANDED      3

// --- $PYRO Flags Bitfield ---
#define FLAG_P1_CONT      (1 << 0)
#define FLAG_P2_CONT      (1 << 1)
#define FLAG_P1_FIRED     (1 << 2)
#define FLAG_P2_FIRED     (1 << 3)
#define FLAG_ARMED        (1 << 4)
#define FLAG_APOGEE       (1 << 5)
