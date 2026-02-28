#pragma once

// --- APRS Configuration ---
#define CALLSIGN        "N9RGK-1"
#define APRS_COMMENT    "Rocket Telemetry"

// --- LoRa Radio Settings (CA2RXU-compatible) ---
#define LORA_FREQ_MHZ       443.775  // 70cm amateur band
#define LORA_BW_KHZ         125.0    // Bandwidth: 125 kHz
#define LORA_SF             12       // Spreading Factor 12 (max range)
#define LORA_CR             5        // Coding Rate 4/5
#define LORA_SYNC_WORD      0x12     // LoRa APRS sync word
#define LORA_POWER_DBM      17       // TX power in dBm (max 20 for RFM96)

// --- Transmit Intervals ---
#define TX_INTERVAL_MS  30000        // 30 seconds between APRS beacons

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
#define ALTIMETER_BAUD    9600
