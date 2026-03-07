#pragma once

// --- APRS Configuration ---
#define CALLSIGN        "N9RGK-1"

// --- Telemetry Mode ---
// Configurable via HTTP interface when USB-connected to ground station.
// MODE_APRS:   APRS position packets only (1 Hz, CA2RXU-compatible)
// MODE_FULL:   Dense telemetry packets only (1 Hz, GPS included in payload)
// MODE_HYBRID: Alternating APRS + dense packets (2 Hz effective rate)
// MODE_EVENT:  APRS position beacons + burst event packets on state changes.
//              Optimized for long-range SF12. Events repeat for reliability.
// MODE_CURVE:  Flight curve mapper. Sends raw pressure + altitude + velocity
//              + state + flight time at maximum practical rate. Designed for
//              post-flight altitude curve reconstruction. No GPS, no APRS —
//              just barometric data as fast as the radio allows.
#define MODE_APRS       0
#define MODE_FULL       1
#define MODE_HYBRID     2
#define MODE_EVENT      3
#define MODE_CURVE      4

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

// --- MODE_EVENT APRS Beacon Rates (ms) ---
// Slower than standard APRS since events carry the critical flight data.
#define EVENT_APRS_RATE_PAD_MS       30000  // 30s heartbeat on pad
#define EVENT_APRS_RATE_ASCENT_MS     3000  // 3s position tracking
#define EVENT_APRS_RATE_DESCENT_MS    5000  // 5s position for recovery
#define EVENT_APRS_RATE_LANDED_MS    10000  // 10s "come find me" beacon

// Event packet repeat count — each event transmitted this many times
// back-to-back for reliability. At SF12 ~1.8s per packet, 3x = ~5.5s burst.
#define EVENT_REPEAT_COUNT     3

// Gap between event packet repeats (ms). Allows radio to reset.
#define EVENT_REPEAT_GAP_MS    200

// Max events that can be queued. Events are processed FIFO.
// In a worst case (simultaneous transitions) we might queue 3-4 events.
#define EVENT_QUEUE_SIZE       8

// Delay after LANDED before sending flight report (ms).
// Allows final events (main detect, etc.) to settle.
#define EVENT_REPORT_DELAY_MS  5000

// --- Event Codes ---
// Transmitted in the {{E:c<code>,...}} packet. Ground station uses these
// to populate the flight event timeline.
#define EVT_BURNOUT      'B'   // Motor burnout — thrust flag 1→0
#define EVT_APOGEE       'A'   // Apogee detected by altimeter
#define EVT_P1_FIRE      1     // Pyro 1 fired (code "P1")
#define EVT_P1_FAIL      2     // Pyro 1 failure declared (code "P1F")
#define EVT_DROGUE_OK    3     // Drogue deployment confirmed (code "D")
#define EVT_DROGUE_FAIL  4     // Drogue failure declared (code "DF")
#define EVT_P2_FIRE      5     // Pyro 2 fired (code "P2")
#define EVT_P2_FAIL      6     // Pyro 2 failure declared (code "P2F")
#define EVT_MAIN_OK      7     // Main deployment confirmed (code "M")
#define EVT_MAIN_FAIL    8     // Main failure declared (code "MF")
#define EVT_LANDED       'L'   // Landed detected
#define EVT_REPORT       'R'   // Flight report — max values + error summary

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

// --- $PYRO Flags Bitfield (16-bit) ---
// Bits 0-5: Original flags (always present from altimeter)
#define FLAG_P1_CONT      (1 << 0)
#define FLAG_P2_CONT      (1 << 1)
#define FLAG_P1_FIRED     (1 << 2)
#define FLAG_P2_FIRED     (1 << 3)
#define FLAG_ARMED        (1 << 4)
#define FLAG_APOGEE       (1 << 5)

// Bits 6-11: Extended flags (set by altimeter's event detection logic).
// These are persistent — once set, they stay set for the rest of the flight.
// The tracker watches for 0→1 transitions to trigger event packets.
#define EFLAG_P1_FAIL      (1 << 6)    // Pyro 1 failure declared
#define EFLAG_P2_FAIL      (1 << 7)    // Pyro 2 failure declared
#define EFLAG_DROGUE_OK    (1 << 8)    // Drogue deployment confirmed
#define EFLAG_DROGUE_FAIL  (1 << 9)    // Drogue failure declared
#define EFLAG_MAIN_OK      (1 << 10)   // Main deployment confirmed
#define EFLAG_MAIN_FAIL    (1 << 11)   // Main failure declared

// --- Flight Report Error Bitfield ---
// Sent in the {{E:cR,...,er<hex>}} flight report event.
// Summarizes all failures detected during the flight.
#define FLERR_P1_FAIL      (1 << 0)
#define FLERR_P2_FAIL      (1 << 1)
#define FLERR_DROGUE_FAIL  (1 << 2)
#define FLERR_MAIN_FAIL    (1 << 3)
#define FLERR_GPS_LOST     (1 << 4)
#define FLERR_ALT_LOST     (1 << 5)
