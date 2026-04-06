#include <Arduino.h>
#include "config.h"
#include "gps_handler.h"
#include "lora_aprs.h"
#include "altimeter_rx.h"
#include "tracker_config.h"
#include "serial_cmd.h"
#include "event_detector.h"
#include "lora_airtime.h"
#include "airtime.h"
#include "session_id.h"

// ---------------------------------------------------------------
// Airtime-aware scheduling
// ---------------------------------------------------------------
// Instead of static interval constants, the scheduler enforces that
// no transmission can happen sooner than the radio's physical airtime
// plus a turnaround margin. This lets all modes work correctly at
// both SF8 (~200ms per APRS packet) and SF12 (~2800ms per APRS packet)
// without any code changes — the scheduler adapts automatically.
//
// The configured intervals (aprs_rate_ascent_ms etc.) set the DESIRED
// rate. The scheduler clamps to max(desired_interval, min_safe_interval).
// ---------------------------------------------------------------

// Cached minimum intervals — recomputed when config changes.
static uint32_t min_aprs_interval_ms = 0;
static uint32_t min_dense_interval_ms = 0;
static uint32_t min_compact_interval_ms = 0;
static uint32_t min_event_interval_ms = 0;
static uint32_t min_curve_interval_ms = 0;

// Recompute minimum safe intervals from current radio config.
// Call at boot and after any config change (e.g. CDC SET command).
//
// Dead air between packets is proportional to airtime:
//   dead_air = max(airtime * pct%, floor_ms)
//
// This keeps the channel clear for the receiver to re-sync and gives
// co-channel devices a window to transmit. The proportional approach
// automatically scales with SF — at SF12 a 3s packet gets ~600ms
// of dead air, not a fixed 200ms that would crowd the channel.
//
// Dead-air budget by packet type:
//   Standard (APRS/Dense/Compact): 20% of airtime, floor 250ms
//   Event repeats:                 15% of airtime, floor 200ms
//   Curve (max throughput):        10% of airtime, floor 150ms
void recompute_airtime_limits() {
    const TrackerConfig* cfg = tracker_config_get();

    // APRS position packet (~89 bytes payload + 3 OE header)
    // +12 bytes for session ID field " Ss:XXXXXXXX"
    min_aprs_interval_ms = lora_min_interval_ms(
        3 + 89, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr, 20, 250);

    // Dense telemetry packet (~175 bytes payload + 3 OE header)
    // +12 bytes for session ID field "ssXXXXXXXX,"
    min_dense_interval_ms = lora_min_interval_ms(
        3 + 175, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr, 20, 250);

    // Compact APRS packet (~72 bytes payload + 3 OE header)
    // +12 bytes for session ID field " Ss:XXXXXXXX"
    min_compact_interval_ms = lora_min_interval_ms(
        3 + 72, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr, 20, 250);

    // Event packet (~67 bytes payload + 3 OE header)
    // +12 bytes for session ID field "ssXXXXXXXX,"
    // Slightly tighter — event bursts need to finish promptly so we
    // can get back to APRS beacons, but still need receiver re-sync time.
    min_event_interval_ms = lora_min_interval_ms(
        3 + 67, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr, 15, 200);

    // Curve packet (~77 bytes payload + 3 OE header)
    // +12 bytes for session ID field "ssXXXXXXXX,"
    // Tightest budget — maximum data rate is the goal, but still leave
    // enough dead air for the radio to fully reset and the receiver to
    // process the previous packet before the next preamble arrives.
    min_curve_interval_ms = lora_min_interval_ms(
        3 + 77, cfg->lora_sf, cfg->lora_bw_khz, cfg->lora_cr, 10, 150);

    Serial.println("Airtime limits (ms):");
    Serial.print("  APRS:    "); Serial.println(min_aprs_interval_ms);
    Serial.print("  Dense:   "); Serial.println(min_dense_interval_ms);
    Serial.print("  Compact: "); Serial.println(min_compact_interval_ms);
    Serial.print("  Event:   "); Serial.println(min_event_interval_ms);
    Serial.print("  Curve:   "); Serial.println(min_curve_interval_ms);
}

// Clamp a desired interval to the minimum safe airtime-based interval.
static inline uint32_t clamp_interval(uint32_t desired, uint32_t minimum) {
    return (desired > minimum) ? desired : minimum;
}

// Get the phase-dependent transmit interval for the current flight state.
// Returns interval in ms, clamped to the minimum safe airtime.
static uint32_t get_phase_interval(uint32_t airtime_min) {
    const TrackerConfig* cfg = tracker_config_get();
    const AltimeterData* alt = altimeter_rx_get();

    uint32_t desired;
    if (!alt->valid) {
        desired = cfg->aprs_rate_pad_ms;
    } else {
        switch (alt->state) {
            case STATE_PAD_IDLE: desired = cfg->aprs_rate_pad_ms; break;
            case STATE_ASCENT:   desired = cfg->aprs_rate_ascent_ms; break;
            case STATE_DESCENT:  desired = cfg->aprs_rate_descent_ms; break;
            case STATE_LANDED:   desired = cfg->aprs_rate_landed_ms; break;
            default:             desired = cfg->aprs_rate_pad_ms; break;
        }
    }

    return clamp_interval(desired, airtime_min);
}

// Get the phase-dependent APRS beacon interval for MODE_EVENT.
// Slower than standard APRS since events carry the critical data.
static uint32_t get_event_aprs_interval() {
    const TrackerConfig* cfg = tracker_config_get();
    const AltimeterData* alt = altimeter_rx_get();

    uint32_t desired;
    if (!alt->valid) {
        desired = cfg->event_aprs_rate_pad_ms;
    } else {
        switch (alt->state) {
            case STATE_PAD_IDLE: desired = cfg->event_aprs_rate_pad_ms; break;
            case STATE_ASCENT:   desired = cfg->event_aprs_rate_ascent_ms; break;
            case STATE_DESCENT:  desired = cfg->event_aprs_rate_descent_ms; break;
            case STATE_LANDED:   desired = cfg->event_aprs_rate_landed_ms; break;
            default:             desired = cfg->event_aprs_rate_pad_ms; break;
        }
    }

    return clamp_interval(desired, min_compact_interval_ms);
}

static const char* mode_name(uint8_t mode) {
    switch (mode) {
        case MODE_APRS:   return "APRS";
        case MODE_FULL:   return "Full";
        case MODE_HYBRID: return "Hybrid";
        case MODE_EVENT:  return "Event";
        case MODE_CURVE:  return "Curve";
        case MODE_TEST:   return "Test";
        default:          return "Unknown";
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // wait up to 3s for USB serial
    Serial.println();
    Serial.println("=== Feather LoRa APRS Tracker ===");

    // Load config from SPI flash (falls back to compiled defaults)
    tracker_config_init();
    const TrackerConfig* cfg = tracker_config_get();

    Serial.print("Callsign: ");
    Serial.println(cfg->callsign);
    Serial.print("Telemetry mode: ");
    Serial.println(mode_name(cfg->telemetry_mode));
    Serial.print("LoRa: SF");
    Serial.print(cfg->lora_sf);
    Serial.print(", BW ");
    Serial.print(cfg->lora_bw_khz, 0);
    Serial.print(" kHz, ");
    Serial.print(cfg->lora_freq_mhz);
    Serial.println(" MHz");

    Serial.print("GPS init... ");
    gps_handler_init();
    Serial.println("ok");

    Serial.print("Altimeter SERCOM1 init (");
    Serial.print(ALTIMETER_BAUD);
    Serial.print(" baud)... ");
    altimeter_rx_init();
    Serial.println("ok");

    Serial.print("LoRa radio init... ");
    lora_aprs_init(cfg);

    // Compute airtime-based minimum intervals for current radio settings
    recompute_airtime_limits();

    // Initialize event detector (used by MODE_EVENT, but always ready)
    event_detector_init();

    // Initialize session ID subsystem
    session_id_init();
    Serial.println("Session ID will be generated on first GPS fix");

    // Initialize CDC serial command handler
    serial_cmd_init();
    Serial.println("CDC serial command handler ready");

    if (cfg->telemetry_mode == MODE_CURVE) {
        Serial.println("*** CURVE MODE: max-rate barometric telemetry ***");
        Serial.print("  Packet interval: ");
        Serial.print(min_curve_interval_ms);
        Serial.println("ms");
        float rate_hz = 1000.0f / min_curve_interval_ms;
        Serial.print("  Effective rate: ");
        Serial.print(rate_hz, 1);
        Serial.println(" Hz");
    }

    if (cfg->telemetry_mode == MODE_TEST) {
        Serial.println();
        Serial.println("*** TEST MODE: USB-controlled transmissions only ***");
        Serial.println("*** Type SEND APRS to transmit a test packet     ***");
        Serial.println();
    }
}

// LED flash patterns to distinguish packet types
static void flash_aprs() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
}

static void flash_dense() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(30);
    digitalWrite(LED_BUILTIN, LOW);
}

// Rapid double-flash for event packets
static void flash_event() {
    for (int i = 0; i < 2; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(50);
    }
}

// Steady glow for curve mode (minimal overhead)
static void flash_curve() {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
    // Feed GPS and altimeter parsers
    gps_handler_update();
    session_id_update();
    altimeter_rx_update();

    // Process incoming CDC serial commands
    serial_cmd_update();

    // Run event detector every cycle (watches for altimeter state transitions).
    // Events are queued even in non-EVENT modes — they just aren't transmitted.
    const AltimeterData* alt = altimeter_rx_get();
    const GpsFix* fix = gps_handler_get_fix();
    event_detector_update(alt, fix);

    // Heartbeat blink to show the board is alive.
    // Suppressed in MODE_CURVE (minimal overhead).
    // MODE_TEST uses a slow single blink every 5s to visually indicate test mode.
    // All other modes blink every 2s.
    const TrackerConfig* cfg = tracker_config_get();
    if (cfg->telemetry_mode == MODE_TEST) {
        static uint32_t last_blink = 0;
        if (millis() - last_blink >= 5000) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(150);
            digitalWrite(LED_BUILTIN, LOW);
            last_blink = millis();
        }
    } else if (cfg->telemetry_mode != MODE_CURVE) {
        static uint32_t last_blink = 0;
        if (millis() - last_blink >= 2000) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(50);
            digitalWrite(LED_BUILTIN, LOW);
            last_blink = millis();
        }
    }

    static uint32_t last_aprs_tx = 0;
    static uint32_t last_dense_tx = 0;
    static uint32_t last_curve_tx = 0;
    uint32_t now = millis();

    switch (cfg->telemetry_mode) {

    // ---------------------------------------------------------------
    // MODE_APRS: Only APRS position packets at phase-dependent rate.
    // ~77 byte packets. At SF8: ~420ms airtime. At SF12: ~3060ms.
    // CA2RXU-compatible.
    // ---------------------------------------------------------------
    case MODE_APRS: {
        uint32_t interval = get_phase_interval(min_aprs_interval_ms);
        if (now - last_aprs_tx >= interval) {
            flash_aprs();
            lora_aprs_send();
            last_aprs_tx = now;

            Serial.print("  [APRS] every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        break;
    }

    // ---------------------------------------------------------------
    // MODE_FULL: Only dense telemetry packets at phase-dependent rate.
    // ~163 byte packets. At SF8: ~645ms airtime. At SF12: ~5840ms.
    // NOT CA2RXU-compatible. GPS embedded in dense payload.
    // ---------------------------------------------------------------
    case MODE_FULL: {
        uint32_t interval = get_phase_interval(min_dense_interval_ms);
        if (now - last_dense_tx >= interval) {
            flash_dense();
            lora_dense_send();
            last_dense_tx = now;

            Serial.print("  [FULL] every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        break;
    }

    // ---------------------------------------------------------------
    // MODE_HYBRID: Alternating APRS + dense packets.
    // APRS at phase-dependent intervals, dense fills the gaps.
    // At SF8: ~2 Hz effective. At SF12: the airtime clamp ensures
    // dense packets only send when there's enough time before the
    // next APRS beacon.
    // ---------------------------------------------------------------
    case MODE_HYBRID:
    default: {
        uint32_t interval = get_phase_interval(min_aprs_interval_ms);
        // Time for an APRS packet?
        if (now - last_aprs_tx >= interval) {
            flash_aprs();
            lora_aprs_send();
            last_aprs_tx = now;
            last_dense_tx = now;  // Reset dense timer to avoid double-send

            Serial.print("  [HYBRID] APRS every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        // Otherwise, is there room for a dense packet?
        else {
            uint32_t dense_interval = clamp_interval(
                cfg->dense_min_interval_ms, min_dense_interval_ms);
            if (now - last_dense_tx >= dense_interval) {
                // Only send dense if we have enough time before next APRS.
                // Need at least the dense airtime + margin before next APRS fires.
                uint32_t time_to_next_aprs = (last_aprs_tx + interval) - now;
                if (time_to_next_aprs > min_dense_interval_ms) {
                    flash_dense();
                    lora_dense_send();
                    last_dense_tx = now;
                }
            }
        }
        break;
    }

    // ---------------------------------------------------------------
    // MODE_EVENT: APRS position beacons + burst event packets.
    //
    // Normal operation: compact APRS beacons at event_aprs_rate_*_ms.
    // When an event is detected: immediately send the event packet
    // EVENT_REPEAT_COUNT times with airtime-clamped gap between sends.
    // APRS beacons pause during event bursts.
    //
    // At SF8:  event packets ~55B take ~340ms, bursts finish in ~1.6s
    // At SF12: event packets ~55B take ~2.4s, bursts finish in ~7.5s
    // ---------------------------------------------------------------
    case MODE_EVENT: {
        // Static state for event burst transmission
        static uint8_t  event_repeats_remaining = 0;
        static uint32_t last_event_tx = 0;

        // --- EVENT BURST MODE ---
        // If we're in the middle of repeating an event, keep going.
        if (event_repeats_remaining > 0) {
            uint32_t event_gap = clamp_interval(EVENT_REPEAT_GAP_MS, min_event_interval_ms);
            if (now - last_event_tx >= event_gap) {
                const FlightEvent* evt = event_detector_peek();
                if (evt) {
                    flash_event();
                    lora_event_send(evt);
                    last_event_tx = now;
                    event_repeats_remaining--;

                    Serial.print("  [EVENT] repeat ");
                    Serial.print(cfg->event_repeat_count - event_repeats_remaining);
                    Serial.print("/");
                    Serial.println(cfg->event_repeat_count);

                    // Done with this event's repeats?
                    if (event_repeats_remaining == 0) {
                        event_detector_pop();
                        // Reset APRS timer so we don't immediately send a beacon
                        last_aprs_tx = now;
                    }
                } else {
                    // Event disappeared from queue (shouldn't happen)
                    event_repeats_remaining = 0;
                }
            }
            break;  // Don't send APRS during event burst
        }

        // --- CHECK FOR NEW EVENTS ---
        // If there's a queued event and we're not currently bursting, start.
        if (event_detector_has_events()) {
            event_repeats_remaining = cfg->event_repeat_count;
            last_event_tx = 0;  // Send first repeat immediately
            break;              // Will send on next loop iteration
        }

        // --- NORMAL APRS BEACON ---
        uint32_t interval = get_event_aprs_interval();
        if (now - last_aprs_tx >= interval) {
            flash_aprs();
            lora_compact_aprs_send();
            last_aprs_tx = now;

            Serial.print("  [EVENT] APRS beacon every ");
            Serial.print(interval);
            Serial.println("ms");
        }
        break;
    }

    // ---------------------------------------------------------------
    // MODE_CURVE: Flight curve mapper.
    //
    // Sends raw barometric data as fast as the radio physically allows.
    // No GPS, no pyro data, no APRS compatibility — just pressure,
    // altitude, velocity, state, flight time (deciseconds), and sequence.
    // Packets use {{C:...}} format for ground station identification.
    //
    // At SF8/BW125:  ~65B → ~420ms airtime → ~2.1 Hz
    // At SF12/BW125: ~65B → ~2800ms airtime → ~0.35 Hz
    //
    // Designed for post-flight altitude/pressure curve reconstruction.
    // Ground station uses the sequence numbers and decisecond timestamps
    // to reconstruct the time series even if packets are lost.
    // ---------------------------------------------------------------
    case MODE_CURVE: {
        if (now - last_curve_tx >= min_curve_interval_ms) {
            flash_curve();
            lora_curve_send();
            last_curve_tx = now;
        }
        break;
    }

    // ---------------------------------------------------------------
    // MODE_TEST: No autonomous transmissions.
    // All transmissions are USB-controlled via SEND commands.
    // ---------------------------------------------------------------
    case MODE_TEST:
        break;

    }  // end switch
}
