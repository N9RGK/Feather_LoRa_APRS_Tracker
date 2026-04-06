#include "event_detector.h"
#include "altimeter_rx.h"
#include "config.h"
#include <Arduino.h>

// --- Previous state for transition detection ---
static uint8_t  prev_state   = STATE_PAD_IDLE;
static uint8_t  prev_thrust  = 0;
static uint16_t prev_flags   = 0;
static bool     prev_valid   = false;
static bool     was_in_flight = false;

// --- Flight report tracking ---
static int32_t  max_alt_m    = 0;
static int32_t  max_vel_mps  = 0;
static uint8_t  flight_errors = 0;     // FLERR_* accumulator
static bool     report_sent  = false;
static uint32_t landed_at_ms = 0;      // millis() when LANDED detected
static bool     gps_was_valid = false;  // Track GPS loss during flight

// --- Event sentence deduplication ---
static bool apogee_from_sentence = false;   // $PYRO_APO already processed
static bool p1_fire_from_sentence = false;  // $PYRO_FIRE ch=1 already processed
static bool p2_fire_from_sentence = false;  // $PYRO_FIRE ch=2 already processed
static bool landing_from_sentence = false;  // $PYRO_LAND already processed

// --- Event queue (circular buffer) ---
static FlightEvent queue[EVENT_QUEUE_SIZE];
static uint8_t q_head = 0;  // next slot to write
static uint8_t q_tail = 0;  // next slot to read
static uint8_t q_count = 0;

static void push_event(const FlightEvent* evt) {
    if (q_count >= EVENT_QUEUE_SIZE) {
        // Queue full — drop oldest event to make room
        q_tail = (q_tail + 1) % EVENT_QUEUE_SIZE;
        q_count--;
    }
    queue[q_head] = *evt;
    q_head = (q_head + 1) % EVENT_QUEUE_SIZE;
    q_count++;
}

// Build an event snapshot from the current altimeter + GPS state.
static FlightEvent make_event(uint8_t code, const AltimeterData* alt,
                               const GpsFix* fix, bool include_gps) {
    FlightEvent evt = {};
    evt.code = code;
    evt.flight_time_s = alt->flight_time_ms / 1000;
    evt.alt_m = alt->alt_cm / 100;
    evt.vel_mps = alt->vel_cms / 100;
    if (include_gps && fix && fix->valid) {
        evt.has_gps = true;
        evt.lat = fix->lat;
        evt.lng = fix->lng;
    }
    return evt;
}

// Build the flight report event with maximums and error summary.
static FlightEvent make_report(const AltimeterData* alt, const GpsFix* fix) {
    FlightEvent evt = {};
    evt.code = EVT_REPORT;
    evt.flight_time_s = alt->flight_time_ms / 1000;
    evt.alt_m = max_alt_m;
    evt.vel_mps = max_vel_mps;
    evt.error_flags = flight_errors;
    if (fix && fix->valid) {
        evt.has_gps = true;
        evt.lat = fix->lat;
        evt.lng = fix->lng;
    }
    return evt;
}

void event_detector_init() {
    event_detector_reset();
}

void event_detector_reset() {
    prev_state = STATE_PAD_IDLE;
    prev_thrust = 0;
    prev_flags = 0;
    prev_valid = false;
    was_in_flight = false;
    max_alt_m = 0;
    max_vel_mps = 0;
    flight_errors = 0;
    report_sent = false;
    landed_at_ms = 0;
    gps_was_valid = false;
    q_head = 0;
    q_tail = 0;
    q_count = 0;
    apogee_from_sentence = false;
    p1_fire_from_sentence = false;
    p2_fire_from_sentence = false;
    landing_from_sentence = false;
}

void event_detector_update(const AltimeterData* alt, const GpsFix* fix) {
    if (!alt || !alt->valid) return;

    uint8_t  cur_state  = alt->state;
    uint8_t  cur_thrust = alt->thrust;
    uint16_t cur_flags  = alt->flags;
    int32_t  cur_alt_m  = alt->alt_cm / 100;
    int32_t  cur_vel_mps = alt->vel_cms / 100;

    // --- Track flight maximums ---
    if (cur_state == STATE_ASCENT || cur_state == STATE_DESCENT) {
        was_in_flight = true;
        if (cur_alt_m > max_alt_m) max_alt_m = cur_alt_m;
        if (cur_vel_mps > max_vel_mps) max_vel_mps = cur_vel_mps;
    }

    // --- Track GPS loss during flight ---
    if (was_in_flight && cur_state != STATE_LANDED) {
        if (fix && fix->valid) {
            gps_was_valid = true;
        } else if (gps_was_valid) {
            // GPS was valid but now isn't — GPS lost during flight
            flight_errors |= FLERR_GPS_LOST;
        }
    }

    // --- Detect altimeter comm loss ---
    // (Handled externally — if altimeter_rx stops providing valid data,
    //  the main loop sets FLERR_ALT_LOST in flight_errors.)

    // Skip transition detection on the very first valid reading
    if (!prev_valid) {
        prev_state = cur_state;
        prev_thrust = cur_thrust;
        prev_flags = cur_flags;
        prev_valid = true;
        return;
    }

    // === EVENT SENTENCES — authoritative altimeter-side events ===
    // These carry exact altitude/time from the altimeter's state machine.
    // If we detect an event here, set a flag to suppress the duplicate
    // detection from the periodic flag transition below.

    if (alt->apogee_event && !apogee_from_sentence) {
        FlightEvent evt = {};
        evt.code = EVT_APOGEE;
        evt.flight_time_s = alt->apogee_time_ms / 1000;
        evt.alt_m = alt->apogee_max_alt_cm / 100;
        evt.vel_mps = cur_vel_mps;  // not in sentence, use periodic value
        push_event(&evt);
        apogee_from_sentence = true;
        // Also mark flags so the flag-based check below won't double-fire
        prev_flags |= FLAG_APOGEE;
        Serial.println("[EVT] Apogee (from $PYRO_APO sentence)");
    }

    if (alt->pyro1_fire_event && !p1_fire_from_sentence) {
        FlightEvent evt = {};
        evt.code = EVT_P1_FIRE;
        evt.flight_time_s = alt->pyro1_fire_time_ms / 1000;
        evt.alt_m = alt->pyro1_fire_alt_cm / 100;
        evt.vel_mps = cur_vel_mps;
        push_event(&evt);
        p1_fire_from_sentence = true;
        prev_flags |= FLAG_P1_FIRED;
        Serial.println("[EVT] Pyro 1 fired (from $PYRO_FIRE sentence)");
    }

    if (alt->pyro2_fire_event && !p2_fire_from_sentence) {
        FlightEvent evt = {};
        evt.code = EVT_P2_FIRE;
        evt.flight_time_s = alt->pyro2_fire_time_ms / 1000;
        evt.alt_m = alt->pyro2_fire_alt_cm / 100;
        evt.vel_mps = cur_vel_mps;
        push_event(&evt);
        p2_fire_from_sentence = true;
        prev_flags |= FLAG_P2_FIRED;
        Serial.println("[EVT] Pyro 2 fired (from $PYRO_FIRE sentence)");
    }

    if (alt->landing_event && !landing_from_sentence) {
        FlightEvent evt = make_event(EVT_LANDED, alt, fix, true);  // include GPS
        evt.flight_time_s = alt->landing_time_ms / 1000;
        evt.alt_m = alt->alt_cm / 100;  // current altitude, not max
        push_event(&evt);
        landing_from_sentence = true;
        // Set prev_state so the state-based check below won't double-fire
        prev_state = STATE_LANDED;
        landed_at_ms = millis();
        report_sent = false;
        Serial.println("[EVT] Landed (from $PYRO_LAND sentence)");
    }

    // === EVENT DETECTION — check for state/flag transitions ===

    // BURNOUT: thrust 1→0
    if (prev_thrust == 1 && cur_thrust == 0) {
        FlightEvent evt = make_event(EVT_BURNOUT, alt, fix, false);
        push_event(&evt);
        Serial.println("[EVT] Burnout detected");
    }

    // APOGEE: FLAG_APOGEE newly set
    if (!(prev_flags & FLAG_APOGEE) && (cur_flags & FLAG_APOGEE)) {
        FlightEvent evt = make_event(EVT_APOGEE, alt, fix, false);
        push_event(&evt);
        Serial.println("[EVT] Apogee detected");
    }

    // PYRO 1 FIRED: FLAG_P1_FIRED newly set
    if (!(prev_flags & FLAG_P1_FIRED) && (cur_flags & FLAG_P1_FIRED)) {
        FlightEvent evt = make_event(EVT_P1_FIRE, alt, fix, false);
        push_event(&evt);
        Serial.println("[EVT] Pyro 1 fired");
    }

    // PYRO 1 FAILURE: EFLAG_P1_FAIL newly set (altimeter declared failure)
    if (!(prev_flags & EFLAG_P1_FAIL) && (cur_flags & EFLAG_P1_FAIL)) {
        FlightEvent evt = make_event(EVT_P1_FAIL, alt, fix, false);
        push_event(&evt);
        flight_errors |= FLERR_P1_FAIL;
        Serial.println("[EVT] Pyro 1 FAILURE");
    }

    // DROGUE DEPLOYED: EFLAG_DROGUE_OK newly set
    if (!(prev_flags & EFLAG_DROGUE_OK) && (cur_flags & EFLAG_DROGUE_OK)) {
        FlightEvent evt = make_event(EVT_DROGUE_OK, alt, fix, false);
        push_event(&evt);
        Serial.println("[EVT] Drogue deployment confirmed");
    }

    // DROGUE FAILURE: EFLAG_DROGUE_FAIL newly set
    if (!(prev_flags & EFLAG_DROGUE_FAIL) && (cur_flags & EFLAG_DROGUE_FAIL)) {
        FlightEvent evt = make_event(EVT_DROGUE_FAIL, alt, fix, false);
        push_event(&evt);
        flight_errors |= FLERR_DROGUE_FAIL;
        Serial.println("[EVT] Drogue FAILURE");
    }

    // PYRO 2 FIRED: FLAG_P2_FIRED newly set
    if (!(prev_flags & FLAG_P2_FIRED) && (cur_flags & FLAG_P2_FIRED)) {
        FlightEvent evt = make_event(EVT_P2_FIRE, alt, fix, false);
        push_event(&evt);
        Serial.println("[EVT] Pyro 2 fired");
    }

    // PYRO 2 FAILURE: EFLAG_P2_FAIL newly set
    if (!(prev_flags & EFLAG_P2_FAIL) && (cur_flags & EFLAG_P2_FAIL)) {
        FlightEvent evt = make_event(EVT_P2_FAIL, alt, fix, false);
        push_event(&evt);
        flight_errors |= FLERR_P2_FAIL;
        Serial.println("[EVT] Pyro 2 FAILURE");
    }

    // MAIN DEPLOYED: EFLAG_MAIN_OK newly set
    if (!(prev_flags & EFLAG_MAIN_OK) && (cur_flags & EFLAG_MAIN_OK)) {
        FlightEvent evt = make_event(EVT_MAIN_OK, alt, fix, false);
        push_event(&evt);
        Serial.println("[EVT] Main deployment confirmed");
    }

    // MAIN FAILURE: EFLAG_MAIN_FAIL newly set
    if (!(prev_flags & EFLAG_MAIN_FAIL) && (cur_flags & EFLAG_MAIN_FAIL)) {
        FlightEvent evt = make_event(EVT_MAIN_FAIL, alt, fix, false);
        push_event(&evt);
        flight_errors |= FLERR_MAIN_FAIL;
        Serial.println("[EVT] Main FAILURE");
    }

    // LANDED: state transitions to STATE_LANDED
    if (prev_state != STATE_LANDED && cur_state == STATE_LANDED) {
        FlightEvent evt = make_event(EVT_LANDED, alt, fix, true);  // include GPS
        push_event(&evt);
        landed_at_ms = millis();
        report_sent = false;
        Serial.println("[EVT] Landed");
    }

    // FLIGHT REPORT: sent once, EVENT_REPORT_DELAY_MS after landing
    if (cur_state == STATE_LANDED && !report_sent && landed_at_ms > 0) {
        if (millis() - landed_at_ms >= EVENT_REPORT_DELAY_MS) {
            FlightEvent evt = make_report(alt, fix);
            push_event(&evt);
            report_sent = true;
            Serial.println("[EVT] Flight report queued");
        }
    }

    // Clear event sentence flags after processing
    // (Must happen every update, not just when events fire, because
    //  the altimeter_rx doesn't auto-clear these flags.)
    altimeter_rx_clear_events();

    // Save current state for next comparison
    prev_state = cur_state;
    prev_thrust = cur_thrust;
    prev_flags = cur_flags;
}

bool event_detector_has_events() {
    return q_count > 0;
}

const FlightEvent* event_detector_peek() {
    if (q_count == 0) return nullptr;
    return &queue[q_tail];
}

void event_detector_pop() {
    if (q_count == 0) return;
    q_tail = (q_tail + 1) % EVENT_QUEUE_SIZE;
    q_count--;
}
