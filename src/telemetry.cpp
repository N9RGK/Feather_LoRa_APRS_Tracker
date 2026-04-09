#include "telemetry.h"
#include "tracker_config.h"
#include "config.h"
#include "session_id.h"
#include <math.h>

// Phase display names for APRS comment field.
// Determined by altimeter state + thrust flag.
// Ground station refines further (DROGUE vs MAIN) using velocity.
const char* telemetry_phase_name(uint8_t state, uint8_t thrust) {
    switch (state) {
        case STATE_PAD_IDLE: return "PAD";
        case STATE_ASCENT:   return thrust ? "BOOST" : "COAST";
        case STATE_DESCENT:  return "DESCENT";
        case STATE_LANDED:   return "LANDED";
        default:             return "UNK";
    }
}

// Event code to string mapping for {{E:c<code>,...}} packets.
const char* telemetry_event_code_str(uint8_t code) {
    switch (code) {
        case EVT_BURNOUT:     return "B";
        case EVT_APOGEE:      return "A";
        case EVT_P1_FIRE:     return "P1";
        case EVT_P1_FAIL:     return "P1F";
        case EVT_DROGUE_OK:   return "D";
        case EVT_DROGUE_FAIL: return "DF";
        case EVT_P2_FIRE:     return "P2";
        case EVT_P2_FAIL:     return "P2F";
        case EVT_MAIN_OK:     return "M";
        case EVT_MAIN_FAIL:   return "MF";
        case EVT_LANDED:      return "L";
        case EVT_REPORT:      return "R";
        default:              return "?";
    }
}

// Build a TNC2-format APRS position+comment packet string.
// Format: CALLSIGN>APRS,WIDE1-1:!DDMM.hhN/DDDMM.hhWO comment
// Comment: "Alt:1677 St:COAST Id:N9RGK-1" — fields match GS aprs_parser.py
// Compatible with CA2RXU LoRa_APRS_Tracker/iGate receivers.
String telemetry_build_aprs_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign) {
    // Build comment field with telemetry summary.
    // GS parser expects: Alt:(-?\d+) St:(\w+) Id:(\S+)
    const char* tid = tracker_config_get()->tracker_id;
    String comment;
    if (alt && alt->valid) {
        int32_t alt_m = alt->alt_cm / 100;
        int32_t vel_ms = alt->vel_cms / 100;
        const char* phase = telemetry_phase_name(alt->state, alt->thrust);

        char buf[80];
        snprintf(buf, sizeof(buf), "Alt:%ld St:%s V:%s%ldm/s Id:%s",
                 (long)alt_m, phase,
                 (vel_ms >= 0) ? "+" : "",
                 (long)vel_ms, tid);
        comment = buf;
    } else {
        // No altimeter — use GPS altitude if available, otherwise 0
        int32_t alt_m = (fix && fix->valid) ? (int32_t)fix->altitude_m : 0;
        char buf[64];
        snprintf(buf, sizeof(buf), "Alt:%ld St:PAD Id:%s", (long)alt_m, tid);
        comment = buf;
    }

    // Append session ID (always available — rolling counter, no GPS dependency)
    comment += " Ss:";
    comment += session_id_get();

    // Fallback to zero-position if GPS not valid
    double lat = fix->valid ? fix->lat : 0.0;
    double lng = fix->valid ? fix->lng : 0.0;

    // Encode position in uncompressed APRS format
    String pos = telemetry_encode_aprs_position(lat, lng);

    // TNC2 format: CALLSIGN>APRS,WIDE1-1:!position comment
    return String(callsign) + ">APRS,WIDE1-1:" + pos + " " + comment;
}

// Build a compact APRS packet for MODE_EVENT.
// Shorter comment: "Alt:1677 St:BOOST Id:N9RGK-1" — matches GS parser.
// At SF12 this saves airtime vs full comment with velocity.
String telemetry_build_compact_aprs_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign) {
    const char* tid = tracker_config_get()->tracker_id;
    String comment;
    if (alt && alt->valid) {
        int32_t alt_m = alt->alt_cm / 100;
        const char* phase = telemetry_phase_name(alt->state, alt->thrust);
        char buf[48];
        snprintf(buf, sizeof(buf), "Alt:%ld St:%s Id:%s", (long)alt_m, phase, tid);
        comment = buf;
    } else {
        int32_t alt_m = (fix && fix->valid) ? (int32_t)fix->altitude_m : 0;
        char buf[48];
        snprintf(buf, sizeof(buf), "Alt:%ld St:PAD Id:%s", (long)alt_m, tid);
        comment = buf;
    }

    // Append session ID (always available — rolling counter, no GPS dependency)
    comment += " Ss:";
    comment += session_id_get();

    double lat = fix->valid ? fix->lat : 0.0;
    double lng = fix->valid ? fix->lng : 0.0;
    String pos = telemetry_encode_aprs_position(lat, lng);

    return String(callsign) + ">APRS,WIDE1-1:" + pos + " " + comment;
}

// Build a dense telemetry packet with full data set.
// Format: CALLSIGN>APRS,WIDE1-1:{{T2:alt5501,vel12.3,mxa5501,st1,th0,...}}
// This is our proprietary format — only our ground station parses it.
String telemetry_build_dense_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign) {
    const TrackerConfig* cfg = tracker_config_get();
    char buf[220];

    // Build session ID field (always available — rolling counter)
    char ss_field[8];
    snprintf(ss_field, sizeof(ss_field), "ss%s,", session_id_get());

    if (alt && alt->valid) {
        int32_t alt_m = alt->alt_cm / 100;
        int32_t mxa_m = alt->max_alt_cm / 100;
        // Velocity: one decimal place (cm/s -> m/s with .1 resolution)
        int32_t vel_whole = alt->vel_cms / 100;
        int32_t vel_frac = (abs(alt->vel_cms) % 100) / 10;  // tenths

        // Build pyro status string from flags
        // Format: continuity1 continuity2 fired1 fired2 armed apogee
        char py[8];
        snprintf(py, sizeof(py), "%c%c%c%c%c%c",
                 (alt->flags & FLAG_P1_CONT)  ? 'C' : '-',
                 (alt->flags & FLAG_P2_CONT)  ? 'C' : '-',
                 (alt->flags & FLAG_P1_FIRED) ? 'F' : '-',
                 (alt->flags & FLAG_P2_FIRED) ? 'F' : '-',
                 (alt->flags & FLAG_ARMED)    ? 'A' : '-',
                 (alt->flags & FLAG_APOGEE)   ? 'G' : '-');

        uint32_t tm_s = alt->flight_time_ms / 1000;

        snprintf(buf, sizeof(buf),
                 "{{T2:id%s,%salt%ld,vel%ld.%ld,mxa%ld,st%u,th%u,"
                 "py%s,pa%u:%u,pr%ld,bt%u,tp%d,tm%lu,sq%u,fl%04X}}",
                 cfg->tracker_id,
                 ss_field,
                 (long)alt_m,
                 (long)vel_whole, (long)vel_frac,
                 (long)mxa_m,
                 alt->state,
                 alt->thrust,
                 py,
                 alt->p1_adc, alt->p2_adc,
                 (long)alt->press_pa,
                 alt->batt_adc,
                 alt->temp_deci_c,
                 (unsigned long)tm_s,
                 alt->seq,
                 alt->flags);
    } else {
        snprintf(buf, sizeof(buf), "{{T2:id%s,%snoalt}}", cfg->tracker_id, ss_field);
    }

    // Append GPS coordinates as fields inside the T2 payload.
    // Critical for Full mode where no APRS position packets are sent.
    // Fields: la<lat>,ln<lng>,gs<speed_kmh>,co<course>,sa<satellites>
    String payload = String(buf);
    if (fix && fix->valid) {
        char gps_buf[80];
        snprintf(gps_buf, sizeof(gps_buf),
                 ",la%.5f,ln%.5f,gs%.1f,co%.0f,sa%u",
                 fix->lat, fix->lng,
                 (double)fix->speed_kmh,
                 (double)fix->course_deg,
                 fix->satellites);
        // Insert before the closing "}}"
        int closingIdx = payload.lastIndexOf("}}");
        if (closingIdx >= 0) {
            payload = payload.substring(0, closingIdx) + gps_buf + "}}";
        }
    }

    String pkt = String(callsign) + ">APRS,WIDE1-1:" + payload;
    return pkt;
}

// Build an event packet from a FlightEvent.
// Format: CALLSIGN>APRS,WIDE1-1:{{E:c<code>,t<time>,a<alt>,v<vel>}}
// LANDED event adds GPS: {{E:cL,t52,a0,v0,la41.12345,ln-73.12345}}
// REPORT event adds errors: {{E:cR,t52,a1677,v89,er00}}
String telemetry_build_event_packet(const FlightEvent* evt, const char* callsign) {
    const char* code_str = telemetry_event_code_str(evt->code);

    char buf[140];
    const TrackerConfig* cfg = tracker_config_get();

    char ss_field[8];
    snprintf(ss_field, sizeof(ss_field), "ss%s,", session_id_get());

    int len = snprintf(buf, sizeof(buf), "{{E:id%s,%sc%s,t%lu,a%ld,v%ld",
                       cfg->tracker_id,
                       ss_field,
                       code_str,
                       (unsigned long)evt->flight_time_s,
                       (long)evt->alt_m,
                       (long)evt->vel_mps);

    // Append GPS position for LANDED and REPORT events
    if (evt->has_gps) {
        len += snprintf(buf + len, sizeof(buf) - len,
                        ",la%.5f,ln%.5f",
                        evt->lat, evt->lng);
    }

    // Append error flags for REPORT event
    if (evt->code == EVT_REPORT) {
        len += snprintf(buf + len, sizeof(buf) - len,
                        ",er%02X", evt->error_flags);
    }

    snprintf(buf + len, sizeof(buf) - len, "}}");

    return String(callsign) + ">APRS,WIDE1-1:" + String(buf);
}

// Build a flight curve packet — raw pressure + altitude + velocity + state +
// flight time + sequence. No GPS, no pyro, no extras. Designed for maximum
// data rate to reconstruct the altitude/pressure curve post-flight.
// Format: CALLSIGN>APRS,WIDE1-1:{{C:pr<press>,a<alt>,v<vel>,st<state>,t<time_ds>,sq<seq>}}
// ~65 bytes total (with header). Smallest practical telemetry packet.
// Flight time is in deciseconds (tenths of a second) for 100ms curve resolution.
String telemetry_build_curve_packet(const AltimeterData* alt, const char* callsign) {
    const TrackerConfig* cfg = tracker_config_get();
    char buf[100];

    char ss_field[8];
    snprintf(ss_field, sizeof(ss_field), "ss%s,", session_id_get());

    if (alt && alt->valid) {
        int32_t alt_m = alt->alt_cm / 100;
        int32_t vel_whole = alt->vel_cms / 100;
        int32_t vel_frac = (abs(alt->vel_cms) % 100) / 10;
        // Deciseconds (100ms resolution) for curve reconstruction fidelity
        uint32_t tm_ds = alt->flight_time_ms / 100;

        snprintf(buf, sizeof(buf),
                 "{{C:id%s,%spr%ld,a%ld,v%ld.%ld,st%u,t%lu,sq%u}}",
                 cfg->tracker_id,
                 ss_field,
                 (long)alt->press_pa,
                 (long)alt_m,
                 (long)vel_whole, (long)vel_frac,
                 alt->state,
                 (unsigned long)tm_ds,
                 alt->seq);
    } else {
        snprintf(buf, sizeof(buf), "{{C:id%s,%snoalt}}", cfg->tracker_id, ss_field);
    }

    return String(callsign) + ">APRS,WIDE1-1:" + String(buf);
}

// Encode lat/lng in uncompressed APRS position format.
// Format: !DDMM.hhN/DDDMM.hhWO
// The 'O' symbol after the longitude indicates "balloon" (closest APRS
// symbol to a rocket — there is no dedicated rocket symbol in APRS).
String telemetry_encode_aprs_position(double lat, double lng) {
    char buf[48];

    // Latitude: DDMM.hhN
    char lat_ns = (lat >= 0) ? 'N' : 'S';
    lat = fabs(lat);
    int lat_deg = (int)lat;
    float lat_min = (lat - lat_deg) * 60.0f;

    // Longitude: DDDMM.hhW
    char lng_ew = (lng >= 0) ? 'E' : 'W';
    lng = fabs(lng);
    int lng_deg = (int)lng;
    float lng_min = (lng - lng_deg) * 60.0f;

    // APRS uncompressed: !DDMM.hhN/DDDMM.hhWO
    snprintf(buf, sizeof(buf), "!%02d%05.2f%c/%03d%05.2f%cO",
             lat_deg, lat_min, lat_ns,
             lng_deg, lng_min, lng_ew);

    return String(buf);
}
