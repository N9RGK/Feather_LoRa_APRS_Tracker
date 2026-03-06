#include "telemetry.h"
#include "config.h"
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

// Build a TNC2-format APRS position+comment packet string.
// Format: CALLSIGN>APRS,WIDE1-1:!DDMM.hhN/DDDMM.hhWO comment
// Comment: "Alt:1677m Phase:COAST V:+12m/s" (~50 chars, human-readable)
// Compatible with CA2RXU LoRa_APRS_Tracker/iGate receivers.
String telemetry_build_aprs_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign) {
    // Build comment field with telemetry summary
    String comment;
    if (alt && alt->valid) {
        int32_t alt_m = alt->alt_cm / 100;
        int32_t vel_ms = alt->vel_cms / 100;
        const char* phase = telemetry_phase_name(alt->state, alt->thrust);

        char buf[64];
        snprintf(buf, sizeof(buf), "Alt:%ldm Phase:%s V:%s%ldm/s",
                 (long)alt_m, phase,
                 (vel_ms >= 0) ? "+" : "",
                 (long)vel_ms);
        comment = buf;
    } else {
        comment = "No altimeter";
    }

    // Fallback to zero-position if GPS not valid
    double lat = fix->valid ? fix->lat : 0.0;
    double lng = fix->valid ? fix->lng : 0.0;

    // Encode position in uncompressed APRS format
    String pos = telemetry_encode_aprs_position(lat, lng);

    // TNC2 format: CALLSIGN>APRS,WIDE1-1:!position comment
    return String(callsign) + ">APRS,WIDE1-1:" + pos + " " + comment;
}

// Build a dense telemetry packet with full data set.
// Format: CALLSIGN>APRS,WIDE1-1:{{T2:alt5501,vel12.3,mxa5501,st1,th0,...}}
// This is our proprietary format — only our ground station parses it.
String telemetry_build_dense_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign) {
    char buf[200];

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
                 "{{T2:alt%ld,vel%ld.%ld,mxa%ld,st%u,th%u,"
                 "py%s,pa%u:%u,pr%ld,bt%u,tp%d,tm%lu,sq%u,fl%02X}}",
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
        snprintf(buf, sizeof(buf), "{{T2:noalt}}");
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

// Encode lat/lng in uncompressed APRS position format.
// Format: !DDMM.hhN/DDDMM.hhWO
// The 'O' symbol after the longitude indicates "balloon" (closest APRS
// symbol to a rocket — there is no dedicated rocket symbol in APRS).
String telemetry_encode_aprs_position(double lat, double lng) {
    char buf[32];

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
