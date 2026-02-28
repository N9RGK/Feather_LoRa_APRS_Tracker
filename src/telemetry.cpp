#include "telemetry.h"
#include "config.h"
#include <math.h>

// Flight state names for APRS comment
static const char* STATE_NAMES[] = {
    "PAD", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"
};

// Build a TNC2-format APRS position+comment packet string.
// Format: CALLSIGN>APRS,WIDE1-1:!DDMM.hhN/DDDMM.hhWO comment
// Compatible with CA2RXU LoRa_APRS_Tracker/iGate receivers.
String telemetry_build_aprs_packet(const GpsFix* fix, const AltimeterData* alt) {
    // Build comment field with telemetry data
    String comment = APRS_COMMENT;
    if (alt && alt->valid) {
        comment += " Alt:";
        comment += (int)alt->altitude_m;
        comment += "m St:";
        if (alt->flight_state < 6) {
            comment += STATE_NAMES[alt->flight_state];
        } else {
            comment += alt->flight_state;
        }
    }

    // Fallback to zero-position if GPS not valid
    double lat = fix->valid ? fix->lat : 0.0;
    double lng = fix->valid ? fix->lng : 0.0;

    // Encode position in uncompressed APRS format
    String pos = telemetry_encode_aprs_position(lat, lng);

    // TNC2 format: CALLSIGN>APRS,WIDE1-1:!position comment
    return String(CALLSIGN) + ">APRS,WIDE1-1:" + pos + comment;
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
