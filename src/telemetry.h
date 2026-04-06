#pragma once

#include <Arduino.h>
#include "gps_handler.h"
#include "altimeter_rx.h"
#include "event_detector.h"

// Build a TNC2-format APRS position+comment packet.
// Comment: "Alt:1677 St:COAST V:+12m/s Id:N9RGK-1" — matches GS aprs_parser.py.
// Falls back to GPS altitude when altimeter unavailable.
String telemetry_build_aprs_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign);

// Build a compact APRS packet for MODE_EVENT.
// Comment: "Alt:1677 St:BOOST Id:N9RGK-1" — shorter, matches GS parser.
String telemetry_build_compact_aprs_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign);

// Build a dense telemetry packet with full data set.
// Format: CALLSIGN>APRS,WIDE1-1:{{T2:key1val1,key2val2,...}}
String telemetry_build_dense_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign);

// Build an event packet from a queued FlightEvent.
// Format: CALLSIGN>APRS,WIDE1-1:{{E:cB,t3,a457,v89}}
// For LANDED/REPORT events, includes GPS and/or error flags.
String telemetry_build_event_packet(const FlightEvent* evt, const char* callsign);

// Build a flight curve packet with raw barometric data for altitude curve
// reconstruction. Minimal payload for maximum rate.
// Format: CALLSIGN>APRS,WIDE1-1:{{C:pr101325,a1677,v89,st1,t12,sq42}}
// ~65 bytes total. At SF8 ~130ms, at SF12 ~2.1s.
String telemetry_build_curve_packet(const AltimeterData* alt, const char* callsign);

// Encode lat/lng in uncompressed APRS position format.
String telemetry_encode_aprs_position(double lat, double lng);

// Get the display name for a flight state + thrust combination.
const char* telemetry_phase_name(uint8_t state, uint8_t thrust);

// Get the string code for an event (e.g. EVT_P1_FIRE -> "P1").
const char* telemetry_event_code_str(uint8_t code);
