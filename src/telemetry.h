#pragma once

#include <Arduino.h>
#include "gps_handler.h"
#include "altimeter_rx.h"

// Build a TNC2-format APRS position+comment packet.
// Comment field: ~50 chars human-readable with altitude, phase, velocity.
// Compatible with CA2RXU LoRa_APRS_Tracker/iGate receivers.
String telemetry_build_aprs_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign);

// Build a dense telemetry packet with full data set.
// Format: CALLSIGN>APRS,WIDE1-1:{{T2:key1val1,key2val2,...}}
String telemetry_build_dense_packet(const GpsFix* fix, const AltimeterData* alt, const char* callsign);

// Encode lat/lng in uncompressed APRS position format.
String telemetry_encode_aprs_position(double lat, double lng);

// Get the display name for a flight state + thrust combination.
const char* telemetry_phase_name(uint8_t state, uint8_t thrust);
