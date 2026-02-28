#pragma once

#include <Arduino.h>
#include "gps_handler.h"
#include "altimeter_rx.h"

String telemetry_build_aprs_packet(const GpsFix* fix, const AltimeterData* alt);
String telemetry_encode_aprs_position(double lat, double lng);
