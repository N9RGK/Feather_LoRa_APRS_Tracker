#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Initialize session ID subsystem. Call once in setup().
void session_id_init();

// Call every loop iteration after gps_handler_update().
// Latches session ID on first valid GPS date/time.
void session_id_update();

// Returns true if session ID has been generated (GPS fix received).
bool session_id_valid();

// Returns the raw 32-bit session ID (epoch seconds).
uint32_t session_id_get();

// Writes the 8-char hex string to buf (must be at least 9 bytes).
// Returns false if session not yet valid.
bool session_id_hex(char* buf, size_t buf_size);
