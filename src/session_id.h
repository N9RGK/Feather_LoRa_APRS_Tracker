#pragma once
#include <stdint.h>

/**
 * Initialize the session ID subsystem.
 * Reads the last session counter from flash, increments it (wrapping 99→1),
 * writes the new value back, and stores it for use in packet generation.
 *
 * Call once at startup, before any radio transmissions.
 * No GPS dependency — session ID is available immediately.
 */
void session_id_init();

/**
 * Get the current session ID as a 2-digit zero-padded string.
 * Returns a pointer to a static buffer (e.g., "01", "42", "99").
 * Valid only after session_id_init() has been called.
 */
const char* session_id_get();

/**
 * Get the current session ID as a numeric value (1-99).
 */
uint8_t session_id_get_numeric();
