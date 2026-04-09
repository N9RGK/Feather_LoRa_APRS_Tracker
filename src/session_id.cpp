#include "session_id.h"
#include <FlashStorage.h>
#include <Arduino.h>
#include <stdio.h>

// Persistent storage for the session counter.
// FlashStorage emulates EEPROM on SAMD21 — survives power cycles,
// written to flash only when commit() is called (once per boot).
typedef struct {
    uint8_t magic;    // 0xA5 = initialized
    uint8_t counter;  // 1-99, rolling
} SessionStore;

FlashStorage(session_flash, SessionStore);

static const uint8_t MAGIC = 0xA5;
static const uint8_t SESSION_MIN = 1;
static const uint8_t SESSION_MAX = 99;

static uint8_t current_session = 0;
static char session_str[4]; // "01\0" to "99\0"

void session_id_init() {
    SessionStore store = session_flash.read();

    uint8_t last_session;
    if (store.magic != MAGIC) {
        // First boot or corrupted — initialize
        last_session = 0;
    } else {
        last_session = store.counter;
    }

    // Increment with wrap
    current_session = last_session + 1;
    if (current_session > SESSION_MAX || current_session < SESSION_MIN) {
        current_session = SESSION_MIN;
    }

    // Write back
    SessionStore new_store;
    new_store.magic = MAGIC;
    new_store.counter = current_session;
    session_flash.write(new_store);

    // Format string
    snprintf(session_str, sizeof(session_str), "%02u", current_session);

    Serial.print("Session ID: ");
    Serial.print(session_str);
    Serial.print(" (counter ");
    Serial.print(current_session);
    Serial.println(")");
}

const char* session_id_get() {
    return session_str;
}

uint8_t session_id_get_numeric() {
    return current_session;
}
