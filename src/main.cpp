#include <Arduino.h>
#include "config.h"
#include "gps_handler.h"
#include "lora_aprs.h"
#include "altimeter_rx.h"

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    while (!Serial && millis() < 3000);  // wait up to 3s for USB serial
    Serial.println();
    Serial.println("=== Feather LoRa APRS Tracker ===");
    Serial.print("Callsign: ");
    Serial.println(CALLSIGN);
    Serial.print("TX interval: ");
    Serial.print(TX_INTERVAL_MS / 1000);
    Serial.println("s");

    Serial.print("GPS init... ");
    gps_handler_init();
    Serial.println("ok");

    Serial.print("Altimeter SERCOM1 init... ");
    altimeter_rx_init();
    Serial.println("ok");

    Serial.print("LoRa radio init... ");
    lora_aprs_init();
}

void loop() {
    gps_handler_update();
    altimeter_rx_update();

    // Heartbeat: brief blink every 2s to show the board is alive
    static uint32_t last_blink = 0;
    if (millis() - last_blink >= 2000) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(50);
        digitalWrite(LED_BUILTIN, LOW);
        last_blink = millis();
    }

    static uint32_t last_tx = 0;
    if (millis() - last_tx >= TX_INTERVAL_MS) {
        // Triple flash to indicate TX
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
        lora_aprs_send();
        last_tx = millis();
    }
}
