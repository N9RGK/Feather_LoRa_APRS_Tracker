#include "altimeter_rx.h"
#include "config.h"
#include "rocket_protocol.h"
#include <Arduino.h>
#include "wiring_private.h"  // pinPeripheral()

// SERCOM1 hardware UART for altimeter data
// RX: pin 11 (PA16, SERCOM1 PAD 0)
// TX: pin 10 (PA18, SERCOM1 PAD 2) — unused, satisfies SERCOM constraint
static Uart altSerial(&sercom1, ALT_RX_PIN, ALT_TX_PIN,
                      SERCOM_RX_PAD_0, UART_TX_PAD_2);

void SERCOM1_Handler() {
    altSerial.IrqHandler();
}

static AltimeterData data = {};
static uint8_t rx_buf[ROCKET_PACKET_SIZE];
static uint8_t rx_idx = 0;

void altimeter_rx_init() {
    altSerial.begin(ALTIMETER_BAUD);
    pinPeripheral(ALT_TX_PIN, PIO_SERCOM);  // pin 10 -> SERCOM1 PAD 2
    pinPeripheral(ALT_RX_PIN, PIO_SERCOM);  // pin 11 -> SERCOM1 PAD 0
}

void altimeter_rx_update() {
    while (altSerial.available() && rx_idx < ROCKET_PACKET_SIZE) {
        rx_buf[rx_idx++] = altSerial.read();
    }
    if (rx_idx >= ROCKET_PACKET_SIZE) {
        RocketPacket pkt;
        if (rocket_packet_deserialize(rx_buf, rx_idx, &pkt)) {
            data.pressure_hpa = pkt.pressure_hpa;
            data.altitude_m   = pkt.altitude_m;
            data.flight_state = pkt.state;
            data.valid        = true;
        }
        rx_idx = 0;
    }
}

const AltimeterData* altimeter_rx_get() {
    return &data;
}
