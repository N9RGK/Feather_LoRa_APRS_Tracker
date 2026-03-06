#pragma once

void lora_aprs_init();

// Send an APRS position packet (CA2RXU-compatible, with 3-byte OE header)
void lora_aprs_send();

// Send a dense telemetry packet (our proprietary format, with 3-byte OE header)
void lora_dense_send();
