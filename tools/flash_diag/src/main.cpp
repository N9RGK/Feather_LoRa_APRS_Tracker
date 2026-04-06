#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_SPIFlash.h>

// --- Determine which flash transport the Adafruit library selects ---
#if defined(EXTERNAL_FLASH_USE_QSPI)
  #define FLASH_PATH "QSPI"
  Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
  #define FLASH_PATH "EXTERNAL_FLASH_USE_SPI (CS=" XSTR(EXTERNAL_FLASH_USE_CS) ")"
  #define FLASH_CS_PIN EXTERNAL_FLASH_USE_CS
  Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS,
                                             EXTERNAL_FLASH_USE_SPI);
#else
  #define FLASH_PATH "Fallback: SS pin"
  #define FLASH_CS_PIN SS
  Adafruit_FlashTransport_SPI flashTransport(SS, SPI);
#endif

// Stringify helper
#define XSTR(x) STR(x)
#define STR(x) #x

Adafruit_SPIFlash flash(&flashTransport);

// RFM96 pins on Feather M0
static const uint8_t RFM_CS  = 8;
static const uint8_t RFM_RST = 4;

// SPI bus pins (SAMD21 defaults)
static const uint8_t PIN_MOSI = MOSI;  // 24 on Feather M0
static const uint8_t PIN_MISO = MISO;  // 22
static const uint8_t PIN_SCK  = SCK;   // 23

static void print_divider(const char* title) {
    Serial.println();
    Serial.print("=== ");
    Serial.print(title);
    Serial.println(" ===");
}

static void print_pin_state(const char* name, uint8_t pin) {
    Serial.print("  ");
    Serial.print(name);
    Serial.print(" (pin ");
    Serial.print(pin);
    Serial.print("): ");
    Serial.println(digitalRead(pin) ? "HIGH" : "LOW");
}

// Read 3-byte JEDEC ID by bit-banging the SPI flash CS
static void manual_spi_probe(uint8_t cs_pin) {
    print_divider("Manual SPI Probe (JEDEC 0x9F)");

    Serial.print("Using CS pin: ");
    Serial.println(cs_pin);

    // Make sure radio CS is high (deselected) to avoid bus contention
    digitalWrite(RFM_CS, HIGH);

    // Ensure SPI is running
    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    digitalWrite(cs_pin, LOW);
    delayMicroseconds(10);

    SPI.transfer(0x9F);  // JEDEC Read ID command
    uint8_t mfr   = SPI.transfer(0x00);
    uint8_t type  = SPI.transfer(0x00);
    uint8_t cap   = SPI.transfer(0x00);

    digitalWrite(cs_pin, HIGH);
    SPI.endTransaction();

    Serial.print("  Raw bytes: 0x");
    if (mfr < 0x10) Serial.print("0");
    Serial.print(mfr, HEX);
    Serial.print(" 0x");
    if (type < 0x10) Serial.print("0");
    Serial.print(type, HEX);
    Serial.print(" 0x");
    if (cap < 0x10) Serial.print("0");
    Serial.println(cap, HEX);

    if (mfr == 0xFF && type == 0xFF && cap == 0xFF) {
        Serial.println("  Result: All 0xFF — no chip responding (bus floating)");
    } else if (mfr == 0x00 && type == 0x00 && cap == 0x00) {
        Serial.println("  Result: All 0x00 — bus stuck low or chip held in reset");
    } else {
        Serial.print("  Result: Got a response! MFR=0x");
        Serial.print(mfr, HEX);
        Serial.print(" TYPE=0x");
        Serial.print(type, HEX);
        Serial.print(" CAP=0x");
        Serial.println(cap, HEX);

        // Decode common manufacturers
        switch (mfr) {
            case 0xEF: Serial.println("  Manufacturer: Winbond"); break;
            case 0xC8: Serial.println("  Manufacturer: GigaDevice"); break;
            case 0x01: Serial.println("  Manufacturer: Cypress/Spansion"); break;
            case 0x20: Serial.println("  Manufacturer: Micron/Numonyx"); break;
            case 0xBF: Serial.println("  Manufacturer: Microchip/SST"); break;
            case 0x1F: Serial.println("  Manufacturer: Adesto/Atmel"); break;
            default:   Serial.println("  Manufacturer: Unknown"); break;
        }

        if (cap > 0 && cap < 0x20) {
            uint32_t size = 1UL << cap;
            Serial.print("  Capacity field suggests: ");
            Serial.print(size / 1024);
            Serial.println(" KB");
        }
    }
}

// Probe the RFM96 radio via SPI to confirm bus is working
static void probe_rfm96() {
    print_divider("RFM96 Radio Probe");

    // Reset the radio cleanly
    pinMode(RFM_RST, OUTPUT);
    digitalWrite(RFM_RST, LOW);
    delay(10);
    digitalWrite(RFM_RST, HIGH);
    delay(10);

    pinMode(RFM_CS, OUTPUT);
    digitalWrite(RFM_CS, HIGH);

    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    // Read RFM96 version register (0x42) — expect 0x12 for SX1276/RFM96
    digitalWrite(RFM_CS, LOW);
    delayMicroseconds(10);
    SPI.transfer(0x42 & 0x7F);  // Read register 0x42 (MSB=0 for read)
    uint8_t version = SPI.transfer(0x00);
    digitalWrite(RFM_CS, HIGH);

    SPI.endTransaction();

    Serial.print("  RFM96 CS pin: ");
    Serial.println(RFM_CS);
    Serial.print("  Version register (0x42): 0x");
    if (version < 0x10) Serial.print("0");
    Serial.println(version, HEX);

    if (version == 0x12) {
        Serial.println("  Result: SX1276/RFM96 detected — SPI bus is working");
    } else if (version == 0xFF || version == 0x00) {
        Serial.println("  Result: No response — check radio wiring/CS pin");
    } else {
        Serial.print("  Result: Unexpected version 0x");
        Serial.print(version, HEX);
        Serial.println(" — might be a different radio chip");
    }
}

static void print_pin_states() {
    print_divider("Pin States");

    print_pin_state("MOSI", PIN_MOSI);
    print_pin_state("MISO", PIN_MISO);
    print_pin_state("SCK ", PIN_SCK);

#ifdef FLASH_CS_PIN
    print_pin_state("Flash CS", FLASH_CS_PIN);
#endif
    print_pin_state("Radio CS", RFM_CS);
    print_pin_state("Radio RST", RFM_RST);

    Serial.print("  SS macro = pin ");
    Serial.println(SS);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);  // Wait for USB serial
    delay(500);

    Serial.println();
    Serial.println("========================================");
    Serial.println("  Feather M0 SPI Flash Diagnostic");
    Serial.println("========================================");

    // --- Flash transport path ---
    print_divider("Flash Transport Selection");
    Serial.print("  #if chain selected: ");
    Serial.println(FLASH_PATH);
#ifdef FLASH_CS_PIN
    Serial.print("  Flash CS pin: ");
    Serial.println(FLASH_CS_PIN);
#endif
    Serial.print("  SS pin: ");
    Serial.println(SS);

    // Deselect radio before touching flash
    pinMode(RFM_CS, OUTPUT);
    digitalWrite(RFM_CS, HIGH);

    // --- Adafruit SPIFlash probe ---
    print_divider("Adafruit SPIFlash Library Probe");
    Serial.print("  flash.begin(): ");
    bool ok = flash.begin();
    Serial.println(ok ? "SUCCESS" : "FAILED");

    if (ok) {
        Serial.print("  JEDEC ID: 0x");
        Serial.println(flash.getJEDECID(), HEX);
        Serial.print("  Flash size: ");
        Serial.print(flash.size() / 1024);
        Serial.println(" KB");
        Serial.print("  Page size: ");
        Serial.println(flash.pageSize());
    } else {
        Serial.println("  Library could not detect a flash chip");
    }

    // --- Manual SPI probe ---
#ifdef FLASH_CS_PIN
    manual_spi_probe(FLASH_CS_PIN);
#else
    manual_spi_probe(SS);
#endif

    // --- RFM96 probe ---
    probe_rfm96();

    // --- Pin states ---
    print_pin_states();

    print_divider("Done");
    Serial.println("Diagnostic complete. Reset board to run again.");
}

void loop() {
    // Nothing — one-shot diagnostic
}
