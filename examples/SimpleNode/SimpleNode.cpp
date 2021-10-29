
#include <Arduino.h>
#include <SPI.h>
#include "lorawan.h"
#ifdef ESP8266
#include <LittleFS.h>
#define FILESYSTEM  LittleFS
#else
#include <LITTLEFS.h>
#define FILESYSTEM  LITTLEFS
#endif

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)

#if __has_include("keys.h")
#include "keys.h" // You can create a fille named keys.h with key difinitions for further privacy
#else
// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { FILLMEIN };
// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { FILLMEIN };
// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };
#endif

void os_getArtEui (u1_t* buf) { memcpy_P (buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P (buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P (buf, APPKEY, 16); }

static uint8_t mydata[] = "Hello world!!!";

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

const lmic_pinmap lmic_pins = {
    .nss = 16,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,                       // reset pin
    .dio = {5, 4, LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO1 is on JP1-1: is io1 - we connect to GPO6
                                    // DIO1 is on JP5-3: is D2 - we connect to GPO5
};

void on_join (u4_t* net_id, devaddr_t* dev_addr, xref2u1_t nwk_key, xref2u1_t art_key) {
    Serial.print ("Joined\n");
}

void on_rx (uint8_t port, const uint8_t* pMessage, size_t nMessage) {
    Serial.printf ("<------ Got data. Port: %u Length: %u\n", port, nMessage);
}

void setup () {
    Serial.begin (115200);

    FILESYSTEM.begin ();
    lorawan.set_SPI_pins (14, 12, 13, 16);
    lorawan.set_file_system (&FILESYSTEM);
    lorawan.init ();
    lorawan.on_joined (on_join);
    lorawan.on_rx_data (on_rx);
}

constexpr auto PERIOD = 30000;

void loop () {
    static time_t last_sent = 0;

    if (millis () - last_sent > PERIOD && lorawan.isJoined()) {
        last_sent = millis ();
        lorawan.send_data_inmediate ((uint8_t*)mydata, sizeof (uint8_t), 1);
        Serial.println ("Send");
    }
    lorawan.loop ();
}

