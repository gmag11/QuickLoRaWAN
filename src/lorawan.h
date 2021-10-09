#ifndef LORAWAN_H
#define LORAWAN_H

#include <lmic.h>
#include <hal/hal.h>
#include <functional>
#include "FS.h"

#define DEBUG_LORAWAN_LIB 1

typedef struct {
    int sck = -1;
    int miso = -1;
    int mosi = -1;
    int cs = -1;
} SPI_pins_t;

typedef struct {
    uint8_t data[MAX_LEN_PAYLOAD];
    uint8_t len = 0;
    uint8_t port = 1;
    bool confirmed = false;
} send_data_t;

typedef struct {
    u4_t up_counter = 0;
    u4_t down_counter = 0;
} link_counters_t;

// typedef struct {
//     u4_t netid = 0;
//     devaddr_t devaddr = 0;
//     u1_t nwkKey[16];
//     u1_t artKey[16];
// } otaa_data_t;

typedef std::function<void (u4_t* netid, devaddr_t* devaddr, xref2u1_t nwkKey, xref2u1_t artKey)> on_joined_cb_t;
typedef std::function<void (bool ack)> on_tx_complete_cb_t;
typedef std::function<void (uint8_t port, const uint8_t* pMessage, size_t nMessage)> on_rx_data_cb_t;

class LoRaWAN {
public:
    void set_SPI_pins (int sck, int miso, int mosi, int cs);
    void init ();
    bool send_data_inmediate (uint8_t* data, size_t len, uint8_t port = 1, bool confirmed = false);
    void loop ();
    void on_joined (on_joined_cb_t cb) {
        on_joined_cb = cb;
    }
    void on_tx_complete (on_tx_complete_cb_t cb) {
        on_tx_complete_cb = cb;
    }
    void on_rx_data (on_rx_data_cb_t cb) {
        on_rx_data_cb = cb;
    }
    void set_file_system (FS* fs) {
        file_system = fs;
    }
    bool isJoined () {
        return joined;
    }
    uint8_t getSF () {
        return LMIC.datarate;
    }

    // It is not advised to set SF in OTAA mode as gateway will tell node to change it according signal to noise ratio
    void set_sf (uint8_t sr) {
        s1_t pow = LMIC.adrTxPow;
        LMIC_setDrTxpow (sr, pow);
    }
    void set_adr (bool enabled) {
        LMIC_setAdrMode (enabled);
    }
    String getSFStr ();
    s1_t get_power () {
        return LMIC.adrTxPow;
    }
    void set_power (s1_t power) {
        u1_t datarate = LMIC.datarate;
        LMIC_setDrTxpow (datarate, power);
    }

private:
    SPI_pins_t spi_pins;
    osjob_t sendjob;
    osjob_t initjob;
    send_data_t job_data;
    FS* file_system = 0;
    lmic_t otaa_data;
    link_counters_t link_counters;
    bool joined = false;

    static void do_send (osjob_t* j);
    static void on_event (void* pUserData, ev_t e);
    static void on_lmic_rx (void* pUserData, uint8_t port, const uint8_t* pMessage, size_t nMessage);
    //void init_func (osjob_t* j);
    on_joined_cb_t on_joined_cb = 0;
    on_tx_complete_cb_t on_tx_complete_cb = 0;
    on_rx_data_cb_t on_rx_data_cb = 0;
    static void init_func (osjob_t* j);
    bool get_session_data ();
    void set_session_data ();
    bool save_session_data ();
    bool save_counters ();
    void calculate_duty_cycle ();
};

extern LoRaWAN lorawan;

#endif // LORAWAN_H