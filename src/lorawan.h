/**
  * @file lorawan.h
  * @version 0.0.2
  * @date 05/10/2021
  * @author German Martin
  * @brief LMIC wrapper library for simple use in Arduino
  *
  * This library allows quick and simple use of LoRaWAN on Espressif microcontrollers with LMIC library features
  *
  */

#ifndef LORAWAN_H
#define LORAWAN_H

#include <lmic.h>
#include <hal/hal.h>
#include <functional>
#include "FS.h"

#define DEBUG_LORAWAN_LIB 1


/**
  * @brief SPI pins definition
  */
typedef struct {
    int sck = -1;
    int miso = -1;
    int mosi = -1;
    int cs = -1;
} SPI_pins_t;


/**
  * @brief Message data and metadata
  */
typedef struct {
    uint8_t data[MAX_LEN_PAYLOAD];
    uint8_t len = 0;
    uint8_t port = 1;
    bool confirmed = false;
} send_data_t;


/**
  * @brief Struct to store counters persistently
  */
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
    /**
     * @brief Set SPI pins to correspond with hardware configuration
     * @param sck SPI Clock signal
     * @param miso SPI MISO signal
     * @param mosi SPI MOSI signal
     * @param cs SPI CS signal
     */
    void set_SPI_pins (int sck, int miso, int mosi, int cs);

    /**
     * @brief Starts LoRaWan communication
     */
    void init ();

    /**
     * @brief Asks LMIC to send this data as soon as it is ready to do so
     * @param data Data buffer to be sent
     * @param len Data length
     * @param port LoRaWAN port
     * @param confirmed `True` if node requires this message to be confirmed
     * @return `True`if packet was accepted
     */
    bool send_data_inmediate (uint8_t* data, size_t len, uint8_t port = 1, bool confirmed = false);

    /**
     * @brief Do periodic tasks inside library and LMIC behind
     */
    void loop ();

    /**
     * @brief Configures a function to be called when node is joined to network using OTAA
     * @param cb Callback function
     */
    void on_joined (on_joined_cb_t cb) {
        on_joined_cb = cb;
    }

    /**
     * @brief Configures a function to be called when an uplink message has been transmitted and rx window finished
     * @param cb Callback function
     */
    void on_tx_complete (on_tx_complete_cb_t cb) {
        on_tx_complete_cb = cb;
    }

    /**
     * @brief Configures a function to be called when node receives a downlink message
     * @param cb Callback function
     */
    void on_rx_data (on_rx_data_cb_t cb) {
        on_rx_data_cb = cb;
    }
    
    /**
     * @brief Configures an already initialized filesystem to store session data. This is recommended for OTAA nodes
     * @param fs Filesystem
     */
    void set_file_system (FS* fs) {
        file_system = fs;
    }

    /**
     * @brief Returns node join status
     * @return For OTAA nodes `true` if node is joined to network and `false` otherwise. For ABP nodes it always returns `true`
     */
    bool isJoined () {
        return joined;
    }
    uint8_t getSF () {
        return LMIC.datarate;
    }

    /**
     * @brief Sets spread factor for subsequent messages.
     *
     *        It is not advised to set SF in OTAA mode unless ADR mode is disabled first
     *
     * @param sr Spread factor. Possible values are defined in `lorabase_xxYYY.h` files in `_dr_xxYYY_t` enumeration.
     */
    void set_sf (uint8_t sr) {
        s1_t pow = LMIC.adrTxPow;
        LMIC_setDrTxpow (sr, pow);
    }

    /**
     * @brief Sets ADR (Auto Data Rate) mode. ADR mode is enabled in LMIC by default
     * @param enabled True to set ADR mode enabled
     */
    void set_adr (bool enabled) {
        LMIC_setAdrMode (enabled);
    }

    /**
     * @brief Gets a string that represents spread factor mode in human readable form
     * @return Spread factor name
     */
    String getSFStr ();

    /**
     * @brief Gets current LoRa module RF power
     * @return RF power in dBm
     */
    s1_t get_power () {
        return LMIC.adrTxPow;
    }

    /**
     * @brief Sets LoRa module RF power
     * @param power RF power in dBm
     */
    void set_power (s1_t power) {
        u1_t datarate = LMIC.datarate;
        LMIC_setDrTxpow (datarate, power);
    }

private:
    SPI_pins_t spi_pins; ///< @brief SPI pin configuration
    osjob_t sendjob;    ///< @brief Message send job handler
    osjob_t initjob;    ///< @brief Initialization job handler
    send_data_t job_data;   ///< @brief Data to send and metadata
    FS* file_system = 0;    ///< @brief Pointer to filesystem used to store LoRaWAN LMIC context
    lmic_t otaa_data;   ///< @brief LMIC context for storing in filesystem
    link_counters_t link_counters;  ///< @brief Downlink and uplink message counters to be stored in filesystem
    bool joined = false;    ///< @brief Join status flag. `True` if node has joined network using OTAA or this is a APB node.

    /**
     * @brief Message sending job
     * @param j Job handler
     */
    static void do_send (osjob_t* j);

    /**
     * @brief Internal LMIC event handler
     * @param pUserData Pointer to user data. In this case this is used to point lorawan singleton object (this)
     * @param e LMIC event
     */
    static void on_event (void* pUserData, ev_t e);

    /**
     * @brief Internal LMIC downlink data handler
     * @param pUserData Pointer to user data. In this case this is used to point lorawan singleton object (this)
     * @param port LoRaWAN port
     * @param pMessage Message buffer
     * @param nMessage Message length
     */
    static void on_lmic_rx (void* pUserData, uint8_t port, const uint8_t* pMessage, size_t nMessage);
    
    on_joined_cb_t on_joined_cb = 0;    ///< @brief Callback to be executed after node is joined to network
    on_tx_complete_cb_t on_tx_complete_cb = 0;  ///< @brief Callback to be executed when transmission and rx window are finished
    on_rx_data_cb_t on_rx_data_cb = 0;  ///< @brief Callback to be executed when downlink data is received

    /**
     * @brief LMIC initialization job
     * @param j Job handler
     */
    static void init_func (osjob_t* j);

    /**
     * @brief Gets session data from file system
     * @return `True` if operation was successful
     */
    bool get_session_data ();

    /**
     * @brief Loads session data into LMIC
     */
    void set_session_data ();

    /**
     * @brief Saves session data to file system
     * @return `True` if operation was successful
     */
    bool save_session_data ();

    /**
     * @brief Save message counters to file system
     * @return `True` if operation was successful
     */
    bool save_counters ();

    /**
     * @brief Recalculates duty cycle. Currently this only clears duty cycle counters
     */
    void calculate_duty_cycle ();
};

extern LoRaWAN lorawan; ///< @brief Singleton instance

#endif // LORAWAN_H