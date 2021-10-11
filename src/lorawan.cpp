#include <Arduino.h>
#include <SPI.h>
#include "lorawan.h"

#ifndef DEBUG_PORT
#define DEBUG_PORT Serial ///< @brief Stream to output debug info. It will normally be `Serial`
#endif // DEBUG_ESP_PORT

#if defined DEBUG_PORT && DEBUG_LORAWAN_LIB

#ifdef ESP8266

const char* IRAM_ATTR extractFileName (const char* path) {
    size_t i = 0;
    size_t pos = 0;
    char* p = (char*)path;
    while (*p) {
        i++;
        if (*p == '/' || *p == '\\') {
            pos = i;
        }
        p++;
    }
    return path + pos;
}

#define DEBUG_LINE_PREFIX() DEBUG_PORT.printf_P (PSTR("[%lu][H:%5lu][%s:%d] %s() | "),millis(),(unsigned long)ESP.getFreeHeap(),extractFileName(__FILE__),__LINE__,__FUNCTION__)
#define DEBUG_LORAWAN(text,...) DEBUG_LINE_PREFIX();DEBUG_PORT.printf_P(PSTR(text),##__VA_ARGS__)

#elif defined ESP32

#define DEFAULT_LOG_TAG "EnigmaIOT"
#define DEBUG_LORAWAN(format,...) ESP_LOGD (DEFAULT_LOG_TAG,"%d Heap: %6d. " format, millis(), ESP.getFreeHeap(), ##__VA_ARGS__)

#else

#define DEBUG_LORAWAN(...)

#endif // ESP8266

#else

#define DEBUG_LORAWAN(...)

#endif // DEBUG_PORT


LoRaWAN lorawan;




auto constexpr CONFIG_FILE = "loraconfig.cfg";
auto constexpr COUNTERS_FILE = "loracounters.cfg";

void LoRaWAN::set_SPI_pins (int sck, int miso, int mosi, int cs) {
    spi_pins.sck = sck;
    spi_pins.miso = miso;
    spi_pins.mosi = mosi;
    spi_pins.cs = cs;

#if defined ESP8266
    SPI.pins (spi_pins.sck, spi_pins.miso, spi_pins.mosi, spi_pins.cs);
#elif defined ESP32
    SPI.begin (spi_pins.sck, spi_pins.miso, spi_pins.mosi, spi_pins.cs);
#endif
}

#if defined DEBUG_PORT && DEBUG_LORAWAN_LIB
void printHex2 (unsigned v) {
    v &= 0xff;
    if (v < 16)
        DEBUG_PORT.print ('0');
    DEBUG_PORT.print (v, HEX);
}
#endif

void LoRaWAN::on_lmic_rx (void* pUserData, uint8_t port, const uint8_t* pMessage, size_t nMessage) {
    LoRaWAN* instance = (LoRaWAN*)pUserData;

#if defined DEBUG_PORT && DEBUG_LORAWAN_LIB
    DEBUG_LORAWAN ("<------ Got data. Port: %u Length: %u --> ", port, nMessage);
    for (size_t i = 0; i < nMessage; ++i) {
        if (i != 0)
            DEBUG_PORT.print ("-");
        printHex2 (pMessage[i]);
    }
    DEBUG_PORT.println ();
#endif
    DEBUG_LORAWAN ("RX_DATA_CB\n");
    if (instance->on_rx_data_cb) {
        instance->on_rx_data_cb (port, pMessage, nMessage);
    }
}

void LoRaWAN::on_event (void* pUserData, ev_t e) {
    LoRaWAN* instance = (LoRaWAN*)pUserData;
    bool ack;
    
    switch (e) {
    case EV_SCAN_TIMEOUT:
        DEBUG_LORAWAN ("EV_SCAN_TIMEOUT\n");
        break;
    case EV_BEACON_FOUND:
        DEBUG_LORAWAN ("EV_BEACON_FOUND\n");
        break;
    case EV_BEACON_MISSED:
        DEBUG_LORAWAN ("EV_BEACON_MISSED\n");
        break;
    case EV_BEACON_TRACKED:
        DEBUG_LORAWAN ("EV_BEACON_TRACKED\n");
        break;
    case EV_JOINING:
        DEBUG_LORAWAN ("EV_JOINING\n");
        break;
    case EV_JOINED:
        DEBUG_LORAWAN ("EV_JOINED\n");
        {
            instance->joined = true;
            LMIC_getSessionKeys (
                &instance->otaa_data.netid,
                &instance->otaa_data.devaddr,
                instance->otaa_data.nwkKey,
                instance->otaa_data.artKey);
            instance->link_counters.up_counter = LMIC.seqnoUp;
            instance->link_counters.down_counter = LMIC.seqnoDn;
#if defined DEBUG_PORT && DEBUG_LORAWAN_LIB
            if (instance->save_session_data ()) {
                DEBUG_LORAWAN ("Joined. Saved session keys\n");
            }
            DEBUG_LORAWAN ("netid: %d\n", &instance->otaa_data.netid);
            DEBUG_LORAWAN ("devaddr: 0x%X\n", &instance->otaa_data.devaddr);
            DEBUG_LORAWAN ("AppSKey: ");
            for (size_t i = 0; i < sizeof (instance->otaa_data.artKey); ++i) {
                if (i != 0)
                    DEBUG_PORT.print ("-");
                printHex2 (instance->otaa_data.artKey[i]);
            }
            DEBUG_PORT.println ("");
            DEBUG_LORAWAN ("NwkSKey: ");
            for (size_t i = 0; i < sizeof (instance->otaa_data.nwkKey); ++i) {
                if (i != 0)
                    DEBUG_PORT.print ("-");
                printHex2 (instance->otaa_data.nwkKey[i]);
            }
            DEBUG_PORT.println ();
#endif
            if (instance->on_joined_cb) {
                instance->on_joined_cb (
                    &instance->otaa_data.netid,
                    &instance->otaa_data.devaddr,
                    instance->otaa_data.nwkKey,
                    instance->otaa_data.artKey);
            }
        }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
    // size, we don't use it in this example.
        LMIC_setLinkCheckMode (0);
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_RFU1:
    ||     DEBUG_LORAWAN ("EV_RFU1\n");
    ||     break;
    */
    case EV_JOIN_FAILED:
        DEBUG_LORAWAN ("EV_JOIN_FAILED\n");
        instance->joined = false;
        break;
    case EV_REJOIN_FAILED:
        DEBUG_LORAWAN ("EV_REJOIN_FAILED\n");
        instance->joined = false;
        break;
    case EV_TXCOMPLETE:
        DEBUG_LORAWAN ("EV_TXCOMPLETE (includes waiting for RX windows)\n");
        ack = false;
        instance->save_counters ();
        if (LMIC.txrxFlags & TXRX_ACK) {
            DEBUG_LORAWAN ("Received ack\n");
            ack = true;
        }
        // if (LMIC.dataLen) {
        //     // Data was received. Extract port number if any.
        //     DEBUG_LORAWAN ("Got data\n");
        //     u1_t bPort = 0;
        //     if (LMIC.txrxFlags & TXRX_PORT) {
        //         bPort = LMIC.frame[LMIC.dataBeg - 1];
        //     }
        //     DEBUG_LORAWAN ("<------ Got data. Port: %u Length: %u\n", bPort, LMIC.dataLen);
        //     u1_t *data = &(LMIC.frame[LMIC.dataBeg]);
        //     for (size_t i = 0; i < LMIC.dataLen; ++i) {
        //         if (i != 0) {
        //             DEBUG_PORT.print ("-");
        //         }
        //         printHex2 (data[i]);
        //     }
        //     DEBUG_PORT.println ();
        //     // Call user-supplied function with port #, pMessage, nMessage;
        //     // nMessage might be zero.
        //     DEBUG_LORAWAN ("RX_CALLBACK\n");
        //     if (instance->on_rx_data_cb) {
        //         instance->on_rx_data_cb (bPort, data, LMIC.dataLen);
        //     }
        // }
        DEBUG_LORAWAN ("TX_COMPLETE_CALLBACK\n");
        if (instance->on_tx_complete_cb) {
            instance->on_tx_complete_cb (ack);
        }
        // Schedule next transmission
        //os_setTimedCallback (&sendjob, os_getTime () + sec2osticks (TX_INTERVAL), do_send);
    break;
    case EV_LOST_TSYNC:
        DEBUG_LORAWAN ("EV_LOST_TSYNC\n");
        break;
    case EV_RESET:
        DEBUG_LORAWAN ("EV_RESET\n");
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        DEBUG_LORAWAN ("EV_RXCOMPLETE\n");
        break;
    case EV_LINK_DEAD:
        DEBUG_LORAWAN ("EV_LINK_DEAD\n");
        instance->joined = false;
        break;
    case EV_LINK_ALIVE:
        DEBUG_LORAWAN ("EV_LINK_ALIVE\n");
        break;
    /*
    || This event is defined but not used in the code. No
    || point in wasting codespace on it.
    ||
    || case EV_SCAN_FOUND:
    ||    DEBUG_LORAWAN ("EV_SCAN_FOUND\n"));
    ||    break;
    */
    case EV_TXSTART:
        DEBUG_LORAWAN ("EV_TXSTART\n");
        break;
    case EV_TXCANCELED:
        DEBUG_LORAWAN ("EV_TXCANCELED\n");
        break;
    case EV_RXSTART:
        /* do not print anything -- it wrecks timing */
        break;
    case EV_JOIN_TXCOMPLETE:
        DEBUG_LORAWAN ("EV_JOIN_TXCOMPLETE: no JoinAccept\n");
        break;

    default:
        DEBUG_LORAWAN ("Unknown event: %u\n", (unsigned)e);
        break;
    }
}

void LoRaWAN::init_func (osjob_t* j) {
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset ();
    lorawan.set_session_data ();
    LMIC_startJoining ();
    DEBUG_LORAWAN ("Init func\n");
}

void LoRaWAN::set_session_data () {
    //LMIC_setSession (otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
    if (otaa_data.devaddr != 0) {
        joined = true;
        otaa_data.client = LMIC.client; // Need to restore pointer so that they do not break after code editing
        otaa_data.osjob = LMIC.osjob;

        LMIC = otaa_data;

        LMIC.seqnoUp = link_counters.up_counter;
        LMIC.seqnoDn = link_counters.down_counter;
    }
}

bool LoRaWAN::get_session_data () {
    File configFile;
    File countersFile;

    if (!file_system) {
        DEBUG_LORAWAN ("No FS present\n");
        return false;
    }
    // if (!file_system->begin ()) {
    //     DEBUG_LORAWAN ("Error starting FS\n");
    //     return false;
    // }
    if (!file_system->exists (CONFIG_FILE) || !file_system->exists (COUNTERS_FILE)) {
        DEBUG_LORAWAN ("Cannot find config file\n");
        return false;
    }
    configFile = file_system->open (CONFIG_FILE, "r");
    countersFile = file_system->open (COUNTERS_FILE, "r");
    if (!configFile || !countersFile) {
        DEBUG_LORAWAN ("Error opening config file\n");
        return false;
    }
    size_t file_size = configFile.size ();
    if (file_size != sizeof (otaa_data)) {
        DEBUG_LORAWAN ("Wrong config file size: %u bytes. Should be %u\n", file_size, sizeof (otaa_data));
        configFile.close ();
        return false;
    }
    file_size = countersFile.size ();
    if (file_size != sizeof (link_counters_t)) {
        DEBUG_LORAWAN ("Wrong counters file size: %u bytes. Should be %u\n", file_size, sizeof (link_counters_t));
        configFile.close ();
        return false;
    }

    size_t bytes_read = configFile.readBytes ((char*)&otaa_data, sizeof (otaa_data));
    configFile.close ();

    if (bytes_read != sizeof (otaa_data)) {
        DEBUG_LORAWAN ("Wrong config data length: %u bytes. Should be %u\n", bytes_read, sizeof (otaa_data));
        return false;
    }

    bytes_read = countersFile.readBytes ((char*)&link_counters, sizeof (link_counters_t));
    countersFile.close ();

    if (bytes_read != sizeof (link_counters_t)) {
        DEBUG_LORAWAN ("Wrong counters data length: %u bytes. Should be %u\n", bytes_read, sizeof (link_counters_t));
        return false;
    }

#if defined DEBUG_PORT && DEBUG_LORAWAN_LIB
    DEBUG_LORAWAN ("------------------\n");
    DEBUG_LORAWAN ("Config file read\n");
    DEBUG_LORAWAN ("netid: %d\n", otaa_data.netid);
    DEBUG_LORAWAN ("devaddr: 0x%X\n", otaa_data.devaddr);
    DEBUG_LORAWAN ("AppSKey: ");
    for (size_t i = 0; i < 16; ++i) {
        if (i != 0)
            DEBUG_PORT.print ("-");
        printHex2 (otaa_data.artKey[i]);
    }
    DEBUG_PORT.println ("");
    DEBUG_LORAWAN ("NwkSKey: ");
    for (size_t i = 0; i < 16; ++i) {
        if (i != 0)
            DEBUG_PORT.print ("-");
        printHex2 (otaa_data.nwkKey[i]);
    }
    DEBUG_PORT.println ();
    DEBUG_LORAWAN ("Up counter: %u\n", link_counters.up_counter);
    DEBUG_LORAWAN ("Down counter: %u\n", link_counters.down_counter);
    DEBUG_LORAWAN ("------------------\n");
#endif
    
    return true;
}

bool LoRaWAN::save_counters () {
    File countersFile;
    if (!file_system) {
        DEBUG_LORAWAN ("No FS present\n");
        return false;
    }
    // if (!file_system->begin ()) {
    //     DEBUG_LORAWAN ("Error starting FS\n");
    //     return false;
    // }
    countersFile = file_system->open (COUNTERS_FILE, "w");
    if (!countersFile) {
        DEBUG_LORAWAN ("Error opening counters file\n");
        return false;
    }
    link_counters.up_counter = LMIC.seqnoUp;
    link_counters.down_counter = LMIC.seqnoDn;

    size_t bytes_written = countersFile.write ((uint8_t*)&link_counters, sizeof (link_counters));
    countersFile.flush ();
    countersFile.close ();

    if (bytes_written != sizeof (link_counters)) {
        DEBUG_LORAWAN ("Wrong file size: %u bytes. Should be %u\n", bytes_written, sizeof (link_counters));
        return false;
    } else {
        DEBUG_LORAWAN ("------------------------\n");
        DEBUG_LORAWAN ("Counters file written: %u bytes\n", bytes_written);
        DEBUG_LORAWAN ("Up counter: %u\n", link_counters.up_counter);
        DEBUG_LORAWAN ("Down counter: %u\n", link_counters.down_counter);
        DEBUG_LORAWAN ("------------------------\n");
    }

    return true;

    return true;
}

void LoRaWAN::calculate_duty_cycle () {
    // unsigned long now = millis ();
// #if defined(CFG_LMIC_EU_like)
//     for (int i = 0; i < MAX_BANDS; i++) {
//         ostime_t correctedAvail = otaa_data.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
//         if (correctedAvail < 0) {
//             correctedAvail = 0;
//         }
//         otaa_data.bands[i].avail = correctedAvail;
//     }

//     otaa_data.globalDutyAvail = otaa_data.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
//     if (otaa_data.globalDutyAvail < 0) {
//         otaa_data.globalDutyAvail = 0;
//     }
// #else
//     DEBUG_LORAWAN ("No DutyCycle recalculation function!\n")
// #endif
#if CFG_LMIC_EU_like
    for (int i = 0; i < MAX_BANDS; i++) {
        otaa_data.bands[i].avail = 0;
    }
#endif
    otaa_data.globalDutyAvail = 0;
}

bool LoRaWAN::save_session_data () {
    File configFile;

    if (!file_system) {
        DEBUG_LORAWAN ("No FS present\n");
        return false;
    }
    // if (!file_system->begin ()) {
    //     DEBUG_LORAWAN ("Error starting FS\n");
    //     return false;
    // }
    configFile = file_system->open (CONFIG_FILE, "w");
    if (!configFile) {
        DEBUG_LORAWAN ("Error opening config file\n");
        return false;
    }
    otaa_data = LMIC;
    calculate_duty_cycle ();
    
    size_t bytes_written = configFile.write ((uint8_t*)&otaa_data, sizeof (otaa_data));
    if (bytes_written != sizeof (otaa_data)) {
        DEBUG_LORAWAN ("Wrong file size: %u bytes. Should be %u\n", bytes_written, sizeof (otaa_data));
        configFile.close ();
        return false;
    } else {
#if defined DEBUG_PORT && DEBUG_LORAWAN_LIB
        DEBUG_LORAWAN ("------------------------\n");
        DEBUG_LORAWAN ("Config file written: %u bytes\n", bytes_written);
        DEBUG_LORAWAN ("netid: %d\n", otaa_data.netid);
        DEBUG_LORAWAN ("devaddr: 0x%X\n", otaa_data.devaddr);
        DEBUG_LORAWAN ("AppSKey: ");
        for (size_t i = 0; i < 16; ++i) {
            if (i != 0)
                DEBUG_PORT.print ("-");
            printHex2 (otaa_data.artKey[i]);
        }
        DEBUG_PORT.println ("");
        DEBUG_LORAWAN ("NwkSKey: ");
        for (size_t i = 0; i < 16; ++i) {
            if (i != 0)
                DEBUG_PORT.print ("-");
            printHex2 (otaa_data.nwkKey[i]);
        }
        DEBUG_PORT.println ();
        DEBUG_LORAWAN ("------------------------\n");
#endif
    }
    configFile.flush ();
    configFile.close ();

    return true;
}

void LoRaWAN::init () {
    // LMIC init
    os_init ();
    LMIC_registerEventCb (on_event, this);
    LMIC_registerRxMessageCb (on_lmic_rx, this);
    os_setCallback (&initjob, init_func);
    
    if (get_session_data ()) {
        //LMIC_setSession (otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
        //LMIC_setSeqnoUp (link_counters.up_counter);
        //LMIC.seqnoDn = link_counters.down_counter;
        DEBUG_LORAWAN ("Got session keys from file\n");
    }
}

void LoRaWAN::do_send (osjob_t* j) {
    //lmic_tx_error_t result = LMIC_ERROR_TX_FAILED;

    if (LMIC.opmode & OP_TXRXPEND) {
        DEBUG_LORAWAN ("OP_TXRXPEND, not sending\n");
    } else {
        // Prepare upstream data transmission at the next possible time.
        /*result = */LMIC_setTxData2 (lorawan.job_data.port, lorawan.job_data.data, lorawan.job_data.len, lorawan.job_data.confirmed);
        DEBUG_LORAWAN ("Packet queued\n");
    }
}

bool LoRaWAN::send_data_inmediate (uint8_t* data, size_t len, uint8_t port, bool confirmed) {
    if (len > MAX_LEN_PAYLOAD) {
        len = MAX_LEN_PAYLOAD;
    }

    memcpy (job_data.data, data, len);
    job_data.len = len;
    job_data.port = port;
    job_data.confirmed = confirmed;

    os_setCallback (&sendjob, do_send);

    return true;
}

void LoRaWAN::loop () {
    os_runloop_once ();
}

// bool LoRaWAN::send_data (uint8_t* data, size_t len, uint8_t port = 1, bool confirmed = false) {
//     lmic_tx_error_t result = LMIC_ERROR_TX_FAILED;

//     // Check if there is not a current TX/RX job running
//     if (LMIC.opmode & OP_TXRXPEND) {
//         DEBUG_LORAWAN ("OP_TXRXPEND, not sending\n");
//     } else {
//         // Prepare upstream data transmission at the next possible time.
//         result = LMIC_setTxData2 (port, data, len, confirmed);
//         DEBUG_LORAWAN ("Packet queued\n");
//     }
//     // Next TX is scheduled after TX_COMPLETE event.
//     return (result == LMIC_ERROR_SUCCESS);

//     sendjob.
// }

String LoRaWAN::getSFStr () {
    String sf;
    switch (LMIC.datarate) {
#if defined(CFG_eu868)
    case EU868_DR_SF12:
        sf = "SF12";
        break;
    case EU868_DR_SF11:
        sf = "SF11";
        break;
    case EU868_DR_SF10:
        sf = "SF10";
        break;
    case EU868_DR_SF9:
        sf = "SF9";
        break;
    case EU868_DR_SF8:
        sf = "SF8";
        break;
    case EU868_DR_SF7:
        sf = "SF7";
        break;
    case EU868_DR_SF7B:
        sf = "SF7B";
        break;
    case EU868_DR_FSK:
        sf = "FSK";
        break;
    case EU868_DR_NONE:
        sf = "NONE";
        break;
#elif defined(CFG_us915) 
    case US915_DR_SF10:
        sf = "SF10";
        break;
    case US915_DR_SF9:
        sf = "SF9";
        break;
    case US915_DR_SF8:
        sf = "SF8";
        break;
    case US915_DR_SF7:
        sf = "SF7";
        break;
    case US915_DR_SF8C:
        sf = "SF8C";
        break;
    case US915_DR_NONE:
        sf = "NONE";
        break;
    case US915_DR_SF12CR:
        sf = "SF12CR";
        break;
    case US915_DR_SF11CR:
        sf = "SF11CR";
        break;
    case US915_DR_SF10CR:
        sf = "SF10CR";
        break;
    case US915_DR_SF9CR:
        sf = "SF9CR";
        break;
    case US915_DR_SF8CR:
        sf = "SF8CR";
        break;
    case US915_DR_SF7CR:
        sf = "SF7CR";
        break;
#elif defined(CFG_au915)
    case AU915_DR_SF12:
        sf = "SF12";
        break;
    case AU915_DR_SF11:
        sf = "SF11";
        break;
    case AU915_DR_SF10:
        sf = "SF10";
        break;
    case AU915_DR_SF9:
        sf = "SF9";
        break;
    case AU915_DR_SF8:
        sf = "SF8";
        break;
    case AU915_DR_SF7:
        sf = "SF7";
        break;
    case AU915_DR_SF8C:
        sf = "SF8C";
        break;
    case AU915_DR_NONE:
        sf = "NONE";
        break;
    case AU915_DR_SF12CR:
        sf = "SF12CR";
        break;
    case AU915_DR_SF11CR:
        sf = "SF11CR";
        break;
    case AU915_DR_SF10CR:
        sf = "SF10CR";
        break;
    case AU915_DR_SF9CR:
        sf = "SF9CR";
        break;
    case AU915_DR_SF8CR:
        sf = "SF8CR";
        break;
    case AU915_DR_SF7CR:
        sf = "SF7CR";
        break;
#elif defined(CFG_as923)
    case AS923_DR_SF12:
        sf = "SF12";
        break;
    case AS923_DR_SF11:
        sf = "SF11";
        break;
    case AS923_DR_SF10:
        sf = "SF10";
        break;
    case AS923_DR_SF9:
        sf = "SF9";
        break;
    case AS923_DR_SF8:
        sf = "SF8";
        break;
    case AS923_DR_SF7:
        sf = "SF7";
        break;
    case AS923_DR_SF7B:
        sf = "SF7C";
        break;
    case AS923_DR_FSK:
        sf = "FSK";
        break;
    case AS923_DR_NONE:
        sf = "NONE";
        break;
#elif defined(CFG_kr920)
    case KR920_DR_SF12:
        sf = "SF12";
        break;
    case KR920_DR_SF11:
        sf = "SF11";
        break;
    case KR920_DR_SF10:
        sf = "SF10";
        break;
    case KR920_DR_SF9:
        sf = "SF9";
        break;
    case KR920_DR_SF8:
        sf = "SF8";
        break;
    case KR920_DR_SF7:
        sf = "SF7";
        break;
    case KR920_DR_NONE:
        sf = "NONE";
        break;
#elif defined(CFG_in866)
    case IN866_DR_SF12:
        sf = "SF12";
        break;
    case IN866_DR_SF11:
        sf = "SF11";
        break;
    case IN866_DR_SF10:
        sf = "SF10";
        break;
    case IN866_DR_SF9:
        sf = "SF9";
        break;
    case IN866_DR_SF8:
        sf = "SF8";
        break;
    case IN866_DR_SF7:
        sf = "SF7";
        break;
    case IN866_DR_RFU:
        sf = "RFU";
        break;
    case IN866_DR_FSK:
        sf = "FSK";
        break;
    case IN866_DR_NONE:
        sf = "NONE";
        break;
#endif
    default:
        sf = "Unknown";
    }
    return sf;
}
