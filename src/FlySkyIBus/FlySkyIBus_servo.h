/*
 * Simple interface to the IBUS protocol of FlySky RC system.
 * This library reads channel data from servo port of ibus.
 * There is also library sending measurement through sensor port of ibus.
 *
 * Based on original work from:
 * https://gitlab.com/timwilkinson/FlySkyIBus
 * https://github.com/bmellink/IBusBM
 *
 * IBUS servo port protocol:
 * https://www.flyskytech.com/info_detail/18.html
 * https://www.flyskytech.com/u_file/photo/20200728/ecc9b73efd.png
 *
 */

#ifndef FlySkyIBus_servo_h
#define FlySkyIBus_servo_h

#include <inttypes.h>

class HardwareSerial;
class Stream;

class FlySkyIBusServo {

public:

    void begin(HardwareSerial& serial); // serial should opened at external with fixed conf: 115200, SERIAL_8N1
    bool loop(void); // call this at least each 1ms
    uint16_t get_channel(uint8_t channel_number); // channel_number range: 0-17
    uint32_t get_last_millis(void); // get last millis that channel data were received. can be use to detect RC failsafe.

private:

    Stream* stream = NULL;

    // packet format: <len><cmd><data....><chkl><chkh>
    static const uint8_t PROTOCOL_TIMEGAP = 3;   // packets are received about every 7ms, so use half of that as time gap.
    static const uint8_t PROTOCOL_LENGTH = 0x20; // fixed length. 0x20 means 32 bytes.
    static const uint8_t PROTOCOL_COMMAND40 = 0x40; // fixed command. 0x40 means data are servo values.
    static const uint8_t PROTOCOL_OVERHEAD = 3;  // 3 bytes are <len>, <chkl>, <chkh>.

    enum State {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};
    uint8_t state = DISCARD;

    uint32_t last = 0;
    uint8_t ptr = 0;
    uint8_t len = 0;
    uint16_t chksum = 0;
    uint8_t lchksum = 0;

    uint8_t buffer[PROTOCOL_LENGTH-PROTOCOL_OVERHEAD] = {0}; // store <cmd><data....>.
    uint16_t channel[18] = {0}; // store decoded servo values. ibus support 18 channels at most.
    uint32_t last_millis = 0; // last millis that channel data were received.

};

#endif
