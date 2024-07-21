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

#include <Arduino.h>
#include "FlySkyIBus_servo.h"

void FlySkyIBusServo::begin(HardwareSerial &serial) {

    // serial should opened at external with fixed conf: 115200, SERIAL_8N1
    // serial.begin(115200, SERIAL_8N1, rxPin, txPin);
    stream = (Stream*) &serial;

    state = DISCARD;

    last = millis();
    ptr = 0;
    len = 0;
    chksum = 0;
    lchksum = 0;

    last_millis = 0;
}

bool FlySkyIBusServo::loop(void) {
    while (stream->available() > 0) {

        uint32_t now = millis();
        if (now - last >= PROTOCOL_TIMEGAP) {
            // will start a new package
            state = GET_LENGTH;
        }
        last = now;

        uint8_t v = stream->read();
        switch (state) {

        case GET_LENGTH:
            // start a new package
            if (v == PROTOCOL_LENGTH) {
                // have got a good package length
                ptr = 0;
                len = v - PROTOCOL_OVERHEAD; // length of <cmd><data....>
                chksum = 0xFFFF - v;
                state = GET_DATA;
            } else {
                // have got an unkown package length
                state = DISCARD;
            }
            break;

        case GET_DATA:
            buffer[ptr++] = v;
            chksum -= v;
            if (ptr == len) {
                // have got all <cmd><data....>
                state = GET_CHKSUML;
            }
            break;

        case GET_CHKSUML:
            lchksum = v;
            state = GET_CHKSUMH;
            break;

        case GET_CHKSUMH:
            if (chksum == (v << 8) + lchksum) {
                // chksum have validated
                if (buffer[0] == PROTOCOL_COMMAND40) {
                    // buffer[0] is <cmd>
                    // parse channel data according to https://www.flyskytech.com/info_detail/18.html
                    for (int i = 0; i < 14; i++) {
                        channel[i] = ((0x0F & buffer[2 * i + 2]) << 8) | (buffer[2 * i + 1]);
                    }
                    channel[14] = ((0xF0 & buffer[ 6]) << 8) | ((0xF0 & buffer[ 4]) << 4) | (0xF0 & buffer[ 2]);
                    channel[15] = ((0xF0 & buffer[12]) << 8) | ((0xF0 & buffer[10]) << 4) | (0xF0 & buffer[ 8]);
                    channel[16] = ((0xF0 & buffer[18]) << 8) | ((0xF0 & buffer[16]) << 4) | (0xF0 & buffer[14]);
                    channel[17] = ((0xF0 & buffer[24]) << 8) | ((0xF0 & buffer[22]) << 4) | (0xF0 & buffer[20]);
                    last_millis = last; // last millis that channel data were received
                    return true;
                }
            }
            state = DISCARD;
            break;

        case DISCARD:
            break;

        default:
            break;
        }
    }
    return false;
}

uint16_t FlySkyIBusServo::get_channel(uint8_t channel_number) {
    if (channel_number < 18) {
        return channel[channel_number];
    } else {
        return 0;
    }
}

uint32_t FlySkyIBusServo::get_last_millis(void) {
    return last_millis;
}
