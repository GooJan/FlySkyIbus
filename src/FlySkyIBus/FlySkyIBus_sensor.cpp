/*
 * Simple interface to the IBUS protocol of FlySky RC system.
 * This library sends measurement through sensor port of ibus.
 * There is also library reading channel data from servo port of ibus.
 *
 * Based on original work from:
 * https://github.com/bmellink/IBusBM
 *
 * IBUS sensor port protocol:
 * https://betaflight.com/docs/wiki/guides/current/ibus-telemetry
 * https://static.rcgroups.net/forums/attachments/1/1/0/6/2/5/a14850329-56-iBus-telemetry.png
 * https://github.com/cleanflight/cleanflight/blob/master/src/main/telemetry/ibus_shared.h
 *
 */

#include <Arduino.h>
#include "FlySkyIBus_sensor.h"

void FlySkyIBusSensor::begin(HardwareSerial &serial) {

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
    number_sensors = 0;
}

bool FlySkyIBusSensor::loop(void) {
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
                // have got a package from flysky receiver, which is just 4 bytes
                ptr = 0;
                len = v - PROTOCOL_OVERHEAD;
                chksum = 0xFFFF - v;
                state = GET_DATA;
            } else {
                // Ibus Telemetry is a half-duplex serial protocol. It shares 1 line for both TX and RX.
                // PROTOCOL_TIMEGAP and PROTOCOL_LENGTH prevent to get messages from the UART TX port loop back to the RX port.
                state = DISCARD;
            }
            break;

        case GET_DATA:
            buffer[ptr++] = v;
            chksum -= v;
            if (ptr == len) {
                // have got all <cmd><data....> (actually there is only <cmd> but no <data>)
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

                uint8_t cmd = buffer[0] & 0xf0;
                uint8_t adr = buffer[0] & 0x0f;

                if (0<adr<=number_sensors) {
                    SensorInfo *s = &sensors[adr-1];

                    switch (cmd) {
                    case PROTOCOL_COMMAND_DISCOVER: // discover sensor: 0x04, 0x81, 0x7A, 0xFF
                        // echo discover command: 0x04, 0x81, 0x7A, 0xFF
                        delayMicroseconds(100);
                        stream->write(0x04);
                        stream->write(PROTOCOL_COMMAND_DISCOVER + adr);
                        chksum = 0xFFFF - (0x04 + PROTOCOL_COMMAND_DISCOVER + adr);
                        break;
                    case PROTOCOL_COMMAND_TYPE: // ask sensor type: 0x04, 0x91, 0x6A, 0xFF
                        // echo sensortype command: 0x06 0x91 0x00 0x02 0x66 0xFF
                        delayMicroseconds(100);
                        stream->write(0x06);
                        stream->write(PROTOCOL_COMMAND_TYPE + adr);
                        stream->write(s->sensor_type);
                        stream->write(s->sensor_length);
                        chksum = 0xFFFF - (0x06 + PROTOCOL_COMMAND_TYPE + adr + s->sensor_type + s->sensor_length);
                        break;
                    case PROTOCOL_COMMAND_VALUE: // ask sensor measurement: 0x04, 0xA1, 0x5A, 0xFF
                        // echo sensor value command: 0x06 0x91 0x00 0x02 0x66 0xFF
                        last_millis = last; // last millis that sensor data were send
                        uint8_t t;
                        delayMicroseconds(100);
                        stream->write(t = 0x04 + s->sensor_length);
                        chksum = 0xFFFF - t;
                        stream->write(t = PROTOCOL_COMMAND_VALUE + adr);
                        chksum -= t;
                        stream->write(t = s->sensor_value & 0x0ff);
                        chksum -= t;
                        stream->write(t = (s->sensor_value >> 8) & 0x0ff);
                        chksum -= t;
                        if (s->sensor_length==4) {
                            stream->write(t = (s->sensor_value >> 16) & 0x0ff);
                            chksum -= t;
                            stream->write(t = (s->sensor_value >> 24) & 0x0ff);
                            chksum -= t;
                        }
                        break;
                    default:
                        adr=0; // unknown command, prevent sending chksum
                        break;
                    }
                    if (adr>0) {
                        stream->write(chksum & 0x0ff);
                        stream->write(chksum >> 8);
                    }
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

uint8_t FlySkyIBusSensor::add_sensor(uint8_t type, uint8_t len, int32_t value) {
    // add sensor type (as defined above) and data length (2 or 4), returns address (addr is from 1 to number_sensors (max is 15))
    if (len!=2 && len!=4) len = 2;
    if (number_sensors < 15) {
        SensorInfo *s = &sensors[number_sensors];
        s->sensor_type = type;
        s->sensor_length = len;
        s->sensor_value = value;
        number_sensors++;
    }
    return number_sensors;
}

void FlySkyIBusSensor::set_sensor_measurement(uint8_t addr, int32_t value) {
    // addr is from 1 to number_sensors (max is 15)
    if (0<addr<=number_sensors)
        sensors[addr-1].sensor_value = value;
}

uint32_t FlySkyIBusSensor::get_last_millis(void) {
    return last_millis;
}
