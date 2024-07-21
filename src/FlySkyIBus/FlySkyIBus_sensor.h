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

#ifndef FlySkyIBus_sensor_h
#define FlySkyIBus_sensor_h

#include <inttypes.h>

#define IBUS_SENSOR_TYPE_TEM            0x01 // Temperature
#define IBUS_SENSOR_TYPE_RPM_FLYSKY     0x02 // RPM
#define IBUS_SENSOR_TYPE_EXTV           0x03 // External voltage
#define IBUS_SENSOR_TYPE_CELL           0x04 // Avg cell voltage
#define IBUS_SENSOR_TYPE_BAT_CURR       0x05 // battery current A * 100
#define IBUS_SENSOR_TYPE_FUEL           0x06 // remaining battery percentage / mah drawn otherwise or fuel level no unit!
#define IBUS_SENSOR_TYPE_RPM            0x07 // throttle value / battery capacity
#define IBUS_SENSOR_TYPE_CMP_HEAD       0x08 // Heading 0..360 deg, 0=north 2bytes
#define IBUS_SENSOR_TYPE_CLIMB_RATE     0x09 // Climb rate, 2 bytes m/s *100
#define IBUS_SENSOR_TYPE_COG            0x0a // Course over ground, 2 bytes, (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
#define IBUS_SENSOR_TYPE_GPS_STATUS     0x0b // GPS status, 2 bytes
#define IBUS_SENSOR_TYPE_ACC_X          0x0c // Acc X, 2 bytes m/s *100 signed
#define IBUS_SENSOR_TYPE_ACC_Y          0x0d // Acc Y, 2 bytes m/s *100 signed
#define IBUS_SENSOR_TYPE_ACC_Z          0x0e // Acc Z, 2 bytes m/s *100 signed
#define IBUS_SENSOR_TYPE_ROLL           0x0f // Roll, 2 bytes deg *100 signed
#define IBUS_SENSOR_TYPE_PITCH          0x10 // Pitch, 2 bytes deg *100 signed
#define IBUS_SENSOR_TYPE_YAW            0x11 // Yaw, 2 bytes deg *100 signed
#define IBUS_SENSOR_TYPE_VERTICAL_SPEED 0x12 // Vertical speed, 2 bytes m/s *100
#define IBUS_SENSOR_TYPE_GROUND_SPEED   0x13 // Speed m/s, 2 bytes m/s *100 different unit than build-in sensor
#define IBUS_SENSOR_TYPE_GPS_DIST       0x14 // Distance from home, 2 bytes dist from home m unsigned
#define IBUS_SENSOR_TYPE_ARMED          0x15 // Armed / unarmed, 2 bytes
#define IBUS_SENSOR_TYPE_FLIGHT_MODE    0x16 // Flight mode, 2 bytes
#define IBUS_SENSOR_TYPE_PRES           0x41 // Pressure
#define IBUS_SENSOR_TYPE_ODO1           0x7c // Odometer1
#define IBUS_SENSOR_TYPE_ODO2           0x7d // Odometer2
#define IBUS_SENSOR_TYPE_SPE            0x7e // Speed km/h, 2 bytes

class HardwareSerial;
class Stream;

class FlySkyIBusSensor {

public:

    void begin(HardwareSerial& serial); // serial should opened at external with fixed conf: 115200, SERIAL_8N1
    bool loop(void); // call this at least each 1ms

    // About max number of sensors:
    // There are 16 possible addresses in iBus,
    // however address 0 is reserved for the rx's internally measured voltage,
    // leaving 15 sensors remaining for user.
    uint8_t add_sensor(uint8_t type, uint8_t len, int32_t value); // add sensor type (as defined above) and data length (2 or 4), returns address (addr is from 1 to number_sensors (max is 15))
    void set_sensor_measurement(uint8_t addr, int32_t value); // addr is from 1 to number_sensors (max is 15)
    uint32_t get_last_millis(void); // get last millis that sensor data were send. can be use to detect RC failsafe.

private:

    Stream* stream = NULL;

    // packet format: <len><cmd><data....><chkl><chkh>
    static const uint8_t PROTOCOL_TIMEGAP = 3;   // packets are received about every 7ms, so use half of that as time gap.
    static const uint8_t PROTOCOL_LENGTH = 0x04; // fixed length. 0x04 means 4 bytes (actually, there is only <cmd> and no <data>)
    static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80; // Command discover sensor (lowest 4 bits are addr/sensor number)
    static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;     // Command set sensor type (lowest 4 bits are addr/sensor number)
    static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;    // Command send sensor data (lowest 4 bits are addr/sensor number)
    static const uint8_t PROTOCOL_OVERHEAD = 3;  // 3 bytes are <len>, <chkl>, <chkh>

    enum State {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};
    uint8_t state = DISCARD;

    uint32_t last = 0;
    uint8_t ptr = 0;
    uint8_t len = 0;
    uint16_t chksum = 0;
    uint8_t lchksum = 0;

    uint8_t buffer[PROTOCOL_LENGTH-PROTOCOL_OVERHEAD] = {0}; // store <cmd><data....>
    uint32_t last_millis = 0; // last millis that sensor data were send.

    typedef struct {
        uint8_t sensor_type = 0;             // sensor type (0,1,2,3, etc)
        uint8_t sensor_length = 0;           // data length for defined sensor (can be 2 or 4)
        int32_t sensor_value = 0;            // sensor data for defined sensors (16 or 32 bits)
    } SensorInfo;
    SensorInfo sensors[15];              // max number of user sensors is 15
	
    uint8_t number_sensors = 0;          // number of sensors

};

#endif
