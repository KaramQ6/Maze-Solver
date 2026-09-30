#ifndef TEST_WIRE_H
#define TEST_WIRE_H

#include <stdint.h>
extern bool test_i2c_ok;
extern uint8_t test_imu_identity;
extern int16_t test_gyro_z;

class TestWire {
    uint8_t reg = 0;
    uint8_t written = 0;
    uint8_t read_index = 0;
public:
    void begin(int, int, int) {}
    void setTimeOut(int) {}
    void beginTransmission(uint8_t) { written = 0; }
    void write(uint8_t value) { if (written++ == 0) reg = value; }
    uint8_t endTransmission(bool = true) { return test_i2c_ok ? 0 : 2; }
    uint8_t requestFrom(uint8_t, uint8_t count) {
        read_index = 0;
        return test_i2c_ok ? count : 0;
    }
    int read() {
        if (reg == 0x75) return test_imu_identity;
        return read_index++ == 0 ? (uint8_t)((uint16_t)test_gyro_z >> 8)
                                : (uint8_t)test_gyro_z;
    }
};

extern TestWire Wire;
#endif
