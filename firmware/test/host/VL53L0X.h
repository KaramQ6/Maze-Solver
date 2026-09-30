#ifndef TEST_VL53L0X_H
#define TEST_VL53L0X_H

#include <stdint.h>
extern int test_next_tof;
extern bool test_tof_init_ok[3];
extern bool test_tof_timeout[3];
extern uint16_t test_tof_ranges[3];
extern uint8_t test_tof_addresses[3];

class VL53L0X {
    int id;
public:
    uint8_t last_status = 0;
    VL53L0X() : id(test_next_tof++) {}
    void setTimeout(int) {}
    bool init() { return test_tof_init_ok[id]; }
    void setAddress(uint8_t address) { test_tof_addresses[id] = address; }
    void startContinuous(int) {}
    uint16_t readRangeContinuousMillimeters() { return test_tof_ranges[id]; }
    bool timeoutOccurred() { return test_tof_timeout[id]; }
};
#endif
