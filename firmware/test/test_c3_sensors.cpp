#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "sensors.h"
#include "Wire.h"

unsigned long test_time_ms = 0;
int test_pins[32] = {};
int test_pwm[32] = {};
int test_channels[32] = {};
bool test_i2c_ok = true;
uint8_t test_imu_identity = 0x70;
int16_t test_gyro_z = 0;
int test_next_tof = 0;
bool test_tof_init_ok[3] = {true, true, true};
bool test_tof_timeout[3] = {};
uint16_t test_tof_ranges[3] = {500, 75, 75};
uint8_t test_tof_addresses[3] = {};
TestWire Wire;
void test_advance_time(unsigned long duration_ms) { test_time_ms += duration_ms; }

int main() {
    test_imu_identity = 0x68; /* A mislabeled MPU6050 must not pass as MPU6500. */
    assert(!sensors_init());
    test_imu_identity = 0x70;
    test_tof_init_ok[1] = false;
    assert(!sensors_init());
    test_tof_init_ok[1] = true;
    assert(sensors_init());
    assert(test_tof_addresses[0] == 0x32);
    assert(test_tof_addresses[1] == 0x31);
    assert(test_tof_addresses[2] == 0x30);

    test_gyro_z = -655; /* -10dps in +Z-up convention is clockwise. */
    delay(10);
    assert(sensors_update_gyro());
    assert(fabsf(sensors_get_heading_deg() - 0.1f) < 0.001f);
    test_i2c_ok = false;
    delay(10);
    assert(!sensors_update_gyro());
    assert(fabsf(sensors_get_heading_deg() - 0.1f) < 0.001f);
    test_i2c_ok = true;
    delay(101);
    assert(!sensors_update_gyro());

    SensorDistances dist;
    assert(sensors_read_distances(&dist));
    assert(!dist.wall_front && dist.wall_left && dist.wall_right);
    test_tof_timeout[0] = true;
    assert(!sensors_read_distances(&dist));
    test_tof_timeout[0] = false;
    test_tof_ranges[0] = 65535;
    assert(!sensors_read_distances(&dist));
    puts("MPU6500 identity, yaw sign, I2C fault, sampling gap, and ToF fault tests passed.");
}
