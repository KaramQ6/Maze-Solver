/**
 * @file sensors.cpp
 * @brief Dynamic I2C sequencer and gyro integration implementation for ESP32.
 * @author MMRC26 Team
 */

#include "sensors.h"
#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

static VL53L0X tof_front;
static VL53L0X tof_left;
static VL53L0X tof_right;

static float current_heading_deg = 0.0f;
static float gyro_z_offset = 0.0f;
static unsigned long last_gyro_time_us = 0;

/* MPU6050 / MPU6500 shared gyro registers; identity is profile-specific. */
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_GYRO_ZOUT_H  0x47

static bool imu_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR_IMU);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool imu_read(uint8_t reg, uint8_t *data, uint8_t count) {
    Wire.beginTransmission(ADDR_IMU);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)ADDR_IMU, count) != count) return false;
    for (uint8_t i = 0; i < count; i++) data[i] = (uint8_t)Wire.read();
    return true;
}

static bool imu_read_gyro_z(int16_t *value) {
    uint8_t data[2];
    if (!imu_read(MPU_REG_GYRO_ZOUT_H, data, 2)) return false;
    *value = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    return true;
}

bool sensors_init(void) {
    /* 1. Hold controllable ToF sensors in hardware shutdown */
#if defined(PIN_XSHUT_FRONT) && (PIN_XSHUT_FRONT >= 0)
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    digitalWrite(PIN_XSHUT_FRONT, LOW);
#endif
    if (PIN_XSHUT_LEFT >= 0) {
        pinMode(PIN_XSHUT_LEFT, OUTPUT);
        digitalWrite(PIN_XSHUT_LEFT, LOW);
    }
    if (PIN_XSHUT_RIGHT >= 0) {
        pinMode(PIN_XSHUT_RIGHT, OUTPUT);
        digitalWrite(PIN_XSHUT_RIGHT, LOW);
    }
    delay(20);

    /* 2. Initialize Hardware I2C Bus on GPIO 5 (SDA) and GPIO 6 (SCL) */
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    Wire.setTimeOut(20);
    delay(50);

    uint8_t identity;
    if (!imu_read(0x75, &identity, 1) || identity != IMU_WHO_AM_I) return false;
    if (!imu_write_reg(MPU_REG_PWR_MGMT_1, 0x01)) return false; /* PLL clock */
    delay(100);
    if (!imu_write_reg(0x1A, 0x03) || !imu_write_reg(0x19, 0x04)
        || !imu_write_reg(MPU_REG_GYRO_CONFIG, 0x08)) return false; /* 200Hz, 500dps */
    delay(20);

    /* 4. Calibrate MPU6050 Z-axis zero bias */
    long sum_z = 0;
    const int CALIB_SAMPLES = 200;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        int16_t value;
        if (!imu_read_gyro_z(&value)) return false;
        sum_z += value;
        delay(5);
    }
    gyro_z_offset = (float)sum_z / (float)CALIB_SAMPLES;
    current_heading_deg = 0.0f;
    last_gyro_time_us = micros();

    /* 5. Boot and initialize Front VL53L0X */
#if defined(PIN_XSHUT_FRONT) && (PIN_XSHUT_FRONT >= 0)
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(15);
#endif
    tof_front.setTimeout(40);
    if (!tof_front.init()) return false;
    tof_front.setAddress(ADDR_TOF_FRONT);
    tof_front.startContinuous(30);

    /* 6. Boot, re-address, and initialize Left VL53L0X */
    if (PIN_XSHUT_LEFT >= 0) {
        digitalWrite(PIN_XSHUT_LEFT, HIGH);
        delay(15);
    }
    tof_left.setTimeout(40);
    if (!tof_left.init()) return false;
    tof_left.setAddress(ADDR_TOF_LEFT);
    tof_left.startContinuous(30);

    /* 7. Boot, re-address, and initialize Right VL53L0X */
    if (PIN_XSHUT_RIGHT >= 0) {
        digitalWrite(PIN_XSHUT_RIGHT, HIGH);
        delay(15);
    }
    tof_right.setTimeout(40);
    if (!tof_right.init()) return false;
    tof_right.setAddress(ADDR_TOF_RIGHT);
    tof_right.startContinuous(30);

    sensors_reset_heading(0.0f);
    return true;
}

bool sensors_read_distances(SensorDistances *out_distances) {
    if (!out_distances) return false;

    uint16_t d_front = tof_front.readRangeContinuousMillimeters();
    if (tof_front.timeoutOccurred() || tof_front.last_status != 0) return false;
    uint16_t d_left  = tof_left.readRangeContinuousMillimeters();
    if (tof_left.timeoutOccurred() || tof_left.last_status != 0) return false;
    uint16_t d_right = tof_right.readRangeContinuousMillimeters();
    if (tof_right.timeoutOccurred() || tof_right.last_status != 0) return false;

    if (d_front == 0 || d_left == 0 || d_right == 0
        || d_front > 4000 || d_left > 4000 || d_right > 4000) return false;

    out_distances->front_mm = (float)d_front;
    out_distances->left_mm  = (float)d_left;
    out_distances->right_mm = (float)d_right;

    out_distances->wall_front = (out_distances->front_mm < WALL_DETECT_THRESHOLD_MM);
    out_distances->wall_left  = (out_distances->left_mm  < WALL_DETECT_THRESHOLD_MM);
    out_distances->wall_right = (out_distances->right_mm < WALL_DETECT_THRESHOLD_MM);
    return true;
}

bool sensors_update_gyro(void) {
    unsigned long now_us = micros();
    float dt = (float)(now_us - last_gyro_time_us) / 1000000.0f;
    last_gyro_time_us = now_us;

    if (dt <= 0.0f) return true;
    if (dt > 0.1f) return false; /* Lost feedback while moving. */

    int16_t raw_z;
    if (!imu_read_gyro_z(&raw_z)) return false;
    float rate_dps = GYRO_Z_SIGN * ((float)raw_z - gyro_z_offset) / 65.5f;

    /* Deadzone filter for residual drift */
    if (fabs(rate_dps) < 0.2f) {
        rate_dps = 0.0f;
    }

    current_heading_deg += rate_dps * dt;
    return true;
}

float sensors_get_heading_deg(void) {
    return current_heading_deg;
}

void sensors_reset_heading(float new_heading_deg) {
    current_heading_deg = new_heading_deg;
    last_gyro_time_us = micros();
}

#else
/* Host test mock implementation */
static float mock_heading = 0.0f;

bool sensors_init(void) {
    mock_heading = 0.0f;
    return true;
}

bool sensors_read_distances(SensorDistances *out_distances) {
    if (!out_distances) return false;
    out_distances->front_mm = 85.0f;
    out_distances->left_mm = 75.0f;
    out_distances->right_mm = 75.0f;
    out_distances->wall_front = true;
    out_distances->wall_left = true;
    out_distances->wall_right = true;
    return true;
}

bool sensors_update_gyro(void) { return true; }

float sensors_get_heading_deg(void) {
    return mock_heading;
}

void sensors_reset_heading(float new_heading_deg) {
    mock_heading = new_heading_deg;
}
#endif
