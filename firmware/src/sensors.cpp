/**
 * @file sensors.cpp
 * @brief Dynamic I2C sequencer and gyro integration implementation for ESP32.
 * @author MMRC26 Team
 */

#include "sensors.h"

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

/* MPU6050 Registers */
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_GYRO_ZOUT_H  0x47

static bool mpu6050_write_reg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static int16_t mpu6050_read_gyro_z(void) {
    Wire.beginTransmission(ADDR_MPU6050);
    Wire.write(MPU_REG_GYRO_ZOUT_H);
    if (Wire.endTransmission(false) != 0) return 0;
    
    if (Wire.requestFrom((uint8_t)ADDR_MPU6050, (uint8_t)2) == 2) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        return (int16_t)((hi << 8) | lo);
    }
    return 0;
}

bool sensors_init(void) {
    /* 1. Hold all 3 ToF sensors in hardware shutdown */
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    pinMode(PIN_XSHUT_LEFT, OUTPUT);
    pinMode(PIN_XSHUT_RIGHT, OUTPUT);
    digitalWrite(PIN_XSHUT_FRONT, LOW);
    digitalWrite(PIN_XSHUT_LEFT, LOW);
    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(20);

    /* 2. Initialize Hardware I2C Bus on GPIO 21 (SDA) and GPIO 22 (SCL) */
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);
    delay(50);

    /* 3. Initialize MPU6050 Gyro */
    mpu6050_write_reg(MPU_REG_PWR_MGMT_1, 0x00);  /* Wake up device */
    delay(10);
    mpu6050_write_reg(MPU_REG_GYRO_CONFIG, 0x08); /* +/- 500 deg/s range (65.5 LSB/(deg/s)) */
    delay(20);

    /* 4. Calibrate MPU6050 Z-axis zero bias */
    long sum_z = 0;
    const int CALIB_SAMPLES = 200;
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        sum_z += mpu6050_read_gyro_z();
        delay(5);
    }
    gyro_z_offset = (float)sum_z / (float)CALIB_SAMPLES;
    current_heading_deg = 0.0f;
    last_gyro_time_us = micros();

    /* 5. Boot and initialize Front VL53L0X (Address 0x29) */
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(15);
    tof_front.setAddress(ADDR_TOF_FRONT);
    tof_front.setTimeout(500);
    if (tof_front.init()) {
        tof_front.startContinuous(30);
    }

    /* 6. Boot, re-address, and initialize Left VL53L0X (Address 0x30) */
    digitalWrite(PIN_XSHUT_LEFT, HIGH);
    delay(15);
    tof_left.setAddress(ADDR_TOF_LEFT);
    tof_left.setTimeout(500);
    if (tof_left.init()) {
        tof_left.startContinuous(30);
    }

    /* 7. Boot, re-address, and initialize Right VL53L0X (Address 0x31) */
    digitalWrite(PIN_XSHUT_RIGHT, HIGH);
    delay(15);
    tof_right.setAddress(ADDR_TOF_RIGHT);
    tof_right.setTimeout(500);
    if (tof_right.init()) {
        tof_right.startContinuous(30);
    }

    return true;
}

void sensors_read_distances(SensorDistances *out_distances) {
    if (!out_distances) return;

    uint16_t d_front = tof_front.readRangeContinuousMillimeters();
    uint16_t d_left  = tof_left.readRangeContinuousMillimeters();
    uint16_t d_right = tof_right.readRangeContinuousMillimeters();

    /* Clamp out-of-range/error values to safe max */
    out_distances->front_mm = (tof_front.timeoutOccurred() || d_front > 800) ? 800.0f : (float)d_front;
    out_distances->left_mm  = (tof_left.timeoutOccurred()  || d_left  > 800) ? 800.0f : (float)d_left;
    out_distances->right_mm = (tof_right.timeoutOccurred() || d_right > 800) ? 800.0f : (float)d_right;

    out_distances->wall_front = (out_distances->front_mm < WALL_DETECT_THRESHOLD_MM);
    out_distances->wall_left  = (out_distances->left_mm  < WALL_DETECT_THRESHOLD_MM);
    out_distances->wall_right = (out_distances->right_mm < WALL_DETECT_THRESHOLD_MM);
}

void sensors_update_gyro(void) {
    unsigned long now_us = micros();
    float dt = (float)(now_us - last_gyro_time_us) / 1000000.0f;
    last_gyro_time_us = now_us;

    if (dt <= 0.0f || dt > 0.1f) {
        return;
    }

    int16_t raw_z = mpu6050_read_gyro_z();
    float rate_dps = ((float)raw_z - gyro_z_offset) / 65.5f;

    /* Deadzone filter for residual drift */
    if (fabs(rate_dps) < 0.2f) {
        rate_dps = 0.0f;
    }

    current_heading_deg += rate_dps * dt;
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

void sensors_read_distances(SensorDistances *out_distances) {
    if (!out_distances) return;
    out_distances->front_mm = 85.0f;
    out_distances->left_mm = 75.0f;
    out_distances->right_mm = 75.0f;
    out_distances->wall_front = true;
    out_distances->wall_left = true;
    out_distances->wall_right = true;
}

void sensors_update_gyro(void) {}

float sensors_get_heading_deg(void) {
    return mock_heading;
}

void sensors_reset_heading(float new_heading_deg) {
    mock_heading = new_heading_deg;
}
#endif
