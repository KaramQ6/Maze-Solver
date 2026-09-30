/**
 * @file config.h
 * @brief Hardware profiles and calibration constants for the micromouse.
 * @author MMRC26 Team
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#if defined(ROBOT_ESP32_C3) || defined(CONFIG_IDF_TARGET_ESP32C3)
/* ESP32-C3 SuperMini + TB6612FNG. Motor pins confirmed from the wiring photo.
 * I2C and XSHUT pins are the proposed wiring documented in docs/HARDWARE_C3.md. */
#define MOTOR_DRIVER_TB6612         1
#define ROBOT_HAS_ENCODERS          0
#define PIN_MOTOR_IN1               10  /* AIN1: left motor */
#define PIN_MOTOR_IN2               20  /* AIN2 */
#define PIN_MOTOR_IN3               8   /* BIN1: right motor; also board LED/strap */
#define PIN_MOTOR_IN4               7   /* BIN2 */
#define PIN_MOTOR_PWMA              21
#define PIN_MOTOR_PWMB              6
/* STBY is wired directly to 3.3V, not to a GPIO. */
#define PIN_I2C_SDA                 4
#define PIN_I2C_SCL                 5
#define PIN_XSHUT_FRONT             3
#define PIN_XSHUT_LEFT              0
#define PIN_XSHUT_RIGHT             1
#define PIN_BUTTON_START            9   /* Onboard BOOT, press only after boot */
#define PIN_LED_STATUS              -1  /* GPIO8 is already used by BIN1 */
#define IMU_WHO_AM_I                0x70 /* MPU6500 */
#define GYRO_Z_SIGN                 -1.0f /* Flat mount, +Z up: clockwise positive */
#else
#define MOTOR_DRIVER_TB6612         0
#define ROBOT_HAS_ENCODERS          1
#define IMU_WHO_AM_I                0x68 /* MPU6050 */
#define GYRO_Z_SIGN                 1.0f

/* MX1508 Dual H-Bridge DC Motor Driver (Header Left Side: D0 - D3) */
#define PIN_MOTOR_IN1               1   /* D0 (GPIO 1) - Left Motor IN1 (LEDC PWM) */
#define PIN_MOTOR_IN2               2   /* D1 (GPIO 2) - Left Motor IN2 (LEDC PWM) */
#define PIN_MOTOR_IN3               3   /* D2 (GPIO 3) - Right Motor IN3 (LEDC PWM) */
#define PIN_MOTOR_IN4               4   /* D3 (GPIO 4) - Right Motor IN4 (LEDC PWM) */

/* I2C Hardware Bus (Header Left Side: D4, D5 - Shared by MPU6050 & VL53L0X) */
#define PIN_I2C_SDA                 5   /* D4 (GPIO 5) - I2C SDA */
#define PIN_I2C_SCL                 6   /* D5 (GPIO 6) - I2C SCL */

/* Hall-Effect Magnetic Encoders (Header Pins: D6, D7) */
#define PIN_ENCODER_LEFT            43  /* D6 (GPIO 43) - Left wheel Hall sensor (INPUT_PULLUP) */
#define PIN_ENCODER_RIGHT           44  /* D7 (GPIO 44) - Right wheel Hall sensor (INPUT_PULLUP) */

/* XSHUT Lines for 3x VL53L0X Laser Distance Sensors */
#define PIN_XSHUT_FRONT             -1  /* Front ToF: Tied to 3.3V (Always ON), or set to 9 (D10) if wired */
#define PIN_XSHUT_RIGHT             7   /* D8 (GPIO 7) - Right ToF shutdown line */
#define PIN_XSHUT_LEFT              8   /* D9 (GPIO 8) - Left ToF shutdown line */

/* Operator Controls & Diagnostics */
#define PIN_BUTTON_START            0   /* Onboard BOOT Button (GPIO 0, active LOW, INPUT_PULLUP) */
#define PIN_LED_STATUS              9   /* D10 (GPIO 9) - Optional External Status LED / Buzzer (-1 if unused) */
#endif

/* Change a sign to -1 if that wheel spins backward for a positive command. */
#define LEFT_MOTOR_SIGN             1
#define RIGHT_MOTOR_SIGN            1

/* ========================================================================== */
/* I2C BUS ADDRESSES                                                          */
/* ========================================================================== */

#define ADDR_TOF_FRONT              0x32 /* Front laser sensor (readdressed at boot from 0x29) */
#define ADDR_TOF_RIGHT              0x30 /* Right laser sensor (readdressed via XSHUT) */
#define ADDR_TOF_LEFT               0x31 /* Left laser sensor (readdressed via XSHUT) */
#define ADDR_IMU                    0x68 /* AD0 to GND; MPU6500 CS to 3.3V */

/* ========================================================================== */
/* MAZE PHYSICAL SPECIFICATIONS (MMRC26)                                      */
/* ========================================================================== */

#define CELL_SIZE_MM                180.0f
#define WALL_THICKNESS_MM            12.0f
#define CELL_PITCH_MM               (CELL_SIZE_MM + WALL_THICKNESS_MM)
#define WALL_DETECT_THRESHOLD_MM    135.0f /* Sensor < 135mm indicates a wall is present */
#define WALL_TARGET_SIDE_MM          75.0f /* Nominal distance to side wall when centered */
#define FRONT_WALL_STOP_MM           85.0f /* Target front distance at cell center */
#define FRONT_REFERENCE_MAX_MM     1200.0f /* Validate against actual wall reflectivity */
#define CELL_ARRIVAL_TOLERANCE_MM     10.0f
#define FORWARD_TIMEOUT_MS          2000

/* ========================================================================== */
/* MOTION & CONTROL TUNING                                                    */
/* ========================================================================== */

#define PWM_FREQUENCY_HZ            20000  /* 20kHz ultrasonic silent PWM */
#define PWM_RESOLUTION_BITS         8      /* 0 - 255 */

#define SEARCH_BASE_PWM             130    /* Exploration speed (smooth for 500 RPM) */
#define SPEEDRUN_BASE_PWM           210    /* Maximum reliable sprint speed */
#define TURN_BASE_PWM               115    /* In-place pivot turn speed */
#define MIN_PWM_DEADZONE             45    /* Minimum PWM to overcome N20 gear static friction */

#define KP_HEADING                  2.50f  /* Heading proportional gain */
#define KD_HEADING                  0.12f  /* Heading derivative gain */

#define KP_WALL                     1.20f  /* Wall centering proportional gain */
#define KD_WALL                     0.06f  /* Wall centering derivative gain */

#define GYRO_DRIFT_COMP_MS          2000   /* 2-second initial stationary calibration */
#define GYRO_FS_SEL_DPS             500.0f /* +/- 500 deg/s gyro full scale */

#endif /* CONFIG_H */
