/**
 * @file config.h
 * @brief Hardware pin mapping and calibration constants for ESP32 Dev Module (ESP-WROOM-32 / 2BB77-ESP32-32X).
 * @author MMRC26 Team
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

/* ========================================================================== */
/* PIN ALLOCATION (ESP32-WROOM-32 / esp32dev)                                 */
/* ========================================================================== */

/* I2C Hardware Bus (Default ESP32 Wire pins) */
#define PIN_I2C_SDA                 21  /* Dedicated hardware I2C SDA */
#define PIN_I2C_SCL                 22  /* Dedicated hardware I2C SCL */

/* XSHUT Lines for 3x VL53L0X Laser Sensors */
#define PIN_XSHUT_FRONT             18  /* Front ToF shutdown line */
#define PIN_XSHUT_LEFT              19  /* Left ToF shutdown line */
#define PIN_XSHUT_RIGHT             23  /* Right ToF shutdown line */

/* TB6612FNG Dual DC Motor Driver */
#define PIN_MOTOR_AIN1              16  /* Left Motor Phase 1 */
#define PIN_MOTOR_AIN2              17  /* Left Motor Phase 2 */
#define PIN_MOTOR_BIN1              26  /* Right Motor Phase 1 */
#define PIN_MOTOR_BIN2              27  /* Right Motor Phase 2 */

/* Operator Controls & Diagnostics */
#define PIN_BUTTON_START            13  /* Start pushbutton (active low, internal pullup) */
#define PIN_LED_STATUS              2   /* Onboard Blue LED on ESP32 DevKit */

/* ========================================================================== */
/* I2C BUS ADDRESSES                                                          */
/* ========================================================================== */

#define ADDR_TOF_FRONT              0x29 /* Front laser distance sensor */
#define ADDR_TOF_LEFT               0x30 /* Left laser distance sensor */
#define ADDR_TOF_RIGHT              0x31 /* Right laser distance sensor */
#define ADDR_MPU6050                0x68 /* 6-DOF IMU gyroscope/accelerometer */

/* ========================================================================== */
/* MAZE PHYSICAL SPECIFICATIONS (MMRC26)                                      */
/* ========================================================================== */

#define CELL_SIZE_MM                180.0f
#define WALL_THICKNESS_MM            12.0f
#define WALL_DETECT_THRESHOLD_MM    135.0f /* Sensor < 135mm indicates a wall is present */
#define WALL_TARGET_SIDE_MM          75.0f /* Nominal distance to side wall when centered */
#define FRONT_WALL_STOP_MM           85.0f /* Target front distance at cell center */

/* ========================================================================== */
/* MOTION & CONTROL TUNING                                                    */
/* ========================================================================== */

#define PWM_FREQUENCY_HZ            20000  /* 20kHz ultrasonic silent PWM */
#define PWM_RESOLUTION_BITS         8      /* 0 - 255 */

#define SEARCH_BASE_PWM             140    /* Exploration speed */
#define SPEEDRUN_BASE_PWM           215    /* Maximum reliable sprint speed */
#define TURN_BASE_PWM               125    /* In-place pivot turn speed */
#define MIN_PWM_DEADZONE             50    /* Minimum PWM to overcome N20 gear static friction */

#define KP_HEADING                  2.50f  /* Heading proportional gain */
#define KD_HEADING                  0.12f  /* Heading derivative gain */

#define KP_WALL                     1.20f  /* Wall centering proportional gain */
#define KD_WALL                     0.06f  /* Wall centering derivative gain */

#define GYRO_DRIFT_COMP_MS          2000   /* 2-second initial stationary calibration */
#define GYRO_FS_SEL_DPS             500.0f /* +/- 500 deg/s gyro full scale */

#endif /* CONFIG_H */
