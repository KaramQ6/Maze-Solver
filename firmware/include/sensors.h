/**
 * @file sensors.h
 * @brief Dynamic I2C sequencer for 3x VL53L0X ToF sensors and MPU6050 gyro odometry.
 * @author MMRC26 Team
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float front_mm;
    float left_mm;
    float right_mm;
    bool wall_front;
    bool wall_left;
    bool wall_right;
} SensorDistances;

/**
 * @brief Initialize I2C bus, sequence ToF addresses via XSHUT, and calibrate MPU6050.
 * @return true if all 4 I2C devices initialized successfully.
 */
bool sensors_init(void);

/**
 * @brief Read all three ToF range sensors and update wall detection flags.
 * @param out_distances Pointer to SensorDistances struct to fill.
 */
void sensors_read_distances(SensorDistances *out_distances);

/**
 * @brief Sample MPU6050 gyro Z-axis and integrate into heading estimate.
 * Call frequently from the control loop (e.g. 100Hz - 200Hz).
 */
void sensors_update_gyro(void);

/**
 * @brief Get current estimated heading in degrees.
 * (Positive = Clockwise / Right, Negative = Counter-Clockwise / Left).
 */
float sensors_get_heading_deg(void);

/**
 * @brief Reset or set current heading to a specific angle (e.g. 0.0f).
 */
void sensors_reset_heading(float new_heading_deg);

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
