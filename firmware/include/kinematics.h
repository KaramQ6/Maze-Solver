/**
 * @file kinematics.h
 * @brief Continuous F1 kinematics & suction fan physics calculations for embedded targets.
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdbool.h>

#define CELL_SIZE_METERS   0.18f
#define DEFAULT_V_MAX      4.0f   /* m/s */
#define DEFAULT_A_MAX      15.0f  /* m/s^2 */
#define DEFAULT_MU         0.8f   /* tire friction */
#define GRAVITY_M_S2       9.81f  /* Earth gravity */
#define TURN_RADIUS_METERS 0.09f  /* 90-degree curve radius */
#define SUCTION_DOWNFORCE_K 3.0f  /* 3G suction multiplier */

typedef struct {
    float max_velocity;
    float max_acceleration;
    float friction_coeff;
    float suction_multiplier;
} KinematicConfig;

/**
 * @brief Compute max cornering velocity limit based on centripetal grip and suction downforce.
 */
float kinematics_max_turn_velocity(const KinematicConfig *config);

/**
 * @brief Calculate physical traversal time for a straight run of N cells with trapezoidal profile.
 */
float kinematics_straight_time(const KinematicConfig *config, int num_cells);

/**
 * @brief Calculate physical traversal time for a 90-degree curve.
 */
float kinematics_turn_time(const KinematicConfig *config);

#endif /* KINEMATICS_H */
