/**
 * @file kinematics.c
 * @brief Continuous F1 kinematics & suction fan physics calculations.
 */

#include "kinematics.h"
#include <math.h>

#define PI_CONST 3.14159265358979323846f

float kinematics_max_turn_velocity(const KinematicConfig *config) {
    if (!config) return 0.5f;

    /* v_turn = sqrt(mu * (1 + k_suction) * g * R) */
    float effective_g = (1.0f + config->suction_multiplier) * GRAVITY_M_S2;
    float max_v = sqrtf(config->friction_coeff * effective_g * TURN_RADIUS_METERS);
    if (max_v > config->max_velocity) {
        max_v = config->max_velocity;
    }
    return max_v;
}

float kinematics_straight_time(const KinematicConfig *config, int num_cells) {
    if (!config || num_cells <= 0) return 0.0f;

    float d = (float)num_cells * CELL_SIZE_METERS;
    float a = config->max_acceleration;
    float v_max = config->max_velocity;

    /* Distance required to accelerate from 0 to v_max: d_accel = v_max^2 / (2 * a) */
    float d_accel = (v_max * v_max) / (2.0f * a);

    if (d >= 2.0f * d_accel) {
        /* Reaches top speed: accel phase + cruise phase + decel phase */
        float t_accel = v_max / a;
        float d_cruise = d - (2.0f * d_accel);
        float t_cruise = d_cruise / v_max;
        return (2.0f * t_accel) + t_cruise;
    } else {
        /* Triangular profile: does not reach top speed */
        return 2.0f * sqrtf(d / a);
    }
}

float kinematics_turn_time(const KinematicConfig *config) {
    if (!config) return 0.2f;

    float v_turn = kinematics_max_turn_velocity(config);
    float arc_length = (PI_CONST * TURN_RADIUS_METERS) / 2.0f;
    return arc_length / v_turn;
}
