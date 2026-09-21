/**
 * @file controller.h
 * @brief Closed-loop motion controller using sensorless gyro heading PID and wall centering.
 * @author MMRC26 Team
 */

#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "config.h"
#include "sensors.h"
#include "motors.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize motion controller state and set initial target heading.
 */
void controller_init(void);

/**
 * @brief Move forward exactly one 18cm maze unit cell with dual PID stabilization.
 * @param wall_ahead true if a front wall is known to block the destination cell.
 * @param base_pwm Nominal speed duty cycle (e.g. SEARCH_BASE_PWM or SPEEDRUN_BASE_PWM).
 */
void controller_step_forward(bool wall_ahead, int base_pwm);

/**
 * @brief Execute a precise in-place 90-degree pivot turn using gyro angular feedback.
 * @param turn_left true for 90-degree counter-clockwise, false for 90-degree clockwise.
 */
void controller_turn_90(bool turn_left);

/**
 * @brief Execute an in-place 180-degree turnaround.
 */
void controller_turn_180(void);

/**
 * @brief Fine-tune alignment at cell stop using front wall distance if present.
 */
void controller_align_with_front_wall(void);

/**
 * @brief Return current controller target heading in degrees.
 */
float controller_get_target_heading(void);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */
