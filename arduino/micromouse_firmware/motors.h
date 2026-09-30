/**
 * @file motors.h
 * @brief Signed-speed motor interface; TB6612FNG or MX1508 selected in config.h.
 * @author MMRC26 Team
 */

#ifndef MOTORS_H
#define MOTORS_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize motor control GPIO pins and configure 20kHz PWM channels.
 */
void motors_init(void);

/**
 * @brief Drive left and right motors with signed PWM values (-255 to +255).
 * Positive = Forward, Negative = Reverse, 0 = Coast/Stop.
 * @param left_pwm Signed duty cycle for Left motor.
 * @param right_pwm Signed duty cycle for Right motor.
 */
void motors_set_speed(int left_pwm, int right_pwm);

/**
 * @brief Put both motors in coasting stop (IN1=0, IN2=0).
 */
void motors_stop(void);

/**
 * @brief Actively brake both motors (IN1=HIGH, IN2=HIGH).
 */
void motors_brake(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
