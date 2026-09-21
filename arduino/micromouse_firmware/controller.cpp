/**
 * @file controller.cpp
 * @brief Closed-loop motion controller implementation.
 * @author MMRC26 Team
 */

#include "controller.h"
#include <math.h>

#ifdef ARDUINO
#include <Arduino.h>

static float target_heading_deg = 0.0f;

void controller_init(void) {
    target_heading_deg = 0.0f;
    sensors_reset_heading(0.0f);
}

float controller_get_target_heading(void) {
    return target_heading_deg;
}

void controller_step_forward(bool wall_ahead, int base_pwm) {
    unsigned long start_time = millis();
    unsigned long last_pid_time = millis();

    /* Calibrated traversal time for 180mm at different PWM rates */
    unsigned long cell_duration_ms = (base_pwm > 180) ? 360 : 520;
    unsigned long timeout_ms = cell_duration_ms + 250; /* Safety ceiling */

    float last_wall_error = 0.0f;
    float last_heading_error = 0.0f;

    while (true) {
        sensors_update_gyro();
        unsigned long now = millis();

        if (now - last_pid_time >= 10) { /* 100 Hz control loop */
            float dt = (float)(now - last_pid_time) / 1000.0f;
            last_pid_time = now;

            SensorDistances dist;
            sensors_read_distances(&dist);

            /* Check termination condition */
            if (wall_ahead) {
                if (dist.front_mm <= FRONT_WALL_STOP_MM || (now - start_time) >= timeout_ms) {
                    break;
                }
            } else {
                if ((now - start_time) >= cell_duration_ms) {
                    break;
                }
            }

            /* 1. Heading Error PID */
            float heading_err = target_heading_deg - sensors_get_heading_deg();
            float heading_d = (dt > 0.0f) ? (heading_err - last_heading_error) / dt : 0.0f;
            last_heading_error = heading_err;

            float u_heading = (KP_HEADING * heading_err) + (KD_HEADING * heading_d);

            /* 2. Wall Centering Error PD */
            float wall_err = 0.0f;
            if (dist.wall_left && dist.wall_right) {
                wall_err = (dist.left_mm - dist.right_mm);
            } else if (dist.wall_left) {
                wall_err = (dist.left_mm - WALL_TARGET_SIDE_MM) * 2.0f;
            } else if (dist.wall_right) {
                wall_err = (WALL_TARGET_SIDE_MM - dist.right_mm) * 2.0f;
            }

            float wall_d = (dt > 0.0f) ? (wall_err - last_wall_error) / dt : 0.0f;
            last_wall_error = wall_err;

            float u_wall = (KP_WALL * (wall_err * 0.1f)) + (KD_WALL * (wall_d * 0.1f));

            /* 3. Combined Steering Command */
            int steer = (int)(u_heading + u_wall);
            steer = constrain(steer, -65, 65);

            motors_set_speed(base_pwm - steer, base_pwm + steer);
        }

        delay(2);
    }

    /* Active Braking sequence to prevent overshoot */
    motors_brake();
    delay(35);
    motors_stop();
    delay(25);
}

void controller_turn_90(bool turn_left) {
    if (turn_left) {
        target_heading_deg -= 90.0f;
    } else {
        target_heading_deg += 90.0f;
    }

    unsigned long start_turn = millis();
    const unsigned long TIMEOUT_TURN_MS = 900;

    while (millis() - start_turn < TIMEOUT_TURN_MS) {
        sensors_update_gyro();

        float error = target_heading_deg - sensors_get_heading_deg();
        if (fabs(error) <= 1.2f) {
            break; /* Reached target angle within tolerance */
        }

        int turn_pwm = (int)(fabs(error) * 2.8f);
        turn_pwm = constrain(turn_pwm, MIN_PWM_DEADZONE + 25, TURN_BASE_PWM);

        if (error > 0.0f) {
            /* Rotate Clockwise (Right) */
            motors_set_speed(turn_pwm, -turn_pwm);
        } else {
            /* Rotate Counter-Clockwise (Left) */
            motors_set_speed(-turn_pwm, turn_pwm);
        }

        delay(5);
    }

    /* Active Braking and settling */
    motors_brake();
    delay(40);
    motors_stop();
    delay(40);
}

void controller_turn_180(void) {
    controller_turn_90(false);
    delay(30);
    controller_turn_90(false);
}

void controller_align_with_front_wall(void) {
    SensorDistances dist;
    sensors_read_distances(&dist);
    if (!dist.wall_front) return;

    float diff = dist.front_mm - FRONT_WALL_STOP_MM;
    if (fabs(diff) > 8.0f && fabs(diff) < 40.0f) {
        int nudge_pwm = (diff > 0) ? (MIN_PWM_DEADZONE + 15) : -(MIN_PWM_DEADZONE + 15);
        motors_set_speed(nudge_pwm, nudge_pwm);
        delay(30);
        motors_brake();
        delay(20);
        motors_stop();
    }
}

#else
/* Host test mock implementation */
static float mock_target_heading = 0.0f;

void controller_init(void) {
    mock_target_heading = 0.0f;
}

float controller_get_target_heading(void) {
    return mock_target_heading;
}

void controller_step_forward(bool wall_ahead, int base_pwm) {
    (void)wall_ahead;
    (void)base_pwm;
}

void controller_turn_90(bool turn_left) {
    mock_target_heading += turn_left ? -90.0f : 90.0f;
}

void controller_turn_180(void) {
    mock_target_heading += 180.0f;
}

void controller_align_with_front_wall(void) {}
#endif
