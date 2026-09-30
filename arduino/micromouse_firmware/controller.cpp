/**
 * @file controller.cpp
 * @brief Closed-loop motion controller implementation.
 * @author MMRC26 Team
 */

#include "controller.h"
#include "encoder.h"
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

bool controller_step_forward(int base_pwm) {
    motors_stop();
#if !ROBOT_HAS_ENCODERS
    SensorDistances reference;
    if (!sensors_read_distances(&reference)
        || reference.front_mm > FRONT_REFERENCE_MAX_MM
        || reference.front_mm < CELL_PITCH_MM + FRONT_WALL_STOP_MM) return false;
    const float initial_front_mm = reference.front_mm;
    float previous_travel_mm = 0.0f;
#endif
    sensors_reset_heading(sensors_get_heading_deg()); /* Reset the idle sampling gap. */
    unsigned long start_time = millis();
    unsigned long last_pid_time = millis();

    /* Encoder-based traversal replaces pure time-based approach.
     * Time ceiling kept only as a hard safety timeout. */
    unsigned long timeout_ms = FORWARD_TIMEOUT_MS;

    /* Reset encoder pulse counters for this cell */
#if ROBOT_HAS_ENCODERS
    encoder_reset();
#endif

    float last_wall_error = 0.0f;
    float last_heading_error = 0.0f;
    bool arrived = false;

    while (true) {
        if (!sensors_update_gyro()) break;
        unsigned long now = millis();

        if (now - last_pid_time >= 10) { /* 100 Hz control loop */
            float dt = (float)(now - last_pid_time) / 1000.0f;
            last_pid_time = now;

            SensorDistances dist;
            if (!sensors_read_distances(&dist)) break;

#if ROBOT_HAS_ENCODERS
            /* Front wall can confirm arrival even before the next encoder pulse. */

            /* Priority 1: Front wall stop (highest reliability — existing behavior) */
            if (dist.front_mm <= FRONT_WALL_STOP_MM) {
                arrived = encoder_has_travelled(CELL_PITCH_MM * 0.8f);
                break;
            }

            /* Priority 2: Encoder cell crossing (battery-voltage independent) */
            if (encoder_cell_crossed()) {
                arrived = true;
                break;
            }
#else
            /* Measure displacement relative to a visible front wall, not elapsed time. */
            float travel_mm = initial_front_mm - dist.front_mm;
            if (travel_mm < previous_travel_mm - CELL_ARRIVAL_TOLERANCE_MM
                || travel_mm - previous_travel_mm > 60.0f
                || travel_mm > CELL_PITCH_MM + CELL_ARRIVAL_TOLERANCE_MM) break;
            previous_travel_mm = travel_mm;
            if (fabsf(travel_mm - CELL_PITCH_MM) <= CELL_ARRIVAL_TOLERANCE_MM) {
                motors_brake();
                delay(35);
                arrived = true;
                for (int i = 0; i < 2; i++) {
                    if (!sensors_read_distances(&dist)
                        || !sensors_update_gyro()
                        || fabsf(target_heading_deg - sensors_get_heading_deg()) > 3.0f
                        || fabsf(initial_front_mm - dist.front_mm - CELL_PITCH_MM)
                           > CELL_ARRIVAL_TOLERANCE_MM) {
                        arrived = false;
                        break;
                    }
                }
                break;
            }
            if (dist.front_mm <= FRONT_WALL_STOP_MM) break;
#endif

            /* Priority 3: Hard safety timeout */
            if ((now - start_time) >= timeout_ms) {
                break;
            }

            /* 1. Heading Error PD */
            float heading_err = target_heading_deg - sensors_get_heading_deg();
            if (fabsf(heading_err) > 10.0f) break;
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
            int steer = (int)(u_heading - u_wall); /* Positive steer turns clockwise. */
            steer = constrain(steer, -65, 65);

            int drive_pwm = base_pwm;
#if !ROBOT_HAS_ENCODERS
            if (CELL_PITCH_MM - previous_travel_mm < 60.0f) {
                drive_pwm = min(base_pwm, MIN_PWM_DEADZONE + 20);
            }
#endif
            motors_set_speed(drive_pwm + steer, drive_pwm - steer);
        }

        delay(2);
    }

    /* Active Braking sequence to prevent overshoot */
    motors_brake();
    delay(35);
    motors_stop();
    delay(25);
    return arrived;
}

bool controller_turn_90(bool turn_left) {
    sensors_reset_heading(sensors_get_heading_deg());
    if (turn_left) {
        target_heading_deg -= 90.0f;
    } else {
        target_heading_deg += 90.0f;
    }

    unsigned long start_turn = millis();
    const unsigned long TIMEOUT_TURN_MS = 900;
    bool aligned = false;

    while (millis() - start_turn < TIMEOUT_TURN_MS) {
        if (!sensors_update_gyro()) break;

        float error = target_heading_deg - sensors_get_heading_deg();
        if (fabs(error) <= 1.2f) {
            aligned = true;
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
    return aligned;
}

bool controller_turn_180(void) {
    if (!controller_turn_90(false)) return false;
    delay(30);
    return controller_turn_90(false);
}

void controller_align_with_front_wall(void) {
    SensorDistances dist;
    if (!sensors_read_distances(&dist) || !dist.wall_front) return;

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

bool controller_step_forward(int base_pwm) {
    (void)base_pwm;
    return false;
}

bool controller_turn_90(bool turn_left) {
    mock_target_heading += turn_left ? -90.0f : 90.0f;
    return true;
}

bool controller_turn_180(void) {
    mock_target_heading += 180.0f;
    return true;
}

void controller_align_with_front_wall(void) {}
#endif
