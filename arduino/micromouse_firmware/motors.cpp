/**
 * @file motors.cpp
 * @brief TB6612FNG 4-pin motor driver implementation using ESP32 PWM.
 * @author MMRC26 Team
 */

#include "motors.h"

#ifdef ARDUINO
#include <Arduino.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
/* Arduino ESP32 Core v3+ LEDC API */
static void pwm_write(uint8_t pin, uint8_t channel, uint32_t duty) {
    (void)channel;
    ledcWrite(pin, duty);
}
#else
/* Arduino ESP32 Core v2 LEDC API */
static void pwm_write(uint8_t pin, uint8_t channel, uint32_t duty) {
    (void)pin;
    ledcWrite(channel, duty);
}
#endif

void motors_init(void) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_MOTOR_AIN1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_AIN2, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_BIN1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_BIN2, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(0, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(2, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(3, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_AIN1, 0);
    ledcAttachPin(PIN_MOTOR_AIN2, 1);
    ledcAttachPin(PIN_MOTOR_BIN1, 2);
    ledcAttachPin(PIN_MOTOR_BIN2, 3);
#endif

    motors_stop();
}

static int apply_deadzone(int val) {
    if (val == 0) return 0;
    if (val > 0 && val < MIN_PWM_DEADZONE) return MIN_PWM_DEADZONE;
    if (val < 0 && val > -MIN_PWM_DEADZONE) return -MIN_PWM_DEADZONE;
    return val;
}

void motors_set_speed(int left_pwm, int right_pwm) {
    left_pwm = constrain(left_pwm, -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);

    left_pwm = apply_deadzone(left_pwm);
    right_pwm = apply_deadzone(right_pwm);

    /* Left Motor (AIN1, AIN2) */
    if (left_pwm > 0) {
        pwm_write(PIN_MOTOR_AIN1, 0, (uint32_t)left_pwm);
        pwm_write(PIN_MOTOR_AIN2, 1, 0);
    } else if (left_pwm < 0) {
        pwm_write(PIN_MOTOR_AIN1, 0, 0);
        pwm_write(PIN_MOTOR_AIN2, 1, (uint32_t)(-left_pwm));
    } else {
        pwm_write(PIN_MOTOR_AIN1, 0, 0);
        pwm_write(PIN_MOTOR_AIN2, 1, 0);
    }

    /* Right Motor (BIN1, BIN2) */
    if (right_pwm > 0) {
        pwm_write(PIN_MOTOR_BIN1, 2, (uint32_t)right_pwm);
        pwm_write(PIN_MOTOR_BIN2, 3, 0);
    } else if (right_pwm < 0) {
        pwm_write(PIN_MOTOR_BIN1, 2, 0);
        pwm_write(PIN_MOTOR_BIN2, 3, (uint32_t)(-right_pwm));
    } else {
        pwm_write(PIN_MOTOR_BIN1, 2, 0);
        pwm_write(PIN_MOTOR_BIN2, 3, 0);
    }
}

void motors_stop(void) {
    pwm_write(PIN_MOTOR_AIN1, 0, 0);
    pwm_write(PIN_MOTOR_AIN2, 1, 0);
    pwm_write(PIN_MOTOR_BIN1, 2, 0);
    pwm_write(PIN_MOTOR_BIN2, 3, 0);
}

void motors_brake(void) {
    pwm_write(PIN_MOTOR_AIN1, 0, 255);
    pwm_write(PIN_MOTOR_AIN2, 1, 255);
    pwm_write(PIN_MOTOR_BIN1, 2, 255);
    pwm_write(PIN_MOTOR_BIN2, 3, 255);
}

#else
/* Host test mock implementation */
void motors_init(void) {}
void motors_set_speed(int left_pwm, int right_pwm) {
    (void)left_pwm;
    (void)right_pwm;
}
void motors_stop(void) {}
void motors_brake(void) {}
#endif
