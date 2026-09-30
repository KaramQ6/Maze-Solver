/**
 * @file motors.cpp
 * @brief TB6612FNG or MX1508 motor driver selected by the hardware profile.
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
#if MOTOR_DRIVER_TB6612
    const uint8_t direction_pins[] = {PIN_MOTOR_IN1, PIN_MOTOR_IN2, PIN_MOTOR_IN3, PIN_MOTOR_IN4};
    for (uint8_t pin : direction_pins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_MOTOR_PWMA, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_PWMB, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(0, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_PWMA, 0);
    ledcAttachPin(PIN_MOTOR_PWMB, 1);
#endif
#else
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_MOTOR_IN1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_IN2, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_IN3, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttach(PIN_MOTOR_IN4, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
#else
    ledcSetup(0, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(1, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(2, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcSetup(3, PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_IN1, 0);
    ledcAttachPin(PIN_MOTOR_IN2, 1);
    ledcAttachPin(PIN_MOTOR_IN3, 2);
    ledcAttachPin(PIN_MOTOR_IN4, 3);
#endif
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
    left_pwm *= LEFT_MOTOR_SIGN;
    right_pwm *= RIGHT_MOTOR_SIGN;
    left_pwm = constrain(left_pwm, -255, 255);
    right_pwm = constrain(right_pwm, -255, 255);

    left_pwm = apply_deadzone(left_pwm);
    right_pwm = apply_deadzone(right_pwm);

#if MOTOR_DRIVER_TB6612
    /* Disable PWM before changing direction; PWM LOW means short brake on TB6612. */
    pwm_write(PIN_MOTOR_PWMA, 0, 0);
    pwm_write(PIN_MOTOR_PWMB, 1, 0);
    digitalWrite(PIN_MOTOR_IN1, left_pwm > 0 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_IN2, left_pwm < 0 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_IN3, right_pwm > 0 ? HIGH : LOW);
    digitalWrite(PIN_MOTOR_IN4, right_pwm < 0 ? HIGH : LOW);
    pwm_write(PIN_MOTOR_PWMA, 0, left_pwm == 0 ? 255 : (uint32_t)abs(left_pwm));
    pwm_write(PIN_MOTOR_PWMB, 1, right_pwm == 0 ? 255 : (uint32_t)abs(right_pwm));
#else
    /* Left Motor: IN1, IN2 */
    if (left_pwm > 0) {
        pwm_write(PIN_MOTOR_IN1, 0, (uint32_t)left_pwm);
        pwm_write(PIN_MOTOR_IN2, 1, 0);
    } else if (left_pwm < 0) {
        pwm_write(PIN_MOTOR_IN1, 0, 0);
        pwm_write(PIN_MOTOR_IN2, 1, (uint32_t)(-left_pwm));
    } else {
        pwm_write(PIN_MOTOR_IN1, 0, 0);
        pwm_write(PIN_MOTOR_IN2, 1, 0);
    }

    /* Right Motor: IN3, IN4 */
    if (right_pwm > 0) {
        pwm_write(PIN_MOTOR_IN3, 2, (uint32_t)right_pwm);
        pwm_write(PIN_MOTOR_IN4, 3, 0);
    } else if (right_pwm < 0) {
        pwm_write(PIN_MOTOR_IN3, 2, 0);
        pwm_write(PIN_MOTOR_IN4, 3, (uint32_t)(-right_pwm));
    } else {
        pwm_write(PIN_MOTOR_IN3, 2, 0);
        pwm_write(PIN_MOTOR_IN4, 3, 0);
    }
#endif
}

void motors_stop(void) {
#if MOTOR_DRIVER_TB6612
    motors_set_speed(0, 0); /* IN1=IN2=LOW, PWM=HIGH: high-impedance stop. */
#else
    pwm_write(PIN_MOTOR_IN1, 0, 0);
    pwm_write(PIN_MOTOR_IN2, 1, 0);
    pwm_write(PIN_MOTOR_IN3, 2, 0);
    pwm_write(PIN_MOTOR_IN4, 3, 0);
#endif
}

void motors_brake(void) {
#if MOTOR_DRIVER_TB6612
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
    digitalWrite(PIN_MOTOR_IN3, HIGH);
    digitalWrite(PIN_MOTOR_IN4, HIGH);
    pwm_write(PIN_MOTOR_PWMA, 0, 255);
    pwm_write(PIN_MOTOR_PWMB, 1, 255);
#else
    pwm_write(PIN_MOTOR_IN1, 0, 255);
    pwm_write(PIN_MOTOR_IN2, 1, 255);
    pwm_write(PIN_MOTOR_IN3, 2, 255);
    pwm_write(PIN_MOTOR_IN4, 3, 255);
#endif
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
