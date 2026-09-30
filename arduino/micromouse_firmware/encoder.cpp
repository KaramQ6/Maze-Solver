/**
 * @file encoder.cpp
 * @brief Hall-effect magnetic encoder implementation for N20 motors.
 *
 * Hardware: 2x A3144 Hall sensors on the encoder pins in config.h.
 *           2x neodymium magnets glued to each wheel, 180° apart.
 *
 * Each magnet pass triggers a FALLING-edge interrupt. A software debounce
 * window rejects false triggers from magnet bounce at high RPM.
 *
 * @author MMRC26 Team
 */

#include "encoder.h"

#if defined(ARDUINO) && ROBOT_HAS_ENCODERS
#include <Arduino.h>

/* ========================================================================== */
/* Wheel Geometry Constants                                                   */
/* ========================================================================== */

/** N20 wheel diameter ~34mm → circumference ~106.8mm */
#define WHEEL_CIRCUMFERENCE_MM  106.8f

/** Number of magnets per wheel (evenly spaced) */
#define MAGNETS_PER_WHEEL       2

/** Distance per encoder pulse = circumference / magnets */
#define MM_PER_PULSE            (WHEEL_CIRCUMFERENCE_MM / (float)MAGNETS_PER_WHEEL)

/**
 * Minimum microseconds between valid pulses (debounce window).
 * At max N20 speed ~500 RPM after gearbox: 500/60 = 8.33 rev/s
 * → 8.33 * 2 magnets = 16.67 pulses/s → ~60ms between pulses.
 * Set debounce to 15ms to allow headroom while rejecting bounce.
 */
#define DEBOUNCE_US             15000

/* ========================================================================== */
/* Volatile ISR State                                                         */
/* ========================================================================== */

static volatile uint32_t left_pulses  = 0;
static volatile uint32_t right_pulses = 0;
static volatile unsigned long left_last_us  = 0;
static volatile unsigned long right_last_us = 0;

/* ========================================================================== */
/* Interrupt Service Routines (must reside in IRAM)                           */
/* ========================================================================== */

static void IRAM_ATTR isr_encoder_left(void) {
    unsigned long now = micros();
    if ((now - left_last_us) > DEBOUNCE_US) {
        left_pulses++;
        left_last_us = now;
    }
}

static void IRAM_ATTR isr_encoder_right(void) {
    unsigned long now = micros();
    if ((now - right_last_us) > DEBOUNCE_US) {
        right_pulses++;
        right_last_us = now;
    }
}

/* ========================================================================== */
/* Public API                                                                 */
/* ========================================================================== */

void encoder_init(void) {
    /* A3144 has an open-collector output; use a module or external pull-up to 3.3V. */
    pinMode(PIN_ENCODER_LEFT, INPUT);
    pinMode(PIN_ENCODER_RIGHT, INPUT);

    left_pulses  = 0;
    right_pulses = 0;
    left_last_us  = 0;
    right_last_us = 0;

    /* A3144 pulls LOW when magnet is detected → FALLING edge */
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_LEFT),  isr_encoder_left,  FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_RIGHT), isr_encoder_right, FALLING);
}

void encoder_reset(void) {
    /* Briefly disable interrupts for atomic reset */
    noInterrupts();
    left_pulses  = 0;
    right_pulses = 0;
    interrupts();
}

uint32_t encoder_get_left_pulses(void) {
    noInterrupts();
    uint32_t val = left_pulses;
    interrupts();
    return val;
}

uint32_t encoder_get_right_pulses(void) {
    noInterrupts();
    uint32_t val = right_pulses;
    interrupts();
    return val;
}

float encoder_get_distance_mm(void) {
    noInterrupts();
    uint32_t lp = left_pulses;
    uint32_t rp = right_pulses;
    interrupts();

    /* Average of both wheels for straight-line estimation */
    float avg_pulses = ((float)lp + (float)rp) / 2.0f;
    return avg_pulses * MM_PER_PULSE;
}

bool encoder_has_travelled(float distance_mm) {
    return encoder_get_left_pulses() * MM_PER_PULSE >= distance_mm
        && encoder_get_right_pulses() * MM_PER_PULSE >= distance_mm;
}

bool encoder_cell_crossed(void) {
    return encoder_has_travelled(CELL_PITCH_MM);
}

#else
/* Host test mock implementation */
static uint32_t mock_left = 0;
static uint32_t mock_right = 0;

void encoder_init(void) {
    mock_left = 0;
    mock_right = 0;
}

void encoder_reset(void) {
    mock_left = 0;
    mock_right = 0;
}

uint32_t encoder_get_left_pulses(void) {
    return mock_left;
}

uint32_t encoder_get_right_pulses(void) {
    return mock_right;
}

float encoder_get_distance_mm(void) {
    return 0.0f;
}

bool encoder_has_travelled(float distance_mm) {
    (void)distance_mm;
    return false;
}

bool encoder_cell_crossed(void) {
    return false;
}
#endif
