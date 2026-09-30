/**
 * @file encoder.h
 * @brief Hall-effect magnetic encoder for N20 DC motors using A3144 sensors.
 *
 * Two neodymium magnets per wheel trigger an A3144 Hall sensor on each side.
 * Pulses are counted on pins from config.h (~53.4mm per pulse with two magnets)
 * for cell-crossing detection.
 *
 * @author MMRC26 Team
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize encoder GPIO pins with interrupts.
 * Must be called after motors_init() and before controller_init().
 */
void encoder_init(void);

/**
 * @brief Reset both left and right pulse counters to zero.
 * Call at the start of each cell traversal.
 */
void encoder_reset(void);

/**
 * @brief Get current pulse count for left wheel.
 * @return Number of magnet passes since last reset (interrupt-safe read).
 */
uint32_t encoder_get_left_pulses(void);

/**
 * @brief Get current pulse count for right wheel.
 * @return Number of magnet passes since last reset (interrupt-safe read).
 */
uint32_t encoder_get_right_pulses(void);

/**
 * @brief Get average distance traveled by both wheels in mm.
 * @return Estimated distance in mm based on pulse count and wheel geometry.
 */
float encoder_get_distance_mm(void);

/** @brief True only when both wheels have traveled at least the requested distance. */
bool encoder_has_travelled(float distance_mm);

/**
 * @brief Check if both wheels have traveled at least one cell pitch (CELL_PITCH_MM).
 * @return true only if neither wheel stalled short of the next cell.
 */
bool encoder_cell_crossed(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */
