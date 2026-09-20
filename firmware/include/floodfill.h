/**
 * @file floodfill.h
 * @brief Zero-allocation multi-source BFS Flood Fill for embedded microcontrollers.
 */

#ifndef FLOODFILL_H
#define FLOODFILL_H

#include "fixed_maze.h"

#define UNREACHABLE_DIST 255

typedef enum {
    CMD_HALT        = 0,
    CMD_FORWARD     = 1,
    CMD_TURN_LEFT   = 2,
    CMD_TURN_RIGHT  = 3,
    CMD_TURN_AROUND = 4
} MoveCommand;

/**
 * @brief Compute distance map from goal cells to all cells using static circular queue BFS.
 * @param grid Pointer to current maze wall model.
 * @param dist 10x10 output array of topological distances.
 */
void floodfill_compute_distances(const MazeGrid *grid, uint8_t dist[MAZE_ROWS][MAZE_COLS]);

/**
 * @brief Compute distance map from any arbitrary target cell (e.g. for return trip to start).
 */
void floodfill_compute_target_distances(const MazeGrid *grid, uint8_t target_row, uint8_t target_col, uint8_t dist[MAZE_ROWS][MAZE_COLS]);

/**
 * @brief Determine next discrete navigation command per MMRC26 deterministic tie-breaking.
 * Updates pose if motion occurs.
 */
MoveCommand floodfill_get_next_move(RobotPose *pose, const MazeGrid *grid, uint8_t dist[MAZE_ROWS][MAZE_COLS]);

#endif /* FLOODFILL_H */
