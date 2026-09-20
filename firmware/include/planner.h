/**
 * @file planner.h
 * @brief Zero-allocation Turn-Weighted A* state-space planner for embedded microcontrollers.
 */

#ifndef PLANNER_H
#define PLANNER_H

#include "fixed_maze.h"
#include "floodfill.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_PATH_COMMANDS 100
#define TURN_PENALTY_DEFAULT 1.5f

/**
 * @brief Compute the optimal turn-penalized command sequence to reach the center goal.
 * @param grid Pointer to current maze wall model.
 * @param start_pose Initial robot cell and direction.
 * @param turn_penalty Cost added for 90-degree turns (e.g. 1.5f).
 * @param out_commands Array to receive generated MoveCommand sequence.
 * @return Number of commands in plan, or 0 if already at goal, or -1 if unreachable.
 */
int planner_plan_path(
    const MazeGrid *grid,
    RobotPose start_pose,
    float turn_penalty,
    MoveCommand out_commands[MAX_PATH_COMMANDS]
);

/**
 * @brief Compute optimal turn-penalized command sequence constrained to verified cells.
 * @param grid Pointer to current maze wall model.
 * @param start_pose Initial robot cell and direction.
 * @param turn_penalty Cost added for 90-degree turns.
 * @param known_cells 2D bool array marking visited cells (or NULL for unconstrained).
 * @param out_commands Array to receive generated MoveCommand sequence.
 * @return Number of commands in plan, or 0 if already at goal, or -1 if unreachable.
 */
int planner_plan_safe_path(
    const MazeGrid *grid,
    RobotPose start_pose,
    float turn_penalty,
    const bool known_cells[MAZE_ROWS][MAZE_COLS],
    MoveCommand out_commands[MAX_PATH_COMMANDS]
);

#ifdef __cplusplus
}
#endif

#endif /* PLANNER_H */
