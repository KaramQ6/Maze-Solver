/**
 * @file test_firmware_sim.cpp
 * @brief Host verification test for firmware algorithms and control structures.
 */

#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "controller.h"

extern "C" {
#include "fixed_maze.h"
#include "floodfill.h"
#include "planner.h"
}

int main(void) {
    printf("--- Running Firmware Host Simulation Test ---\n");

    /* 1. Test Grid Init */
    MazeGrid maze;
    maze_init(&maze);

    /* Verify outer boundaries */
    assert(maze_has_wall(&maze, 0, 0, DIR_NORTH) == true);
    assert(maze_has_wall(&maze, 0, 0, DIR_WEST) == true);
    assert(maze_has_wall(&maze, 9, 9, DIR_SOUTH) == true);
    assert(maze_has_wall(&maze, 9, 9, DIR_EAST) == true);
    /* Verify start cell peg (9, 0 East wall) */
    assert(maze_has_wall(&maze, 9, 0, DIR_EAST) == true);
    printf("[PASS] Grid initialization and boundary walls verified.\n");

    /* 2. Test FloodFill BFS on empty maze */
    uint8_t dist_map[MAZE_ROWS][MAZE_COLS];
    floodfill_compute_distances(&maze, dist_map);

    /* Goal cells must have distance 0 */
    assert(dist_map[4][4] == 0);
    assert(dist_map[4][5] == 0);
    assert(dist_map[5][4] == 0);
    assert(dist_map[5][5] == 0);
    assert(maze_is_goal(4, 4) == true);
    assert(maze_is_goal(5, 5) == true);
    assert(maze_is_goal(0, 0) == false);
    printf("[PASS] FloodFill distance map computed correctly (goals at distance 0).\n");

    /* 3. Test Navigation step from Start Square (9, 0) */
    RobotPose pose = {{9, 0}, DIR_NORTH};
    MoveCommand cmd = floodfill_get_next_move(&pose, &maze, dist_map);
    assert(cmd == CMD_FORWARD);
    assert(pose.pos.row == 8);
    assert(pose.pos.col == 0);
    printf("[PASS] Deterministic next move correctly stepped North into (8, 0).\n");

    /* 4. Test Turn-Weighted A* Path Planner */
    MoveCommand speed_cmds[MAX_PATH_COMMANDS];
    RobotPose start_pose = {{9, 0}, DIR_NORTH};
    int num_cmds = planner_plan_path(&maze, start_pose, TURN_PENALTY_DEFAULT, speed_cmds);

    assert(num_cmds > 0);
    printf("[PASS] Planner computed speed run path with %d commands.\n", num_cmds);

    /* 4b. Test Safe Path Planner with visited_cells constraint */
    bool visited[MAZE_ROWS][MAZE_COLS];
    memset(visited, 0, sizeof(visited));
    /* Mark a valid corridor to goal: (9,0) -> (4,0) -> (4,4) */
    for (int r = 4; r <= 9; r++) visited[r][0] = true;
    for (int c = 0; c <= 4; c++) visited[4][c] = true;
    int num_safe_cmds = planner_plan_safe_path(&maze, start_pose, TURN_PENALTY_DEFAULT, visited, speed_cmds);
    assert(num_safe_cmds > 0);
    printf("[PASS] Safe planner computed visited-constrained path with %d commands.\n", num_safe_cmds);

    /* 5. Test Sensors & Controller stubs */
    assert(sensors_init() == true);
    SensorDistances dist;
    sensors_read_distances(&dist);
    assert(dist.wall_front == true);

    controller_init();
    assert(controller_get_target_heading() == 0.0f);
    controller_turn_90(false); /* Turn right */
    assert(controller_get_target_heading() == 90.0f);
    controller_turn_90(true);  /* Turn left */
    assert(controller_get_target_heading() == 0.0f);
    printf("[PASS] Motion controller target heading transitions verified.\n");

    printf("\n>>> ALL FIRMWARE UNIT TESTS PASSED SUCCESSFULLY! <<<\n");
    return 0;
}
