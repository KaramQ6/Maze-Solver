/**
 * @file test_firmware.c
 * @brief Unit test suite for C99 embedded firmware modules.
 */

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <string.h>

#include "fixed_maze.h"
#include "floodfill.h"
#include "planner.h"
#include "kinematics.h"

static void test_maze_grid(void) {
    printf("[TEST] Running test_maze_grid...\n");
    MazeGrid grid;
    maze_init(&grid);

    /* Boundary walls */
    assert(maze_has_wall(&grid, 0, 0, DIR_NORTH) == true);
    assert(maze_has_wall(&grid, 0, 0, DIR_WEST) == true);
    assert(maze_has_wall(&grid, 9, 9, DIR_SOUTH) == true);
    assert(maze_has_wall(&grid, 9, 9, DIR_EAST) == true);

    /* Start cell (9,0) East wall set by competition defaults */
    assert(maze_has_wall(&grid, 9, 0, DIR_EAST) == true);
    /* Neighbor (9,1) West wall must also be true */
    assert(maze_has_wall(&grid, 9, 1, DIR_WEST) == true);

    /* Interior walls initially open */
    assert(maze_has_wall(&grid, 1, 1, DIR_NORTH) == false);
    assert(maze_has_wall(&grid, 1, 1, DIR_SOUTH) == false);

    /* Set interior wall */
    maze_set_wall(&grid, 1, 1, DIR_EAST, true);
    assert(maze_has_wall(&grid, 1, 1, DIR_EAST) == true);
    assert(maze_has_wall(&grid, 1, 2, DIR_WEST) == true);

    /* Clear interior wall */
    maze_set_wall(&grid, 1, 1, DIR_EAST, false);
    assert(maze_has_wall(&grid, 1, 1, DIR_EAST) == false);
    assert(maze_has_wall(&grid, 1, 2, DIR_WEST) == false);

    /* Island goal checks */
    assert(maze_is_goal(4, 4) == true);
    assert(maze_is_goal(4, 5) == true);
    assert(maze_is_goal(5, 4) == true);
    assert(maze_is_goal(5, 5) == true);
    assert(maze_is_goal(0, 0) == false);

    printf("[PASS] test_maze_grid passed successfully.\n");
}

static void test_floodfill(void) {
    printf("[TEST] Running test_floodfill...\n");
    MazeGrid grid;
    maze_init(&grid);

    uint8_t dist[MAZE_ROWS][MAZE_COLS];
    floodfill_compute_distances(&grid, dist);

    /* Goals must have distance 0 */
    assert(dist[4][4] == 0);
    assert(dist[4][5] == 0);
    assert(dist[5][4] == 0);
    assert(dist[5][5] == 0);

    /* Surrounding cells must have distance 1 */
    assert(dist[3][4] == 1);
    assert(dist[4][3] == 1);
    assert(dist[6][4] == 1);
    assert(dist[4][6] == 1);

    /* Start cell (9,0) should have valid positive distance */
    assert(dist[9][0] > 0 && dist[9][0] < UNREACHABLE_DIST);

    /* Test get_next_move */
    RobotPose pose = { .pos = {9, 0}, .dir = DIR_NORTH };
    MoveCommand cmd = floodfill_get_next_move(&pose, &grid, dist);
    /* Since east wall is closed and north is open, it should move forward (north) */
    assert(cmd == CMD_FORWARD);
    assert(pose.pos.row == 8);
    assert(pose.pos.col == 0);

    printf("[PASS] test_floodfill passed successfully.\n");
}

static void test_planner(void) {
    printf("[TEST] Running test_planner...\n");
    MazeGrid grid;
    maze_init(&grid);

    RobotPose start_pose = { .pos = {9, 0}, .dir = DIR_NORTH };
    MoveCommand commands[MAX_PATH_COMMANDS];

    int num_cmds = planner_plan_path(&grid, start_pose, TURN_PENALTY_DEFAULT, commands);
    assert(num_cmds > 0);
    printf("  Generated %d commands from (9,0) to goal.\n", num_cmds);

    /* Simulate following commands and verify it reaches goal */
    RobotPose sim_pose = start_pose;
    static const int8_t r_off[4] = {-1, 0, 1, 0};
    static const int8_t c_off[4] = {0, 1, 0, -1};

    for (int i = 0; i < num_cmds; i++) {
        switch (commands[i]) {
            case CMD_FORWARD:
                assert(!maze_has_wall(&grid, sim_pose.pos.row, sim_pose.pos.col, sim_pose.dir));
                sim_pose.pos.row = (uint8_t)(sim_pose.pos.row + r_off[sim_pose.dir]);
                sim_pose.pos.col = (uint8_t)(sim_pose.pos.col + c_off[sim_pose.dir]);
                break;
            case CMD_TURN_LEFT:
                sim_pose.dir = dir_left(sim_pose.dir);
                break;
            case CMD_TURN_RIGHT:
                sim_pose.dir = dir_right(sim_pose.dir);
                break;
            case CMD_TURN_AROUND:
                sim_pose.dir = dir_opposite(sim_pose.dir);
                break;
            case CMD_HALT:
                break;
        }
    }

    assert(maze_is_goal(sim_pose.pos.row, sim_pose.pos.col));
    printf("[PASS] test_planner passed successfully (Goal reached at %d, %d).\n",
           sim_pose.pos.row, sim_pose.pos.col);
}

static void test_safe_planner_rejects_unvisited_goal(void) {
    MazeGrid grid;
    maze_init(&grid);
    bool visited[MAZE_ROWS][MAZE_COLS];
    memset(visited, 0, sizeof(visited));
    visited[5][3] = true;
    MoveCommand commands[MAX_PATH_COMMANDS];
    RobotPose start = { .pos = {5, 3}, .dir = DIR_EAST };

    assert(planner_plan_safe_path(&grid, start, TURN_PENALTY_DEFAULT, visited, commands) == -1);
}

static void test_kinematics(void) {
    printf("[TEST] Running test_kinematics...\n");
    KinematicConfig baseline = {
        .max_velocity = DEFAULT_V_MAX,
        .max_acceleration = DEFAULT_A_MAX,
        .friction_coeff = DEFAULT_MU,
        .suction_multiplier = 0.0f /* No suction */
    };

    KinematicConfig suction = {
        .max_velocity = DEFAULT_V_MAX,
        .max_acceleration = DEFAULT_A_MAX,
        .friction_coeff = DEFAULT_MU,
        .suction_multiplier = SUCTION_DOWNFORCE_K /* 3.0 downforce */
    };

    float v_base = kinematics_max_turn_velocity(&baseline);
    float v_suction = kinematics_max_turn_velocity(&suction);

    printf("  Turn speed (No suction): %.2f m/s\n", v_base);
    printf("  Turn speed (With 3G suction): %.2f m/s\n", v_suction);

    /* Suction must significantly increase cornering velocity */
    assert(v_suction > v_base * 1.8f);

    float t_turn_base = kinematics_turn_time(&baseline);
    float t_turn_suction = kinematics_turn_time(&suction);
    assert(t_turn_suction < t_turn_base);

    float t_straight = kinematics_straight_time(&suction, 5);
    assert(t_straight > 0.0f);
    printf("  5-cell straight sprint time: %.3f s\n", t_straight);

    printf("[PASS] test_kinematics passed successfully.\n");
}

int main(void) {
    printf("===========================================\n");
    printf("  MMRC26 Embedded C99 Firmware Test Suite  \n");
    printf("===========================================\n");

    test_maze_grid();
    test_floodfill();
    test_planner();
    test_safe_planner_rejects_unvisited_goal();
    test_kinematics();

    printf("\n>>> ALL 4 C99 EMBEDDED TEST SUITES PASSED! <<<\n");
    return 0;
}
