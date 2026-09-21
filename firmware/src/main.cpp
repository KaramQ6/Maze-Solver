/**
 * @file main.cpp
 * @brief Autonomous Micromouse Maze Solver firmware for Seeed Studio XIAO ESP32-S3 (MMRC26).
 * @author MMRC26 Team
 */

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "controller.h"

extern "C" {
#include "fixed_maze.h"
#include "floodfill.h"
#include "planner.h"
}

#ifdef ARDUINO
#include <Arduino.h>

typedef enum {
    MODE_IDLE,
    MODE_SEARCH_RUN,
    MODE_GOAL_BANKED,
    MODE_SPEED_RUN
} RobotMode;

static MazeGrid maze;
static uint8_t dist_map[MAZE_ROWS][MAZE_COLS];
static MoveCommand speed_commands[MAX_PATH_COMMANDS];
static int num_speed_commands = 0;
static RobotPose current_pose;
static bool visited_cells[MAZE_ROWS][MAZE_COLS];
static RobotMode current_mode = MODE_IDLE;
static int successful_runs_count = 0;

static void blink_led(int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH);
        delay(delay_ms);
        digitalWrite(PIN_LED_STATUS, LOW);
        delay(delay_ms);
    }
}

void setup() {
    /* 1. Configure Operator IO */
    pinMode(PIN_BUTTON_START, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_SPARE, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);
    digitalWrite(PIN_SPARE, LOW);

    /* 2. Startup notification */
    blink_led(3, 100);

    /* 3. Initialize Actuators and Sensors */
    motors_init();
    sensors_init();
    controller_init();

    /* 4. Pre-load known structural maze boundaries */
    maze_init(&maze);

    /* 5. Initialize Pose at standard MMRC26 Start Square (9, 0) facing North */
    current_pose.pos.row = 9;
    current_pose.pos.col = 0;
    current_pose.dir = DIR_NORTH;
    memset(visited_cells, 0, sizeof(visited_cells));
    visited_cells[9][0] = true;

    current_mode = MODE_IDLE;
    blink_led(2, 200);
}

static void execute_search_step() {
    /* 1. Read ToF sensors at current cell */
    SensorDistances dist;
    sensors_read_distances(&dist);

    uint8_t r = current_pose.pos.row;
    uint8_t c = current_pose.pos.col;
    visited_cells[r][c] = true;
    Direction fwd = current_pose.dir;
    Direction left = (Direction)((fwd + 3) % 4);
    Direction right = (Direction)((fwd + 1) % 4);

    /* 2. Update single-source-of-truth wall arrays */
    maze_set_wall(&maze, r, c, fwd, dist.wall_front);
    maze_set_wall(&maze, r, c, left, dist.wall_left);
    maze_set_wall(&maze, r, c, right, dist.wall_right);

    /* 3. Recompute BFS floodfill distance field */
    floodfill_compute_distances(&maze, dist_map);

    /* 4. Query deterministic next movement */
    MoveCommand cmd = floodfill_get_next_move(&current_pose, &maze, dist_map);

    /* 5. Execute physical motion */
    switch (cmd) {
        case CMD_FORWARD: {
            bool wall_ahead = maze_has_wall(&maze, current_pose.pos.row, current_pose.pos.col, current_pose.dir);
            controller_step_forward(wall_ahead, SEARCH_BASE_PWM);
            visited_cells[current_pose.pos.row][current_pose.pos.col] = true;
            break;
        }
        case CMD_TURN_LEFT:
            controller_turn_90(true);
            break;

        case CMD_TURN_RIGHT:
            controller_turn_90(false);
            break;

        case CMD_TURN_AROUND:
            controller_turn_180();
            break;

        case CMD_HALT:
        default:
            motors_brake();
            break;
    }

    /* 6. Check if reached Island Destination Zone (4,4), (4,5), (5,4), (5,5) */
    if (maze_is_goal(current_pose.pos.row, current_pose.pos.col)) {
        motors_brake();
        delay(50);
        motors_stop();

        successful_runs_count++;
        current_mode = MODE_GOAL_BANKED;

        /* Precompute Turn-Weighted A* safe path for Speed Runs (zero phantom wall collisions) */
        RobotPose start_pose = {{9, 0}, DIR_NORTH};
        num_speed_commands = planner_plan_safe_path(&maze, start_pose, TURN_PENALTY_DEFAULT, visited_cells, speed_commands);

        /* Solid LED indicates Run #1 is banked and Speed Run path is locked */
        digitalWrite(PIN_LED_STATUS, HIGH);
    }
}

static void execute_speed_run() {
    /* Reset controller heading at Start Cell */
    controller_init();
    digitalWrite(PIN_LED_STATUS, HIGH);

    for (int i = 0; i < num_speed_commands; i++) {
        MoveCommand cmd = speed_commands[i];
        switch (cmd) {
            case CMD_FORWARD: {
                /* Check if next cell has a front wall */
                bool is_last_step = (i == num_speed_commands - 1);
                controller_step_forward(is_last_step, SPEEDRUN_BASE_PWM);
                break;
            }
            case CMD_TURN_LEFT:
                controller_turn_90(true);
                break;

            case CMD_TURN_RIGHT:
                controller_turn_90(false);
                break;

            case CMD_TURN_AROUND:
                controller_turn_180();
                break;

            case CMD_HALT:
            default:
                break;
        }
    }

    /* Banked another successful run! */
    motors_brake();
    delay(50);
    motors_stop();

    successful_runs_count++;
    current_mode = MODE_GOAL_BANKED;
    blink_led(5, 80);
    digitalWrite(PIN_LED_STATUS, HIGH);
}

void loop() {
    switch (current_mode) {
        case MODE_IDLE:
            /* Wait for button press on D8 to start Search Run */
            if (digitalRead(PIN_BUTTON_START) == LOW) {
                delay(50); /* Debounce */
                if (digitalRead(PIN_BUTTON_START) == LOW) {
                    /* Operator release window */
                    blink_led(3, 150);
                    delay(800);

                    current_pose.pos.row = 9;
                    current_pose.pos.col = 0;
                    current_pose.dir = DIR_NORTH;
                    controller_init();

                    current_mode = MODE_SEARCH_RUN;
                }
            }
            delay(20);
            break;

        case MODE_SEARCH_RUN:
            execute_search_step();
            break;

        case MODE_GOAL_BANKED:
            /* Waiting for operator to reposition to Start Square or press button for Speed Run */
            if (digitalRead(PIN_BUTTON_START) == LOW) {
                delay(50);
                if (digitalRead(PIN_BUTTON_START) == LOW) {
                    blink_led(3, 100);
                    delay(800);

                    current_pose.pos.row = 9;
                    current_pose.pos.col = 0;
                    current_pose.dir = DIR_NORTH;

                    current_mode = MODE_SPEED_RUN;
                }
            }
            delay(20);
            break;

        case MODE_SPEED_RUN:
            execute_speed_run();
            break;
    }
}

#else
/* Host test entry point */
int main(void) {
    return 0;
}
#endif
