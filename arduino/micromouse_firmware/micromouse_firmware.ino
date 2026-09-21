/**
 * @file micromouse_firmware.ino
 * @brief Main Arduino Sketch for DOIT ESP32 DevKit V1 (MMRC26 Micromouse).
 * @author MMRC26 Team
 * 
 * Required Library in Arduino IDE:
 * Sketch -> Include Library -> Manage Libraries... -> Search "VL53L0X" by Pololu -> Install.
 */

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "controller.h"
#include "fixed_maze.h"
#include "floodfill.h"
#include "planner.h"

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
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  MMRC26 Micromouse - ESP32 DevKit V1  ");
    Serial.println("========================================");

    /* 1. Configure Operator IO */
    pinMode(PIN_BUTTON_START, INPUT_PULLUP);
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);

    /* 2. Startup notification */
    blink_led(3, 100);

    /* 3. Initialize Actuators and Sensors */
    Serial.println("[INIT] Initializing Motors...");
    motors_init();

    Serial.println("[INIT] Initializing Sensors & Calibrating Gyro...");
    sensors_init();

    Serial.println("[INIT] Initializing Motion Controller...");
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

    Serial.println("[READY] Robot in IDLE state. Press Start Button (GPIO 13) to begin Search Run.");
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

    Serial.printf("[SEARCH] Cell (%d, %d) | Walls: [F:%d L:%d R:%d] | Move: %d\n",
                  r, c, dist.wall_front, dist.wall_left, dist.wall_right, cmd);

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

        Serial.println("\n**************************************************");
        Serial.printf(">>> ISLAND GOAL REACHED! Banked Run #%d <<<\n", successful_runs_count);
        Serial.println("**************************************************");

        /* Precompute Turn-Weighted A* safe path for Speed Runs (zero phantom wall collisions) */
        RobotPose start_pose = {{9, 0}, DIR_NORTH};
        num_speed_commands = planner_plan_safe_path(&maze, start_pose, TURN_PENALTY_DEFAULT, visited_cells, speed_commands);
        Serial.printf("[PLANNER] Computed Safe Speed Path: %d commands.\n", num_speed_commands);

        /* Solid LED indicates Run is banked and Speed Run path is locked */
        digitalWrite(PIN_LED_STATUS, HIGH);
    }
}

static void execute_speed_run() {
    Serial.println("\n[SPEEDRUN] Executing Precomputed Sprint Path...");
    controller_init();
    digitalWrite(PIN_LED_STATUS, HIGH);

    for (int i = 0; i < num_speed_commands; i++) {
        MoveCommand cmd = speed_commands[i];
        switch (cmd) {
            case CMD_FORWARD: {
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

    motors_brake();
    delay(50);
    motors_stop();

    successful_runs_count++;
    current_mode = MODE_GOAL_BANKED;

    Serial.printf("[SPEEDRUN COMPLETE] Banked Run #%d!\n", successful_runs_count);
    blink_led(5, 80);
    digitalWrite(PIN_LED_STATUS, HIGH);
}

void loop() {
    switch (current_mode) {
        case MODE_IDLE:
            if (digitalRead(PIN_BUTTON_START) == LOW) {
                delay(50);
                if (digitalRead(PIN_BUTTON_START) == LOW) {
                    Serial.println("[START] Search Run triggered. Starting in 1 second...");
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
            if (digitalRead(PIN_BUTTON_START) == LOW) {
                delay(50);
                if (digitalRead(PIN_BUTTON_START) == LOW) {
                    Serial.println("[START] Speed Run triggered. Sprinting in 1 second...");
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
