/**
 * @file micromouse_firmware.ino
 * @brief Micromouse sketch; ESP32-C3 automatically selects the SuperMini hardware profile.
 * @author MMRC26 Team
 * 
 * Required Library in Arduino IDE:
 * Sketch -> Include Library -> Manage Libraries... -> Search "VL53L0X" by Pololu -> Install.
 */

#include "config.h"
#include "sensors.h"
#include "motors.h"
#include "controller.h"
#include "encoder.h"
#include "fixed_maze.h"
#include "floodfill.h"
#include "planner.h"
#include <string.h>

typedef enum {
    MODE_IDLE,
    MODE_SEARCH_RUN,
    MODE_GOAL_BANKED,
    MODE_SPEED_RUN,
    MODE_FAULT
} RobotMode;

static MazeGrid maze;
static uint8_t dist_map[MAZE_ROWS][MAZE_COLS];
static MoveCommand speed_commands[MAX_PATH_COMMANDS];
static int num_speed_commands = 0;
static RobotPose current_pose;
static bool visited_cells[MAZE_ROWS][MAZE_COLS];
static RobotMode current_mode = MODE_IDLE;
static int successful_runs_count = 0;

static void set_status_led(bool on) {
#if PIN_LED_STATUS >= 0
    digitalWrite(PIN_LED_STATUS, on ? HIGH : LOW);
#else
    (void)on;
#endif
}

static void blink_led(int times, int delay_ms) {
#if defined(PIN_LED_STATUS) && (PIN_LED_STATUS >= 0)
    for (int i = 0; i < times; i++) {
        digitalWrite(PIN_LED_STATUS, HIGH);
        delay(delay_ms);
        digitalWrite(PIN_LED_STATUS, LOW);
        delay(delay_ms);
    }
#else
    (void)times;
    (void)delay_ms;
#endif
}

static void stop_on_fault() {
    motors_brake();
    motors_stop();
    current_mode = MODE_FAULT;
    Serial.println("[FAULT] Motion or sensor failure; run aborted.");
}

static bool execute_motion(MoveCommand cmd, int pwm) {
    switch (cmd) {
        case CMD_FORWARD:
            return controller_step_forward(pwm);
        case CMD_TURN_LEFT:
            return controller_turn_90(true);
        case CMD_TURN_RIGHT:
            return controller_turn_90(false);
        case CMD_TURN_AROUND:
            return controller_turn_180();
        default:
            return false;
    }
}

static bool read_confirmed_walls(SensorDistances *result) {
    int front = 0, left = 0, right = 0;
    int samples = 3;
    for (int i = 0; i < samples; i++) {
        if (!sensors_read_distances(result)) return false;
        front += result->wall_front;
        left += result->wall_left;
        right += result->wall_right;
        if (i == 2 && ((front > 0 && front < 3) || (left > 0 && left < 3)
                       || (right > 0 && right < 3))) samples = 5;
    }
    result->wall_front = front > samples / 2;
    result->wall_left = left > samples / 2;
    result->wall_right = right > samples / 2;
    return true;
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  MMRC26 - Selected hardware profile  ");
    Serial.println("========================================");

    /* 1. Configure Operator IO (Onboard BOOT Button on GPIO 0) */
    pinMode(PIN_BUTTON_START, INPUT_PULLUP);
#if defined(PIN_LED_STATUS) && (PIN_LED_STATUS >= 0)
    pinMode(PIN_LED_STATUS, OUTPUT);
    digitalWrite(PIN_LED_STATUS, LOW);
#endif

    /* 2. Startup notification */
    blink_led(3, 100);

    /* 3. Initialize Actuators and Sensors */
    Serial.println("[INIT] Initializing Motors...");
    motors_init();

    Serial.println("[INIT] Initializing Sensors & Calibrating Gyro...");
    if (!sensors_init()) {
        stop_on_fault();
        return;
    }

    Serial.println("[INIT] Initializing Encoders...");
#if ROBOT_HAS_ENCODERS
    encoder_init();
#endif

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

    Serial.println("[READY] Press and release BOOT after power-on to begin Search Run.");
}

static void execute_search_step() {
    /* 1. Read ToF sensors at current cell */
    SensorDistances dist;
    if (!read_confirmed_walls(&dist)) {
        stop_on_fault();
        return;
    }

    uint8_t r = current_pose.pos.row;
    uint8_t c = current_pose.pos.col;
    visited_cells[r][c] = true;
    Direction fwd = current_pose.dir;
    Direction left = (Direction)((fwd + 3) % 4);
    Direction right = (Direction)((fwd + 1) % 4);

    /* 2. Update single-source-of-truth wall arrays */
    if (dist.wall_front) maze_set_wall(&maze, r, c, fwd, true);
    if (dist.wall_left) maze_set_wall(&maze, r, c, left, true);
    if (dist.wall_right) maze_set_wall(&maze, r, c, right, true);

    /* 3. Recompute BFS floodfill distance field */
    floodfill_compute_distances(&maze, dist_map);
    if (dist_map[r][c] == UNREACHABLE_DIST) {
        stop_on_fault();
        return;
    }

    /* 4. Query deterministic next movement */
    RobotPose next_pose = current_pose;
    MoveCommand cmd = floodfill_get_next_move(&next_pose, &maze, dist_map);

    Serial.printf("[SEARCH] Cell (%d, %d) | Walls: [F:%d L:%d R:%d] | Move: %d\n",
                  r, c, dist.wall_front, dist.wall_left, dist.wall_right, cmd);

    /* 5. Execute physical motion */
    if (!execute_motion(cmd, SEARCH_BASE_PWM)) {
        stop_on_fault();
        return;
    }
    current_pose = next_pose;
    visited_cells[current_pose.pos.row][current_pose.pos.col] = true;

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
        if (num_speed_commands <= 0) {
            stop_on_fault();
            return;
        }

        /* Solid LED indicates Run is banked and Speed Run path is locked */
        set_status_led(true);
    }
}

static void execute_speed_run() {
    Serial.println("\n[SPEEDRUN] Executing Precomputed Sprint Path...");
    controller_init();
    set_status_led(true);

    static const int8_t row_offset[4] = {-1, 0, 1, 0};
    static const int8_t col_offset[4] = {0, 1, 0, -1};
    for (int i = 0; i < num_speed_commands; i++) {
        MoveCommand cmd = speed_commands[i];
        RobotPose next_pose = current_pose;
        switch (cmd) {
            case CMD_FORWARD: {
                int row = (int)current_pose.pos.row + row_offset[current_pose.dir];
                int col = (int)current_pose.pos.col + col_offset[current_pose.dir];
                if (row < 0 || row >= MAZE_ROWS || col < 0 || col >= MAZE_COLS
                    || maze_has_wall(&maze, current_pose.pos.row, current_pose.pos.col, current_pose.dir)) {
                    stop_on_fault();
                    return;
                }
                next_pose.pos.row = (uint8_t)row;
                next_pose.pos.col = (uint8_t)col;
                break;
            }
            case CMD_TURN_LEFT:
                next_pose.dir = dir_left(current_pose.dir);
                break;

            case CMD_TURN_RIGHT:
                next_pose.dir = dir_right(current_pose.dir);
                break;

            case CMD_TURN_AROUND:
                next_pose.dir = dir_opposite(current_pose.dir);
                break;

            case CMD_HALT:
            default:
                stop_on_fault();
                return;
        }
        if (!execute_motion(cmd, SPEEDRUN_BASE_PWM)) {
            stop_on_fault();
            return;
        }
        current_pose = next_pose;
    }

    if (!maze_is_goal(current_pose.pos.row, current_pose.pos.col)) {
        stop_on_fault();
        return;
    }

    motors_brake();
    delay(50);
    motors_stop();

    successful_runs_count++;
    current_mode = MODE_GOAL_BANKED;

    Serial.printf("[SPEEDRUN COMPLETE] Banked Run #%d!\n", successful_runs_count);
    blink_led(5, 80);
    set_status_led(true);
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

        case MODE_FAULT:
            delay(20);
            break;
    }
}
