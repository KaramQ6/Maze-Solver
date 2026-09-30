/**
 * @file planner.c
 * @brief Zero-allocation Turn-Weighted A* state-space planner implementation.
 */

#include "planner.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <float.h>

#define NUM_STATES 400

static const int8_t ROW_OFFSETS[4] = {-1, 0, 1, 0}; /* NORTH, EAST, SOUTH, WEST */
static const int8_t COL_OFFSETS[4] = {0, 1, 0, -1};

typedef struct {
    float f_score;
    uint16_t state_id;
} HeapNode;

typedef struct {
    HeapNode nodes[NUM_STATES];
    int16_t positions[NUM_STATES];
    int size;
} MinHeap;

static void heap_push(MinHeap *heap, uint16_t state_id, float f_score) {
    int i = heap->positions[state_id];
    if (i < 0) {
        i = heap->size++;
    }
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (heap->nodes[parent].f_score <= f_score) break;
        heap->nodes[i] = heap->nodes[parent];
        heap->positions[heap->nodes[i].state_id] = (int16_t)i;
        i = parent;
    }
    heap->nodes[i].state_id = state_id;
    heap->nodes[i].f_score = f_score;
    heap->positions[state_id] = (int16_t)i;
}

static HeapNode heap_pop(MinHeap *heap) {
    HeapNode min_node = heap->nodes[0];
    HeapNode last = heap->nodes[--heap->size];
    heap->positions[min_node.state_id] = -1;
    if (heap->size == 0) return min_node;

    int i = 0;
    while (i * 2 + 1 < heap->size) {
        int left = i * 2 + 1;
        int right = i * 2 + 2;
        int smallest = left;
        if (right < heap->size && heap->nodes[right].f_score < heap->nodes[left].f_score) {
            smallest = right;
        }
        if (last.f_score <= heap->nodes[smallest].f_score) break;
        heap->nodes[i] = heap->nodes[smallest];
        heap->positions[heap->nodes[i].state_id] = (int16_t)i;
        i = smallest;
    }
    heap->nodes[i] = last;
    heap->positions[last.state_id] = (int16_t)i;
    return min_node;
}

static inline uint16_t encode_state(uint8_t r, uint8_t c, Direction d) {
    return (uint16_t)(((r * 10) + c) * 4 + d);
}

static inline void decode_state(uint16_t state_id, uint8_t *r, uint8_t *c, Direction *d) {
    *d = (Direction)(state_id % 4);
    uint16_t cell_idx = state_id / 4;
    *r = (uint8_t)(cell_idx / 10);
    *c = (uint8_t)(cell_idx % 10);
}

static float heuristic_manhattan(uint8_t r, uint8_t c) {
    static const uint8_t goals[4][2] = {{4, 4}, {4, 5}, {5, 4}, {5, 5}};
    float min_h = 100.0f;
    for (int i = 0; i < 4; i++) {
        float h = (float)(abs((int)r - (int)goals[i][0]) + abs((int)c - (int)goals[i][1]));
        if (h < min_h) min_h = h;
    }
    return min_h;
}

int planner_plan_safe_path(
    const MazeGrid *grid,
    RobotPose start_pose,
    float turn_penalty,
    bool known_cells[MAZE_ROWS][MAZE_COLS],
    MoveCommand out_commands[MAX_PATH_COMMANDS]
) {
    if (!grid || !out_commands || !isfinite(turn_penalty) || turn_penalty < 0.0f
        || start_pose.pos.row >= MAZE_ROWS || start_pose.pos.col >= MAZE_COLS
        || start_pose.dir < DIR_NORTH || start_pose.dir > DIR_WEST) return -1;

    if (maze_is_goal(start_pose.pos.row, start_pose.pos.col)) {
        return 0;
    }

    static float g_score[NUM_STATES];
    static int16_t parent_state[NUM_STATES];
    static uint8_t parent_action[NUM_STATES];
    static bool closed[NUM_STATES];

    for (int i = 0; i < NUM_STATES; i++) {
        g_score[i] = FLT_MAX;
        parent_state[i] = -1;
        parent_action[i] = 0;
        closed[i] = false;
    }

    static MinHeap heap; /* Keep the bounded open set off the ESP32 loop task stack. */
    heap.size = 0;
    for (int i = 0; i < NUM_STATES; i++) heap.positions[i] = -1;

    uint16_t start_id = encode_state(start_pose.pos.row, start_pose.pos.col, start_pose.dir);
    g_score[start_id] = 0.0f;
    heap_push(&heap, start_id, heuristic_manhattan(start_pose.pos.row, start_pose.pos.col));

    uint16_t goal_reached_state = 0xFFFF;

    while (heap.size > 0) {
        HeapNode top = heap_pop(&heap);
        uint16_t curr_id = top.state_id;

        if (closed[curr_id]) continue;
        closed[curr_id] = true;

        uint8_t r, c;
        Direction d;
        decode_state(curr_id, &r, &c, &d);

        if (maze_is_goal(r, c)) {
            goal_reached_state = curr_id;
            break;
        }

        /* 1. Forward action */
        if (!maze_has_wall(grid, r, c, d)) {
            int8_t nr = (int8_t)(r + ROW_OFFSETS[d]);
            int8_t nc = (int8_t)(c + COL_OFFSETS[d]);
            if (nr >= 0 && nr < MAZE_ROWS && nc >= 0 && nc < MAZE_COLS) {
                if (known_cells != NULL && !known_cells[nr][nc]) {
                    /* Skip unverified cells when safety constraint is active */
                } else {
                    uint16_t next_id = encode_state((uint8_t)nr, (uint8_t)nc, d);
                    float tentative_g = g_score[curr_id] + 1.0f;
                    if (tentative_g < g_score[next_id]) {
                        g_score[next_id] = tentative_g;
                        parent_state[next_id] = (int16_t)curr_id;
                        parent_action[next_id] = (uint8_t)CMD_FORWARD;
                        float f = tentative_g + heuristic_manhattan((uint8_t)nr, (uint8_t)nc);
                        heap_push(&heap, next_id, f);
                    }
                }
            }
        }

        /* 2. Turn Left action */
        {
            Direction left_d = dir_left(d);
            uint16_t next_id = encode_state(r, c, left_d);
            float tentative_g = g_score[curr_id] + turn_penalty;
            if (tentative_g < g_score[next_id]) {
                g_score[next_id] = tentative_g;
                parent_state[next_id] = (int16_t)curr_id;
                parent_action[next_id] = (uint8_t)CMD_TURN_LEFT;
                float f = tentative_g + heuristic_manhattan(r, c);
                heap_push(&heap, next_id, f);
            }
        }

        /* 3. Turn Right action */
        {
            Direction right_d = dir_right(d);
            uint16_t next_id = encode_state(r, c, right_d);
            float tentative_g = g_score[curr_id] + turn_penalty;
            if (tentative_g < g_score[next_id]) {
                g_score[next_id] = tentative_g;
                parent_state[next_id] = (int16_t)curr_id;
                parent_action[next_id] = (uint8_t)CMD_TURN_RIGHT;
                float f = tentative_g + heuristic_manhattan(r, c);
                heap_push(&heap, next_id, f);
            }
        }

        /* 4. Turn Around action */
        {
            Direction back_d = dir_opposite(d);
            uint16_t next_id = encode_state(r, c, back_d);
            float tentative_g = g_score[curr_id] + 2.0f * turn_penalty;
            if (tentative_g < g_score[next_id]) {
                g_score[next_id] = tentative_g;
                parent_state[next_id] = (int16_t)curr_id;
                parent_action[next_id] = (uint8_t)CMD_TURN_AROUND;
                float f = tentative_g + heuristic_manhattan(r, c);
                heap_push(&heap, next_id, f);
            }
        }
    }

    if (goal_reached_state == 0xFFFF) {
        return -1; /* No path found */
    }

    /* Reconstruct path */
    static MoveCommand temp_commands[MAX_PATH_COMMANDS];
    int count = 0;
    uint16_t curr = goal_reached_state;

    while (curr != start_id && count < MAX_PATH_COMMANDS) {
        temp_commands[count++] = (MoveCommand)parent_action[curr];
        curr = (uint16_t)parent_state[curr];
    }
    if (curr != start_id) return -1;

    /* Reverse commands into out_commands */
    for (int i = 0; i < count; i++) {
        out_commands[i] = temp_commands[count - 1 - i];
    }

    return count;
}

int planner_plan_path(
    const MazeGrid *grid,
    RobotPose start_pose,
    float turn_penalty,
    MoveCommand out_commands[MAX_PATH_COMMANDS]
) {
    return planner_plan_safe_path(grid, start_pose, turn_penalty, NULL, out_commands);
}
