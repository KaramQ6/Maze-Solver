/**
 * @file floodfill.c
 * @brief Zero-allocation BFS floodfill algorithm implementation.
 */

#include "floodfill.h"
#include <string.h>

static const int8_t ROW_OFFSETS[4] = {-1, 0, 1, 0}; /* NORTH, EAST, SOUTH, WEST */
static const int8_t COL_OFFSETS[4] = {0, 1, 0, -1};

void floodfill_compute_distances(const MazeGrid *grid, uint8_t dist[MAZE_ROWS][MAZE_COLS]) {
    if (!grid || !dist) return;

    memset(dist, UNREACHABLE_DIST, sizeof(uint8_t) * MAZE_ROWS * MAZE_COLS);

    Cell queue[100];
    int head = 0;
    int tail = 0;

    /* Seed the 4 goal cells (4,4), (4,5), (5,4), (5,5) */
    static const uint8_t goals[4][2] = {{4, 4}, {4, 5}, {5, 4}, {5, 5}};
    for (int i = 0; i < 4; i++) {
        uint8_t gr = goals[i][0];
        uint8_t gc = goals[i][1];
        dist[gr][gc] = 0;
        queue[tail++] = (Cell){gr, gc};
    }

    while (head < tail) {
        Cell curr = queue[head++];
        uint8_t d = dist[curr.row][curr.col];

        for (int dir_idx = 0; dir_idx < 4; dir_idx++) {
            Direction dir = (Direction)dir_idx;
            if (!maze_has_wall(grid, curr.row, curr.col, dir)) {
                int8_t nr = (int8_t)(curr.row + ROW_OFFSETS[dir]);
                int8_t nc = (int8_t)(curr.col + COL_OFFSETS[dir]);

                if (nr >= 0 && nr < MAZE_ROWS && nc >= 0 && nc < MAZE_COLS) {
                    if (dist[nr][nc] > d + 1) {
                        dist[nr][nc] = (uint8_t)(d + 1);
                        queue[tail++] = (Cell){(uint8_t)nr, (uint8_t)nc};
                    }
                }
            }
        }
    }
}

void floodfill_compute_target_distances(const MazeGrid *grid, uint8_t target_row, uint8_t target_col, uint8_t dist[MAZE_ROWS][MAZE_COLS]) {
    if (!grid || !dist || target_row >= MAZE_ROWS || target_col >= MAZE_COLS) return;

    memset(dist, UNREACHABLE_DIST, sizeof(uint8_t) * MAZE_ROWS * MAZE_COLS);

    Cell queue[100];
    int head = 0;
    int tail = 0;

    dist[target_row][target_col] = 0;
    queue[tail++] = (Cell){target_row, target_col};

    while (head < tail) {
        Cell curr = queue[head++];
        uint8_t d = dist[curr.row][curr.col];

        for (int dir_idx = 0; dir_idx < 4; dir_idx++) {
            Direction dir = (Direction)dir_idx;
            if (!maze_has_wall(grid, curr.row, curr.col, dir)) {
                int8_t nr = (int8_t)(curr.row + ROW_OFFSETS[dir]);
                int8_t nc = (int8_t)(curr.col + COL_OFFSETS[dir]);

                if (nr >= 0 && nr < MAZE_ROWS && nc >= 0 && nc < MAZE_COLS) {
                    if (dist[nr][nc] > d + 1) {
                        dist[nr][nc] = (uint8_t)(d + 1);
                        queue[tail++] = (Cell){(uint8_t)nr, (uint8_t)nc};
                    }
                }
            }
        }
    }
}

MoveCommand floodfill_get_next_move(RobotPose *pose, const MazeGrid *grid, uint8_t dist[MAZE_ROWS][MAZE_COLS]) {
    if (!pose || !grid || !dist) return CMD_HALT;

    uint8_t r = pose->pos.row;
    uint8_t c = pose->pos.col;

    if (maze_is_goal(r, c)) {
        return CMD_HALT;
    }

    /* Find minimum distance among passable neighbors */
    uint8_t min_dist = 255;
    bool passable[4] = {false, false, false, false};

    for (int d = 0; d < 4; d++) {
        Direction dir = (Direction)d;
        if (!maze_has_wall(grid, r, c, dir)) {
            int8_t nr = (int8_t)(r + ROW_OFFSETS[dir]);
            int8_t nc = (int8_t)(c + COL_OFFSETS[dir]);
            if (nr >= 0 && nr < MAZE_ROWS && nc >= 0 && nc < MAZE_COLS) {
                passable[dir] = true;
                if (dist[nr][nc] < min_dist) {
                    min_dist = dist[nr][nc];
                }
            }
        }
    }

    if (min_dist == 255) {
        /* No exit: turn around */
        pose->dir = dir_opposite(pose->dir);
        return CMD_TURN_AROUND;
    }

    /* Tie-breaking hierarchy per Master Plan §4.1:
     * 1. Forward
     * 2. Left
     * 3. Right
     * 4. Turn Around
     */
    Direction forward = pose->dir;
    Direction left = dir_left(pose->dir);
    Direction right = dir_right(pose->dir);
    Direction back = dir_opposite(pose->dir);

    #define CANDIDATE_MATCHES_MIN(d) (passable[(d)] && dist[r + ROW_OFFSETS[(d)]][c + COL_OFFSETS[(d)]] == min_dist)

    Direction target_dir;
    if (CANDIDATE_MATCHES_MIN(forward)) {
        target_dir = forward;
    } else if (CANDIDATE_MATCHES_MIN(left)) {
        target_dir = left;
    } else if (CANDIDATE_MATCHES_MIN(right)) {
        target_dir = right;
    } else {
        target_dir = back;
    }

    #undef CANDIDATE_MATCHES_MIN

    if (target_dir == forward) {
        pose->pos.row = (uint8_t)(r + ROW_OFFSETS[forward]);
        pose->pos.col = (uint8_t)(c + COL_OFFSETS[forward]);
        return CMD_FORWARD;
    } else if (target_dir == left) {
        pose->dir = left;
        return CMD_TURN_LEFT;
    } else if (target_dir == right) {
        pose->dir = right;
        return CMD_TURN_RIGHT;
    } else {
        pose->dir = back;
        return CMD_TURN_AROUND;
    }
}
