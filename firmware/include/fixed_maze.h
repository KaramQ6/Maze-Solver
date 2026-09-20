/**
 * @file fixed_maze.h
 * @brief Zero-allocation bit-packed 10x10 maze grid for embedded microcontrollers (STM32/RP2040).
 * @author MMRC26 Team
 * 
 * Total RAM footprint: 42 bytes.
 */

#ifndef FIXED_MAZE_H
#define FIXED_MAZE_H

#include <stdint.h>
#include <stdbool.h>

#define MAZE_ROWS 10
#define MAZE_COLS 10

typedef enum {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3
} Direction;

typedef struct {
    uint8_t row;
    uint8_t col;
} Cell;

typedef struct {
    Cell pos;
    Direction dir;
} RobotPose;

typedef struct {
    uint16_t h_walls[MAZE_ROWS + 1]; /* 11 rows of 10 bits (bits 0..9) */
    uint16_t v_walls[MAZE_ROWS];     /* 10 rows of 11 bits (bits 0..10) */
} MazeGrid;

/**
 * @brief Initialize maze with outer boundaries and start cell peg (9, 0 East wall).
 * @param grid Pointer to MazeGrid structure.
 */
void maze_init(MazeGrid *grid);

/**
 * @brief Query if a wall exists adjacent to cell (row, col) in direction dir.
 */
bool maze_has_wall(const MazeGrid *grid, uint8_t row, uint8_t col, Direction dir);

/**
 * @brief Set or clear a wall adjacent to cell (row, col) in direction dir.
 */
void maze_set_wall(MazeGrid *grid, uint8_t row, uint8_t col, Direction dir, bool present);

/**
 * @brief Check if cell is one of the 4 center island goal cells (4,4), (4,5), (5,4), (5,5).
 */
bool maze_is_goal(uint8_t row, uint8_t col);

/**
 * @brief Get opposite direction (North <-> South, East <-> West).
 */
static inline Direction dir_opposite(Direction dir) {
    return (Direction)((dir + 2) % 4);
}

/**
 * @brief Turn direction 90 deg clockwise.
 */
static inline Direction dir_right(Direction dir) {
    return (Direction)((dir + 1) % 4);
}

/**
 * @brief Turn direction 90 deg counter-clockwise.
 */
static inline Direction dir_left(Direction dir) {
    return (Direction)((dir + 3) % 4);
}

#endif /* FIXED_MAZE_H */
