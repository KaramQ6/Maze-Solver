/**
 * @file fixed_maze.c
 * @brief Zero-allocation bit-packed 10x10 maze grid implementation.
 */

#include "fixed_maze.h"
#include <string.h>

void maze_init(MazeGrid *grid) {
    if (!grid) return;

    memset(grid->h_walls, 0, sizeof(grid->h_walls));
    memset(grid->v_walls, 0, sizeof(grid->v_walls));

    /* Set outer horizontal boundaries: row 0 (North edge) and row 10 (South edge) */
    grid->h_walls[0] = 0x03FF;         /* 10 bits: 1111111111 (cols 0..9) */
    grid->h_walls[MAZE_ROWS] = 0x03FF; /* 10 bits */

    /* Set outer vertical boundaries: col 0 (West edge) and col 10 (East edge) */
    for (int r = 0; r < MAZE_ROWS; r++) {
        grid->v_walls[r] |= (1U << 0);          /* West perimeter */
        grid->v_walls[r] |= (1U << MAZE_COLS);  /* East perimeter (col 10) */
    }

    /* Start cell (row 9, col 0) East wall (§5.a default) */
    maze_set_wall(grid, 9, 0, DIR_EAST, true);
}

bool maze_has_wall(const MazeGrid *grid, uint8_t row, uint8_t col, Direction dir) {
    if (!grid || row >= MAZE_ROWS || col >= MAZE_COLS) {
        return true; /* Treat out-of-bounds as solid wall */
    }

    switch (dir) {
        case DIR_NORTH:
            return (grid->h_walls[row] & (1U << col)) != 0;
        case DIR_SOUTH:
            return (grid->h_walls[row + 1] & (1U << col)) != 0;
        case DIR_WEST:
            return (grid->v_walls[row] & (1U << col)) != 0;
        case DIR_EAST:
            return (grid->v_walls[row] & (1U << (col + 1))) != 0;
        default:
            return true;
    }
}

void maze_set_wall(MazeGrid *grid, uint8_t row, uint8_t col, Direction dir, bool present) {
    if (!grid || row >= MAZE_ROWS || col >= MAZE_COLS) {
        return;
    }

    uint16_t mask;
    switch (dir) {
        case DIR_NORTH:
            mask = (uint16_t)(1U << col);
            if (present) grid->h_walls[row] |= mask;
            else grid->h_walls[row] &= ~mask;
            break;
        case DIR_SOUTH:
            mask = (uint16_t)(1U << col);
            if (present) grid->h_walls[row + 1] |= mask;
            else grid->h_walls[row + 1] &= ~mask;
            break;
        case DIR_WEST:
            mask = (uint16_t)(1U << col);
            if (present) grid->v_walls[row] |= mask;
            else grid->v_walls[row] &= ~mask;
            break;
        case DIR_EAST:
            mask = (uint16_t)(1U << (col + 1));
            if (present) grid->v_walls[row] |= mask;
            else grid->v_walls[row] &= ~mask;
            break;
    }
}

bool maze_is_goal(uint8_t row, uint8_t col) {
    return (row == 4 || row == 5) && (col == 4 || col == 5);
}
