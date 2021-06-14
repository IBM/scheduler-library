/*
 * Copyright 2021 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef H_OCC_GRID_H
#define H_OCC_GRID_H

#include <stdint.h>
#include <stdbool.h>
#include "base_task_types.h"

#define OCC_GRID_UNKNOWN_VAL     0
#define OCC_GRID_NO_OBSTACLE_VAL 1
#define OCC_GRID_OBSTACLE_VAL    2
#define OCC_GRID_MY_CAR_VAL      3

extern bool show_local_occ_grid;
extern bool show_remote_occ_grid;
extern bool show_fused_occ_grid;
extern bool show_side_by_occ_grids;

extern char* occ_grid_from_value_str[4];

extern unsigned MAX_GRID_DIST_FAR_IDX;
//extern unsigned GRID_DIST_STEP_SIZE;
#define GRID_DIST_STEP_SIZE 5

//extern unsigned OCC_GRID_X_DIM;
//extern unsigned OCC_GRID_Y_DIM;
#define OCC_GRID_X_DIM  NUM_LANES
#define OCC_GRID_Y_DIM  101

extern unsigned OCC_NEXT_LANE_CLOSE;
extern unsigned OCC_NEXT_LANE_FAR[3];

extern unsigned MY_CAR_OCC_GRID_SIZE;

extern uint8_t local_occ_grid[OCC_GRID_X_DIM][OCC_GRID_Y_DIM];
extern uint8_t remote_occ_grid[OCC_GRID_X_DIM][OCC_GRID_Y_DIM];
extern uint8_t fused_occ_grid[OCC_GRID_X_DIM][OCC_GRID_Y_DIM];

void print_local_occupancy_grid();
void print_remote_occupancy_grid();
void print_fused_occupancy_grid();
void print_side_by_side_occupancy_grids();

#endif
