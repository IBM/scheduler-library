/*
 * Copyright 2019 IBM
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

#ifndef H_CREATE_GRID_TASK_INCLUDE_H
#define H_CREATE_GRID_TASK_INCLUDE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include "base_task_types.h"
#include "scheduler.h"

#define MAX_CREATE_GRID_IN_DATA_SIZE 2000
#define MAX_OCC_X  7
#define MAX_OCC_Y  100

struct create_grid_io_t {
  lane_t lane;
  unsigned in_data_size;
  uint8_t* in_data;
  unsigned occ_x_dim;
  unsigned occ_y_dim;
  uint8_t* occ_grid;
  lane_t * position;
  unsigned * x_dim;
  unsigned * y_dim;

  create_grid_io_t(
    lane_t lane, unsigned in_data_size, uint8_t* in_data, unsigned occ_x_dim, unsigned occ_y_dim,
    uint8_t* occ_grid):
    lane(lane), in_data_size(in_data_size), in_data(in_data), occ_x_dim(occ_x_dim),
    occ_y_dim(occ_y_dim), occ_grid(occ_grid), position(position), x_dim(x_dim), y_dim(y_dim) {}
};

// This is a structure that defines the "CREATE_GRID" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "Plan-and-Control" Task view of "data"
  lane_t     position;
  unsigned   in_data_size;
  uint8_t    in_data[MAX_CREATE_GRID_IN_DATA_SIZE];
  unsigned   occ_x_dim;
  unsigned   occ_y_dim;
  uint8_t    occ_grid[MAX_OCC_X * MAX_OCC_Y];
} create_grid_data_struct_t;

typedef struct {
  struct timeval call_start;
  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
} create_grid_timing_data_t;


void print_create_grid_metadata_block_contents(task_metadata_entry* mb);

void output_create_grid_task_type_run_stats(scheduler_datastate* sptr, unsigned my_task_type,
    unsigned total_accel_types);

void execute_on_cpu_create_grid_accelerator(task_metadata_entry* task_metadata_block);
void execute_on_hwr_vit_create_grid_accelerator(task_metadata_entry* task_metadata_block);
void execute_on_hwr_fft_create_grid_accelerator(task_metadata_entry* task_metadata_block);
void execute_on_hwr_cv_create_grid_accelerator(task_metadata_entry* task_metadata_block);

void set_up_create_grid_task_on_accel_profile_data();

task_metadata_entry* set_up_create_grid_task(scheduler_datastate* sptr,
    task_type_t create_grid_task_type, task_criticality_t crit_level,
    bool use_auto_finish, int32_t dag_id, void * args);

void create_grid_auto_finish_routine(task_metadata_entry* mb);
void finish_create_grid_execution(task_metadata_entry* create_grid_metadata_block,
                                  void * args); //vehicle_state_t* new_vehicle_state);

#endif
