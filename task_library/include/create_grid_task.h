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

// This is a structure that defines the "CREATE_GRID" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "Plan-and-Control" Task view of "data"
  lane_t     position;
  unsigned   in_data_size;
  uint8_t    in_data[MAX_CREATE_GRID_IN_DATA_SIZE];
  unsigned   occ_x_dim;
  unsigned   occ_y_dim;
  uint8_t    occ_grid[MAX_OCC_X*MAX_OCC_Y];
} create_grid_data_struct_t;


typedef struct {
  struct timeval call_start;
  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
} create_grid_timing_data_t;


void print_create_grid_metadata_block_contents(task_metadata_block_t* mb);

void output_create_grid_task_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_task_type, unsigned total_accel_types);

void execute_on_cpu_create_grid_accelerator(task_metadata_block_t* task_metadata_block);
void execute_on_hwr_vit_create_grid_accelerator(task_metadata_block_t* task_metadata_block);
void execute_on_hwr_fft_create_grid_accelerator(task_metadata_block_t* task_metadata_block);
void execute_on_hwr_cv_create_grid_accelerator(task_metadata_block_t* task_metadata_block);

void set_up_create_grid_task_on_accel_profile_data();

task_metadata_block_t* set_up_create_grid_task(scheduler_datastate_block_t* sptr,
					     task_type_t create_grid_task_type, task_criticality_t crit_level,
					     bool use_auto_finish, int32_t dag_id, va_list var_list);

void create_grid_auto_finish_routine(task_metadata_block_t* mb);
void finish_create_grid_execution(task_metadata_block_t* create_grid_metadata_block, va_list var_list); //vehicle_state_t* new_vehicle_state);

#endif
