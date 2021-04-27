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

#ifndef H_CV_TASK_INCLUDE_H
#define H_CV_TASK_INCLUDE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include "base_types.h"

// Some Profiling Data:
//#define usecHwrCV   150000

// This is a structure that defines the "CV" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "CV" Task view of "data"
  label_t object_label; // The deteremined label of the object in the image
}  cv_data_struct_t;


typedef struct {
  struct timeval call_start;
  struct timeval parse_start;
  struct timeval time_val[MAX_TASK_TIMING_SETS-2];

  uint64_t call_sec[MAX_ACCEL_TYPES];
  uint64_t parse_sec[MAX_ACCEL_TYPES];
  uint64_t time_sec[(MAX_TASK_TIMING_SETS-2)*MAX_ACCEL_TYPES];

  uint64_t call_usec[MAX_ACCEL_TYPES];
  uint64_t parse_usec[MAX_ACCEL_TYPES];
  uint64_t time_usec[(MAX_TASK_TIMING_SETS-2)*MAX_ACCEL_TYPES];
} cv_timing_data_t;

// These are some "fake" times (models the execution of CV timing)
extern unsigned cv_cpu_run_time_in_usec;
extern unsigned cv_fake_hwr_run_time_in_usec;

void print_cv_metadata_block_contents(task_metadata_block_t* mb);

void output_cv_task_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_task_type, unsigned total_accel_types);

void execute_hwr_cv_accelerator(task_metadata_block_t* task_metadata_block);
void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block);

#endif
