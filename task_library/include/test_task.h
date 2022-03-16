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

#ifndef H_TEST_TASK_INCLUDE_H
#define H_TEST_TASK_INCLUDE_H

#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include "base_task_types.h"

extern unsigned num_Crit_test_tasks;
extern unsigned num_Base_test_tasks;

// These are some "fake" times (models the execution of TEST timing)
extern unsigned test_on_cpu_run_time_in_usec;
extern unsigned test_on_hwr_fft_run_time_in_usec;
extern unsigned test_on_hwr_vit_run_time_in_usec;
extern unsigned test_on_hwr_cv_run_time_in_usec;

// This is a structure that defines the "TEST" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "TEST" Task view of "data"
  size_t in_size;
  label_t object_label; // The determined label of the object in the image
} test_data_struct_t;

typedef struct {
  struct timeval call_start;
  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
} test_timing_data_t;

void print_test_metadata_block_contents(void *mb);

void output_test_task_type_run_stats(void *sptr, unsigned my_task_type, unsigned total_accel_types);

void execute_on_cpu_test_accelerator(void *task_metadata_block);
void execute_on_hwr_vit_test_accelerator(void *task_metadata_block);
void execute_on_hwr_fft_test_accelerator(void *task_metadata_block);
void execute_on_hwr_cv_test_accelerator(void *task_metadata_block);

void set_up_test_task_on_accel_profile_data();

void *set_up_test_task(void *sptr, task_type_t test_task_type,
                       task_criticality_t crit_level, bool use_auto_finish,
                       int32_t dag_id, int32_t task_id, void *);

void test_auto_finish_routine(void *mb);
void finish_test_execution(void *test_metadata_block, void *);

extern std::map<uint64_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> test_profile;
#endif
