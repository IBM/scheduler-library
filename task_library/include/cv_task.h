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

#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include "base_task_types.h"
#include "scheduler.h"

// Some Profiling Data:
//#define usecHwrCV   150000

//CV: cv_tr_label
struct cv_io_t {
  label_t in_label;
  label_t * obj_label;
  size_t obj_label_size;
  cv_io_t(label_t in_label, label_t * obj_label, size_t obj_label_size) :
    in_label(in_label), obj_label(obj_label), obj_label_size(obj_label_size) {}
};

// This is a structure that defines the "CV" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "CV" Task view of "data"
  label_t object_label; // The deteremined label of the object in the image
} cv_data_struct_t;

typedef struct {
  struct timeval call_start;
  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];

  struct timeval nvdla_start;
  uint64_t nvdla_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t nvdla_usec[SCHED_MAX_ACCEL_TYPES];

  struct timeval parse_start;
  uint64_t parse_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t parse_usec[SCHED_MAX_ACCEL_TYPES];
} cv_timing_data_t;

// These are some "fake" times (models the execution of CV timing)
extern unsigned cv_cpu_run_time_in_usec;
extern unsigned cv_fake_hwr_run_time_in_usec;

extern "C" void print_cv_metadata_block_contents(void * mb);

extern "C" void output_cv_task_type_run_stats(void * sptr, unsigned my_task_type, unsigned total_accel_types);

extern "C" void execute_hwr_cv_accelerator(void * cv_io_ptr);
extern "C" void execute_cpu_cv_accelerator(void * cv_io_ptr);

void set_up_cv_task_on_accel_profile_data();

extern "C" void cv_auto_finish_routine(void * mb);
extern "C" void finish_cv_execution(void * fft_metadata_block, void * args); // label_t* out_label);
extern std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> cv_profile;
#endif
