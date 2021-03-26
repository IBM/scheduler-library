/*
 * Copyright 2020 IBM
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

#ifndef H_SCHEDULER_CV_INTERFACE_H
#define H_SCHEDULER_CV_INTERFACE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include "base_types.h"

//#include "scheduler.h"

// Some Profiling Data:
#define usecHwrCV   150000

// This is a structure that defines the "CV" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "CV" Task view of "data"
  label_t object_label; // The deteremined label of the object in the image
}  cv_data_struct_t;


typedef struct {
  struct timeval call_start;
  struct timeval parse_start;

  unsigned comp_by[MAX_TASK_TARGETS];

  // 0 = timings for cpu_accel_T and 1 = cv_hwr_accel_t
  uint64_t call_sec[MAX_TASK_TARGETS];
  uint64_t parse_sec[MAX_TASK_TARGETS];
  uint64_t time_sec[(16-2)*MAX_TASK_TARGETS];

  uint64_t parse_usec[MAX_TASK_TARGETS];
  uint64_t call_usec[MAX_TASK_TARGETS];
  uint64_t time_usec[(16-2)*MAX_TASK_TARGETS];
} cv_timing_data_t;

// These are some "fake" times (models the execution of CV timing)
extern unsigned cv_cpu_run_time_in_usec;
extern unsigned cv_fake_hwr_run_time_in_usec;

#endif
