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

#ifndef H_SCHEDULER_VIT_INTERFACE_H
#define H_SCHEDULER_VIT_INTERFACE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include "base_types.h"

//#include "scheduler.h"

// Some Profiling Data:
#define usecHwrVIT0   5950
#define usecHwrVIT1  67000
#define usecHwrVIT2 135000
#define usecHwrVIT3 191000

// This is a structure that defines the "Viterbi" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "Viterbi" view of "data"
  int32_t n_data_bits;
  int32_t n_cbps;
  int32_t n_traceback;
  int32_t psdu_size;
  int32_t inMem_size;    // The first inMem_size bytes of theData are the inMem (input memories)
  int32_t inData_size;   // The next inData_size bytes of theData are the inData (input data)
  int32_t outData_size;  // The next outData_size bytes of theData are the outData (output data)
  uint8_t theData[64*1024]; // This is larger than needed (~24780 + 18585) but less than FFT requires (so okay)
}  viterbi_data_struct_t;

typedef struct {
  struct timeval dodec_start;
  struct timeval depunc_start;
  struct timeval time_val[16-2];

  unsigned comp_by[MAX_TASK_TARGETS];

  // 0 = timings for cpu_accel_T and 1 = vit_hwr_accel_t
  uint64_t dodec_sec[MAX_TASK_TARGETS];
  uint64_t depunc_sec[MAX_TASK_TARGETS];
  uint64_t time_sec[(16-2)*MAX_TASK_TARGETS];

  uint64_t dodec_usec[MAX_TASK_TARGETS];
  uint64_t depunc_usec[MAX_TASK_TARGETS];
  uint64_t time_usec[(16-2)*MAX_TASK_TARGETS];
} vit_timing_data_t;

#endif
