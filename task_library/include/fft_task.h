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

#ifndef H_FFT_TASK_INCLUDE_H
#define H_FFT_TASK_INCLUDE_H

#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include "base_task_types.h"
#include "scheduler.h"

//// This is the number of fft samples (the log of the samples, e.g. 10 = 1024
//// samples, 14 = 16k-samples)
//extern unsigned crit_fft_samples_set;

// This is a structure that defines the "FFT" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting  the data space.
typedef struct {                // The "FFT" Task view of "data"
  int32_t log_nsamples;         // The Log2 of the number of samples in this FFT
  float theData[2 * (1 << 14)]; // MAx supported samples (2^14) * 2 float per complex input/output
} fft_data_struct_t;

// The following structures are for timing analysis (per job type)
typedef struct {
  struct timeval call_start;
  struct timeval fft_start;
  struct timeval fft_br_start;
  struct timeval bitrev_start;
  struct timeval fft_cvtin_start;
  struct timeval fft_comp_start;
  struct timeval fft_cvtout_start;
  struct timeval cdfmcw_start;

  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_br_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t bitrev_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_cvtin_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_comp_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_cvtout_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t cdfmcw_sec[SCHED_MAX_ACCEL_TYPES];

  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_br_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t bitrev_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_cvtin_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_comp_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t fft_cvtout_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t cdfmcw_usec[SCHED_MAX_ACCEL_TYPES];
} fft_timing_data_t;

void print_fft_metadata_block_contents(/*task_metadata_entry*/ void* mb);

void set_up_fft_task_on_accel_profile_data();

void init_fft_parameters(unsigned n, uint32_t log_nsamples);

void output_fft_task_type_run_stats(void *sptr_ptr, unsigned my_task_type,
                                    unsigned total_accel_types);

void execute_hwr_fft_accelerator(void *task_metadata_block);
void execute_cpu_fft_accelerator(void *task_metadata_block);

void *set_up_fft_task(void *sptr, task_type_t fft_task_type,
                      task_criticality_t crit_level, bool use_auto_finish,
                      int32_t dag_id, int32_t task_id, void *);

void finish_fft_execution(void *fft_metadata_block, void *); // float* results);

#endif
