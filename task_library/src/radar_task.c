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

#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

//#define VERBOSE
#include "radar_task.h"
#include "scheduler.h"
#include "verbose.h"

// CONSTANTS
#define RADAR_c 300000000.0 // Speed of Light in Meters/Sec
#define RADAR_threshold -100;

// This now illustrates the use of the "task metadata" to transfer information
// for a radar (FFT) operation.
//  NOTE: We request a metadata block form the scheduler -- if one is not
//  currently available, then what?
//     OPTIONS: 1. Wait for one to become available
//              2. Purge a lower-criticality task (if there is one), else wait
//              3. Drop task if this is lowest-criticality (?), else wait
//   To make the task independent of the calling program, we need to copy over
//   the data into the metadata block
//      This treats the scheduler like an off-load accelerator, in many ways.
//   Then we should try to make thes CPU accelerators run in independent threads
//   (pthreads, I think)?
//      This will let us refine the non-blocking behavior, and start the more
//      detailed behavior of the
//        scheduler implementation (i.e. ranking, queue management, etc.)

// We declare this for all possible legal RADAR log_nsamples inputs, but RADAR
// tasks can only be 1k or 16k samples
uint64_t radar_profile[15][SCHED_MAX_ACCEL_TYPES];

void set_up_radar_task_on_accel_profile_data() {
  for (int li = 0; li < SCHED_MAX_ACCEL_TYPES; li++) {
    for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
      radar_profile[li][ai] = ACINFPROF;
      radar_profile[li][ai] = ACINFPROF;
    }
  }
#ifdef COMPILE_TO_ESP
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  radar_profile[10][SCHED_CPU_ACCEL_T] = 23000;
  radar_profile[10][SCHED_EPOCHS_1D_FFT_ACCEL_T] = 6000;
  radar_profile[14][SCHED_CPU_ACCEL_T] = 600000;
  radar_profile[14][SCHED_EPOCHS_1D_FFT_ACCEL_T] = 143000;
#else
  // This is for out x86 all-software runs...
  radar_profile[10][SCHED_CPU_ACCEL_T] = 50;
  radar_profile[14][SCHED_CPU_ACCEL_T] = 1250;
#endif
  DEBUG(
      printf("\n%15s : %18s %18s %18s %18s\n", "PROFILES", "CPU", "VIT-HWR",
             "FFT-HWR", "CV-HWR");
      printf("%15s :", "radar_profile[0]");
      for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES;
           ai++) { printf(" 0x%016lx", radar_profile[0][ai]); } printf("\n");
      printf("%15s :", "radar_profile[1]");
      for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES;
           ai++) { printf(" 0x%016lx", radar_profile[1][ai]); } printf("\n");
      printf("\n"));
}

/*task_metadata_block_t*/ void *
set_up_radar_task(/*scheduler_datastate_block_t*/ void *sptr_ptr,
                  task_type_t radar_task_type, task_criticality_t crit_level,
                  bool use_auto_finish, int32_t dag_id, ...) {

  va_list var_list;
  va_start(var_list, dag_id);
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t *)sptr_ptr;
#ifdef TIME
  gettimeofday(&start_exec_rad, NULL);
#endif
  // uint32_t log_nsamples, float * inputs
  uint32_t log_nsamples = va_arg(var_list, uint32_t);
  float *inputs = va_arg(var_list, float *);

  // Request a MetadataBlock (for an RADAR task at Critical Level)
  task_metadata_block_t *radar_mb_ptr = NULL;
  DEBUG(printf("Calling get_task_metadata_block for Critical RADAR-Task %u\n",
               radar_task_type));
  do {
    radar_mb_ptr = get_task_metadata_block(
        sptr, dag_id, radar_task_type, crit_level, radar_profile[log_nsamples]);
    // usleep(get_mb_holdoff);
  } while (0); //(*mb_ptr == NULL);
#ifdef TIME
  struct timeval got_time;
  gettimeofday(&got_time, NULL);
  exec_get_rad_sec += got_time.tv_sec - start_exec_rad.tv_sec;
  exec_get_rad_usec += got_time.tv_usec - start_exec_rad.tv_usec;
#endif
  if (radar_mb_ptr == NULL) {
    // We ran out of metadata blocks -- PANIC!
    printf("Out of metadata blocks for FFT -- PANIC Quit the run (for now)\n");
    dump_all_metadata_blocks_states(sptr);
    exit(-4);
  }
  if (use_auto_finish) {
    radar_mb_ptr->atFinish =
        sptr->auto_finish_task_function[radar_task_type]; // get_auto_finish_routine(sptr,
                                                          // radar_task_type);
  } else {
    radar_mb_ptr->atFinish = NULL;
  }

  DEBUG(printf("MB%u In start_radar_execution\n", radar_mb_ptr->block_id));

  fft_timing_data_t *radar_timings_p = (fft_timing_data_t *)&(
      radar_mb_ptr->task_timings[radar_mb_ptr->task_type]);
  fft_data_struct_t *radar_data_p =
      (fft_data_struct_t *)(radar_mb_ptr->data_space);
  radar_data_p->log_nsamples = log_nsamples;
  radar_mb_ptr->data_size = 2 * (1 << log_nsamples) * sizeof(float);
  // Copy over our task data to the MetaData Block
  // radar_mb_ptr->data = (uint8_t*)data;
  float *mdataptr = (float *)radar_data_p->theData;
  DEBUG(printf("scpdff: log_n = %u data_size = %u mdatp = %p\n",
               radar_data_p->log_nsamples, radar_mb_ptr->data_size, mdataptr));
  for (int i = 0; i < 2 * (1 << log_nsamples); i++) {
    mdataptr[i] = inputs[i];
  }

#ifdef INT_TIME
  gettimeofday(&(radar_timings_p->call_start), NULL);
#endif
  // This now ends this block -- we've kicked off execution
  return radar_mb_ptr;
}

// NOTE: This routine DOES NOT copy out the FFT data results --
//   this only computes the distance to nearest obstacle, and returns that.
distance_t
do_finish_radar_computations(task_metadata_block_t *radar_metadata_block) {
  int tidx = radar_metadata_block->accelerator_type;
  fft_timing_data_t *radar_timings_p = (fft_timing_data_t *)&(
      radar_metadata_block->task_timings[radar_metadata_block->task_type]);
  fft_data_struct_t *radar_data_p =
      (fft_data_struct_t *)(radar_metadata_block->data_space);
  uint32_t fft_log_nsamples = radar_data_p->log_nsamples;
  float *data = (float *)radar_data_p->theData;
#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  radar_timings_p->call_sec[tidx] +=
      stop_time.tv_sec - radar_timings_p->call_start.tv_sec;
  radar_timings_p->call_usec[tidx] +=
      stop_time.tv_usec - radar_timings_p->call_start.tv_usec;

  gettimeofday(&(radar_timings_p->cdfmcw_start), NULL);
#endif // INT_TIME

  unsigned RADAR_N = 0;    // The number of samples (2^LOGN)
  float RADAR_fs = 0.0;    // Sampling Frequency
  float RADAR_alpha = 0.0; // Chirp rate (saw-tooth)
  // float   RADAR_psd_threshold = 1e-10*pow(8192,2);  // ~= 0.006711 and 450 ~=
  // 0.163635 in 16K
  float RADAR_psd_threshold = 0.0067108864;

  switch (fft_log_nsamples) {
  case 10:
    RADAR_fs = 204800.0;
    RADAR_alpha = 30000000000.0;
    RADAR_psd_threshold = 0.000316; // 1e-10*pow(8192,2);  // 450m ~= 0.000638
                                    // so psd_thres ~= 0.000316 ?
    break;
  case 14:
    RADAR_fs = 32768000.0;
    RADAR_alpha = 4800000000000.0;
    RADAR_psd_threshold = 1e-10 * pow(8192, 2);
    break;
  default:
    printf("ERROR : Unsupported Log-N FFT Samples Value: %u\n",
           fft_log_nsamples);
    exit(-1);
  }
  RADAR_N = (1 << fft_log_nsamples);

  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int i;
  float temp;
  for (i = 0; i < RADAR_N; i++) {
    temp = (pow(data[2 * i], 2) + pow(data[2 * i + 1], 2)) / 100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = i;
    }
  }
  float distance =
      ((float)(max_index * ((float)RADAR_fs) / ((float)(RADAR_N)))) * 0.5 *
      RADAR_c / ((float)(RADAR_alpha));
  // printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance,
  // max_psd, max_index);

  // printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));
  if (max_psd <= RADAR_psd_threshold) {
    distance = INFINITY;
  }

#ifdef INT_TIME
  struct timeval cdfmcw_stop;
  gettimeofday(&cdfmcw_stop, NULL);
  radar_timings_p->cdfmcw_sec[tidx] +=
      cdfmcw_stop.tv_sec - radar_timings_p->cdfmcw_start.tv_sec;
  radar_timings_p->cdfmcw_usec[tidx] +=
      cdfmcw_stop.tv_usec - radar_timings_p->cdfmcw_start.tv_usec;
#endif // INT_TIME
  return distance;
}

// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
void radar_auto_finish_routine(/*task_metadata_block_t*/ void *mb_ptr) {
  task_metadata_block_t *mb = (task_metadata_block_t *)mb_ptr;
  TDEBUG(scheduler_datastate_block_t *sptr = mb->scheduler_datastate_pointer;
         printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",
                mb->block_id, sptr->task_name_str[mb->task_type],
                sptr->task_criticality_str[mb->crit_level],
                sptr->accel_name_str[mb->accelerator_type],
                mb->accelerator_id));
  // Call this so we get final stats (call-time)
  distance_t distance = do_finish_radar_computations(mb);

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n", mb->block_id));
  free_task_metadata_block(mb);
}

// NOTE: This routine DOES NOT copy out the FFT data results --
//   this only computes the distance to nearest obstacle, and returns that.
void finish_radar_execution(/*task_metadata_block_t*/void *radar_metadata_block_ptr,
                            ...) {
  va_list var_list;
  va_start(var_list, radar_metadata_block_ptr);
  task_metadata_block_t *radar_metadata_block = (task_metadata_block_t*) radar_metadata_block_ptr;
  // float* obj_dist)
  float *obj_dist = va_arg(var_list, float *);

  distance_t distance = do_finish_radar_computations(radar_metadata_block);
  *obj_dist = distance;

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n",
               radar_metadata_block->block_id));
  free_task_metadata_block(radar_metadata_block);
}
