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
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

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
#include "verbose.h"

#include "fuse_grids_task.h"
#include "scheduler.h"

void print_fuse_grids_metadata_block_contents(task_metadata_block_t *mb) {
  print_base_metadata_block_contents(mb);
  fuse_grids_data_struct_t *fuse_grids_data_p = (fuse_grids_data_struct_t *)(mb->data_space);
  printf("  PLAN_CTRL: my_lane       = %u\n", fuse_grids_data_p->my_lane);
  printf("  PLAN_CTRL: your_lane     = %u\n", fuse_grids_data_p->your_lane);
  printf("  PLAN_CTRL: occ_x_dim     = %u\n", fuse_grids_data_p->occ_x_dim);
  printf("  PLAN_CTRL: occ_y_dim     = %u\n", fuse_grids_data_p->occ_y_dim);
  printf("  PLAN_CTRL: my_occ_grid   = %p\n", fuse_grids_data_p->my_occ_grid);
  printf("  PLAN_CTRL: your_occ_grid = %p\n", fuse_grids_data_p->your_occ_grid);
}

void output_fuse_grids_task_type_run_stats(scheduler_datastate_block_t *sptr,
                                          unsigned my_task_type,
                                          unsigned total_accel_types) {

  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n", my_task_type, sptr->task_name_str[my_task_type], sptr->freed_metadata_blocks[my_task_type], total_accel_types);
  // The PLAN_CTRL/CNN Task Timing Info
  unsigned total_fuse_grids_comp_by[total_accel_types + 1];
  uint64_t total_fuse_grids_call_usec[total_accel_types + 1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_fuse_grids_comp_by[ai] = 0;
    total_fuse_grids_call_usec[ai] = 0;
  }
  for (int ai = 0; ai < total_accel_types; ai++) {
    if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type, sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      fuse_grids_timing_data_t *fuse_grids_timings_p = (fuse_grids_timing_data_t *)&(sptr->master_metadata_pool[bi].task_timings[my_task_type]);
      unsigned this_comp_by = (unsigned)(sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
      uint64_t this_fuse_grids_call_usec = (uint64_t)(fuse_grids_timings_p->call_sec[ai]) * 1000000 + (uint64_t)(fuse_grids_timings_p->call_usec[ai]);
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu usec\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_fuse_grids_call_usec);
      } else {
        if ((this_comp_by + this_fuse_grids_call_usec) != 0) {
          printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_fuse_grids_call_usec);
        }
      }
      // Per acceleration (CPU, HWR)
      total_fuse_grids_comp_by[ai] += this_comp_by;
      total_fuse_grids_call_usec[ai] += this_fuse_grids_call_usec;
      // Overall Total
      total_fuse_grids_comp_by[total_accel_types] += this_comp_by;
      total_fuse_grids_call_usec[total_accel_types] += this_fuse_grids_call_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  }   // for (ai = 0 .. total_accel_types)

  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type, sptr->task_name_str[my_task_type]);
  printf("     CNN-call  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fuse_grids_call_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fuse_grids_comp_by[ai], total_fuse_grids_call_usec[ai], avg);
  }
  {
    double avg = (double)total_fuse_grids_call_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fuse_grids_comp_by[total_accel_types], total_fuse_grids_call_usec[total_accel_types], avg);
  }
}

void execute_on_cpu_fuse_grids_accelerator(task_metadata_block_t *task_metadata_block) {
  DEBUG(printf("In execute_on_cpu_fuse_grids_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level));
  int aidx = task_metadata_block->accelerator_type;
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  fuse_grids_data_struct_t *fuse_grids_data_p = (fuse_grids_data_struct_t *)(task_metadata_block->data_space);
  fuse_grids_timing_data_t *fuse_grids_timings_p = (fuse_grids_timing_data_t *)&(task_metadata_block->task_timings[task_metadata_block->task_type]);

  DEBUG(printf("In the fuse_grids task : position %u input_data %p occ_grid %p\n", 
               fuse_grids_data_p->position, 
               fuse_grids_data_p->input_data,
               fuse_grids_data_p->occ_grid));
  
 #ifdef INT_TIME
  gettimeofday(&(fuse_grids_timings_p->call_start), NULL);
 #endif


#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fuse_grids_timings_p->call_sec[aidx] += stop_time.tv_sec - fuse_grids_timings_p->call_start.tv_sec;
  fuse_grids_timings_p->call_usec[aidx] += stop_time.tv_usec - fuse_grids_timings_p->call_start.tv_usec;
#endif


  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

uint64_t fuse_grids_profile[SCHED_MAX_ACCEL_TYPES];
void set_up_fuse_grids_task_on_accel_profile_data() {
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    fuse_grids_profile[ai] = ACINFPROF;
  }
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  fuse_grids_profile[SCHED_CPU_ACCEL_T] = 1; // Picked a small value...

  DEBUG(printf("\n%15s : %18s %18s %18s %18s\n", "PROFILES", "CPU", "VIT-HWR", "FFT-HWR", "CV-HWR");
        printf("%15s :", "pnc_profile");
        for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES;
             ai++) { printf(" 0x%016lx", fuse_grids_profile[ai]); } printf("\n");
        printf("\n"));
}

task_metadata_block_t *set_up_fuse_grids_task(scheduler_datastate_block_t *sptr,
					       task_type_t fuse_grids_task_type, task_criticality_t crit_level,
					       bool use_auto_finish, int32_t dag_id, va_list var_list)
{
//unsigned time_step, unsigned repeat_factor,
//  label_t object_label, distance_t object_dist, message_t safe_lanes_msg,
//  vehicle_state_t vehicle_state) {
 #ifdef TIME
  gettimeofday(&start_exec_pandc, NULL);
 #endif
  lane_t my_lane = va_arg(var_list, lane_t);
  lane_t your_lane = va_arg(var_list, lane_t);
  unsigned occ_x_dim = va_arg(var_list, unsigned);
  unsigned occ_y_dim = va_arg(var_list, unsigned);
  uint8_t* my_occ_grid = va_arg(var_list, uint8_t*);
  uint8_t* your_occ_grid = va_arg(var_list, uint8_t*);
  
  // Request a MetadataBlock (for an PLAN_CTRL task at Critical Level)
  task_metadata_block_t *fuse_grids_mb_ptr = NULL;
  DEBUG(printf("Calling get_task_metadata_block for Critical PLAN_CTRL-Task %u\n", fuse_grids_task_type));
  do {
    fuse_grids_mb_ptr = get_task_metadata_block(sptr, dag_id, fuse_grids_task_type, crit_level, fuse_grids_profile);
    // usleep(get_mb_holdoff);
  } while (0); //(*mb_ptr == NULL);
#ifdef TIME
  struct timeval got_time;
  gettimeofday(&got_time, NULL);
  exec_get_pandc_sec += got_time.tv_sec - start_exec_pandc.tv_sec;
  exec_get_pandc_usec += got_time.tv_usec - start_exec_pandc.tv_usec;
#endif
  if (fuse_grids_mb_ptr == NULL) {
    // We ran out of metadata blocks -- PANIC!
    printf("Out of metadata blocks for PLAN_CTRL -- PANIC Quit the run (for now)\n");
    dump_all_metadata_blocks_states(sptr);
    exit(-4);
  }
  if (use_auto_finish) {
    fuse_grids_mb_ptr->atFinish = sptr->auto_finish_task_function[fuse_grids_task_type]; // get_auto_finish_routine(sptr, fuse_grids_task_type);
  } else {
    fuse_grids_mb_ptr->atFinish = NULL;
  }
  DEBUG(printf("MB%u In start_fuse_grids_execution\n", fuse_grids_mb_ptr->block_id));

  fuse_grids_timing_data_t *fuse_grids_timings_p = (fuse_grids_timing_data_t *)&(fuse_grids_mb_ptr->task_timings[fuse_grids_mb_ptr->task_type]);
  fuse_grids_data_struct_t *fuse_grids_data_p = (fuse_grids_data_struct_t *)(fuse_grids_mb_ptr->data_space);
  // Set the inputs for the plan-and-control task
  fuse_grids_data_p->my_lane = my_lane; // The current car position (mine)
  fuse_grids_data_p->your_lane = your_lane; // The current car position (mine)
  fuse_grids_data_p->occ_x_dim = occ_x_dim;
  fuse_grids_data_p->occ_y_dim = occ_y_dim;
  for (int i = 0; i < occ_x_dim * occ_y_dim; i++) {
    fuse_grids_data_p->my_occ_grid[i] = my_occ_grid[i];
    fuse_grids_data_p->your_occ_grid[i] = your_occ_grid[i];
  }
  DEBUG(printf("   Set MB%u my_lane %u your_lane %u x_dim %u y_dim %u\n", 
               fuse_grids_mb_ptr->block_id,
	       fuse_grids_data_p->my_laneposition, fuse_grids_data_p->you_lane,
               fuse_grids_data_p->occ_x_dim, fuse_grids_data_p->occ_y_dim));

#ifdef INT_TIME
  gettimeofday(&(fuse_grids_timings_p->call_start), NULL);
#endif
  return fuse_grids_mb_ptr;
}

// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
void fuse_grids_auto_finish_routine(task_metadata_block_t *mb) {
  TDEBUG(scheduler_datastate_block_t *sptr = mb->scheduler_datastate_pointer;
         printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",
                mb->block_id, sptr->task_name_str[mb->task_type],
                sptr->task_criticality_str[mb->crit_level],
                sptr->accel_name_str[mb->accelerator_type],
                mb->accelerator_id));
  DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
  free_task_metadata_block(mb);
}

// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the PLAN_CTRL results (As we used
//   to) but this seems un-necessary since we only want the final "distance"
//   really.
void finish_fuse_grids_execution(task_metadata_block_t *fuse_grids_metadata_block, va_list var_list)
{
  // vehicle_state_t *new_vehicle_state)
  vehicle_state_t* new_vehicle_state = va_arg(var_list, vehicle_state_t*);
  unsigned* position = va_arg(var_list, unsigned *);
  unsigned* x_dim = va_arg(var_list, unsigned *);
  unsigned* y_dim = va_arg(var_list, unsigned *);
  uint8_t*  occ_grid = va_arg(var_list, uint8_t *);

  int tidx = fuse_grids_metadata_block->accelerator_type;
  fuse_grids_timing_data_t *fuse_grids_timings_p = (fuse_grids_timing_data_t *)&(fuse_grids_metadata_block->task_timings[fuse_grids_metadata_block->task_type]);
  fuse_grids_data_struct_t *fuse_grids_data_p = (fuse_grids_data_struct_t *)(fuse_grids_metadata_block->data_space);
#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fuse_grids_timings_p->call_sec[tidx] += stop_time.tv_sec - fuse_grids_timings_p->call_start.tv_sec;
  fuse_grids_timings_p->call_usec[tidx] += stop_time.tv_usec - fuse_grids_timings_p->call_start.tv_usec;
#endif // INT_TIME

  *position = fuse_grids_data_p->my_lane;
  *x_dim = fuse_grids_data_p->occ_x_dim;
  *y_dim = fuse_grids_data_p->occ_y_dim;
  for (int i = 0; i < fuse_grids_data_p->occ_x_dim * fuse_grids_data_p->occ_y_dim; i++) {
    occ_grid[i] = fuse_grids_data_p->my_occ_grid[i];
  }

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n", fuse_grids_metadata_block->block_id));
  free_task_metadata_block(fuse_grids_metadata_block);
}
