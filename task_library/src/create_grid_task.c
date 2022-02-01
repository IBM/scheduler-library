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

#include "create_grid_task.h"
#include "scheduler.h"

void print_create_grid_metadata_block_contents(task_metadata_entry *mb) {
  print_base_metadata_block_contents(mb);
  create_grid_data_struct_t *create_grid_data_p = (create_grid_data_struct_t *)(mb->data_space);
  printf("  PLAN_CTRL: position   = %u\n", create_grid_data_p->position);
  printf("  PLAN_CTRL: input_data = %p\n", create_grid_data_p->in_data);
  printf("  PLAN_CTRL: occ_grid   = %p\n", create_grid_data_p->occ_grid);
}

void output_create_grid_task_type_run_stats(scheduler_datastate *sptr,
    unsigned my_task_type,
    unsigned total_accel_types) {

  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n",
         my_task_type, sptr->task_name_str[my_task_type],
         sptr->freed_metadata_blocks[my_task_type], total_accel_types);
  // The PLAN_CTRL/CNN Task Timing Info
  unsigned total_create_grid_comp_by[total_accel_types + 1];
  uint64_t total_create_grid_call_usec[total_accel_types + 1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_create_grid_comp_by[ai] = 0;
    total_create_grid_call_usec[ai] = 0;
  }
  for (int ai = 0; ai < total_accel_types; ai++) {
    if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type,
             sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      create_grid_timing_data_t *create_grid_timings_p = (create_grid_timing_data_t *) &
          (sptr->master_metadata_pool[bi].task_timings[my_task_type]);
      unsigned this_comp_by = (unsigned)(
                                sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
      uint64_t this_create_grid_call_usec = (uint64_t)(create_grid_timings_p->call_sec[ai]) * 1000000 +
                                            (uint64_t)(create_grid_timings_p->call_usec[ai]);
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu usec\n", bi, ai, sptr->accel_name_str[ai],
               this_comp_by, this_create_grid_call_usec);
      } else {
        if ((this_comp_by + this_create_grid_call_usec) != 0) {
          printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu\n", bi, ai, sptr->accel_name_str[ai],
                 this_comp_by, this_create_grid_call_usec);
        }
      }
      // Per acceleration (CPU, HWR)
      total_create_grid_comp_by[ai] += this_comp_by;
      total_create_grid_call_usec[ai] += this_create_grid_call_usec;
      // Overall Total
      total_create_grid_comp_by[total_accel_types] += this_comp_by;
      total_create_grid_call_usec[total_accel_types] += this_create_grid_call_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  }   // for (ai = 0 .. total_accel_types)

  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type,
         sptr->task_name_str[my_task_type]);
  printf("     CNN-call  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_create_grid_call_usec[ai] / (double)
                 sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
           sptr->accel_name_str[ai], total_create_grid_comp_by[ai],
           total_create_grid_call_usec[ai], avg);
  }
  {
    double avg = (double)total_create_grid_call_usec[total_accel_types] /
                 (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
           total_create_grid_comp_by[total_accel_types],
           total_create_grid_call_usec[total_accel_types], avg);
  }
}

void execute_on_cpu_create_grid_accelerator(task_metadata_entry *task_metadata_block) {
  DEBUG(printf("In execute_on_cpu_create_grid_accelerator: MB %d  CL %d\n",
               task_metadata_block->block_id, task_metadata_block->crit_level));
  int aidx = task_metadata_block->accelerator_type;
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  create_grid_data_struct_t *create_grid_data_p = (create_grid_data_struct_t *)(
        task_metadata_block->data_space);
  create_grid_timing_data_t *create_grid_timings_p = (create_grid_timing_data_t *) &
      (task_metadata_block->task_timings[task_metadata_block->task_type]);

  DEBUG(printf("In the create_grid task : position %u input_data %p occ_grid %p\n",
               create_grid_data_p->position,
               create_grid_data_p->input_data,
               create_grid_data_p->occ_grid));

#ifdef INT_TIME
  gettimeofday(&(create_grid_timings_p->call_start), NULL);
#endif


#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  create_grid_timings_p->call_sec[aidx] += stop_time.tv_sec -
      create_grid_timings_p->call_start.tv_sec;
  create_grid_timings_p->call_usec[aidx] += stop_time.tv_usec -
      create_grid_timings_p->call_start.tv_usec;
#endif


  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

uint64_t create_grid_profile[SCHED_MAX_ACCEL_TYPES];
void set_up_create_grid_task_on_accel_profile_data() {
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    create_grid_profile[ai] = ACINFPROF;
  }
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  create_grid_profile[SCHED_CPU_ACCEL_T] = 1; // Picked a small value...

  DEBUG(printf("\n%15s : %18s %18s %18s %18s\n", "PROFILES", "CPU", "VIT-HWR", "FFT-HWR", "CV-HWR");
        printf("%15s :", "pnc_profile");
        for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES;
  ai++) { printf(" 0x%016lx", create_grid_profile[ai]); } printf("\n");
  printf("\n"));
}

task_metadata_entry *set_up_create_grid_task(scheduler_datastate *sptr,
    task_type_t create_grid_task_type, task_criticality_t crit_level,
    bool use_auto_finish, int32_t dag_id, int32_t task_id, void * args) {
//unsigned time_step, unsigned repeat_factor,
//  label_t object_label, distance_t object_dist, message_t safe_lanes_msg,
//  vehicle_state_t vehicle_state) {
#ifdef TIME
  gettimeofday(&start_exec_pandc, NULL);
#endif
  lane_t lane           = ((create_grid_input_t *) args)->lane;
  unsigned in_data_size = ((create_grid_input_t *) args)->in_data_size;
  uint8_t* in_data      = ((create_grid_input_t *) args)->in_data;
  unsigned occ_x_dim    = ((create_grid_input_t *) args)->occ_x_dim;
  unsigned occ_y_dim    = ((create_grid_input_t *) args)->occ_y_dim;
  uint8_t* occ_grid     = ((create_grid_input_t *) args)->occ_grid;

  // Request a MetadataBlock (for an PLAN_CTRL task at Critical Level)
  task_metadata_entry *create_grid_mb_ptr = NULL;
  DEBUG(printf("Calling get_task_metadata_block for Critical PLAN_CTRL-Task %u\n",
               create_grid_task_type));
  do {
    create_grid_mb_ptr = get_task_metadata_block(sptr, dag_id, task_id, create_grid_task_type,
                         crit_level, create_grid_profile);
    // usleep(get_mb_holdoff);
  } while (0); //(*mb_ptr == NULL);
#ifdef TIME
  struct timeval got_time;
  gettimeofday(&got_time, NULL);
  exec_get_pandc_sec += got_time.tv_sec - start_exec_pandc.tv_sec;
  exec_get_pandc_usec += got_time.tv_usec - start_exec_pandc.tv_usec;
#endif
  if (create_grid_mb_ptr == NULL) {
    // We ran out of metadata blocks -- PANIC!
    printf("Out of metadata blocks for PLAN_CTRL -- PANIC Quit the run (for now)\n");
    dump_all_metadata_blocks_states(sptr);
    exit(-4);
  }
  if (use_auto_finish) {
    create_grid_mb_ptr->atFinish = (void (*)(task_metadata_entry *))(
                                     sptr->auto_finish_task_function[create_grid_task_type]); // get_auto_finish_routine(sptr, create_grid_task_type);
  } else {
    create_grid_mb_ptr->atFinish = NULL;
  }
  DEBUG(printf("MB%u In start_create_grid_execution\n", create_grid_mb_ptr->block_id));

  create_grid_timing_data_t *create_grid_timings_p = (create_grid_timing_data_t *) &
      (create_grid_mb_ptr->task_timings[create_grid_mb_ptr->task_type]);
  create_grid_data_struct_t *create_grid_data_p = (create_grid_data_struct_t *)(
        create_grid_mb_ptr->data_space);
  // Set the inputs for the plan-and-control task
  create_grid_data_p->position = lane; // The current car position (mine)
  create_grid_data_p->in_data_size = in_data_size;
  for (int i = 0; i < in_data_size; i++) {
    create_grid_data_p->in_data[i] = in_data[i];
  }
  create_grid_data_p->occ_x_dim = occ_x_dim;
  create_grid_data_p->occ_y_dim = occ_y_dim;
  for (int i = 0; i < occ_x_dim * occ_y_dim; i++) {
    create_grid_data_p->occ_grid[i] = 0x0;
  }
  DEBUG(printf("   Set MB%u position %u in_size %u and x_dim %u y_dim %u\n",
               create_grid_mb_ptr->block_id,
               create_grid_data_p->position, create_grid_data_p->in_data_size,
               create_grid_data_p->occ_x_dim, create_grid_data_p->occ_y_dim));

#ifdef INT_TIME
  gettimeofday(&(create_grid_timings_p->call_start), NULL);
#endif
  return create_grid_mb_ptr;
}

// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
void create_grid_auto_finish_routine(task_metadata_entry *mb) {
  TDEBUG(scheduler_datastate *sptr = mb->scheduler_datastate_pointer;
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
void finish_create_grid_execution(task_metadata_entry *create_grid_metadata_block,
                                  va_list var_list) {
  // vehicle_state_t *new_vehicle_state)
  //TODO: Create output struct
  vehicle_state_t* new_vehicle_state = va_arg(var_list, vehicle_state_t*);
  unsigned* position = va_arg(var_list, unsigned *);
  unsigned* x_dim = va_arg(var_list, unsigned *);
  unsigned* y_dim = va_arg(var_list, unsigned *);
  uint8_t*  occ_grid = va_arg(var_list, uint8_t *);

  int tidx = create_grid_metadata_block->accelerator_type;
  create_grid_timing_data_t *create_grid_timings_p = (create_grid_timing_data_t *) &
      (create_grid_metadata_block->task_timings[create_grid_metadata_block->task_type]);
  create_grid_data_struct_t *create_grid_data_p = (create_grid_data_struct_t *)(
        create_grid_metadata_block->data_space);
#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  create_grid_timings_p->call_sec[tidx] += stop_time.tv_sec -
      create_grid_timings_p->call_start.tv_sec;
  create_grid_timings_p->call_usec[tidx] += stop_time.tv_usec -
      create_grid_timings_p->call_start.tv_usec;
#endif // INT_TIME

  *position = create_grid_data_p->position;
  *x_dim = create_grid_data_p->occ_x_dim;
  *y_dim = create_grid_data_p->occ_y_dim;
  for (int i = 0; i < create_grid_data_p->occ_x_dim * create_grid_data_p->occ_y_dim; i++) {
    occ_grid[i] = create_grid_data_p->occ_grid[i];
  }

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n", create_grid_metadata_block->block_id));
  free_task_metadata_block(create_grid_metadata_block);
}
