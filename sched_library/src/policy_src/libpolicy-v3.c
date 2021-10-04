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

#include <stdio.h>
#include <stdlib.h>
#include "scheduler.h"
#include "verbose.h"

/**
   #undef DEBUG
   #define DEBUG(x) x
**/

// This is an initialization routine
//  This taeks in a generic pointer (e.g. to a policy-defined structure, etc.)
// and sets internal policy-required parameters, etc.

extern "C" void
initialize_assign_task_to_pe(void * in_parm_ptr) {
  ; // Nothing to do
}

// This is an accelerator selection policy that prefers the accelerator target that results in earliest projected finish time.
//   This one scans through all the potential accelerators, and if the accelerator can
//    execute this type of job, AND the proposed accelerator's finish time is earlier than any
//    prior (selected) accelerator, then it prefers that accelerator.
// This version is non-blocking, in that if the current task is selected to target an
//  accelerator that is not currently available, it will continue to check younger
//  ready tasks from the task queue.

extern "C" ready_mb_task_queue_entry_t *
assign_task_to_pe(scheduler_datastate_block_t* sptr, ready_mb_task_queue_entry_t* ready_task_entry) {
  //Choose task out of order to be scheduled based on least finish time and available accelerator
  ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
  task_metadata_block_t * task_metadata_block = NULL;
  for (int i = 0; i < sptr->num_tasks_in_ready_queue; ++i) {
    task_metadata_block = &(sptr->master_metadata_pool[selected_task_entry->block_id]);
    if (task_metadata_block == NULL) {
      printf("SCHED-FFFQ: ERROR: Ready Task Queue entry %u is NULL even though num_tasks_in_ready_queue = %d depicts otherwise?\n", i,
             sptr->num_tasks_in_ready_queue);
      //pthread_mutex_unlock(&schedule_from_queue_mutex);
      exit( -19);
    }
    if (task_metadata_block->task_type == NO_Task) {
      printf("SCHED-FFFQ: ERROR: Ready Task Queue entry %u Task Type is NO_Task?\n", i);
      //pthread_mutex_unlock(&schedule_from_queue_mutex);
      exit( -20);
    }

    DEBUG(printf("SCHED-FFFQ: In fastest_finish_time_first_queued for Entry %u : MB%d Task %u = %s\n", i, task_metadata_block->block_id,
                 task_metadata_block->task_type, sptr->task_name_str[task_metadata_block->task_type]));
#ifdef INT_TIME
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
#endif

    int accel_type     = task_metadata_block->accelerator_type; //no_accelerator_t;
    int accel_id       = task_metadata_block->accelerator_id;   //-1;
    DEBUG(printf("SCHED-FFFQ: ACCEL_TY %u ACC_DI %u\n", accel_type, accel_id));
    //if (task_metadata_block->accelerator_type == no_accelerator_t || task_metadata_block->accelerator_id == -1) {
    if ((accel_type == NO_Accelerator) || (accel_id == -1)) {
      //DEBUG(printf("SCHED-FFFQ: In fastest_finish_time_first_queued policy for MB%u\n", task_metadata_block->block_id));
      uint64_t proj_finish_time = ACINFPROF;

      // Find an acceptable accelerator for this task (task_type)
      for (int check_accel = sptr->next_avail_accel_id - 1; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
        DEBUG(printf("SCHED-FFFQ: task %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->task_type,
                     sptr->task_name_str[task_metadata_block->task_type], check_accel, sptr->accel_name_str[check_accel],
                     sptr->scheduler_execute_task_function[check_accel][task_metadata_block->task_type]));
        if (sptr->scheduler_execute_task_function[check_accel][task_metadata_block->task_type] != NULL) {
          DEBUG(printf("SCHED-FFFQ: task %u check_accel = %u Tprof 0x%016lx proj_finish_time 0x%016lx : %u\n", task_metadata_block->task_type, check_accel,
                       task_metadata_block->task_on_accel_profile[check_accel], proj_finish_time, (task_metadata_block->task_on_accel_profile[check_accel] < proj_finish_time)));
          uint64_t new_proj_finish_time;
          int i = 0;
          DEBUG(printf("SCHED-FFFQ:  Checking from i = %u : num_acc = %u\n", i, sptr->num_accelerators_of_type[check_accel]));
          while ((i < sptr->num_accelerators_of_type[check_accel])) { // && (accel_id < 0)) {
            DEBUG(printf("SCHED-FFFQ:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, sptr->accel_name_str[check_accel], check_accel, i,
                         sptr->accelerator_in_use_by[check_accel][i]));
            int bi = sptr->accelerator_in_use_by[check_accel][i];
            if (bi == -1) { // Not in use -- available
              new_proj_finish_time = task_metadata_block->task_on_accel_profile[check_accel];
              DEBUG(printf("SCHED-FFFQ:     So AcTy %u Acc %u projected finish_time = %lu\n", check_accel, i, new_proj_finish_time));
            } else {
              // Compute the remaining execution time (estimate) for task currently on accelerator
              int64_t elapsed_sec = current_time.tv_sec - sptr->master_metadata_pool[bi].sched_timings.running_start.tv_sec;
              int64_t elapsed_usec = current_time.tv_usec - sptr->master_metadata_pool[bi].sched_timings.running_start.tv_usec;
              int64_t total_elapsed_usec = elapsed_sec * 1000000 + elapsed_usec;
              int64_t remaining_time = sptr->master_metadata_pool[bi].task_on_accel_profile[check_accel] - total_elapsed_usec;
              if (remaining_time < 0) { remaining_time = 0; }
              // and add that to the projected task run time to get the estimated finish time.
              new_proj_finish_time = task_metadata_block->task_on_accel_profile[check_accel] + remaining_time;
              DEBUG(printf("SCHED-FFFQ:     So AcTy %u Acc %u projected finish_time = %lu = %lu + %lu\n", check_accel, i, new_proj_finish_time,
                           task_metadata_block->task_on_accel_profile[check_accel], remaining_time));
            }
            if (new_proj_finish_time < proj_finish_time) {
              accel_type = check_accel;
              accel_id = i;
              proj_finish_time = new_proj_finish_time;
              DEBUG(printf("SCHED-FFFQ:   SELECT: acc_ty %u acc_id %u proj_finish_time %lu\n", accel_type, accel_id, proj_finish_time));
            }
            i++;
            sptr->scheduler_decision_checks += i;
          } // while (i < num_accelerators_of_type
        } // if (accelerator can execute this task_type)
      } // for (int check_accel = ...
      // At this point, we must have a "best" accelerator selected for this task
      sptr->scheduler_decisions++;
      if ((accel_type == NO_Accelerator) || (accel_id == -1)) {
        printf("SCHED-FFFQ: ERROR: Ready Task Queue entry %u Task Type %u %s couldn't find an accelerator: acc_ty %u id %d\n", i, task_metadata_block->task_type,
               sptr->task_name_str[task_metadata_block->task_type], accel_type, accel_id);
        //pthread_mutex_unlock(&schedule_from_queue_mutex);
        exit( -21);
      }
      //task_metadata_block->accelerator_type = accel_type;
      //task_metadata_block->accelerator_id   = accel_id;
    } // if (task not already assigned to an accelerator

    // Check if best accelerator is available
    //if (accelerator_in_use_by[task_metadata_block->accelerator_type][task_metadata_block->accelerator_id] == -1) {

    DEBUG(printf("SCHED-FFFQ: ACCEL_TY %u ACC_DI %u\n", accel_type, accel_id));
    if (sptr->accelerator_in_use_by[accel_type][accel_id] == -1) {
      // Task is schedulable on the best accelerator
      task_metadata_block->accelerator_type = accel_type;
      task_metadata_block->accelerator_id   = accel_id;
      DEBUG(printf("SCHED-FFFQ: MB%u Selected best accel type: %d  accel_id: %d\n", task_metadata_block->block_id, task_metadata_block->accelerator_type,
                   task_metadata_block->accelerator_id));
      return selected_task_entry;
    } else {
      DEBUG(printf("SCHED-FFFQ: MB%u Not-Available for best accel type: %d  accel_id: %d -- move to next ready task...\n", task_metadata_block->block_id, accel_type,
                   accel_id));
    }
    selected_task_entry = selected_task_entry->next;
  } // for (int i = 0; i < sptr->num_tasks_in_ready_queue; ++i)

  // No task found that can be scheduled on its best accelerator
  return NULL;
}
