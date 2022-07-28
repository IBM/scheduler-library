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

// This is an initialization routine
//  This taeks in a generic pointer (e.g. to a policy-defined structure, etc.)
// and sets internal policy-required parameters, etc.

extern "C" void
initialize_assign_task_to_pe(void * in_parm_ptr) {
  ; // Nothing to do
}

// This is an accelerator selection policy that prefers the accelerator target that results in earliest projected finish time.
//   This one scans through all the potential accelerators, and if the accelerator can
//    execute this type of task, AND the proposed accelerator's finish time is earlier than any
//    prior (selected) accelerator, then it prefers that accelerator.

extern "C" ready_mb_task_queue_entry_t *
assign_task_to_pe(scheduler_datastate* sptr) {
  //Choose head of ready queue to be scheduled
  std::list<ready_mb_task_queue_entry_t*> & ready_queue = sptr->ready_mb_task_queue_pool;

  pthread_mutex_lock(&(sptr->task_queue_mutex));
  ready_mb_task_queue_entry_t* selected_task_entry = ready_queue.front();
  pthread_mutex_unlock(&(sptr->task_queue_mutex));
  task_metadata_entry * task_metadata_block = NULL;
  if (selected_task_entry != NULL) {
    task_metadata_block = &(sptr->master_metadata_pool[selected_task_entry->block_id]);
  } else {
    printf("ERROR : First Ready Task Queue entry is NULL?\n");
    //pthread_mutex_unlock(&schedule_from_queue_mutex);
    exit( -19);  
  }
  if (task_metadata_block == NULL) {
    printf("ERROR : First Ready Task Queue entry is NULL?\n");
    //pthread_mutex_unlock(&schedule_from_queue_mutex);
    exit( -19);
  }
  DEBUG(printf("SCHED_FF: In fastest_to_slowest_first_available policy for MB%u : task %u %s \n", task_metadata_block->block_id, task_metadata_block->task_type,
               sptr->task_name_str[task_metadata_block->task_type]));

  struct timeval current_time;
  gettimeofday(&current_time, NULL);
  DEBUG(printf("SCHED_FF:  Got the current_time as %lu\n", current_time.tv_sec * 1000000 + current_time.tv_usec));

  int proposed_accel = NO_Accelerator;
  int accel_type     = NO_Accelerator;
  int accel_id       = -1;
  uint64_t proj_finish_time = ACINFPROF;
  if (task_metadata_block->task_type != NO_Task) {
    // Find an acceptable accelerator for this task (task_type)
    for (int check_accel = sptr->next_avail_accel_id - 1; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
      DEBUG(printf("SCHED_FF: task %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->task_type, sptr->task_name_str[task_metadata_block->task_type],
                   check_accel, sptr->accel_name_str[check_accel], sptr->scheduler_execute_task_function[check_accel][task_metadata_block->task_type]));
      if (sptr->scheduler_execute_task_function[check_accel][task_metadata_block->task_type] != NULL) {
        DEBUG(printf("SCHED_FF: task %u check_accel = %u Tprof 0x%016lx proj_finish_time 0x%016lx : %u\n", task_metadata_block->task_type, check_accel,
                     task_metadata_block->task_on_accel_profile[check_accel], proj_finish_time, (task_metadata_block->task_on_accel_profile[check_accel] < proj_finish_time)));
        //if (task_metadata_block->task_on_accel_profile[check_accel] < proj_finish_time)
        {
          uint64_t new_proj_finish_time;
          int i = 0;
          DEBUG(printf("SCHED_FF:  Checking from i = %u : num_acc = %u\n", i, sptr->num_accelerators_of_type[check_accel]));
          while ((i < sptr->num_accelerators_of_type[check_accel])) { // && (accel_id < 0)) {
            DEBUG(printf("SCHED_FF:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, sptr->accel_name_str[check_accel], check_accel, i,
                         sptr->accelerator_in_use_by[check_accel][i]));
            int bi = sptr->accelerator_in_use_by[check_accel][i];
            if (bi == -1) { // Not in use -- available
              new_proj_finish_time = task_metadata_block->task_on_accel_profile[check_accel];
              DEBUG(printf("SCHED_FF:     So AcTy %u Acc %u projected finish_time = %lu\n", check_accel, i, new_proj_finish_time));
            } else {
              // Compute the remaining execution time (estimate) for task currently on accelerator
              int64_t elapsed_sec = current_time.tv_sec - sptr->master_metadata_pool[bi].sched_timings.running_start.tv_sec;
              int64_t elapsed_usec = current_time.tv_usec - sptr->master_metadata_pool[bi].sched_timings.running_start.tv_usec;
              int64_t total_elapsed_usec = elapsed_sec * 1000000 + elapsed_usec;
              int64_t remaining_time = sptr->master_metadata_pool[bi].task_on_accel_profile[check_accel] - total_elapsed_usec;
              if (remaining_time < 0) { remaining_time = 0; }
              // and add that to the projected task run time to get the estimated finish time.
              new_proj_finish_time = task_metadata_block->task_on_accel_profile[check_accel] + remaining_time;
              DEBUG(printf("SCHED_FF:     So AcTy %u Acc %u projected finish_time = %lu = %lu + %lu\n", check_accel, i, new_proj_finish_time,
                           task_metadata_block->task_on_accel_profile[check_accel], remaining_time));
            }
            if (new_proj_finish_time < proj_finish_time) {
              proposed_accel = check_accel;
              accel_type = proposed_accel;
              accel_id = i;
              proj_finish_time = new_proj_finish_time;
              DEBUG(printf("SCHED_FF:   SELECT: prop_acc %u acc_ty %u acc_id %u proj_finish_time %lu\n", proposed_accel, accel_type, accel_id, proj_finish_time));
            }
            i++;
            sptr->scheduler_decision_checks += i;
          }
        } //
      } // if (accelerator can execute this task_type)
    } // for (int check_accel = ...
  } else {
    printf("ERROR : fastest_to_slowest_first_available called for unknown task type: %u\n", task_metadata_block->task_type);
    exit( -15);
  }

#ifdef INT_TIME
  struct timeval decis_time;
  gettimeofday(&decis_time, NULL);
  sptr->scheduler_decision_time_usec += 1000000 * (decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
  sptr->scheduler_decisions++;
#endif
  // Okay, here we should have selected a target accelerator
  // Creating a "busy spin loop" where we constantly try to allocate
  //  This metablock to an accelerator, until one gets free...
  unsigned wait_iter = 0;
  while (sptr->accelerator_in_use_by[accel_type][accel_id] != -1) { // Not in use -- available
    wait_iter++;
  }
  task_metadata_block->accelerator_type = accel_type;
  task_metadata_block->accelerator_id = accel_id;
  DEBUG(printf("SCHED_FF : task %u accel_ty %u accel %u : wait_iter %u\n", task_metadata_block->task_type, accel_type, accel_id, wait_iter));

  return selected_task_entry;
}
