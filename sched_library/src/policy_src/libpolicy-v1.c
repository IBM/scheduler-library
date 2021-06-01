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
#include "scheduler.h"
#include "verbose.h"

// This is an initialization routine
//  This taeks in a generic pointer (e.g. to a policy-defined structure, etc.)
// and sets internal policy-required parameters, etc.

void 
initialize_assign_task_to_pe(void * in_parm_ptr)
{
  ; // Nothing to do
}


// This is a basic accelerator selection policy:
//   This one scans through all the potential accelerators, and if the accelerator can
//    execute this type of task, AND the proposed accelerator is FASTER than any
//    prior (selected) accelerator, then it prefers that accelerator.
//   The seeking for an accelerator repeats until an eligible is found.
// This is "blocking" in that it spins until this task is allocated to an accelerator.

ready_mb_task_queue_entry_t *
assign_task_to_pe(scheduler_datastate_block_t* sptr, ready_mb_task_queue_entry_t* ready_task_entry)
{
  //TODO: Make function to get task block from head of ready queue
  //Choose head of ready queue to be scheduled
  ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
  task_metadata_block_t * task_metadata_block = NULL;
  if (selected_task_entry != NULL) {
    task_metadata_block = &(sptr->master_metadata_pool[selected_task_entry->block_id]);
  }
  if (task_metadata_block == NULL) {
    printf("ERROR : First Ready Task Queue entry is NULL?\n");
    //pthread_mutex_unlock(&schedule_from_queue_mutex);
    cleanup_and_exit(sptr, -19);
  }
  DEBUG(printf("THE-SCHED: In fastest_to_slowest_first_available policy for MB%u\n", task_metadata_block->block_id));
 #ifdef INT_TIME
  struct timeval current_time;
  gettimeofday(&current_time, NULL);
 #endif
  int proposed_accel = NO_Accelerator;
  int accel_type     = NO_Accelerator;
  int accel_id       = -1;
  uint64_t prop_time = ACINFPROF;
  if (task_metadata_block->task_type != NO_Task) {
    do { // We will spin (in this policy) until we find an available accelerator...
      // Find an acceptable accelerator for this task (task_type)
      for (int check_accel = sptr->next_avail_accel_id-1; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
        DEBUG(printf("F2S_FA: task %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->task_type, sptr->task_name_str[task_metadata_block->task_type], check_accel, sptr->accel_name_str[check_accel], sptr->scheduler_execute_task_function[check_accel][task_metadata_block->task_type]));
        if (sptr->scheduler_execute_task_function[check_accel][task_metadata_block->task_type] != NULL) {
          DEBUG(printf("F2S_FA: task %u check_accel = %u Tprof 0x%016lx prop_time 0x%016lx : %u\n", task_metadata_block->task_type, check_accel, task_metadata_block->task_on_accel_profile[check_accel], prop_time, (task_metadata_block->task_on_accel_profile[check_accel] < prop_time)));
          if (task_metadata_block->task_on_accel_profile[check_accel] < prop_time) {
            int i = 0;
            DEBUG(printf("F2S_FA:  Checking from i = %u : num_acc = %u\n", i, sptr->num_accelerators_of_type[check_accel]));
            while ((i < sptr->num_accelerators_of_type[check_accel]) && (accel_id < 0)) {
              DEBUG(printf("F2S_FA:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, sptr->accel_name_str[check_accel], check_accel, i, sptr->accelerator_in_use_by[check_accel][i]));
              if (sptr->accelerator_in_use_by[check_accel][i] == -1) { // Not in use -- available
                proposed_accel = check_accel;
                accel_type = proposed_accel;
                accel_id = i;
                prop_time = task_metadata_block->task_on_accel_profile[proposed_accel];
                DEBUG(printf("F2S_FA:   SELECT: prop_acc %u acc_ty %u acc_id %u prop_time %lu\n", proposed_accel, accel_type, accel_id, prop_time));
              }
              i++;
              sptr->scheduler_decision_checks += i;
            }
          } // if (accelerator is currently available)
        } // if (accelerator can execute this task_type)
      } // for (int check_accel = ...
    } while(accel_type == NO_Accelerator);
  } else {
    printf("ERROR : fastest_to_slowest_first_available called for unknown task type: %u\n", task_metadata_block->task_type);
    cleanup_and_exit(sptr, -15);
  }

 #ifdef INT_TIME
  struct timeval decis_time;
  gettimeofday(&decis_time, NULL);
  sptr->scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
 #endif
  sptr->scheduler_decisions++;
  // Okay, here we should have a good task to schedule... and knwo the accelerator is available.
  task_metadata_block->accelerator_type = accel_type;
  task_metadata_block->accelerator_id = accel_id;

  return selected_task_entry;
}
