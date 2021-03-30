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


// Scheduler Library statistics
stats_t* stats;

status_t initialize_policy(stats_t* s)
{
  if (s == NULL)
    return error;
  stats = s;

  return success;
}


// This is an accelerator selection policy that prefers the accelerator target that results in earliest projected finish time.
//   This one scans through all the potential accelerators, and if the accelerator can
//    execute this type of job, AND the proposed accelerator's finish time is earlier than any
//    prior (selected) accelerator, then it prefers that accelerator.
// This version is non-blocking, in that if the current task is selected to target an
//  accelerator that is not currently available, it will continue to check younger
//  ready tasks from the task queue.

ready_mb_task_queue_entry_t *
assign_task_to_pe(ready_mb_task_queue_entry_t* ready_task_entry)
{
  //Choose task out of order to be scheduled based on least finish time and available accelerator
  ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
  task_metadata_block_t * task_metadata_block = NULL;
  for (int i = 0; i < num_tasks_in_ready_queue; ++i)
  {
    task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
    if (task_metadata_block == NULL) {
      printf("SCHED-FFFQ: ERROR : Ready Task Queue entry %u is NULL even though num_tasks_in_ready_queue = %d depicts otherwise?\n", i, num_tasks_in_ready_queue);
      //pthread_mutex_unlock(&schedule_from_queue_mutex);
      cleanup_and_exit(-19);
    }
    if (task_metadata_block->job_type == NO_TASK_JOB) {
      printf("SCHED-FFFQ: ERROR : Ready Task Queue entry %u Job Type is NO_TASK_JOB?\n", i);
      //pthread_mutex_unlock(&schedule_from_queue_mutex);
      cleanup_and_exit(-20);
    }

    DEBUG(printf("SCHED-FFFQ: In fastest_finish_time_first_queued for Entry %u : MB%d Task %s\n", i, task_metadata_block->block_id, task_job_str[task_metadata_block->job_type]));
               #ifdef INT_TIME
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
               #endif

    int accel_type     = task_metadata_block->accelerator_type; //no_accelerator_t;
    int accel_id       = task_metadata_block->accelerator_id;   //-1;
    //if (task_metadata_block->accelerator_type == no_accelerator_t || task_metadata_block->accelerator_id == -1) {
    if ((accel_type == no_accelerator_t) || (accel_id == -1)) {
      DEBUG(printf("FFFQ: In fastest_finish_time_first_queued policy for MB%u\n", task_metadata_block->block_id));
      uint64_t proj_finish_time = ACINFPROF;

      // Find an acceptable accelerator for this task (job_type)
      for (int check_accel = NUM_ACCEL_TYPES-2; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
        DEBUG(printf("SCHED-FFFQ: job %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->job_type, task_job_str[task_metadata_block->job_type], check_accel, accel_type_str[check_accel], scheduler_execute_task_function[task_metadata_block->job_type][check_accel]));
        if (scheduler_execute_task_function[task_metadata_block->job_type][check_accel] != NULL) {
          DEBUG(printf("SCHED-FFFQ: job %u check_accel = %u Tprof 0x%016llx proj_finish_time 0x%016llx : %u\n", task_metadata_block->job_type, check_accel, task_metadata_block->task_profile[check_accel], proj_finish_time, (task_metadata_block->task_profile[check_accel] < proj_finish_time)));
          uint64_t new_proj_finish_time;
          int i = 0;
          DEBUG(printf("SCHED-FFFQ:  Checking from i = %u : num_acc = %u\n", i, num_accelerators_of_type[check_accel]));
          while ((i < num_accelerators_of_type[check_accel])) { // && (accel_id < 0)) {
            DEBUG(printf("SCHED-FFFQ:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, accel_type_str[check_accel], check_accel, i, accelerator_in_use_by[check_accel][i]));
            int bi = accelerator_in_use_by[check_accel][i];
            if (bi == -1) { // Not in use -- available
              new_proj_finish_time = task_metadata_block->task_profile[check_accel];
              DEBUG(printf("SCHED-FFFQ:     So AcTy %u Acc %u projected finish_time = %lu\n", check_accel, i, new_proj_finish_time));
            } else {
              // Compute the remaining execution time (estimate) for job currently on accelerator
              uint64_t elapsed_sec = current_time.tv_sec - master_metadata_pool[bi].sched_timings.running_start.tv_sec;
              uint64_t elapsed_usec = current_time.tv_usec - master_metadata_pool[bi].sched_timings.running_start.tv_usec;
              uint64_t total_elapsed_usec = elapsed_sec*1000000 + elapsed_usec;
              uint64_t remaining_time = master_metadata_pool[bi].task_profile[check_accel] - total_elapsed_usec;
              // and add that to the projected task run time to get the estimated finish time.
              new_proj_finish_time = task_metadata_block->task_profile[check_accel] + remaining_time;
              DEBUG(printf("SCHED-FFFQ:     So AcTy %u Acc %u projected finish_time = %lu = %lu + %lu\n", check_accel, i, new_proj_finish_time, task_metadata_block->task_profile[check_accel], remaining_time));
            }
            if (new_proj_finish_time < proj_finish_time) {
              accel_type = check_accel;
              accel_id = i;
              proj_finish_time = new_proj_finish_time;
              DEBUG(printf("SCHED-FFFQ:   SELECT: acc_ty %u acc_id %u proj_finish_time %lu\n", accel_type, accel_id, proj_finish_time));
            }
            i++;
            stats->scheduler_decision_checks += i;
          } // while (i < num_accelerators_of_type
        } // if (accelerator can execute this job_type)
      } // for (int check_accel = ...
      // At this point, we must have a "best" accelerator selected for this task
      stats->scheduler_decisions++;
      if ((accel_type == no_accelerator_t) || (accel_id == -1)) {
        printf("SCHED-FFFQ: ERROR : Ready Task Queue entry %u Job Type %u %s couldn't find an accelerator: acc_ty %u id %d\n", i, task_metadata_block->job_type, task_job_str[task_metadata_block->job_type], accel_type, accel_id);
        //pthread_mutex_unlock(&schedule_from_queue_mutex);
        cleanup_and_exit(-21);
      }
      //task_metadata_block->accelerator_type = accel_type;
      //task_metadata_block->accelerator_id   = accel_id;
    } // if (task not already assigned to an accelerator

    // Check if best accelerator is available
    //if (accelerator_in_use_by[task_metadata_block->accelerator_type][task_metadata_block->accelerator_id] == -1) {
    if (accelerator_in_use_by[accel_type][accel_id] == -1) {
      // Task is schedulable on the best accelerator
      DEBUG(printf("SCHED-FFFQ: Best accel type: %d id: accel_id: %d tid: %d\n", task_metadata_block->accelerator_type, task_metadata_block->accelerator_id, task_metadata_block->thread_id));
      task_metadata_block->accelerator_type = accel_type;
      task_metadata_block->accelerator_id   = accel_id;
      return selected_task_entry;
    }

    selected_task_entry = selected_task_entry->next;
  }

  // No task found that can be scheduled on its best accelerator
  return NULL;
}
