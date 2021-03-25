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

// This routine selects an available accelerator for the given job, 
//  The accelerator is selected according to a policy
//  The policies are implemented in separate functions.
ready_mb_task_queue_entry_t *
select_task_and_target_accelerator_new(ready_mb_task_queue_entry_t* ready_task_entry)
{
  //TODO: Make function to get task block from head of ready queue
  //Choose head of ready queue to be scheduled
  ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
  task_metadata_block_t * task_metadata_block = NULL;
  if (selected_task_entry != NULL) {
    task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
  }
  if (task_metadata_block == NULL) {
    printf("ERROR : First Ready Task Queue entry is NULL?\n");
    //pthread_mutex_unlock(&schedule_from_queue_mutex);
    cleanup_and_exit(-19);
  }
  DEBUG(printf("THE-SCHED: In fastest_to_slowest_first_available policy for MB%u\n", task_metadata_block->block_id));
 #ifdef INT_TIME
  struct timeval current_time;
  gettimeofday(&current_time, NULL);
 #endif
  int proposed_accel = no_accelerator_t;
  int accel_type     = no_accelerator_t;
  int accel_id       = -1;
  switch(task_metadata_block->job_type) {
  case FFT_TASK: {
    // Scheduler should now run this either on CPU or FFT
    do {
      int i = 0;
#ifdef HW_FFT
      proposed_accel = fft_hwr_accel_t;
      while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
        if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
          accel_type = proposed_accel;
          accel_id = i;
        }
        i++;
      } // while (loop through HWR FFT accelerators)
      scheduler_decision_checks += i;
#endif
      if (accel_id < 0) { // Didn't find one
        i = 0;
        proposed_accel = cpu_accel_t;
        while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
          if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
            accel_type = proposed_accel;
            accel_id = i;
          }
          i++;
        } // while (loop through CPU FFT accelerators)
	scheduler_decision_checks += i;
      } // if (accel_id < 0)
    } while (accel_type == no_accelerator_t);
  } break;
  case VITERBI_TASK: {
    do {
      int i = 0;
#ifdef HW_VIT
      proposed_accel = vit_hwr_accel_t;
      while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
        if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
          accel_type = proposed_accel;
          accel_id = i;
        }
        i++;
      } // while (loop through HWR VITERBI accelerators)
      scheduler_decision_checks += i;
#endif
      if (accel_id < 0) { // Didn't find one
        i = 0;
        proposed_accel = cpu_accel_t;
        while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
          if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
            accel_type = proposed_accel;
            accel_id = i;
          }
          i++;
        } // while (loop through CPU VITERBI accelerators)
	scheduler_decision_checks += i;
      } // if (accel_id < 0)
    } while (accel_type == no_accelerator_t);
  } break;
  case CV_TASK: {
    do {
      int i = 0;
#if (defined(HW_CV) || defined(FAKE_HW_CV))
      proposed_accel = cv_hwr_accel_t;
      while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
        if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
          accel_type = proposed_accel;
          accel_id = i;
        }
        i++;
      } // while (loop through HWR CV accelerators)
      scheduler_decision_checks += i;
#endif
#ifndef HW_ONLY_CV
      if (accel_id < 0) { // Didn't find one
        i = 0;
        proposed_accel = cpu_accel_t;
        while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
          if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
            accel_type = proposed_accel;
            accel_id = i;
          }
          i++;
        } // while (loop through CPU CV accelerators)
	scheduler_decision_checks += i;
      } // if (accel_id < 0)
#endif
    } while (accel_type == no_accelerator_t);
  } break;
  default:
    printf("ERROR : fastest_to_slowest_first_available called for unknown task type: %u\n", task_metadata_block->job_type);
    cleanup_and_exit(-15);
  }

 #ifdef INT_TIME
  struct timeval decis_time;
  gettimeofday(&decis_time, NULL);
  scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
 #endif
  scheduler_decisions++;
  // Okay, here we should have a good task to schedule...
  // Creating a "busy spin loop" where we constantly try to allocate
  //  This metablock to an accelerator, until one gets free...
  do {
    int i = 0;
    while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
      if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
        accel_type = proposed_accel;
        accel_id = i;
      }
      i++;
    }
  } while (accel_type == no_accelerator_t);
  task_metadata_block->accelerator_type = accel_type;
  task_metadata_block->accelerator_id = accel_id;

  return selected_task_entry;
}
