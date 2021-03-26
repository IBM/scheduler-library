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

unsigned HW_THRESHOLD[NUM_JOB_TYPES][NUM_ACCEL_TYPES-1] = { {101, 101, 101, 101},   // NO_JOB : 0% chance of using any HWR
#ifdef HW_FFT
							    {101,  25, 101, 101},   // FFT : 75% chance on HWR on FFT_HWR
#else
							    {101, 101, 101, 101},   // NO_HWR_FFT : 0% chance of using any HWR
#endif
#ifdef HW_FFT
							    {101, 101,  25, 101},   // VIT : 75% chance on HWR on VIT_HWR
#else
							    {101, 101, 101, 101},   // NO_HWR_VIT : 0% chance of using any HWR
#endif
#if (defined(HW_CV) || defined(FAKE_HW_CV))
							    {101, 101, 101,  25} }; // CV  : 75% chance on HWR on CV_HWR
#else
							    {101, 101, 101, 101} }; // NO_HWR_CV : 0% chance of using any HWR
#endif

// This is a basic accelerator selection policy:
//   This one selects an accelerator type (HWR or CPU) randomly
//   If an accelerators of that type is not available, it waits until it is.

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
  if (selected_task_entry == NULL) {
    printf("Ready queue empty\n");
  }
  if (task_metadata_block == NULL) {
    printf("ERROR : First Ready Task Queue entry is NULL?\n");
    //pthread_mutex_unlock(&schedule_from_queue_mutex);
    cleanup_and_exit(-19);
  }

  DEBUG(printf("THE-SCHED: In pick_accel_and_wait_for_available policy for MB%u\n", task_metadata_block->block_id));
 #ifdef INT_TIME
  struct timeval current_time;
  gettimeofday(&current_time, NULL);
 #endif
  int proposed_accel = cpu_accel_t;
  int accel_type     = no_accelerator_t;
  int accel_id       = -1;
  if (task_metadata_block->job_type > NO_TASK_JOB) {
    // Scheduler should now run this either on CPU or HWR
    int num = (rand() % (100)); // Return a value from [0,99]
    for (int i = 1; i < NUM_ACCEL_TYPES-1; i++) {
      if (num >= HW_THRESHOLD[task_metadata_block->job_type][i]) {
        // Execute on hardware
        proposed_accel = i; // hwr_accel_t;
      }
      scheduler_decision_checks++;
    }
  } else {
    printf("ERROR : pick_accel_and_wait_for_available called for unknown task type: %u\n", task_metadata_block->job_type);
    cleanup_and_exit(-15);
  }

  // Okay, here we should have a good task to schedule...
  // Creating a "busy spin loop" where we constantly try to allocate
  //  This metablock to an accelerator, until one gets free...
 #ifdef INT_TIME
  struct timeval decis_time;
  gettimeofday(&decis_time, NULL);
  scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
 #endif
  scheduler_decisions++;
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
