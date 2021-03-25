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
  DEBUG(printf("SCHED_FFF: In fastest_finish_time_first policy for MB%u task %s\n", task_metadata_block->block_id, task_job_str[task_metadata_block->job_type]));
  int num_proposed_accel_types = 0;
  int proposed_accel[5] = {no_accelerator_t, no_accelerator_t, no_accelerator_t, no_accelerator_t, no_accelerator_t};
  int accel_type     = no_accelerator_t;
  int accel_id       = -1;
  int best_accel_id  = -1;

  uint64_t least_finish_time = 0xffffffffffffffffLL; // MAX for a 64-bit value
  uint64_t finish_time = 0;
  uint64_t remaining_time = 0;

  struct timeval current_time;
  uint64_t elapsed_sec, elapsed_usec, total_elapsed_usec;

  gettimeofday(&current_time, NULL);
  DEBUG(printf("SCHED_FFF:  Got the current_time as %lu\n", current_time.tv_sec*1000000 + current_time.tv_usec));

  switch(task_metadata_block->job_type) {
  case FFT_TASK: {   // Scheduler should run this either on CPU or FFT
    proposed_accel[num_proposed_accel_types++] = cpu_accel_t;
    DEBUG(printf("SCHED_FFF:    Set prop_acc[%u] = %u = %s  with %u FFT\n", (num_proposed_accel_types-1), proposed_accel[num_proposed_accel_types-1], accel_type_str[proposed_accel[num_proposed_accel_types-1]], num_accelerators_of_type[fft_hwr_accel_t]));
//#ifdef HW_FFT
    DEBUG(printf("SCHED_FFF:     Have HW_FFT : NUM_FFT_ACCEL = %u num_accel_of_type[FFT] = %u\n", NUM_FFT_ACCEL, num_accelerators_of_type[fft_hwr_accel_t]));
    if (num_accelerators_of_type[fft_hwr_accel_t]/*NUM_FFT_ACCEL*/ > 0) {
	  proposed_accel[num_proposed_accel_types++] = fft_hwr_accel_t;
	  DEBUG(printf("SCHED_FFF:    Set prop_acc[%u] = %u = %s\n", (num_proposed_accel_types-1), proposed_accel[num_proposed_accel_types-1], accel_type_str[proposed_accel[num_proposed_accel_types-1]]));
    }
//#endif
  } break;
  case VITERBI_TASK: {  // Scheduler should run this either on CPU or VIT
    proposed_accel[num_proposed_accel_types++] = cpu_accel_t;
    DEBUG(printf("SCHED_FFF:    Set prop_acc[%u] = %u = %s  with %u VIT\n", (num_proposed_accel_types-1), proposed_accel[num_proposed_accel_types-1], accel_type_str[proposed_accel[num_proposed_accel_types-1]], num_accelerators_of_type[vit_hwr_accel_t])); //NUM_VIT_ACCEL));
#ifdef HW_VIT
    DEBUG(printf("SCHED_FFF:     Have HW_VIT : NUM_VIT_ACCEL = %u num_accel_of_type[VIT] = %u\n", NUM_VIT_ACCEL, num_accelerators_of_type[vit_hwr_accel_t]));
    if (num_accelerators_of_type[vit_hwr_accel_t]/*NUM_VIT_ACCEL*/ > 0) {
    proposed_accel[num_proposed_accel_types++] = vit_hwr_accel_t;
    DEBUG(printf("SCHED_FFF:    Set prop_acc[%u] = %u = %s\n", (num_proposed_accel_types-1), proposed_accel[num_proposed_accel_types-1], accel_type_str[proposed_accel[num_proposed_accel_types-1]]));
    }
#endif
  } break;
  case CV_TASK: {   // Scheduler should run this either on CPU or CV
    num_proposed_accel_types = 0;
#ifndef HW_ONLY_CV
    proposed_accel[num_proposed_accel_types++] = cpu_accel_t;
#endif
#if (defined(HW_CV) || defined(FAKE_HW_CV))
    if (NUM_CV_ACCEL > 0) {
      proposed_accel[num_proposed_accel_types++] = cv_hwr_accel_t;
    }
#endif
  } break;
  default:
    printf("ERROR : fastest_finish_time_first called for unknown task type: %u\n", task_metadata_block->job_type);
    cleanup_and_exit(-15);
  }

  DEBUG(printf("SCHED_FFF:  There are %u  proposed accel types:\n", num_proposed_accel_types);
	  for (int pi = 0; pi < num_proposed_accel_types; pi++) {
	  printf("SCHED_FFF:       prop_acc[%u] = %u = %s\n", pi, proposed_accel[pi], accel_type_str[proposed_accel[pi]]);
  });

  // Now that we know the set of proposed accelerators,
  //  scan through to find which one will produce the earliest estimated finish time
  for (int pi = 0; pi < num_proposed_accel_types; pi++) {
     DEBUG(printf("SCHED_FFF:   Working on Proposed Accel Type %u  %s (there are %u)\n", pi, accel_type_str[proposed_accel[pi]], num_accelerators_of_type[proposed_accel[pi]]));
    for (int i = 0; i < num_accelerators_of_type[proposed_accel[pi]]; ++i) {
      int bi = accelerator_in_use_by[proposed_accel[pi]][i];
      DEBUG(printf("SCHED_FFF:      Have Accel Type %u Number %u In-Use-By %d\n", pi, i, bi));
      if (bi == -1) { // The accelerator is Free
	// The estimated task finish time is taken from the task profiling information
	finish_time = task_metadata_block->task_profile[proposed_accel[pi]];
	DEBUG(printf("SCHED_FFF:     So projected finish_time = %lu\n", finish_time));
      } else { // Accel is running a task
	// Compute the remaining execution time (estimate) for job currently on accelerator
	elapsed_sec = current_time.tv_sec - master_metadata_pool[bi].sched_timings.running_start.tv_sec;
	elapsed_usec = current_time.tv_usec - master_metadata_pool[bi].sched_timings.running_start.tv_usec;
	total_elapsed_usec = elapsed_sec*1000000 + elapsed_usec;
	remaining_time = master_metadata_pool[bi].task_profile[proposed_accel[pi]] - total_elapsed_usec;
	// and add that to the projected task run time to get the estimated finish time.
	finish_time = task_metadata_block->task_profile[proposed_accel[pi]] + remaining_time;
	DEBUG(printf("SCHED_FFF:     So projected finish_time = %lu + %lu = %lu\n", task_metadata_block->task_profile[proposed_accel[pi]] , remaining_time, finish_time));
      }
      DEBUG(printf("SCHED_FFF:             finish_time = %lu = 0x%016lx\n", finish_time, finish_time));
      DEBUG(printf("SCHED_FFF:    vs least_finish_time = %lu = 0x%016lx\n", least_finish_time, least_finish_time));
      if (finish_time < least_finish_time) {
	best_accel_id = i;
	accel_type = proposed_accel[pi];
	least_finish_time = finish_time;
	DEBUG(printf("SCHED_FFF: NEW best_accel_id = %u with least_finish_time %lu\n", best_accel_id, least_finish_time));
      }
      //printf("SCHED_FFF: For accel %u %u : bi = %u : finish_time = %lu\n", pi, i, bi, finish_time);
    } // for (i = spin through proposed accelerators)
    scheduler_decision_checks += num_accelerators_of_type[proposed_accel[pi]];
  } // for (pi goes through proposed_accelerator_types)

 #ifdef INT_TIME
  struct timeval decis_time;
  gettimeofday(&decis_time, NULL);
  scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
  scheduler_decisions++;
 #endif
  // Okay, here we should have a good task to schedule...
  // Creating a "busy spin loop" where we constantly try to allocate
  // this metablock to best accelerator, until it is free and task is allocated
  while (accel_id < 0) {
    DEBUG(printf("SCHED_FFF: Busy accel type: %d id: accel_id: %d\n", accel_type, best_accel_id));
    if (accelerator_in_use_by[accel_type][best_accel_id] == -1) {
      // Not in use -- available
      accel_id = best_accel_id;
    }
  }
  task_metadata_block->accelerator_type = accel_type;
  task_metadata_block->accelerator_id = accel_id;

  return selected_task_entry;
}
