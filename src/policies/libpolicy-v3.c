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
  //Choose task out of order to be scheduled based on least finish time and available accelerator
  ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
  task_metadata_block_t * task_metadata_block = NULL;
  for (int i = 0; i < num_tasks_in_ready_queue; ++i)
    {
      task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
      if (task_metadata_block == NULL) {
	printf("SCHED-FFFQ: ERROR : Ready Task Queue entry is NULL even though num_tasks_in_ready_queue = %d depicts otherwise?\n", num_tasks_in_ready_queue);
	//pthread_mutex_unlock(&schedule_from_queue_mutex);
	cleanup_and_exit(-19);
      }
      DEBUG(printf("SCHED-FFFQ: In fastest_finish_time_first_queued for Entry %u : MB%d Task %s\n", i, task_metadata_block->block_id, task_job_str[task_metadata_block->job_type]));
      if (task_metadata_block->accelerator_type == no_accelerator_t || task_metadata_block->accelerator_id == -1) {
	      DEBUG(printf("FFFQ: In fastest_finish_time_first_queued policy for MB%u\n", task_metadata_block->block_id));
	int num_proposed_accel_types = 1;
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
	DEBUG(printf("SCHED-FFFQ:  Got the current_time as %lu\n", current_time.tv_sec*1000000 + current_time.tv_usec));

	switch(task_metadata_block->job_type) {
	case FFT_TASK: {   // Scheduler should run this either on CPU or FFT
	  proposed_accel[0] = cpu_accel_t;
        #ifdef HW_FFT
	  if (num_accelerators_of_type[fft_hwr_accel_t] /*NUM_FFT_ACCEL*/ > 0) {
	    proposed_accel[num_proposed_accel_types++] = fft_hwr_accel_t;
	  }
        #endif
	} break;
	case VITERBI_TASK: {  // Scheduler should run this either on CPU or VIT
	  proposed_accel[0] = cpu_accel_t;
	  DEBUG(printf("SCHED-FFFQ:  Set proposed_accel[%u] = %u = %s\n", num_proposed_accel_types, proposed_accel[num_proposed_accel_types-1], accel_type_str[proposed_accel[num_proposed_accel_types-1]]));
        #ifdef HW_VIT
	  if (num_accelerators_of_type[vit_hwr_accel_t] /*NUM_VIT_ACCEL*/ > 0) {
	    proposed_accel[num_proposed_accel_types++] = vit_hwr_accel_t;
	  DEBUG(printf("SCHED-FFFQ:  Set proposed_accel[%u] = %u = %s\n", num_proposed_accel_types, proposed_accel[num_proposed_accel_types-1], accel_type_str[proposed_accel[num_proposed_accel_types-1]]));
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
	  printf("SCHED-FFFQ: ERROR : fastest_finish_time_first called for unknown task type: %u\n", task_metadata_block->job_type);
	  cleanup_and_exit(-15);
	}

	DEBUG(printf("SCHED-FFFQ:  Got a total of %u proposed accel types\n", num_proposed_accel_types));

	// Now that we know the set of proposed accelerators,
	//  scan through to find which one will produce the earliest estimated finish time
	for (int pi = 0; pi < num_proposed_accel_types; pi++) {
		DEBUG(printf("SCHED-FFFQ:   Working on Proposed Accel Type %u = %s\n", pi, accel_type_str[proposed_accel[pi]]));
		DEBUG(printf("SCHED-FFFQ: num_acc_of_ty = %u\n", num_accelerators_of_type[proposed_accel[pi]]));
	  for (int i = 0; i < num_accelerators_of_type[proposed_accel[pi]]; ++i) {
	    int bi = accelerator_in_use_by[proposed_accel[pi]][i];

	    //Estimated execution time for tasks ahead in queue scheduled on same accelerator id
	    ready_mb_task_queue_entry_t* task_ahead_entry = ready_task_entry;
	    task_metadata_block_t * ahead_task_metadata_block = NULL;
	    uint64_t ahead_execution_time = 0;
	    while(task_ahead_entry != selected_task_entry) {
	      ahead_task_metadata_block = &(master_metadata_pool[task_ahead_entry->block_id]);
	      if(ahead_task_metadata_block->accelerator_type == proposed_accel[pi] && ahead_task_metadata_block->accelerator_id == i) {
		ahead_execution_time += ahead_task_metadata_block->task_profile[proposed_accel[pi]];
	      }
	      task_ahead_entry = task_ahead_entry->next;
	    }

	    DEBUG(printf("SCHED-FFFQ:    Have Accel Type %u Number %u In-Use-By %d\n", pi, i, bi));
	    if (bi == -1) { // The accelerator is Free
	      // The estimated task finish time is taken from the task profiling information
	      finish_time = task_metadata_block->task_profile[proposed_accel[pi]];
	      DEBUG(printf("THE-SCHED:     So projected finish_time = %lu\n", finish_time));
	    } else { // Accel is running a task
	      // Compute the remaining execution time (estimate) for job currently on accelerator
	      elapsed_sec = current_time.tv_sec - master_metadata_pool[bi].sched_timings.running_start.tv_sec;
	      elapsed_usec = current_time.tv_usec - master_metadata_pool[bi].sched_timings.running_start.tv_usec;
	      total_elapsed_usec = elapsed_sec*1000000 + elapsed_usec;
	      remaining_time = master_metadata_pool[bi].task_profile[proposed_accel[pi]] - total_elapsed_usec;
	      // and add that to the projected task run time to get the estimated finish time.
	      finish_time = task_metadata_block->task_profile[proposed_accel[pi]] + remaining_time;
	      DEBUG(printf("THE-SCHED:     So projected finish_time = %lu + %lu = %lu\n", task_metadata_block->task_profile[proposed_accel[pi]] , remaining_time, finish_time));
	    }
	    // and add that to the projected run time of tasks ahead in queue to be scheduled on same accelerator to get the estimated finish time
	    finish_time += ahead_execution_time;
	    DEBUG(printf("SCHED-FFFQ:             finish_time = %lu = 0x%016lx\n", finish_time, finish_time));
	    DEBUG(printf("SCHED-FFFQ:    vs least_finish_time = %lu = 0x%016lx\n", least_finish_time, least_finish_time));
	    if (finish_time < least_finish_time) {
	      best_accel_id = i;
	      accel_type = proposed_accel[pi];
	      least_finish_time = finish_time;
	      DEBUG(printf("THE-SCHED: NEW best_accel_id = %u with least_finish_time %lu\n", best_accel_id, least_finish_time));
	    }
	    //printf("THE-SCHED: For accel %u %u : bi = %u : finish_time = %lu\n", pi, i, bi, finish_time);
	  } // for (i = spin through proposed accelerators)
	  scheduler_decision_checks += num_accelerators_of_type[proposed_accel[pi]];

	} // for (pi goes through proposed_accelerator_types)

	// Assign tasks to the least finish time accelerator
	task_metadata_block->accelerator_type = accel_type;
	task_metadata_block->accelerator_id = best_accel_id;

       #ifdef INT_TIME
        struct timeval decis_time;
        gettimeofday(&decis_time, NULL);
	scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
        scheduler_decisions++;
       #endif
      }
      // Check if best accelerator is available
      if (accelerator_in_use_by[task_metadata_block->accelerator_type][task_metadata_block->accelerator_id] == -1) {
	// Task is schedulable on the best accelerator
	DEBUG(printf("SCHED-FFFQ: Best accel type: %d id: accel_id: %d tid: %d\n", task_metadata_block->accelerator_type, task_metadata_block->accelerator_id, task_metadata_block->thread_id));
	return selected_task_entry;
      }

      selected_task_entry = selected_task_entry->next;
    }
  // No task found that can be scheduled on its best accelerator
  return NULL;
}
