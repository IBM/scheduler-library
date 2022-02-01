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

// This is a routine to set static ranks
//  This takes in a generic pointer (e.g. to a policy-defined structure, etc.)
// and sets internal policy-required parameters, etc.

extern "C" void
assign_static_rank(scheduler_datastate* sptr, dag_metadata_entry * dag_ptr) {
   // printf("[%u.%u] Entering DYNAMIC ranking\n", task->dag_id, task->task_id);
}

extern "C" void
assign_dynamic_rank(scheduler_datastate* sptr, task_metadata_entry * task) {
   // printf("[%u.%u] Entering DYNAMIC ranking\n", task->dag_id, task->task_id);

#ifdef INT_TIME
   struct timeval current_time;
   gettimeofday(&current_time, NULL);
   // Compute the remaining execution time (estimate) for task currently on accelerator
   int64_t elapsed_sec = current_time.tv_sec -
                         task->sched_timings.get_start.tv_sec;
   int64_t elapsed_usec = current_time.tv_usec -
                          task->sched_timings.get_start.tv_usec;
   int64_t total_elapsed_usec = elapsed_sec * 1000000 + elapsed_usec;
#endif

   int64_t wcet_slack = task->deadline_time - total_elapsed_usec - task->task_max_time;
   int64_t bcet_slack = task->deadline_time - total_elapsed_usec - task->task_min_time;
   float slack = 0;
   if (task->crit_level > 1) {
      if (wcet_slack >= 0) {
         DEBUG(printf("Deadline: %lu, Elapsed time: %lu Max time: %lu, Min time: %lu, wcet_slack: %d, crit_level: %d\n",
                      task->deadline_time, total_elapsed_usec, task->task_max_time, task->task_min_time, wcet_slack,
                      task->crit_level););
         slack = 1 + wcet_slack;
         task->rank_hom = (100000 * (task->crit_level)) / slack;
         task->rank_het = 3;
      } else if (bcet_slack >= 0) {
         slack = 1 + bcet_slack;
         task->rank_hom = (100000 * (task->crit_level)) / slack;
         task->rank_het = 4;
      } else {
         slack = 1 + 0.99 / bcet_slack;
         task->rank_hom = (100000 * (task->crit_level)) / slack;
         task->rank_het = 5;
      }
   } else {
      if (wcet_slack >= 0) {
         slack = 1 + wcet_slack;
         task->rank_hom = (100000 * (task->crit_level)) / slack;
         task->rank_het = 2;
      } else if (bcet_slack >= 0) {
         slack = 1 + bcet_slack;
         task->rank_hom = (100000 * (task->crit_level)) / slack;
         task->rank_het = 1;
      } else {
         slack = 1 + 0.99 / bcet_slack;
         task->rank_hom = (100000 * (task->crit_level)) / slack;
         task->rank_het = 0;
      }
   }
   DEBUG(printf("Rank assigned dynamically: Hom: %lf, Het: %d\n", task->rank_hom, task->rank_het););
}