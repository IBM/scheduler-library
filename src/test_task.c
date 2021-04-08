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
#include <limits.h>

#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "utils.h"
//#define VERBOSE
#include "verbose.h"

#include "scheduler.h"
#include "test_task.h"

unsigned num_Crit_test_tasks = 0;
unsigned num_Base_test_tasks = 0;

#ifdef COMPILE_TO_ESP
unsigned test_on_cpu_run_time_in_usec        = 25000;
unsigned test_on_hwr_fft_run_time_in_usec    =  6000;
unsigned test_on_hwr_vit_run_time_in_usec    =  9000;
unsigned test_on_hwr_cv_run_time_in_usec     =  9000;
#else
unsigned test_on_cpu_run_time_in_usec        =     1;
unsigned test_on_hwr_fft_run_time_in_usec    =     1;
unsigned test_on_hwr_vit_run_time_in_usec    =     1;
unsigned test_on_hwr_cv_run_time_in_usec     =     1;
#endif

void print_test_metadata_block_contents(task_metadata_block_t* mb) {
  print_base_metadata_block_contents(mb);
}


void
output_test_task_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_task_id, unsigned total_accel_types)
{
  
  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n", my_task_id, sptr->task_name_str[my_task_id], sptr->freed_metadata_blocks[my_task_id], total_accel_types);
  // The TEST/CNN Task Timing Info
  unsigned total_test_comp_by[total_accel_types+1];
  uint64_t total_test_call_usec[total_accel_types+1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_test_comp_by[ai] = 0;
    total_test_call_usec[ai] = 0;
  }
  for (int ai = 0; ai < total_accel_types; ai++) {
    if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[ai][my_task_id] != NULL)) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_id, sptr->task_name_str[my_task_id], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      test_timing_data_t * test_timings_p = (test_timing_data_t*)&(sptr->master_metadata_pool[bi].task_timings[my_task_id]);
      unsigned this_comp_by = (unsigned)(sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_id]);
      uint64_t this_test_call_usec = (uint64_t)(test_timings_p->call_sec[ai]) * 1000000 + (uint64_t)(test_timings_p->call_usec[ai]);
      if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[ai][my_task_id] != NULL)) {
	printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu usec\n", bi, ai, sptr->accel_name_str[my_task_id], this_comp_by, this_test_call_usec);
      } else {
	if ((this_comp_by + this_test_call_usec) != 0) {
	  printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu\n", bi, ai, sptr->accel_name_str[my_task_id], this_comp_by, this_test_call_usec);
	}
      }
      // Per acceleration (CPU, HWR)
      total_test_comp_by[ai] += this_comp_by;
      total_test_call_usec[ai]  += this_test_call_usec;
      // Overall Total
      total_test_comp_by[total_accel_types] += this_comp_by;
      total_test_call_usec[total_accel_types]  += this_test_call_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  } // for (ai = 0 .. total_accel_types)

  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_id, sptr->task_name_str[my_task_id]);
  printf("     CNN-call  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_test_call_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_test_comp_by[ai], total_test_call_usec[ai], avg);
  }
  {
    double avg = (double)total_test_call_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_test_comp_by[total_accel_types], total_test_call_usec[total_accel_types], avg);
  }
}



void
execute_on_hwr_fft_test_accelerator(task_metadata_block_t* task_metadata_block)
{
  int fn = task_metadata_block->accelerator_id;
  int aidx = task_metadata_block->accelerator_type;
  test_timing_data_t * test_timings_p = (test_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  TDEBUG(printf("In execute_hwr_test_accelerator on TEST_HWR Accel %u : MB%d  CL %d\n", fn, task_metadata_block->block_id, task_metadata_block->crit_level));

 #ifdef INT_TIME
  gettimeofday(&(test_timings_p->call_start), NULL);
 #endif
  // This usleep call stands in as the "Fake" CNN accelerator
  usleep(test_on_hwr_fft_run_time_in_usec);
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  test_timings_p->call_sec[aidx]  += stop_time.tv_sec  - test_timings_p->call_start.tv_sec;
  test_timings_p->call_usec[aidx] += stop_time.tv_usec - test_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_TEST: Set Call_Sec[%u] to %lu %lu\n", aidx, test_timings_p->call_sec[aidx], test_timings_p->call_usec[aidx]));
 #endif

  TDEBUG(printf("MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

void
execute_on_hwr_vit_test_accelerator(task_metadata_block_t* task_metadata_block)
{
  int fn = task_metadata_block->accelerator_id;
  int aidx = task_metadata_block->accelerator_type;
  test_timing_data_t * test_timings_p = (test_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  TDEBUG(printf("In execute_hwr_test_accelerator on TEST_HWR Accel %u : MB%d  CL %d\n", fn, task_metadata_block->block_id, task_metadata_block->crit_level));

 #ifdef INT_TIME
  gettimeofday(&(test_timings_p->call_start), NULL);
 #endif
  // This usleep call stands in as the "Fake" CNN accelerator
  usleep(test_on_hwr_vit_run_time_in_usec);
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  test_timings_p->call_sec[aidx]  += stop_time.tv_sec  - test_timings_p->call_start.tv_sec;
  test_timings_p->call_usec[aidx] += stop_time.tv_usec - test_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_TEST: Set Call_Sec[%u] to %lu %lu\n", aidx, test_timings_p->call_sec[aidx], test_timings_p->call_usec[aidx]));
 #endif

  TDEBUG(printf("MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

void
execute_on_hwr_cv_test_accelerator(task_metadata_block_t* task_metadata_block)
{
  int fn = task_metadata_block->accelerator_id;
  int aidx = task_metadata_block->accelerator_type;
  test_timing_data_t * test_timings_p = (test_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  TDEBUG(printf("In execute_hwr_test_accelerator on TEST_HWR Accel %u : MB%d  CL %d\n", fn, task_metadata_block->block_id, task_metadata_block->crit_level));

 #ifdef INT_TIME
  gettimeofday(&(test_timings_p->call_start), NULL);
 #endif
  // This usleep call stands in as the "Fake" CNN accelerator
  usleep(test_on_hwr_cv_run_time_in_usec);
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  test_timings_p->call_sec[aidx]  += stop_time.tv_sec  - test_timings_p->call_start.tv_sec;
  test_timings_p->call_usec[aidx] += stop_time.tv_usec - test_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_TEST: Set Call_Sec[%u] to %lu %lu\n", aidx, test_timings_p->call_sec[aidx], test_timings_p->call_usec[aidx]));
 #endif

  TDEBUG(printf("MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

void execute_on_cpu_test_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_on_cpu_test_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level ));
  int aidx = task_metadata_block->accelerator_type;
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  test_timing_data_t * test_timings_p = (test_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);

 #ifdef INT_TIME
  gettimeofday(&(test_timings_p->call_start), NULL);
 #endif

  usleep(test_on_cpu_run_time_in_usec);

 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  test_timings_p->call_sec[aidx]  += stop_time.tv_sec  - test_timings_p->call_start.tv_sec;
  test_timings_p->call_usec[aidx] += stop_time.tv_usec - test_timings_p->call_start.tv_usec;
 #endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}




