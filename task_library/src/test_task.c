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
#include <stdarg.h>
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

//#define VERBOSE
#include "verbose.h"

#include "scheduler.h"
#include "test_task.h"

unsigned num_Crit_test_tasks = 0;
unsigned num_Base_test_tasks = 0;

void print_test_metadata_block_contents(/*task_metadata_block_t*/void* mb_ptr) {
  task_metadata_block_t* mb = (task_metadata_block_t*)mb_ptr;
  print_base_metadata_block_contents(mb);
}


void
output_test_task_type_run_stats(/*scheduler_datastate_block_t*/void* sptr_ptr, unsigned my_task_type, unsigned total_accel_types)
{
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t*) sptr_ptr;
  
  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n", my_task_type, sptr->task_name_str[my_task_type], sptr->freed_metadata_blocks[my_task_type], total_accel_types);
  // The TEST/CNN Task Timing Info
  unsigned total_test_comp_by[total_accel_types+1];
  uint64_t total_test_call_usec[total_accel_types+1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_test_comp_by[ai] = 0;
    total_test_call_usec[ai] = 0;
  }
  for (int ai = 0; ai < total_accel_types; ai++) {
    if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type, sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      test_timing_data_t * test_timings_p = (test_timing_data_t*)&(sptr->master_metadata_pool[bi].task_timings[my_task_type]);
      unsigned this_comp_by = (unsigned)(sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
      uint64_t this_test_call_usec = (uint64_t)(test_timings_p->call_sec[ai]) * 1000000 + (uint64_t)(test_timings_p->call_usec[ai]);
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
	printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu usec\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_test_call_usec);
      } else {
	if ((this_comp_by + this_test_call_usec) != 0) {
	  printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_test_call_usec);
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

  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type, sptr->task_name_str[my_task_type]);
  printf("     CNN-call  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_test_call_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_test_comp_by[ai], total_test_call_usec[ai], avg);
  }
  {
    double avg = (double)total_test_call_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_test_comp_by[total_accel_types], total_test_call_usec[total_accel_types], avg);
  }
}



void
execute_on_hwr_fft_test_accelerator(/*task_metadata_block_t*/void* task_metadata_block_ptr)
{
  task_metadata_block_t *task_metadata_block = (task_metadata_block_t*)task_metadata_block_ptr;
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

  TDEBUG(printf("MB%u TEST_FFThwr calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

void
execute_on_hwr_vit_test_accelerator(/*task_metadata_block_t*/void* task_metadata_block_ptr)
{
  task_metadata_block_t *task_metadata_block = (task_metadata_block_t*)task_metadata_block_ptr;
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

  TDEBUG(printf("MB%u TEST_VIThwr calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

void
execute_on_hwr_cv_test_accelerator(/*task_metadata_block_t*/void* task_metadata_block_ptr)
{
  task_metadata_block_t *task_metadata_block = (task_metadata_block_t*)task_metadata_block_ptr;
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

  TDEBUG(printf("MB%u TEST_CVhwr calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

void execute_on_cpu_test_accelerator(/*task_metadata_block_t*/void* task_metadata_block_ptr)
{
  task_metadata_block_t *task_metadata_block = (task_metadata_block_t*)task_metadata_block_ptr;
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

  TDEBUG(printf("MB%u TEST_CPU calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}


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

uint64_t test_profile[SCHED_MAX_ACCEL_TYPES];
void set_up_test_task_on_accel_profile_data() {
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    test_profile[ai] = ACINFPROF;
  }

  test_profile[SCHED_CPU_ACCEL_T]           = test_on_cpu_run_time_in_usec;
  test_profile[SCHED_EPOCHS_VITDEC_ACCEL_T] = test_on_hwr_vit_run_time_in_usec;
  test_profile[SCHED_EPOCHS_1D_FFT_ACCEL_T] = test_on_hwr_fft_run_time_in_usec;
  test_profile[SCHED_EPOCHS_CV_CNN_ACCEL_T] = test_on_hwr_cv_run_time_in_usec;
  
  DEBUG(printf("\n%15s : %18s %18s %18s %18s\n", "PROFILES", "CPU", "VIT-HWR", "FFT-HWR", "CV-HWR");
        printf("%15s :", "pnc_profile");
        for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", test_profile[ai]); } printf("\n");
        printf("\n"));
}

/*task_metadata_block_t*/void*
set_up_test_task(/*scheduler_datastate_block_t*/void* sptr_ptr,
		 task_type_t test_task_type, task_criticality_t crit_level,
		 bool use_auto_finish, int32_t dag_id, ...)
{
  va_list var_list;
  va_start(var_list, dag_id);
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t*)sptr_ptr;
 #ifdef TIME
  gettimeofday(&start_exec_test, NULL);
 #endif
  // Request a MetadataBlock (for an TEST task at Critical Level)
  task_metadata_block_t* test_mb_ptr = NULL;
  DEBUG(printf("Calling get_task_metadata_block for Critical TEST-Task %u\n", test_task_type));
  do {
    test_mb_ptr = get_task_metadata_block(sptr, dag_id, test_task_type, crit_level, test_profile);
    //usleep(get_mb_holdoff);
  } while (0); //(*mb_ptr == NULL);
 #ifdef TIME
  struct timeval got_time;
  gettimeofday(&got_time, NULL);
  exec_get_test_sec  += got_time.tv_sec  - start_exec_test.tv_sec;
  exec_get_test_usec += got_time.tv_usec - start_exec_test.tv_usec;
 #endif
  //printf("TEST Crit Profile: %e %e %e %e %e\n", test_profile[crit_test_samples_set][0], test_profile[crit_test_samples_set][1], test_profile[crit_test_samples_set][2], test_profile[crit_test_samples_set][3], test_profile[crit_test_samples_set][4]);
  if (test_mb_ptr == NULL) {
    // We ran out of metadata blocks -- PANIC!
    printf("Out of metadata blocks for TEST -- PANIC Quit the run (for now)\n");
    dump_all_metadata_blocks_states(sptr);
    exit (-4);
  }
  if (use_auto_finish) {
    test_mb_ptr->atFinish = sptr->auto_finish_task_function[test_task_type]; // get_auto_finish_routine(sptr, test_task_type);
  } else {
    test_mb_ptr->atFinish = NULL;
  }
  DEBUG(printf("MB%u In start_test_execution\n", test_mb_ptr->block_id));

  test_timing_data_t * test_timings_p = (test_timing_data_t*)&(test_mb_ptr->task_timings[test_mb_ptr->task_type]);
  test_data_struct_t * test_data_p    = (test_data_struct_t*)(test_mb_ptr->data_space);
  // Currently we don't send in any data this way (though we should include the input image here)

#ifdef INT_TIME
  gettimeofday(&(test_timings_p->call_start), NULL);
 #endif
  // This now ends this block -- we've kicked off execution
  return test_mb_ptr;
}


// This is a default "finish" routine that can be included in the start_executiond call
// for a task that is to be executed, but whose results are not used...
// 
void test_auto_finish_routine(/*task_metadata_block_t*/void* mb_ptr)
{
  task_metadata_block_t *mb = (task_metadata_block_t*)mb_ptr;
  TDEBUG(scheduler_datastate_block_t* sptr = mb->scheduler_datastate_pointer;
	 printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n", mb->block_id, sptr->task_name_str[mb->task_type], sptr->task_criticality_str[mb->crit_level], sptr->accel_name_str[mb->accelerator_type], mb->accelerator_id));
  DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
  free_task_metadata_block(mb);
}




// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the TEST results (As we used to)
//   but this seems un-necessary since we only want the final "distance" really.
void finish_test_execution(/*task_metadata_block_t*/void* test_metadata_block_ptr, ...)
{
  va_list var_list;
  va_start(var_list, test_metadata_block_ptr);
  task_metadata_block_t *test_metadata_block = (task_metadata_block_t*)test_metadata_block_ptr;
  int tidx = test_metadata_block->accelerator_type;
  test_timing_data_t * test_timings_p = (test_timing_data_t*)&(test_metadata_block->task_timings[test_metadata_block->task_type]);
  test_data_struct_t * test_data_p    = (test_data_struct_t*)(test_metadata_block->data_space);
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  test_timings_p->call_sec[tidx]  += stop_time.tv_sec  - test_timings_p->call_start.tv_sec;
  test_timings_p->call_usec[tidx] += stop_time.tv_usec - test_timings_p->call_start.tv_usec;
 #endif // INT_TIME

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n", test_metadata_block->block_id));
  free_task_metadata_block(test_metadata_block);
}



