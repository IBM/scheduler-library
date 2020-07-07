/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>
#include <unistd.h>

#include "verbose.h"
#include "scheduler.h"


void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_cv_accelerator: MB %d  CL %d\n", task_metadata_block->metadata_block_id, task_metadata_block->criticality_level ));
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  
 #ifdef INT_TIME
  gettimeofday(&(task_metadata_block->cv_timings.call_start), NULL);
 #endif

  usleep(cv_cpu_run_time_in_usec);
  
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  task_metadata_block->cv_timings.call_sec[tidx]  += stop_time.tv_sec  - task_metadata_block->cv_timings.call_start.tv_sec;
  task_metadata_block->cv_timings.call_usec[tidx] += stop_time.tv_usec - task_metadata_block->cv_timings.call_start.tv_usec;
 #endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

