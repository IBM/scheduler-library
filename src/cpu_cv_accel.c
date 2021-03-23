/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>

#include <sys/time.h>
#include <unistd.h>

#include "verbose.h"
#include "scheduler.h"


void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_cv_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level ));
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[CV_TASK]);
  cv_timings_p->comp_by[tidx]++;

 #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
 #endif

  usleep(cv_cpu_run_time_in_usec);

 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[tidx]  += stop_time.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[tidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
 #endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

