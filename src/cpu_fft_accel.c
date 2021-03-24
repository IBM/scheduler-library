/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "verbose.h"
#include "scheduler.h"
#include "sched_fft.h"

#include "fft-1d.h"

#include "calc_fmcw_dist.h"

// Putting this into a pthreads invocation mode...
void execute_cpu_fft_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_fft_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level ));
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->job_type]); // FFT_TASK]);
  fft_data_struct_t * fft_data_p    = (fft_data_struct_t*)&(task_metadata_block->data_space);
  int32_t fft_log_nsamples = fft_data_p->log_nsamples;
  float * data = (float*)(fft_data_p->theData);
  fft_timings_p->comp_by[tidx]++;

 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_start), NULL);
 #endif // INT_TIME
  fft(task_metadata_block, data, 1<<fft_log_nsamples, fft_log_nsamples, -1);
  /* for (int j = 0; j < 2 * (1<<fft_log_nsamples); j++) { */
  /*   printf("%u,%f\n", j, data[j]); */
  /* } */
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fft_timings_p->fft_sec[tidx]  += stop_time.tv_sec  - fft_timings_p->fft_start.tv_sec;
  fft_timings_p->fft_usec[tidx] += stop_time.tv_usec - fft_timings_p->fft_start.tv_usec;
 #endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

