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

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "scheduler.h"
#include "fft_task.h"
#include "fft_accel.h" // required for the execute_on_hwr functions

#include "fft-1d.h"
#include "calc_fmcw_dist.h"


void print_fft_metadata_block_contents(task_metadata_block_t* mb) {
  print_base_metadata_block_contents(mb);
}

// Right now default to max of 16k-samples
#define  MAX_fft_log_nsamples  14    // Maximum FFT samples per invocation size
//unsigned crit_fft_log_nsamples = MAX_fft_log_nsamples; // Log2 of num FFT samples in Critical FFT tasks
unsigned crit_fft_samples_set  = 0; // The sample set used for Critical Task FFT


#ifdef COMPILE_TO_ESP
#include "fixed_point.h"
#endif
#include "calc_fmcw_dist.h"

#ifdef HW_FFT
unsigned int fft_rev(unsigned int v)
{
  unsigned int r = v;
  int s = sizeof(v) * CHAR_BIT - 1;

  for (v >>= 1; v; v >>= 1) {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s;
  return r;
}

void fft_bit_reverse(float *w, unsigned int n, unsigned int bits)
{
  unsigned int i, s, shift;

  s = sizeof(i) * CHAR_BIT - 1;
  shift = s - bits + 1;

  for (i = 0; i < n; i++) {
    unsigned int r;
    float t_real, t_imag;

    r = fft_rev(i);
    r >>= shift;

    if (i < r) {
      t_real = w[2 * i];
      t_imag = w[2 * i + 1];
      w[2 * i] = w[2 * r];
      w[2 * i + 1] = w[2 * r + 1];
      w[2 * r] = t_real;
      w[2 * r + 1] = t_imag;
    }
  }
}

static void fft_in_hw(scheduler_datastate_block_t* sptr, int *fd, struct fftHW_access *desc)
{
  if (ioctl(*fd, FFTHW_IOC_ACCESS, *desc)) {
    perror("ERROR : fft_in_hw : IOCTL:");
    cleanup_and_exit(sptr, EXIT_FAILURE);
  }
}
#endif






void
output_fft_task_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_task_id, unsigned total_accel_types)
{
  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n", my_task_id, sptr->task_name_str[my_task_id], sptr->freed_metadata_blocks[my_task_id], total_accel_types);

  // The FFT Tasks Timing Info
  unsigned total_fft_comp_by[total_accel_types+1];
  uint64_t total_fft_call_usec[total_accel_types+1];
  uint64_t total_fft_usec[total_accel_types+1];
  uint64_t total_fft_br_usec[total_accel_types+1];
  uint64_t total_bitrev_usec[total_accel_types+1];
  uint64_t total_fft_cvtin_usec[total_accel_types+1];
  uint64_t total_fft_comp_usec[total_accel_types+1];
  uint64_t total_fft_cvtout_usec[total_accel_types+1];
  uint64_t total_cdfmcw_usec[total_accel_types+1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_fft_comp_by[ai] = 0;
    total_fft_call_usec[ai] = 0;
    total_fft_usec[ai] = 0;
    total_fft_br_usec[ai] = 0;
    total_bitrev_usec[ai] = 0;
    total_fft_cvtin_usec[ai] = 0;
    total_fft_comp_usec[ai] = 0;
    total_fft_cvtout_usec[ai] = 0;
    total_cdfmcw_usec[ai] = 0;
  }
  // Loop though all (known) task types
  for (int ai = 0; ai < total_accel_types; ai++) {
    if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[my_task_id][ai] != NULL)) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_id, sptr->task_name_str[my_task_id], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(sptr->master_metadata_pool[bi].task_timings[my_task_id]);
      unsigned this_comp_by = (unsigned)(fft_timings_p->comp_by[ai]);
      uint64_t this_fft_call_usec = (uint64_t)(fft_timings_p->call_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->call_usec[ai]);
      uint64_t this_fft_usec = (uint64_t)(fft_timings_p->fft_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->fft_usec[ai]);
      uint64_t this_fft_br_usec = (uint64_t)(fft_timings_p->fft_br_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->fft_br_usec[ai]);
      uint64_t this_bitrev_usec = (uint64_t)(fft_timings_p->bitrev_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->bitrev_usec[ai]);
      uint64_t this_fft_cvtin_usec = (uint64_t)(fft_timings_p->fft_cvtin_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->fft_cvtin_usec[ai]);
      uint64_t this_fft_comp_usec = (uint64_t)(fft_timings_p->fft_comp_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->fft_comp_usec[ai]);
      uint64_t this_fft_cvtout_usec = (uint64_t)(fft_timings_p->fft_cvtout_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->fft_cvtout_usec[ai]);
      uint64_t this_cdfmcw_usec = (uint64_t)(fft_timings_p->cdfmcw_sec[ai]) * 1000000 + (uint64_t)(fft_timings_p->cdfmcw_usec[ai]);
      if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[my_task_id][ai] != NULL)) {
	printf("    Block %3u : %u %s : CmpBy %8u call %15lu fft %15lu fft_br %15lu br %15lu cvtin %15lu calc %15lu cvto %15lu fmcw %15lu usec\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_fft_call_usec, this_fft_usec, this_fft_br_usec, this_bitrev_usec, this_fft_cvtin_usec, this_fft_comp_usec, this_fft_cvtout_usec, this_cdfmcw_usec);
      } else {
	if ((this_comp_by + this_fft_call_usec + this_fft_usec + this_fft_br_usec + this_bitrev_usec + this_fft_cvtin_usec + this_fft_comp_usec + this_fft_cvtout_usec + this_cdfmcw_usec) != 0) {
	  printf("  ERROR: Block %3u : %u %s : CmpBy %8u call %15lu fft %15lu fft_br %15lu br %15lu cvtin %15lu calc %15lu cvto %15lu fmcw %15lu usec\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_fft_call_usec, this_fft_usec, this_fft_br_usec, this_bitrev_usec, this_fft_cvtin_usec, this_fft_comp_usec, this_fft_cvtout_usec, this_cdfmcw_usec);
	}
      }
      // Per acceleration (CPU, HWR)
      total_fft_comp_by[ai]     += this_comp_by;
      total_fft_call_usec[ai]   += this_fft_call_usec;
      total_fft_usec[ai]        += this_fft_usec;
      total_fft_br_usec[ai]     += this_fft_br_usec;
      total_bitrev_usec[ai]     += this_bitrev_usec;
      total_fft_cvtin_usec[ai]  += this_fft_cvtin_usec;
      total_fft_comp_usec[ai]   += this_fft_comp_usec;
      total_fft_cvtout_usec[ai] += this_fft_cvtout_usec;
      total_cdfmcw_usec[ai]     += this_cdfmcw_usec;
      // Overall Total
      total_fft_comp_by[total_accel_types]     += this_comp_by;
      total_fft_call_usec[total_accel_types]   += this_fft_call_usec;
      total_fft_usec[total_accel_types]        += this_fft_usec;
      total_fft_br_usec[total_accel_types]     += this_fft_br_usec;
      total_bitrev_usec[total_accel_types]     += this_bitrev_usec;
      total_fft_cvtin_usec[total_accel_types]  += this_fft_cvtin_usec;
      total_fft_comp_usec[total_accel_types]   += this_fft_comp_usec;
      total_fft_cvtout_usec[total_accel_types] += this_fft_cvtout_usec;
      total_cdfmcw_usec[total_accel_types]     += this_cdfmcw_usec;
    } // for (bi over Metadata blocks)
  } // for (ai = 0 .. total_accel_types)

  printf("\nAggregate TID %u %s  Tasks Total Timing Data: %u finished FFT tasks\n", my_task_id, sptr->task_name_str[my_task_id],sptr->freed_metadata_blocks[my_task_id]);
  printf("     fft-call run time    ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_call_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_call_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_call_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_call_usec[total_accel_types], avg);
  }

  printf("     fft-total run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_usec[total_accel_types], avg);
  }

  
  printf("     bit-reverse run time ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_br_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_br_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_br_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_br_usec[total_accel_types], avg);
  }

  
  printf("     bit-reverse run time ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_br_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_br_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_br_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_br_usec[total_accel_types], avg);
  }


  printf("     bit-rev run time     ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_bitrev_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_bitrev_usec[ai], avg);
  }
  {
    double avg = (double)total_bitrev_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_bitrev_usec[total_accel_types], avg);
  }

  printf("     fft-cvtin run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_cvtin_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_cvtin_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_cvtin_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_cvtin_usec[total_accel_types], avg);
  }
  
  printf("     fft-comp run time    ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_comp_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_comp_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_comp_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_comp_usec[total_accel_types], avg);
  }
  
  printf("     fft-cvtout run time  ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_fft_cvtout_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_cvtout_usec[ai], avg);
  }
  {
    double avg = (double)total_fft_cvtout_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_fft_cvtout_usec[total_accel_types], avg);
  }
  
  printf("     calc-dist run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_cdfmcw_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_fft_comp_by[ai], total_cdfmcw_usec[ai], avg);
  }
  {
    double avg = (double)total_cdfmcw_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_fft_comp_by[total_accel_types], total_cdfmcw_usec[total_accel_types], avg);
  }
}



void
execute_hwr_fft_accelerator(task_metadata_block_t* task_metadata_block)
{
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  int tidx = task_metadata_block->accelerator_type;
  int fn = task_metadata_block->accelerator_id;
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  fft_data_struct_t * fft_data_p    = (fft_data_struct_t*)&(task_metadata_block->data_space);
  uint32_t log_nsamples = fft_data_p->log_nsamples;
  fft_timings_p->comp_by[tidx]++;
  DEBUG(printf("EHFA: MB%u In execute_hwr_fft_accelerator on FFT_HWR Accel %u : MB%d  CL %d  %u log_nsamples\n", task_metadata_block->block_id, fn, task_metadata_block->block_id, task_metadata_block->crit_level, log_nsamples));
 #ifdef INT_TIME
  //gettimeofday(&(task_metadata_block->fft_timings.fft_start), NULL);
  gettimeofday(&(fft_timings_p->fft_start), NULL);
 #endif // INT_TIME
#ifdef HW_FFT
  // Now we call the init_fft_parameters for the target FFT HWR accelerator and the specific log_nsamples for this invocation
  init_fft_parameters(fn, log_nsamples);

  DEBUG(printf("EHFA:   MB%u converting from float to fixed-point\n", task_metadata_block->block_id));
 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_cvtin_start), NULL);
 #endif // INT_TIME
  // convert input from float to fixed point
  float * data = (float*)(fft_data_p->theData);
  for (int j = 0; j < 2 * (1 << log_nsamples); j++) {
    fftHW_lmem[fn][j] = float2fx(data[j], FX_IL);
  }
 #ifdef INT_TIME
  struct timeval cvtin_stop;
  gettimeofday(&cvtin_stop, NULL);
  fft_timings_p->fft_cvtin_sec[tidx]   += cvtin_stop.tv_sec  - fft_timings_p->fft_cvtin_start.tv_sec;
  fft_timings_p->fft_cvtin_usec[tidx]  += cvtin_stop.tv_usec - fft_timings_p->fft_cvtin_start.tv_usec;
 #endif // INT_TIME

  // Call the FFT Accelerator
  //    NOTE: Currently this is blocking-wait for call to complete
 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_comp_start), NULL);
 #endif // INT_TIME
  DEBUG(printf("EHFA:   MB%u calling the HW_FFT[%u]\n", task_metadata_block->block_id, fn));
  fft_in_hw(sptr, &(fftHW_fd[fn]), &(fftHW_desc[fn]));
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fft_timings_p->fft_comp_sec[tidx]   += stop_time.tv_sec  - fft_timings_p->fft_comp_start.tv_sec;
  fft_timings_p->fft_comp_usec[tidx]  += stop_time.tv_usec - fft_timings_p->fft_comp_start.tv_usec;
 #endif
  // convert output from fixed point to float
  DEBUG(printf("EHFA:   converting from fixed-point to float\n"));
 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_cvtout_start), NULL);
 #endif // INT_TIME
  for (int j = 0; j < 2 * (1 << log_nsamples); j++) {
    data[j] = (float)fx2float(fftHW_lmem[fn][j], FX_IL);
    SDEBUG(printf("MB%u : Data[ %u ] = %f\n", task_metadata_block->block_id, j, data[j]));
  }
 #ifdef INT_TIME
  struct timeval cvtout_stop;
  gettimeofday(&cvtout_stop, NULL);
  fft_timings_p->fft_cvtout_sec[tidx]   += cvtout_stop.tv_sec  - fft_timings_p->fft_cvtout_start.tv_sec;
  fft_timings_p->fft_cvtout_usec[tidx]  += cvtout_stop.tv_usec - fft_timings_p->fft_cvtout_start.tv_usec;
  /* #ifdef INT_TIME */
  /*  struct timeval stop_time; */
  /*  gettimeofday(&stop_time, NULL); */
  fft_timings_p->fft_sec[tidx]   += cvtout_stop.tv_sec  - fft_timings_p->fft_start.tv_sec;
  fft_timings_p->fft_usec[tidx]  += cvtout_stop.tv_usec - fft_timings_p->fft_start.tv_usec;
 #endif // INT_TIME

  DEBUG(printf("EHFA: MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);

#else
  printf("ERROR : This executable DOES NOT support Hardware-FFT execution!\n");
  cleanup_and_exit(sptr, -2);
#endif
}

void execute_cpu_fft_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_fft_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level ));
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  int tidx = task_metadata_block->accelerator_type;
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
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

