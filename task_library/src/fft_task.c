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

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

 //#define VERBOSE
#include "verbose.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "fft_accel.h" // required for the execute_on_hwr functions
#include "fft_task.h"
#include "scheduler.h"

#include "calc_fmcw_dist.h"
#include "fft-1d.h"

std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> fft_profile; // FFT tasks can be 1k or 16k samplesw

extern "C" {
  void print_fft_metadata_block_contents(/*task_metadata_entry*/ void * mb_ptr) {
    task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
    print_base_metadata_block_contents(mb);
    fft_io_t * fft_data_p = (fft_io_t *) (mb->data_space);
    printf("  FFT using logn_samples %u for %u samples\n", fft_data_p->log_nsamples,
      (1 << fft_data_p->log_nsamples));
  }}

// Right now default to max of 16k-samples
#define MAX_fft_log_nsamples 14 // Maximum FFT samples per invocation size
// unsigned crit_fft_log_nsamples = MAX_fft_log_nsamples; // Log2 of num FFT
// samples in Critical FFT tasks

#ifdef COMPILE_TO_ESP
#include "fixed_point.h"
#endif
#include "calc_fmcw_dist.h"

#ifdef HW_FFT
unsigned int fft2_rev(unsigned int v) {
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

void fft2_bit_reverse(float * w, unsigned int n, unsigned int bits) {
  unsigned int i, s, shift;

  s = sizeof(i) * CHAR_BIT - 1;
  shift = s - bits + 1;

  for (i = 0; i < n; i++) {
    unsigned int r;
    float t_real, t_imag;

    r = fft2_rev(i);
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

#endif

extern "C" {
  void output_fft_task_type_run_stats( /*scheduler_datastate*/ void * sptr_ptr, unsigned my_task_type,
    unsigned total_accel_types) {
    scheduler_datastate * sptr = (scheduler_datastate *) sptr_ptr;
    printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n",
      my_task_type, sptr->task_name_str[my_task_type],
      sptr->freed_metadata_blocks[my_task_type], total_accel_types);

    // The FFT Tasks Timing Info
    unsigned total_fft_comp_by[total_accel_types + 1];
    uint64_t total_fft_call_usec[total_accel_types + 1];
    uint64_t total_fft_usec[total_accel_types + 1];
    uint64_t total_fft_br_usec[total_accel_types + 1];
    uint64_t total_bitrev_usec[total_accel_types + 1];
    uint64_t total_fft_cvtin_usec[total_accel_types + 1];
    uint64_t total_fft_comp_usec[total_accel_types + 1];
    uint64_t total_fft_cvtout_usec[total_accel_types + 1];
    uint64_t total_cdfmcw_usec[total_accel_types + 1];
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
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type,
          sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
      }
      for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
        fft_timing_data_t * fft_timings_p = (fft_timing_data_t *) &
          (sptr->master_metadata_pool[bi].task_timings[my_task_type]);
        unsigned this_comp_by = (unsigned) (
          sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
        uint64_t this_fft_call_usec = (uint64_t) (fft_timings_p->call_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->call_usec[ai]);
        uint64_t this_fft_usec = (uint64_t) (fft_timings_p->fft_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->fft_usec[ai]);
        uint64_t this_fft_br_usec = (uint64_t) (fft_timings_p->fft_br_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->fft_br_usec[ai]);
        uint64_t this_bitrev_usec = (uint64_t) (fft_timings_p->bitrev_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->bitrev_usec[ai]);
        uint64_t this_fft_cvtin_usec = (uint64_t) (fft_timings_p->fft_cvtin_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->fft_cvtin_usec[ai]);
        uint64_t this_fft_comp_usec = (uint64_t) (fft_timings_p->fft_comp_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->fft_comp_usec[ai]);
        uint64_t this_fft_cvtout_usec = (uint64_t) (fft_timings_p->fft_cvtout_sec[ai]) * 1000000 +
          (uint64_t) (fft_timings_p->fft_cvtout_usec[ai]);
        uint64_t this_cdfmcw_usec = (uint64_t) (fft_timings_p->cdfmcw_sec[ai]) * 1000000 + (uint64_t) (
          fft_timings_p->cdfmcw_usec[ai]);
        if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
          printf("    Block %3u : %u %s : CmpBy %8u call %15lu fft %15lu fft_br %15lu br %15lu cvtin %15lu calc %15lu cvto %15lu fmcw %15lu usec\n",
            bi, ai, sptr->accel_name_str[ai], this_comp_by,
            this_fft_call_usec, this_fft_usec, this_fft_br_usec,
            this_bitrev_usec, this_fft_cvtin_usec, this_fft_comp_usec,
            this_fft_cvtout_usec, this_cdfmcw_usec);
        }
        else {
          if ((this_comp_by + this_fft_call_usec + this_fft_usec + this_fft_br_usec + this_bitrev_usec +
            this_fft_cvtin_usec + this_fft_comp_usec + this_fft_cvtout_usec +
            this_cdfmcw_usec) != 0) {
            printf("  ERROR: Block %3u : %u %s : CmpBy %8u call %15lu fft %15lu fft_br %15lu br %15lu cvtin %15lu calc %15lu cvto %15lu fmcw %15lu usec\n",
              bi, ai, sptr->accel_name_str[ai], this_comp_by,
              this_fft_call_usec, this_fft_usec, this_fft_br_usec,
              this_bitrev_usec, this_fft_cvtin_usec, this_fft_comp_usec,
              this_fft_cvtout_usec, this_cdfmcw_usec);
          }
        }
        // Per acceleration (CPU, HWR)
        total_fft_comp_by[ai] += this_comp_by;
        total_fft_call_usec[ai] += this_fft_call_usec;
        total_fft_usec[ai] += this_fft_usec;
        total_fft_br_usec[ai] += this_fft_br_usec;
        total_bitrev_usec[ai] += this_bitrev_usec;
        total_fft_cvtin_usec[ai] += this_fft_cvtin_usec;
        total_fft_comp_usec[ai] += this_fft_comp_usec;
        total_fft_cvtout_usec[ai] += this_fft_cvtout_usec;
        total_cdfmcw_usec[ai] += this_cdfmcw_usec;
        // Overall Total
        total_fft_comp_by[total_accel_types] += this_comp_by;
        total_fft_call_usec[total_accel_types] += this_fft_call_usec;
        total_fft_usec[total_accel_types] += this_fft_usec;
        total_fft_br_usec[total_accel_types] += this_fft_br_usec;
        total_bitrev_usec[total_accel_types] += this_bitrev_usec;
        total_fft_cvtin_usec[total_accel_types] += this_fft_cvtin_usec;
        total_fft_comp_usec[total_accel_types] += this_fft_comp_usec;
        total_fft_cvtout_usec[total_accel_types] += this_fft_cvtout_usec;
        total_cdfmcw_usec[total_accel_types] += this_cdfmcw_usec;
      } // for (bi over Metadata blocks)
    } // for (ai = 0 .. total_accel_types)

    printf("\nAggregate TID %u %s  Tasks Total Timing Data: %u finished FFT tasks\n", my_task_type,
      sptr->task_name_str[my_task_type],
      sptr->freed_metadata_blocks[my_task_type]);
    printf("     fft-call run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_fft_call_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_call_usec[ai], avg);
    }
    {
      double avg = (double) total_fft_call_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_fft_call_usec[total_accel_types], avg);
    }

    printf("     fft-total run time\n                        ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_fft_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_usec[ai], avg);
    }
    {
      double avg = (double) total_fft_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_fft_usec[total_accel_types], avg);
    }

    printf("     bit-reverse run time\n                      ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_fft_br_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_br_usec[ai], avg);
    }
    {
      double avg = (double) total_fft_br_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_fft_br_usec[total_accel_types], avg);
    }

    printf("     bit-rev run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_bitrev_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_bitrev_usec[ai], avg);
    }
    {
      double avg = (double) total_bitrev_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_bitrev_usec[total_accel_types], avg);
    }

    printf("     fft-cvtin run time\n                        ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_fft_cvtin_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_cvtin_usec[ai], avg);
    }
    {
      double avg = (double) total_fft_cvtin_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_fft_cvtin_usec[total_accel_types], avg);
    }

    printf("     fft-comp run time\n                         ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_fft_comp_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_comp_usec[ai], avg);
    }
    {
      double avg = (double) total_fft_comp_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_fft_comp_usec[total_accel_types], avg);
    }

    printf("     fft-cvtout run time\n                       ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_fft_cvtout_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_fft_cvtout_usec[ai], avg);
    }
    {
      double avg = (double) total_fft_cvtout_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_fft_cvtout_usec[total_accel_types], avg);
    }

    printf("     calc-dist run time\n                        ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_cdfmcw_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_fft_comp_by[ai], total_cdfmcw_usec[ai], avg);
    }
    {
      double avg = (double) total_cdfmcw_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_fft_comp_by[total_accel_types], total_cdfmcw_usec[total_accel_types], avg);
    }
  }
}

extern "C" {
  void execute_hwr_fft_accelerator(size_t in_size, uint32_t log_nsamples, float * inputs_ptr, size_t inputs_ptr_size) { //, int accelerator_id) {
    //TODO: Figure how to pass accelerator id as fn
    int fn = 0; //accelerator_id;
    // fft_io_t * fft_data_p = (fft_io_t *) (fft_io_ptr);
    // uint32_t log_nsamples = log_nsamples;
#ifdef HW_FFT
    // Now we call the init_fft_parameters for the target FFT HWR accelerator and the specific log_nsamples for this invocation
    init_fft_parameters(fn, log_nsamples);

    // convert input from float to fixed point
    float * data = (float *) (inputs_ptr);
    for (int j = 0; j < 2 * (1 << log_nsamples); j++) {
      fftHW_lmem[fn][j] = float2fx(data[j], FX_IL);
    }

    // Call the FFT Accelerator
    //    TODO: Change this. Currently this is blocking-wait for call to complete
    fft_in_hw(&(fftHW_fd[fn]), &(fftHW_desc[fn]));
    // convert output from fixed point to float
    DEBUG(printf("EHFA:   converting from fixed-point to float\n"));
    for (int j = 0; j < 2 * (1 << log_nsamples); j++) {
      data[j] = (float) fx2float(fftHW_lmem[fn][j], FX_IL);
      SDEBUG(printf("MB%u : Data[ %u ] = %f\n", task_metadata_block->block_id, j, data[j]));
    }

#else
    printf("ERROR : This executable DOES NOT support Hardware-FFT execution!\n");
    exit(-2);
#endif
  }
}

extern "C" {
  void execute_cpu_fft_accelerator(void * fft_io_ptr) {

    fft_io_t * fft_data_p = (fft_io_t *) (fft_io_ptr);
    int32_t fft_log_nsamples = fft_data_p->log_nsamples;
    float * data = (float *) (fft_data_p->inputs_ptr);


    fft(data, 1 << fft_log_nsamples, fft_log_nsamples, -1);
    /* for (int j = 0; j < 2 * (1<<fft_log_nsamples); j++) { */
    /*   printf("%u,%f\n", j, data[j]); */
    /* } */

  }
}

// We set up this data for all possible legal FFT log_nsamples sizes (though we only use 1k and 16k so far)
void set_up_fft_task_on_accel_profile_data() {
  for (int si = 0; si <= 14; si++) {
    for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
      fft_profile[si][ai] = ACINFPROF;
    }
  }
#ifdef COMPILE_TO_ESP
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  fft_profile[10][SCHED_CPU_ACCEL_T] = 23000;
  fft_profile[10][SCHED_EPOCHS_1D_FFT_ACCEL_T] = 6000;
  fft_profile[14][SCHED_CPU_ACCEL_T] = 600000;
  fft_profile[14][SCHED_EPOCHS_1D_FFT_ACCEL_T] = 143000;
#else
  fft_profile[10][SCHED_CPU_ACCEL_T] = 50;
  fft_profile[14][SCHED_CPU_ACCEL_T] = 1250;
  std::cout << "FFT profile[10]: " << fft_profile[10] << std::endl;

#endif
  DEBUG(printf("\n%18s : %18s %18s %18s %18s\n", "FFT-PROFILES", "CPU", "FFT-HWR", "VIT-HWR", "CV-HWR");
  printf("%15s :", "fft_profile[10]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", fft_profile[10][ai]); } printf("\n");
  printf("%15s :", "fft_profile[14]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", fft_profile[14][ai]); } printf("\n");
  printf("\n"));
}
// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
void fft_auto_finish_routine(/*task_metadata_entry*/ void * mb_ptr) {
  task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
  TDEBUG(scheduler_datastate * sptr = mb->scheduler_datastate_pointer;
  printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",
    mb->block_id, sptr->task_name_str[mb->task_type],
    sptr->task_criticality_str[mb->crit_level],
    sptr->accel_name_str[mb->accelerator_type],
    mb->accelerator_id));
  DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
  free_task_metadata_block(mb);
  // Thread is done -- We shouldn't need to do anything else -- when it returns from its starting function it should exit.
}

// NOTE: This routine simply copies out the results to a provided memory space ("results")

void finish_fft_execution(
  /*task_metadata_entry*/ void * fft_metadata_block_ptr, void * args) {// float* results)

  task_metadata_entry * fft_metadata_block =
    (task_metadata_entry *) fft_metadata_block_ptr;
  // float* results)
  float * results = (float *) args;

  int tidx = fft_metadata_block->accelerator_type;
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t *) &
    (fft_metadata_block->task_timings[fft_metadata_block->task_type]);
  fft_io_t * fft_data_p = (fft_io_t *) (fft_metadata_block->data_space);
  uint32_t fft_log_nsamples = fft_data_p->log_nsamples;
  float * data = (float *) fft_data_p->inputs_ptr;
#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fft_timings_p->call_sec[tidx] += stop_time.tv_sec - fft_timings_p->call_start.tv_sec;
  fft_timings_p->call_usec[tidx] += stop_time.tv_usec - fft_timings_p->call_start.tv_usec;

  gettimeofday(&(fft_timings_p->cdfmcw_start), NULL);
#endif // INT_TIME

  // Copy out task data from the MetaData Block to the provided memory space
  DEBUG(printf("scpdff: log_n = %u data_size = %u mdatp = %p\n", fft_data_p->log_nsamples,
    fft_metadata_block->data_size, data));
  for (int i = 0; i < 2 * (1 << fft_log_nsamples); i++) {
    results[i] = data[i];
  }

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n", fft_metadata_block->block_id));
  free_task_metadata_block(fft_metadata_block);
}
