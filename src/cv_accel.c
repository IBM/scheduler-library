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
#include "cv_sched.h"
#include "cv_accel.h"

#define DMA_WORD_PER_BEAT(_st)  (sizeof(void *) / _st)

unsigned cv_cpu_run_time_in_usec      = 10000;
unsigned cv_fake_hwr_run_time_in_usec =  1000;


void print_cv_metadata_block_contents(task_metadata_block_t* mb) {
  print_base_metadata_block_contents(mb);
}


void
do_cv_accel_type_initialization()
{
 #ifdef HW_CV
 #endif
}


void
do_cv_accel_type_closeout()
{
  // Clean up any hardware accelerator stuff
 #ifdef HW_CV
 #endif
}


void
output_cv_accel_type_run_stats()
{
  char* ti_label[2] = {"CPU", "HWR"};
  printf("\n  Per-MetaData-Block CV Timing Data: %u finished CV tasks\n", freed_metadata_blocks[CV_TASK]);
  // The CV/CNN Task Timing Info
  unsigned total_cv_comp_by[3] = {0, 0, 0};
  uint64_t total_cv_call_usec[3] = {0, 0, 0}; // re-use the FFT one by same name...
  uint64_t total_parse_usec[3] = {0, 0, 0};
  for (int ti = 0; ti < 2; ti++) {
    for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
      cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(master_metadata_pool[bi].task_timings[CV_TASK]);
      unsigned this_comp_by = (unsigned)(cv_timings_p->comp_by[ti]);
      uint64_t this_cv_call_usec = (uint64_t)(cv_timings_p->call_sec[ti]) * 1000000 + (uint64_t)(cv_timings_p->call_usec[ti]);
      uint64_t this_parse_usec = (uint64_t)(cv_timings_p->parse_sec[ti]) * 1000000 + (uint64_t)(cv_timings_p->parse_usec[ti]);
      printf("Block %3u : %u %s : CV %8u call-time %15lu parse %15lu usec\n", bi, ti, ti_label[ti], this_comp_by, this_cv_call_usec, this_parse_usec);
      // Per acceleration (CPU, HWR)
      total_cv_comp_by[ti] += this_comp_by;
      total_cv_call_usec[ti]  += this_cv_call_usec;
      total_parse_usec[ti] += this_parse_usec;
      // Overall Total
      total_cv_comp_by[2] += this_comp_by;
      total_cv_call_usec[2]  += this_cv_call_usec;
      total_parse_usec[2] += this_parse_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  } // for (ti = 0, 1)
  printf("\nAggregate CV Tasks Total Timing Data:\n");
  double avg0, avg1, avg2;
  avg0 = (double)total_cv_call_usec[0] / (double) freed_metadata_blocks[CV_TASK];
  avg1 = (double)total_cv_call_usec[1] / (double) freed_metadata_blocks[CV_TASK];
  avg2 = (double)total_cv_call_usec[2] / (double) freed_metadata_blocks[CV_TASK];
  printf("     CNN-call  run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_cv_comp_by[0], total_cv_call_usec[0], avg0, 1, ti_label[1], total_cv_comp_by[1], total_cv_call_usec[1], avg1, total_cv_comp_by[2], total_cv_call_usec[2], avg2);
  avg0 = (double)total_parse_usec[0] / (double) freed_metadata_blocks[CV_TASK];
  avg1 = (double)total_parse_usec[1] / (double) freed_metadata_blocks[CV_TASK];
  avg2 = (double)total_parse_usec[2] / (double) freed_metadata_blocks[CV_TASK];
  printf("     get_label run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_cv_comp_by[0], total_parse_usec[0], avg0, 1, ti_label[1], total_cv_comp_by[1], total_parse_usec[1], avg1, total_cv_comp_by[2], total_parse_usec[2], avg2);
}


static inline label_t parse_output_dimg() {
  FILE *file_p = fopen("./output.dimg", "r");
  const size_t n_classes = 5;
  float probs[n_classes];
  for (size_t i = 0; i< n_classes; i++) {
    if (fscanf(file_p, "%f", &probs[i]) != 1) {
      printf("Didn't parse the probs[%ld] from output.dimg\n", i);
    }
  }
  float max_val = 0.0f;
  size_t max_idx = -1;
  for (size_t i = 0; i < n_classes; i++) {
    if (probs[i] > max_val) {
      max_val = probs[i], max_idx = i;
    }
  }
  fclose(file_p);
  return (label_t)max_idx;
}


void
execute_hwr_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  int fn = task_metadata_block->accelerator_id;
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]); // CV_TASK]);
  cv_timings_p->comp_by[tidx]++;
  TDEBUG(printf("In execute_hwr_cv_accelerator on CV_HWR Accel %u : MB%d  CL %d\n", fn, task_metadata_block->block_id, task_metadata_block->crit_level));
#ifdef HW_CV
  // Add the call to the NVDLA stuff here.
  printf("Doing the system call : './nvdla_runtime --loadable hpvm-mod.nvdla --image 2004_2.jpg --rawdump'\n");
  //printf("Doing the system call : './nvdla_runtime --loadable mio_loadable.nvdla --image three.jpg'\n");
  #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
  #endif

  int sret = system("./nvdla_runtime --loadable hpvm-mod.nvdla --image 0003_0.jpg --rawdump");
  //int sret = system("./nvdla_runtime --loadable mio_loadable.nvdla --image three.jpg");
  if (sret == -1) {
    printf(" The system call returned -1 -- an error occured?\n");
  }
 #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->parse_start), NULL);
  cv_timings_p->call_sec[tidx]  += cv_timings_p->parse_start.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[tidx]  += cv_timings_p->parse_start.tv_usec  - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("REAL_HW_CV: Set Call_Sec[%u] to %llu %llu\n", cv_timings_p->call_sec[tidx], cv_timings_p->call_usec[tidx]));
 #endif
  label_t pred_label = parse_output_dimg();
 #ifdef INT_TIME
  struct timeval stop;
  gettimeofday(&(stop), NULL);
  cv_timings_p->parse_sec[tidx]  += stop.tv_sec  - cv_timings_p->parse_start.tv_sec;
  cv_timings_p->parse_usec[tidx] += stop.tv_usec - cv_timings_p->parse_start.tv_usec;
 #endif
  TDEBUG(printf("---> Predicted label = %d\n", pred_label));
  // Set result into the metatdata block
  task_metadata_block->data_view.cv_data.object_label = pred_label;

#else // Of #ifdef HW_CV
 #ifdef FAKE_HW_CV
  #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
  #endif
  // This usleep call stands in as the "Fake" CNN accelerator
  usleep(cv_fake_hwr_run_time_in_usec);
  #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[tidx]  += stop_time.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[tidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_CV: Set Call_Sec[%u] to %lu %lu\n", tidx, cv_timings_p->call_sec[tidx], cv_timings_p->call_usec[tidx]));
  #endif

 #else
  printf("ERROR : This executable DOES NOT support Hardware-CV execution!\n");
  cleanup_and_exit(-2);
 #endif
#endif
  TDEBUG(printf("MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}


void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_cv_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level ));
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]); // CV_TASK]);
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




