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

#ifndef H_VIT_TASK_INCLUDE_H
#define H_VIT_TASK_INCLUDE_H

#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include "viterbi_types.h"
#include "base_task_types.h"
#include "scheduler.h"

 //Viterbi: vit_msgs_size, &(vdentry_p->ofdm_p), sizeof(ofdm_param), &(vdentry_p->frame_p), sizeof(frame_param), vdentry_p->in_bits, sizeof(uint8_t)
struct vit_io_t {
  size_t in_size; //For profiling
  ofdm_param * ofdm_ptr;
  size_t ofdm_param_size;
  frame_param * frame_ptr;
  size_t frame_ptr_size;
  int * d_ntraceback_arg;
  size_t d_ntraceback_arg_sz; 
  uint8_t * vit_data_in;
  size_t vit_data_in_size;
  uint8_t * vit_data_out;
  size_t vit_data_out_size;
  vit_io_t(ofdm_param * ofdm_ptr, size_t ofdm_param_size, frame_param * frame_ptr, size_t frame_ptr_size, uint8_t * vit_data, size_t vit_data_size) :
    ofdm_ptr(ofdm_ptr), ofdm_param_size(ofdm_param_size), frame_ptr(frame_ptr),
    frame_ptr_size(frame_ptr_size), vit_data_in(vit_data_in), vit_data_in_size(vit_data_in_size), vit_data_out(vit_data_out), vit_data_out_size(vit_data_out_size) {} //, in_size(msize) {}
};

// This is a structure that defines the "Viterbi" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.

typedef struct {
  struct timeval call_start;
  struct timeval dodec_start;
  struct timeval depunc_start;
  struct timeval depunc_stop;

  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t dodec_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t dodec_usec[SCHED_MAX_ACCEL_TYPES];

  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t depunc_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t depunc_usec[SCHED_MAX_ACCEL_TYPES];
} vit_timing_data_t;

extern "C" void print_viterbi_metadata_block_contents(void * mb);

void init_vit_parameters(int vn);

extern "C" void output_vit_task_type_run_stats(void * sptr, unsigned my_task_type, unsigned total_accel_types);

extern "C" void exec_vit_task_on_vit_hwr_accel(void * vit_io_ptr);
extern "C" void exec_vit_task_on_cpu_accel(void * vit_io_ptr);

void set_up_vit_task_on_accel_profile_data();


extern "C" void viterbi_auto_finish_routine(void * mb);
extern "C" void finish_viterbi_execution(void * vit_metadata_block); // message_t* message_id, char* out_msg_txt);
extern std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> vit_profile;
#endif
