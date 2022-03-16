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

#include "base_task_types.h"
#include "scheduler.h"
#include "viterbi_base.h"

//Viterbi: vit_msgs_size, &(vdentry_p->ofdm_p), sizeof(ofdm_param), &(vdentry_p->frame_p), sizeof(frame_param), vdentry_p->in_bits, sizeof(uint8_t)
struct viterbi_input_t {
  message_size_t  msize;
  ofdm_param *    ofdm_ptr;
  size_t          ofdm_param_size;
  frame_param *   frame_ptr;
  size_t          frame_ptr_size;
  uint8_t *       in_bits;
  size_t          in_bits_size;
  viterbi_input_t(message_size_t msize, ofdm_param *ofdm_ptr, size_t ofdm_param_size,
                  frame_param *frame_ptr, size_t frame_ptr_size, uint8_t *in_bits,
                  size_t in_bits_size) :
    msize(msize), ofdm_ptr(ofdm_ptr), ofdm_param_size(ofdm_param_size), frame_ptr(frame_ptr),
    frame_ptr_size(frame_ptr_size), in_bits(in_bits),
    in_bits_size(in_bits_size) {}
};

// This is a structure that defines the "Viterbi" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "Viterbi" view of "data"
  int32_t n_data_bits;
  int32_t n_cbps;
  int32_t n_traceback;
  int32_t psdu_size;
  int32_t inMem_size;   // The first inMem_size bytes of theData are the inMem (input memories)
  int32_t inData_size;  // The next inData_size bytes of theData are the inData (input data)
  int32_t outData_size; // The next outData_size bytes of theData are the outData (output data)
  uint8_t theData[64 * 1024]; // Larger than needed (~24780 + 18585) but less than FFT (so okay)
} viterbi_data_struct_t;

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

void print_viterbi_metadata_block_contents(void *mb);

void init_vit_parameters(int vn);

void output_vit_task_type_run_stats(void *sptr, unsigned my_task_type, unsigned total_accel_types);

void exec_vit_task_on_vit_hwr_accel(void *task_metadata_block);
void exec_vit_task_on_cpu_accel(void *task_metadata_block);

void set_up_vit_task_on_accel_profile_data();

void *set_up_vit_task(void *sptr, task_type_t vit_task_type,
                      task_criticality_t crit_level, bool use_auto_finish,
                      int32_t dag_id, int32_t task_id, void *);

void viterbi_auto_finish_routine(void *mb);
void finish_viterbi_execution(void *vit_metadata_block,
                              void * args); // message_t* message_id, char* out_msg_txt);
extern std::map<uint64_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> vit_profile;
#endif
