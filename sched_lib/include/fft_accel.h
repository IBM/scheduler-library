/*
 * Copyright 2021 IBM
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

#ifndef H_FFT_ACCEL_INCLUDE_H
#define H_FFT_ACCEL_INCLUDE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#define MAX_RADAR_LOGN            14        // Max we allow is 16k samples
#define MAX_RADAR_N     (1<<MAX_RADAR_LOGN) // Max we allow is 16k samples

#ifdef COMPILE_TO_ESP
 #include "contig.h"
 #include "mini-era.h"

 #define DMA_WORD_PER_BEAT(_st)  (sizeof(void *) / _st)

 extern int fftHW_fd[NUM_FFT_ACCEL];
 extern contig_handle_t fftHW_mem[NUM_FFT_ACCEL];

 extern fftHW_token_t* fftHW_lmem[NUM_FFT_ACCEL];  // Pointer to local version (mapping) of fftHW_mem
 extern fftHW_token_t* fftHW_li_mem[NUM_FFT_ACCEL]; // Pointer to input memory block
 extern fftHW_token_t* fftHW_lo_mem[NUM_FFT_ACCEL]; // Pointer to output memory block
 extern size_t fftHW_in_len[NUM_FFT_ACCEL];
 extern size_t fftHW_out_len[NUM_FFT_ACCEL];
 extern size_t fftHW_in_size[NUM_FFT_ACCEL];
 extern size_t fftHW_out_size[NUM_FFT_ACCEL];
 extern size_t fftHW_out_offset[NUM_FFT_ACCEL];
 extern size_t fftHW_size[NUM_FFT_ACCEL];
 extern struct fftHW_access fftHW_desc[NUM_FFT_ACCEL];
#endif

void do_fft_accel_type_initialization(scheduler_datastate_block_t* sptr);

#ifdef HW_FFT
void fft_in_hw(scheduler_datastate_block_t* sptr, int *fd, struct fftHW_access *desc);
#endif

void do_fft_accel_type_closeout(scheduler_datastate_block_t* sptr);
void output_fft_accel_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_accel_id, unsigned total_task_types);

#endif
