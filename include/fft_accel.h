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


void print_fft_metadata_block_contents(task_metadata_block_t* mb);

 #ifdef HW_FFT
  void init_fft_parameters(unsigned n, uint32_t log_nsamples);
  unsigned int fft_rev(unsigned int v);
  void fft_bit_reverse(float *w, unsigned int n, unsigned int bits);
  static void fft_in_hw(int *fd, struct fftHW_access *desc);
 #endif

void execute_hwr_fft_accelerator(task_metadata_block_t* task_metadata_block);
void do_fft_task_type_initialization();
void do_fft_task_type_closeout();
void output_fft_task_type_run_stats();
#endif
