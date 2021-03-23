/*
 * Copyright 2020 IBM
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

#ifndef H_ACCELERATORS_H
#define H_ACCELERATORS_H

extern void do_task_type_initialization();
extern void do_task_type_closeout();
extern void output_task_type_run_stats();


extern void init_fft_parameters(int fn);
extern void print_fft_metadata_block_contents(task_metadata_block_t* mb);
extern void execute_cpu_fft_accelerator(task_metadata_block_t* task_metadata_block);
extern void execute_hwr_fft_accelerator(task_metadata_block_t* task_metadata_block);

extern void init_vit_parameters(int vn);
extern void print_viterbi_metadata_block_contents(task_metadata_block_t* mb);
extern void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback, unsigned char *inMemory, unsigned char *outMemory);
extern void execute_cpu_viterbi_accelerator(task_metadata_block_t* task_metadata_block);
extern void execute_hwr_viterbi_accelerator(task_metadata_block_t* task_metadata_block);

extern void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block);
extern void execute_hwr_cv_accelerator(task_metadata_block_t* task_metadata_block);


#endif
