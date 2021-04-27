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

#ifndef H_VIT_ACCEL_INCLUDE_H
#define H_VIT_ACCEL_INCLUDE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include "base_types.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"

#define DMA_WORD_PER_BEAT(_st)  (sizeof(void *) / _st)

extern int vitHW_fd[NUM_VIT_ACCEL];
extern contig_handle_t vitHW_mem[NUM_VIT_ACCEL];
extern vitHW_token_t *vitHW_lmem[NUM_VIT_ACCEL];   // Pointer to local view of contig memory
extern vitHW_token_t *vitHW_li_mem[NUM_VIT_ACCEL]; // Pointer to input memory block
extern vitHW_token_t *vitHW_lo_mem[NUM_VIT_ACCEL]; // Pointer to output memory block
extern size_t vitHW_in_len[NUM_VIT_ACCEL];
extern size_t vitHW_out_len[NUM_VIT_ACCEL];
extern size_t vitHW_in_size[NUM_VIT_ACCEL];
extern size_t vitHW_out_size[NUM_VIT_ACCEL];
extern size_t vitHW_out_offset[NUM_VIT_ACCEL];
extern size_t vitHW_size[NUM_VIT_ACCEL];
extern struct vitdodec_access vitHW_desc[NUM_VIT_ACCEL];
#endif

void do_vit_accel_type_initialization(scheduler_datastate_block_t* sptr);

#ifdef HW_VIT
void do_decoding_hw(scheduler_datastate_block_t* sptr, int *fd, struct vitdodec_access *desc);
#endif

void do_vit_accel_type_closeout(scheduler_datastate_block_t* sptr);
void output_vit_accel_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_accel_id, unsigned total_task_types);

#endif
