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
#include "vit_accel.h"


void
do_vit_accel_type_initialization()
{
 #ifdef HW_VIT
  // This initializes the Viterbi Accelerator Pool
  for (int vi = 0; vi < NUM_VIT_ACCEL; vi++) {
    DEBUG(printf("Init Viterbi parameters on acclerator %u\n", vi));
    init_vit_parameters(vi);
    snprintf(vitAccelName[vi], 63, "%s.%u", VIT_DEV_BASE, vi);
    printf(" Accelerator %u opening Vit-Do-Decode device %s\n", vi, vitAccelName[vi]);
    vitHW_fd[vi] = open(vitAccelName[vi], O_RDWR, 0);
    if(vitHW_fd < 0) {
      fprintf(stderr, "Error: cannot open %s", vitAccelName[vi]);
      cleanup_and_exit(EXIT_FAILURE);
    }

    vitHW_lmem[vi] = contig_alloc(vitHW_size[vi], &(vitHW_mem[vi]));
    if (vitHW_lmem[vi] == NULL) {
      fprintf(stderr, "Error: cannot allocate %zu contig bytes", vitHW_size[vi]);
      cleanup_and_exit(EXIT_FAILURE);
    }
    vitHW_li_mem[vi] = &(vitHW_lmem[vi][0]);
    vitHW_lo_mem[vi] = &(vitHW_lmem[vi][vitHW_out_offset[vi]]);
    printf(" Set vitHW_li_mem = %p  AND vitHW_lo_mem = %p\n", vitHW_li_mem[vi], vitHW_lo_mem[vi]);

    vitHW_desc[vi].esp.run = true;
    vitHW_desc[vi].esp.coherence = ACC_COH_NONE;
    vitHW_desc[vi].esp.p2p_store = 0;
    vitHW_desc[vi].esp.p2p_nsrcs = 0;
    vitHW_desc[vi].esp.contig = contig_to_khandle(vitHW_mem[vi]);
  }
#endif
}


void
do_vit_accel_type_closeout()
{
  // Clean up any hardware accelerator stuff
#ifdef HW_VIT
  for (int vi = 0; vi < NUM_VIT_ACCEL; vi++) {
    contig_free(vitHW_mem[vi]);
    close(vitHW_fd[vi]);
  }
#endif
}


void
output_vit_accel_type_run_stats(unsigned my_accel_id, unsigned total_task_types)
{
}


