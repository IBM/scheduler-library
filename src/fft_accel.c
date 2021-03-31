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
#include "fft_accel.h"

#include "fft-1d.h"
#include "calc_fmcw_dist.h"

void
do_fft_accel_type_initialization()
{
 #ifdef HW_FFT
  // This initializes the FFT Accelerator Pool
  printf("Initializing the %u total FFT Accelerators...\n", NUM_FFT_ACCEL);
  for (int fi = 0; fi < NUM_FFT_ACCEL; fi++) {
    // Inititalize to the "largest legal" FFT size
    printf("Calling init_fft_parameters for Accel %u (of %u) and LOGN %u\n", fi, NUM_FFT_ACCEL, MAX_RADAR_LOGN);
    init_fft_parameters(fi, MAX_RADAR_LOGN);

    snprintf(fftAccelName[fi], 63, "%s.%u", FFT_DEV_BASE, fi);
    printf(" Acclerator %u opening FFT device %s\n", fi, fftAccelName[fi]);
    fftHW_fd[fi] = open(fftAccelName[fi], O_RDWR, 0);
    if (fftHW_fd[fi] < 0) {
      fprintf(stderr, "Error: cannot open %s", fftAccelName[fi]);
      cleanup_and_exit(EXIT_FAILURE);
    }

    printf(" Allocate hardware buffer of size %u\n", fftHW_size[fi]);
    fftHW_lmem[fi] = contig_alloc(fftHW_size[fi], &(fftHW_mem[fi]));
    if (fftHW_lmem[fi] == NULL) {
      fprintf(stderr, "Error: cannot allocate %zu contig bytes", fftHW_size[fi]);
      cleanup_and_exit(EXIT_FAILURE);
    }

    fftHW_li_mem[fi] = &(fftHW_lmem[fi][0]);
    fftHW_lo_mem[fi] = &(fftHW_lmem[fi][fftHW_out_offset[fi]]);
    printf(" Set fftHW_li_mem = %p  AND fftHW_lo_mem = %p\n", fftHW_li_mem[fi], fftHW_lo_mem[fi]);

    fftHW_desc[fi].esp.run = true;
    fftHW_desc[fi].esp.coherence = ACC_COH_NONE;
    fftHW_desc[fi].esp.p2p_store = 0;
    fftHW_desc[fi].esp.p2p_nsrcs = 0;
    //fftHW_desc[fi].esp.p2p_srcs = {"", "", "", ""};
    fftHW_desc[fi].esp.contig = contig_to_khandle(fftHW_mem[fi]);

   #if USE_FFT_ACCEL_VERSION == 1
    // Always use BIT-REV in HW for now -- simpler interface, etc.
    fftHW_desc[fi].do_bitrev  = FFTHW_DO_BITREV;
   #elif USE_FFT_ACCEL_VERSION == 2
    fftHW_desc[fi].num_ffts      = 1;  // We only use one at a time in this applciation.
    fftHW_desc[fi].do_inverse    = FFTHW_NO_INVERSE;
    fftHW_desc[fi].do_shift      = FFTHW_NO_SHIFT;
    fftHW_desc[fi].scale_factor = 1;
   #endif
    //fftHW_desc[fi].logn_samples  = log_nsamples; 
    fftHW_desc[fi].src_offset = 0;
    fftHW_desc[fi].dst_offset = 0;
  }
 #endif

}


void
do_fft_accel_type_closeout()
{
  // Clean up any hardware accelerator stuff
 #ifdef HW_FFT
  for (int fi = 0; fi < NUM_FFT_ACCEL; fi++) {
    contig_free(fftHW_mem[fi]);
    close(fftHW_fd[fi]);
  }
 #endif
}


void
output_fft_accel_type_run_stats(unsigned my_accel_id, unsigned total_task_types)
{
  ; // Nothing to do here (yet)
}


