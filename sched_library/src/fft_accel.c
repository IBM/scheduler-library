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

//#define VERBOSE
#include "verbose.h"


#include "scheduler.h"
#include "fft_accel.h"

#ifdef HW_FFT
// These are FFT Hardware Accelerator Variables, etc.
char fftAccelName[NUM_FFT_ACCEL][64];// = {"/dev/fft.0", "/dev/fft.1", "/dev/fft.2", "/dev/fft.3", "/dev/fft.4", "/dev/fft.5"};

int fftHW_fd[NUM_FFT_ACCEL];
contig_handle_t fftHW_mem[NUM_FFT_ACCEL];

fftHW_token_t* fftHW_lmem[NUM_FFT_ACCEL];  // Pointer to local version (mapping) of fftHW_mem
fftHW_token_t* fftHW_li_mem[NUM_FFT_ACCEL]; // Pointer to input memory block
fftHW_token_t* fftHW_lo_mem[NUM_FFT_ACCEL]; // Pointer to output memory block
size_t fftHW_in_len[NUM_FFT_ACCEL];
size_t fftHW_out_len[NUM_FFT_ACCEL];
size_t fftHW_in_size[NUM_FFT_ACCEL];
size_t fftHW_out_size[NUM_FFT_ACCEL];
size_t fftHW_out_offset[NUM_FFT_ACCEL];
size_t fftHW_size[NUM_FFT_ACCEL];
struct fftHW_access fftHW_desc[NUM_FFT_ACCEL];


/* User-defined code */
void init_fft_parameters(unsigned n, uint32_t log_nsamples) {
  size_t fftHW_in_words_adj;
  size_t fftHW_out_words_adj;
  int len = 1 << log_nsamples;
  DEBUG(printf("  In init_fft_parameters with n = %u and logn = %u\n", n, log_nsamples));
#if (USE_FFT_ACCEL_VERSION == 1) // fft_stratus
#ifdef HW_FFT_BITREV
  fftHW_desc[n].do_bitrev  = FFTHW_DO_BITREV;
#else
  fftHW_desc[n].do_bitrev  = FFTHW_NO_BITREV;
#endif
  fftHW_desc[n].log_len    = log_nsamples;

#elif (USE_FFT_ACCEL_VERSION == 2) // fft2_stratus
  fftHW_desc[n].scale_factor = 0;
  fftHW_desc[n].logn_samples = log_nsamples;
  fftHW_desc[n].num_ffts     = 1;
  fftHW_desc[n].do_inverse   = 0;
  fftHW_desc[n].do_shift     = 0;
#endif

  if (DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)) == 0) {
    fftHW_in_words_adj  = 2 * len;
    fftHW_out_words_adj = 2 * len;
  } else {
    fftHW_in_words_adj = round_up(2 * len, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
    fftHW_out_words_adj = round_up(2 * len, DMA_WORD_PER_BEAT(sizeof(fftHW_token_t)));
  }
  fftHW_in_len[n] = fftHW_in_words_adj;
  fftHW_out_len[n] =  fftHW_out_words_adj;
  fftHW_in_size[n] = fftHW_in_len[n] * sizeof(fftHW_token_t);
  fftHW_out_size[n] = fftHW_out_len[n] * sizeof(fftHW_token_t);
  fftHW_out_offset[n] = 0;
  fftHW_size[n] = (fftHW_out_offset[n] * sizeof(fftHW_token_t)) + fftHW_out_size[n];
  DEBUG(printf("  returning from init_fft_parameters for HW_FFT[%u]\n", n));
}
#endif // HW_FFT


void
do_fft_accel_type_initialization(scheduler_datastate* sptr) {
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
      exit(EXIT_FAILURE);
    }

    printf(" Allocate hardware buffer of size %u\n", fftHW_size[fi]);
    fftHW_lmem[fi] = contig_alloc(fftHW_size[fi], &(fftHW_mem[fi]));
    if (fftHW_lmem[fi] == NULL) {
      fprintf(stderr, "Error: cannot allocate %zu contig bytes", fftHW_size[fi]);
      exit(EXIT_FAILURE);
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



#ifdef HW_FFT
void fft_in_hw(scheduler_datastate* sptr, int *fd, struct fftHW_access *desc) {
  if (ioctl(*fd, FFTHW_IOC_ACCESS, *desc)) {
    perror("ERROR : fft_in_hw : IOCTL:");
    exit(EXIT_FAILURE);
  }
}
#endif

void
do_fft_accel_type_closeout(scheduler_datastate* sptr) {
  // Clean up any hardware accelerator stuff
#ifdef HW_FFT
  for (int fi = 0; fi < NUM_FFT_ACCEL; fi++) {
    contig_free(fftHW_mem[fi]);
    close(fftHW_fd[fi]);
  }
#endif
}


void
output_fft_accel_type_run_stats(scheduler_datastate* sptr, unsigned my_accel_id, unsigned total_task_types) {
  ; // Nothing to do here (yet)
}


