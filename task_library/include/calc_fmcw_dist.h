/* -*-Mode: C;-*- */
#ifndef INCLUDED_CALC_FMCW_DIST_H
#define INCLUDED_CALC_FMCW_DIST_H

#include <stdint.h>
#include "scheduler.h"

/* Some global FFT Radar definitions */
/* extern unsigned RADAR_LOGN;  // Log2 of the number of samples */
/* extern unsigned RADAR_N;     // The number of samples (2^LOGN) */
/* extern float    RADAR_fs;    // Sampling Frequency */
/* extern float    RADAR_alpha; // Chirp rate (saw-tooth) */

/* Some function declarations */
//extern void  init_calculate_peak_dist();

void start_calculate_peak_dist_from_fmcw(task_metadata_entry* fft_metadata_block, uint32_t fft_log_nsamples, float* data);
extern float finish_calculate_peak_dist_from_fmcw(task_metadata_entry* metatask_block_ptr);

#ifdef INT_TIME
extern uint64_t calc_sec;
extern uint64_t calc_usec;

extern uint64_t fft_sec;
extern uint64_t fft_usec;

extern uint64_t fft_br_sec;
extern uint64_t fft_br_usec;

extern uint64_t fft_cvtin_sec;
extern uint64_t fft_cvtin_usec;

extern uint64_t fft_cvtout_sec;
extern uint64_t fft_cvtout_usec;

extern uint64_t cdfmcw_sec;
extern uint64_t cdfmcw_usec;
#endif

#endif
