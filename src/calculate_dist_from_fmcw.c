/* -*-Mode: C;-*- */

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include "fft-1d.h"

#include "calc_fmcw_dist.h"
#include "scheduler.h"

// CONSTANTS
#define RADAR_c          300000000.0  // Speed of Light in Meters/Sec
#define RADAR_threshold -100;


// This now illustrates the use of the "task metadata" to transfer information for an FFT operation.
//  NOTE: We request a metadata block form the scheduler -- if one is not currently available, then what?
//     OPTIONS: 1. Wait for one to become available
//              2. Purge a lower-criticality task (if there is one), else wait
//              3. Drop task if this is lowest-criticality (?), else wait
//   To make the task independent of the calling program, we need to copy over the data into the metadata block
//      This treats the scheduler like an off-load accelerator, in many ways.
//   Then we should try to make thes CPU accelerators run in independent threads (pthreads, I think)?
//      This will let us refine the non-blocking behavior, and start the more detailed behavior of the
//        scheduler implementation (i.e. ranking, queue management, etc.)

void start_calculate_peak_dist_from_fmcw(task_metadata_block_t* fft_metadata_block, uint32_t fft_log_nsamples, float* data)
{
  int tidx = 0; // (fft_metadata_block->accelerator_type != cpu_accel_t);
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(fft_metadata_block->task_timings[FFT_TASK]);
  fft_data_struct_t * fft_data_p    = (fft_data_struct_t*)&(fft_metadata_block->data_space);
  //fft_metadata_block->data_view.fft_data.log_nsamples = fft_log_nsamples;
  fft_data_p->log_nsamples = fft_log_nsamples;
  fft_metadata_block->data_size = 2 * (1<<fft_log_nsamples) * sizeof(float);
  // Copy over our task data to the MetaData Block
  //fft_metadata_block->data = (uint8_t*)data;
  float* mdataptr = (float*)fft_data_p->theData;
  for (int i = 0; i < 2*(1<<fft_log_nsamples); i++) {
    mdataptr[i] = data[i];
  }

 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->call_start), NULL);
 #endif
  //  schedule_fft(data);
  request_execution(fft_metadata_block);
  // This now ends this block -- we've kicked off execution
};


// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the FFT results (As we used to)
//   but this seems un-necessary since we only want the final "distance" really.
float
finish_calculate_peak_dist_from_fmcw(task_metadata_block_t* fft_metadata_block)
{
  int tidx = (fft_metadata_block->accelerator_type != cpu_accel_t);
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(fft_metadata_block->task_timings[FFT_TASK]);
  fft_data_struct_t * fft_data_p    = (fft_data_struct_t*)&(fft_metadata_block->data_space);
  uint32_t fft_log_nsamples = fft_data_p->log_nsamples;
  float*   data = (float*)fft_data_p->theData;
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fft_timings_p->call_sec[tidx]  += stop_time.tv_sec  - fft_timings_p->call_start.tv_sec;
  fft_timings_p->call_usec[tidx] += stop_time.tv_usec - fft_timings_p->call_start.tv_usec;

  gettimeofday(&(fft_timings_p->cdfmcw_start), NULL);
 #endif // INT_TIME

  unsigned RADAR_N       = 0;   // The number of samples (2^LOGN)
  float    RADAR_fs      = 0.0; // Sampling Frequency
  float    RADAR_alpha   = 0.0; // Chirp rate (saw-tooth)
  //float   RADAR_psd_threshold = 1e-10*pow(8192,2);  // ~= 0.006711 and 450 ~= 0.163635 in 16K
  float   RADAR_psd_threshold = 0.0067108864;

  switch (fft_log_nsamples) {
  case 10:
    RADAR_fs    = 204800.0;
    RADAR_alpha = 30000000000.0;
    RADAR_psd_threshold = 0.000316; // 1e-10*pow(8192,2);  // 450m ~= 0.000638 so psd_thres ~= 0.000316 ?
    break;
  case 14:
    RADAR_fs    = 32768000.0;
    RADAR_alpha = 4800000000000.0;
    RADAR_psd_threshold = 1e-10*pow(8192,2);
    break;
  default:
    printf("ERROR : Unsupported Log-N FFT Samples Value: %u\n", fft_log_nsamples);
    exit(-1);
  }
  RADAR_N = (1 << fft_log_nsamples);

  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int i;
  float temp;
  for (i=0; i < RADAR_N; i++) {
    temp = (pow(data[2*i],2) + pow(data[2*i+1],2))/100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = i;
    }
  }
  float distance = ((float)(max_index*((float)RADAR_fs)/((float)(RADAR_N))))*0.5*RADAR_c/((float)(RADAR_alpha));
  //printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index);
 #ifdef INT_TIME
  struct timeval cdfmcw_stop;
  gettimeofday(&cdfmcw_stop, NULL);
  fft_timings_p->cdfmcw_sec[tidx]  += cdfmcw_stop.tv_sec  - fft_timings_p->cdfmcw_start.tv_sec;
  fft_timings_p->cdfmcw_usec[tidx] += cdfmcw_stop.tv_usec - fft_timings_p->cdfmcw_start.tv_usec;
 #endif // INT_TIME
  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));
  if (max_psd > RADAR_psd_threshold) {
    return distance;
  } else {
    return INFINITY;
  }
}

