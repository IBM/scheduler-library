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
#include "fft_sched.h"
#include "vit_sched.h"
#include "cv_sched.h"
#include "accelerators.h" // include AFTER scheduler.h -- needs types form scheduler.h

static unsigned DMA_WORD_PER_BEAT(unsigned _st)
{
  return (sizeof(void *) / _st);
}


void print_fft_metadata_block_contents(task_metadata_block_t* mb) {
  print_base_metadata_block_contents(mb);
}

void print_viterbi_metadata_block_contents(task_metadata_block_t* mb)
{  
  print_base_metadata_block_contents(mb);
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)&(mb->data_space);
  int32_t  inMem_offset = 0;
  int32_t  inData_offset = vdata->inMem_size;
  int32_t  outData_offset = inData_offset + vdata->inData_size;
  uint8_t* in_Mem  = &(vdata->theData[inMem_offset]);
  uint8_t* in_Data = &(vdata->theData[inData_offset]);
  uint8_t* out_Data = &(vdata->theData[outData_offset]);
  printf("   Viterbi Data: @ %p\n", vdata);
  printf("      n_cbps      = %d\n", vdata->n_cbps);
  printf("      n_traceback = %d\n", vdata->n_traceback);
  printf("      n_data_bits = %d\n", vdata->n_data_bits);
  printf("      psdu_size   = %d\n", vdata->psdu_size);
  printf("      in_Mem_size   = %d\n", vdata->inMem_size);
  printf("      in_Data_size  = %d\n", vdata->inData_size);
  printf("      out_Data_size = %d\n", vdata->outData_size);
  printf("      inMem_offset  = %d\n", inMem_offset);
  printf("      inData_offset  = %d\n", inData_offset);
  printf("      outData_offset = %d\n", outData_offset);
  printf("      in_Mem   @ %p\n", &(vdata->theData[inMem_offset]));
  printf("      in_Data  @ %p\n",  &(vdata->theData[inData_offset]));
  printf("      out_Data @ %p\n",  &(vdata->theData[outData_offset]));
}



#ifdef HW_VIT
// These are Viterbi Harware Accelerator Variables, etc.
char    vitAccelName[NUM_VIT_ACCEL][64]; // = {"/dev/vitdodec.0", "/dev/vitdodec.1", "/dev/vitdodec.2", "/dev/vitdodec.3", "/dev/vitdodec.4", "/dev/vitdodec.5"};
int vitHW_fd[NUM_VIT_ACCEL];
contig_handle_t vitHW_mem[NUM_VIT_ACCEL];
vitHW_token_t *vitHW_lmem[NUM_VIT_ACCEL];   // Pointer to local view of contig memory
vitHW_token_t *vitHW_li_mem[NUM_VIT_ACCEL]; // Pointer to input memory block
vitHW_token_t *vitHW_lo_mem[NUM_VIT_ACCEL]; // Pointer to output memory block
size_t vitHW_in_len[NUM_VIT_ACCEL];
size_t vitHW_out_len[NUM_VIT_ACCEL];
size_t vitHW_in_size[NUM_VIT_ACCEL];
size_t vitHW_out_size[NUM_VIT_ACCEL];
size_t vitHW_out_offset[NUM_VIT_ACCEL];
size_t vitHW_size[NUM_VIT_ACCEL];

struct vitdodec_access vitHW_desc[NUM_VIT_ACCEL];


void init_vit_parameters(int vn)
{
  size_t vitHW_in_words_adj;
  size_t vitHW_out_words_adj;
  //printf("Doing init_vit_parameters\n");
  if (DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)) == 0) {
    vitHW_in_words_adj  = 24852;
    vitHW_out_words_adj = 18585;
  } else {
    vitHW_in_words_adj  = round_up(24852, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
    vitHW_out_words_adj = round_up(18585, DMA_WORD_PER_BEAT(sizeof(vitHW_token_t)));
  }
  vitHW_in_len[vn] = vitHW_in_words_adj;
  vitHW_out_len[vn] =  vitHW_out_words_adj;
  vitHW_in_size[vn] = vitHW_in_len[vn] * sizeof(vitHW_token_t);
  vitHW_out_size[vn] = vitHW_out_len[vn] * sizeof(vitHW_token_t);
  vitHW_out_offset[vn] = vitHW_in_len[vn];
  vitHW_size[vn] = (vitHW_out_offset[vn] * sizeof(vitHW_token_t)) + vitHW_out_size[vn];
}
#endif // HW_VIT


// Right now default to max of 16k-samples
#define  MAX_fft_log_nsamples  14    // Maximum FFT samples per invocation size
//unsigned crit_fft_log_nsamples = MAX_fft_log_nsamples; // Log2 of num FFT samples in Critical FFT tasks
unsigned crit_fft_samples_set  = 0; // The sample set used for Critical Task FFT

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
void init_fft_parameters(unsigned n, uint32_t log_nsamples)
{
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


#ifdef COMPILE_TO_ESP
#include "fixed_point.h"
#endif
#include "calc_fmcw_dist.h"

#ifdef HW_FFT
unsigned int fft_rev(unsigned int v)
{
  unsigned int r = v;
  int s = sizeof(v) * CHAR_BIT - 1;

  for (v >>= 1; v; v >>= 1) {
    r <<= 1;
    r |= v & 1;
    s--;
  }
  r <<= s;
  return r;
}

void fft_bit_reverse(float *w, unsigned int n, unsigned int bits)
{
  unsigned int i, s, shift;

  s = sizeof(i) * CHAR_BIT - 1;
  shift = s - bits + 1;

  for (i = 0; i < n; i++) {
    unsigned int r;
    float t_real, t_imag;

    r = fft_rev(i);
    r >>= shift;

    if (i < r) {
      t_real = w[2 * i];
      t_imag = w[2 * i + 1];
      w[2 * i] = w[2 * r];
      w[2 * i + 1] = w[2 * r + 1];
      w[2 * r] = t_real;
      w[2 * r + 1] = t_imag;
    }
  }
}

static void fft_in_hw(int *fd, struct fftHW_access *desc)
{
  if (ioctl(*fd, FFTHW_IOC_ACCESS, *desc)) {
    perror("ERROR : fft_in_hw : IOCTL:");
    cleanup_and_exit(EXIT_FAILURE);
  }
}
#endif

void
execute_hwr_fft_accelerator(task_metadata_block_t* task_metadata_block)
{
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  int fn = task_metadata_block->accelerator_id;
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->job_type]); // FFT_TASK]);
  fft_data_struct_t * fft_data_p    = (fft_data_struct_t*)&(task_metadata_block->data_space);
  uint32_t log_nsamples = fft_data_p->log_nsamples;
  //task_metadata_block->task_timings[FFT_TASK].comp_by[tidx]++;
  fft_timings_p->comp_by[tidx]++;
  DEBUG(printf("EHFA: MB%u In execute_hwr_fft_accelerator on FFT_HWR Accel %u : MB%d  CL %d  %u log_nsamples\n", task_metadata_block->block_id, fn, task_metadata_block->block_id, task_metadata_block->crit_level, log_nsamples));
 #ifdef INT_TIME
  //gettimeofday(&(task_metadata_block->fft_timings.fft_start), NULL);
  gettimeofday(&(fft_timings_p->fft_start), NULL);
 #endif // INT_TIME
#ifdef HW_FFT
  // Now we call the init_fft_parameters for the target FFT HWR accelerator and the specific log_nsamples for this invocation
  init_fft_parameters(fn, log_nsamples);

  DEBUG(printf("EHFA:   MB%u converting from float to fixed-point\n", task_metadata_block->block_id));
 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_cvtin_start), NULL);
 #endif // INT_TIME
  // convert input from float to fixed point
  float * data = (float*)(fft_data_p->theData);
  for (int j = 0; j < 2 * (1 << log_nsamples); j++) {
    fftHW_lmem[fn][j] = float2fx(data[j], FX_IL);
  }
 #ifdef INT_TIME
  struct timeval cvtin_stop;
  gettimeofday(&cvtin_stop, NULL);
  fft_timings_p->fft_cvtin_sec[tidx]   += cvtin_stop.tv_sec  - fft_timings_p->fft_cvtin_start.tv_sec;
  fft_timings_p->fft_cvtin_usec[tidx]  += cvtin_stop.tv_usec - fft_timings_p->fft_cvtin_start.tv_usec;
 #endif // INT_TIME

  // Call the FFT Accelerator
  //    NOTE: Currently this is blocking-wait for call to complete
 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_comp_start), NULL);
 #endif // INT_TIME
  DEBUG(printf("EHFA:   MB%u calling the HW_FFT[%u]\n", task_metadata_block->block_id, fn));
  fft_in_hw(&(fftHW_fd[fn]), &(fftHW_desc[fn]));
 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  fft_timings_p->fft_comp_sec[tidx]   += stop_time.tv_sec  - fft_timings_p->fft_comp_start.tv_sec;
  fft_timings_p->fft_comp_usec[tidx]  += stop_time.tv_usec - fft_timings_p->fft_comp_start.tv_usec;
 #endif
  // convert output from fixed point to float
  DEBUG(printf("EHFA:   converting from fixed-point to float\n"));
 #ifdef INT_TIME
  gettimeofday(&(fft_timings_p->fft_cvtout_start), NULL);
 #endif // INT_TIME
  for (int j = 0; j < 2 * (1 << log_nsamples); j++) {
    data[j] = (float)fx2float(fftHW_lmem[fn][j], FX_IL);
    DEBUG(printf("MB%u : Data[ %u ] = %f\n", task_metadata_block->block_id, j, data[j]));
  }
 #ifdef INT_TIME
  struct timeval cvtout_stop;
  gettimeofday(&cvtout_stop, NULL);
  fft_timings_p->fft_cvtout_sec[tidx]   += cvtout_stop.tv_sec  - fft_timings_p->fft_cvtout_start.tv_sec;
  fft_timings_p->fft_cvtout_usec[tidx]  += cvtout_stop.tv_usec - fft_timings_p->fft_cvtout_start.tv_usec;
  /* #ifdef INT_TIME */
  /*  struct timeval stop_time; */
  /*  gettimeofday(&stop_time, NULL); */
  fft_timings_p->fft_sec[tidx]   += cvtout_stop.tv_sec  - fft_timings_p->fft_start.tv_sec;
  fft_timings_p->fft_usec[tidx]  += cvtout_stop.tv_usec - fft_timings_p->fft_start.tv_usec;
 #endif // INT_TIME

  DEBUG(printf("EHFA: MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);

#else
  printf("ERROR : This executable DOES NOT support Hardware-FFT execution!\n");
  cleanup_and_exit(-2);
#endif
}





#ifdef HW_VIT
static void do_decoding_hw(int *fd, struct vitdodec_access *desc)
{
  if (ioctl(*fd, VITDODEC_IOC_ACCESS, *desc)) {
    perror("ERROR : do_decoding_in_hw : IOCTL:");
    cleanup_and_exit(EXIT_FAILURE);
  }
}
#endif


void
execute_hwr_viterbi_accelerator(task_metadata_block_t* task_metadata_block)
{
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  int vn = task_metadata_block->accelerator_id;
  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->job_type]); // VITERBI_TASK]);
  //task_metadata_block->vit_timings.comp_by[tidx]++;
  vit_timings_p->comp_by[tidx]++;
  DEBUG(printf("EHVA: In execute_hwr_viterbi_accelerator on FFT_HWR Accel %u : MB%d  CL %d\n", vn, task_metadata_block->block_id, task_metadata_block->crit_level));
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)&(task_metadata_block->data_space);
  int32_t  in_cbps = vdata->n_cbps;
  int32_t  in_ntraceback = vdata->n_traceback;
  int32_t  in_data_bits = vdata->n_data_bits;
  int32_t  inMem_offset = 0;
  int32_t  inData_offset = vdata->inMem_size;
  int32_t  outData_offset = inData_offset + vdata->inData_size;
  uint8_t* in_Mem  = &(vdata->theData[inMem_offset]);
  uint8_t* in_Data = &(vdata->theData[inData_offset]);
  uint8_t* out_Data = &(vdata->theData[outData_offset]);

#ifdef HW_VIT
  DEBUG(printf("EHVA:   setting up HW_VIT parameters\n"));
  vitHW_desc[vn].cbps = in_cbps;
  vitHW_desc[vn].ntraceback = in_ntraceback;
  vitHW_desc[vn].data_bits = in_data_bits;

  DEBUG(printf("EHVA:   setting up HW_VIT memory\n"));
  uint8_t* hwrInMem  = vitHW_li_mem[vn];
  uint8_t* hwrOutMem = vitHW_lo_mem[vn];
  for (int ti = 0; ti < 70; ti ++) {
    hwrInMem[ti] = in_Mem[ti];
  }
  hwrInMem[70] = 0;
  hwrInMem[71] = 0;
  int imi = 72;
  for (int ti = 0; ti < MAX_ENCODED_BITS; ti ++) {
    hwrInMem[imi++] = in_Data[ti];
  }
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti ++) {
    out_Data[ti] = 0;
  }

 #ifdef INT_TIME
  gettimeofday(&(vit_timings_p->dodec_start), NULL);
 #endif
  DEBUG(printf("EHVA:   calling do_decoding_hw for HW_VIT[%u]\n", vn));
  do_decoding_hw(&(vitHW_fd[vn]), &(vitHW_desc[vn]));
 #ifdef INT_TIME
  struct timeval dodec_stop;
  gettimeofday(&(dodec_stop), NULL);
  vit_timings_p->dodec_sec[tidx]  += dodec_stop.tv_sec  - vit_timings_p->dodec_start.tv_sec;
  vit_timings_p->dodec_usec[tidx] += dodec_stop.tv_usec - vit_timings_p->dodec_start.tv_usec;
 #endif
  // Copy output data from HWR memory to Metadata Block Memory.
  DEBUG(printf("EHVA:   copying out the HW_VIT result data\n"));
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti ++) {
    out_Data[ti] = hwrOutMem[ti];
  }
  SDEBUG(printf("EHVA: MB%u at end of HWR VITERBI:\n    out_Data : ", task_metadata_block->block_id);
	for (int ti = 0; ti < 80 /*(MAX_ENCODED_BITS * 3 / 4)*/; ti ++) {
	  printf("%u ", out_Data[ti]);
	});

  DEBUG(printf("EHVA: MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);

#else // HW_VIT
  printf("ERROR : This executable DOES NOT support Viterbi Hardware execution!\n");
  cleanup_and_exit(-3);
#endif // HW_VIT
}


static inline label_t parse_output_dimg() {
  FILE *file_p = fopen("./output.dimg", "r");
  const size_t n_classes = 5;
  float probs[n_classes];
  for (size_t i = 0; i< n_classes; i++) {
    if (fscanf(file_p, "%f", &probs[i]) != 1) {
      printf("Didn't parse the probs[%ld] from output.dimg\n", i);
    }
  }
  float max_val = 0.0f;
  size_t max_idx = -1;
  for (size_t i = 0; i < n_classes; i++) {
    if (probs[i] > max_val) {
      max_val = probs[i], max_idx = i;
    }
  }
  fclose(file_p);
  return (label_t)max_idx;
}

void
execute_hwr_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  int fn = task_metadata_block->accelerator_id;
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->job_type]); // CV_TASK]);
  cv_timings_p->comp_by[tidx]++;
  TDEBUG(printf("In execute_hwr_cv_accelerator on CV_HWR Accel %u : MB%d  CL %d\n", fn, task_metadata_block->block_id, task_metadata_block->crit_level));
#ifdef HW_CV
  // Add the call to the NVDLA stuff here.
  printf("Doing the system call : './nvdla_runtime --loadable hpvm-mod.nvdla --image 2004_2.jpg --rawdump'\n");
  //printf("Doing the system call : './nvdla_runtime --loadable mio_loadable.nvdla --image three.jpg'\n");
  #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
  #endif

  int sret = system("./nvdla_runtime --loadable hpvm-mod.nvdla --image 0003_0.jpg --rawdump");
  //int sret = system("./nvdla_runtime --loadable mio_loadable.nvdla --image three.jpg");
  if (sret == -1) {
    printf(" The system call returned -1 -- an error occured?\n");
  }
 #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->parse_start), NULL);
  cv_timings_p->call_sec[tidx]  += cv_timings_p->parse_start.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[tidx]  += cv_timings_p->parse_start.tv_usec  - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("REAL_HW_CV: Set Call_Sec[%u] to %llu %llu\n", cv_timings_p->call_sec[tidx], cv_timings_p->call_usec[tidx]));
 #endif
  label_t pred_label = parse_output_dimg();
 #ifdef INT_TIME
  struct timeval stop;
  gettimeofday(&(stop), NULL);
  cv_timings_p->parse_sec[tidx]  += stop.tv_sec  - cv_timings_p->parse_start.tv_sec;
  cv_timings_p->parse_usec[tidx] += stop.tv_usec - cv_timings_p->parse_start.tv_usec;
 #endif
  TDEBUG(printf("---> Predicted label = %d\n", pred_label));
  // Set result into the metatdata block
  task_metadata_block->data_view.cv_data.object_label = pred_label;

#else // Of #ifdef HW_CV
 #ifdef FAKE_HW_CV
  #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
  #endif
  // This usleep call stands in as the "Fake" CNN accelerator
  usleep(cv_fake_hwr_run_time_in_usec);
  #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[tidx]  += stop_time.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[tidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_CV: Set Call_Sec[%u] to %lu %lu\n", tidx, cv_timings_p->call_sec[tidx], cv_timings_p->call_usec[tidx]));
  #endif

 #else
  printf("ERROR : This executable DOES NOT support Hardware-CV execution!\n");
  cleanup_and_exit(-2);
 #endif
#endif
  TDEBUG(printf("MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}



void
do_task_type_initialization()
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
do_task_type_closeout()
{
  // Clean up any hardware accelerator stuff
 #ifdef HW_VIT
  for (int vi = 0; vi < NUM_VIT_ACCEL; vi++) {
    contig_free(vitHW_mem[vi]);
    close(vitHW_fd[vi]);
  }
 #endif

 #ifdef HW_FFT
  for (int fi = 0; fi < NUM_FFT_ACCEL; fi++) {
    contig_free(fftHW_mem[fi]);
    close(fftHW_fd[fi]);
  }
 #endif
}


void
output_task_type_run_stats()
{
  printf("\nPer-MetaData-Block Job Timing Data:\n");
  printf("\n  Per-MetaData-Block FFT Timing Data:\n");
  char* ti_label[2] = {"CPU", "HWR"};
  {
    // The FFT Tasks Timing Info
    unsigned total_fft_comp_by[3] = {0, 0, 0};
    uint64_t total_fft_call_usec[3] = {0, 0, 0};
    uint64_t total_fft_usec[3] = {0, 0, 0};
    uint64_t total_fft_br_usec[3] = {0, 0, 0};
    uint64_t total_bitrev_usec[3] = {0, 0, 0};
    uint64_t total_fft_cvtin_usec[3] = {0, 0, 0};
    uint64_t total_fft_comp_usec[3] = {0, 0, 0};
    uint64_t total_fft_cvtout_usec[3] = {0, 0, 0};
    uint64_t total_cdfmcw_usec[3] = {0, 0, 0};
    for (int ti = 0; ti < 2; ti++) {
      for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
	fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(master_metadata_pool[bi].task_timings[FFT_TASK]);
        unsigned this_comp_by = (unsigned)(fft_timings_p->comp_by[ti]);
        uint64_t this_fft_call_usec = (uint64_t)(fft_timings_p->call_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->call_usec[ti]);
        uint64_t this_fft_usec = (uint64_t)(fft_timings_p->fft_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->fft_usec[ti]);
        uint64_t this_fft_br_usec = (uint64_t)(fft_timings_p->fft_br_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->fft_br_usec[ti]);
        uint64_t this_bitrev_usec = (uint64_t)(fft_timings_p->bitrev_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->bitrev_usec[ti]);
        uint64_t this_fft_cvtin_usec = (uint64_t)(fft_timings_p->fft_cvtin_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->fft_cvtin_usec[ti]);
        uint64_t this_fft_comp_usec = (uint64_t)(fft_timings_p->fft_comp_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->fft_comp_usec[ti]);
        uint64_t this_fft_cvtout_usec = (uint64_t)(fft_timings_p->fft_cvtout_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->fft_cvtout_usec[ti]);
        uint64_t this_cdfmcw_usec = (uint64_t)(fft_timings_p->cdfmcw_sec[ti]) * 1000000 + (uint64_t)(fft_timings_p->cdfmcw_usec[ti]);
        printf("Block %3u : %u %s : FFT CB %8u call %15lu fft %15lu fft_br %15lu br %15lu cvtin %15lu calc %15lu cvto %15lu fmcw %15lu usec\n", bi, ti, ti_label[ti], this_comp_by, this_fft_call_usec, this_fft_usec, this_fft_br_usec, this_bitrev_usec, this_fft_cvtin_usec, this_fft_comp_usec, this_fft_cvtout_usec, this_cdfmcw_usec);
        // Per acceleration (CPU, HWR)
        total_fft_comp_by[ti]     += this_comp_by;
        total_fft_call_usec[ti]   += this_fft_call_usec;
        total_fft_usec[ti]        += this_fft_usec;
        total_fft_br_usec[ti]     += this_fft_br_usec;
        total_bitrev_usec[ti]     += this_bitrev_usec;
        total_fft_cvtin_usec[ti]  += this_fft_cvtin_usec;
        total_fft_comp_usec[ti]   += this_fft_comp_usec;
        total_fft_cvtout_usec[ti] += this_fft_cvtout_usec;
        total_cdfmcw_usec[ti]     += this_cdfmcw_usec;
	// Overall Total
        total_fft_comp_by[2]     += this_comp_by;
        total_fft_call_usec[2]   += this_fft_call_usec;
        total_fft_usec[2]        += this_fft_usec;
        total_fft_br_usec[2]     += this_fft_br_usec;
        total_bitrev_usec[2]     += this_bitrev_usec;
        total_fft_cvtin_usec[2]  += this_fft_cvtin_usec;
        total_fft_comp_usec[2]   += this_fft_comp_usec;
        total_fft_cvtout_usec[2] += this_fft_cvtout_usec;
        total_cdfmcw_usec[2]     += this_cdfmcw_usec;
      } // for (bi over Metadata blocks)
    } // for (ti = 0, 1)    
    printf("\nAggregate FFT Tasks Total Timing Data: %u finished FFT tasks\n", freed_metadata_blocks[FFT_TASK]);
    double avg0, avg1, avg2;
    avg0 = (double)total_fft_call_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_fft_call_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_fft_call_usec[0] + total_fft_call_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     fft-call run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_fft_call_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_fft_call_usec[1], avg1, total_fft_comp_by[2], total_fft_call_usec[2], avg2);
    avg0 = (double)total_fft_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_fft_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_fft_usec[0] + total_fft_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     fft-total run time    %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_fft_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_fft_usec[1], avg1, total_fft_comp_by[2], total_fft_usec[2], avg2);
    avg0= (double)total_fft_br_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_fft_br_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_fft_br_usec[0] + total_fft_br_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     bit-reverse run time %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_fft_br_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_fft_br_usec[1], avg1, total_fft_comp_by[2], total_fft_br_usec[2], avg2);
    avg0 = (double)total_bitrev_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_bitrev_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_bitrev_usec[0] + total_bitrev_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     bit-rev run time     %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_bitrev_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_bitrev_usec[1], avg1, total_fft_comp_by[2], total_bitrev_usec[2], avg2);
    avg0 = (double)total_fft_cvtin_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_fft_cvtin_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_fft_cvtin_usec[0] + total_fft_cvtin_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     fft-cvtin run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_fft_cvtin_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_fft_cvtin_usec[1], avg1, total_fft_comp_by[2], total_fft_cvtin_usec[2], avg2);
    avg0 = (double)total_fft_comp_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_fft_comp_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_fft_comp_usec[0] + total_fft_comp_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     fft-comp run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_fft_comp_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_fft_comp_usec[1], avg1, total_fft_comp_by[2], total_fft_comp_usec[2], avg2);
    avg0 = (double)total_fft_cvtout_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_fft_cvtout_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_fft_cvtout_usec[0] + total_fft_cvtout_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     fft-cvtout run time  %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_fft_cvtout_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_fft_cvtout_usec[1], avg1, total_fft_comp_by[2], total_fft_cvtout_usec[2], avg2);
    avg0 = (double)total_cdfmcw_usec[0] / (double) freed_metadata_blocks[FFT_TASK];
    avg1 = (double)total_cdfmcw_usec[1] / (double) freed_metadata_blocks[FFT_TASK];
    avg2 = (double)(total_cdfmcw_usec[0] + total_cdfmcw_usec[1]) / (double) freed_metadata_blocks[FFT_TASK];
    printf("     calc-dist run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_fft_comp_by[0], total_cdfmcw_usec[0], avg0, 1, ti_label[1], total_fft_comp_by[1], total_cdfmcw_usec[1], avg1, total_fft_comp_by[2], total_cdfmcw_usec[2], avg2);

    printf("\n  Per-MetaData-Block VITERBI Timing Data: %u finished VITERBI tasks\n", freed_metadata_blocks[VITERBI_TASK]);
    // The Viterbi Task Timing Info
    unsigned total_vit_comp_by[3] = {0, 0, 0};
    uint64_t total_depunc_usec[3] = {0, 0, 0};
    uint64_t total_dodec_usec[3] = {0, 0, 0};
    for (int ti = 0; ti < 2; ti++) {
      for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
	vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(master_metadata_pool[bi].task_timings[VITERBI_TASK]);
        unsigned this_comp_by = (unsigned)(vit_timings_p->comp_by[ti]);
        uint64_t this_depunc_usec = (uint64_t)(vit_timings_p->depunc_sec[ti]) * 1000000 + (uint64_t)(vit_timings_p->depunc_usec[ti]);
        uint64_t this_dodec_usec = (uint64_t)(vit_timings_p->dodec_sec[ti]) * 1000000 + (uint64_t)(vit_timings_p->dodec_usec[ti]);
        printf("Block %3u : %u %s : VITERBI %8u depunc %15lu dodecode %15lu usec\n", bi, ti, ti_label[ti], this_comp_by, this_depunc_usec, this_dodec_usec);
        // Per acceleration (CPU, HWR)
        total_vit_comp_by[ti] += this_comp_by;
        total_depunc_usec[ti] += this_depunc_usec;
        total_dodec_usec[ti]  += this_dodec_usec;
        // Overall Total
        total_vit_comp_by[2] += this_comp_by;
        total_depunc_usec[2] += this_depunc_usec;
        total_dodec_usec[2]  += this_dodec_usec;
      } // for (bi = 1 .. numMetatdataBlocks)
    } // for (ti = 0, 1)
    printf("\nAggregate VITERBI Tasks Total Timing Data:\n");
    avg0 = (double)total_depunc_usec[0] / (double) freed_metadata_blocks[VITERBI_TASK];
    avg1 = (double)total_depunc_usec[1] / (double) freed_metadata_blocks[VITERBI_TASK];
    avg2 = (double)total_depunc_usec[2] / (double) freed_metadata_blocks[VITERBI_TASK];
    printf("     depuncture  run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_vit_comp_by[0], total_depunc_usec[0], avg0, 1, ti_label[1], total_vit_comp_by[1], total_depunc_usec[1], avg1, total_vit_comp_by[2], total_depunc_usec[2], avg2);
    avg0 = (double)total_dodec_usec[0] / (double) freed_metadata_blocks[VITERBI_TASK];
    avg1 = (double)total_dodec_usec[1] / (double) freed_metadata_blocks[VITERBI_TASK];
    avg2 = (double)total_dodec_usec[2] / (double) freed_metadata_blocks[VITERBI_TASK];
    printf("     do-decoding run time   %u %s%8u  %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_vit_comp_by[0], total_dodec_usec[0], avg0, 1, ti_label[1], total_vit_comp_by[1], total_dodec_usec[1], avg1, total_vit_comp_by[2], total_dodec_usec[2], avg2);


    printf("\n  Per-MetaData-Block CV Timing Data: %u finished CV tasks\n", freed_metadata_blocks[CV_TASK]);
    // The CV/CNN Task Timing Info
    unsigned total_cv_comp_by[3] = {0, 0, 0};
    uint64_t total_cv_call_usec[3] = {0, 0, 0}; // re-use the FFT one by same name...
    uint64_t total_parse_usec[3] = {0, 0, 0};
    for (int ti = 0; ti < 2; ti++) {
      for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
	cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(master_metadata_pool[bi].task_timings[CV_TASK]);
	unsigned this_comp_by = (unsigned)(cv_timings_p->comp_by[ti]);
        uint64_t this_cv_call_usec = (uint64_t)(cv_timings_p->call_sec[ti]) * 1000000 + (uint64_t)(cv_timings_p->call_usec[ti]);
        uint64_t this_parse_usec = (uint64_t)(cv_timings_p->parse_sec[ti]) * 1000000 + (uint64_t)(cv_timings_p->parse_usec[ti]);
        printf("Block %3u : %u %s : CV %8u call-time %15lu parse %15lu usec\n", bi, ti, ti_label[ti], this_comp_by, this_cv_call_usec, this_parse_usec);
        // Per acceleration (CPU, HWR)
        total_cv_comp_by[ti] += this_comp_by;
        total_cv_call_usec[ti]  += this_cv_call_usec;
        total_parse_usec[ti] += this_parse_usec;
        // Overall Total
        total_cv_comp_by[2] += this_comp_by;
        total_cv_call_usec[2]  += this_cv_call_usec;
        total_parse_usec[2] += this_parse_usec;
      } // for (bi = 1 .. numMetatdataBlocks)
    } // for (ti = 0, 1)
    printf("\nAggregate CV Tasks Total Timing Data:\n");
    avg0 = (double)total_cv_call_usec[0] / (double) freed_metadata_blocks[CV_TASK];
    avg1 = (double)total_cv_call_usec[1] / (double) freed_metadata_blocks[CV_TASK];
    avg2 = (double)total_cv_call_usec[2] / (double) freed_metadata_blocks[CV_TASK];
    printf("     CNN-call  run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_cv_comp_by[0], total_cv_call_usec[0], avg0, 1, ti_label[1], total_cv_comp_by[1], total_cv_call_usec[1], avg1, total_cv_comp_by[2], total_cv_call_usec[2], avg2);
    avg0 = (double)total_parse_usec[0] / (double) freed_metadata_blocks[CV_TASK];
    avg1 = (double)total_parse_usec[1] / (double) freed_metadata_blocks[CV_TASK];
    avg2 = (double)total_parse_usec[2] / (double) freed_metadata_blocks[CV_TASK];
    printf("     get_label run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_cv_comp_by[0], total_parse_usec[0], avg0, 1, ti_label[1], total_cv_comp_by[1], total_parse_usec[1], avg1, total_cv_comp_by[2], total_parse_usec[2], avg2);
  }

}


void execute_cpu_viterbi_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_viterbi_accelerator\n"));
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)&(task_metadata_block->data_space);
  int32_t  in_cbps = vdata->n_cbps;
  int32_t  in_ntraceback = vdata->n_traceback;
  int32_t  in_data_bits = vdata->n_data_bits;
  int32_t  inMem_offset = 0;
  int32_t  inData_offset = vdata->inMem_size;
  int32_t  outData_offset = inData_offset + vdata->inData_size;
  uint8_t* in_Mem  = &(vdata->theData[inMem_offset]);
  uint8_t* in_Data = &(vdata->theData[inData_offset]);
  uint8_t* out_Data = &(vdata->theData[outData_offset]);
  int tidx = (task_metadata_block->accelerator_type != cpu_accel_t);

  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(task_metadata_block->task_timings[VITERBI_TASK]);
  vit_timings_p->comp_by[tidx]++;

#ifdef INT_TIME
  gettimeofday(&(vit_timings_p->dodec_start), NULL);
#endif

  DEBUG(for (int i = 0; i < 20; i++) {
      printf("CPU_VIT_PRE_RUN_INPUT %3u : ID  %3u : IM  %3u  %3u\n", i, in_Data[i], in_Mem[i+inData_offset], i+inData_offset); // cpuOutMem[i]);
    });
  do_cpu_viterbi_function(in_data_bits, in_cbps, in_ntraceback, in_Mem, out_Data); // cpuInMem, cpuOutMem);
  DEBUG(for (int i = 0; i < 20; i++) {
      printf("CPU_VIT_OUT %3u : %3u @ %p \n", i, out_Data[i], &(out_Data[i])); // cpuOutMem[i]);
    });

#ifdef INT_TIME
  struct timeval dodec_stop;
  gettimeofday(&(dodec_stop), NULL);
  vit_timings_p->dodec_sec[tidx]  += dodec_stop.tv_sec  - vit_timings_p->dodec_start.tv_sec;
  vit_timings_p->dodec_usec[tidx] += dodec_stop.tv_usec - vit_timings_p->dodec_start.tv_usec;
#endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}



