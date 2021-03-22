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
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)&(mb->data_view.vit_data);
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


static void init_vit_parameters(int vn)
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
static void init_fft_parameters(unsigned n, uint32_t log_nsamples)
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
  fft_timing_data_t * fft_timings_p = (fft_timing_data_t*)&(task_metadata_block->task_timings[FFT_TASK]);
  //fft_data_struct_t * fft_data_p = task_metadata_block->task_timings[FFT_TASK];
  uint32_t log_nsamples = task_metadata_block->data_view.fft_data.log_nsamples;
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
  float * data = (float*)(task_metadata_block->data_view.fft_data.theData);
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
  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(task_metadata_block->task_timings[VITERBI_TASK]);
  //task_metadata_block->vit_timings.comp_by[tidx]++;
  vit_timings_p->comp_by[tidx]++;
  DEBUG(printf("EHVA: In execute_hwr_viterbi_accelerator on FFT_HWR Accel %u : MB%d  CL %d\n", vn, task_metadata_block->block_id, task_metadata_block->crit_level));
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)&(task_metadata_block->data_view.vit_data);
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
  vit_timings_p->dodec_usec[tidx] += dodec_stop.tv_usec - vit_timings_p->\dodec_start.tv_usec;
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
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[CV_TASK]);
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

