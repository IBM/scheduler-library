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
#include "vit_sched.h"
#include "vit_accel.h"
#include "accelerators.h" // include AFTER scheduler.h -- needs types form scheduler.h

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
do_vit_task_type_initialization()
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
do_vit_task_type_closeout()
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
output_vit_task_type_run_stats()
{
  char* ti_label[2] = {"CPU", "HWR"};
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
  double avg0, avg1, avg2;
  avg0 = (double)total_depunc_usec[0] / (double) freed_metadata_blocks[VITERBI_TASK];
  avg1 = (double)total_depunc_usec[1] / (double) freed_metadata_blocks[VITERBI_TASK];
  avg2 = (double)total_depunc_usec[2] / (double) freed_metadata_blocks[VITERBI_TASK];
  printf("     depuncture  run time   %u %s %8u %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_vit_comp_by[0], total_depunc_usec[0], avg0, 1, ti_label[1], total_vit_comp_by[1], total_depunc_usec[1], avg1, total_vit_comp_by[2], total_depunc_usec[2], avg2);
  avg0 = (double)total_dodec_usec[0] / (double) freed_metadata_blocks[VITERBI_TASK];
  avg1 = (double)total_dodec_usec[1] / (double) freed_metadata_blocks[VITERBI_TASK];
  avg2 = (double)total_dodec_usec[2] / (double) freed_metadata_blocks[VITERBI_TASK];
  printf("     do-decoding run time   %u %s%8u  %15lu usec %16.3lf avg : %u %s %8u %15lu usec %16.3lf avg : TOT %8u %15lu usec %16.3lf avg\n", 0, ti_label[0], total_vit_comp_by[0], total_dodec_usec[0], avg0, 1, ti_label[1], total_vit_comp_by[1], total_dodec_usec[1], avg1, total_vit_comp_by[2], total_dodec_usec[2], avg2);
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



