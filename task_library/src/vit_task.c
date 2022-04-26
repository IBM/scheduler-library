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

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "viterbi_base.h"
 //#define VERBOSE
#include "verbose.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "scheduler.h"
#include "vit_accel.h" // required for the execute_on_hwr functions
#include "vit_task.h"


#ifdef INT_TIME
extern uint64_t call_sec;
extern uint64_t dodec_sec;
extern uint64_t dodec_usec;

extern uint64_t call_usec;
extern uint64_t depunc_sec;
extern uint64_t depunc_usec;
#endif

extern ofdm_param ofdm;
extern frame_param frame;

std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> vit_profile; // Vit messages can by short,


// Metrics for each state
unsigned char d_mmresult[64] __attribute__((aligned(16)));
// Paths for each state
unsigned char d_ppresult[TRACEBACK_MAX][64] __attribute__((aligned(16)));

// Forward Declarations:
void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback,
  unsigned char * inMemory, unsigned char * outMemory);
void vit_task_reset();
void vit_task_start_decode(task_metadata_entry * vit_metadata_block, ofdm_param * ofdm,
  frame_param * frame, uint8_t * in);
uint8_t * vit_task_finish_decode(task_metadata_entry * mb_ptr, int * n_dec_char);

// extern "C" {
//   void print_viterbi_metadata_block_contents(/*task_metadata_entry*/ void * mb_ptr) {
//     task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
//     print_base_metadata_block_contents(mb);
//     viterbi_data_struct_t * vdata = (viterbi_data_struct_t *) (mb->data_space);
//     int32_t  inData_offset = vdata->inMem_size;
//     int32_t  outData_offset = inData_offset + vdata->inData_size;
//     uint8_t * in_Mem = &(vdata->theData[0]);
//     uint8_t * in_Data = &(vdata->theData[inData_offset]);
//     uint8_t * out_Data = &(vdata->theData[outData_offset]);
//     printf("   Viterbi Data: @ %p\n", vdata);
//     printf("      n_cbps      = %d\n", vdata->n_cbps);
//     printf("      n_traceback = %d\n", vdata->n_traceback);
//     printf("      n_data_bits = %d\n", vdata->n_data_bits);
//     printf("      psdu_size   = %d\n", vdata->psdu_size);
//     printf("      in_Mem_size   = %d\n", vdata->inMem_size);
//     printf("      in_Data_size  = %d\n", vdata->inData_size);
//     printf("      out_Data_size = %d\n", vdata->outData_size);
//     printf("      inMem_offset  = %d\n", 0);
//     printf("      inData_offset  = %d\n", inData_offset);
//     printf("      outData_offset = %d\n", outData_offset);
//     printf("      in_Mem   @ %p\n", &(vdata->theData[0]));
//     printf("      in_Data  @ %p\n", &(vdata->theData[inData_offset]));
//     printf("      out_Data @ %p\n", &(vdata->theData[outData_offset]));
//   }
// }

extern "C" {
  void output_vit_task_type_run_stats(/*scheduler_datastate*/ void * sptr_ptr, unsigned my_task_type,
    unsigned total_accel_types) {
    scheduler_datastate * sptr = (scheduler_datastate *) sptr_ptr;
    printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n",
      my_task_type, sptr->task_name_str[my_task_type], sptr->freed_metadata_blocks[my_task_type],
      total_accel_types);
    // The Viterbi Task Timing Info
    unsigned total_vit_comp_by[total_accel_types + 1];
    uint64_t total_call_usec[total_accel_types + 1];
    uint64_t total_depunc_usec[total_accel_types + 1];
    uint64_t total_dodec_usec[total_accel_types + 1];
    for (int ai = 0; ai <= total_accel_types; ai++) {
      total_vit_comp_by[ai] = 0;
      total_call_usec[ai] = 0;
      total_depunc_usec[ai] = 0;
      total_dodec_usec[ai] = 0;
    }
    for (int ai = 0; ai < total_accel_types; ai++) {
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type,
          sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
      }
      for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
        vit_timing_data_t * vit_timings_p = (vit_timing_data_t *) &
          (sptr->master_metadata_pool[bi].task_timings[my_task_type]);
        unsigned this_comp_by = (unsigned) (
          sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
        uint64_t this_call_usec = (uint64_t) (vit_timings_p->call_sec[ai]) * 1000000 + (uint64_t) (
          vit_timings_p->call_usec[ai]);
        uint64_t this_depunc_usec = (uint64_t) (vit_timings_p->depunc_sec[ai]) * 1000000 + (uint64_t) (
          vit_timings_p->depunc_usec[ai]);
        uint64_t this_dodec_usec = (uint64_t) (vit_timings_p->dodec_sec[ai]) * 1000000 + (uint64_t) (
          vit_timings_p->dodec_usec[ai]);
        if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
          printf("    Block %3u : AI %u %s : CmpBy %8u depunc %15lu dodecode %15lu call %15lu usec\n",
            bi, ai, sptr->accel_name_str[ai], this_comp_by, this_depunc_usec, this_dodec_usec, this_call_usec);
        }
        else {
          if ((this_comp_by + this_depunc_usec + this_dodec_usec) != 0) {
            printf("  ERROR: Block %3u : AI %u %s : CmpBy %8u depunc %15lu dodecode %15lu call %15lu usec\n",
              bi, ai, sptr->accel_name_str[ai], this_comp_by, this_depunc_usec, this_dodec_usec, this_call_usec);
          }
        }
        // Per acceleration (CPU, HWR)
        total_vit_comp_by[ai] += this_comp_by;
        total_call_usec[ai] += this_call_usec;
        total_depunc_usec[ai] += this_depunc_usec;
        total_dodec_usec[ai] += this_dodec_usec;
        // Overall Total
        total_vit_comp_by[total_accel_types] += this_comp_by;
        total_call_usec[total_accel_types] += this_call_usec;
        total_depunc_usec[total_accel_types] += this_depunc_usec;
        total_dodec_usec[total_accel_types] += this_dodec_usec;
        //printf("VIT: call_usec %lu : total %u = %lu : TOTAL = %lu\n", this_call_usec, ai, total_call_usec[ai], total_call_usec[total_accel_types]);
      } // for (bi = 1 .. numMetatdataBlocks)
    } // for (ai = 0 .. total_accel_types)
    printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type,
      sptr->task_name_str[my_task_type]);
    printf("     Call        run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_call_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai,
        sptr->accel_name_str[ai], total_vit_comp_by[ai], total_call_usec[ai], avg);
    }
    {
      double avg = (double) total_call_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_vit_comp_by[total_accel_types], total_call_usec[total_accel_types], avg);
    }

    printf("     depuncture  run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_depunc_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai,
        sptr->accel_name_str[ai], total_vit_comp_by[ai], total_depunc_usec[ai], avg);
    }
    {
      double avg = (double) total_depunc_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_vit_comp_by[total_accel_types], total_depunc_usec[total_accel_types], avg);
    }

    printf("     do-decoding run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_dodec_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai,
        sptr->accel_name_str[ai], total_vit_comp_by[ai], total_dodec_usec[ai], avg);
    }
    {
      double avg = (double) total_dodec_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", total_accel_types,
        "TOTAL", total_vit_comp_by[total_accel_types],
        total_dodec_usec[total_accel_types], avg);
    }
  }
}

extern "C" {
  void exec_vit_task_on_vit_hwr_accel( /*task_metadata_entry*/ void * vit_io_ptr) {//, int accelerator_id) {
    int vn = 0; //accelerator_id;
    vit_io_t * vdata = (vit_io_t *) (vit_io_ptr);
    int32_t  in_cbps = vdata->ofdm_ptr->n_cbps;
    int32_t  in_ntraceback = d_ntraceback;
    int32_t  in_data_bits = vdata->frame_ptr->n_data_bits;
    int32_t  inData_offset = 72;
    int32_t  outData_offset = inData_offset + MAX_ENCODED_BITS;
    uint8_t * in_Mem = &(vdata->vit_data[0]);
    uint8_t * in_Data = &(vdata->vit_data[inData_offset]);
    uint8_t * out_Data = &(vdata->vit_data[outData_offset]);

#ifdef HW_VIT
    DEBUG(printf("EHVA:   setting up HW_VIT parameters\n"));
    vitHW_desc[vn].cbps = in_cbps;
    vitHW_desc[vn].ntraceback = in_ntraceback;
    vitHW_desc[vn].data_bits = in_data_bits;

    DEBUG(printf("EHVA:   setting up HW_VIT memory\n"));
    uint8_t * hwrInMem = (uint8_t *) vitHW_li_mem[vn];
    uint8_t * hwrOutMem = (uint8_t *) vitHW_lo_mem[vn];
    for (int ti = 0; ti < 70; ti++) {
      hwrInMem[ti] = in_Mem[ti];
    }
    hwrInMem[70] = 0;
    hwrInMem[71] = 0;
    int imi = 72;
    for (int ti = 0; ti < MAX_ENCODED_BITS; ti++) {
      hwrInMem[imi++] = in_Data[ti];
    }
    for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti++) {
      out_Data[ti] = 0;
    }

    DEBUG(printf("EHVA:   calling do_decoding_hw for HW_VIT[%u]\n", vn));
    do_decoding_hw(&(vitHW_fd[vn]), &(vitHW_desc[vn]));
    // Copy output data from HWR memory to Metadata Block Memory.
    DEBUG(printf("EHVA:   copying out the HW_VIT result data\n"));
    for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti++) {
      out_Data[ti] = hwrOutMem[ti];
    }
    SDEBUG(
      for (int ti = 0; ti < 80 /*(MAX_ENCODED_BITS * 3 / 4)*/; ti++) {
        printf("%u ", out_Data[ti]);
      }
    );

#else // HW_VIT
    printf("ERROR : This executable DOES NOT support Viterbi Hardware execution!\n");
    exit(-3);
#endif // HW_VIT
  }
}


void set_up_vit_task_on_accel_profile_data() {
  for (int si = 0; si < 4; si++) {
    for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
      vit_profile[si][ai] = ACINFPROF;
    }
  }
#ifdef COMPILE_TO_ESP
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  vit_profile[0][SCHED_CPU_ACCEL_T] = 120000;
  vit_profile[0][SCHED_EPOCHS_VITDEC_ACCEL_T] = 5950;
  vit_profile[1][SCHED_CPU_ACCEL_T] = 1700000;
  vit_profile[1][SCHED_EPOCHS_VITDEC_ACCEL_T] = 67000;
  vit_profile[2][SCHED_CPU_ACCEL_T] = 3400000;
  vit_profile[2][SCHED_EPOCHS_VITDEC_ACCEL_T] = 135000;
  vit_profile[3][SCHED_CPU_ACCEL_T] = 4800000;
  vit_profile[3][SCHED_EPOCHS_VITDEC_ACCEL_T] = 191000;
#else
  vit_profile[0][SCHED_CPU_ACCEL_T] = 200;
  vit_profile[1][SCHED_CPU_ACCEL_T] = 2400;
  vit_profile[2][SCHED_CPU_ACCEL_T] = 5000;
  vit_profile[3][SCHED_CPU_ACCEL_T] = 6600;
#endif

  DEBUG(printf("\n%18s : %18s %18s %18s %18s\n", "VIT-PROFILES", "CPU", "FFT-HWR", "VITT-HWR",
    "CV-HWR");
  printf("%15s :", "vit_profile[0]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", vit_profile[0][ai]); } printf("\n");
  printf("%15s :", "vit_profile[1]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", vit_profile[1][ai]); } printf("\n");
  printf("%15s :", "vit_profile[2]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", vit_profile[2][ai]); } printf("\n");
  printf("%15s :", "vit_profile[3]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", vit_profile[3][ai]); } printf("\n");
  printf("\n"));
}
