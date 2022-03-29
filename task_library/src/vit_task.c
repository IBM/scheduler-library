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
#include "viterbi_types.h"
//#define VERBOSE
#include "verbose.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "scheduler.h"
#include "vit_accel.h" // required for the execute_on_hwr functions
#include "vit_task.h"

// GLOBAL VARIABLES
typedef union branchtab27_u {
  unsigned char c[32];
} t_branchtab27;

int d_ntraceback;
int d_k;

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

#ifdef COMPILE_TO_ESP
extern t_branchtab27 d_branchtab27_generic[2];
#else
t_branchtab27 d_branchtab27_generic[2];
#endif


ofdm_param ofdm = { (Encoding) 0,   //  encoding   : 0 = BPSK_1_2
                    13,   //  rate_field : rate field ofSIGNAL header
                    1,   //  n_bpsc     : coded bits per subcarrier
                    48,   //  n_cbps     : coded bits per OFDM symbol
                    24
                  }; //  n_dbps     : data bits per OFDM symbol

frame_param frame = {  1528,    // psdu_size      : PSDU size in bytes
                       511,    // n_sym          : number of OFDM symbols
                       18,    // n_pad          : number of padding bits in DATA field
                       24528,    // n_encoded_bits : number of encoded bits
                       12264
                    };  // n_data_bits    : number of data bits, including service and padding


// Metrics for each state
unsigned char d_mmresult[64] __attribute__((aligned(16)));
// Paths for each state
unsigned char d_ppresult[TRACEBACK_MAX][64] __attribute__((aligned(16)));

// Forward Declarations:
void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback,
                             unsigned char *inMemory, unsigned char *outMemory);
void vit_task_reset();
void vit_task_start_decode(task_metadata_entry *vit_metadata_block, ofdm_param *ofdm,
                           frame_param *frame, uint8_t *in);
uint8_t *vit_task_finish_decode(task_metadata_entry *mb_ptr, int *n_dec_char);

extern "C" {
void print_viterbi_metadata_block_contents(/*task_metadata_entry*/ void *mb_ptr) {
  task_metadata_entry *mb = (task_metadata_entry *)mb_ptr;
  print_base_metadata_block_contents(mb);
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)(mb->data_space);
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
  printf("      in_Data  @ %p\n", &(vdata->theData[inData_offset]));
  printf("      out_Data @ %p\n", &(vdata->theData[outData_offset]));
}
}

extern "C" {
void output_vit_task_type_run_stats(/*scheduler_datastate*/ void *sptr_ptr, unsigned my_task_type,
    unsigned total_accel_types) {
  scheduler_datastate *sptr = (scheduler_datastate *)sptr_ptr;
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
      vit_timing_data_t *vit_timings_p = (vit_timing_data_t *) &
                                         ( sptr->master_metadata_pool[bi].task_timings[my_task_type]);
      unsigned this_comp_by = (unsigned)(
                                sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
      uint64_t this_call_usec = (uint64_t)(vit_timings_p->call_sec[ai]) * 1000000 + (uint64_t)(
                                  vit_timings_p->call_usec[ai]);
      uint64_t this_depunc_usec = (uint64_t)(vit_timings_p->depunc_sec[ai]) * 1000000 + (uint64_t)(
                                    vit_timings_p->depunc_usec[ai]);
      uint64_t this_dodec_usec = (uint64_t)(vit_timings_p->dodec_sec[ai]) * 1000000 + (uint64_t)(
                                   vit_timings_p->dodec_usec[ai]);
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("    Block %3u : AI %u %s : CmpBy %8u depunc %15lu dodecode %15lu call %15lu usec\n",
               bi, ai, sptr->accel_name_str[ai], this_comp_by, this_depunc_usec, this_dodec_usec, this_call_usec);
      } else {
        if ((this_comp_by + this_depunc_usec + this_dodec_usec) != 0) {
          printf("  ERROR: Block %3u : AI %u %s : CmpBy %8u depunc %15lu dodecode %15lu call %15lu usec\n",
                 bi, ai, sptr->accel_name_str[ai], this_comp_by, this_depunc_usec, this_dodec_usec, this_call_usec);
        }
      }
      // Per acceleration (CPU, HWR)
      total_vit_comp_by[ai] += this_comp_by;
      total_call_usec[ai] += this_call_usec;
      total_depunc_usec[ai] += this_depunc_usec;
      total_dodec_usec[ai]  += this_dodec_usec;
      // Overall Total
      total_vit_comp_by[total_accel_types] += this_comp_by;
      total_call_usec[total_accel_types] += this_call_usec;
      total_depunc_usec[total_accel_types] += this_depunc_usec;
      total_dodec_usec[total_accel_types]  += this_dodec_usec;
      //printf("VIT: call_usec %lu : total %u = %lu : TOTAL = %lu\n", this_call_usec, ai, total_call_usec[ai], total_call_usec[total_accel_types]);
    } // for (bi = 1 .. numMetatdataBlocks)
  } // for (ai = 0 .. total_accel_types)
  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type,
         sptr->task_name_str[my_task_type]);
  printf("     Call        run time\n                          ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_call_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai,
           sptr->accel_name_str[ai], total_vit_comp_by[ai], total_call_usec[ai], avg);
  }
  {
    double avg = (double)total_call_usec[total_accel_types] / (double)
                 sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
           total_vit_comp_by[total_accel_types], total_call_usec[total_accel_types], avg);
  }

  printf("     depuncture  run time\n                          ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_depunc_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai,
           sptr->accel_name_str[ai], total_vit_comp_by[ai], total_depunc_usec[ai], avg);
  }
  {
    double avg = (double)total_depunc_usec[total_accel_types] / (double)
                 sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
           total_vit_comp_by[total_accel_types], total_depunc_usec[total_accel_types], avg);
  }

  printf("     do-decoding run time\n                          ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_dodec_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai,
           sptr->accel_name_str[ai], total_vit_comp_by[ai], total_dodec_usec[ai], avg);
  }
  {
    double avg = (double)total_dodec_usec[total_accel_types] / (double)
                 sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", total_accel_types,
           "TOTAL", total_vit_comp_by[total_accel_types],
           total_dodec_usec[total_accel_types], avg);
  }
}
}

extern "C" {
void exec_vit_task_on_vit_hwr_accel( /*task_metadata_entry*/ void *task_metadata_block_ptr) {
  task_metadata_entry *task_metadata_block = (task_metadata_entry *)task_metadata_block_ptr;
  scheduler_datastate *sptr = task_metadata_block->scheduler_datastate_pointer;
  int aidx = task_metadata_block->accelerator_type;
  int vn = task_metadata_block->accelerator_id;
  vit_timing_data_t *vit_timings_p = (vit_timing_data_t *) &
                                     (task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  DEBUG(printf("EHVA: In exec_vit_task_on_vit_hwr_accel on VIT_HWR Accel %u : MB%d  CL %d\n", vn,
               task_metadata_block->block_id,
               task_metadata_block->crit_level));
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)(task_metadata_block->data_space);
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
  uint8_t * hwrInMem = (uint8_t *) vitHW_li_mem[vn];
  uint8_t * hwrOutMem = (uint8_t *) vitHW_lo_mem[vn];
  for (int ti = 0; ti < 70; ti ++) {
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

#ifdef INT_TIME
  gettimeofday(&(vit_timings_p->dodec_start), NULL);
#endif
  DEBUG(printf("EHVA:   calling do_decoding_hw for HW_VIT[%u]\n", vn));
  do_decoding_hw(sptr, &(vitHW_fd[vn]), &(vitHW_desc[vn]));
#ifdef INT_TIME
  struct timeval dodec_stop;
  gettimeofday(&(dodec_stop), NULL);
  vit_timings_p->dodec_sec[aidx]  += dodec_stop.tv_sec  - vit_timings_p->dodec_start.tv_sec;
  vit_timings_p->dodec_usec[aidx] += dodec_stop.tv_usec - vit_timings_p->dodec_start.tv_usec;
#endif
  // Copy output data from HWR memory to Metadata Block Memory.
  DEBUG(printf("EHVA:   copying out the HW_VIT result data\n"));
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti++) {
    out_Data[ti] = hwrOutMem[ti];
  }
  SDEBUG(printf("EHVA: MB%u at end of HWR VITERBI:\n    out_Data : ", task_metadata_block->block_id);
  for (int ti = 0; ti < 80 /*(MAX_ENCODED_BITS * 3 / 4)*/; ti ++) {
  printf("%u ", out_Data[ti]);
  });

  DEBUG(printf("EHVA: MB%u VIT_HWR calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);

#else // HW_VIT
  printf("ERROR : This executable DOES NOT support Viterbi Hardware execution!\n");
  exit(-3);
#endif // HW_VIT
}
}

extern "C" {
void exec_vit_task_on_cpu_accel(/*task_metadata_entry*/ void *task_metadata_block_ptr) {
  task_metadata_entry *task_metadata_block = (task_metadata_entry *)task_metadata_block_ptr;
  DEBUG(printf("In exec_vit_task_on_cpu_accel\n"));
  scheduler_datastate* sptr = task_metadata_block->scheduler_datastate_pointer;
  viterbi_data_struct_t* vdata = (viterbi_data_struct_t*)(task_metadata_block->data_space);
  int32_t  in_cbps = vdata->n_cbps;
  int32_t  in_ntraceback = vdata->n_traceback;
  int32_t  in_data_bits = vdata->n_data_bits;
  int32_t  inMem_offset = 0;
  int32_t  inData_offset = vdata->inMem_size;
  int32_t  outData_offset = inData_offset + vdata->inData_size;
  uint8_t* in_Mem  = &(vdata->theData[inMem_offset]);
  uint8_t* in_Data = &(vdata->theData[inData_offset]);
  uint8_t* out_Data = &(vdata->theData[outData_offset]);
  int aidx = task_metadata_block->accelerator_type;

  vit_timing_data_t* vit_timings_p = (vit_timing_data_t *) &
                                     (task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;

#ifdef INT_TIME
  gettimeofday(&(vit_timings_p->dodec_start), NULL);
#endif

  SDEBUG(for (int i = 0; i < 20; i++) {
  printf("CPU_VIT_PRE_RUN_INPUT %3u : ID  %3u : IM  %3u  %3u\n", i, in_Data[i],
         in_Mem[i + inData_offset], i + inData_offset); // cpuOutMem[i]);
  });
  do_cpu_viterbi_function(in_data_bits, in_cbps, in_ntraceback, in_Mem,
                          out_Data); // cpuInMem, cpuOutMem);
  SDEBUG(for (int i = 0; i < 20; i++) {
  printf("CPU_VIT_OUT %3u : %3u @ %p \n", i, out_Data[i], &(out_Data[i])); // cpuOutMem[i]);
  });

#ifdef INT_TIME
  struct timeval dodec_stop;
  gettimeofday(&(dodec_stop), NULL);
  vit_timings_p->dodec_sec[aidx]  += dodec_stop.tv_sec  - vit_timings_p->dodec_start.tv_sec;
  vit_timings_p->dodec_usec[aidx] += dodec_stop.tv_usec - vit_timings_p->dodec_start.tv_usec;
#endif

  TDEBUG(printf("[%u.%u] MB%u VIT_CPU calling mark_task_done...\n", task_metadata_block->dag_id,
                task_metadata_block->task_id, task_metadata_block->block_id););
  mark_task_done(task_metadata_block);
}
}


/*
 * Copyright 1995 Phil Karn, KA9Q
 * Copyright 2008 Free Software Foundation, Inc.
 * 2014 Added SSE2 implementation Bogdan Diaconescu
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/*
 * Viterbi decoder for K=7 rate=1/2 convolutional code
 * Some modifications from original Karn code by Matt Ettus
 * Major modifications by adding SSE2 code by Bogdan Diaconescu
 */

/* This is the main "do_decoding" function; takes the necessary inputs
 * from the decode call (above) and does the decoding, outputing the decoded result.
 */
// INPUTSOUTPUTS:          :  I/O   : Offset : Size
//    in_cbps               : INPUT  :     X  : int = 4 bytes (REGISTER)
//    in_ntraceback         : INPUT  :     X  : int = 4 bytes (REGISTER)
//    in_n_data_bits        : INPUT  :     X  : int = 4 bytes (REGISTER)
//    d_branchtab27_generic : INPUT  :     0  : uint8_t[2][32] = 64 bytes
//    in_depuncture_pattern : INPUT  :    64  : uint8_t[8] (max is 6 bytes + 2 padding bytes)
//    depd_data             : INPUT  :    72  : uint8_t[MAX_ENCODED_BITS == 24780] (depunctured data)
//    <return_val>          : OUTPUT : 24852  : uint8_t[MAX_ENCODED_BITS * 3 / 4 == 18585 ] : The decoded data stream

/* THESE ARE JUST USED LOCALLY IN THIS FUNCTION NOW  */
/*  BUT they must reset to zero on each invocation   */
/*  AND they might be used in other places in GnuRadio? */
//    d_metric0_generic     : INPUT  : uint8_t[64]
//    d_metric1_generic     : INPUT  : uint8_t[64]
//    d_path0_generic       : INPUT  : uint8_t[64]
//    d_path1_generic       : INPUT  : uint8_t[64]
//    d_store_pos           : INPUT  : int (position in circular traceback buffer?)
//    d_mmresult            : OUTPUT : uint8_t[64]
//    d_ppresult            : OUTPUT : uint8_t[ntraceback_MAX][ 64 bytes ]


void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback,
                             unsigned char *inMemory, unsigned char *outMemory) {
  int in_count = 0;
  int out_count = 0;
  int n_decoded = 0;

  unsigned char *d_brtab27[2] = {&(inMemory[0]), &(inMemory[32])};

  unsigned char *in_depuncture_pattern = &(inMemory[64]);
  uint8_t *depd_data = &(inMemory[72]);
  uint8_t *l_decoded = &(outMemory[0]);

  DO_VERBOSE({
    printf("\nVBS: in_cbps        = %u\n", in_cbps);
    printf("VBS: in_ntraceback  = %u\n", in_ntraceback);
    printf("VBS: in_n_data_bits = %u\n", in_n_data_bits);
    for (int ti = 0; ti < 2; ti++) {
      printf("d_brtab[%u] = [ ", ti);
      for (int tj = 0; tj < 32; tj++) {
        if (tj > 0) {
          printf(", ");
        }
        printf("%u", d_brtab27[ti][tj]);
      }
      printf(" ]\n");
    }
    printf("VBS: in_depuncture_pattern = [ ");
    for (int ti = 0; ti < 6; ti++) {
      if (ti > 0) {
        printf(", ");
      }
      printf("%02x", in_depuncture_pattern[ti]);
    }
    printf("]\n");
    printf("\nVBS: depd_data : %p\n", depd_data);
    {
      int per_row = 0;
      printf("%p : ", &depd_data[0]);
      for (int ti = 0; ti < MAX_ENCODED_BITS; ti++) {
        per_row++;
        if ((per_row % 8) == 0) {
          printf(" ");
        }
        printf("%u", depd_data[ti]);
        if (per_row == 39) {
          printf("\n");
          printf("%p : ", &depd_data[ti]);
          per_row = 0;
        }
      }
      printf("\n");
    }
    printf("\n");
    printf("\n\n");
  });

  uint8_t l_metric0_generic[64];
  uint8_t l_metric1_generic[64];
  uint8_t l_path0_generic[64];
  uint8_t l_path1_generic[64];
  uint8_t l_mmresult[64];
  uint8_t l_ppresult[TRACEBACK_MAX][64];
  int l_store_pos = 0;

  // This is the "reset" portion:
  //  Do this before the real operation so local memories are "cleared to zero"
  // d_store_pos = 0;
  for (int i = 0; i < 64; i++) {
    l_metric0_generic[i] = 0;
    l_path0_generic[i] = 0;
    l_metric1_generic[i] = 0;
    l_path1_generic[i] = 0;
    l_mmresult[i] = 0;
    for (int j = 0; j < TRACEBACK_MAX; j++) {
      l_ppresult[j][i] = 0;
    }
  }

  int viterbi_butterfly_calls = 0;
  while (n_decoded < in_n_data_bits) {
    if ((in_count % 4) == 0) { //0 or 3
      /* The basic Viterbi decoder operation, called a "butterfly"
       * operation because of the way it looks on a trellis diagram. Each
       * butterfly involves an Add-Compare-Select (ACS) operation on the two nodes
       * where the 0 and 1 paths from the current node merge at the next step of
       * the trellis.
       *
       * The code polynomials are assumed to have 1's on both ends. Given a
       * function encode_state() that returns the two symbols for a given
       * encoder state in the low two bits, such a code will have the following
       * identities for even 'n' < 64:
       *
       *  encode_state(n) = encode_state(n+65)
       *  encode_state(n+1) = encode_state(n+64) = (3 ^ encode_state(n))
       *
       * Any convolutional code you would actually want to use will have
       * these properties, so these assumptions aren't too limiting.
       *
       * Doing this as a macro lets the compiler evaluate at compile time the
       * many expressions that depend on the loop index and encoder state and
       * emit them as immediate arguments.
       * This makes an enormous difference on register-starved machines such
       * as the Intel x86 family where evaluating these expressions at runtime
       * would spill over into memory.
       */
      {
        unsigned char *mm0       = l_metric0_generic;
        unsigned char *mm1       = l_metric1_generic;
        unsigned char *pp0       = l_path0_generic;
        unsigned char *pp1       = l_path1_generic;
        unsigned char *symbols   = &depd_data[in_count & 0xfffffffc];

        // These are used to "virtually" rename the uses below (for symmetry; reduces code size)
        //  Really these are functionally "offset pointers" into the above arrays....
        unsigned char *metric0, *metric1;
        unsigned char *path0, *path1;

        // Operate on 4 symbols (2 bits) at a time

        unsigned char m0[16], m1[16], m2[16], m3[16], decision0[16], decision1[16], survivor0[16],
                 survivor1[16];
        unsigned char metsv[16], metsvm[16];
        unsigned char shift0[16], shift1[16];
        unsigned char tmp0[16], tmp1[16];
        unsigned char sym0v[16], sym1v[16];
        unsigned short simd_epi16;
        unsigned int   first_symbol;
        unsigned int   second_symbol;

        // Set up for the first two symbols (0 and 1)
        metric0 = mm0;
        path0 = pp0;
        metric1 = mm1;
        path1 = pp1;
        first_symbol = 0;
        second_symbol = first_symbol + 1;
        for (int j = 0; j < 16; j++) {
          sym0v[j] = symbols[first_symbol];
          sym1v[j] = symbols[second_symbol];
        }

        for (int s = 0; s < 2; s++) { // iterate across the 2 symbol groups
          // This is the basic viterbi butterfly for 2 symbols (we need therefore 2 passes for 4 total symbols)
          for (int i = 0; i < 2; i++) {
            if (symbols[first_symbol] == 2) {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = d_brtab27[1][(i * 16) + j] ^ sym1v[j];
                metsv[j] = 1 - metsvm[j];
              }
            } else if (symbols[second_symbol] == 2) {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = d_brtab27[0][(i * 16) + j] ^ sym0v[j];
                metsv[j] = 1 - metsvm[j];
              }
            } else {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = (d_brtab27[0][(i * 16) + j] ^ sym0v[j]) + (d_brtab27[1][(i * 16) + j] ^ sym1v[j]);
                metsv[j] = 2 - metsvm[j];
              }
            }

            for (int j = 0; j < 16; j++) {
              m0[j] = metric0[(i * 16) + j] + metsv[j];
              m1[j] = metric0[((i + 2) * 16) + j] + metsvm[j];
              m2[j] = metric0[(i * 16) + j] + metsvm[j];
              m3[j] = metric0[((i + 2) * 16) + j] + metsv[j];
            }

            for (int j = 0; j < 16; j++) {
              decision0[j] = ((m0[j] - m1[j]) > 0) ? 0xff : 0x0;
              decision1[j] = ((m2[j] - m3[j]) > 0) ? 0xff : 0x0;
              survivor0[j] = (decision0[j] & m0[j]) | ((~decision0[j]) & m1[j]);
              survivor1[j] = (decision1[j] & m2[j]) | ((~decision1[j]) & m3[j]);
            }

            for (int j = 0; j < 16; j += 2) {
              simd_epi16 = path0[(i * 16) + j];
              simd_epi16 |= path0[(i * 16) + (j + 1)] << 8;
              simd_epi16 <<= 1;
              shift0[j] = simd_epi16;
              shift0[j + 1] = simd_epi16 >> 8;

              simd_epi16 = path0[((i + 2) * 16) + j];
              simd_epi16 |= path0[((i + 2) * 16) + (j + 1)] << 8;
              simd_epi16 <<= 1;
              shift1[j] = simd_epi16;
              shift1[j + 1] = simd_epi16 >> 8;
            }
            for (int j = 0; j < 16; j++) {
              shift1[j] = shift1[j] + 1;
            }

            for (int j = 0, k = 0; j < 16; j += 2, k++) {
              metric1[(2 * i * 16) + j] = survivor0[k];
              metric1[(2 * i * 16) + (j + 1)] = survivor1[k];
            }
            for (int j = 0; j < 16; j++) {
              tmp0[j] = (decision0[j] & shift0[j]) | ((~decision0[j]) & shift1[j]);
            }

            for (int j = 0, k = 8; j < 16; j += 2, k++) {
              metric1[((2 * i + 1) * 16) + j] = survivor0[k];
              metric1[((2 * i + 1) * 16) + (j + 1)] = survivor1[k];
            }
            for (int j = 0; j < 16; j++) {
              tmp1[j] = (decision1[j] & shift0[j]) | ((~decision1[j]) & shift1[j]);
            }

            for (int j = 0, k = 0; j < 16; j += 2, k++) {
              path1[(2 * i * 16) + j] = tmp0[k];
              path1[(2 * i * 16) + (j + 1)] = tmp1[k];
            }
            for (int j = 0, k = 8; j < 16; j += 2, k++) {
              path1[((2 * i + 1) * 16) + j] = tmp0[k];
              path1[((2 * i + 1) * 16) + (j + 1)] = tmp1[k];
            }
          }

          // Set up for the second two symbols (2 and 3)
          metric0 = mm1;
          path0 = pp1;
          metric1 = mm0;
          path1 = pp0;
          first_symbol = 2;
          second_symbol = first_symbol + 1;
          for (int j = 0; j < 16; j++) {
            sym0v[j] = symbols[first_symbol];
            sym1v[j] = symbols[second_symbol];
          }
        }
      } // END of call to viterbi_butterfly2_generic
      viterbi_butterfly_calls++; // Do not increment until after the comparison code.

      if ((in_count > 0) && (in_count % 16) == 8) { // 8 or 11
        unsigned char c;
        //  Find current best path
        //
        // INPUTS/OUTPUTS:
        //    RET_VAL     : (ignored)
        //    mm0         : INPUT/OUTPUT  : Array [ 64 ]
        //    mm1         : INPUT/OUTPUT  : Array [ 64 ]
        //    pp0         : INPUT/OUTPUT  : Array [ 64 ]
        //    pp1         : INPUT/OUTPUT  : Array [ 64 ]
        //    ntraceback  : INPUT         : int (I think effectively const for given run type; here 5 I think)
        //    outbuf      : OUTPUT        : 1 byte
        //    l_store_pos : GLOBAL IN/OUT : int (position in circular traceback buffer?)

        //    l_mmresult  : GLOBAL OUTPUT : Array [ 64 bytes ]
        //    l_ppresult  : GLOBAL OUTPUT : Array [ntraceback][ 64 bytes ]

        // CALL : viterbi_get_output_generic(l_metric0_generic, l_path0_generic, in_ntraceback, &c);
        // unsigned char viterbi_get_output_generic(unsigned char *mm0, unsigned char *pp0, int ntraceback, unsigned char *outbuf)
        {
          unsigned char *mm0       = l_metric0_generic;
          unsigned char *pp0       = l_path0_generic;
          int ntraceback = in_ntraceback;
          unsigned char *outbuf = &c;

          int i;
          int bestmetric, minmetric;
          int beststate = 0;
          int pos = 0;
          int j;

          // circular buffer with the last ntraceback paths
          l_store_pos = (l_store_pos + 1) % ntraceback;

          for (i = 0; i < 4; i++) {
            for (j = 0; j < 16; j++) {
              l_mmresult[(i * 16) + j] = mm0[(i * 16) + j];
              l_ppresult[l_store_pos][(i * 16) + j] = pp0[(i * 16) + j];
            }
          }

          // Find out the best final state
          bestmetric = l_mmresult[beststate];
          minmetric = l_mmresult[beststate];

          for (i = 1; i < 64; i++) {
            if (l_mmresult[i] > bestmetric) {
              bestmetric = l_mmresult[i];
              beststate = i;
            }
            if (l_mmresult[i] < minmetric) {
              minmetric = l_mmresult[i];
            }
          }

          // Trace back
          for (i = 0, pos = l_store_pos; i < (ntraceback - 1); i++) {
            // Obtain the state from the output bits
            // by clocking in the output bits in reverse order.
            // The state has only 6 bits
            beststate = l_ppresult[pos][beststate] >> 2;
            pos = (pos - 1 + ntraceback) % ntraceback;
          }

          // Store output byte
          *outbuf = l_ppresult[pos][beststate];

          for (i = 0; i < 4; i++) {
            for (j = 0; j < 16; j++) {
              pp0[(i * 16) + j] = 0;
              mm0[(i * 16) + j] = mm0[(i * 16) + j] - minmetric;
            }
          }

          //return bestmetric;
        }

        //std::cout << "OUTPUT: " << (unsigned int)c << std::endl;
        if (out_count >= in_ntraceback) {
          for (int i = 0; i < 8; i++) {
            l_decoded[(out_count - in_ntraceback) * 8 + i] = (c >> (7 - i)) & 0x1;
            SDEBUG(printf("l_decoded[ %u ] oc %u tb %u i %u written as %u\n",
                          (out_count - in_ntraceback) * 8 + i, out_count, in_ntraceback, i,
                          l_decoded[(out_count - in_ntraceback) * 8 + i]));
            n_decoded++;
          }
        }
        out_count++;
      }
    }
    in_count++;
  }
}

/*
 * Copyright 1995 Phil Karn, KA9Q
 * Copyright 2008 Free Software Foundation, Inc.
 * 2014 Added SSE2 implementation Bogdan Diaconescu
 *
 * This file is part of GNU Radio
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

/*
 * Viterbi decoder for K=7 rate=1/2 convolutional code
 * Some modifications from original Karn code by Matt Ettus
 * Major modifications by adding SSE2 code by Bogdan Diaconescu
 */

/* #undef DEBUG */
/*  #define DEBUG(x) x */
/* #undef DO_VERBOSE */
/*  #define DO_VERBOSE(x) x */

// This routine "depunctures" the input data stream according to the
//  relevant encoding parameters, etc. and returns the depunctured data.

uint8_t* vit_task_depuncture(uint8_t *in) {
  int count;
  int n_cbps = d_ofdm->n_cbps;
  uint8_t *depunctured;
  //printf("Depunture call...\n");
  if (d_ntraceback == 5) {
    count = d_frame->n_sym * n_cbps;
    depunctured = in;
  } else {
    depunctured = d_depunctured;
    count = 0;
    for (int i = 0; i < d_frame->n_sym; i++) {
      for (int k = 0; k < n_cbps; k++) {
        while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
          depunctured[count] = 2;
          count++;
        }

        // Insert received bits
        depunctured[count] = in[i * n_cbps + k];
        count++;

        while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
          depunctured[count] = 2;
          count++;
        }
      }
    }
  }
  //printf("  depuncture count = %u\n", count);
  return depunctured;
}

void vit_task_reset() {
  int i, j;

  int polys[2] = { 0x6d, 0x4f };
  for (i = 0; i < 32; i++) {
    d_branchtab27_generic[0].c[i] = (polys[0] < 0) ^ PARTAB[(2 * i) & abs(polys[0])] ? 1 : 0;
    d_branchtab27_generic[1].c[i] = (polys[1] < 0) ^ PARTAB[(2 * i) & abs(polys[1])] ? 1 : 0;
  }

  switch (d_ofdm->encoding) {
  case BPSK_1_2:
  case QPSK_1_2:
  case QAM16_1_2:
    d_ntraceback = 5;
    d_depuncture_pattern = PUNCTURE_1_2;
    d_k = 1;
    break;
  case QAM64_2_3:
    d_ntraceback = 9;
    d_depuncture_pattern = PUNCTURE_2_3;
    d_k = 2;
    break;
  case BPSK_3_4:
  case QPSK_3_4:
  case QAM16_3_4:
  case QAM64_3_4:
    d_ntraceback = 10;
    d_depuncture_pattern = PUNCTURE_3_4;
    d_k = 3;
    break;
  }
}

/* This is the main "decode" function; it prepares data and repeatedly
 * calls the viterbi butterfly2 routine to do steps of decoding.
 */
// INPUTS/OUTPUTS:
//    ofdm   : INPUT  : Struct (see utils.h) [enum, char, int, int, int]
//    frame  : INPUT  : Struct (see utils.h) [int, int, int, int]
//    in     : INPUT  : uint8_t Array [ MAX_ENCODED_BITS == 24780 ]
//  <return> : OUTPUT : uint8_t Array [ MAX_ENCODED_BITS * 3 / 4 == 18585 ] : The decoded data stream

//uint8_t* decode(ofdm_param *ofdm, frame_param *frame, uint8_t *in, int* n_dec_char) {
void
vit_task_start_decode(task_metadata_entry* vit_metadata_block, ofdm_param *ofdm, frame_param *frame,
                      uint8_t *in) {
  d_ofdm = ofdm;
  d_frame = frame;
  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*) &
                                      (vit_metadata_block->task_timings[vit_metadata_block->task_type]);
  vit_task_reset();

#ifdef INT_TIME
  gettimeofday(&vit_timings_p->depunc_start, NULL);
#endif
  uint8_t *depunctured = vit_task_depuncture(in);
#ifdef INT_TIME
  struct timeval depunc_stop;
  gettimeofday(&vit_timings_p->depunc_stop, NULL);
#endif
  DO_VERBOSE({
    printf("VBS: depunctured = [\n");
    for (int ti = 0; ti < MAX_ENCODED_BITS; ti ++) {
      if (ti > 0) { printf(", "); }
      if ((ti > 0) && ((ti % 8) == 0)) { printf("  "); }
      if ((ti > 0) && ((ti % 40) == 0)) { printf("\n"); }
      printf("%02x", depunctured[ti]);
    }
    printf("\n");
  });

  // Set up the task_metadata scope block
  vit_metadata_block->data_size = 43365; // MAX size?
  // Copy over our task data to the MetaData Block
  // Get a viterbi_data_struct_t "View" of the metablock data pointer.
  // Copy inputs into the vdsptr data view of the metadata_block metadata data segment
  viterbi_data_struct_t* vdsptr = (viterbi_data_struct_t*)(vit_metadata_block->data_space);
  vdsptr->n_data_bits = frame->n_data_bits;
  vdsptr->n_cbps      = ofdm->n_cbps;
  vdsptr->n_traceback = d_ntraceback;
  vdsptr->psdu_size   = frame->psdu_size;
  vdsptr->inMem_size = 72; // fixed -- always (add the 2 padding bytes)
  vdsptr->inData_size = MAX_ENCODED_BITS; // Using the max value here for now/safety
  vdsptr->outData_size = (MAX_ENCODED_BITS * 3 / 4); //  Using the max value here for now/safety
  uint8_t* in_Mem   = &(vdsptr->theData[0]);
  uint8_t* in_Data  = &(vdsptr->theData[vdsptr->inMem_size]);
  uint8_t* out_Data = &(vdsptr->theData[vdsptr->inMem_size + vdsptr->inData_size]);
  // Copy some multi-block stuff into a single memory (cleaner transport)
  DEBUG(printf("SET UP VITERBI TASK: \n");
        print_viterbi_metadata_block_contents(vit_metadata_block);
        printf("      in_Mem   @ %p\n", in_Mem);
        printf("      in_Data  @ %p\n", in_Data);
        printf("      out_Data @ %p\n", out_Data));
  {
    // scope block for definition of imi
    int imi = 0;
    for (int ti = 0; ti < 2; ti++) {
      for (int tj = 0; tj < 32; tj++) {
        in_Mem[imi++] = d_branchtab27_generic[ti].c[tj];
      }
    }
    if (imi != 64) { printf("ERROR : imi = %u and should be 64\n", imi); }
    // imi = 64;
    for (int ti = 0; ti < 6; ti++) {
      vdsptr->theData[imi++] = d_depuncture_pattern[ti];
    }
    if (imi != 70) { printf("ERROR : imi = %u and should be 70\n", imi); }
  } // scope block for defn of imi

  for (int ti = 0; ti < MAX_ENCODED_BITS;
       ti++) { // This is over-kill for messages that are not max size
    in_Data[ti] = depunctured[ti];
    //DEBUG(if (ti < 32) { printf("HERE : in_Data %3u : %u\n", ti, in_Data[ti]); });
  }
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4);
       ti++) { // This zeros out the full-size OUTPUT area
    out_Data[ti] = 0;
    // vdsptr->theData[imi++] = 0;
  }
  // Call the do_decoding routine
#ifdef INT_TIME
  gettimeofday(&(vit_timings_p->call_start), NULL);
#endif
}


//uint8_t* decode(ofdm_param *ofdm, frame_param *frame, uint8_t *in, int* n_dec_char) {
uint8_t* vit_task_finish_decode(task_metadata_entry* vit_metadata_block, int* psdu_size_out) {
  // Set up the Viterbit Data view of the metatdata block data
  int aidx = vit_metadata_block->accelerator_type;
  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*) &
                                      (vit_metadata_block->task_timings[vit_metadata_block->task_type]);

  viterbi_data_struct_t* vdsptr = (viterbi_data_struct_t*)(vit_metadata_block->data_space);
  uint8_t* in_Mem   = &(vdsptr->theData[0]);
  uint8_t* in_Data  = &(vdsptr->theData[vdsptr->inMem_size]);
  uint8_t* out_Data = &(vdsptr->theData[vdsptr->inMem_size + vdsptr->inData_size]);

  *psdu_size_out = vdsptr->psdu_size;

  // We write this timing here, since we now know the Accelerator ID to which this is accounted.
#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  vit_timings_p->call_sec[aidx]  += stop_time.tv_sec  - vit_timings_p->call_start.tv_sec;
  vit_timings_p->call_usec[aidx] += stop_time.tv_usec - vit_timings_p->call_start.tv_usec;
  //printf("VIT: call_start_usec = %lu  call_stop_usec = %lu\n", vit_timings_p->call_start.tv_usec, stop_time.tv_usec);

  vit_timings_p->depunc_sec[aidx]  += vit_timings_p->depunc_stop.tv_sec  -
                                      vit_timings_p->depunc_start.tv_sec;
  vit_timings_p->depunc_usec[aidx] += vit_timings_p->depunc_stop.tv_usec -
                                      vit_timings_p->depunc_start.tv_usec;
  //printf("Set AIDX %u depunc_sec = %lu  depunc_sec = %lu\n", aidx, vit_timings_p->depunc_sec[aidx], vit_timings_p->depunc_usec[aidx]);
#endif

  DEBUG(printf("MB%u is in vit_task_finish_decode for VITERBI TASK:\n", vit_metadata_block->block_id);
        print_viterbi_metadata_block_contents(vit_metadata_block));
  SDEBUG(printf("MB%u OUTPUT: ", vit_metadata_block->block_id));
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti++) { // This covers the full-size OUTPUT area
    d_decoded[ti] = out_Data[ti];
    //DEBUG(if (ti < 31) { printf("FIN_VIT_OUT %3u : %3u @ %p \n", ti, out_Data[ti], &(out_Data[ti]));});
    SDEBUG(if (ti < 80) { printf("%u", out_Data[ti]); });
  }
  SDEBUG(printf("\n\n");
  for (int i = 0; i < 32; i++) {
  printf("VIT_OUT %3u : %3u \n", i, d_decoded[i]);
  });

  return d_decoded;
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

extern "C" {
/*task_metadata_entry*/ void *
set_up_vit_task(/*scheduler_datastate*/ void *sptr_ptr,
                                        task_type_t vit_task_type, task_criticality_t crit_level,
                                        bool use_auto_finish, int32_t dag_id, int32_t task_id, void *args) {
  scheduler_datastate *sptr = (scheduler_datastate *)sptr_ptr;
#ifdef TIME
  gettimeofday(&start_exec_vit, NULL);
#endif
  // message_size_t msize, ofdm_param* ofdm_ptr, frame_param* frame_ptr,
  // uint8_t* in_bits)
  message_size_t msize = ((viterbi_input_t *) args)->msize;
  ofdm_param *ofdm_ptr = ((viterbi_input_t *) args)->ofdm_ptr;
  size_t ofdm_param_size = ((viterbi_input_t *) args)->ofdm_param_size;
  frame_param *frame_ptr = ((viterbi_input_t *) args)->frame_ptr;
  size_t frame_ptr_size = ((viterbi_input_t *) args)->frame_ptr_size;
  uint8_t *in_bits = ((viterbi_input_t *) args)->in_bits;
  size_t in_bits_size = ((viterbi_input_t *) args)->in_bits_size;


  // Request a MetadataBlock (for an VIT task at Critical Level)
  task_metadata_entry *vit_mb_ptr = NULL;
  DEBUG(printf("Calling get_task_metadata_block for Critical VIT-Task %u\n", vit_task_type));
  do {
    vit_mb_ptr = get_task_metadata_block(sptr, dag_id, task_id, vit_task_type, crit_level,
                                         vit_profile[msize]);
    // usleep(get_mb_holdoff);
  } while (0); //(*mb_ptr == NULL);
#ifdef TIME
  struct timeval got_time;
  gettimeofday(&got_time, NULL);
  exec_get_vit_sec  += got_time.tv_sec  - start_exec_vit.tv_sec;
  exec_get_vit_usec += got_time.tv_usec - start_exec_vit.tv_usec;
#endif
  //printf("VIT Crit Profile: %e %e %e %e %e\n", vit_profile[crit_vit_samples_set][0], vit_profile[crit_vit_samples_set][1], vit_profile[crit_vit_samples_set][2], vit_profile[crit_vit_samples_set][3], vit_profile[crit_vit_samples_set][4]);
  if (vit_mb_ptr == NULL) {
    // We ran out of metadata blocks -- PANIC!
    printf("Out of metadata blocks for VIT -- PANIC Quit the run (for now)\n");
    dump_all_metadata_blocks_states(sptr);
    exit(-4);
  }
  if (use_auto_finish) {
    vit_mb_ptr->atFinish = (void (*)(task_metadata_entry *))(
                             sptr->auto_finish_task_function[vit_task_type]); // get_auto_finish_routine(vit_task_type);
  } else {
    vit_mb_ptr->atFinish = NULL;
  }

  DEBUG(printf("MB%u In start_execution_of_vit_kernel\n", vit_mb_ptr->block_id));
  // Send each message (here they are all the same) through the viterbi decoder
  //DEBUG(printf("  MB%u Calling the viterbi decode routine for message %u\n", mb_ptr->block_id, trace_msg->msg_num));
  // NOTE: vit_task_start_decode ends in the request_execution (for now)
  vit_task_start_decode(vit_mb_ptr, ofdm_ptr, frame_ptr, in_bits);

  // This now ends this block -- we've kicked off execution
  return vit_mb_ptr;
}
}

// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
extern "C" {
void viterbi_auto_finish_routine(/*task_metadata_entry*/ void *mb_ptr) {
  task_metadata_entry *mb = (task_metadata_entry *)mb_ptr;
  TDEBUG(scheduler_datastate *sptr = mb->scheduler_datastate_pointer;
         printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",
                mb->block_id, sptr->task_name_str[mb->task_type],
                sptr->task_criticality_str[mb->crit_level],
                sptr->accel_name_str[mb->accelerator_type],
                mb->accelerator_id));
  DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
  free_task_metadata_block(mb);
}
}

extern void descrambler(uint8_t* in, int psdusize, char* out_msg, uint8_t* ref, uint8_t *msg);

// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the VIT results (As we used to)
//   but this seems un-necessary since we only want the final "distance" really.
extern "C" {
void finish_viterbi_execution(
  /*task_metadata_entry*/ void *vit_metadata_block_ptr,
  void *args) { // message_t* message_id)

  task_metadata_entry *vit_metadata_block = (task_metadata_entry *)vit_metadata_block_ptr;
  // message_size_t msize, ofdm_param* ofdm_ptr, frame_param* frame_ptr, uint8_t* in_bits)
  message_t *message_id = (message_t *) args;
  // char *out_msg_text = va_arg(var_list, char *);
  // struct timeval stop_vit_post_processing, start_vit_post_processing;
  // uint64_t vit_post_processing_sec = 0LL;
  // uint64_t vit_post_processing_usec = 0LL;

  // gettimeofday(&start_vit_post_processing, NULL);
  char out_msg_text[1600];

  message_t msg = NUM_MESSAGES;
  uint8_t *result;

  int psdusize; // set by vit_task_finish_decode call...
  DEBUG(printf("  MB%u Calling the vit_task_finish_decode routine\n", vit_metadata_block->block_id));
  result = vit_task_finish_decode(vit_metadata_block, &psdusize);
  // descramble the output - put it in result
  DEBUG(printf("  MB%u Calling the viterbi descrambler routine; psdusize = %u\n",
               vit_metadata_block->block_id, psdusize));
  descrambler(result, psdusize, out_msg_text, NULL /*descram_ref*/, NULL /*msg*/);

  // Here we look at the message string and select proper message_t
  switch (out_msg_text[3]) {
  case '0' : msg = safe_to_move_right_or_left; break;
  case '1' : msg = safe_to_move_right_only; break;
  case '2' : msg = safe_to_move_left_only; break;
  case '3' : msg = unsafe_to_move_left_or_right; break;
  default  : msg = NUM_MESSAGES; break;
  }
  DEBUG(printf("MB%u The finish_viterbi_execution found message %u\n", vit_metadata_block->block_id,
               msg));
  *message_id = msg;

  // gettimeofday(&stop_vit_post_processing, NULL);
  // vit_post_processing_sec += stop_vit_post_processing.tv_sec - start_vit_post_processing.tv_sec;
  // vit_post_processing_usec += stop_vit_post_processing.tv_usec - start_vit_post_processing.tv_usec;

  // int64_t vit_post_processing_time = 1000000 * vit_post_processing_sec + vit_post_processing_usec;

  // std::cout << "Vit post processing time: " << vit_post_processing_time << std::endl;

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u fin_vit Calling free_task_metadata_block\n", vit_metadata_block->block_id));
  free_task_metadata_block(vit_metadata_block);
}
}