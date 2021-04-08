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
#include "vit_task.h"
#include "vit_accel.h" // required for the execute_on_hwr functions


// Forward Declarations:
void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback, unsigned char *inMemory, unsigned char *outMemory);


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
static void do_decoding_hw(scheduler_datastate_block_t* sptr, int *fd, struct vitdodec_access *desc)
{
  if (ioctl(*fd, VITDODEC_IOC_ACCESS, *desc)) {
    perror("ERROR : do_decoding_in_hw : IOCTL:");
    cleanup_and_exit(sptr, EXIT_FAILURE);
  }
}
#endif

void
output_vit_task_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_task_id, unsigned total_accel_types)
{
  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n", my_task_id, sptr->task_name_str[my_task_id], sptr->freed_metadata_blocks[my_task_id], total_accel_types);
  // The Viterbi Task Timing Info
  unsigned total_vit_comp_by[total_accel_types+1];
  uint64_t total_depunc_usec[total_accel_types+1];
  uint64_t total_dodec_usec[total_accel_types+1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_vit_comp_by[ai] = 0;
    total_depunc_usec[ai] = 0;
    total_dodec_usec[ai] = 0;
  }  
  for (int ai = 0; ai < total_accel_types; ai++) {
    if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[ai][my_task_id] != NULL)) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_id, sptr->task_name_str[my_task_id], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(sptr->master_metadata_pool[bi].task_timings[my_task_id]);
      unsigned this_comp_by = (unsigned)(sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_id]);
      uint64_t this_depunc_usec = (uint64_t)(vit_timings_p->depunc_sec[ai]) * 1000000 + (uint64_t)(vit_timings_p->depunc_usec[ai]);
      uint64_t this_dodec_usec = (uint64_t)(vit_timings_p->dodec_sec[ai]) * 1000000 + (uint64_t)(vit_timings_p->dodec_usec[ai]);
      if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[ai][my_task_id] != NULL)) {
	printf("    Block %3u : AI %u %s : CmpBy %8u depunc %15lu dodecode %15lu usec\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_depunc_usec, this_dodec_usec);
      } else {
	if ((this_comp_by + this_depunc_usec + this_dodec_usec) != 0) {
	  printf("  ERROR: Block %3u : AI %u %s : CmpBy %8u depunc %15lu dodecode %15lu usec\n", bi, ai, sptr->accel_name_str[ai], this_comp_by, this_depunc_usec, this_dodec_usec);
	}
      }
      // Per acceleration (CPU, HWR)
      total_vit_comp_by[ai] += this_comp_by;
      total_depunc_usec[ai] += this_depunc_usec;
      total_dodec_usec[ai]  += this_dodec_usec;
      // Overall Total
      total_vit_comp_by[total_accel_types] += this_comp_by;
      total_depunc_usec[total_accel_types] += this_depunc_usec;
      total_dodec_usec[total_accel_types]  += this_dodec_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  } // for (ai = 0 .. total_accel_types)
  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_id, sptr->task_name_str[my_task_id]);
  printf("     depuncture  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_depunc_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai, sptr->accel_name_str[ai], total_vit_comp_by[ai], total_depunc_usec[ai], avg);
  }
  {
    double avg = (double)total_depunc_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_vit_comp_by[total_accel_types], total_depunc_usec[total_accel_types], avg);
  }
  
  printf("     do-decoding run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_dodec_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", ai, sptr->accel_name_str[ai], total_vit_comp_by[ai], total_dodec_usec[ai], avg);
  }
  {
    double avg = (double)total_dodec_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                            ", total_accel_types, "TOTAL", total_vit_comp_by[total_accel_types], total_dodec_usec[total_accel_types], avg);
  }
}


void
exec_vit_task_on_vit_hwr_accel(task_metadata_block_t* task_metadata_block)
{
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  int aidx = task_metadata_block->accelerator_type;
  int vn = task_metadata_block->accelerator_id;
  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  DEBUG(printf("EHVA: In exec_vit_task_on_vit_hwr_accel on FFT_HWR Accel %u : MB%d  CL %d\n", vn, task_metadata_block->block_id, task_metadata_block->crit_level));
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
  do_decoding_hw(sptr, &(vitHW_fd[vn]), &(vitHW_desc[vn]));
 #ifdef INT_TIME
  struct timeval dodec_stop;
  gettimeofday(&(dodec_stop), NULL);
  vit_timings_p->dodec_sec[aidx]  += dodec_stop.tv_sec  - vit_timings_p->dodec_start.tv_sec;
  vit_timings_p->dodec_usec[aidx] += dodec_stop.tv_usec - vit_timings_p->dodec_start.tv_usec;
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
  cleanup_and_exit(sptr, -3);
#endif // HW_VIT
}


void exec_vit_task_on_cpu_accel(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In exec_vit_task_on_cpu_accel\n"));
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
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
  int aidx = task_metadata_block->accelerator_type;

  vit_timing_data_t * vit_timings_p = (vit_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;

#ifdef INT_TIME
  gettimeofday(&(vit_timings_p->dodec_start), NULL);
#endif

  SDEBUG(for (int i = 0; i < 20; i++) {
      printf("CPU_VIT_PRE_RUN_INPUT %3u : ID  %3u : IM  %3u  %3u\n", i, in_Data[i], in_Mem[i+inData_offset], i+inData_offset); // cpuOutMem[i]);
    });
  do_cpu_viterbi_function(in_data_bits, in_cbps, in_ntraceback, in_Mem, out_Data); // cpuInMem, cpuOutMem);
  SDEBUG(for (int i = 0; i < 20; i++) {
      printf("CPU_VIT_OUT %3u : %3u @ %p \n", i, out_Data[i], &(out_Data[i])); // cpuOutMem[i]);
    });

#ifdef INT_TIME
  struct timeval dodec_stop;
  gettimeofday(&(dodec_stop), NULL);
  vit_timings_p->dodec_sec[aidx]  += dodec_stop.tv_sec  - vit_timings_p->dodec_start.tv_sec;
  vit_timings_p->dodec_usec[aidx] += dodec_stop.tv_usec - vit_timings_p->dodec_start.tv_usec;
#endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
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
#include "viterbi_flat.h"
#include "viterbi_standalone.h"


#undef  GENERATE_CHECK_VALUES
//#define  GENERATE_CHECK_VALUES


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


void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback, unsigned char *inMemory, unsigned char *outMemory)
{
  int in_count = 0;
  int out_count = 0;
  int n_decoded = 0;


  unsigned char* d_brtab27[2] = { &(inMemory[    0]), 
                                  &(inMemory[   32]) };

  unsigned char*  in_depuncture_pattern  = &(inMemory[   64]);
  uint8_t* depd_data                     = &(inMemory[   72]);
  uint8_t* l_decoded                     = &(outMemory[   0]);

  DO_VERBOSE({
      printf("\nVBS: in_cbps        = %u\n", in_cbps);
      printf("VBS: in_ntraceback  = %u\n", in_ntraceback);
      printf("VBS: in_n_data_bits = %u\n", in_n_data_bits);
      for (int ti = 0; ti < 2; ti ++) {
	printf("d_brtab[%u] = [ ", ti);
	for (int tj = 0; tj < 32; tj++) {
	  if (tj > 0) { printf(", "); }
	  printf("%u", d_brtab27[ti][tj]);
	}
	printf(" ]\n");
      }
      printf("VBS: in_depuncture_pattern = [ ");
      for (int ti = 0; ti < 6; ti ++) {
	if (ti > 0) { printf(", "); }
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

  uint8_t  l_metric0_generic[64];
  uint8_t  l_metric1_generic[64];
  uint8_t  l_path0_generic[64];
  uint8_t  l_path1_generic[64];
  uint8_t  l_mmresult[64];
  uint8_t  l_ppresult[TRACEBACK_MAX][64];
  int      l_store_pos = 0;

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
  while(n_decoded < in_n_data_bits) {
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
       * 	encode_state(n) = encode_state(n+65)
       *	encode_state(n+1) = encode_state(n+64) = (3 ^ encode_state(n))
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

	unsigned char m0[16], m1[16], m2[16], m3[16], decision0[16], decision1[16], survivor0[16], survivor1[16];
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
	second_symbol = first_symbol+1;
	for (int j = 0; j < 16; j++) {
	  sym0v[j] = symbols[first_symbol];
	  sym1v[j] = symbols[second_symbol];
	}

	for (int s = 0; s < 2; s++) { // iterate across the 2 symbol groups
	  // This is the basic viterbi butterfly for 2 symbols (we need therefore 2 passes for 4 total symbols)
	  for (int i = 0; i < 2; i++) {
	    if (symbols[first_symbol] == 2) {
	      for (int j = 0; j < 16; j++) {
		metsvm[j] = d_brtab27[1][(i*16) + j] ^ sym1v[j];
		metsv[j] = 1 - metsvm[j];
	      }
	    }
	    else if (symbols[second_symbol] == 2) {
	      for (int j = 0; j < 16; j++) {
		metsvm[j] = d_brtab27[0][(i*16) + j] ^ sym0v[j];
		metsv[j] = 1 - metsvm[j];
	      }
	    }
	    else {
	      for (int j = 0; j < 16; j++) {
		metsvm[j] = (d_brtab27[0][(i*16) + j] ^ sym0v[j]) + (d_brtab27[1][(i*16) + j] ^ sym1v[j]);
		metsv[j] = 2 - metsvm[j];
	      }
	    }

	    for (int j = 0; j < 16; j++) {
	      m0[j] = metric0[(i*16) + j] + metsv[j];
	      m1[j] = metric0[((i+2)*16) + j] + metsvm[j];
	      m2[j] = metric0[(i*16) + j] + metsvm[j];
	      m3[j] = metric0[((i+2)*16) + j] + metsv[j];
	    }

	    for (int j = 0; j < 16; j++) {
	      decision0[j] = ((m0[j] - m1[j]) > 0) ? 0xff : 0x0;
	      decision1[j] = ((m2[j] - m3[j]) > 0) ? 0xff : 0x0;
	      survivor0[j] = (decision0[j] & m0[j]) | ((~decision0[j]) & m1[j]);
	      survivor1[j] = (decision1[j] & m2[j]) | ((~decision1[j]) & m3[j]);
	    }

	    for (int j = 0; j < 16; j += 2) {
	      simd_epi16 = path0[(i*16) + j];
	      simd_epi16 |= path0[(i*16) + (j+1)] << 8;
	      simd_epi16 <<= 1;
	      shift0[j] = simd_epi16;
	      shift0[j+1] = simd_epi16 >> 8;

	      simd_epi16 = path0[((i+2)*16) + j];
	      simd_epi16 |= path0[((i+2)*16) + (j+1)] << 8;
	      simd_epi16 <<= 1;
	      shift1[j] = simd_epi16;
	      shift1[j+1] = simd_epi16 >> 8;
	    }
	    for (int j = 0; j < 16; j++) {
	      shift1[j] = shift1[j] + 1;
	    }

	    for (int j = 0, k = 0; j < 16; j += 2, k++) {
	      metric1[(2*i*16) + j] = survivor0[k];
	      metric1[(2*i*16) + (j+1)] = survivor1[k];
	    }
	    for (int j = 0; j < 16; j++) {
	      tmp0[j] = (decision0[j] & shift0[j]) | ((~decision0[j]) & shift1[j]);
	    }

	    for (int j = 0, k = 8; j < 16; j += 2, k++) {
	      metric1[((2*i+1)*16) + j] = survivor0[k];
	      metric1[((2*i+1)*16) + (j+1)] = survivor1[k];
	    }
	    for (int j = 0; j < 16; j++) {
	      tmp1[j] = (decision1[j] & shift0[j]) | ((~decision1[j]) & shift1[j]);
	    }

	    for (int j = 0, k = 0; j < 16; j += 2, k++) {
	      path1[(2*i*16) + j] = tmp0[k];
	      path1[(2*i*16) + (j+1)] = tmp1[k];
	    }
	    for (int j = 0, k = 8; j < 16; j += 2, k++) {
	      path1[((2*i+1)*16) + j] = tmp0[k];
	      path1[((2*i+1)*16) + (j+1)] = tmp1[k];
	    }
	  }

	  // Set up for the second two symbols (2 and 3)
	  metric0 = mm1;
	  path0 = pp1;
	  metric1 = mm0;
	  path1 = pp0;
	  first_symbol = 2;
	  second_symbol = first_symbol+1;
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
	      l_mmresult[(i*16) + j] = mm0[(i*16) + j];
	      l_ppresult[l_store_pos][(i*16) + j] = pp0[(i*16) + j];
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
	      pp0[(i*16) + j] = 0;
	      mm0[(i*16) + j] = mm0[(i*16) + j] - minmetric;
	    }
	  }

	  //return bestmetric;
	}

	//std::cout << "OUTPUT: " << (unsigned int)c << std::endl; 
	if (out_count >= in_ntraceback) {
	  for (int i= 0; i < 8; i++) {
	    l_decoded[(out_count - in_ntraceback) * 8 + i] = (c >> (7 - i)) & 0x1;
	    SDEBUG(printf("l_decoded[ %u ] oc %u tb %u i %u written as %u\n", (out_count - in_ntraceback) * 8 + i, out_count, in_ntraceback, i, l_decoded[(out_count - in_ntraceback) * 8 + i]));
	    n_decoded++;
	  }
	}
	out_count++;
      }
    }
    in_count++;
  }
}





