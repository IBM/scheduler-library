#ifdef HPVM
#define MAX_ENCODED_BITS ((16 + 8 * MAX_PSDU_SIZE + 6) * 2 + 288)
#define TRACEBACK_MAX 24

#include "hpvm_tasks.h"
#include "base_types.h"
#include "verbose.h"
#include "unistd.h"

#include "viterbi_base.h"
#include "viterbi_types.h"
#include "viterbi_standalone.h"

#ifdef COMPILE_TO_ESP
// This would be RISC-V FPGA
static unsigned cv_cpu_run_time_in_usec = 5000000;
#else
// This would be x86
static unsigned cv_cpu_run_time_in_usec = 10000;
#endif

#ifdef FAKE_HW_CV
#ifdef COMPILE_TO_ESP
static unsigned cv_fake_hwr_run_time_in_usec = 150000;
#else
static unsigned cv_fake_hwr_run_time_in_usec = 1000;
#endif
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define RADAR_c 300000000.0 // Speed of Light in Meters/Sec

static unsigned int _rev(unsigned int v) {
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

// GLOBAL VARIABLES for Viterbi decoding
extern t_branchtab27 d_branchtab27_generic[2];

int d_ntraceback;
int d_k;

void descrambler(uint8_t * in, int psdusize, char * out_msg, uint8_t * ref, uint8_t * msg) { //definition
  uint32_t output_length = (psdusize) +2; //output is 2 more bytes than psdu_size
  uint32_t msg_length = (psdusize) -28;
  uint8_t out[output_length];
  int state = 0; //start
  int verbose = ((ref != NULL) && (msg != NULL));
  // find the initial state of LFSR (linear feedback shift register: 7 bits) from first 7 input bits
  for (int i = 0; i < 7; i++) {
    if (*(in + i)) {
      state |= 1 << (6 - i);
    }
  }
  //init o/p array to zeros
  for (int i = 0; i < output_length; i++) {
    out[i] = 0;
  }

  out[0] = state; //initial value
  int feedback;
  int bit;
  int index = 0;
  int mod = 0;
  for (int i = 7; i < (psdusize * 8) + 16; i++) { // 2 bytes more than psdu_size -> convert to bits
    feedback = ((!!(state & 64))) ^ (!!(state & 8));
    bit = feedback ^ (*(in + i) & 0x1);
    index = i / 8;
    mod = i % 8;
    int comp1, comp2, val, comp3;
    comp1 = (bit << mod);
    val = out[index];
    comp2 = val | comp1;
    out[index] = comp2;
    comp3 = out[index];
    state = ((state << 1) & 0x7e) | feedback;
  }

  for (int i = 0; i < msg_length; i++) {
    out_msg[i] = out[i + 26];
  }
  out_msg[msg_length] = '\0';

  if (verbose) {
    printf("\n");
    printf(">>>>>> Descrambler output is here: >>>>>> \n");
    int  des_error_count = 0;
    for (int i = 0; i < output_length; i++) {
      if (out[i] != *(ref + i)) {
        printf(">>>>>> Miscompare: descrambler[%d] = %u vs %u = EXPECTED_VALUE[%d]\n", i, out[i], *(ref + i), i);
        des_error_count++;
      }
    }
    if (des_error_count != 0) {
      printf(">>>>>> Mismatch in the descrambler block, please check the inputs and algorithm one last time. >>>>>> \n");
    }
    else {
      printf("!!!!!! Great Job, descrambler algorithm works fine for the given configuration. !!!!!! \n");
    }
    printf("\n");
    printf(">>>>>> Decoded text message is here: >>>>>> \n");

    for (int i = 0; i < msg_length; i++) {
      printf("%c", out_msg[i]);
    }
    printf("\n");

    int  msg_error_count = 0;
    for (int i = 0; i < msg_length; i++) {
      if (out_msg[i] != *(msg + i)) {
        printf(">>>>>> Miscompare: text_msg[%c] = %c vs %c = EXPECTED_VALUE[%c]\n", i, out_msg[i], *(msg + i), i);
        msg_error_count++;
      }
    }
    if (msg_error_count != 0) {
      printf(">>>>>> Mismatch in the text message, please check the inputs and algorithm one last time. >>>>>> \n");
    }
    else {
      printf("!!!!!! Great Job, text message decoding algorithm works fine for the given configuration. !!!!!! \n");
    }
    printf("\n");
  }
}

void
#ifdef HCC
inline __attribute__((always_inline))
#endif
vit_task_reset(ofdm_param * ofdm_ptr) {
  switch (ofdm_ptr->encoding) {
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

void
#ifdef HCC
inline __attribute__((always_inline))
#endif
vit_task_depuncture(ofdm_param * ofdm_ptr, frame_param * frame_ptr, uint8_t * in_bits, uint8_t * vit_depunctured) {
  int count;
  int n_cbps = ofdm_ptr->n_cbps;
  //printf("Depunture call...\n");
  if (d_ntraceback == 5) {
    count = frame_ptr->n_sym * n_cbps;
    for (int i = 0; i < MAX_ENCODED_BITS; i++) {
      vit_depunctured[i] = in_bits[i];
    }
  }
  else {
    for (int i = 0; i < MAX_ENCODED_BITS; i++) {
      vit_depunctured[i] = d_depunctured[i];
    }
    count = 0;
    for (int i = 0; i < frame_ptr->n_sym; i++) {
      for (int k = 0; k < n_cbps; k++) {
        while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
          vit_depunctured[count] = 2;
          count++;
        }

        // Insert received bits
        vit_depunctured[count] = in_bits[i * n_cbps + k];
        count++;

        while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
          vit_depunctured[count] = 2;
          count++;
        }
      }
    }
  }
}

void
#ifdef HCC
inline __attribute__((always_inline))
#endif
vit_task_start_decode(uint8_t * vit_depunctured, uint8_t * vit_data) {
  int32_t inMem_size = 72; // fixed -- always (add the 2 padding bytes)
  int32_t inData_size = MAX_ENCODED_BITS; // Using the max value here for now/safety
  int32_t outData_size = (MAX_ENCODED_BITS * 3 / 4); //  Using the max value here for now/safety
  uint8_t * in_Mem = vit_data;
  uint8_t * in_Data = &(vit_data[inMem_size]);
  uint8_t * out_Data = &(vit_data[inMem_size + inData_size]);
  // Copy some multi-block stuff into a single memory (cleaner transport)
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
      in_Mem[imi++] = d_depuncture_pattern[ti];
    }
    if (imi != 70) { printf("ERROR : imi = %u and should be 70\n", imi); }
  } // scope block for defn of imi
  for (int ti = 0; ti < MAX_ENCODED_BITS; ti++) { // This is over-kill for messages that are not max size
    in_Data[ti] = vit_depunctured[ti];
    //DEBUG(if (ti < 32) { printf("HERE : in_Data %3u : %u\n", ti, in_Data[ti]); });
  }
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4);
    ti++) { // This zeros out the full-size OUTPUT area
    out_Data[ti] = 0;
    // vdsptr->theData[imi++] = 0;
  }
  // Call the do_decoding routine
}

void do_viterbi_function(int in_n_data_bits, int in_cbps, int in_ntraceback,
  unsigned char * inMemory, unsigned char * outMemory) {
  int in_count = 0;
  int out_count = 0;
  int n_decoded = 0;

  unsigned char * d_brtab27[2] = { &(inMemory[0]), &(inMemory[32]) };

  unsigned char * in_depuncture_pattern = &(inMemory[64]);
  uint8_t * depd_data = &(inMemory[72]);
  uint8_t * l_decoded = &(outMemory[0]);

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
        unsigned char * mm0 = l_metric0_generic;
        unsigned char * mm1 = l_metric1_generic;
        unsigned char * pp0 = l_path0_generic;
        unsigned char * pp1 = l_path1_generic;
        unsigned char * symbols = &depd_data[in_count & 0xfffffffc];

        // These are used to "virtually" rename the uses below (for symmetry; reduces code size)
        //  Really these are functionally "offset pointers" into the above arrays....
        unsigned char * metric0, * metric1;
        unsigned char * path0, * path1;

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
            }
            else if (symbols[second_symbol] == 2) {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = d_brtab27[0][(i * 16) + j] ^ sym0v[j];
                metsv[j] = 1 - metsvm[j];
              }
            }
            else {
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
          unsigned char * mm0 = l_metric0_generic;
          unsigned char * pp0 = l_path0_generic;
          int ntraceback = in_ntraceback;
          unsigned char * outbuf = &c;

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


void
#ifdef HCC
inline __attribute__((always_inline))
#endif
vit_leaf(size_t in_size, message_size_t msg_size, ofdm_param * ofdm_ptr, size_t ofdm_size,
  frame_param * frame_ptr, size_t frame_ptr_size, uint8_t * vit_data, size_t vit_data_size) {

  DEBUG(printf("-- Vitterbi Leaf Node --\n"));

  __hpvm__hint(DEVICE);

#ifndef HCC
  __hpvm__attributes(5, ofdm_ptr, frame_ptr, in_bits, message_id, out_msg_text,
    2, message_id, out_msg_text);
#endif
  __hpvm__task(VIT_TASK);
  // Call the do_decoding routine
  // START OF EXEC_VIT_ON_CPU
  //

  // DEBUG(printf("In exec_vit_task_on_cpu_accel\n"));
  int32_t in_cbps = ofdm_ptr->n_cbps;
  int32_t in_ntraceback = d_ntraceback;
  int32_t in_data_bits = frame_ptr->n_data_bits;
  uint8_t * in_Mem = &(vit_data[0]);
  uint8_t * in_Data = &(vit_data[72]);
  uint8_t * out_Data = &(vit_data[72 + MAX_ENCODED_BITS]);

  do_viterbi_function(in_data_bits, in_cbps, in_ntraceback, in_Mem, out_Data);

  /*
  SDEBUG(for (int i = 0; i < 20; i++) {
      printf("CPU_VIT_OUT %3u : %3u @ %p \n", i, out_Data[i], &(out_Data[i])); // cpuOutMem[i]);
    });
    */

    // void finish_viterbi_execution(task_metadata_entry* vit_metadata_block,
    // va_list var_list) // message_t* message_id, char* out_msg_text)

  // We've finished the execution and lifetime for this task; free its metadata
  // DEBUG(printf("  MB%u fin_vit Calling free_task_metadata_block\n",
  // vit_metadata_block->block_id));
  // free_task_metadata_block(vit_metadata_block);

#ifndef HCC
  __hpvm__return(2, message_id, out_msg_text);
#endif
}


void
vit_post(frame_param * frame_ptr, size_t frame_ptr_size, uint8_t * vit_data, size_t vit_data_size, message_t * message_id, size_t msg_id_size, char * out_msg_text, size_t out_msg_text_size) {

  // __hpvm__hint(CPU_TARGET);

  message_t msg = NUM_MESSAGES;

  int psdusize = frame_ptr->psdu_size;; // set by finish_decode call...
  uint8_t * out_Data = &(vit_data[72 + MAX_ENCODED_BITS]);

  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4);
    ti++) { // This covers the full-size OUTPUT area
    d_decoded[ti] = out_Data[ti];
  }

  // descramble the output - put it in result
  descrambler(d_decoded, psdusize, out_msg_text, NULL /*descram_ref*/, NULL /*msg*/);

  // Here we look at the message string and select proper message_t
  switch (out_msg_text[3]) {
  case '0':
    msg = safe_to_move_right_or_left;
    break;
  case '1':
    msg = safe_to_move_right_only;
    break;
  case '2':
    msg = safe_to_move_left_only;
    break;
  case '3':
    msg = unsafe_to_move_left_or_right;
    break;
  default:
    msg = NUM_MESSAGES;
    break;
  }
  *message_id = msg;
}

void
#ifdef HCC
inline __attribute__((always_inline))
#endif
cv_leaf(size_t in_size, label_t in_label, label_t * obj_label, size_t obj_label_size) {
  DEBUG(printf("-- CV Leaf Node --\n"));
  __hpvm__hint(DEVICE);

#ifndef HCC
  __hpvm__attributes(1, obj_label, 1, obj_label);
#endif
  __hpvm__task(CV_TASK);

  // CPU CV sleeps for a particular time
  usleep(cv_cpu_run_time_in_usec);

  *obj_label = in_label;

#ifndef HCC
  __hpvm__return(1, obj_label);
#endif
}

static float * bit_reverse(float * w, uint32_t N, uint32_t bits) {
  uint32_t i, s, shift;
  s = sizeof(i) * CHAR_BIT - 1;
  shift = s - bits + 1;

  for (i = 0; i < N; i++) {
    uint32_t r;
    float t_real, t_imag;
    r = _rev(i);
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

  return w;
}

void
__attribute__ ((noinline))
radar_leaf(size_t in_size, uint32_t log_nsamples, float * inputs_ptr) {


  DEBUG(printf("-- Radar Leaf Node --\n"));

#ifndef HCC

  __hpvm__attributes(2, distance_ptr, inputs_ptr, 2, distance_ptr, inputs_ptr);
#endif

  int sign = -1;

  // execute_cpu_fft_accelerator
  // data = data
  // N = 1 << fft_log_nsamples

  // uint32_t log_nsamples = log_nsamples; // fft_data_p->log_nsamples;
  float * data = inputs_ptr;                // (float*)(fft_data_p->theData);

  uint32_t N = 1 << log_nsamples;

  uint32_t transform_length;
  uint32_t a, b, i, j, bit;
  float theta, t_real, t_imag, w_real, w_imag, s, t, s2, z_real, z_imag;
  transform_length = 1;

  /* bit reversal */
  bit_reverse(data, N, log_nsamples);

  /* calculation */
  for (bit = 0; bit < log_nsamples; bit++) {
    w_real = 1.0;
    w_imag = 0.0;

    theta = 1.0 * sign * M_PI / (float) transform_length;

    s = sin(theta);
    t = sin(0.5 * theta);
    s2 = 2.0 * t * t;

    for (a = 0; a < transform_length; a++) {
      for (b = 0; b < N; b += 2 * transform_length) {
        i = b + a;
        j = b + a + transform_length;

        z_real = data[2 * j];
        z_imag = data[2 * j + 1];

        t_real = w_real * z_real - w_imag * z_imag;
        t_imag = w_real * z_imag + w_imag * z_real;

        /* write the result */
        data[2 * j] = data[2 * i] - t_real;
        data[2 * j + 1] = data[2 * i + 1] - t_imag;
        data[2 * i] += t_real;
        data[2 * i + 1] += t_imag;

        // printf(",%u,%u,%u,%u,%u,%f,%u,%f,%u,%f,%u,%f,%f,%f\n", a, b, i, j,
        // 2*j, data[2*j], 2*j+1, data[2*j+1], 2*i, data[2*i], 2*i+1,
        // data[2*i+1], t_real, t_imag);
      }

      /* adjust w */
      t_real = w_real - (s * w_imag + s2 * w_real);
      t_imag = w_imag + (s * w_real - s2 * w_imag);
      w_real = t_real;
      w_imag = t_imag;
    }

    transform_length *= 2;
  }
}


void
#ifdef HCC
inline __attribute__((always_inline))
#endif
radar_post(uint32_t log_nsamples, float * inputs_ptr, distance_t * distance_ptr) {
  //
  // uint32_t log_nsamples = radar_data_p->log_nsamples;
  float * data = inputs_ptr;

  unsigned RADAR_N = 0;    // The number of samples (2^LOGN)
  float RADAR_fs = 0.0;    // Sampling Frequency
  float RADAR_alpha = 0.0; // Chirp rate (saw-tooth)
  // float   RADAR_psd_threshold = 1e-10*pow(8192,2);  // ~= 0.006711 and 450 ~=
  // 0.163635 in 16K
  float RADAR_psd_threshold = 0.0067108864;

  switch (log_nsamples) {
  case 10:
    RADAR_fs = 204800.0;
    RADAR_alpha = 30000000000.0;
    RADAR_psd_threshold = 0.000316; // 1e-10*pow(8192,2);  // 450m ~= 0.000638
    // so psd_thres ~= 0.000316 ?
    break;
  case 14:
    RADAR_fs = 32768000.0;
    RADAR_alpha = 4800000000000.0;
    RADAR_psd_threshold = 1e-10 * pow(8192, 2);
    break;
  default:
    printf("ERROR : Unsupported Log-N FFT Samples Value: %u\n", log_nsamples);
    exit(-1);
  }
  RADAR_N = (1 << log_nsamples);

  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int _i;
  float temp;
  for (_i = 0; _i < RADAR_N; _i++) {
    temp = (pow(data[2 * _i], 2) + pow(data[2 * _i + 1], 2)) / 100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = _i;
    }
  }
  float distance = ((float) (max_index * ((float) RADAR_fs) / ((float) (RADAR_N)))) * 0.5 * RADAR_c / ((float) (RADAR_alpha));
  DEBUG(printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index));

  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));
  if (max_psd <= RADAR_psd_threshold) {
    distance = INFINITY;
  }

  DEBUG(printf("Assigning distance to pointer\n"));

  DEBUG(printf("Distance ptr %p\n", distance_ptr));
  *distance_ptr = distance;

  DEBUG(printf("Distance in object: %.3f\n", *distance_ptr));
  DEBUG(printf("Finished executing Radar_leaf\n"));

#ifndef HCC
  __hpvm__return(1, distance_ptr);
#endif
}

void
#ifdef HCC
inline __attribute__((always_inline))
#endif
pnc_leaf(size_t in_size, unsigned time_step, unsigned repeat_factor, label_t * obj_label, size_t obj_label_size,
  distance_t * distance_ptr, size_t distance_ptr_size,
  message_t * message_id, size_t msg_id_size, vehicle_state_t * current_vehicle_state, size_t current_vehicle_state_size,
  vehicle_state_t * new_vehicle_state, size_t new_vehicle_state_size, char * out_msg_text, size_t out_msg_text_size
) {

  DEBUG(printf("-- PNC Leaf Node --\n"));


  // __hpvm__hint(DEVICE);

#ifndef HCC
  __hpvm__attributes(6, current_vehicle_state, new_vehicle_state, distance_ptr,
    obj_label, message_id, out_msg_text, 1, new_vehicle_state);
#endif
  // __hpvm__task(PNC_TASK);


  vehicle_state_t * vehicle_state = current_vehicle_state;

  message_t safe_lanes_msg = *message_id;

  distance_t obj_distance = *distance_ptr;
  DEBUG(printf("In the plan_and_control task : label %u %s distance %.1f (T1 %.1f T2 %.1f T3 %.1f) message %u\n", *obj_label, object_names[*obj_label],
    obj_distance, PNC_THRESHOLD_1, PNC_THRESHOLD_2, PNC_THRESHOLD_3, safe_lanes_msg));
  DEBUG(printf("Plan-Ctrl: current Vehicle-State : Active %u Lane %u Speed %.1f\n", vehicle_state->active, vehicle_state->lane, vehicle_state->speed));

  // Start with outpu vehicle state is a copy of input vehicle state...
  // plan_ctrl_data_p->new_vehicle_state = plan_ctrl_data_p->vehicle_state;
  if (!vehicle_state->active) {
    // Our car is broken and burning, no plan-and-control possible -- nothing to
    // do
  }
  else if ( //(plan_ctrl_data_p->object_label != no_object) && // For safety,
 // assume every return is from SOMETHING we should not hit!
    ((obj_distance <= PNC_THRESHOLD_1)
#ifdef USE_SIM_ENVIRON
      || ((vehicle_state->speed < car_goal_speed) &&
        (obj_distance <= PNC_THRESHOLD_2))
#endif
      )) {
    // This covers all cases where we have an obstacle "too close" ahead of us
    if (obj_distance <= IMPACT_DISTANCE) {
      // We've crashed into an obstacle...
      printf("WHOOPS: We've suffered a collision on time_step %u!\n", time_step);
      // fprintf(stderr, "WHOOPS: We've suffered a collision on time_step %u!\n", plan_ctrl_data_p->time_step);
      new_vehicle_state->speed = 0.0;
      new_vehicle_state->active = false;
    }
    else {
      // Some object ahead of us that needs to be avoided.
      DEBUG(printf("  In lane %s with object %u at %.1f\n", lane_names[vehicle_state->lane], *obj_label, obj_distance));
      switch (safe_lanes_msg) {
      case safe_to_move_right_or_left:
        /* Bias is move right, UNLESS we are in the Right lane and would then
         * head into the RHazard Lane */
        if (vehicle_state->lane < right) {
          DEBUG(printf("   In %s with Safe_L_or_R : Moving Right\n", lane_names[vehicle_state->lane]));
          new_vehicle_state->lane = (lane_t) ((int) new_vehicle_state->lane + 1);
        }
        else {
          DEBUG(printf("   In %s with Safe_L_or_R : Moving Left\n", lane_names[vehicle_state->lane]));
          new_vehicle_state->lane = (lane_t) ((int) new_vehicle_state->lane - 1);
        }
        break; // prefer right lane
      case safe_to_move_right_only:
        printf("   In %s with Safe_R_only : Moving Right\n", lane_names[vehicle_state->lane]);
        new_vehicle_state->lane = (lane_t) ((int) new_vehicle_state->lane + 1);
        break;
      case safe_to_move_left_only:
        printf("   In %s with Safe_L_Only : Moving Left\n", lane_names[vehicle_state->lane]);
        new_vehicle_state->lane = (lane_t) ((int) new_vehicle_state->lane - 1);
        break;
      case unsafe_to_move_left_or_right:
#ifdef USE_SIM_ENVIRON
        if (vehicle_state->speed > car_decel_rate) {
          new_vehicle_state->speed = ->vehicle_state->speed - car_decel_rate; // was / 2.0;
          DEBUG(printf("   In %s with No_Safe_Move -- SLOWING DOWN from %.2f to %.2f\n", lane_names[vehicle_state->lane], vehicle_state.speed, new_vehicle_state->speed));
        }
        else {
          DEBUG(printf("   In %s with No_Safe_Move -- Going < 15.0 so STOPPING!\n", lane_names[vehicle_state->lane]));
          new_vehicle_state->speed = 0.0;
        }
#else
        DEBUG(printf("   In %s with No_Safe_Move : STOPPING\n", lane_names[vehicle_state->lane]));
        new_vehicle_state->speed = 0.0;
#endif
        break; /* Stop!!! */
      default:
        printf(" ERROR  In %s with UNDEFINED MESSAGE: %u\n", lane_names[vehicle_state->lane], safe_lanes_msg);
        exit(-90);
      }
    } // end of "we have some obstacle too close ahead of us"
  }
  else {
    // No obstacle-inspired lane change, so try now to occupy the center lane
    switch (vehicle_state->lane) {
    case lhazard:
    case left:
      if ((safe_lanes_msg == safe_to_move_right_or_left) ||
        (safe_lanes_msg == safe_to_move_right_only)) {
        DEBUG(printf("  In %s with Can_move_Right: Moving Right\n", lane_names[vehicle_state->lane]));
        new_vehicle_state->lane = (lane_t) ((int) new_vehicle_state->lane + 1);
      }
      break;
    case center:
      // No need to alter, already in the center
      DEBUG(printf("No need to alter, already in the center\n"));
      break;
    case right:
    case rhazard:
      if ((safe_lanes_msg == safe_to_move_right_or_left) ||
        (safe_lanes_msg == safe_to_move_left_only)) {
        DEBUG(printf("  In %s with Can_move_Left : Moving Left\n", lane_names[vehicle_state->lane]));
        new_vehicle_state->lane = (lane_t) ((int) new_vehicle_state->lane - 1);
      }
      break;
    default:
      printf("Unknown vehicle_state->lane value of %u\n", vehicle_state->lane);
      exit(-91);
    }
#ifdef USE_SIM_ENVIRON
    if ((vehicle_state->speed <
      car_goal_speed) && // We are going slower than we want to, and
     //((plan_ctrl_data_p->object_label == no_object) ||
     //// There is no object ahead of us -- don't need;
     // NOTHING is at
     // INF_PLAN_CTRL_DATA_P->OBJECT_DISTANCE
      (obj_distance >= PNC_THRESHOLD_2)) { // Any object is far enough away
      if (vehicle_state->speed <= (car_goal_speed - car_accel_rate)) {
        new_vehicle_state->speed += 15.0;
      }
      else {
        new_vehicle_state->speed = car_goal_speed;
      }
      DEBUG(printf("  Going %.2f : slower than target speed %.2f : Speeding up to %.2f\n", vehicle_state->speed, 50.0, new_vehicle_state->speed));
    }
#endif
  } // end of plan-and-control logic functions...

  DEBUG(printf("Plan-Ctrl:     new Vehicle-State : Active %u Lane %u Speed %.1f\n", new_vehicle_state->active, new_vehicle_state->lane,
    new_vehicle_state->speed));

#ifndef HCC
  __hpvm__return(1, new_vehicle_state);
#endif
}

void MiniERARoot(
  size_t vit_size, message_size_t msg_size, uint8_t * vit_data, size_t vit_data_size, ofdm_param * ofdm_ptr, size_t ofdm_size, frame_param * frame_ptr, size_t frame_ptr_size, uint8_t * in_bits, size_t in_bit_size, uint8_t * vit_depunctured, size_t vit_depunctured_size, message_t * message_id, size_t msg_id_size, char * out_msg_text, size_t out_msg_text_size, size_t fft_size, uint32_t log_nsamples, float * inputs_ptr, size_t inputs_ptr_size, distance_t * distance_ptr, size_t distance_ptr_size, size_t cv_size, label_t in_label, label_t * obj_label, size_t obj_label_size, size_t pnc_size, unsigned time_step, unsigned repeat_factor, vehicle_state_t * current_vehicle_state, size_t current_vehicle_state_size, vehicle_state_t * new_vehicle_state, size_t new_vehicle_state_size
) {
#ifdef HCC
  void * Section = __hetero_section_begin();
  {
    void * VIT_reset_depuncture = __hetero_task_begin(4,  vit_depunctured, vit_depunctured_size, ofdm_ptr, ofdm_size, frame_ptr, frame_ptr_size, in_bits, in_bit_size, 1, vit_depunctured, vit_depunctured_size,
      /* Optional Node Name */ "VIT_reset_depuncture");
    __hpvm__hint(CPU_TARGET);
    __hpvm__task(VIT_RESET_DEPUNCTURE);
    // printf("In VIT_RESET_DEPUNCTURE\n");

    vit_task_reset(ofdm_ptr);
    vit_task_depuncture(ofdm_ptr, frame_ptr, in_bits, vit_depunctured);

    __hetero_task_end(VIT_reset_depuncture);

    void * VIT_setup = __hetero_task_begin(2, vit_data, vit_data_size , vit_depunctured, vit_depunctured_size, 1, vit_data, vit_data_size,
      /* Optional Node Name */ "VIT_setup");
    __hpvm__hint(CPU_TARGET);
    __hpvm__task(VIT_SETUP);

    // printf("In VIT_SETUP\n");
    vit_task_start_decode(vit_depunctured, vit_data);

    __hetero_task_end(VIT_setup);

    void * VIT = __hetero_task_begin(5, vit_size, msg_size, ofdm_ptr, ofdm_size,
      frame_ptr, frame_ptr_size, vit_data, vit_data_size,
      1, vit_data, vit_data_size,
      /* Optional Node Name */ "VIT");

    // Body will be inlined into task
    // printf("In VIT_LEAF\n");
    vit_leaf(vit_size, msg_size, ofdm_ptr, ofdm_size,
      frame_ptr, frame_ptr_size, vit_data, vit_data_size);

    __hetero_task_end(VIT);

    void * VIT_post = __hetero_task_begin(4, frame_ptr, frame_ptr_size, vit_data, vit_data_size, message_id, msg_id_size, out_msg_text, out_msg_text_size , 2, message_id, msg_id_size, out_msg_text, out_msg_text_size,
      /* Optional Node Name */ "VIT_post");
    __hpvm__hint(CPU_TARGET);
    __hpvm__task(VIT_POST);

    // printf("In VIT_POST\n");
    vit_post(frame_ptr, frame_ptr_size, vit_data, vit_data_size, message_id, msg_id_size, out_msg_text, out_msg_text_size);

    __hetero_task_end(VIT_post);

    void * RADAR = __hetero_task_begin(3, fft_size, log_nsamples, inputs_ptr, inputs_ptr_size, 1, inputs_ptr, inputs_ptr_size,
      /* Optional Node Name */ "RADAR");
    // printf("In RADAR_LEAF\n");
    // Body will be inlined into task
    __hpvm__task(RADAR_TASK);
    __hpvm__hint(DEVICE);
    radar_leaf(fft_size, log_nsamples, inputs_ptr);

    __hetero_task_end(RADAR);



    void * RADAR_POST = __hetero_task_begin(3 , log_nsamples, inputs_ptr, inputs_ptr_size , distance_ptr, distance_ptr_size, 1, distance_ptr, distance_ptr_size,
      /* Optional Node Name */ "RADAR_POST");
    // printf("In RADAR_POST\n");
    // Body will be inlined into task
    radar_post(log_nsamples, inputs_ptr, distance_ptr);

    __hetero_task_end(RADAR_POST);


    void * CV = __hetero_task_begin(3, cv_size, in_label, obj_label, obj_label_size,
      1, obj_label, obj_label_size,
      /* Optional Node Name */ "CV");

    // Body will be inlined into task
    cv_leaf(cv_size, in_label, obj_label, obj_label_size);

    __hetero_task_end(CV);


    void * PNC = __hetero_task_begin(9, pnc_size, time_step, repeat_factor, obj_label, obj_label_size, distance_ptr, distance_ptr_size, message_id, msg_id_size, current_vehicle_state, current_vehicle_state_size, out_msg_text, out_msg_text_size, new_vehicle_state, new_vehicle_state_size, 1, new_vehicle_state, new_vehicle_state_size,
      /* Optional Node Name */ "PNC");

    // Body will be inlined into task
    pnc_leaf(pnc_size, time_step, repeat_factor, obj_label, obj_label_size,
      distance_ptr, distance_ptr_size, message_id, msg_id_size, current_vehicle_state, current_vehicle_state_size, new_vehicle_state, new_vehicle_state_size,
      out_msg_text, out_msg_text_size);

    __hetero_task_end(PNC);


  } 
  __hetero_section_end(Section);


#else


  // Describe DAG using HPVM-C
  __hpvm__hint(CPU_TARGET);
  __hpvm__attributes(10, ofdm_ptr, frame_ptr, in_bits, message_id, out_msg_text, obj_label,
    distance_ptr, inputs_ptr, new_vehicle_state, current_vehicle_state,
    1, new_vehicle_state);

  void * ViterbiNode = __hpvm__createNodeND(0, vit_leaf, /* Node Criticality */ HPVM_CRITICAL);

  __hpvm__bindIn(ViterbiNode, 0, 0, 0);
  __hpvm__bindIn(ViterbiNode, 1, 1, 0);
  __hpvm__bindIn(ViterbiNode, 2, 2, 0);
  __hpvm__bindIn(ViterbiNode, 3, 3, 0);
  __hpvm__bindIn(ViterbiNode, 4, 4, 0);
  __hpvm__bindIn(ViterbiNode, 5, 5, 0);
  __hpvm__bindIn(ViterbiNode, 6, 6, 0);
  __hpvm__bindIn(ViterbiNode, 7, 7, 0);
  __hpvm__bindIn(ViterbiNode, 8, 8, 0);
  __hpvm__bindIn(ViterbiNode, 9, 9, 0);
  __hpvm__bindIn(ViterbiNode, 10, 10, 0);

  void * CVNode = __hpvm__createNodeND(0, cv_leaf, /* Node Criticality */ HPVM_CRITICAL);

  __hpvm__bindIn(CVNode, 11, 0, 0);
  __hpvm__bindIn(CVNode, 12, 1, 0);
  __hpvm__bindIn(CVNode, 13, 2, 0);

  void * RadarNode = __hpvm__createNodeND(0, radar_leaf, /* Node Criticality */ HPVM_CRITICAL);

  __hpvm__bindIn(RadarNode, 14, 0, 0);
  __hpvm__bindIn(RadarNode, 15, 1, 0);
  __hpvm__bindIn(RadarNode, 16, 2, 0);
  __hpvm__bindIn(RadarNode, 17, 3, 0);
  __hpvm__bindIn(RadarNode, 18, 4, 0);

  void * PNCNode = __hpvm__createNodeND(0, pnc_leaf, /* Node Criticality */ HPVM_CRITICAL);

  __hpvm__bindIn(PNCNode, 19, 0, 0);
  __hpvm__bindIn(PNCNode, 20, 1, 0);

  // CV Outputs
  __hpvm__edge(CVNode, PNCNode, /* replType */ 1, 0, 2, /* isStream */ 0);
  __hpvm__bindIn(PNCNode, 13, 3, 0);


  // Radar Outputs
  __hpvm__edge(RadarNode, PNCNode, /* replType */ 1, 0, 4, /* isStream */ 0);
  __hpvm__bindIn(PNCNode, 18, 5, 0);


  // Viterbi Outputs
  __hpvm__edge(ViterbiNode, PNCNode, /* replType */ 1, 0, 6, /* isStream */ 0);
  __hpvm__bindIn(PNCNode, 8, 7, 0);


  __hpvm__edge(ViterbiNode, PNCNode, /* replType */ 1, 1, 12, /* isStream */ 0);
  __hpvm__bindIn(PNCNode, 10, 13, 0);


  __hpvm__bindIn(PNCNode, 21, 8, 0);
  __hpvm__bindIn(PNCNode, 22, 9, 0);
  __hpvm__bindIn(PNCNode, 23, 10, 0);
  __hpvm__bindIn(PNCNode, 24, 11, 0);


  __hpvm__bindOut(PNCNode, 0, 0, /* isStream */ 0);

#endif
}


void hpvm_launch(RootIn * _DFGArgs, message_size_t vit_msgs_size, vit_dict_entry_t * vdentry_p, uint8_t * vit_depunctured, message_t * message, char * out_msg_text, uint8_t * vit_data, label_t * cv_tr_label, unsigned log_nsamples, float * radar_inputs, distance_t * distance, unsigned time_step, unsigned pandc_repeat_factor, vehicle_state_t * vehicle_state, vehicle_state_t * new_vehicle_state) {
  DEBUG(printf("In hpvm_launch()\n"));


#ifndef HCC


  // Use HPVM-C 


  RootIn * DFGArgs = (RootIn *) malloc(sizeof(RootIn));

  /* -- HPVM Host Code -- */

  DFGArgs->in_label = *cv_tr_label;
  DFGArgs->obj_label = cv_tr_label;
  DFGArgs->obj_label_size = sizeof(label_t);

  DFGArgs->time_step = time_step;
  DFGArgs->log_nsamples = log_nsamples;
  DFGArgs->inputs_ptr = radar_inputs;// copy in radar_inputs explicitly.
  // described in base_types.h
  DFGArgs->inputs_ptr_size = 2 * (1 << MAX_RADAR_LOGN) * sizeof(float);



  DFGArgs->distance_ptr = distance;
  DFGArgs->distance_ptr_size = sizeof(distance_t);

  DFGArgs->msg_size = vit_msgs_size;
  DFGArgs->ofdm_ptr = &vdentry_p->ofdm_p;
  DFGArgs->ofdm_size = sizeof(ofdm_param);
  DFGArgs->frame_ptr = &vdentry_p->frame_p;
  DFGArgs->frame_ptr_size = sizeof(frame_param);
  DFGArgs->in_bits = vdentry_p->in_bits;
  DFGArgs->in_bit_size = sizeof(uint8_t);
  DFGArgs->message_id = message;
  DFGArgs->msg_id_size = sizeof(message_t);
  DFGArgs->out_msg_text = out_msg_text;
  DFGArgs->out_msg_text_size = 1600;

  DFGArgs->current_vehicle_state = vehicle_state;
  DFGArgs->current_vehicle_state_size = sizeof(vehicle_state);
  DFGArgs->new_vehicle_state = new_vehicle_state;
  DFGArgs->new_vehicle_state_size = sizeof(new_vehicle_state);
  DFGArgs->repeat_factor = pandc_repeat_factor;

  // Add relavent memory to memory tracker
  llvm_hpvm_track_mem(distance, sizeof(distance_t));
  llvm_hpvm_track_mem(DFGArgs->inputs_ptr, 2 * (1 << MAX_RADAR_LOGN));
  llvm_hpvm_track_mem(cv_tr_label, sizeof(label_t));
  llvm_hpvm_track_mem(&vdentry_p->ofdm_p, sizeof(ofdm_param));
  llvm_hpvm_track_mem(&vdentry_p->frame_p, sizeof(frame_param));
  llvm_hpvm_track_mem(vdentry_p->in_bits, sizeof(uint8_t));
  llvm_hpvm_track_mem(message, sizeof(message_t));
  llvm_hpvm_track_mem(out_msg_text, 1600);
  llvm_hpvm_track_mem(vehicle_state, sizeof(new_vehicle_state));
  llvm_hpvm_track_mem(new_vehicle_state, sizeof(new_vehicle_state));

  DEBUG(printf("\n\nLaunching ERA pipeline!\n"));

  void * ERADFG = __hpvm__launch(0, MiniERARoot, (void *) DFGArgs);
  __hpvm__wait(ERADFG);

  DEBUG(printf("\n\nFinished executing ERA pipeline!\n"));
  DEBUG(printf("\n\nRequesting Memory!\n"));

  // Requesting memory back from DFG
  llvm_hpvm_request_mem(new_vehicle_state, sizeof(vehicle_state_t));

  // Remove relavent memory from memory tracker

  llvm_hpvm_untrack_mem(cv_tr_label);
  llvm_hpvm_untrack_mem(DFGArgs->inputs_ptr);
  llvm_hpvm_untrack_mem(distance);
  llvm_hpvm_untrack_mem(&vdentry_p->ofdm_p);
  llvm_hpvm_untrack_mem(&vdentry_p->frame_p);
  llvm_hpvm_untrack_mem(vdentry_p->in_bits);
  llvm_hpvm_untrack_mem(message);
  llvm_hpvm_untrack_mem(out_msg_text);
  llvm_hpvm_untrack_mem(new_vehicle_state);
  llvm_hpvm_untrack_mem(vehicle_state);

#else

  size_t radar_inputs_size = 2 * (1 << MAX_RADAR_LOGN) * sizeof(float);

  // Use Hetero-C++ 
  printf("log_nsamples:%d %p, radar_inputs: %lf %p\n", log_nsamples, &log_nsamples, radar_inputs[0], radar_inputs);
  void * DFG = __hetero_launch((void *) MiniERARoot,
    /* Num Input Pairs */ 21,
    (size_t) vit_msgs_size,
    vit_msgs_size,
    &(vit_data[0]), sizeof(vit_data),
    &vdentry_p->ofdm_p, sizeof(ofdm_param),
    &vdentry_p->frame_p, sizeof(frame_param),
    vdentry_p->in_bits, sizeof(vdentry_p->in_bits),
    vit_depunctured, sizeof(vit_depunctured),
    message, sizeof(message_t),
    out_msg_text, 1600,
    (size_t) log_nsamples,
    log_nsamples,
    radar_inputs, radar_inputs_size,
    distance, sizeof(distance_t),
    (size_t) sizeof(label_t),
    *cv_tr_label,
    cv_tr_label, sizeof(label_t),
    (size_t) sizeof(distance_t) + sizeof(message_t) + sizeof(label_t),
    time_step,
    pandc_repeat_factor,
    vehicle_state, sizeof(vehicle_state_t),
    new_vehicle_state, sizeof(vehicle_state_t),
    /* Num Output Pairs */ 1,
    new_vehicle_state, sizeof(vehicle_state_t)
  );
  __hetero_wait(DFG);

#endif


  DEBUG(printf("[ HPVM ] New vehicle state: lane %u speed %.1f\n\n", new_vehicle_state->lane, new_vehicle_state->speed));

#ifndef HCC
  free(DFGArgs);
#endif
}

extern "C" {
  RootIn * hpvm_initialize() {
    __hpvm__init();
    RootIn * DFGArgs = (RootIn *) malloc(sizeof(DFGArgs));
    return DFGArgs;
  }
}

void hpvm_cleanup(RootIn * DFGArgs) {
  free(DFGArgs);
  __hpvm__cleanup();
}



#ifdef HPVM_BASE_CRIT

void vit_leaf_base(message_size_t msg_size, ofdm_param * ofdm_ptr, size_t ofdm_size,
  frame_param * frame_ptr, size_t frame_ptr_size, uint8_t * in_bits,
  size_t in_bit_size, message_t * message_id, size_t msg_id_size,
  char * out_msg_text, size_t out_msg_text_size) {

  DEBUG(printf("-- Vitterbi Leaf Node --\n"));

  __hpvm__hint(DEVICE);
  __hpvm__attributes(5, ofdm_ptr, frame_ptr, in_bits, message_id, out_msg_text,
    2, message_id, out_msg_text);
  __hpvm__task(VIT_TASK);

  reset(ofdm_ptr, ofdm_size);

  uint8_t * depunctured = depuncture(in_bits, ofdm_ptr, frame_ptr);
  /*
  DO_VERBOSE({
          printf("VBS: depunctured = [\n");
          for (int ti = 0; ti < MAX_ENCODED_BITS; ti ++) {
          if (ti > 0) { printf(", "); }
          if ((ti > 0) && ((ti % 8) == 0)) { printf("  "); }
          if ((ti > 0) && ((ti % 40) == 0)) { printf("\n"); }
          printf("%02x", depunctured[ti]);
          }
          printf("\n");
          });*/

  int32_t n_data_bits = frame_ptr->n_data_bits;
  int32_t n_cbps = ofdm_ptr->n_cbps;
  int32_t n_traceback = d_ntraceback;
  int32_t psdu_size = frame_ptr->psdu_size;
  int32_t inMem_size = 72; // fixed -- always (add the 2 padding bytes)
  int32_t inData_size = MAX_ENCODED_BITS; // Using the max value here for now/safety
  int32_t outData_size = (MAX_ENCODED_BITS * 3 / 4); //  Using the max value here for now/safety
  uint8_t in_Mem[inMem_size];     // &(theData[0]);
  uint8_t in_Data[inData_size];   //= &(theData[inMem_size]);
  uint8_t out_Data[outData_size]; //= &(theData[inMem_size + inData_size]);

  // Copy some multi-block stuff into a single memory (cleaner transport)
  //
  {
    // scope block for definition of imi
    int imi = 0;
    for (int ti = 0; ti < 2; ti++) {
      for (int tj = 0; tj < 32; tj++) {
        in_Mem[imi++] = d_branchtab27_generic[ti].c[tj];
      }
    }
    if (imi != 64) {
      printf("ERROR : imi = %u and should be 64\n", imi);
    }
    // imi = 64;
    for (int ti = 0; ti < 6; ti++) {
      in_Mem[imi++] = d_depuncture_pattern[ti];
    }
    if (imi != 70) {
      printf("ERROR : imi = %u and should be 70\n", imi);
    }
  } // scope block for defn of imi

  for (int ti = 0; ti < MAX_ENCODED_BITS;
    ti++) { // This is over-kill for messages that are not max size
    in_Data[ti] = depunctured[ti];
    // DEBUG(if (ti < 32) { printf("HERE : in_Data %3u : %u\n", ti,
    // in_Data[ti]); });
  }
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4); ti++) { // This zeros out the full-size OUTPUT area
    out_Data[ti] = 0;
    // vdsptr->theData[imi++] = 0;
  }
  // Call the do_decoding routine
  // START OF EXEC_VIT_ON_CPU
  //
  // DEBUG(printf("In exec_vit_task_on_cpu_accel\n"));
  int32_t in_cbps = n_cbps;
  int32_t in_ntraceback = n_traceback;
  int32_t in_data_bits = n_data_bits;

  // do_cpu_viterbi_function(in_data_bits, in_cbps, in_ntraceback, in_Mem,
  // out_Data); // cpuInMem, cpuOutMem);

  // void do_cpu_viterbi_function(int in_n_data_bits, int in_cbps, int
  // in_ntraceback, unsigned char *inMemory, unsigned char *outMemory)

  int in_count = 0;
  int out_count = 0;
  int n_decoded = 0;

  unsigned char * d_brtab27[2] = { &(in_Mem[0]), &(in_Mem[32]) };

  unsigned char * in_depuncture_pattern = &(in_Mem[64]);
  uint8_t * depd_data = &(in_Data[0]);
  uint8_t * l_decoded = &(out_Data[0]);

  /*
  DO_VERBOSE({
      printf("\nVBS: in_cbps        = %u\n", in_cbps);
      printf("VBS: in_ntraceback  = %u\n", in_ntraceback);
      printf("VBS: in_data_bits = %u\n", in_data_bits);
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
    });*/

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
  while (n_decoded < in_data_bits) {
    if ((in_count % 4) == 0) { // 0 or 3
      /* The basic Viterbi decoder operation, called a "butterfly"
       * operation because of the way it looks on a trellis diagram. Each
       * butterfly involves an Add-Compare-Select (ACS) operation on the two
       *nodes where the 0 and 1 paths from the current node merge at the next
       *step of the trellis.
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
        unsigned char * mm0 = l_metric0_generic;
        unsigned char * mm1 = l_metric1_generic;
        unsigned char * pp0 = l_path0_generic;
        unsigned char * pp1 = l_path1_generic;
        unsigned char * symbols = &depd_data[in_count & 0xfffffffc];

        // These are used to "virtually" rename the uses below (for symmetry;
        // reduces code size)
        //  Really these are functionally "offset pointers" into the above
        //  arrays....
        unsigned char * metric0, * metric1;
        unsigned char * path0, * path1;

        // Operate on 4 symbols (2 bits) at a time

        unsigned char m0[16], m1[16], m2[16], m3[16], decision0[16],
          decision1[16], survivor0[16], survivor1[16];
        unsigned char metsv[16], metsvm[16];
        unsigned char shift0[16], shift1[16];
        unsigned char tmp0[16], tmp1[16];
        unsigned char sym0v[16], sym1v[16];
        unsigned short simd_epi16;
        unsigned int first_symbol;
        unsigned int second_symbol;

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
          // This is the basic viterbi butterfly for 2 symbols (we need
          // therefore 2 passes for 4 total symbols)
          for (int i = 0; i < 2; i++) {
            if (symbols[first_symbol] == 2) {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = d_brtab27[1][(i * 16) + j] ^ sym1v[j];
                metsv[j] = 1 - metsvm[j];
              }
            }
            else if (symbols[second_symbol] == 2) {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = d_brtab27[0][(i * 16) + j] ^ sym0v[j];
                metsv[j] = 1 - metsvm[j];
              }
            }
            else {
              for (int j = 0; j < 16; j++) {
                metsvm[j] = (d_brtab27[0][(i * 16) + j] ^ sym0v[j]) +
                  (d_brtab27[1][(i * 16) + j] ^ sym1v[j]);
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
              tmp0[j] =
                (decision0[j] & shift0[j]) | ((~decision0[j]) & shift1[j]);
            }

            for (int j = 0, k = 8; j < 16; j += 2, k++) {
              metric1[((2 * i + 1) * 16) + j] = survivor0[k];
              metric1[((2 * i + 1) * 16) + (j + 1)] = survivor1[k];
            }
            for (int j = 0; j < 16; j++) {
              tmp1[j] =
                (decision1[j] & shift0[j]) | ((~decision1[j]) & shift1[j]);
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
      }                          // END of call to viterbi_butterfly2_generic
      viterbi_butterfly_calls++; // Do not increment until after the comparison
      // code.

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
        //    ntraceback  : INPUT         : int (I think effectively const for
        //    given run type; here 5 I think) outbuf      : OUTPUT        : 1
        //    byte l_store_pos : GLOBAL IN/OUT : int (position in circular
        //    traceback buffer?)

        //    l_mmresult  : GLOBAL OUTPUT : Array [ 64 bytes ]
        //    l_ppresult  : GLOBAL OUTPUT : Array [ntraceback][ 64 bytes ]

        // CALL : viterbi_get_output_generic(l_metric0_generic, l_path0_generic,
        // in_ntraceback, &c); unsigned char viterbi_get_output_generic(unsigned
        // char *mm0, unsigned char *pp0, int ntraceback, unsigned char *outbuf)
        {
          unsigned char * mm0 = l_metric0_generic;
          unsigned char * pp0 = l_path0_generic;
          int ntraceback = in_ntraceback;
          unsigned char * outbuf = &c;

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

          // return bestmetric;
        }

        // std::cout << "OUTPUT: " << (unsigned int)c << std::endl;
        if (out_count >= in_ntraceback) {
          for (int i = 0; i < 8; i++) {
            l_decoded[(out_count - in_ntraceback) * 8 + i] =
              (c >> (7 - i)) & 0x1;
            // SDEBUG(printf("l_decoded[ %u ] oc %u tb %u i %u written as %u\n",
            // (out_count - in_ntraceback) * 8 + i, out_count, in_ntraceback, i,
            // l_decoded[(out_count - in_ntraceback) * 8 + i]));
            n_decoded++;
          }
        }
        out_count++;
      }
    }
    in_count++;
  }

  /*
  SDEBUG(for (int i = 0; i < 20; i++) {
      printf("CPU_VIT_OUT %3u : %3u @ %p \n", i, out_Data[i], &(out_Data[i])); // cpuOutMem[i]);
    });
    */

    // void finish_viterbi_execution(task_metadata_entry* vit_metadata_block,
    // va_list var_list) // message_t* message_id, char* out_msg_text)

  message_t msg = NUM_MESSAGES;
  uint8_t * result;

  int psdusize = psdu_size; // set by finish_decode call...
  // DEBUG(printf("  MB%u Calling the finish_decode routine\n",
  // vit_metadata_block->block_id));
  // result = finish_decode(vit_metadata_block, &psdusize);
  //
  for (int ti = 0; ti < (MAX_ENCODED_BITS * 3 / 4);
    ti++) { // This covers the full-size OUTPUT area
    d_decoded[ti] = out_Data[ti];
    // DEBUG(if (ti < 31) { printf("FIN_VIT_OUT %3u : %3u @ %p \n", ti,
    // out_Data[ti], &(out_Data[ti]));});
    // SDEBUG(if (ti < 80) { printf("%u", out_Data[ti]); });
  }
  /*
  SDEBUG(printf("\n\n");
         for (int i = 0; i < 32; i++) {
      printf("VIT_OUT %3u : %3u \n", i, d_decoded[i]);
    });*/

    // descramble the output - put it in result
  descrambler(d_decoded, psdusize, out_msg_text, NULL /*descram_ref*/, NULL /*msg*/);

  // Here we look at the message string and select proper message_t
  switch (out_msg_text[3]) {
  case '0':
    msg = safe_to_move_right_or_left;
    break;
  case '1':
    msg = safe_to_move_right_only;
    break;
  case '2':
    msg = safe_to_move_left_only;
    break;
  case '3':
    msg = unsafe_to_move_left_or_right;
    break;
  default:
    msg = NUM_MESSAGES;
    break;
  }
  // DEBUG(printf("MB%u The finish_viterbi_execution found message %u\n",
  // vit_metadata_block->block_id, msg));
  *message_id = msg;
  // We've finished the execution and lifetime for this task; free its metadata
  // DEBUG(printf("  MB%u fin_vit Calling free_task_metadata_block\n",
  // vit_metadata_block->block_id));
  // free_task_metadata_block(vit_metadata_block);

  __hpvm__return(2, message_id, out_msg_text);
}

void cv_leaf_base(label_t in_label, label_t * obj_label, size_t obj_label_size) {
  DEBUG(printf("-- CV Leaf Node Base --\n"));
  __hpvm__hint(DEVICE);
  __hpvm__attributes(1, obj_label, 1, obj_label);
  __hpvm__task(CV_TASK);

  // CPU CV sleeps for a particular time
  usleep(cv_cpu_run_time_in_usec);

  *obj_label = in_label;

  __hpvm__return(1, obj_label);
}

void radar_leaf_base(uint32_t log_nsamples, float * inputs_ptr, size_t inputs_ptr_size,
  distance_t * distance_ptr, size_t distance_ptr_size
) {


  DEBUG(printf("-- Radar Leaf Node --\n"));
  __hpvm__hint(DEVICE);
  __hpvm__attributes(2, distance_ptr, inputs_ptr, 2, distance_ptr, inputs_ptr);
  __hpvm__task(RADAR_TASK);

  size_t data_size = 2 * (1 << log_nsamples) * sizeof(float);

  float * radar_inputs = (float *) malloc(inputs_ptr_size);

  for (int copy = 0; copy < 2 * (1 << MAX_RADAR_LOGN); copy++) {
    radar_inputs[copy] = inputs_ptr[copy];
  }
  // radar_data_p->theData == mdataptr
  /*
  for (int test = 0; test < 2 * (1 << log_nsamples); test++) {
    printf("%.3f ",inputs_ptr[test]);
   }
  printf("\n");
  */

  int sign = -1;

  // execute_cpu_fft_accelerator
  // data = data
  // N = 1 << log_nsamples

  int32_t fft_log_nsamples = log_nsamples; // fft_data_p->log_nsamples;
  float * data = radar_inputs;                // (float*)(fft_data_p->theData);

  unsigned int N = 1 << fft_log_nsamples;

  unsigned int transform_length;
  unsigned int a, b, i, j, bit;
  float theta, t_real, t_imag, w_real, w_imag, s, t, s2, z_real, z_imag;
  transform_length = 1;

  /* bit reversal */
  bit_reverse(data, N, fft_log_nsamples);

  /* calculation */
  for (bit = 0; bit < fft_log_nsamples; bit++) {
    w_real = 1.0;
    w_imag = 0.0;

    theta = 1.0 * sign * M_PI / (float) transform_length;

    s = sin(theta);
    t = sin(0.5 * theta);
    s2 = 2.0 * t * t;

    for (a = 0; a < transform_length; a++) {
      for (b = 0; b < N; b += 2 * transform_length) {
        i = b + a;
        j = b + a + transform_length;

        z_real = data[2 * j];
        z_imag = data[2 * j + 1];

        t_real = w_real * z_real - w_imag * z_imag;
        t_imag = w_real * z_imag + w_imag * z_real;

        /* write the result */
        data[2 * j] = data[2 * i] - t_real;
        data[2 * j + 1] = data[2 * i + 1] - t_imag;
        data[2 * i] += t_real;
        data[2 * i + 1] += t_imag;

        // printf(",%u,%u,%u,%u,%u,%f,%u,%f,%u,%f,%u,%f,%f,%f\n", a, b, i, j,
        // 2*j, data[2*j], 2*j+1, data[2*j+1], 2*i, data[2*i], 2*i+1,
        // data[2*i+1], t_real, t_imag);
      }

      /* adjust w */
      t_real = w_real - (s * w_imag + s2 * w_real);
      t_imag = w_imag + (s * w_real - s2 * w_imag);
      w_real = t_real;
      w_imag = t_imag;
    }

    transform_length *= 2;
  }

  // distance_t
  // do_finish_radar_computations(task_metadata_entry *radar_metadata_block)
  //
  // uint32_t fft_log_nsamples = radar_data_p->log_nsamples;
  // float *data = (float *)radar_data_p->theData;

  unsigned RADAR_N = 0;    // The number of samples (2^LOGN)
  float RADAR_fs = 0.0;    // Sampling Frequency
  float RADAR_alpha = 0.0; // Chirp rate (saw-tooth)
  // float   RADAR_psd_threshold = 1e-10*pow(8192,2);  // ~= 0.006711 and 450 ~=
  // 0.163635 in 16K
  float RADAR_psd_threshold = 0.0067108864;

  switch (fft_log_nsamples) {
  case 10:
    RADAR_fs = 204800.0;
    RADAR_alpha = 30000000000.0;
    RADAR_psd_threshold = 0.000316; // 1e-10*pow(8192,2);  // 450m ~= 0.000638
    // so psd_thres ~= 0.000316 ?
    break;
  case 14:
    RADAR_fs = 32768000.0;
    RADAR_alpha = 4800000000000.0;
    RADAR_psd_threshold = 1e-10 * pow(8192, 2);
    break;
  default:
    printf("ERROR : Unsupported Log-N FFT Samples Value: %u\n", fft_log_nsamples);
    exit(-1);
  }
  RADAR_N = (1 << fft_log_nsamples);

  float max_psd = 0;
  unsigned int max_index = 0;
  unsigned int _i;
  float temp;
  for (_i = 0; _i < RADAR_N; _i++) {
    temp = (pow(data[2 * _i], 2) + pow(data[2 * _i + 1], 2)) / 100.0;
    if (temp > max_psd) {
      max_psd = temp;
      max_index = _i;
    }
  }
  float distance = ((float) (max_index * ((float) RADAR_fs) / ((float) (RADAR_N)))) * 0.5 * RADAR_c / ((float) (RADAR_alpha));
  DEBUG(printf("Max distance is %.3f\nMax PSD is %4E\nMax index is %d\n", distance, max_psd, max_index));

  //printf("max_psd = %f  vs %f\n", max_psd, 1e-10*pow(8192,2));
  if (max_psd <= RADAR_psd_threshold) {
    distance = INFINITY;
  }

  DEBUG(printf("Assigning distance to pointer\n"));

  DEBUG(printf("Distance ptr %p\n", distance_ptr));
  *distance_ptr = distance;

  DEBUG(printf("Distance in object: %.3f\n", *distance_ptr));
  DEBUG(printf("Finished executing Radar_leaf\n"));

  __hpvm__return(1, distance_ptr);
}
void VITRoot(
  message_size_t msg_size, ofdm_param * ofdm_ptr, size_t ofdm_size,
  frame_param * frame_ptr, size_t frame_ptr_size, uint8_t * in_bits,
  size_t in_bit_size, message_t * message_id, size_t msg_id_size, char * out_msg_text,
  size_t out_msg_text_size
) {

  __hpvm__hint(CPU_TARGET);
  __hpvm__attributes(5, ofdm_ptr, frame_ptr, in_bits, message_id,
    out_msg_text, 2, message_id, out_msg_text);

  void * ViterbiNode = __hpvm__createNodeND(0, vit_leaf_base, /* Node Criticality */ HPVM_BASE);

  __hpvm__bindIn(ViterbiNode, 0, 0, 0);
  __hpvm__bindIn(ViterbiNode, 1, 1, 0);
  __hpvm__bindIn(ViterbiNode, 2, 2, 0);
  __hpvm__bindIn(ViterbiNode, 3, 3, 0);
  __hpvm__bindIn(ViterbiNode, 4, 4, 0);
  __hpvm__bindIn(ViterbiNode, 5, 5, 0);
  __hpvm__bindIn(ViterbiNode, 6, 6, 0);
  __hpvm__bindIn(ViterbiNode, 7, 7, 0);
  __hpvm__bindIn(ViterbiNode, 8, 8, 0);
  __hpvm__bindIn(ViterbiNode, 9, 9, 0);
  __hpvm__bindIn(ViterbiNode, 10, 10, 0);

  // __hpvm__bindOut(ViterbiNode, 0, 0, /* isStream */ 0);
  // __hpvm__bindOut(ViterbiNode, 1, 1, /* isStream */ 0);

}




void hpvm_launch_base_VIT(int vit_base_msg_size, vit_dict_entry_t * base_vdentry) {
  VitRootIn * VitArgs = (VitRootIn *) malloc(sizeof(VitRootIn));

  message_t out_message;
  char out_msg_text[1600];

  VitArgs->msg_size = (message_size_t) vit_base_msg_size;
  VitArgs->ofdm_ptr = &base_vdentry->ofdm_p;
  VitArgs->ofdm_size = sizeof(ofdm_param);
  VitArgs->frame_ptr = &base_vdentry->frame_p;
  VitArgs->frame_ptr_size = sizeof(frame_param);
  VitArgs->in_bits = base_vdentry->in_bits;
  VitArgs->in_bit_size = sizeof(uint8_t);
  VitArgs->message_id = &out_message;
  VitArgs->msg_id_size = sizeof(message_t);
  VitArgs->out_msg_text = out_msg_text;
  VitArgs->out_msg_text_size = 1600;

  llvm_hpvm_track_mem(&base_vdentry->ofdm_p, sizeof(ofdm_param));
  llvm_hpvm_track_mem(&base_vdentry->frame_p, sizeof(frame_param));
  llvm_hpvm_track_mem(&base_vdentry->in_bits, sizeof(uint8_t));
  llvm_hpvm_track_mem(&out_message, sizeof(message_t));
  llvm_hpvm_track_mem(out_msg_text, 1600);

  void * BASE_VIT_DFG = __hpvm__launch(0, VITRoot, (void *) VitArgs);

  free(VitArgs);
}

void RadarRoot(
  uint32_t log_nsamples, float * inputs_ptr, size_t inputs_ptr_size,
  distance_t * distance_ptr, size_t distance_ptr_size) {
  __hpvm__hint(CPU_TARGET);
  __hpvm__attributes(2, inputs_ptr, distance_ptr, 1, distance_ptr);

  void * RadarNode = __hpvm__createNodeND(0, radar_leaf_base, /* Node Criticality */ HPVM_BASE);

  __hpvm__bindIn(RadarNode, 0, 0, 0);
  __hpvm__bindIn(RadarNode, 1, 1, 0);
  __hpvm__bindIn(RadarNode, 2, 2, 0);
  __hpvm__bindIn(RadarNode, 3, 3, 0);
  __hpvm__bindIn(RadarNode, 4, 4, 0);

  // __hpvm__bindOut(RadarNode, 0, 0, 0);


}

void hpvm_launch_base_RADAR(unsigned base_log_nsamples, float * base_radar_inputs) {
  RadarRootIn * RadarArgs = (RadarRootIn *) malloc(sizeof(RadarRootIn));

  distance_t out_distance;

  RadarArgs->log_nsamples = base_log_nsamples;
  RadarArgs->inputs_ptr = base_radar_inputs;
  RadarArgs->inputs_ptr_size = 2 * (1 << MAX_RADAR_LOGN) * sizeof(float);
  RadarArgs->distance_ptr = &out_distance;
  RadarArgs->distance_ptr_size = sizeof(distance_t);

  llvm_hpvm_track_mem(&out_distance, sizeof(distance_t));
  llvm_hpvm_track_mem(base_radar_inputs, RadarArgs->inputs_ptr_size);

  void * RadarDFG = __hpvm__launch(0, RadarRoot, (void *) RadarArgs);


  free(RadarArgs);

}

void CVRoot(
  label_t in_label, label_t * obj_label, size_t obj_label_size) {
  __hpvm__hint(CPU_TARGET);
  __hpvm__attributes(1, obj_label, 1, obj_label);

  void * CVNode = __hpvm__createNodeND(0, cv_leaf_base, /* Node Criticality */ HPVM_BASE);

  __hpvm__bindIn(CVNode, 0, 0, 0);
  __hpvm__bindIn(CVNode, 1, 1, 0);
  __hpvm__bindIn(CVNode, 2, 2, 0);

  // __hpvm__bindOut(CVNode, 0, 0, 0);
}


void hpvm_launch_base_CV(label_t cv_tr_label) {
  CVRootIn * CVArgs = (CVRootIn *) malloc(sizeof(CVRootIn));

  label_t out_label;

  CVArgs->in_label = cv_tr_label;
  CVArgs->obj_label = &out_label;
  CVArgs->obj_label_size = sizeof(label_t);

  llvm_hpvm_track_mem(&out_label, sizeof(label_t));

  void * CVDFG = __hpvm__launch(0, CVRoot, (void *) CVArgs);


  free(CVArgs);
}



#endif

#endif
