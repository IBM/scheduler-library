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

#ifndef _RECV_PIPE_H
#define _RECV_PIPE_H

#include "sdr_base.h"
#include "gr_equalizer.h"  // for INT_TIME extern declarations
#include "sync_short.h"    // for INT_TIME extern declarations
#include "sync_long.h"     // for INT_TIME extern declarations
#include "ofdm.h"          // for INT_TIME extern declarations

#ifdef INT_TIME
/* This is RECV PIPE internal Timing information (gathering resources) */
extern uint64_t r_pipe_sec;
extern uint64_t r_pipe_usec;

extern uint64_t r_cmpcnj_sec;
extern uint64_t r_cmpcnj_usec;

extern uint64_t r_cmpmpy_sec;
extern uint64_t r_cmpmpy_usec;

extern uint64_t r_firc_sec;
extern uint64_t r_firc_usec;

extern uint64_t r_cmpmag_sec;
extern uint64_t r_cmpmag_usec;

extern uint64_t r_cmpmag2_sec;
extern uint64_t r_cmpmag2_usec;

extern uint64_t r_fir_sec;
extern uint64_t r_fir_usec;

extern uint64_t r_div_sec;
extern uint64_t r_div_usec;

extern uint64_t r_sshort_sec;
extern uint64_t r_sshort_usec;

extern uint64_t r_slong_sec;
extern uint64_t r_slong_usec;

extern uint64_t r_fft_sec;
extern uint64_t r_fft_usec;

extern uint64_t r_eqlz_sec;
extern uint64_t r_eqlz_usec;

extern uint64_t r_decsignl_sec;
extern uint64_t r_decsignl_usec;

extern uint64_t r_descrmbl_sec;
extern uint64_t r_descrmbl_usec;

extern uint64_t r_zz_sec;
extern uint64_t r_zz_usec;

#ifdef RECV_HW_FFT

extern uint64_t r_fHtotal_sec;
extern uint64_t r_fHtotal_usec;

extern uint64_t r_fHcvtin_sec;
extern uint64_t r_fHcvtin_usec;

extern uint64_t r_fHcomp_sec;
extern uint64_t r_fHcomp_usec;

extern uint64_t r_fHcvtout_sec;
extern uint64_t r_fHcvtout_usec;
#endif
#endif

void recv_pipe_init();

void do_recv_pipeline(int num_recvd_vals, float* recvd_in_real, float* recvd_in_imag, int* vit_msg_len, ofdm_param* ofdm_parms, frame_param* frame_parms, uint8_t* vit_msg);


#endif
