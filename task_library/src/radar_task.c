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
#include <math.h>
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

 //#define VERBOSE
#include "radar_task.h"
#include "scheduler.h"
#include "verbose.h"

// CONSTANTS
#define RADAR_c 300000000.0 // Speed of Light in Meters/Sec
#define RADAR_threshold -100;

std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> radar_profile;

// This now illustrates the use of the "task metadata" to transfer information
// for a radar (FFT) operation.
//  NOTE: We request a metadata block form the scheduler -- if one is not
//  currently available, then what?
//     OPTIONS: 1. Wait for one to become available
//              2. Purge a lower-criticality task (if there is one), else wait
//              3. Drop task if this is lowest-criticality (?), else wait
//   To make the task independent of the calling program, we need to copy over
//   the data into the metadata block
//      This treats the scheduler like an off-load accelerator, in many ways.
//   Then we should try to make thes CPU accelerators run in independent threads
//   (pthreads, I think)?
//      This will let us refine the non-blocking behavior, and start the more
//      detailed behavior of the
//        scheduler implementation (i.e. ranking, queue management, etc.)

// We declare this for all possible legal RADAR log_nsamples inputs, but RADAR tasks can only be 1k or 16k samples

void set_up_radar_task_on_accel_profile_data() {
  for (int si = 0; si <= 14; si++) {
    for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
      radar_profile[si][ai] = ACINFPROF;
    }
  }
#ifdef COMPILE_TO_ESP
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  radar_profile[10][SCHED_CPU_ACCEL_T] = 23000;
  radar_profile[10][SCHED_EPOCHS_1D_FFT_ACCEL_T] = 6000;
  radar_profile[14][SCHED_CPU_ACCEL_T] = 600000;
  radar_profile[14][SCHED_EPOCHS_1D_FFT_ACCEL_T] = 143000;
#else
  // This is for out x86 all-software runs...
  radar_profile[10][SCHED_CPU_ACCEL_T] = 50;
  radar_profile[14][SCHED_CPU_ACCEL_T] = 1250;
  std::cout << "Radar profile[10]: " << radar_profile[10] << std::endl;
#endif
  DEBUG(printf("\n%18s : %18s %18s %18s %18s\n", "RADAR-PROFILES", "CPU", "FFT-HWR", "VIT-HWR",
    "CV-HWR");
  printf("%15s :", "radar_profile[10]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", radar_profile[10][ai]); } printf("\n");
  printf("%15s :", "radar_profile[14]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) { printf(" 0x%016lx", radar_profile[14][ai]); } printf("\n");
  printf("\n"));
}
