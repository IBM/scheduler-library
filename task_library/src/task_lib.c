/*
 * Copyright 2021 IBM
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
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include <map>

#include "task_lib.h"

#include "vit_task.h"
#include "fft_task.h"
#include "radar_task.h"
#include "cv_task.h"
#include "test_task.h"
#include "create_grid_task.h"

std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> cpu_profile;

extern "C" {
  void initialize_task_lib() {
    set_up_vit_task_on_accel_profile_data();
    set_up_fft_task_on_accel_profile_data();
    set_up_radar_task_on_accel_profile_data();
    set_up_cv_task_on_accel_profile_data();
    set_up_test_task_on_accel_profile_data();
    set_up_create_grid_task_on_accel_profile_data();

    for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
      cpu_profile[0][ai] = ACINFPROF;
    }
    cpu_profile[0][SCHED_CPU_ACCEL_T] = 1; //Set a low number
    //TODO: Need to update as the task completes
  }

}
