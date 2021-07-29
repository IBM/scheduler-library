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

#ifndef H_RADAR_TASK_INCLUDE_H
#define H_RADAR_TASK_INCLUDE_H

// This code "extends" the FFT_TASK to become a RADAR_TASK
//  This means there is a particular interpreteation applied to the FFT result
//  In practice, it uses a LOT of the base FFT data...

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdarg.h>

#include "base_task_types.h"

#include "fft_task.h"

void set_up_radar_task_on_accel_profile_data();

/*task_metadata_block_t*/ void* set_up_radar_task(/*scheduler_datastate_block_t*/ void* sptr,
						  task_type_t radar_task_type, task_criticality_t crit_level,
						  bool use_auto_finish, int32_t dag_id, /*va_list var_list*/ void* args);

void radar_auto_finish_routine(/*task_metadata_block_t*/ void* mb);
void finish_radar_execution(/*task_metadata_block_t*/ void* radar_metadata_block, void* args);

#endif
