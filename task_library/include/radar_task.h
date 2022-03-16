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

//Radar: radar_log_nsamples_per_dict_set[crit_fft_samples_set], radar_inputs, 2 * (1 << MAX_RADAR_LOGN) * sizeof(float)
struct radar_input_t {
        uint32_t in_size; //For profiling
        uint32_t log_nsamples;
        float *inputs;
        size_t inputs_size;
        radar_input_t(uint32_t log_nsamples, float *inputs, size_t inputs_size):
                in_size(log_nsamples), log_nsamples(log_nsamples), inputs(inputs), inputs_size(inputs_size) {}
};

void set_up_radar_task_on_accel_profile_data();

/*task_metadata_entry*/ void* set_up_radar_task(/*scheduler_datastate*/ void* sptr,
                task_type_t radar_task_type, task_criticality_t crit_level,
                bool use_auto_finish, int32_t dag_id, int32_t task_id, void* args);

void radar_auto_finish_routine(/*task_metadata_entry*/ void* mb);
void finish_radar_execution(/*task_metadata_entry*/ void* radar_metadata_block, void* args);
extern std::map<uint64_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> radar_profile;
#endif
