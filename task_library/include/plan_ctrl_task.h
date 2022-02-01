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

#ifndef H_PLAN_CTRL_TASK_INCLUDE_H
#define H_PLAN_CTRL_TASK_INCLUDE_H

#include <pthread.h>
#include <stdint.h>
#include <sys/time.h>

#include "base_task_types.h"
#include "scheduler.h"

//Plan and contrl: time_step, pandc_repeat_factor, &label, sizeof(label_t),  &distance, sizeof(distance_t), &message, sizeof(message_t) , &vehicle_state
struct pnc_input_t {
  unsigned time_step;
  unsigned repeat_factor;
  label_t* object_label_ptr;
  size_t object_label_size;
  distance_t* xfer_object_dist_ptr;
  size_t dist_size;
  message_t* safe_lanes_msg_ptr;
  size_t safe_lanes_msg_size;
  vehicle_state_t* vehicle_state_ptr;
  pnc_input_t(  unsigned time_step, unsigned repeat_factor, label_t* object_label_ptr,
                size_t object_label_size, distance_t* xfer_object_dist_ptr,
                size_t dist_size, message_t* safe_lanes_msg_ptr, size_t safe_lanes_msg_size,
                vehicle_state_t* vehicle_state_ptr) :
    time_step(time_step), repeat_factor(repeat_factor), object_label_ptr(object_label_ptr),
    object_label_size(object_label_size),
    xfer_object_dist_ptr(xfer_object_dist_ptr), dist_size(dist_size),
    safe_lanes_msg_ptr(safe_lanes_msg_ptr), safe_lanes_msg_size(safe_lanes_msg_size),
    vehicle_state_ptr(vehicle_state_ptr) {}
};

// This is a structure that defines the "PLAN_CTRL" task's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "Plan-and-Control" Task view of "data"
  unsigned        time_step;         // The current time-step of the simulation
  unsigned        repeat_factor;     // The number of repeated computations to do
  label_t         object_label;      // The determined label of the object in the image
  distance_t      object_distance;   // The distance to the closest vehicle in our lane
  message_t       safe_lanes_msg;    // The message indicating which lanes are safe to change into
  vehicle_state_t vehicle_state;     // The current (input) vehicle state
  vehicle_state_t new_vehicle_state; // The new (oputput) vehicle state
} plan_ctrl_data_struct_t;

typedef struct {
  struct timeval call_start;
  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
} plan_ctrl_timing_data_t;

void print_plan_ctrl_metadata_block_contents(void *mb);

void output_plan_ctrl_task_type_run_stats(void *sptr, unsigned my_task_type,
    unsigned total_accel_types);

void execute_on_cpu_plan_ctrl_accelerator(void *task_metadata_block);
void execute_on_hwr_vit_plan_ctrl_accelerator(void *task_metadata_block);
void execute_on_hwr_fft_plan_ctrl_accelerator(void *task_metadata_block);
void execute_on_hwr_cv_plan_ctrl_accelerator(void *task_metadata_block);

void set_up_plan_ctrl_task_on_accel_profile_data();

void *set_up_plan_ctrl_task(void *sptr, task_type_t plan_ctrl_task_type,
                            task_criticality_t crit_level, bool use_auto_finish,
                            int32_t dag_id, int32_t task_id, void *args);

void plan_ctrl_auto_finish_routine(void *mb);
void finish_plan_ctrl_execution(void *plan_ctrl_metadata_block,
                                void *var_list); // vehicle_state_t* new_vehicle_state);

#endif
