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

#ifndef H_DAG_LIB_INCLUDE_H
#define H_DAG_LIB_INCLUDE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <iostream>
#include <string>

#include "base_task_types.h"

#include "cv_task.h"
#include "radar_task.h"
#include "vit_task.h"
#include "plan_ctrl_task.h"
#include "test_task.h"
#include "scheduler.h"

#include "verbose.h"

//Functions to create static DAGs
Graph * create_graph(char * graphml_filename);
Graph * create_graph_miniERA(bool no_crit_cnn_task, unsigned num_Crit_test_tasks,
                             task_type_t radar_task_type, task_type_t vit_task_type, task_type_t plan_ctrl_task_type,
                             task_type_t cv_task_type, task_type_t test_task_type,
                             //RADAR IOuts
                             uint32_t log_nsamples, float *inputs, size_t inputs_size,
                             //VIT IOuts
                             message_size_t msize, ofdm_param *ofdm_ptr, size_t ofdm_param_size, frame_param *frame_ptr,
                             size_t frame_ptr_size, uint8_t *in_bits, size_t in_bits_size,
                             //CV IOuts
                             label_t cv_tr_label,
                             //PnC IOuts
                             unsigned time_step, unsigned repeat_factor, label_t* object_label_ptr, size_t object_label_size,
                             distance_t* xfer_object_dist_ptr, size_t dist_size, message_t* safe_lanes_msg_ptr,
                             size_t safe_lanes_msg_size, vehicle_state_t* vehicle_state_ptr);
void create_graph_cv(Graph * graph_ptr, task_type_t cv_task_type, label_t cv_tr_label,
                     label_t* object_label_ptr);
void create_graph_vit(Graph * graph_ptr, task_type_t vit_task_type, message_size_t msize,
                      ofdm_param *ofdm_ptr, size_t ofdm_param_size, frame_param *frame_ptr, size_t frame_ptr_size,
                      uint8_t *in_bits, size_t in_bits_size, message_t* safe_lanes_msg_ptr);
void create_graph_rad(Graph * graph_ptr, task_type_t radar_task_type, uint32_t log_nsamples,
                      float *inputs, size_t inputs_size, distance_t* xfer_object_dist_ptr);
void create_graph_test(Graph * graph_ptr, task_type_t test_task_type);
void create_graph_pnc(Graph * graph_ptr, task_type_t plan_ctrl_task_type, unsigned time_step,
                      unsigned repeat_factor, label_t* object_label_ptr, size_t object_label_size,
                      distance_t* xfer_object_dist_ptr, size_t dist_size, message_t* safe_lanes_msg_ptr,
                      size_t safe_lanes_msg_size, vehicle_state_t* vehicle_state_ptr);

#endif
