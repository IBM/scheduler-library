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

#include "dag_lib.h"

//inputs
//CV: cv_input_t
//Radar: radar_input_t
//Viterbi: viterbi_input_t
//Test: NA
//Plan and contrl: pnc_input_t

//outputs:
//radar: &distance
//viterbi: &message
//cv: &label
//test: NULL
//pnc: &vehicle_state

//Functions to create static DAGs
void create_graph_miniERA(Graph * graph_ptr, bool no_crit_cnn_task, unsigned num_Crit_test_tasks,
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
                          size_t safe_lanes_msg_size, vehicle_state_t* vehicle_state_ptr) {


    //Create graph with cv, vit, rad and test; pnc has dependency on all the above
    Graph &graph = *graph_ptr;
    uint8_t id = 0;

    //Add vertices
    vertex_t radar_vertex = boost::add_vertex(graph);
    graph[radar_vertex].vertex_id = id++;
    vertex_t viterbi_vertex = boost::add_vertex(graph);
    graph[viterbi_vertex].vertex_id = id++;
    vertex_t pnc_vertex = boost::add_vertex(graph);
    graph[pnc_vertex].vertex_id = id++;


    //Add edges to pnc from sensor tasks
    boost::add_edge(radar_vertex, pnc_vertex, graph);
    boost::add_edge(viterbi_vertex, pnc_vertex, graph);

    //Set vertex parameters
    graph[radar_vertex].task_type = radar_task_type;
    graph[viterbi_vertex].task_type = vit_task_type;
    graph[pnc_vertex].task_type = plan_ctrl_task_type;

    graph[radar_vertex].input_ptr = new radar_input_t(log_nsamples, inputs, inputs_size);
    graph[radar_vertex].output_ptr = xfer_object_dist_ptr;
    graph[radar_vertex].vertex_status = TASK_FREE;

    graph[viterbi_vertex].input_ptr = new viterbi_input_t(msize, ofdm_ptr, ofdm_param_size, frame_ptr,
            frame_ptr_size, in_bits, in_bits_size);
    graph[viterbi_vertex].output_ptr = safe_lanes_msg_ptr;
    graph[viterbi_vertex].vertex_status = TASK_FREE;

    if (!no_crit_cnn_task) {
        vertex_t cv_vertex = boost::add_vertex(graph);
        graph[cv_vertex].vertex_id = id++;
        boost::add_edge(cv_vertex, pnc_vertex, graph);
        graph[cv_vertex].task_type = cv_task_type;
        graph[cv_vertex].input_ptr = new cv_input_t(cv_tr_label);
        graph[cv_vertex].output_ptr = object_label_ptr;
        graph[cv_vertex].vertex_status = TASK_FREE;
    }

    if (num_Crit_test_tasks > 0) {
        vertex_t test_vertex = boost::add_vertex(graph);
        graph[test_vertex].vertex_id = id++;
        boost::add_edge(test_vertex, pnc_vertex, graph);
        graph[test_vertex].task_type = test_task_type;
        graph[test_vertex].input_ptr = NULL; // No input
        graph[test_vertex].output_ptr = NULL;
        graph[test_vertex].vertex_status = TASK_FREE;
    }

    graph[pnc_vertex].input_ptr = new pnc_input_t(time_step, repeat_factor, object_label_ptr,
            object_label_size, xfer_object_dist_ptr, dist_size, safe_lanes_msg_ptr, safe_lanes_msg_size,
            vehicle_state_ptr);
    graph[pnc_vertex].output_ptr = vehicle_state_ptr;
    graph[pnc_vertex].vertex_status = TASK_FREE;
}

void create_graph_cv(Graph * graph_ptr, task_type_t cv_task_type, label_t cv_tr_label,
                     label_t* object_label_ptr) {
    Graph &graph = *graph_ptr;
    uint8_t id = 0;
    vertex_t cv_vertex = boost::add_vertex(graph);
    graph[cv_vertex].vertex_id = id++;
    graph[cv_vertex].task_type = cv_task_type;
    graph[cv_vertex].input_ptr = new cv_input_t(cv_tr_label);
    graph[cv_vertex].output_ptr = object_label_ptr;
    graph[cv_vertex].vertex_status = TASK_FREE;

}

void create_graph_vit(Graph * graph_ptr, task_type_t vit_task_type, message_size_t msize,
                      ofdm_param *ofdm_ptr, size_t ofdm_param_size, frame_param *frame_ptr, size_t frame_ptr_size,
                      uint8_t *in_bits, size_t in_bits_size, message_t* safe_lanes_msg_ptr) {
    Graph &graph = *graph_ptr;
    uint8_t id = 0;
    vertex_t viterbi_vertex = boost::add_vertex(graph);
    graph[viterbi_vertex].vertex_id = id++;
    graph[viterbi_vertex].task_type = vit_task_type;
    graph[viterbi_vertex].input_ptr = new viterbi_input_t(msize, ofdm_ptr, ofdm_param_size, frame_ptr,
            frame_ptr_size, in_bits, in_bits_size);
    graph[viterbi_vertex].output_ptr = safe_lanes_msg_ptr;
    graph[viterbi_vertex].vertex_status = TASK_FREE;
}

void create_graph_rad(Graph * graph_ptr, task_type_t radar_task_type, uint32_t log_nsamples,
                      float *inputs, size_t inputs_size, distance_t* xfer_object_dist_ptr) {
    Graph &graph = *graph_ptr;
    uint8_t id = 0;
    vertex_t radar_vertex = boost::add_vertex(graph);
    graph[radar_vertex].vertex_id = id++;
    graph[radar_vertex].task_type = radar_task_type;
    graph[radar_vertex].input_ptr = new radar_input_t(log_nsamples, inputs, inputs_size);
    graph[radar_vertex].output_ptr = xfer_object_dist_ptr;
    graph[radar_vertex].vertex_status = TASK_FREE;
}

void create_graph_test(Graph * graph_ptr, task_type_t test_task_type) {
    Graph &graph = *graph_ptr;
    uint8_t id = 0;
    vertex_t test_vertex = boost::add_vertex(graph);
    graph[test_vertex].vertex_id = id++;
    graph[test_vertex].task_type = test_task_type;
    graph[test_vertex].input_ptr = NULL; // No input
    graph[test_vertex].output_ptr = NULL;
    graph[test_vertex].vertex_status = TASK_FREE;
}

void create_graph_pnc(Graph * graph_ptr, task_type_t plan_ctrl_task_type, unsigned time_step,
                      unsigned repeat_factor, label_t* object_label_ptr, size_t object_label_size,
                      distance_t* xfer_object_dist_ptr, size_t dist_size, message_t* safe_lanes_msg_ptr,
                      size_t safe_lanes_msg_size, vehicle_state_t* vehicle_state_ptr) {
    Graph &graph = *graph_ptr;
    uint8_t id = 0;
    vertex_t pnc_vertex = boost::add_vertex(graph);
    graph[pnc_vertex].vertex_id = id++;
    graph[pnc_vertex].task_type = plan_ctrl_task_type;
    graph[pnc_vertex].input_ptr = new pnc_input_t(time_step, repeat_factor, object_label_ptr,
            object_label_size, xfer_object_dist_ptr, dist_size,
            safe_lanes_msg_ptr, safe_lanes_msg_size, vehicle_state_ptr);
    graph[pnc_vertex].output_ptr = vehicle_state_ptr;
    graph[pnc_vertex].vertex_status = TASK_FREE;
}



