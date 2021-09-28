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

#ifndef _kernels_api_h
#define _kernels_api_h

#include "verbose.h"

#define TIME

#include "base_types.h"

#include "scheduler.h"

/* These are some top-level defines for the dictionaries */
#ifndef HPVM
#include "fft_accel.h"
#include "calc_fmcw_dist.h"
#endif

#define VITERBI_MSG_LENGTHS     4
#define VITERBI_MSGS_PER_STEP   3

extern unsigned crit_fft_samples_set; // The sample set used for Critical Task FFT
extern bool  output_viz_trace;

extern vehicle_state_t other_car; // This is the reported state of the other car (from their transmission)

extern char* lane_names[NUM_LANES];
extern char* message_names[NUM_MESSAGES];
extern char* object_names[NUM_OBJECTS];
extern char  object_char[NUM_OBJECTS];

extern unsigned vit_msgs_size;
extern unsigned vit_msgs_per_step;
extern const char* vit_msgs_size_str[VITERBI_MSG_LENGTHS];
extern const char* vit_msgs_per_step_str[VITERBI_MSGS_PER_STEP];

extern unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
extern unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')
extern unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
extern char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

extern char  nearest_obj[NUM_LANES];
extern float nearest_dist[NUM_LANES];

extern unsigned hist_total_objs[NUM_LANES * MAX_OBJ_IN_LANE];

extern unsigned rand_seed;

#define MAX_RDICT_SAMPLE_SETS   4   // This should be updated eventually...
extern unsigned int num_radar_samples_sets;
extern unsigned int radar_log_nsamples_per_dict_set[MAX_RDICT_SAMPLE_SETS];


typedef enum {
  TEST_TASK_INIT = 0,
  TEST_TASK_START,
  TEST_TASK_DONE
} test_res_t;

/*
typedef struct {
  unsigned int id;
} test_dict_entry_t;
*/

/* Input Trace Functions */
#ifndef USE_SIM_ENVIRON
 #include "read_trace.h"
#endif

void output_VizTrace_line(unsigned min_obst_lane, unsigned max_obst_lane, vehicle_state_t* vehicle_state, vehicle_state_t* other_car);

/* Kernels initialization */
status_t init_cv_kernel(char* py_file, char* dict_fn);
label_t run_object_classification(unsigned tr_val);
label_t iterate_cv_kernel(vehicle_state_t vs);
void start_execution_of_cv_kernel(task_metadata_block_t* mb_ptr, label_t in_tr_val);
label_t finish_execution_of_cv_kernel(task_metadata_block_t* mb_ptr);
void    post_execute_cv_kernel(label_t tr_val, label_t d_object);

status_t init_radar_kernel(char* dict_fn, int set_to_use);
radar_dict_entry_t* iterate_radar_kernel(vehicle_state_t vs);
radar_dict_entry_t* select_random_radar_input();
radar_dict_entry_t* select_critical_radar_input(radar_dict_entry_t* rdentry_p);
void       post_execute_radar_kernel(unsigned set, unsigned index, distance_t tr_dist, distance_t dist);

message_t get_safe_dir_message_from_fused_occ_map(vehicle_state_t vs);
status_t init_vit_kernel(char* dict_fn);
vit_dict_entry_t* iterate_vit_kernel(vehicle_state_t vs, message_t* tr_message );
vit_dict_entry_t* select_specific_vit_input(int l_num, int m_num);
vit_dict_entry_t* select_random_vit_input();
void start_execution_of_vit_kernel(task_metadata_block_t* mb_ptr, vit_dict_entry_t* trace_msg);
message_t finish_execution_of_vit_kernel(task_metadata_block_t* mb_ptr);
void      post_execute_vit_kernel(message_t tr_msg, message_t dec_msg);

status_t   init_test_kernel(char* dict_fn);
test_dict_entry_t* iterate_test_kernel(vehicle_state_t vs);
void       start_execution_of_test_kernel(task_metadata_block_t* mb_ptr, test_dict_entry_t* tde);
test_res_t finish_execution_of_test_kernel(task_metadata_block_t* mb_ptr);
void       post_execute_test_kernel(test_res_t gold_res, test_res_t exec_res);



void            start_execution_of_plan_ctrl_kernel(task_metadata_block_t* mb_ptr);
vehicle_state_t finish_execution_of_plan_ctrl_kernel(task_metadata_block_t* mb_ptr);
//vehicle_state_t plan_and_control(label_t, distance_t, message_t, vehicle_state_t);

// These routines are used for any finalk, end-of-run operations/output
void closeout_cv_kernel(void);
void closeout_radar_kernel(void);
void closeout_vit_kernel(void);
void closeout_test_kernel(void);

#endif
