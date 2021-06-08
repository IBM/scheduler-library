/*
 * Copyright 2020 IBM
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

#ifndef H_BASE_TYPES_H
#define H_BASE_TYPES_H
#include <stdbool.h>
#include "base_task_types.h"

/* Types definitions */
// This is the type for distance measurements
typedef float distance_t;

#define MY_APP_ACCEL_TYPES  4

#if(0) // These are defined in the TASKS now
/* Pre-defined labels used by the computer vision kernel */
typedef enum {
  myself = -1,
  no_label = 0,
  car,
  truck,
  pedestrian,
  bicycle
} label_t;


/* The potential (horizontal) positions of any object (i.e. lane indications) */
typedef enum {
  lhazard = 0, 
  left, 
  center, 
  right,
  rhazard,
} lane_t;

/* These are some global type defines, etc. */
typedef struct
{
  bool active;
  lane_t lane;
  float speed;
} vehicle_state_t;



/* Pre-defined messages used by the Viterbi decoding kernel */
/*  These now conform to version 0.4 of the specification   */
typedef enum {
  safe_to_move_right_or_left   = 0,
  safe_to_move_right_only      = 1,
  safe_to_move_left_only       = 2,
  unsafe_to_move_left_or_right = 3,
  num_message_t
} message_t;

/* These are GLOBAL and affect the underlying world, etc. */
#define NUM_LANES     5
#define NUM_OBJECTS   5
#define NUM_MESSAGES  4
#endif

#define MAX_OBJ_IN_LANE  16

#define MAX_DISTANCE     500.0  // Max resolution distance of radar is < 500.0m
#define DIST_STEP_SIZE    50.0
#define INF_DISTANCE     (MAX_DISTANCE + DIST_STEP_SIZE)
#define RADAR_BUCKET_DISTANCE  DIST_STEP_SIZE  // The radar is in steps of 50

/* These thresholds (in meters) are used by the plan_and_control()
 * function to make plan and control decisions.
 */
#define THRESHOLD_1 155.0
#define THRESHOLD_2 205.0
#define THRESHOLD_3 305.0

#define VIT_CLEAR_THRESHOLD  THRESHOLD_1


typedef struct {
  unsigned int index;          // A global index (of all radar dictionary entries
  unsigned int set;            // The set this entry is in
  unsigned int index_in_set;   // The index in the set for this entry
  unsigned int return_id;      // An entry-defined return ID 
  unsigned int log_nsamples;
  float distance;
  float return_data[2 * (1<<MAX_RADAR_LOGN)];
} radar_dict_entry_t;

#include "viterbi_utils.h"
typedef struct {
  unsigned int msg_num;
  unsigned int msg_id;
  ofdm_param   ofdm_p;
  frame_param  frame_p;
  uint8_t*     in_bits; //[MAX_ENCODED_BITS];
} vit_dict_entry_t;


typedef struct {
  unsigned int id;
} test_dict_entry_t;


extern float IMPACT_DISTANCE; // Minimum distance at which an obstacle "impacts" MyCar (collision case)

extern char* lane_names[NUM_LANES];
extern char* message_names[NUM_MESSAGES];
extern char* object_names[NUM_OBJECTS];

#ifdef USE_SIM_ENVIRON
extern float car_goal_speed;
extern float car_accel_rate;
extern float car_decel_rate;
#endif

extern char  nearest_obj[NUM_LANES];
extern float nearest_dist[NUM_LANES];

#endif
