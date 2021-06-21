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


#define MAX_OBJ_IN_LANE  16

#define MY_APP_ACCEL_TYPES  4

typedef struct {
  unsigned int index;          // A global index (of all radar dictionary entries
  unsigned int set;            // The set this entry is in
  unsigned int index_in_set;   // The index in the set for this entry
  unsigned int return_id;      // An entry-defined return ID 
  unsigned int log_nsamples;
  float distance;
  float return_data[2 * (1<<MAX_RADAR_LOGN)];
} radar_dict_entry_t;

#include "viterbi_types.h"
typedef struct {
  unsigned int msg_num;
  unsigned int msg_id;
  ofdm_param   ofdm_p;
  frame_param  frame_p;
  uint8_t      in_bits[MAX_ENCODED_BITS];
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

extern distance_t MAX_DISTANCE;
extern distance_t MAX_DIST_STEP_SIZE;
extern distance_t INF_DISTANCE;

#endif
