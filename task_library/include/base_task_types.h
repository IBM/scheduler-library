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

#ifndef INCLUDED_BASE_TASK_TYPES_H
#define INCLUDED_BASE_TASK_TYPES_H
#include <stdbool.h>

/* Types definitions */
// This is the type for distance measurements
typedef float distance_t;

/* Pre-defined labels used by the computer vision kernel */
typedef enum {
  myself = -1,
  no_label = 0,
  car,
  truck,
  pedestrian,
  bicycle,
  NUM_OBJECTS
} label_t;


/* The potential (horizontal) positions of any object (i.e. lane indications) */
typedef enum {
  lhazard = 0, 
  left, 
  center, 
  right,
  rhazard,
  NUM_LANES
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
  NUM_MESSAGES
} message_t;

typedef enum {
  vit_tiny_msg = 0,
  vit_small_msg,
  vit_medium_msg,
  vit_max_msg
} message_size_t;



/* These are GLOBAL and affect the underlying world, etc. */

#define MAX_DISTANCE        500.0  // Max resolution distance of radar is < 500.0m
#define MAX_DIST_STEP_SIZE  50.0
#define INF_DISTANCE        (MAX_DISTANCE + MAX_DIST_STEP_SIZE)
//#define RADAR_BUCKET_DISTANCE  DIST_STEP_SIZE  // The radar is in steps of 50

/* These thresholds (in meters) are used by the plan_and_control()
 * function to make plan and control decisions.
 */
#define PNC_THRESHOLD_1 155.0
#define PNC_THRESHOLD_2 205.0
#define PNC_THRESHOLD_3 305.0

#define VIT_CLEAR_THRESHOLD  PNC_THRESHOLD_1

extern float IMPACT_DISTANCE; // Minimum distance at which an obstacle "impacts" MyCar (collision case)

extern char* lane_names[NUM_LANES];
extern char* message_names[NUM_MESSAGES];
extern char* object_names[NUM_OBJECTS];

#ifdef USE_SIM_ENVIRON
extern float car_goal_speed;
extern float car_accel_rate;
extern float car_decel_rate;
#endif

#endif
