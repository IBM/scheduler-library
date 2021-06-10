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

#ifndef _sim_environs_h
#define _sim_environs_h

#include "verbose.h"
#include "base_types.h"
#include "base_task_types.h"

/* Types definitions */

/* Object state; includes lane, type of object, and speed (0, 1, 2, ...) */
typedef struct obj_struct {
  unsigned    obj_id;
  label_t     object;
  lane_t      lane;
  float       speed;
  distance_t  distance;
  distance_t  size; // Size of object in distance_t units
  
  struct obj_struct* previous;  // The previous object in the list
  struct obj_struct* next;      // The next object in the list
} object_state_t;


/* The "world" consists of the (7) lanes (horizontal) and N vertical distances *
 *       "x"     0         1         2         3        4          5        6
 *   "y"   | LHazard |   Left  |  L-Mid  |  Middle |  R-Mid  |  Right  | RHazard |
 *         |---------|---------|---------|---------|---------|---------|---------|
 *    N    |         |         |         | Truck   |         |         |         |
 *  N-1    |         |         |         |         |         |         |         |
 *  N-2    |         |  Car    |         |         |         |         |         |
 *  N-3    |         |         |         |         |         |         |         |
 *  ...
 *    4    |         |         |         |         |         |         |         |
 *    3    |         |         |         |         |         |         |         |
 *    2    |         |         |         |         |         |         |         |
 *    1    |         |  Pedest |         |         |         |         |         |
 *    0    |         |         |  Myself |         |         |         |         |
 */

// This is the master list of all currently live objects in the world.
//  This is a sorted list (by distance) of objects per lane
extern unsigned global_object_id;
extern object_state_t* the_objects[NUM_LANES];

// External variables
extern float MAX_OBJECT_SIZE; // Max size of an object
extern float MIN_OBJECT_DIST;

extern unsigned max_time_steps; // The max stime steps to simulate

extern bool all_obstacle_lanes_mode;

extern float car_goal_speed;  // The speed My Car wants to maintain (if possible)
extern float car_accel_rate;  // The rate of acceleration toward goal_speed
extern float car_decel_rate;  // The rate of deceleration toward goal_speed

// Function/interface declarations
void     print_object(object_state_t* st);
status_t init_sim_environs(char* wdecsc_fn, vehicle_state_t* vehicle_state);
bool     iterate_sim_environs(vehicle_state_t vehicle_state);
void     visualize_world();


#endif

