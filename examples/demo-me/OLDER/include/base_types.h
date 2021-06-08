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

/* Types definitions */
typedef enum {false, true} bool_t;
typedef enum {error, success} status_t;

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
  bool_t active;
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


#endif
