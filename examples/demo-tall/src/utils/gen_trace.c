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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string.h>
#include <unistd.h>

typedef enum {success, error} status_t;

#include "sim_environs.h"
#include "getopt.h"

bool output_viz_trace = false;
bool output_source_trace = true;

float car_goal_speed = 50.0;
float car_goal_distance = 0.0; 

char world_desc_file_name[256] = "default_world.desc";
float IMPACT_DISTANCE = 50.0;  // Minimum distance at which an obstacle "impacts" MyCar (collision case)

unsigned rand_seed = 0; // Only used if -r <N> option set
bool all_obstacle_lanes_mode = false;

unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')
unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

distance_t MAX_DISTANCE =       500.0;  // Max resolution distance of radar is < 500.0m
distance_t MAX_DIST_STEP_SIZE = 50.0;
distance_t INF_DISTANCE =       550.0; // (MAX_DISTANCE + MAX_DIST_STEP_SIZE)


char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N' };
float    nearest_dist[NUM_LANES];// = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE };


void print_usage(char *pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h          : print this helpful usage info\n");
  printf("    -w <wfile>  : defines the world environment parameters description file <wfile> to use\n");
  printf("    -r <N>      : Sets the rand random number seed to N\n");
  printf("    -A          : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)\n");
}

int main(int argc, char *argv[])
{
  vehicle_state_t vehicle_state;
  vehicle_state.active = true;
  vehicle_state.lane = center;
  vehicle_state.speed = 50;

  int opt;

  while ((opt = getopt(argc, argv, ":hAw:r:D:")) != -1) {
    switch (opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
      break;
    case 'w':
      snprintf(world_desc_file_name, 255, "%s", optarg);
      break;
    case 'A':
      all_obstacle_lanes_mode = true;
      break;
    case 'r':
      rand_seed = atoi(optarg);
      break;
    case 'D':
      MAX_DISTANCE = atoi(optarg);
      INF_DISTANCE = (MAX_DISTANCE + MAX_DIST_STEP_SIZE);
      printf("Set MAX_DISTANCE to %.1f and INF_DISTANCE to %.1f\n", MAX_DISTANCE, INF_DISTANCE);
      break;
    }
  }

  // optind is for the extra arguments which were not parsed
  for (; optind < argc; optind++) {
    printf("extra arguments: %s\n", argv[optind]);
  }

  printf("Setting up world space from %s\n", world_desc_file_name);
  if (init_sim_environs(world_desc_file_name, &vehicle_state) == error) {
    printf("ERROR in init_sim_environs... exiting\n");
    exit(-1);
  }

  printf("All-Obstacle-Lanes-Mode = %u\n", all_obstacle_lanes_mode);
  
  printf("\n");
  for (int i = 0; i < 5000; i++) {
    DEBUG(printf("\n\nTime Step %d\n", i));
    iterate_sim_environs(vehicle_state);
    //visualize_world();
  }
  return 0;
}
