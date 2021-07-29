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

#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h> // Unbuntu 18 on x86_64 has this at /usr/include/x86_64-linux-gnu/sys/socket.h
#include <arpa/inet.h> // for inet_addr
#include <sys/time.h>
#include <unistd.h>
#include <math.h>

#include "viterbi_types.h"
#include "sdr_base.h"
#ifndef HPVM
#include "fft_task.h"
#include "vit_task.h"
#include "cv_task.h"
#include "test_task.h"
#include "plan_ctrl_task.h"
#endif
#include "kernels_api.h"

#ifdef USE_SIM_ENVIRON
 #include "sim_environs.h"
#else
 #include "read_trace.h"
#endif

extern unsigned time_step;
extern unsigned task_size_variability;

distance_t MAX_DISTANCE =       500.0;  // Max resolution distance of radar is < 500.0m
distance_t MAX_DIST_STEP_SIZE = 50.0;
distance_t INF_DISTANCE =       550.0; // (MAX_DISTANCE + MAX_DIST_STEP_SIZE)
//#define RADAR_BUCKET_DISTANCE  DIST_STEP_SIZE  // The radar is in steps of 50

#if (BUILD_WITH_N_LANES == 5)
char* lane_names[NUM_LANES] = {"LHazard", "Left", "Center", "Right", "RHazard" };
#elif (BUILD_WITH_N_LANES == 9)
char* lane_names[NUM_LANES] = {"LHazard", "Far-Left", "Left", "LeftCntr", "Center", "RtCntr", "Right", "Far-Right", "RHazard" };
#endif

char* message_names[NUM_MESSAGES] = {"Safe_L_or_R", "Safe_R_only", "Safe_L_only", "Unsafe_L_or_R" };
char* object_names[NUM_OBJECTS] = {"Nothing", "Car", "Truck", "Person", "Bike" };
char  object_char[NUM_OBJECTS] = {'N', 'C', 'T', 'P', 'B' };


#ifdef VERBOSE
bool output_viz_trace = true;
#else
bool output_viz_trace = false;
#endif
bool output_source_trace = false;


unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')

unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

#if (BUILD_WITH_N_LANES == 5)
char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N'};
float    nearest_dist[NUM_LANES]; // = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE};
#elif (BUILD_WITH_N_LANES == 9)
char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N', 'N', 'N', 'N', 'N' };
float    nearest_dist[NUM_LANES]; // = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE };
#endif

unsigned hist_total_objs[NUM_LANES * MAX_OBJ_IN_LANE];

unsigned rand_seed = 0; // Only used if -r <N> option set

distance_t RADAR_BUCKET_DISTANCE = 0.0;
distance_t IMPACT_DISTANCE; //       = MAX_DIST_STEP_SIZE; // Minimum distance at which a

vehicle_state_t other_car; // This is the reported state fo the other car (from their transmission)

unsigned label_match[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0};  // Times CNN matched dictionary
unsigned label_lookup[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0}; // Times we used CNN for object classification
unsigned label_mismatch[NUM_OBJECTS][NUM_OBJECTS] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};



#define MAX_RDICT_ENTRIES      120   // This should be updated eventually...
unsigned int         num_radar_samples_sets = 0;
unsigned int         radar_dict_items_per_set = 0;
radar_dict_entry_t** the_radar_return_dict;
float                radar_bucket_size_per_dict_set[MAX_RDICT_SAMPLE_SETS];
unsigned int         radar_log_nsamples_per_dict_set[MAX_RDICT_SAMPLE_SETS];

unsigned radar_total_calc = 0;
unsigned hist_pct_errs[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES][5];// = {0, 0, 0, 0, 0}; // One per distance, plus global?
unsigned hist_distances[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES];
char*    hist_pct_err_label[5] = {"   0%", "<  1%", "< 10%", "<100%", ">100%"};
unsigned radar_inputs_histogram[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES];

unsigned viterbi_messages_histogram[1502];

uint8_t descramble[1600]; // I think this covers our max use cases
uint8_t actual_msg[1600];

unsigned int      num_viterbi_dictionary_items = 0;
vit_dict_entry_t* the_viterbi_trace_dict;

unsigned vit_msgs_per_step;
const char* vit_msgs_per_step_str[VITERBI_MSGS_PER_STEP] = {"One message per time step",
							    "One message per obstacle per time step",
							    "One msg per obstacle + 1 per time step" };

unsigned total_msgs = 0; // Total messages decoded during the full run
unsigned bad_decode_msgs = 0; // Total messages decoded incorrectly during the full run



unsigned total_test_tasks = 0;  // Total test tasks executed during the full run
unsigned bad_test_task_res = 0; // Total test task "bad-resutls" during the full run


distance_t PNC_THRESHOLD_1 = 155.0;
distance_t PNC_THRESHOLD_2 = 205.0;
distance_t PNC_THRESHOLD_3 = 305.0;
distance_t VIT_CLEAR_THRESHOLD = 155.0;

#ifdef ENABLE_NVDLA
 extern void initNVDLA();
 extern void runImageonNVDLAWrapper(char *Image);
#endif


void output_VizTrace_line(unsigned min_obst_lane, unsigned max_obst_lane, vehicle_state_t* vehicle_state, vehicle_state_t* other_car)
{
  if (!vehicle_state->active) {
    printf("%4u  VizTrace: %d:%u,", time_step, -vehicle_state->lane, 0); //(unsigned)vehicle_state->distance);
  } else {
    printf("%4u  VizTrace: %d:%u,", time_step, vehicle_state->lane, 0); //(unsigned)vehicle_state->distance);
  }      
  for (int in_lane = min_obst_lane; in_lane < max_obst_lane; in_lane++) {
    //printf("[%u]", in_lane);
    if (output_viz_trace && (in_lane > min_obst_lane)) {
      printf(",");
    }
    for (int oi = 0; oi < obj_in_lane[in_lane]; oi++) {
      if (oi > 0) { printf(" "); }
      printf("%c:%u", lane_obj[in_lane][oi], lane_dist[in_lane][oi]);
    }
  }
  if (other_car->active) {
    printf(",%d:%u\n", other_car->lane, 0); // (unsigned)other_car->distance);
  } else {
    printf(",%d:%u\n", -other_car->lane, 0); //(unsigned)other_car->distance);
  }

} // output_VizTrace_line

status_t init_cv_kernel(char* py_file, char* dict_fn)
{
  //DEBUG(
  printf("In the init_cv_kernel routine\n");//);

  /** The CV kernel uses a different method to select appropriate inputs; dictionary not needed **/
  // Initialization to run Keras CNN code

 #ifdef ENABLE_NVDLA
  // Initialize NVDLA
  printf("  Calling the initNVDLA routine\n");
  initNVDLA();
  printf("  Back from the initNVDLA routine\n");
 #endif
  return success;
}


label_t iterate_cv_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_cv_kernel\n"));

  unsigned tr_val = 0; // Default nothing
  switch(nearest_obj[vs.lane]) {
    case 'N' : tr_val = no_label; break;
    case 'B' : tr_val = bicycle; break;
    case 'C' : tr_val = car; break;
    case 'P' : tr_val = pedestrian; break;
    case 'T' : tr_val = truck; break;
  default: printf("ERROR : Unknown object type in cv trace: '%c'\n", nearest_obj[vs.lane]); 
           exit(-2);
  }
  label_t d_object = (label_t)tr_val;

  return d_object;
}

void post_execute_cv_kernel(label_t tr_val, label_t cv_object)
{
  //printf("CV_POST: Compare %u to %u\n", tr_val, cv_object);
  if (cv_object == tr_val) {
    label_match[tr_val]++;
    label_match[NUM_OBJECTS]++;
  } else {
    DEBUG(printf("CV-MISMATCH: cv_object %u tr_val %u\n", cv_object, tr_val));
    label_mismatch[tr_val][cv_object]++;
  }
  label_lookup[NUM_OBJECTS]++;
  label_lookup[tr_val]++;
}


status_t init_radar_kernel(char* dict_fn, int set_to_use)
{
  DEBUG(printf("In init_radar_kernel...\n"));

  // Read in the radar distances dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF) {
    printf("Error: unable to open dictionary file %s\n", dict_fn);
    fclose(dictF);
    return error;
  }
  // Read the number of definitions
  if (fscanf(dictF, "%u %u\n", &num_radar_samples_sets, &radar_dict_items_per_set) != 2) {
    printf("ERROR reading the number of Radar Dictionary sets and items per set\n");
    exit(-2);
  }
  printf("  There are %u dictionary sets of %u entries each\n", num_radar_samples_sets, radar_dict_items_per_set);
  the_radar_return_dict = (radar_dict_entry_t**)calloc(num_radar_samples_sets, sizeof(radar_dict_entry_t*));
  if (the_radar_return_dict == NULL) {
    printf("ERROR : Cannot allocate Radar Trace Dictionary memory space\n");
    fclose(dictF);
    return error;
  }
  if (set_to_use >= num_radar_samples_sets) {
    printf("ERROR : Specified set to use is %u but only %u sets in the dictionary file\n", set_to_use, num_radar_samples_sets);
    fclose(dictF);
    return error;
  }
  for (int si = 0; si < num_radar_samples_sets; si++) {
    the_radar_return_dict[si] = (radar_dict_entry_t*)calloc(radar_dict_items_per_set, sizeof(radar_dict_entry_t));
    if (the_radar_return_dict[si] == NULL) {
      printf("ERROR : Cannot allocate Radar Trace Dictionary memory space for set %u\n", si);
      fclose(dictF);
      return error;
    }
  }
  unsigned tot_dict_values = 0;
  unsigned tot_index = 0;
  for (int si = 0; si < num_radar_samples_sets; si++) {
    if (fscanf(dictF, "%u %f\n", &(radar_log_nsamples_per_dict_set[si]), &(radar_bucket_size_per_dict_set[si])) != 2) {
      printf("ERROR reading the Log2 number of Radar Dictionary samples for set %u\n", si);
      exit(-2);
    }
    DEBUG(printf("  Dictionary set %u entries should all have %u log_nsamples (with bucket-size %.1f)\n", si, radar_log_nsamples_per_dict_set[si], radar_bucket_size_per_dict_set[si]));
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      unsigned entry_id;
      unsigned entry_log_nsamples;
      float entry_dist;
      unsigned entry_dict_values = 0;
      printf("    Reading Radar dict set %u entry %u\n", si, di);
      if (fscanf(dictF, "%u %u %f", &entry_id, &entry_log_nsamples, &entry_dist) != 3) {
	printf("ERROR reading Radar Dictionary set %u entry %u header\n", si, di);
        exit(-2);
      }
      if (radar_log_nsamples_per_dict_set[si] != entry_log_nsamples) {
	printf("ERROR reading Radar Dictionary set %u entry %u header : Mismatch in log2 samples : %u vs %u\n", si, di, entry_log_nsamples, radar_log_nsamples_per_dict_set[si]);
	exit(-2);
      }

      DEBUG(printf("  Reading rad dictionary set %u entry %u : %u %u %f\n", si, di, entry_id, entry_log_nsamples, entry_dist));
      the_radar_return_dict[si][di].index = tot_index++;  // Set, and increment total index
      the_radar_return_dict[si][di].set = si;
      the_radar_return_dict[si][di].index_in_set = di;
      the_radar_return_dict[si][di].return_id = entry_id;
      the_radar_return_dict[si][di].log_nsamples = entry_log_nsamples;
      the_radar_return_dict[si][di].distance =  entry_dist;
      the_radar_return_dict[si][di].radar_return_data = (float*)malloc(2 * (1<<entry_log_nsamples) * sizeof(float));
      for (int i = 0; i < 2*(1<<entry_log_nsamples); i++) {
	float fin;
	if (fscanf(dictF, "%f", &fin) != 1) {
	  printf("ERROR reading Radar Dictionary set %u entry %u data entries\n", si, di);
	  exit(-2);
	}
	the_radar_return_dict[si][di].radar_return_data[i] = fin;
	tot_dict_values++;
	entry_dict_values++;
      }
      DEBUG(printf("    Read in dict set %u entry %u with %u total values\n", si, di, entry_dict_values));
    } // for (int di across radar dictionary entries per set
    DEBUG(printf("   Done reading in Radar dictionary set %u\n", si));
  } // for (si across radar dictionary sets)
  DEBUG(printf("  Read %u sets with %u entries totalling %u values across them all\n", num_radar_samples_sets, radar_dict_items_per_set, tot_dict_values));

  if (!feof(dictF)) {
    printf("NOTE: Did not hit eof on the radar dictionary file %s\n", dict_fn);
    while(!feof(dictF)) {
      char c;
      if (fscanf(dictF, "%c", &c) != 1) {
	printf("Couldn't read final character\n");
      } else {
	printf("Next char is %c = %u = 0x%x\n", c, c, c);
      }
    }
    //if (!feof(dictF)) { printf("and still no EOF\n"); }
  }
  fclose(dictF);

  RADAR_BUCKET_DISTANCE = radar_bucket_size_per_dict_set[set_to_use];
  printf("Using RADAR_BUCKET_DISTANCE of %.1f\n", RADAR_BUCKET_DISTANCE);
  
  // Initialize hist_pct_errs values
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      hist_distances[si][di] = 0;
      for (int i = 0; i < 5; i++) {
	hist_pct_errs[si][di][i] = 0;
      }
    }
  }

    //Clear the inputs (injected) histogram
  for (int i = 0; i < MAX_RDICT_SAMPLE_SETS; i++) {
    for (int j = 0; j < MAX_RDICT_ENTRIES; j++) {
      radar_inputs_histogram[i][j] = 0;
    }
  }

  return success;
}

// These routine selects a random FFT input (from the dictionary) or the specific Critical Radar Task FFT Input
//  Used to support variable non-critical task sizes, and histogram stats gathering.
radar_dict_entry_t* select_random_radar_input()
{
  int s_num = (rand() % (num_radar_samples_sets)); // Return a value from [0,num_radar_samples_sets) */
  int e_num = (rand() % (radar_dict_items_per_set)); // Return a value from [0,radar_dict_items_per_set) */
  radar_inputs_histogram[s_num][e_num]++;
  return &(the_radar_return_dict[s_num][e_num]);
}

radar_dict_entry_t* select_critical_radar_input(radar_dict_entry_t* rdentry_p)
{
  radar_inputs_histogram[rdentry_p->set][rdentry_p->index_in_set]++;
  return rdentry_p;
}



radar_dict_entry_t* iterate_radar_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_radar_kernel\n"));
  unsigned tr_val = nearest_dist[vs.lane] / RADAR_BUCKET_DISTANCE;  // The proper message for this time step and car-lane
  radar_inputs_histogram[crit_fft_samples_set][tr_val]++;
  return &(the_radar_return_dict[crit_fft_samples_set][tr_val]);
}


void post_execute_radar_kernel(unsigned set, unsigned index, distance_t tr_dist, distance_t dist)
{
  // Get an error estimate (Root-Squared?)
  float error;
  radar_total_calc++;
  hist_distances[set][index]++;
  if ((tr_dist >= 500.0) && (dist > 10000.0)) {
    error = 0.0;
  } else {
    error = (tr_dist - dist);
  }
  float abs_err = fabs(error);
  float pct_err;
  if (tr_dist != 0.0) {
    pct_err = abs_err/tr_dist;
  } else {
    pct_err = abs_err;
  }

  DEBUG(printf("%f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", tr_dist, dist, error, abs_err, pct_err));
  //printf("IDX: %u :: %f vs %f : ERROR : %f   ABS_ERR : %f PCT_ERR : %f\n", index, tr_dist, dist, error, abs_err, pct_err);
  if (pct_err == 0.0) {
    hist_pct_errs[set][index][0]++;
  } else if (pct_err < 0.01) {
    hist_pct_errs[set][index][1]++;
  } else if (pct_err < 0.1) {
    printf("RADAR_LT010_ERR : TS %u : %f vs %f : ERROR : %f   PCT_ERR : %f\n", time_step, tr_dist, dist, error, pct_err);
    hist_pct_errs[set][index][2]++;
  } else if (pct_err < 1.00) {
    printf("RADAR_LT100_ERR : TS %u : %f vs %f : ERROR : %f   PCT_ERR : %f\n", time_step, tr_dist, dist, error, pct_err);
    hist_pct_errs[set][index][3]++;
  } else {
    printf("RADAR_GT100_ERR : TS %u : %f vs %f : ERROR : %f   PCT_ERR : %f\n", time_step, tr_dist, dist, error, pct_err);
    hist_pct_errs[set][index][4]++;
  }
}



/* This is the initialization of the Viterbi dictionary data, etc.
 * The format is:
 *  <n> = number of dictionary entries (message types)
 * For each dictionary entry:
 *  n1 n2 n3 n4 n5 : OFDM parms:
 *  m1 m2 m3 m4 m5 : FRAME parms:
 *  x1 x2 x3 ...   : The message bits (input to decode routine)
 */

// The PORTS are defined in the compilation process

char wifi_inet_addr_str[20] = "127.0.0.1";

int xmit_sock = 0;
int recv_sock = 0;

unsigned xmit_count = 0;
unsigned recv_count = 0;

char *ack = "OK";

xmit_msg_t wifi_out_msg;

struct sockaddr_in xmit_servaddr;
struct sockaddr_in recv_servaddr;

struct timeval stop_wifi_pipe, start_wifi_pipe;
uint64_t wifi_pipe_sec  = 0LL;
uint64_t wifi_pipe_usec = 0LL;

struct timeval stop_wifi_send, start_wifi_send;
uint64_t wifi_send_sec  = 0LL;
uint64_t wifi_send_usec = 0LL;

struct timeval stop_wifi_send_rl, start_wifi_send_rl;
uint64_t wifi_send_rl_sec  = 0LL;
uint64_t wifi_send_rl_usec = 0LL;

struct timeval stop_wifi_send_im, start_wifi_send_im;
uint64_t wifi_send_im_sec  = 0LL;
uint64_t wifi_send_im_usec = 0LL;

struct timeval stop_wifi_recv_th, start_wifi_recv_th;
uint64_t wifi_recv_th_sec  = 0LL;
uint64_t wifi_recv_th_usec = 0LL;

struct timeval stop_wifi_lmap_wait, start_wifi_lmap_wait;
uint64_t wifi_lmap_wait_sec  = 0LL;
uint64_t wifi_lmap_wait_usec = 0LL;

struct timeval stop_wifi_recv_wait, start_wifi_recv_wait;
uint64_t wifi_recv_wait_sec  = 0LL;
uint64_t wifi_recv_wait_usec = 0LL;

struct timeval stop_wifi_recv_all, start_wifi_recv_all;
uint64_t wifi_recv_all_sec  = 0LL;
uint64_t wifi_recv_all_usec = 0LL;

struct timeval stop_wifi_recv, start_wifi_recv;
uint64_t wifi_recv_sec  = 0LL;
uint64_t wifi_recv_usec = 0LL;

struct timeval stop_recv_pipe, start_recv_pipe;
uint64_t recv_pipe_sec  = 0LL;
uint64_t recv_pipe_usec = 0LL;

uint8_t recvd_in[sizeof(vit_dict_entry_t) + MAX_ENCODED_BITS*sizeof(uint8_t) + 4096];
vit_dict_entry_t temp_vit_dict_entry;

#include "occ_grid.h"

bool show_local_occ_grid = false;
bool show_remote_occ_grid = false;
bool show_fused_occ_grid = false;
bool show_side_by_occ_grids = false;

char* occ_grid_from_local_value_str[OCC_GRID_NUM_OF_VALS] = { "???",   // OCC_GRID_UNKNOWN_VAL     0
							      "   ",   // OCC_GRID_NO_OBSTACLE_VAL 1
							      "XXX",   // OCC_GRID_OBSTACLE_VAL    2
							      "{V}",   // OCC_GRID_YOUR_CAR_VAL    3
							      "[A]",   // OCC_GRID_MY_CAR_VAL      4
							      "ERR" }; // OCC_GRID_ERROR_VAL       5


char* occ_grid_from_remote_value_str[OCC_GRID_NUM_OF_VALS] = { "???",   // OCC_GRID_UNKNOWN_VAL     0
							       "   ",   // OCC_GRID_NO_OBSTACLE_VAL 1
							       "XXX",   // OCC_GRID_OBSTACLE_VAL    2
							       "[A]",   // OCC_GRID_YOUR_CAR_VAL    3
							       "{V}",   // OCC_GRID_MY_CAR_VAL      4
							       "ERR" }; // OCC_GRID_ERROR_VAL       5


unsigned MAX_GRID_DIST_NEAR_FAR_IDX = 3;
//unsigned GRID_DIST_STEP_SIZE = 5;

//unsigned OCC_GRID_X_DIM = NUM_LANES;
//unsigned OCC_GRID_Y_DIM = 101;    // MAX_DISTANCE / GRID_DIST_STEP_SIZE;
// - THESE ARE IN occ_grid.h : #define OCC_GRID_X_DIM  NUM_LANES
// - THESE ARE IN occ_grid.h : #define OCC_GRID_Y_DIM  101

unsigned OCC_NEXT_LANE_NEAR[4] = {500, 100, 75, 50};
unsigned OCC_NEXT_LANE_FAR[4]  = {0, 175, 300, 450};
unsigned OCC_NEXT_LANE_NEAR_GRID[4];
unsigned OCC_NEXT_LANE_FAR_GRID[4];

unsigned MY_CAR_OCC_GRID_SIZE = 5; // 25 / GRID_DIST_STEP_SIZE;

uint8_t local_occ_grid[OCC_GRID_X_DIM][OCC_GRID_Y_DIM];
uint8_t remote_occ_grid[OCC_GRID_X_DIM][OCC_GRID_Y_DIM];
uint8_t fused_occ_grid[OCC_GRID_X_DIM][OCC_GRID_Y_DIM];

status_t init_vit_kernel(char* dict_fn)
{
  DEBUG(printf("In init_vit_kernel...\n"));
  distance_t IMPACT_DISTANCE = MAX_DIST_STEP_SIZE; // Minimum distance at which a
 
  // Set up some globals to be used in the run...
  printf("Setting Near/Far Grid to : ");
  for (int i = 0; i < 4; i++) {
    OCC_NEXT_LANE_NEAR_GRID[i] = OCC_NEXT_LANE_NEAR[i]/GRID_DIST_STEP_SIZE; 
    OCC_NEXT_LANE_FAR_GRID[i]  = OCC_NEXT_LANE_FAR[i]/GRID_DIST_STEP_SIZE;
    printf("%u %u : ", OCC_NEXT_LANE_NEAR_GRID[i], OCC_NEXT_LANE_FAR_GRID[i]);
  }
  printf("\n");
  
  //printf(" XMIT-PORT = %u and RECV-PORT = %u on IP %s\n", XMIT_PORT, RECV_PORT, wifi_inet_addr_str);
  // Set up the WiFi interchange Sockets.
  // Open and connect to the XMIT_SERVER
  printf("Connecting to xmit-server at IP %s PORT %u\n", wifi_inet_addr_str, XMIT_PORT);
  if ((xmit_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
    printf("WIFI XMIT Socket creation failed...\n");
    exit(0);
  }
  else {
    printf("WIFI XMIT Socket successfully created..\n");
  }

  xmit_servaddr.sin_family = AF_INET;  /* Address family = Internet */
  xmit_servaddr.sin_addr.s_addr = inet_addr(wifi_inet_addr_str);  /* Set IP address to localhost */
  xmit_servaddr.sin_port = htons(XMIT_PORT);  /* Set port number, using htons function to use proper byte order */
  memset(xmit_servaddr.sin_zero, '\0', sizeof(xmit_servaddr.sin_zero));  /* Set all bits of the padding field to 0 */

  while (true) {
    if (connect(xmit_sock, (struct sockaddr*)&xmit_servaddr, sizeof(xmit_servaddr)) != 0) {
      printf("connection with the WIFI XMIT server failed...\n");
      sleep(1);
      continue;
    }
    else {
      printf("connected to the WIFI XMIT server..\n");
      break;
    }
  }

  // Open and connect to the RECV_SERVER 
  printf("Connecting to recv-server at IP %s PORT %u\n", wifi_inet_addr_str, RECV_PORT);
  if ((recv_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)  {
    printf("WIFI RECV Socket creation failed...\n");
    exit(0);
  }
  else {
    printf("WIFI RECV Socket successfully created..\n");
  }

  recv_servaddr.sin_family = AF_INET;
  recv_servaddr.sin_addr.s_addr = inet_addr(wifi_inet_addr_str); //"127.0.0.1");
  recv_servaddr.sin_port = htons(RECV_PORT);
  memset(recv_servaddr.sin_zero, '\0', sizeof(recv_servaddr.sin_zero));  /* Set all bits of the padding field to 0 */

  while (true) {
    if (connect(recv_sock, (struct sockaddr*)&recv_servaddr, sizeof(recv_servaddr)) != 0) {
      printf("connection with the WIFI RECV server failed...\n");
      sleep(1);
      continue;
    }
    else {
      printf("connected to the WIFI RECV server..\n");
      break;
    }
  }

  //Clear the messages (injected) histogram
  for (int i = 0; i < 1502; i++) {
    viterbi_messages_histogram[i] = 0;
  }
  DEBUG(printf("DONE with init_vit_kernel -- returning success\n"));
  return success;
}

/* Each time-step of the trace, we read in the
 * trace values for the various lanes
 */


int read_all(int sock, char* buffer, int xfer_in_bytes)
{
  char * ptr;
  int message_size = xfer_in_bytes;
  char* message_ptr = buffer;
  int total_recvd = 0;
  while(total_recvd < message_size) {
    unsigned rem_len = (message_size - total_recvd);
    int valread = read(sock , message_ptr, rem_len);
    message_ptr = message_ptr + valread;
    total_recvd += valread;
    SDEBUG(printf("        read %d bytes for %d total bytes of %d\n", valread, total_recvd, message_size));
    if (valread == 0) {
      DEBUG(printf("  read_all got ZERO bytes -- END of TRANSFER?\n"));
      return total_recvd;
    }
  }
  return total_recvd;
}



// This actually uses the standard inputs/values (e.g. the local state) for most of this,
//  but it resolves whether the "other" car is in the "danger proximity" from the fused map
/* #undef  DEBUG */
/* #define DEBUG(x) x */

message_t get_safe_dir_message_from_fused_occ_map(vehicle_state_t vs)
{
  unsigned msg_val = 0; // set a default to avoid compiler messages 
  DEBUG(printf("At time %u We are in lane %u %s TH %.1f : obj in %u is %c at %.1f\n", time_step, vs.lane, lane_names[vs.lane], VIT_CLEAR_THRESHOLD, vs.lane, nearest_obj[vs.lane], nearest_dist[vs.lane]));
  switch (vs.lane) {
  case lhazard:
    {
      unsigned nd_l = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[lhazard+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      DEBUG(printf("  A: Lane %u TH %.1f : obj in %u is %c at %u\n", vs.lane, VIT_CLEAR_THRESHOLD, vs.lane+1, nearest_obj[vs.lane+1], nd_l));
      if ((nearest_obj[lhazard+1] != 'N') && (nd_l < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the lhazard+1 lane within threshold distance
	msg_val = 3; // Unsafe to move from lhazard lane into the lhazard+1 lane
      } else {
	// Check that the other "MY_CAR" is not in one of the 2 lanes to the right
	//   (or else it might merge into the lane I'm merging into)
	DEBUG(printf(" fused %u = %u  and %u = %u\n", lhazard+1, fused_occ_grid[lhazard+1][1], lhazard+1, fused_occ_grid[lhazard+1][1]));
	if ((fused_occ_grid[lhazard+1][1] != OCC_GRID_MY_CAR_VAL) && (fused_occ_grid[lhazard+1][1] != OCC_GRID_MY_CAR_VAL)) {
	  msg_val = 1;
	} else {
	  msg_val = 3;
	}
      }
    }
    break;
#if (BUILD_WITH_N_LANES == 5)
  case left: // the lane immediately to the right of the lhazard
#elif (BUILD_WITH_N_LANES == 9)
  case far_left: // the lane immediately to the right of the lhazard
#endif
    {
      unsigned ndm1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      unsigned ndp1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      msg_val = 0;
      DEBUG(printf(" B:  Lane %u TH %.1f : obj in %u is %c at %.1f : obj in %u is %c at %.1f\n", vs.lane, VIT_CLEAR_THRESHOLD, 
		   vs.lane-1, nearest_obj[vs.lane-1], nearest_dist[vs.lane-1],
		   vs.lane+1, nearest_obj[vs.lane+1], nearest_dist[vs.lane+1]));
      DEBUG(printf(" fused %u = %u\n", vs.lane-1, fused_occ_grid[vs.lane-1][1]));
      if (((nearest_obj[vs.lane-1] != 'N') && (ndm1 < VIT_CLEAR_THRESHOLD)) // Some object is in the Left lane at distance 0 or 1
	  || ((fused_occ_grid[vs.lane-1][1] == OCC_GRID_MY_CAR_VAL))) {
	DEBUG(printf("    Marking unsafe to move left\n"));
	msg_val += 1; // Unsafe to move from this lane to the left.
      }
      DEBUG(printf(" fused %u = %u  and %u = %u\n", vs.lane+1, fused_occ_grid[vs.lane+1][1], vs.lane+2, fused_occ_grid[vs.lane+2][1]));
      if (((nearest_obj[vs.lane+1] != 'N') && (ndp1 < VIT_CLEAR_THRESHOLD)) // Some object is in the Right lane at distance 0 or 1
	  || (fused_occ_grid[vs.lane+1][1] == OCC_GRID_MY_CAR_VAL) || (fused_occ_grid[vs.lane+2][1] == OCC_GRID_MY_CAR_VAL) ) {
	DEBUG(printf("    Marking unsafe to move right\n"));
	msg_val += 2; // Unsafe to move from this lane to the right.
      }
    }
    break;

#if (BUILD_WITH_N_LANES == 5)
  case center: // Lanes with 2 lanes on each side...
#elif (BUILD_WITH_N_LANES == 9)
  case left:  // Lanes with 2 lanes on each side...
  case l_center:
  case center:
  case r_center:
  case right:
#endif
    {
      unsigned ndm1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      unsigned ndp1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      msg_val = 0;
      DEBUG(printf("  C: Lane %u TH %.1f : obj in %u is %c at %.1f : obj in %u is %c at %.1f\n", vs.lane, VIT_CLEAR_THRESHOLD, 
		   vs.lane-1, nearest_obj[vs.lane-1], nearest_dist[vs.lane-1],
		   vs.lane+1, nearest_obj[vs.lane+1], nearest_dist[vs.lane+1]));
      DEBUG(printf("   NObj %c at %u vs %u  with fused %u = %u vs %u  and %u = %u vs %u\n", nearest_obj[vs.lane-1], ndm1, (unsigned)VIT_CLEAR_THRESHOLD, vs.lane-1, fused_occ_grid[vs.lane-1][1], OCC_GRID_MY_CAR_VAL, vs.lane-2, fused_occ_grid[vs.lane-2][1], OCC_GRID_MY_CAR_VAL));
      if (((nearest_obj[vs.lane-1] != 'N') && (ndm1 < VIT_CLEAR_THRESHOLD)) // Some object is in the Right lane at distance 0 or 1
	  || (fused_occ_grid[vs.lane-1][1] == OCC_GRID_MY_CAR_VAL) || (fused_occ_grid[vs.lane-2][1] == OCC_GRID_MY_CAR_VAL) ) {
	DEBUG(printf("    Marking unsafe to move left\n"));
	msg_val += 1; // Unsafe to move from this lane to the left.
      }
      DEBUG(printf("   NObj %c at %u vs %u  with fused %u = %u vs %u  and %u = %u vs %u\n", nearest_obj[vs.lane+1], ndp1, (unsigned)VIT_CLEAR_THRESHOLD, vs.lane+1, fused_occ_grid[vs.lane+1][1], OCC_GRID_MY_CAR_VAL, vs.lane+2, fused_occ_grid[vs.lane+2][1], OCC_GRID_MY_CAR_VAL));
      if (((nearest_obj[vs.lane+1] != 'N') && (ndp1 < VIT_CLEAR_THRESHOLD)) // Some object is in the Right lane at distance 0 or 1
	  || (fused_occ_grid[vs.lane+1][1] == OCC_GRID_MY_CAR_VAL) || (fused_occ_grid[vs.lane+2][1] == OCC_GRID_MY_CAR_VAL) ) {
	DEBUG(printf("    Marking unsafe to move right\n"));
	msg_val += 2; // Unsafe to move from this lane to the right.
      }
    }
    break;

#if (BUILD_WITH_N_LANES == 5)
  case right: // the lane immediately to the left of the rhazard
#elif (BUILD_WITH_N_LANES == 9)
  case far_right: // the lane immediately to the left of the rhazard
#endif
    {
      unsigned ndm1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      unsigned ndp1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      msg_val = 0;
      DEBUG(printf("  D: Lane %u TH %.1f : obj in %u is %c at %.1f : obj in %u is %c at %.1f\n", vs.lane, VIT_CLEAR_THRESHOLD, 
		   vs.lane-1, nearest_obj[vs.lane-1], nearest_dist[vs.lane-1],
		   vs.lane+1, nearest_obj[vs.lane+1], nearest_dist[vs.lane+1]));
      DEBUG(printf(" fused %u = %u  and %u = %u\n", vs.lane-1, fused_occ_grid[vs.lane-1][1], vs.lane-2, fused_occ_grid[vs.lane-2][1]));
      if (((nearest_obj[vs.lane-1] != 'N') && (ndm1 < VIT_CLEAR_THRESHOLD)) // Some object is in the Right lane at distance 0 or 1
	  || (fused_occ_grid[vs.lane-1][1] == OCC_GRID_MY_CAR_VAL) || (fused_occ_grid[vs.lane-2][1] == OCC_GRID_MY_CAR_VAL) ) {
	DEBUG(printf("    Marking unsafe to move left\n"));
	msg_val += 1; // Unsafe to move from this lane to the left.
      }
      DEBUG(printf(" fused %u = %u\n", vs.lane+1, fused_occ_grid[vs.lane+1][1]));
      if (((nearest_obj[vs.lane+1] != 'N') && (ndp1 < VIT_CLEAR_THRESHOLD)) // Some object is in the Right lane at distance 0 or 1
	  || ((fused_occ_grid[vs.lane+1][1] == OCC_GRID_MY_CAR_VAL)) ) {
	DEBUG(printf("    Marking unsafe to move right\n"));
	msg_val += 2; // Unsafe to move from this lane to the right.
      }
    }
    break;

  case rhazard:
    {
      unsigned nd_r = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[rhazard-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
      DEBUG(printf("  E: Lane %u TH %.1f : obj in %u is %c at %u\n", vs.lane, VIT_CLEAR_THRESHOLD, vs.lane-1, nearest_obj[vs.lane-1], nd_r));
      if ((nearest_obj[rhazard-1] != 'N') && (nd_r < VIT_CLEAR_THRESHOLD)) {
	// Some object is in the rhazard-1 lane within threshold distance
	msg_val = 3; // Unsafe to move from rhazard lane to the left
      } else {
	if ((fused_occ_grid[rhazard-1][1] != OCC_GRID_MY_CAR_VAL) && (fused_occ_grid[rhazard-1][1] != OCC_GRID_MY_CAR_VAL)) {
	  msg_val = 2;
	} else {
	  msg_val = 3;
	}
      }
    }
    break;
  default:
    printf("ERROR : car is in lane %u (outside legal lanes)\n", vs.lane);
    break;
  }
  DEBUG(printf("Viterbi final message for lane %u %s = %u\n", vs.lane, lane_names[vs.lane], msg_val));
  return msg_val;
}
/* #undef  DEBUG */
/* #define DEBUG(x) */

#define DO_INTERACTIVE(x)

/* #undef DEBUG */
/* #define DEBUG(x) x */
vit_dict_entry_t* iterate_vit_kernel(vehicle_state_t vs, message_t* tr_message)
{
  DEBUG(printf("In iterate_vit_kernel in lane %u = %s\n", vs.lane, lane_names[vs.lane]));
  // Form my local occupancy grid map
  //  We go through the lanes around the one I occupy, using a "fan" of vision such that
  //   I "see" the first bstacle straight ahead,
  //   and I "see" any obstacle in the neighboring lane that is either wihtin N distance or at greater than M distance,
  //   and I "see" into the 3rd lane at 2M distance or greater, etc.
  {
    for (int ix = 0; ix < OCC_GRID_X_DIM; ix++) {
      for (int iy = 0; iy < OCC_GRID_Y_DIM; iy++) {
	local_occ_grid[ix][iy] = OCC_GRID_UNKNOWN_VAL;
	remote_occ_grid[ix][iy] = OCC_GRID_UNKNOWN_VAL;
	fused_occ_grid[ix][iy] = OCC_GRID_UNKNOWN_VAL;
      }
    }

    DEBUG(printf("OBJ_IN_LANE : ");
	  for (int i = 0; i < NUM_LANES; i++) {
	    printf("(%u %u) ", i, obj_in_lane[i]);
	  } printf("\n"));

    // First manage vision in my current lane -- find the closest object ahdead of me.
    int my_lane = vs.lane;
    int nobj = obj_in_lane[my_lane];
    DEBUG(printf("MY Lane %u : obj = %d OBJ %c at %u\n", my_lane, nobj, lane_obj[my_lane][nobj-1], lane_dist[my_lane][nobj-1]));
    {
      int i = nobj-1;
      if ((nobj > 0) && (lane_obj[my_lane][i] != 'N') && (lane_dist[my_lane][i] <= MAX_DISTANCE)) {
	int dist = lane_dist[my_lane][i];
	int grid_dist = (int)(lane_dist[my_lane][i] / GRID_DIST_STEP_SIZE);
	DEBUG(printf("MY Lane %u NEAREST : dist %u gd %u\n", my_lane, dist, grid_dist));
	DEBUG(printf("   Filling lane %u from %d to %d with NO-OBSTACLE\n", my_lane, 0, grid_dist-1));
	for (int yi = 0; yi < grid_dist; yi++) {
	  local_occ_grid[my_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
	}
	local_occ_grid[my_lane][grid_dist] = OCC_GRID_OBSTACLE_VAL;
      } else {
	DEBUG(printf("   Filling lane %u from %d to %d with NO-OBSTACLE\n", my_lane, 0, OCC_GRID_Y_DIM-1));
	for (int yi = 0; yi < OCC_GRID_Y_DIM; yi++) {
	  local_occ_grid[my_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
	}
      }
    }
    // Now we will check the neighboring lanes - span 2 lanes to each side
    for (int lane_over = -2; lane_over <= 2; lane_over++) {
      int check_lane = my_lane + lane_over;
      if ((check_lane >= 0) && (check_lane < NUM_LANES) && (check_lane != my_lane)) { // check lane_over lanes to the left and right 
	int i = obj_in_lane[check_lane]-1;  // i starets at the "last" (nearest) object
	int abs_lane_diff = abs(lane_over);
	DEBUG(printf(" Lane-Over %d LANE %u : obj = %d %c : dist %u CL %u FAR %u\n", lane_over, check_lane, i, lane_obj[check_lane][i], lane_dist[check_lane][i], OCC_NEXT_LANE_NEAR[abs_lane_diff], OCC_NEXT_LANE_FAR[abs_lane_diff]));
	// First, we check the nearest object overall in the lane -- if it is 'N' then no obstacles in the lane
	if ((i < 0) || (lane_obj[check_lane][i] == 'N')) {
	  if (lane_over <= 2) {
	    DEBUG(printf("   Filling lane %u from %d to %d with NO-OBSTACLE\n", check_lane, 0, OCC_NEXT_LANE_NEAR_GRID[abs_lane_diff]));
	    for (int yi = 0; yi < OCC_NEXT_LANE_NEAR_GRID[abs_lane_diff]; yi++) {
	      local_occ_grid[check_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
	    }
	  }		
	  int grid_min = OCC_NEXT_LANE_FAR[abs_lane_diff] / GRID_DIST_STEP_SIZE;
	  DEBUG(printf("   Filling lane %u from %d to %d with NO-OBSTACLE\n", check_lane, grid_min, OCC_GRID_Y_DIM-1));
	  for (int yi = grid_min; yi < OCC_GRID_Y_DIM; yi++) {
	    local_occ_grid[check_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
	  }
	  //DEBUG(printf("     So lane %u dist %u = %u\n", check_lane, OCC_GRID_Y_DIM-1, local_occ_grid[check_lane][OCC_GRID_Y_DIM-1]));
	} else {
	  bool found_close = false;
	  // Okay, so we have some obstacle in the lane...
	  while (i >= 0) { // While we still have un-considered objects...
	    unsigned dist = lane_dist[check_lane][i];
	    // first check whether the obstacle is in the "close" distance...
	    if (dist <= OCC_NEXT_LANE_NEAR[abs_lane_diff]) {
	      if (found_close) {
		DEBUG(printf(" we already found a NEAR object in lane %u %s\n", check_lane, lane_names[check_lane]));
	      } else {
		int grid_dist = (int)(lane_dist[check_lane][i] / GRID_DIST_STEP_SIZE);
		DEBUG(printf("  ==> FOUND %c Lane-Over %d Lane %u %s NEAR : dist %u gd %u\n", lane_obj[check_lane][i], lane_over, check_lane, lane_names[check_lane], dist, grid_dist));
		DEBUG(printf("   Filling lane %u from %d to %d with NO-OBSTACLE\n", check_lane, 0, grid_dist-1));
		for (int yi = 0; yi < grid_dist; yi++) {
		  local_occ_grid[check_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
		}
		DEBUG(printf("   Filling lane %u %d with OBSTACLE\n", check_lane, grid_dist));
		local_occ_grid[check_lane][grid_dist] = OCC_GRID_OBSTACLE_VAL;
		found_close = true;
	      } 
	    } else if ((dist <= MAX_DISTANCE) && (dist >= OCC_NEXT_LANE_FAR[abs_lane_diff])) {
	      int grid_dist = (int)(lane_dist[check_lane][i] / GRID_DIST_STEP_SIZE);
	      DEBUG(printf("  ==> FOUND %c Lane-Over %d Lane %u %s FAR : dist %u gd %u\n", lane_obj[check_lane][i], lane_over, check_lane, lane_names[check_lane], dist, grid_dist));
	      int grid_min = OCC_NEXT_LANE_FAR[abs_lane_diff] / GRID_DIST_STEP_SIZE;
	      DEBUG(printf("   Filling lane %u from %d to %d with NO-OBSTACLE\n", check_lane, grid_min, grid_dist-1));
	      for (int yi = grid_min; yi < grid_dist; yi++) {
		local_occ_grid[check_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
	      }
	      DEBUG(printf("   Filling lane %u %d with OBSTACLE\n", check_lane, grid_dist));
	      local_occ_grid[check_lane][grid_dist] = OCC_GRID_OBSTACLE_VAL;
	    }
	    i--;
	  } // while (i >=- 0)
	  if ((!found_close) && (lane_over <= 2)) {
	    // Need to fill in the "close" space with NO_OBSTACLE
	    for (int yi = 0; yi < OCC_NEXT_LANE_NEAR_GRID[abs_lane_diff]; yi++) {
	      local_occ_grid[check_lane][yi] = OCC_GRID_NO_OBSTACLE_VAL;
	    }
	  }		
	} // else of if (nearest obj == 'N')
      } // if (check_lane >=0) && (check_lane != my_lane)
    } // for (lanex_over from [-2 to 2]
    
    DEBUG(printf("Setting MY_CAR in Lane %u from %u to %u\n", my_lane, 0, MY_CAR_OCC_GRID_SIZE));
    for (int i = 0; i < MY_CAR_OCC_GRID_SIZE; i++) {
      local_occ_grid[my_lane][i] = OCC_GRID_MY_CAR_VAL;
    }
    DEBUG(printf("Done forming local-occ-grid...\n\n"));
    if (show_local_occ_grid) {
      print_local_occupancy_grid();
    }
  }

 /* #undef DEBUG */
 /* #define DEBUG(x) */
  
  // Send the base-line (my local) Occupancy Map information to XMIT socket.
  // Connect to the Wifi-Socket and send the n_xmit_out
  char w_buffer[10];
 #ifdef INT_TIME
  gettimeofday(&start_wifi_send, NULL);
 #endif
  {
    DEBUG(printf("Preparing to send WIFI message:\n"));
    wifi_out_msg.car_state = vs;
    int gi = 0;
    for (int xi = 0; xi < OCC_GRID_X_DIM; xi++) {
      for (int yi = 0; yi < OCC_GRID_Y_DIM; yi++) {
	wifi_out_msg.raw_occ_grid[gi] = local_occ_grid[xi][yi];
	gi++;
      }
    }
    DEBUG(printf("Total length is %lu\n", sizeof(xmit_msg_t)));
    viterbi_messages_histogram[sizeof(xmit_msg_t)]++;
  }
  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });


  unsigned xfer_bytes = sizeof(xmit_msg_t);
  snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
  DEBUG(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
  send(xmit_sock, w_buffer, 8, 0);
  DEBUG(printf("     Send %lu values %u bytes on XMIT port %u socket\n", sizeof(xmit_msg_t), xfer_bytes, XMIT_PORT));
  {
    char isACK[3];
    int ackread = read_all(xmit_sock, isACK, 2);
    if ((isACK[0] != ack[0]) || (isACK[1] != ack[1])) {
      printf("ERROR : Failed to get ACK from receiver after XMIT %u\n", xmit_count);
      exit(-2);
    } else {
      DEBUG(printf(" GOT a header %u ACK from the XMIT target\n", xmit_count));
    }
  }
  
 #ifdef INT_TIME
  gettimeofday(&start_wifi_send, NULL);
 #endif
  DEBUG(printf("Calling send for the XMIT %u data-body...\n", xmit_count));
  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
  send(xmit_sock, (char*)(&wifi_out_msg), sizeof(xmit_msg_t), 0);
  {
    char isACK[3];
    int ackread = read_all(xmit_sock, isACK, 2);
    if ((isACK[0] != ack[0]) || (isACK[1] != ack[1])) {
      printf("ERROR : Failed to get ACK from receiver after XMIT %u\n", xmit_count);
      exit(-2);
    } else {
      DEBUG(printf(" GOT a data body %u ACK from the XMIT target\n", xmit_count));
    }
  }
 #ifdef INT_TIME
  gettimeofday(&stop_wifi_send, NULL);
  wifi_send_sec   += stop_wifi_send.tv_sec  - start_wifi_send.tv_sec;
  wifi_send_usec  += stop_wifi_send.tv_usec - start_wifi_send.tv_usec;
  wifi_send_sec   += stop_wifi_send.tv_sec  - start_wifi_send.tv_sec;
  wifi_send_usec  += stop_wifi_send.tv_usec - start_wifi_send.tv_usec;
 #endif
  /*DEBUG(printf("XFER %4u : Dumping XMIT-PIPE bytes\n", xmit_count);
	for (int i = 0; i < wifi_out_msg_len; i++) {
	  if ((i % 56) == 0) { printf("\n"); }
	  if ((i % 7) == 0) { printf(" "); }
	  printf("%u", wifi_out_msg[i]);
	}
	printf("\n"));*/
  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
  xmit_count++;
  
  // Now receive the other car's Wifi data (input Viterbi Message)
 #ifdef INT_TIME
  gettimeofday(&start_wifi_recv_wait, NULL);
 #endif

  int n_recvd_in;
  DEBUG(printf("\nNow calling read_all to receive the remote-input header...\n"));
  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
  char r_buffer[10] = "\0\0\0\0\0\0\0\0\0\0";
  int valread = read_all(recv_sock, r_buffer, 8);
  DEBUG(printf("  RECV done\n"));
  DEBUG(printf("  RECV got %d bytes\n", valread));
  DEBUG(printf("  RECV got %d bytes :'%s'\n", valread, r_buffer));
  SDEBUG(printf("  RECV msg psn %s\n", "01234567890"));
  if (valread != 8) {
    printf(" ERROR init_vit_kernel RECV got a wrong-sized length indicator message\n");
    exit(-6);
  }
 #ifdef INT_TIME
  gettimeofday(&start_wifi_recv_all, NULL);
 #endif
  if(!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) {
    printf("ERROR: Unexpected message from WiFi...\n");
    exit(-3);
  }
  send(recv_sock, ack, 2, 0);

  char * ptr;
  unsigned xfer_in_bytes = strtol(r_buffer+1, &ptr, 10);

  n_recvd_in = xfer_in_bytes / sizeof(uint8_t);
  DEBUG(printf("     Recv %u UINT8_T values %u bytes from RECV port %u socket\n", n_recvd_in, xfer_in_bytes, RECV_PORT));
  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });
 #ifdef INT_TIME
  gettimeofday(&start_wifi_recv, NULL);
 #endif	
  valread = read_all(recv_sock, (char*)recvd_in, xfer_in_bytes);
  if (valread < xfer_in_bytes) {
    if (valread == 0) {
      printf("  RECV REAL got ZERO bytes -- END of TRANSFER?\n");
      exit(-1);
    } else {
      printf("  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
      exit(-1);
    }
  }
  SDEBUG(printf("XFER %4u : Dumping %u RECV-PIPE bytes out of %u at %p\n", recv_count, 32, n_recvd_in, recvd_in);
	for (int i = 0; i < 32 /*n_recvd_in*/; i++) {
	  printf("  Byte %5u : 0x%02x\n", i, recvd_in[i]);
	});
  DEBUG(printf("XFER %4u : Dumping %u RECV-PIPE bytes out of %u\n", recv_count, 256, n_recvd_in);
	for (int i = 0; i < 256 /*n_recvd_in*/; i++) {
	  if ((i % 16) == 0) { printf("\n     "); }
	  printf("%02x ", recvd_in[i]);
	});
  send(recv_sock, ack, 2, 0);
  DO_INTERACTIVE({printf("** press a key **"); char c = getc(stdin); });

  
 #ifdef INT_TIME
  gettimeofday(&stop_wifi_recv, NULL);
  gettimeofday(&stop_wifi_recv_all, NULL);
  wifi_recv_wait_sec  += start_wifi_recv_all.tv_sec  - start_wifi_recv_wait.tv_sec;
  wifi_recv_wait_usec += start_wifi_recv_all.tv_usec - start_wifi_recv_wait.tv_usec;
  wifi_recv_all_sec   += stop_wifi_recv_all.tv_sec  - start_wifi_recv_all.tv_sec;
  wifi_recv_all_usec  += stop_wifi_recv_all.tv_usec - start_wifi_recv_all.tv_usec;
  wifi_recv_sec    += stop_wifi_recv.tv_sec  - start_wifi_recv.tv_sec;
  wifi_recv_usec   += stop_wifi_recv.tv_usec - start_wifi_recv.tv_usec;
 #endif
  recv_count++;

  vit_msg_struct_t* vit_msg_ptr = (struct vit_msg_struct*) recvd_in;
 
  DEBUG(printf("\nVIT_MSG_STRUCT\n");
	printf("  unsigned     msg_len   @ %p = %u\n",  &(vit_msg_ptr->len), vit_msg_ptr->len);   //recvd_in);
	printf("  ve_statete_t car_state @ %p = a %u l %u d %.1f s %.1f\n",  &(vit_msg_ptr->car_state), vit_msg_ptr->car_state.active , vit_msg_ptr->car_state.lane , vit_msg_ptr->car_state.distance , vit_msg_ptr->car_state.speed);
	printf("  ofdm_param*  ofdm_ptr  @ %p\n",  &(vit_msg_ptr->ofdm));  // (ofdm_param*)(recvd_in + sizeof(unsigned)));
	printf("  frame_param* frame_ptr @ %p\n",  &(vit_msg_ptr->frame)); // (frame_param*)(recvd_in + sizeof(unsigned) + sizeof(ofdm_param)));
	printf("  uint8_t*     data_ptr  @ %p\n",  &(vit_msg_ptr->msg)));  // (recvd_in + sizeof(unsigned) + sizeof(ofdm_param) + sizeof(frame_param)));
	
  temp_vit_dict_entry.msg_num = 0;
  temp_vit_dict_entry.msg_id  = 0;
  other_car = vit_msg_ptr->car_state;
  temp_vit_dict_entry.ofdm_p  = vit_msg_ptr->ofdm;
  temp_vit_dict_entry.frame_p = vit_msg_ptr->frame;
  temp_vit_dict_entry.in_bits = vit_msg_ptr->msg;

  SDEBUG(for (int tti = 0 ; tti < sizeof(struct vit_msg_struct); tti++) {
      printf("RECVD_IN %5u @ %p : 0x%02x\n", tti, &(recvd_in[tti]), recvd_in[tti]);
    });
  
  hist_total_objs[total_obj]++;
  return &temp_vit_dict_entry;
}


extern void start_decode(task_metadata_block_t* vit_metadata_block, ofdm_param *ofdm, frame_param *frame, uint8_t *in);
extern uint8_t* finish_decode(task_metadata_block_t* mb_ptr, int* n_dec_char);

void post_execute_vit_kernel(message_t tr_msg, message_t dec_msg)
{
  total_msgs++;
  if (dec_msg != tr_msg) {
    bad_decode_msgs++;
  }
}

#ifndef HPVM
// The Test Task is just a dummy task we can apply to any accelerator
//  during a run, with a fixed execution time...

status_t init_test_kernel(char* dict_fn)
{
  DEBUG(printf("In init_test_kernel...\n"));
  /* Read in the object images dictionary file
     FILE *dictF = fopen(dict_fn,"r");
     if (!dictF)
     {
     printf("Error: unable to open test-task dictionary definition file %s\n", dict_fn);
     return error;
     }

     fclose(dictF);
  */

  DEBUG(printf("DONE with init_test_kernel -- returning success\n"));
  return success;
}

test_dict_entry_t* iterate_test_kernel(vehicle_state_t vs)
{
  DEBUG(printf("In iterate_test_kernel in lane %u = %s\n", vs.lane, lane_names[vs.lane]));
  test_dict_entry_t* tde = NULL; // Currently we don't have a test-dictionary

  return tde;
}

void start_execution_of_test_kernel(task_metadata_block_t*  mb_ptr, test_dict_entry_t* trace_msg)
{
  DEBUG(printf("MB%u In start_execution_of_test_kernel\n", mb_ptr->block_id));
  request_execution(mb_ptr);
}

test_res_t finish_execution_of_test_kernel(task_metadata_block_t* mb_ptr)
{
  test_res_t tres = TEST_TASK_DONE;
  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u fin_test Calling free_task_metadata_block\n", mb_ptr->block_id));
  free_task_metadata_block(mb_ptr);
  
  DEBUG(printf("MB%u The finish_execution_of_test_kernel is returning tres %u\n", mb_ptr->block_id, tres));
  return tres;
}

void post_execute_test_kernel(test_res_t gold_res, test_res_t exec_res)
{
  total_test_tasks++;
  if (exec_res != gold_res) {
    bad_test_task_res++;
  }
}
#endif
/* #undef DEBUG */
/* #define DEBUG(x) x */



void closeout_cv_kernel()
{
  float label_correct_pctg = (100.0*label_match[NUM_OBJECTS])/(1.0*label_lookup[NUM_OBJECTS]);
  printf("\nFinal CV CNN Accuracy: %u correct labelings over %u classifications = %.2f%%\n", label_match[NUM_OBJECTS], label_lookup[NUM_OBJECTS], label_correct_pctg);
  for (int i = 0; i < NUM_OBJECTS; i++) {
    label_correct_pctg = (100.0*label_match[i])/(1.0*label_lookup[i]);
    printf("  CV CNN Accuracy for %10s : %u correct labelings of %u classifications = %.2f%%\n", object_names[i], label_match[i], label_lookup[i], label_correct_pctg);
  }

  unsigned errs = label_lookup[NUM_OBJECTS] - label_match[NUM_OBJECTS];
  if (errs > 0) {
    printf("\nAnalysis of the %u mis-identifications:\n", errs);
    for (int i = 0; i < NUM_OBJECTS; i++) {
      for (int j = 0; j < NUM_OBJECTS; j++) {
	if (label_mismatch[i][j] != 0) {
	  printf("  Mislabeled (real) %10s as %10s (CV) on %u occasions\n", object_names[i], object_names[j], label_mismatch[i][j]);
	}
      }
    }

    printf("\nAnalysis (dual-view) of the %u mis-identifications:\n", errs);
    for (int j = 0; j < NUM_OBJECTS; j++) {
      for (int i = 0; i < NUM_OBJECTS; i++) {
	if (label_mismatch[i][j] != 0) {
	  printf("  Mislabeled (real) %10s as %10s (CV) on %u occasions\n", object_names[i], object_names[j], label_mismatch[i][j]);
	}
      }
    }
  }
}

void closeout_radar_kernel()
{
  printf("\nHistogram of Radar Distances:\n");
  printf("    %3s | %3s | %8s | %9s \n", "Set", "Idx", "Distance", "Occurs");
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      printf("    %3u | %3u | %8.3f | %9u \n", si, di, the_radar_return_dict[si][di].distance, hist_distances[si][di]);
    }
  }

  printf("\nHistogram of Radar Distance Abs-Pct-Err:\n");
  unsigned totals[] = {0, 0, 0, 0, 0};

  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      printf("    Set %u Entry %u Id %u Distance %f Occurs %u Histogram:\n", si, di, the_radar_return_dict[si][di].index, the_radar_return_dict[si][di].distance, hist_distances[si][di]);
      for (int i = 0; i < 5; i++) {
	printf("    %7s | %9u \n", hist_pct_err_label[i], hist_pct_errs[si][di][i]);
	totals[i] += hist_pct_errs[si][di][i];
      }
    }
  }

  printf("\n  TOTALS Histogram of Radar Distance Abs-Pct-Err:\n");
  for (int i = 0; i < 5; i++) {
    printf("  %7s | %9u \n", hist_pct_err_label[i], totals[i]);
  }


  printf("\nHistogram of Radar Task Inputs Used:\n");
  printf("    %3s | %5s | %9s \n", "Set", "Entry", "NumOccurs");
  for (int si = 0; si < num_radar_samples_sets; si++) {
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      printf("    %3u | %3u | %9u \n", si, di, radar_inputs_histogram[si][di]);
    }
  }
  printf("\n");
}

void closeout_vit_kernel()
{
  // Nothing to do?

  printf("\nHistogram of Total Objects:\n");
  unsigned sum = 0;
  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    if (hist_total_objs[i] != 0) {
      printf("%3u | %9u \n", i, hist_total_objs[i]);
      sum += i*hist_total_objs[i];
    }
  }
  double avg_objs = (1.0 * sum)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  printf("There were %.3lf obstacles per time step (average)\n", avg_objs);
  double avg_msgs = (1.0 * total_msgs)/(1.0 * radar_total_calc); // radar_total_calc == total time steps
  printf("There were %.3lf messages per time step (average)\n", avg_msgs);
  printf("There were %u bad decodes of the %u messages\n", bad_decode_msgs, total_msgs);

  printf("\nHistogram of Viterbi Messages:\n");
  printf("    %4s | %9s \n", "MLen", "NumOccurs");
  for (int li = 0; li < 1502; li++) {
    if (viterbi_messages_histogram[li] != 0) {
      printf("    %4u | %9u \n", li, viterbi_messages_histogram[li]);
    }
  }
  printf("\n");

  if (xmit_sock != 0) {
    close(xmit_sock);
  }
  if (recv_sock != 0) {
    close(recv_sock);
  }
}



void closeout_test_kernel()
{
  // Nothing to do?
  printf("\nThere were a total of %u Test-Tasks run, with %u bad Test-Task results\n", total_test_tasks, bad_test_task_res);
  printf("\n");
}

