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

#include "scheduler.h" // for cleanup_and_exit
#include "fft_task.h"
#include "vit_task.h"
#include "cv_task.h"
#include "test_task.h"
#include "plan_ctrl_task.h"

#include "kernels_api.h"

#ifdef USE_SIM_ENVIRON
 #include "sim_environs.h"
#else
 #include "read_trace.h"
#endif

extern unsigned time_step;
extern unsigned task_size_variability;

char* lane_names[NUM_LANES] = {"LHazard", "Left", "Center", "Right", "RHazard" };
char* message_names[NUM_MESSAGES] = {"Safe_L_or_R", "Safe_R_only", "Safe_L_only", "Unsafe_L_or_R" };
char* object_names[NUM_OBJECTS] = {"Nothing", "Car", "Truck", "Person", "Bike" };


#ifdef VERBOSE
bool output_viz_trace = true;
#else
bool output_viz_trace = false;
#endif

unsigned total_obj; // Total non-'N' obstacle objects across all lanes this time step
unsigned obj_in_lane[NUM_LANES]; // Number of obstacle objects in each lane this time step (at least one, 'n')
unsigned lane_dist[NUM_LANES][MAX_OBJ_IN_LANE]; // The distance to each obstacle object in each lane
char     lane_obj[NUM_LANES][MAX_OBJ_IN_LANE]; // The type of each obstacle object in each lane

char     nearest_obj[NUM_LANES]  = { 'N', 'N', 'N', 'N', 'N' };
float    nearest_dist[NUM_LANES] = { INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE, INF_DISTANCE };

unsigned hist_total_objs[NUM_LANES * MAX_OBJ_IN_LANE];

unsigned rand_seed = 0; // Only used if -r <N> option set

float IMPACT_DISTANCE = 50.0; // Minimum distance at which an obstacle "impacts" MyCar (collision case)


/* These are types, functions, etc. required for VITERBI */
//#include "viterbi_flat.h"




/* These are some top-level defines needed for CV kernel */
/* #define IMAGE_SIZE  32  // What size are these? */
/* typedef struct { */
/*   unsigned int image_id; */
/*   label_t  object; */
/*   unsigned image_data[IMAGE_SIZE]; */
/* } cv_dict_entry_t; */

/** The CV kernel uses a different method to select appropriate inputs; dictionary not needed
unsigned int     num_cv_dictionary_items = 0;
cv_dict_entry_t* the_cv_object_dict;
**/
unsigned label_match[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0};  // Times CNN matched dictionary
unsigned label_lookup[NUM_OBJECTS+1] = {0, 0, 0, 0, 0, 0}; // Times we used CNN for object classification
unsigned label_mismatch[NUM_OBJECTS][NUM_OBJECTS] = {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}};


/* These are some top-level defines needed for RADAR */
/* typedef struct { */
/*   unsigned int index; */
/*   unsigned int return_id; */
/*   unsigned int log_nsamples; */
/*   float distance; */
/*   float return_data[2 * MAX_RADAR_N]; */
/* } radar_dict_entry_t; */

#define MAX_RDICT_ENTRIES      12   // This should be updated eventually...
unsigned int         num_radar_samples_sets = 0;
unsigned int         radar_dict_items_per_set = 0;
radar_dict_entry_t** the_radar_return_dict;
unsigned int         radar_log_nsamples_per_dict_set[MAX_RDICT_SAMPLE_SETS];

unsigned radar_total_calc = 0;
unsigned hist_pct_errs[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES][5];// = {0, 0, 0, 0, 0}; // One per distance, plus global?
unsigned hist_distances[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES];
char*    hist_pct_err_label[5] = {"   0%", "<  1%", "< 10%", "<100%", ">100%"};
unsigned radar_inputs_histogram[MAX_RDICT_SAMPLE_SETS][MAX_RDICT_ENTRIES];

#define VITERBI_LENGTHS  4
unsigned viterbi_messages_histogram[VITERBI_LENGTHS][NUM_MESSAGES];

/* These are some top-level defines needed for VITERBI */
/* typedef struct { */
/*   unsigned int msg_num; */
/*   unsigned int msg_id; */
/*   ofdm_param   ofdm_p; */
/*   frame_param  frame_p; */
/*   uint8_t      in_bits[MAX_ENCODED_BITS]; */
/* } vit_dict_entry_t; */

uint8_t descramble[1600]; // I think this covers our max use cases
uint8_t actual_msg[1600];

unsigned int      num_viterbi_dictionary_items = 0;
vit_dict_entry_t* the_viterbi_trace_dict;

unsigned vit_msgs_size;
unsigned vit_msgs_per_step;
const char* vit_msgs_size_str[VITERBI_MSG_LENGTHS] = {"SHORT", "MEDIUM", "LONG", "MAXIMUM"};
const char* vit_msgs_per_step_str[VITERBI_MSGS_PER_STEP] = {"One message per time step",
							    "One message per obstacle per time step",
							    "One msg per obstacle + 1 per time step" };
unsigned viterbi_messages_histogram[VITERBI_MSG_LENGTHS][NUM_MESSAGES];

unsigned total_msgs = 0; // Total messages decoded during the full run
unsigned bad_decode_msgs = 0; // Total messages decoded incorrectly during the full run



unsigned total_test_tasks = 0;  // Total test tasks executed during the full run
unsigned bad_test_task_res = 0; // Total test task "bad-resutls" during the full run





status_t init_cv_kernel(scheduler_datastate_block_t* sptr, char* py_file, char* dict_fn)
{
  DEBUG(printf("In the init_cv_kernel routine\n"));
  /** The CV kernel uses a different method to select appropriate inputs; dictionary not needed **/
  // Initialization to run Keras CNN code
  return success;
}

label_t iterate_cv_kernel(scheduler_datastate_block_t* sptr, vehicle_state_t vs)
{
  DEBUG(printf("In iterate_cv_kernel\n"));

  unsigned tr_val = 0; // Default nothing
  switch(nearest_obj[vs.lane]) {
    case 'N' : tr_val = no_label; break;
    case 'B' : tr_val = bicycle; break;
    case 'C' : tr_val = car; break;
    case 'P' : tr_val = pedestrian; break;
    case 'T' : tr_val = truck; break;
  default: printf("ERROR : Unknown object type in cv trace: '%c'\n", nearest_obj[vs.lane]); cleanup_and_exit(sptr, -2);
  }
  label_t d_object = (label_t)tr_val;

  return d_object;
}

/*
void start_execution_of_cv_kernel(task_metadata_block_t* mb_ptr, label_t in_tr_val)
{
  DEBUG(printf("MB%u In start_execution_of_cv_kernel\n", mb_ptr->block_id));
  // 2) Set up to request object detection on an image frame *
  int tidx = mb_ptr->accelerator_type;
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(mb_ptr->task_timings[mb_ptr->task_type]);
  cv_data_struct_t * cv_data_p    = (cv_data_struct_t*)(mb_ptr->data_space);
  // Currently we don't send in any data this way (though we should include the input image here)
  // We will pre-set the result to match the trace input value (in case we "fake" the accelerator execution)
  cv_data_p->object_label = in_tr_val;
  //DEBUG(printf("CV Kernel: MB%u set object as %u from %u\n", mb_ptr->block_id, cv_data_p->object_label, in_tr_val));
 #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
 #endif
  //  schedule_task(data);
  request_execution(mb_ptr);
  // This now ends this block -- we've kicked off execution
}
*/
/*
label_t finish_execution_of_cv_kernel(task_metadata_block_t* mb_ptr)
{
  DEBUG(printf("MB%u In finish_execution_of_cv_kernel\n", mb_ptr->block_id));
  cv_data_struct_t * cv_data_p    = (cv_data_struct_t*)(mb_ptr->data_space);
  label_t the_label = cv_data_p->object_label;
  //DEBUG(printf("CV Kernel: Finish label for MB%u is %u\n", mb_ptr->block_id, cv_data_p->object_label));
  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u fin_cv Calling free_task_metadata_block\n", mb_ptr->block_id));
  free_task_metadata_block(mb_ptr);

  return the_label;
}
*/
void post_execute_cv_kernel(label_t tr_val, label_t cv_object)
{
  //printf("CV_POST: Compare %u to %u\n", tr_val, cv_object);
  if (cv_object == tr_val) {
    label_match[cv_object]++;
    label_match[NUM_OBJECTS]++;
  } else {
    label_mismatch[tr_val][cv_object]++;
  }
  label_lookup[NUM_OBJECTS]++;
  label_lookup[cv_object]++;
}



status_t init_radar_kernel(scheduler_datastate_block_t* sptr, char* dict_fn)
{
  DEBUG(printf("In init_radar_kernel...\n"));

  // Read in the radar distances dictionary file
  FILE *dictF = fopen(dict_fn,"r");
  if (!dictF)
  {
    printf("Error: unable to open dictionary file %s\n", dict_fn);
    fclose(dictF);
    return error;
  }
  // Read the number of definitions
  if (fscanf(dictF, "%u %u\n", &num_radar_samples_sets, &radar_dict_items_per_set) != 2) {
    printf("ERROR reading the number of Radar Dictionary sets and items per set\n");
    cleanup_and_exit(sptr, -2);
  }
  DEBUG(printf("  There are %u dictionary sets of %u entries each\n", num_radar_samples_sets, radar_dict_items_per_set));
  the_radar_return_dict = (radar_dict_entry_t**)calloc(num_radar_samples_sets, sizeof(radar_dict_entry_t*));
  if (the_radar_return_dict == NULL) {
    printf("ERROR : Cannot allocate Radar Trace Dictionary memory space\n");
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
    if (fscanf(dictF, "%u\n", &(radar_log_nsamples_per_dict_set[si])) != 1) {
      printf("ERROR reading the number of Radar Dictionary samples for set %u\n", si);
      cleanup_and_exit(sptr, -2);
    }
    DEBUG(printf("  Dictionary set %u entries should all have %u log_nsamples\n", si, radar_log_nsamples_per_dict_set[si]));
    for (int di = 0; di < radar_dict_items_per_set; di++) {
      unsigned entry_id;
      unsigned entry_log_nsamples;
      float entry_dist;
      unsigned entry_dict_values = 0;
      if (fscanf(dictF, "%u %u %f", &entry_id, &entry_log_nsamples, &entry_dist) != 3) {
	printf("ERROR reading Radar Dictionary set %u entry %u header\n", si, di);
	cleanup_and_exit(sptr, -2);
      }
      if (radar_log_nsamples_per_dict_set[si] != entry_log_nsamples) {
	printf("ERROR reading Radar Dictionary set %u entry %u header : Mismatch in log2 samples : %u vs %u\n", si, di, entry_log_nsamples, radar_log_nsamples_per_dict_set[si]);
	cleanup_and_exit(sptr, -2);
      }

      DEBUG(printf("  Reading rad dictionary set %u entry %u : %u %u %f\n", si, di, entry_id, entry_log_nsamples, entry_dist));
      the_radar_return_dict[si][di].index = tot_index++;  // Set, and increment total index
      the_radar_return_dict[si][di].set = si;
      the_radar_return_dict[si][di].index_in_set = di;
      the_radar_return_dict[si][di].return_id = entry_id;
      the_radar_return_dict[si][di].log_nsamples = entry_log_nsamples;
      the_radar_return_dict[si][di].distance =  entry_dist;
      for (int i = 0; i < 2*(1<<entry_log_nsamples); i++) {
	float fin;
	if (fscanf(dictF, "%f", &fin) != 1) {
	  printf("ERROR reading Radar Dictionary set %u entry %u data entries\n", si, di);
	  cleanup_and_exit(sptr, -2);
	}
	the_radar_return_dict[si][di].return_data[i] = fin;
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



radar_dict_entry_t* iterate_radar_kernel(scheduler_datastate_block_t* sptr, vehicle_state_t vs)
{
  DEBUG(printf("In iterate_radar_kernel\n"));
  unsigned tr_val = nearest_dist[vs.lane] / RADAR_BUCKET_DISTANCE;  // The proper message for this time step and car-lane
  radar_inputs_histogram[crit_fft_samples_set][tr_val]++;
  return &(the_radar_return_dict[crit_fft_samples_set][tr_val]);
}


/*void start_execution_of_radar_kernel(task_metadata_block_t* mb_ptr, uint32_t log_nsamples, float * inputs)
{
  DEBUG(printf("MB%u In start_execution_of_radar_kernel\n", mb_ptr->block_id));
  //DEBUG(printf("  MB%u Calling start_calculate_peak_dist_from_fmcw\n", mb_ptr->block_id));
  start_calculate_peak_dist_from_fmcw(mb_ptr, log_nsamples, inputs);
  }*/

 /*
distance_t finish_execution_of_radar_kernel(task_metadata_block_t* mb_ptr)
{
  DEBUG(printf("MB%u In finish_execution_of_radar_kernel\n", mb_ptr->block_id));
  DEBUG(printf("  MB%u Calling finalize_calculate_peak_dist_from_fmcw\n", mb_ptr->block_id));
  distance_t dist = finish_calculate_peak_dist_from_fmcw(mb_ptr);

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u fin_rad Calling free_task_metadata_block\n", mb_ptr->block_id));
  free_task_metadata_block(mb_ptr);

  DEBUG(printf("  MB%u Returning distance = %.1f\n", mb_ptr->block_id, dist));
  return dist;
  }*/


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

char wifi_msg[1500];
int wifi_msg_len;

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

status_t init_vit_kernel(scheduler_datastate_block_t* sptr, char* dict_fn)
{
  DEBUG(printf("In init_vit_kernel...\n"));
  //printf(" XMIT-PORT = %u and RECV-PORT = %u on IP %s\n", XMIT_PORT, RECV_PORT, wifi_inet_addr_str);
  snprintf(wifi_msg, 1500, "XMIT-PORT %u to RECV-PORT %u : This is a test message sent via WiFi.",  XMIT_PORT, RECV_PORT);
  wifi_msg_len = strlen(wifi_msg);
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

  if (vit_msgs_size >= VITERBI_LENGTHS) {
    printf("ERROR: Specified too large a vit_msgs_size (-v option): %u but max is %u\n", vit_msgs_size, VITERBI_LENGTHS);
    cleanup_and_exit(sptr, -1);
  }
  /***
   // Read in the object images dictionary file
   FILE *dictF = fopen(dict_fn,"r");
   if (!dictF)
   {
   printf("Error: unable to open viterbi dictionary definition file %s\n", dict_fn);
   return error;
   }

   // Read in the trace message dictionary from the trace file
   // Read the number of messages
   if (fscanf(dictF, "%u\n", &num_viterbi_dictionary_items) != 1) {
   printf("ERROR reading the number of Viterbi Dictionary items\n");
   cleanup_and_exit(sptr, -2);
   }
   DEBUG(printf("  There are %u dictionary entries\n", num_viterbi_dictionary_items));
   the_viterbi_trace_dict = (vit_dict_entry_t*)calloc(num_viterbi_dictionary_items, sizeof(vit_dict_entry_t));
   if (the_viterbi_trace_dict == NULL)
   {
   printf("ERROR : Cannot allocate Viterbi Trace Dictionary memory space\n");
   fclose(dictF);
   return error;
   }

   // Read in each dictionary item
   for (int i = 0; i < num_viterbi_dictionary_items; i++)
   {
   DEBUG(printf("  Reading vit dictionary entry %u\n", i)); //the_viterbi_trace_dict[i].msg_id));

   int mnum, mid;
   if (fscanf(dictF, "%d %d\n", &mnum, &mid) != 2) {
   printf("Error reading viterbi kernel dictionary enry %u header: Message_number and Message_id\n", i);
   cleanup_and_exit(sptr, -2);
   }
   DEBUG(printf(" V_MSG: num %d Id %d\n", mnum, mid));
   if (mnum != i) {
   printf("ERROR : Check Viterbi Dictionary : i = %d but Mnum = %d  (Mid = %d)\n", i, mnum, mid);
   cleanup_and_exit(sptr, -5);
   }
   the_viterbi_trace_dict[i].msg_num = mnum;
   the_viterbi_trace_dict[i].msg_id = mid;

   int in_bpsc, in_cbps, in_dbps, in_encoding, in_rate; // OFDM PARMS
   if (fscanf(dictF, "%d %d %d %d %d\n", &in_bpsc, &in_cbps, &in_dbps, &in_encoding, &in_rate) != 5) {
   printf("Error reading viterbi kernel dictionary entry %u bpsc, cbps, dbps, encoding and rate info\n", i);
   cleanup_and_exit(sptr, -2);
   }

   DEBUG(printf("  OFDM: %d %d %d %d %d\n", in_bpsc, in_cbps, in_dbps, in_encoding, in_rate));
   the_viterbi_trace_dict[i].ofdm_p.encoding   = in_encoding;
   the_viterbi_trace_dict[i].ofdm_p.n_bpsc     = in_bpsc;
   the_viterbi_trace_dict[i].ofdm_p.n_cbps     = in_cbps;
   the_viterbi_trace_dict[i].ofdm_p.n_dbps     = in_dbps;
   the_viterbi_trace_dict[i].ofdm_p.rate_field = in_rate;

   int in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits;
   if (fscanf(dictF, "%d %d %d %d %d\n", &in_pdsu_size, &in_sym, &in_pad, &in_encoded_bits, &in_data_bits) != 5) {
   printf("Error reading viterbi kernel dictionary entry %u psdu num_sym, pad, n_encoded_bits and n_data_bits\n", i);
   cleanup_and_exit(sptr, -2);
   }
   DEBUG(printf("  FRAME: %d %d %d %d %d\n", in_pdsu_size, in_sym, in_pad, in_encoded_bits, in_data_bits));
   the_viterbi_trace_dict[i].frame_p.psdu_size      = in_pdsu_size;
   the_viterbi_trace_dict[i].frame_p.n_sym          = in_sym;
   the_viterbi_trace_dict[i].frame_p.n_pad          = in_pad;
   the_viterbi_trace_dict[i].frame_p.n_encoded_bits = in_encoded_bits;
   the_viterbi_trace_dict[i].frame_p.n_data_bits    = in_data_bits;

   int num_in_bits = in_encoded_bits + 10; // strlen(str3)+10; //additional 10 values
   DEBUG(printf("  Reading %u in_bits\n", num_in_bits));
   for (int ci = 0; ci < num_in_bits; ci++) {
   unsigned c;
   if (fscanf(dictF, "%u ", &c) != 1) {
   printf("Error reading viterbi kernel dictionary entry %u data\n", i);
   cleanup_and_exit(sptr, -6);
   }
   #ifdef SUPER_VERBOSE
   printf("%u ", c);
   #endif
   the_viterbi_trace_dict[i].in_bits[ci] = (uint8_t)c;
   }
   DEBUG(printf("\n"));
   }
   fclose(dictF);
  
  //Clear the messages (injected) histogram
  for (int i = 0; i < VITERBI_LENGTHS; i++) {
    for (int j = 0; j < NUM_MESSAGES; j++) {
      viterbi_messages_histogram[i][j] = 0;
    }
  }

  for (int i = 0; i < NUM_LANES * MAX_OBJ_IN_LANE; i++) {
    hist_total_objs[i] = 0;
  }
  **/

  DEBUG(printf("DONE with init_vit_kernel -- returning success\n"));
  return success;
}

/* Each time-step of the trace, we read in the
 * trace values for the left, middle and right lanes
 * (i.e. which message if the autonomous car is in the
 *  left, middle or right lane).
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


uint8_t recvd_in[sizeof(vit_dict_entry_t) + MAX_ENCODED_BITS*sizeof(uint8_t) + 4096];
vit_dict_entry_t temp_vit_dict_entry;

vit_dict_entry_t* iterate_vit_kernel(scheduler_datastate_block_t* sptr, vehicle_state_t vs)
{
  DEBUG(printf("In iterate_vit_kernel in lane %u = %s\n", vs.lane, lane_names[vs.lane]));
  // Send the base-line (my local) Occupancy Map type information fir XMIT socket.
  // Connect to the Wifi-Socket and send the n_xmit_out
  char w_buffer[10];
 #ifdef INT_TIME
  gettimeofday(&start_wifi_send, NULL);
 #endif	
  unsigned xfer_bytes = wifi_msg_len * sizeof(char);
  snprintf(w_buffer, 9, "X%-6uX", xfer_bytes);
  DEBUG(printf("\nXMIT Sending %s on XMIT port %u socket\n", w_buffer, XMIT_PORT));
  send(xmit_sock, w_buffer, 8, 0);
  DEBUG(printf("     Send %u REAL values %u bytes on XMIT port %u socket\n", wifi_msg_len, xfer_bytes, XMIT_PORT));
 #ifdef INT_TIME
  gettimeofday(&start_wifi_send, NULL);
 #endif	
  send(xmit_sock, (char*)(wifi_msg),wifi_msg_len*sizeof(float), 0);
 #ifdef INT_TIME
  gettimeofday(&stop_wifi_send, NULL);
  wifi_send_sec   += stop_wifi_send.tv_sec  - start_wifi_send.tv_sec;
  wifi_send_usec  += stop_wifi_send.tv_usec - start_wifi_send.tv_usec;
  wifi_send_sec   += stop_wifi_send.tv_sec  - start_wifi_send.tv_sec;
  wifi_send_usec  += stop_wifi_send.tv_usec - start_wifi_send.tv_usec;
 #endif
  DEBUG(printf("     Send %u IMAG values %u bytes on XMIT port %u socket\n", wifi_msg_len, xfer_bytes, XMIT_PORT));
  DEBUG(printf("XFER %4u : Dumping XMIT-PIPE bytes\n    \"", xmit_count);
	  for (int i = 0; i < wifi_msg_len; i++) {
	    //printf("XFER %4u IMAG-byte %6u : %f\n", xmit_count, i, xmit_out_imag[i]);
	    printf("%c", wifi_msg[i]);
	  }
	  printf("\"\n"));
  xmit_count++;
  
  // Now receive the other car's Wifi data (input Viterbi Message)
 #ifdef INT_TIME
  gettimeofday(&start_wifi_recv_wait, NULL);
 #endif

  int n_recvd_in;

  char r_buffer[10];
  int valread = read_all(recv_sock, r_buffer, 8);
  DEBUG(printf("  RECV got %d bytes :'%s'\n", valread, r_buffer));
  SDEBUG(printf("  RECV msg psn %s\n", "01234567890"));
  if (valread != 8) {
    printf(" ERROR inti_vit_kernel RECV got a wrong-sized length indicator message\n");
    cleanup_and_exit(sptr, -6);
  }
 #ifdef INT_TIME
  gettimeofday(&start_wifi_recv_all, NULL);
 #endif
  if(!(r_buffer[0] == 'X' && r_buffer[7] == 'X')) {
    printf("ERROR: Unexpected message from WiFi...\n");
    cleanup_and_exit(sptr, -3);
  }
  send(recv_sock, ack, 2, 0);

  char * ptr;
  unsigned xfer_in_bytes = strtol(r_buffer+1, &ptr, 10);

  n_recvd_in = xfer_in_bytes / sizeof(uint8_t);
  DEBUG(printf("     Recv %u UINT8_T values %u bytes from RECV port %u socket\n", n_recvd_in, xfer_in_bytes, RECV_PORT));
 #ifdef INT_TIME
  gettimeofday(&start_wifi_recv, NULL);
 #endif	
  valread = read_all(recv_sock, (char*)recvd_in, xfer_in_bytes);
  if (valread < xfer_in_bytes) {
    if (valread == 0) {
      printf("  RECV REAL got ZERO bytes -- END of TRANSFER?\n");
      cleanup_and_exit(sptr, -1);
    } else {
      printf("  RECV REAL got TOO FEW bytes %u vs %u : INTERRUPTED TRANSFER?\n", valread, xfer_in_bytes);
      cleanup_and_exit(sptr, -1);
    }
  }
  DEBUG(printf("XFER %4u : Dumping %u RECV-PIPE bytes out of %u at %p\n", recv_count, 32, n_recvd_in, recvd_in);
	for (int i = 0; i < 32 /*n_recvd_in*/; i++) {
	  printf("  Byte %5u : 0x%02x\n", i, recvd_in[i]);
	});

  
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

  unsigned     msg_len   = *((unsigned*)recvd_in);
  ofdm_param*  ofdm_ptr  = (ofdm_param*)(recvd_in + sizeof(unsigned));
  frame_param* frame_ptr = (frame_param*)(recvd_in + sizeof(unsigned) + sizeof(ofdm_param));
  uint8_t*     data_ptr  = recvd_in +sizeof(unsigned) + sizeof(ofdm_param) + sizeof(frame_param);
  temp_vit_dict_entry.msg_num = 0;
  temp_vit_dict_entry.msg_id  = 0;
  temp_vit_dict_entry.ofdm_p  = *ofdm_ptr;
  temp_vit_dict_entry.frame_p = *frame_ptr;
  temp_vit_dict_entry.in_bits = data_ptr;

  /**
     hist_total_objs[total_obj]++;
     unsigned tr_val = 0; // set a default to avoid compiler messages
     switch (vs.lane) {
     case lhazard:
     {
     unsigned nd_1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
     DEBUG(printf("  Lane %u : obj in %u is %c at %u\n", vs.lane, vs.lane+1, nearest_obj[vs.lane+1], nd_1));
     if ((nearest_obj[1] != 'N') && (nd_1 < VIT_CLEAR_THRESHOLD)) {
     // Some object is in the left lane within threshold distance
     tr_val = 3; // Unsafe to move from lhazard lane into the left lane
     } else {
     tr_val = 1;
     }
     }
     break;
     case left:
     case center:
     case right:
     {
     unsigned ndp1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane+1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
     unsigned ndm1 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[vs.lane-1] / RADAR_BUCKET_DISTANCE); // floor by bucket...
     tr_val = 0;
     DEBUG(printf("  Lane %u : obj in %u is %c at %.1f : obj in %u is %c at %.1f\n", vs.lane,
     vs.lane-1, nearest_obj[vs.lane-1], nearest_dist[vs.lane-1],
     vs.lane+1, nearest_obj[vs.lane+1], nearest_dist[vs.lane+1]));
     if ((nearest_obj[vs.lane-1] != 'N') && (ndm1 < VIT_CLEAR_THRESHOLD)) {
     // Some object is in the Left lane at distance 0 or 1
     DEBUG(printf("    Marking unsafe to move left\n"));
     tr_val += 1; // Unsafe to move from this lane to the left.
     }
     if ((nearest_obj[vs.lane+1] != 'N') && (ndp1 < VIT_CLEAR_THRESHOLD)) {
     // Some object is in the Right lane at distance 0 or 1
     DEBUG(printf("    Marking unsafe to move right\n"));
     tr_val += 2; // Unsafe to move from this lane to the right.
     }
     }
     break;
     case rhazard:
     {
     unsigned nd_3 = RADAR_BUCKET_DISTANCE * (unsigned)(nearest_dist[3] / RADAR_BUCKET_DISTANCE); // floor by bucket...
     DEBUG(printf("  Lane %u : obj in %u is %c at %u\n", vs.lane, vs.lane-1, nearest_obj[vs.lane-1], nd_3));
     if ((nearest_obj[3] != 'N') && (nd_3 < VIT_CLEAR_THRESHOLD)) {
     // Some object is in the right lane within threshold distance
     tr_val = 3; // Unsafe to move from center lane to the right.
     } else {
     tr_val = 2;
     }
     }
     break;
     }

     DEBUG(printf("Viterbi final message for lane %u %s = %u\n", vs.lane, lane_names[vs.lane], tr_val));

     vit_dict_entry_t* trace_msg; // Will hold msg input data for decode, based on trace input

     // Here we determine short or long messages, based on global vit_msgs_size; offset is into the Dictionary
     int msg_offset = vit_msgs_size * NUM_MESSAGES; // 0 = short messages, 4 = long messages

     viterbi_messages_histogram[vit_msgs_size][tr_val]++;
     switch(tr_val) {
     case 0: // safe_to_move_right_or_left
     trace_msg = &(the_viterbi_trace_dict[0 + msg_offset]);
     break;
     case 1: // safe_to_move_right
     trace_msg = &(the_viterbi_trace_dict[1 + msg_offset]);
     break;
     case 2: // safe_to_move_left
     trace_msg = &(the_viterbi_trace_dict[2 + msg_offset]);
     break;
     case 3: // unsafe_to_move_left_or_right
     trace_msg = &(the_viterbi_trace_dict[3 + msg_offset]);
     break;
     }
     DEBUG(printf(" VIT: Using msg %u Id %u : %s \n", trace_msg->msg_num, trace_msg->msg_id, message_names[trace_msg->msg_id]));
     return trace_msg;
  **/
  return &temp_vit_dict_entry;
}

// These routines are used to select a random (non-critical?) Viterbi message input
//  to support variable message sizes per iteration
vit_dict_entry_t* select_specific_vit_input(int l_num, int m_num)
{
  viterbi_messages_histogram[l_num][m_num]++;
  return&(the_viterbi_trace_dict[NUM_MESSAGES*l_num + m_num]);
}

vit_dict_entry_t* select_random_vit_input()
{
  int l_num = (rand() % (VITERBI_LENGTHS)); // Return a value from [0,VITERBI_LENGTHS)
  int m_num = (rand() % (NUM_MESSAGES)); // Return a value from [0,NUM_MESSAGES)
  viterbi_messages_histogram[l_num][m_num]++;
  return&(the_viterbi_trace_dict[NUM_MESSAGES*l_num + m_num]);
}

extern void start_decode(task_metadata_block_t* vit_metadata_block, ofdm_param *ofdm, frame_param *frame, uint8_t *in);
extern uint8_t* finish_decode(task_metadata_block_t* mb_ptr, int* n_dec_char);

/*void start_execution_of_vit_kernel(task_metadata_block_t*  mb_ptr, vit_dict_entry_t* trace_msg)
{
  DEBUG(printf("MB%u In start_execution_of_vit_kernel\n", mb_ptr->block_id));
  // Send each message (here they are all the same) through the viterbi decoder
  //DEBUG(printf("  MB%u Calling the viterbi decode routine for message %u\n", mb_ptr->block_id, trace_msg->msg_num));
  start_decode(mb_ptr, &(trace_msg->ofdm_p), &(trace_msg->frame_p), &(trace_msg->in_bits[0]));
  }
*/
/*
message_t finish_execution_of_vit_kernel(task_metadata_block_t* mb_ptr)
{
  // Send each message (here they are all the same) through the viterbi decoder
  message_t msg = num_message_t;
  uint8_t *result;
  char     msg_text[1600]; // Big enough to hold largest message (1500?)

  int psdusize; // set by finish_decode call...
  DEBUG(printf("  MB%u Calling the finish_decode routine\n", mb_ptr->block_id));
  result = finish_decode(mb_ptr, &psdusize);
  // descramble the output - put it in result
  DEBUG(printf("  MB%u Calling the viterbi descrambler routine; psdusize = %u\n", mb_ptr->block_id, psdusize));
  descrambler(result, psdusize, msg_text, NULL /*descram_ref*, NULL /*msg*);

 #if(0)
  printf("MB%u PSDU %u : Msg : = `", mb_ptr->block_id, psdusize);
  for (int ci = 0; ci < (psdusize - 26); ci++) {
    printf("%c", msg_text[ci]);
  }
  printf("'\n");
 #endif
  // Here we look at the message string and select proper message_t
  switch(msg_text[3]) {
  case '0' : msg = safe_to_move_right_or_left; break;
  case '1' : msg = safe_to_move_right_only; break;
  case '2' : msg = safe_to_move_left_only; break;
  case '3' : msg = unsafe_to_move_left_or_right; break;
  default  : msg = num_message_t; break;
  }

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u fin_vit Calling free_task_metadata_block\n", mb_ptr->block_id));
  free_task_metadata_block(mb_ptr);
  
  DEBUG(printf("MB%u The execute_vit_kernel is returning msg %u\n", mb_ptr->block_id, msg));
  return msg;
}
*/
void post_execute_vit_kernel(message_t tr_msg, message_t dec_msg)
{
  total_msgs++;
  if (dec_msg != tr_msg) {
    bad_decode_msgs++;
  }
}


// The Test Task is just a dummy task we can apply to any accelerator
//  during a run, with a fixed execution time...

status_t init_test_kernel(scheduler_datastate_block_t* sptr, char* dict_fn)
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

test_dict_entry_t* iterate_test_kernel(scheduler_datastate_block_t* sptr, vehicle_state_t vs)
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

/* #undef DEBUG */
/* #define DEBUG(x) x */

vehicle_state_t plan_and_control(label_t label, distance_t distance, message_t message, vehicle_state_t vehicle_state)
{
  printf("In the plan_and_control routine...\n");
  DEBUG(printf("In the plan_and_control routine : label %u %s distance %.1f (T1 %.1f T1 %.1f T3 %.1f) message %u\n", 
	       label, object_names[label], distance, THRESHOLD_1, THRESHOLD_2, THRESHOLD_3, message));
  vehicle_state_t new_vehicle_state = vehicle_state;
  if (!vehicle_state.active) {
    // Our car is broken and burning, no plan-and-control possible.
    return vehicle_state;
  }
  
  if (//(label != no_label) && // For safety, assume every return is from SOMETHING we should not hit!
      ((distance <= THRESHOLD_1)
       #ifdef USE_SIM_ENVIRON
       || ((vehicle_state.speed < car_goal_speed) && (distance <= THRESHOLD_2))
       #endif
       )) {
    if (distance <= IMPACT_DISTANCE) {
      printf("WHOOPS: We've suffered a collision on time_step %u!\n", time_step);
      //fprintf(stderr, "WHOOPS: We've suffered a collision on time_step %u!\n", time_step);
      new_vehicle_state.speed = 0.0;
      new_vehicle_state.active = false; // We should add visualizer stuff for this!
      return new_vehicle_state;
    }
    
    // Some object ahead of us that needs to be avoided.
    DEBUG(printf("  In lane %s with %c (%u) at %.1f (trace: %.1f)\n", lane_names[vehicle_state.lane], nearest_obj[vehicle_state.lane], label, distance, nearest_dist[vehicle_state.lane]));
    switch (message) {
      case safe_to_move_right_or_left   :
	/* Bias is move right, UNLESS we are in the Right lane and would then head into the RHazard Lane */
	if (vehicle_state.lane < right) { 
	  DEBUG(printf("   In %s with Safe_L_or_R : Moving Right\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.lane += 1;
	} else {
	  DEBUG(printf("   In %s with Safe_L_or_R : Moving Left\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.lane -= 1;
	}	  
	break; // prefer right lane
      case safe_to_move_right_only      :
	DEBUG(printf("   In %s with Safe_R_only : Moving Right\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane += 1;
	break;
      case safe_to_move_left_only       :
	DEBUG(printf("   In %s with Safe_L_Only : Moving Left\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane -= 1;
	break;
      case unsafe_to_move_left_or_right :
	#ifdef USE_SIM_ENVIRON
	if (vehicle_state.speed > car_decel_rate) {
	  new_vehicle_state.speed = vehicle_state.speed - car_decel_rate; // was / 2.0;
	  DEBUG(printf("   In %s with No_Safe_Move -- SLOWING DOWN from %.2f to %.2f\n", lane_names[vehicle_state.lane], vehicle_state.speed, new_vehicle_state.speed));
	} else {
	  DEBUG(printf("   In %s with No_Safe_Move -- Going < 15.0 so STOPPING!\n", lane_names[vehicle_state.lane]));
	  new_vehicle_state.speed = 0.0;
	}
	#else
	DEBUG(printf("   In %s with No_Safe_Move : STOPPING\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.speed = 0.0;
	#endif
	break; /* Stop!!! */
    default:
      printf(" ERROR  In %s with UNDEFINED MESSAGE: %u\n", lane_names[vehicle_state.lane], message);
      //cleanup_and_exit(sptr, -6);
    }
  } else {
    // No obstacle-inspired lane change, so try now to occupy the center lane
    switch (vehicle_state.lane) {
    case lhazard:
    case left:
      if ((message == safe_to_move_right_or_left) ||
	  (message == safe_to_move_right_only)) {
	DEBUG(printf("  In %s with Can_move_Right: Moving Right\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane += 1;
      }
      break;
    case center:
      // No need to alter, already in the center
      break;
    case right:
    case rhazard:
      if ((message == safe_to_move_right_or_left) ||
	  (message == safe_to_move_left_only)) {
	DEBUG(printf("  In %s with Can_move_Left : Moving Left\n", lane_names[vehicle_state.lane]));
	new_vehicle_state.lane -= 1;
      }
      break;
    }
    #ifdef USE_SIM_ENVIRON
    if ((vehicle_state.speed < car_goal_speed) &&  // We are going slower than we want to, and
	//((label == no_label) ||      // There is no object ahead of us -- don't need; NOTHING is at INF_DISTANCE
	(distance >= THRESHOLD_2)) { // Any object is far enough away 
      if (vehicle_state.speed <= (car_goal_speed - car_accel_rate)) {
	new_vehicle_state.speed += 15.0;
      } else {
	new_vehicle_state.speed = car_goal_speed;
      }
      DEBUG(printf("  Going %.2f : slower than target speed %.2f : Speeding up to %.2f\n", vehicle_state.speed, 50.0, new_vehicle_state.speed));
    }
    #endif
  } // else clause


  return new_vehicle_state;
}
/* #undef DEBUG */
/* #define DEBUG(x) */


void closeout_cv_kernel()
{
  float label_correct_pctg = (100.0*label_match[NUM_OBJECTS])/(1.0*label_lookup[NUM_OBJECTS]);
  printf("\nFinal CV CNN Accuracy: %u correct of %u classifications = %.2f%%\n", label_match[NUM_OBJECTS], label_lookup[NUM_OBJECTS], label_correct_pctg);
  for (int i = 0; i < NUM_OBJECTS; i++) {
    label_correct_pctg = (100.0*label_match[i])/(1.0*label_lookup[i]);
    printf("  CV CNN Accuracy for %10s : %u correct of %u classifications = %.2f%%\n", object_names[i], label_match[i], label_lookup[i], label_correct_pctg);
  }

  unsigned errs = label_lookup[NUM_OBJECTS] - label_match[NUM_OBJECTS];
  if (errs > 0) {
    printf("\nAnalysis of the %u mis-identifications:\n", errs);
    for (int i = 0; i < NUM_OBJECTS; i++) {
      for (int j = 0; j < NUM_OBJECTS; j++) {
	if (label_mismatch[i][j] != 0) {
	  printf("  Mislabeled %10s as %10s on %u occasions\n", object_names[i], object_names[j], label_mismatch[i][j]);
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
  printf("    %3s | %3s | %9s \n", "Len", "Msg", "NumOccurs");
  for (int li = 0; li < VITERBI_LENGTHS; li++) {
    for (int mi = 0; mi < NUM_MESSAGES; mi++) {
      printf("    %3u | %3u | %9u \n", li, mi, viterbi_messages_histogram[li][mi]);
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







void start_execution_of_plan_ctrl_kernel(task_metadata_block_t* mb_ptr)
{
  int tidx = mb_ptr->accelerator_type;
  plan_ctrl_timing_data_t * plan_ctrl_timings_p = (plan_ctrl_timing_data_t*)&(mb_ptr->task_timings[mb_ptr->task_type]);
  // Data is set up prior to call here...
  //plan_ctrl_data_struct_t * plan_ctrl_data_p    = (plan_ctrl_data_struct_t*)(mb_ptr->data_space);
 #ifdef INT_TIME
  gettimeofday(&(plan_ctrl_timings_p->call_start), NULL);
 #endif
  request_execution(mb_ptr);
  // This now ends this block -- we've kicked off execution
}

vehicle_state_t finish_execution_of_plan_ctrl_kernel(task_metadata_block_t* mb_ptr)
{
  DEBUG(printf("In finish_execution_of_plan_ctrl_kernel\n"));
  plan_ctrl_data_struct_t * plan_ctrl_data_p    = (plan_ctrl_data_struct_t*)(mb_ptr->data_space);
  vehicle_state_t vehicle_state = plan_ctrl_data_p->new_vehicle_state;
  DEBUG(printf("finish PnC-Task : Vehicle State: Active %u Lane %u Speed %.1f\n", vehicle_state.active, vehicle_state.lane, vehicle_state.speed));
  // We've finished the execution and lifetime for this task; free its metadata
  free_task_metadata_block(mb_ptr);

  return vehicle_state;
}

