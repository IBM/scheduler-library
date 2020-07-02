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
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>

#include "verbose.h"

#include "scheduler.h"
#include "kernels_api.h"
#include "sim_environs.h"
#include "getopt.h"

#define TIME


char h264_dict[256]; 
char cv_dict[256]; 
char rad_dict[256];
char vit_dict[256];

bool_t bypass_h264_functions = false; // This is a global-disable of executing H264 execution functions...

//TODO: profile all possible fft and decoder sizes
//Dummy numbers for development
float fft_profile[NUM_ACCEL_TYPES] = {20, 0.2, 0, 0};
float vit_profile[NUM_ACCEL_TYPES] = {3, 0, 0.03, 0};

bool_t all_obstacle_lanes_mode = false;
unsigned time_step;
unsigned pandc_repeat_factor = 1;
unsigned task_size_variability;

void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h         : print this helpful usage info\n");
  printf("    -o         : print the Visualizer output traace information during the run\n");
  printf("    -R <file>  : defines the input Radar dictionary file <file> to use\n");
  printf("    -V <file>  : defines the input Viterbi dictionary file <file> to use\n");
  printf("    -C <file>  : defines the input CV/CNN dictionary file <file> to use\n");
  //printf("    -H <file>  : defines the input H264 dictionary file <file> to use\n");
  //printf("    -b         : Bypass (do not use) the H264 functions in this run.\n");
#ifdef USE_SIM_ENVIRON
  printf("    -s <N>     : Sets the max number of time steps to simulate\n");
  printf("    -r <N>     : Sets the rand random number seed to N\n");
  printf("    -A         : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)\n");
  printf("    -W <wfile> : defines the world environment parameters description file <wfile> to use\n");
#else
  printf("    -t <trace> : defines the input trace file <trace> to use\n");
#endif
  printf("    -p <N>     : defines the plan-and-control repeat factor (calls per time step -- default is 1)\n");
  printf("    -f <N>     : defines which Radar Dictionary Set is used for Critical FFT Tasks\n");
  printf("               :      Each Set of Radar Dictionary Entries Can use a different sample size, etc.\n");
  
  printf("    -F <N>     : Adds <N> additional (non-critical) FFT tasks per time step.\n");
  printf("    -v <N>     : defines Viterbi message size:\n");
  printf("               :      0 = Short messages (4 characters)\n");
  printf("               :      1 = Medium messages (500 characters)\n");
  printf("               :      2 = Long messages (1000 characters)\n");
  printf("               :      3 = Max-sized messages (1500 characters)\n");
  printf("    -M <N>     : Adds <N> additional (non-critical) Viterbi message tasks per time step.\n");
  printf("    -S <N>     : Task-Size Variability: Varies the sizes of input tasks where appropriate\n");
  printf("               :      0 = No variability (e.g. all messages same size, etc.)\n");
  printf("    -P <N>     : defines the Scheduler Accelerator Selection Policy:\n");
  printf("               :      0 = Select_Accelerator_Type_And_Wait\n");
  printf("               :      1 = Fastest_to_Slowest_First_Available\n");
  printf("               :      2 = Fastest_Finish_Time_First\n");
}


// This is just a call-through to the scheduler routine, but we can also print a message here...
//  This SHOULD be a routine that "does the right work" for a given task, and then releases the MetaData Block
void base_release_metadata_block(task_metadata_block_t* mb)
{
  TDEBUG(printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n", mb->block_id, task_job_str[mb->job_type], task_criticality_str[mb->crit_level], accel_type_str[mb->accelerator_type], mb->accelerator_id));
  free_task_metadata_block(mb);
  // Thread is done -- We shouldn't need to do anything else -- when it returns from its starting function it should exit.
}


	 
int main(int argc, char *argv[])
{
  vehicle_state_t vehicle_state;
  label_t label;
  distance_t distance;
  message_t message;
#ifdef USE_SIM_ENVIRON
  char* world_desc_file_name = "default_world.desc";
#else
  char* trace_file = "";
#endif
  int opt;

  rad_dict[0] = '\0';
  vit_dict[0] = '\0';
  cv_dict[0]  = '\0';
  h264_dict[0]  = '\0';

  unsigned additional_fft_tasks_per_time_step = 0;
  unsigned additional_vit_tasks_per_time_step = 0;

  //printf("SIZEOF pthread_t : %lu\n", sizeof(pthread_t));
  
  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  while((opt = getopt(argc, argv, ":hAbot:v:s:r:W:R:V:C:H:f:p:F:M:P:S:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 'A':
      all_obstacle_lanes_mode = true;
      break;
    case 'o':
      output_viz_trace = true;
      break;
    case 'R':
      snprintf(rad_dict, 255, "%s", optarg);
      break;
    case 'C':
      snprintf(cv_dict, 255, "%s", optarg);
      break;
    case 'H':
      snprintf(h264_dict, 255, "%s", optarg);
      break;
    case 'V':
      snprintf(vit_dict, 255, "%s", optarg);
      break;
    case 'b':
      bypass_h264_functions = true;
      break;
    case 's':
#ifdef USE_SIM_ENVIRON
      max_time_steps = atoi(optarg);
      printf("Using %u maximum time steps (simulation)\n", max_time_steps);
#endif
      break;
    case 'p':
      pandc_repeat_factor = atoi(optarg);
      printf("Using Plan-Adn-Control repeat factor %u\n", pandc_repeat_factor);
      break;
    case 'f':
      crit_fft_samples_set = atoi(optarg);
      printf("Using Radar Dictionary samples set %u for the critical FFT tasks\n", crit_fft_samples_set);
      break;
    case 'r':
#ifdef USE_SIM_ENVIRON
      rand_seed = atoi(optarg);
#endif
      break;
    case 't':
#ifndef USE_SIM_ENVIRON
      trace_file = optarg;
      printf("Using trace file: %s\n", trace_file);
#endif
      break;
    case 'v':
      vit_msgs_size = atoi(optarg);
      if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
	printf("ERROR: Specified viterbi message size (%u) is larger than max (%u) : from the -v option\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
	exit(-1);
      } else {
	printf("Using viterbi message size %u = %s\n", vit_msgs_size, vit_msgs_size_str[vit_msgs_size]);
      }
      break;
    case 'S':
      task_size_variability = atoi(optarg);
      printf("Using task-size variability behavior %u\n", task_size_variability);
      break;
    case 'W':
#ifdef USE_SIM_ENVIRON
      world_desc_file_name = optarg;
      printf("Using world description file: %s\n", world_desc_file_name);
#endif
      break;
    case 'F':
      additional_fft_tasks_per_time_step = atoi(optarg);
      break;
    case 'M':
      additional_vit_tasks_per_time_step = atoi(optarg);
      break;
    case 'P':
      global_scheduler_selection_policy = atoi(optarg);
      printf("Opting for Scheduler Policy %u\n", global_scheduler_selection_policy);
      break;

    case ':':
      printf("option needs a value\n");
      break;
    case '?':
      printf("unknown option: %c\n", optopt);
    break;
    }
  }

  // optind is for the extra arguments
  // which are not parsed
  for(; optind < argc; optind++){
    printf("extra arguments: %s\n", argv[optind]);
  }

  if (pandc_repeat_factor == 0) {
    printf("ERROR - Pland-and-Control repeat factor must be >= 1 : %u specified (with '-p' option)\n", pandc_repeat_factor);
    exit(-1);
  }

  if (rad_dict[0] == '\0') {
    sprintf(rad_dict, "traces/norm_radar_16k_dictionary.dfn");
  }
  if (vit_dict[0] == '\0') {
    sprintf(vit_dict, "traces/vit_dictionary.dfn");
  }
  if (cv_dict[0] == '\0') {
    sprintf(cv_dict, "traces/objects_dictionary.dfn");
  }
  if (h264_dict[0] == '\0') {
    sprintf(h264_dict, "traces/h264_dictionary.dfn");
  }

  printf("\nDictionaries:\n");
  printf("   CV/CNN : %s\n", cv_dict);
  printf("   Radar  : %s\n", rad_dict);
  printf("   Viterbi: %s\n", vit_dict);

  printf("\n There are %u additional FFT and %u addtional Viterbi tasks per time step\n", additional_fft_tasks_per_time_step, additional_vit_tasks_per_time_step);
  
  /* We plan to use three separate trace files to drive the three different kernels
   * that are part of mini-ERA (CV, radar, Viterbi). All these three trace files
   * are required to have the same base name, using the file extension to indicate
   * which kernel the trace corresponds to (cv, rad, vit).
   */
  /* if (argc != 2) */
  /* { */
  /*   printf("Usage: %s <trace_basename>\n\n", argv[0]); */
  /*   printf("Where <trace_basename> is the basename of the trace files to load:\n"); */
  /*   printf("  <trace_basename>.cv  : trace to feed the computer vision kernel\n"); */
  /*   printf("  <trace_basename>.rad : trace to feed the radar (FFT-1D) kernel\n"); */
  /*   printf("  <trace_basename>.vit : trace to feed the Viterbi decoding kernel\n"); */

  /*   return 1; */
  /* } */


  char cv_py_file[] = "../cv/keras_cnn/lenet.py";

  printf("Doing initialization tasks...\n");
  initialize_scheduler();

#ifndef USE_SIM_ENVIRON
  /* Trace filename construction */
  /* char * trace_file = argv[1]; */
  //printf("Input trace file: %s\n", trace_file);

  /* Trace Reader initialization */
  if (!init_trace_reader(trace_file))
  {
    printf("Error: the trace reader couldn't be initialized properly.\n");
    return 1;
  }
#endif

  /* Kernels initialization */
  /**if (bypass_h264_functions) {
    printf("Bypassing the H264 Functionality in this run...\n");
  } else {
    printf("Initializing the H264 kernel...\n");
    if (!init_h264_kernel(h264_dict))
      {
	printf("Error: the H264 decoding kernel couldn't be initialized properly.\n");
	return 1;
      }
      }**/

  printf("Initializing the CV kernel...\n");
  if (!init_cv_kernel(cv_py_file, cv_dict))
  {
    printf("Error: the computer vision kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Radar kernel...\n");
  if (!init_rad_kernel(rad_dict))
  {
    printf("Error: the radar kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Viterbi kernel...\n");
  if (!init_vit_kernel(vit_dict))
  {
    printf("Error: the Viterbi decoding kernel couldn't be initialized properly.\n");
    return 1;
  }

  if (crit_fft_samples_set >= num_radar_samples_sets) {
    printf("ERROR : Selected FFT Tasks from Radar Dictionary Set %u but there are only %u sets in the dictionary %s\n", crit_fft_samples_set, num_radar_samples_sets, rad_dict);
    exit(-1);
  }
    
  if (global_scheduler_selection_policy > NUM_SELECTION_POLICIES) {
    printf("ERROR : Selected Scheduler Policy (%u) is larger than number of policies (%u)\n", global_scheduler_selection_policy, NUM_SELECTION_POLICIES);
    exit(-1);
  }
  printf("Scheduler is using Policy %u : %s\n", global_scheduler_selection_policy, scheduler_selection_policy_str[global_scheduler_selection_policy]);
  
  /* We assume the vehicle starts in the following state:
   *  - Lane: center
   *  - Speed: 50 mph
   */
  vehicle_state.active  = true;
  vehicle_state.lane    = center;
  vehicle_state.speed   = 50;
  DEBUG(printf("Vehicle starts with the following state: active: %u lane %u speed %.1f\n", vehicle_state.active, vehicle_state.lane, vehicle_state.speed));

  #ifdef USE_SIM_ENVIRON
  // In simulation mode, we could start the main car is a different state (lane, speed)
  init_sim_environs(world_desc_file_name, &vehicle_state);
  #endif

/*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
  time_step = 0;
 #ifdef TIME
  struct timeval stop_prog, start_prog;

  struct timeval stop_iter_rad, start_iter_rad;
  struct timeval stop_iter_vit, start_iter_vit;
  struct timeval stop_iter_cv , start_iter_cv;
  struct timeval stop_iter_h264 , start_iter_h264;

  uint64_t iter_rad_sec = 0LL;
  uint64_t iter_vit_sec = 0LL;
  uint64_t iter_cv_sec  = 0LL;
  uint64_t iter_h264_sec  = 0LL;

  uint64_t iter_rad_usec = 0LL;
  uint64_t iter_vit_usec = 0LL;
  uint64_t iter_cv_usec  = 0LL;
  uint64_t iter_h264_usec  = 0LL;

  struct timeval stop_exec_rad, start_exec_rad;
  struct timeval stop_exec_vit, start_exec_vit;
  struct timeval stop_exec_cv , start_exec_cv;
  struct timeval stop_exec_h264 , start_exec_h264;

  uint64_t exec_rad_sec = 0LL;
  uint64_t exec_vit_sec = 0LL;
  uint64_t exec_cv_sec  = 0LL;
  uint64_t exec_h264_sec  = 0LL;

  uint64_t exec_rad_usec = 0LL;
  uint64_t exec_vit_usec = 0LL;
  uint64_t exec_cv_usec  = 0LL;
  uint64_t exec_h264_usec  = 0LL;

  struct timeval stop_exec_pandc , start_exec_pandc;
  uint64_t exec_pandc_sec  = 0LL;
  uint64_t exec_pandc_usec  = 0LL;

  struct timeval stop_wait_all_crit, start_wait_all_crit;
  uint64_t wait_all_crit_sec = 0LL;
  uint64_t wait_all_crit_usec = 0LL;
 #endif // TIME

  printf("Starting the main loop...\n");
  /* The input trace contains the per-epoch (time-step) input data */
 #ifdef TIME
  gettimeofday(&start_prog, NULL);
 #endif
  
 #ifdef USE_SIM_ENVIRON
  DEBUG(printf("\n\nTime Step %d\n", time_step));
  while (iterate_sim_environs(vehicle_state))
 #else //TRACE DRIVEN MODE
  read_next_trace_record(vehicle_state);
  while (!eof_trace_reader())
 #endif
  {
    DEBUG(printf("Vehicle_State: Lane %u %s Speed %.1f\n", vehicle_state.lane, lane_names[vehicle_state.lane], vehicle_state.speed));

    /* The computer vision kernel performs object recognition on the
     * next image, and returns the corresponding label. 
     * This process takes place locally (i.e. within this car).
     */
    /**
   #ifdef TIME
    gettimeofday(&start_iter_h264, NULL);
   #endif
    h264_dict_entry_t* hdep = 0x0;
    if (!bypass_h264_functions) {
      hdep = iterate_h264_kernel(vehicle_state);
    }
   #ifdef TIME
    gettimeofday(&stop_iter_h264, NULL);
    iter_h264_sec  += stop_iter_h264.tv_sec  - start_iter_h264.tv_sec;
    iter_h264_usec += stop_iter_h264.tv_usec - start_iter_h264.tv_usec;
   #endif
    **/
    /* The computer vision kernel performs object recognition on the
     * next image, and returns the corresponding label. 
     * This process takes place locally (i.e. within this car).
     */
   #ifdef TIME
    gettimeofday(&start_iter_cv, NULL);
   #endif
    label_t cv_tr_label = iterate_cv_kernel(vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_cv, NULL);
    iter_cv_sec  += stop_iter_cv.tv_sec  - start_iter_cv.tv_sec;
    iter_cv_usec += stop_iter_cv.tv_usec - start_iter_cv.tv_usec;
   #endif

    /* The radar kernel performs distance estimation on the next radar
     * data, and returns the estimated distance to the object.
     */
   #ifdef TIME
    gettimeofday(&start_iter_rad, NULL);
   #endif
    radar_dict_entry_t* rdentry_p = iterate_rad_kernel(vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_rad, NULL);
    iter_rad_sec  += stop_iter_rad.tv_sec  - start_iter_rad.tv_sec;
    iter_rad_usec += stop_iter_rad.tv_usec - start_iter_rad.tv_usec;
   #endif
    distance_t rdict_dist = rdentry_p->distance;
    float * radar_inputs = rdentry_p->return_data;

    /* The Viterbi decoding kernel performs Viterbi decoding on the next
     * OFDM symbol (message), and returns the extracted message.
     * This message can come from another car (including, for example,
     * its 'pose') or from the infrastructure (like speed violation or
     * road construction warnings). For simplicity, we define a fix set
     * of message classes (e.g. car on the right, car on the left, etc.)
     */
   #ifdef TIME
    gettimeofday(&start_iter_vit, NULL);
   #endif
    vit_dict_entry_t* vdentry_p = iterate_vit_kernel(vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_vit, NULL);
    iter_vit_sec  += stop_iter_vit.tv_sec  - start_iter_vit.tv_sec;
    iter_vit_usec += stop_iter_vit.tv_usec - start_iter_vit.tv_usec;
   #endif

    // EXECUTE the kernels using the now known inputs
    /**
   #ifdef TIME
    gettimeofday(&start_exec_h264, NULL);
   #endif
    char* found_frame_ptr = 0x0;
    if (!bypass_h264_functions) {
      execute_h264_kernel(hdep, found_frame_ptr);
    } else {
      found_frame_ptr = (char*)0xAD065BED;
    }
   #ifdef TIME
    gettimeofday(&stop_exec_h264, NULL);
    exec_h264_sec  += stop_exec_h264.tv_sec  - start_exec_h264.tv_sec;
    exec_h264_usec += stop_exec_h264.tv_usec - start_exec_h264.tv_usec;
   #endif
    **/
   #ifdef TIME
    gettimeofday(&start_exec_cv, NULL);
   #endif
    label = execute_cv_kernel(cv_tr_label);
   #ifdef TIME
    gettimeofday(&stop_exec_cv, NULL);
    exec_cv_sec  += stop_exec_cv.tv_sec  - start_exec_cv.tv_sec;
    exec_cv_usec += stop_exec_cv.tv_usec - start_exec_cv.tv_usec;

    gettimeofday(&start_exec_rad, NULL);
   #endif
    // Request a MetadataBlock (for an FFT_TASK at Critical Level)
    task_metadata_block_t* fft_mb_ptr = get_task_metadata_block(FFT_TASK, CRITICAL_TASK, fft_profile);
    if (fft_mb_ptr == NULL) {
      // We ran out of metadata blocks -- PANIC!
      printf("Out of metadata blocks for FFT -- PANIC Quit the run (for now)\n");
      exit (-4);
    }
    fft_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
    start_execution_of_rad_kernel(fft_mb_ptr, radar_log_nsamples_per_dict_set[crit_fft_samples_set], radar_inputs); // Critical RADAR task
    for (int i = 0; i < additional_fft_tasks_per_time_step; i++) {
      task_metadata_block_t* fft_mb_ptr_2 = get_task_metadata_block(FFT_TASK, BASE_TASK, fft_profile);
      if (fft_mb_ptr_2 == NULL) {
	printf("Out of metadata blocks for Non-Critical FFT -- PANIC Quit the run (for now)\n");
	exit (-5);
      }
      fft_mb_ptr_2->atFinish = base_release_metadata_block;
      radar_dict_entry_t* rdentry_p2;
      if (task_size_variability == 0) {
	rdentry_p2 = select_critical_radar_input(rdentry_p);
      } else {
	rdentry_p2 = select_random_radar_input();
      }
      float* addl_radar_inputs = rdentry_p2->return_data;
      start_execution_of_rad_kernel(fft_mb_ptr_2, radar_log_nsamples_per_dict_set[crit_fft_samples_set], addl_radar_inputs); // Critical RADAR task
    }

    DEBUG(printf("FFT_TASK_BLOCK: ID = %u\n", fft_mb_ptr->metadata_block_id));
   #ifdef TIME
    gettimeofday(&start_exec_vit, NULL);
   #endif
    //NOTE Removed the num_messages stuff -- need to do this differently (separate invocations of this process per message)
    // Request a MetadataBlock (for an VITERBI_TASK at Critical Level)
    task_metadata_block_t* vit_mb_ptr = get_task_metadata_block(VITERBI_TASK, 3, vit_profile);
    if (vit_mb_ptr == NULL) {
      // We ran out of metadata blocks -- PANIC!
      printf("Out of metadata blocks for VITERBI -- PANIC Quit the run (for now)\n");
      exit (-4);
    }
    vit_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
    start_execution_of_vit_kernel(vit_mb_ptr, vdentry_p); // Critical VITERBI task
    DEBUG(printf("VIT_TASK_BLOCK: ID = %u\n", vit_mb_ptr->metadata_block_id));
    for (int i = 0; i < additional_vit_tasks_per_time_step; i++) {
      task_metadata_block_t* vit_mb_ptr_2 = get_task_metadata_block(VITERBI_TASK, BASE_TASK, vit_profile);
      if (vit_mb_ptr_2 == NULL) {
	printf("Out of metadata blocks for Non-Critical VIT -- PANIC Quit the run (for now)\n");
	exit (-5);
      }
      vit_mb_ptr_2->atFinish = base_release_metadata_block;
      vit_dict_entry_t* vdentry2_p;
      if (task_size_variability == 0) {
	int lnum = vdentry_p->msg_num / NUM_MESSAGES;
	int m_id = vdentry_p->msg_num % NUM_MESSAGES;
	if (m_id != vdentry_p->msg_id) {
	  printf("WARNING: MSG_NUM %u : LNUM %u M_ID %u MSG_ID %u\n", vdentry_p->msg_num, lnum, m_id, vdentry_p->msg_id);
	}
	vdentry2_p = select_specific_vit_input(lnum, m_id);
      } else {
	DEBUG(printf("Note: electing a random Vit Message for base-task %u\n", vit_mb_ptr_2->block_id));
	vdentry2_p = select_random_vit_input();
      }
      start_execution_of_vit_kernel(vit_mb_ptr_2, vdentry2_p); // Non-Critical VITERBI task
    }
   #ifdef TIME
    gettimeofday(&stop_exec_vit, NULL);
    exec_vit_sec  += stop_exec_vit.tv_sec  - start_exec_vit.tv_sec;
    exec_vit_usec += stop_exec_vit.tv_usec - start_exec_vit.tv_usec;
   #endif

   #ifdef TIME
    gettimeofday(&start_wait_all_crit, NULL);
   #endif

    TDEBUG(printf("MAIN: Calling wait_all_critical\n"));
    wait_all_critical();

   #ifdef TIME
    gettimeofday(&stop_wait_all_crit, NULL);
    wait_all_crit_sec  += stop_wait_all_crit.tv_sec  - start_wait_all_crit.tv_sec;
    wait_all_crit_usec += stop_wait_all_crit.tv_usec - start_wait_all_crit.tv_usec;
   #endif
    
    distance = finish_execution_of_rad_kernel(fft_mb_ptr);
    message = finish_execution_of_vit_kernel(vit_mb_ptr);
   #ifdef TIME
    gettimeofday(&stop_exec_rad, NULL);
    exec_rad_sec  += stop_exec_rad.tv_sec  - start_exec_rad.tv_sec;
    exec_rad_usec += stop_exec_rad.tv_usec - start_exec_rad.tv_usec;
   #endif

    // POST-EXECUTE each kernel to gather stats, etc.
    /*if (!bypass_h264_functions) {
      post_execute_h264_kernel();
      }*/
    post_execute_cv_kernel(cv_tr_label, label);
    post_execute_rad_kernel(rdentry_p->set, rdentry_p->index_in_set, rdict_dist, distance);
    post_execute_vit_kernel(vdentry_p->msg_id, message);

    /* The plan_and_control() function makes planning and control decisions
     * based on the currently perceived information. It returns the new
     * vehicle state.
     */
    DEBUG(printf("Time Step %3u : Calling Plan and Control %u times with message %u and distance %.1f\n", pandc_repeat_factor, time_step, message, distance));
    vehicle_state_t new_vehicle_state;
   #ifdef TIME
    gettimeofday(&start_exec_pandc, NULL);
   #endif
    for (int prfi = 0; prfi <= pandc_repeat_factor; prfi++) {
      new_vehicle_state = plan_and_control(label, distance, message, vehicle_state);
    }
   #ifdef TIME
    gettimeofday(&stop_exec_pandc, NULL);
    exec_pandc_sec  += stop_exec_pandc.tv_sec  - start_exec_pandc.tv_sec;
    exec_pandc_usec += stop_exec_pandc.tv_usec - start_exec_pandc.tv_usec;
   #endif
    vehicle_state = new_vehicle_state;

    DEBUG(printf("New vehicle state: lane %u speed %.1f\n\n", vehicle_state.lane, vehicle_state.speed));

   #ifdef TIME
    time_step++;
   #endif

    // TEST - trying this here.
    //wait_all_tasks_finish();
    
    #ifndef USE_SIM_ENVIRON
    read_next_trace_record(vehicle_state);
    #endif
  }

  // This is the end of time steps... wait for all tasks to be finished (?)
  // Adding this results in never completing...  not sure why.
  // wait_all_tasks_finish();
  
 #ifdef TIME
  gettimeofday(&stop_prog, NULL);
 #endif

  /* All the trace/simulation-time has been completed -- Quitting... */
  printf("\nRun completed %u time steps\n\n", time_step);

  if (!bypass_h264_functions) {
    //closeout_h264_kernel();
  }
  closeout_cv_kernel();
  closeout_rad_kernel();
  closeout_vit_kernel();

  #ifdef TIME
  {
    uint64_t total_exec = (uint64_t) (stop_prog.tv_sec - start_prog.tv_sec) * 1000000 + (uint64_t) (stop_prog.tv_usec - start_prog.tv_usec);
    uint64_t iter_rad   = (uint64_t) (iter_rad_sec) * 1000000 + (uint64_t) (iter_rad_usec);
    uint64_t iter_vit   = (uint64_t) (iter_vit_sec) * 1000000 + (uint64_t) (iter_vit_usec);
    uint64_t iter_cv    = (uint64_t) (iter_cv_sec)  * 1000000 + (uint64_t) (iter_cv_usec);
    //uint64_t iter_h264  = (uint64_t) (iter_h264_sec) * 1000000 + (uint64_t) (iter_h264_usec);
    uint64_t exec_rad   = (uint64_t) (exec_rad_sec) * 1000000 + (uint64_t) (exec_rad_usec);
    uint64_t exec_vit   = (uint64_t) (exec_vit_sec) * 1000000 + (uint64_t) (exec_vit_usec);
    uint64_t exec_cv    = (uint64_t) (exec_cv_sec)  * 1000000 + (uint64_t) (exec_cv_usec);
    //uint64_t exec_h264  = (uint64_t) (exec_h264_sec) * 1000000 + (uint64_t) (exec_h264_usec);
    uint64_t exec_pandc = (uint64_t) (exec_pandc_sec) * 1000000 + (uint64_t) (exec_pandc_usec);
    uint64_t wait_all_crit   = (uint64_t) (wait_all_crit_sec) * 1000000 + (uint64_t) (wait_all_crit_usec);
    printf("\nProgram total execution time      %lu usec\n", total_exec);
    printf("  iterate_rad_kernel run time       %lu usec\n", iter_rad);
    printf("  iterate_vit_kernel run time       %lu usec\n", iter_vit);
    printf("  iterate_cv_kernel run time        %lu usec\n", iter_cv);
    //printf("  iterate_h264_kernel run time      %lu usec\n", iter_h264);
    printf("  Crit execute_rad_kernel run time  %lu usec\n", exec_rad);
    printf("  Crit execute_vit_kernel run time  %lu usec\n", exec_vit);
    printf("  Crit execute_cv_kernel run time   %lu usec\n", exec_cv);
    //printf("  execute_h264_kernel run time      %lu usec\n", exec_h264);
    printf("  plan_and_control run time         %lu usec at %u factor\n", exec_pandc, pandc_repeat_factor);
    printf("  wait_all_critical run time        %lu usec\n", wait_all_crit);
  }
 #endif // TIME
  shutdown_scheduler();
  printf("\nDone.\n");
  return 0;
}
