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
#include "fft_accel.h"
#include "vit_accel.h"
#include "cv_accel.h"

#include "fft_task.h"
#include "vit_task.h"
#include "cv_task.h"
#include "test_task.h"

#include "kernels_api.h"
#include "sim_environs.h"
#include "getopt.h"

/* Input Trace Functions */
#ifndef USE_SIM_ENVIRON
 #include "read_trace.h"
#else
 #include "sim_environs.h"
#endif

#define TIME

//#define get_mb_holdoff 10  // usec

char h264_dict[256]; 
char cv_dict[256]; 
char rad_dict[256];
char vit_dict[256];

bool_t bypass_h264_functions = false; // This is a global-disable of executing H264 execution functions...


#include "cpu_accel.h"
#include "fft_accel.h"
#include "vit_accel.h"
#include "cv_accel.h"

// Storage to hold the accelerator IDs (returned when we register accelerators) of the accelerator types we will need/use
accelerator_type_t cpu_accel_id;
accelerator_type_t fft_hwr_accel_id;
accelerator_type_t vit_hwr_accel_id;
accelerator_type_t cv_hwr_accel_id;

//#define my_num_accel_types  4

// Storage to hold the task IDs (returned when we register taske) of the task types we will need/use
//task_type_t no_task_type;
task_type_t fft_task_type;
task_type_t vit_task_type;
task_type_t cv_task_type;
task_type_t test_task_type;

#define my_num_task_types  4

unsigned accel_limit_cpu = NUM_CPU_ACCEL;
unsigned accel_limit_fft = NUM_FFT_ACCEL;
unsigned accel_limit_vit = NUM_VIT_ACCEL;
unsigned accel_limit_cv  = NUM_CV_ACCEL;

// This is a set of HW Threshold values for use in the P0 (Policy-v0)
//  This is of size [NUM_TASKS][MAX_ACCELERATORS] (so dynamically allocated when we know NUM_TASKS)
//  The order is also significant, so "filled in" as we register the tasks
unsigned** p0_hw_threshold;

// These are defined by the task-type
uint64_t fft_profile[2][MAX_ACCEL_TYPES]; // FFT tasks can be 1k or 16k samplesw
uint64_t vit_profile[4][MAX_ACCEL_TYPES]; // Vit messages can by short, medium, long, or max
uint64_t cv_profile[MAX_ACCEL_TYPES];
uint64_t test_profile[MAX_ACCEL_TYPES];

bool_t all_obstacle_lanes_mode = false;
bool_t no_crit_cnn_task = false;
unsigned time_step;
unsigned pandc_repeat_factor = 1;
unsigned task_size_variability;


void print_usage(char * pname) {
  printf("Usage: %s <OPTIONS>\n", pname);
  printf(" OPTIONS:\n");
  printf("    -h          : print this helpful usage info\n");
  printf("    -o          : print the Visualizer output traace information during the run\n");
  printf("    -R <file>   : defines the input Radar dictionary file <file> to use\n");
  printf("    -V <file>   : defines the input Viterbi dictionary file <file> to use\n");
  printf("    -C <file>   : defines the input CV/CNN dictionary file <file> to use\n");
  //printf("    -H <file>  : defines the input H264 dictionary file <file> to use\n");
  //printf("    -b         : Bypass (do not use) the H264 functions in this run.\n");
  printf("    -s <N>      : Sets the max number of time steps to simulate\n");
 #ifdef USE_SIM_ENVIRON
  printf("    -r <N>      : Sets the rand random number seed to N\n");
  printf("    -A          : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)\n");
  printf("    -W <wfile>  : defines the world environment parameters description file <wfile> to use\n");
 #else
  printf("    -t <trace>  : defines the input trace file <trace> to use\n");
 #endif
  printf("    -p <N>      : defines the plan-and-control repeat factor (calls per time step -- default is 1)\n");
  printf("    -f <N>      : defines which Radar Dictionary Set is used for Critical FFT Tasks\n");
  printf("                :      Each Set of Radar Dictionary Entries Can use a different sample size, etc.\n");
  
  printf("    -N <N>      : Adds <N> additional (non-critical) CV/CNN tasks per time step.\n");
  printf("    -D <N>      : Delay (in usec) of CPU CV Tasks (faked execution)\n");
 #ifdef FAKE_HW_CV
  printf("    -d <N>      : Delay (in usec) of HWR CV Tasks (faked execution)\n");
 #endif
  printf("    -F <N>      : Adds <N> additional (non-critical) FFT tasks per time step.\n");
  printf("    -v <N>      : defines Viterbi message size:\n");
  printf("                :      0 = Short messages (4 characters)\n");
  printf("                :      1 = Medium messages (500 characters)\n");
  printf("                :      2 = Long messages (1000 characters)\n");
  printf("                :      3 = Max-sized messages (1500 characters)\n");
  printf("    -M <N>      : Adds <N> additional (non-critical) Viterbi message tasks per time step.\n");
  printf("    -S <N>      : Task-Size Variability: Varies the sizes of input tasks where appropriate\n");
  printf("                :      0 = No variability (e.g. all messages same size, etc.)\n");
  printf("    -u <N>      : Sets the hold-off usec for checks on work in the scheduler queue\n");
  printf("                :   This reduces the busy-spin-loop rate for the scheduler thread\n");
  printf("    -B <N>      : Sets the number of Metadata Blocks (max) to <N>\n");
  printf("    -T <N>      : Sets the number of Task Types (max) to <N> (but must be >= 4 for this usage)\n");
  printf("    -P <policy> : defines the task scheduling policy <policy> to use (<policy> is a string)\n");
  printf("                :   <policy> needs to exist as a dynamic shared object (DSO) with filename lib<policy>.so\n");
  printf("    -L <tuple>  : Sets the limits on number of each accelerator type available in this run.\n");
  printf("                :      tuple = #CPU,#FFT,#VIT,#CV (string interpreted internally)\n");
  printf("    -X <tuple>  : Sets the Test-Task parameters for this run; default is NO Test-Tasks.\n");
  printf("                :   Two tuple formats are acceptable:\n");
  printf("                :      tuple = #Crit,#Base : Number of per-time-step Critical and Base Test-tasks injected\n");
  printf("                :      tuple = #Crit,#Base,tCPU,tFFT,tVIT,tCV : Num Crit and Base tasks, and usec exec time\n");
  printf("                :              per each accelerator type\n");
  #ifdef SL_VIZ
  printf("\n");
  printf(" Options for the STOMP-viz visualization tool (tracing enabled):\n");
  printf("    -i <N>      : Number of executed tasks (of any type) before starting task logging\n");
  printf("                :   If not specified, then logging starts with the execution\n");
  printf("    -I <type>   : Task type that triggers task logging for the STOMP-viz visualization tool\n");
  printf("                :   If not specified, then logging starts with the execution\n");
  printf("                :  NOTE: If -i and -I are specified, then logging starts when either condition is satisfied\n");
  printf("    -e <N>      : Number of executed tasks (of any type) before stopping task logging\n");
  printf("                :   This parameter is mandatory to keep control of the trace file size\n");
#endif

}


// This is just a call-through to the scheduler routine, but we can also print a message here...
//  This SHOULD be a routine that "does the right work" for a given task, and then releases the MetaData Block
void base_release_metadata_block(task_metadata_block_t* mb)
{
  TDEBUG(scheduler_datastate_block_t* sptr = mb->scheduler_datastate_pointer;
	 printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n", mb->block_id, sptr->task_name_str[mb->task_type], sptr->task_criticality_str[mb->crit_level], sptr->accel_name_str[mb->accelerator_type], mb->accelerator_id));
  free_task_metadata_block(mb);
  // Thread is done -- We shouldn't need to do anything else -- when it returns from its starting function it should exit.
}

void radar_release_metadata_block(task_metadata_block_t* mb)
{
  TDEBUG(scheduler_datastate_block_t* sptr = mb->scheduler_datastate_pointer;
	 printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n", mb->block_id, sptr->task_name_str[mb->task_type], sptr->task_criticality_str[mb->crit_level], sptr->accel_name_str[mb->accelerator_type], mb->accelerator_id));
  // Call this so we get final stats (call-time)
  distance_t distance = finish_execution_of_rad_kernel(mb);

  free_task_metadata_block(mb);
  // Thread is done -- We shouldn't need to do anything else -- when it returns from its starting function it should exit.
}




void set_up_scheduler_accelerators_and_tasks(scheduler_datastate_block_t* sptr) {
  printf("\nSetting up/Registering the ACCELERATORS...\n");
  accelerator_pool_defn_info_t accel_def;

  sprintf(accel_def.name, "CPU-Acc");
  sprintf(accel_def.description, "Run task on a RISC-V CPU thread");
  accel_def.number_available        = accel_limit_cpu;
  accel_def.do_accel_initialization = &do_cpu_accel_type_initialization;
  accel_def.do_accel_closeout       = &do_cpu_accel_type_closeout;
  accel_def.output_accel_run_stats  = &output_cpu_accel_type_run_stats;
  cpu_accel_id = register_accelerator_pool(sptr, &accel_def);

  sprintf(accel_def.name, "VIT-HW-Acc");
  sprintf(accel_def.description, "Run task on the Viterbi-Decode Hardware Accelerator");
  accel_def.number_available        = accel_limit_vit;
  accel_def.do_accel_initialization = &do_vit_accel_type_initialization;
  accel_def.do_accel_closeout       = &do_vit_accel_type_closeout;
  accel_def.output_accel_run_stats  = &output_vit_accel_type_run_stats;
  vit_hwr_accel_id = register_accelerator_pool(sptr, &accel_def);
    
  sprintf(accel_def.name, "FFT-HW-Acc");
  sprintf(accel_def.description, "Run task on the 1-D FFT Hardware Accelerator");
  accel_def.number_available        = accel_limit_fft;
  accel_def.do_accel_initialization = &do_fft_accel_type_initialization;
  accel_def.do_accel_closeout       = &do_fft_accel_type_closeout;
  accel_def.output_accel_run_stats  = &output_fft_accel_type_run_stats;
  fft_hwr_accel_id = register_accelerator_pool(sptr, &accel_def);

  sprintf(accel_def.name, "CV-HW-Acc");
  sprintf(accel_def.description, "Run task on the CV/CNN NVDLA Hardware Accelerator");
  accel_def.number_available        = accel_limit_cv;
  accel_def.do_accel_initialization = &do_cv_accel_type_initialization;
  accel_def.do_accel_closeout       = &do_cv_accel_type_closeout;
  accel_def.output_accel_run_stats  = &output_cv_accel_type_run_stats;
  cv_hwr_accel_id = register_accelerator_pool(sptr, &accel_def);

  // Now set up the Task Types...
  printf("\nSetting up/Registering the TASK TYPES...\n");
  task_type_defn_info_t task_defn;

  /* sprintf(task_defn.name, "NO-Task"); */
  /* sprintf(task_defn.description, "A place-holder that indicates NO task to execute"); */
  /* task_defn.print_metadata_block_contents  = &print_base_metadata_block_contents; */
  /* task_defn.output_task_type_run_stats  = NULL; */
  /* no_task_type = register_task_type(sptr, &task_defn); */

  sprintf(task_defn.name, "VIT-Task");
  sprintf(task_defn.description, "A Viterbi Decoding task to execute");
  task_defn.print_metadata_block_contents  = &print_viterbi_metadata_block_contents;
  task_defn.output_task_type_run_stats  = &output_vit_task_type_run_stats;
  vit_task_type = register_task_type(sptr, &task_defn);
  //printf("Setting up %s with task-type %u\n", task_defn.name, vit_task_type);
  register_accel_can_exec_task(sptr, cpu_accel_id,     vit_task_type, &exec_vit_task_on_cpu_accel);
  register_accel_can_exec_task(sptr, vit_hwr_accel_id, vit_task_type, &exec_vit_task_on_vit_hwr_accel);
  if (NUM_VIT_ACCEL > 0) {
    // Add the new Policy-v0 HW_Threshold values for VIT tasks
    p0_hw_threshold[vit_task_type][vit_hwr_accel_id] = 25; // ~75% chance to use VIT HWR for Vit Tasks
    printf("Set p0_hw_threshold[%s][%s] = %u\n", sptr->task_name_str[vit_task_type], sptr->accel_name_str[vit_hwr_accel_id], p0_hw_threshold[vit_task_type][vit_hwr_accel_id]);
  }
  
  sprintf(task_defn.name, "CV-Task");
  sprintf(task_defn.description, "A CV/CNN task to execute");
  task_defn.print_metadata_block_contents  = &print_cv_metadata_block_contents;
  task_defn.output_task_type_run_stats  = &output_cv_task_type_run_stats;
  cv_task_type = register_task_type(sptr, &task_defn);
  //printf("Setting up %s with task-type %u\n", task_defn.name, cv_task_type);
  register_accel_can_exec_task(sptr, cpu_accel_id,    cv_task_type, &execute_cpu_cv_accelerator);
  register_accel_can_exec_task(sptr, cv_hwr_accel_id, cv_task_type, &execute_hwr_cv_accelerator);
  if (NUM_CV_ACCEL > 0) {
    // Add the new Policy-v0 HW_Threshold values for CV tasks
    p0_hw_threshold[cv_task_type][cv_hwr_accel_id] = 25; // ~75% chance to use CV HWR for CV Tasks
    printf("Set p0_hw_threshold[%s][%s] = %u\n", sptr->task_name_str[cv_task_type], sptr->accel_name_str[cv_hwr_accel_id], p0_hw_threshold[cv_task_type][cv_hwr_accel_id]);
  }
  
  sprintf(task_defn.name, "FFT-Task");
  sprintf(task_defn.description, "A 1-D FFT task to execute");
  task_defn.print_metadata_block_contents  = &print_fft_metadata_block_contents;
  task_defn.output_task_type_run_stats  = &output_fft_task_type_run_stats;
  fft_task_type = register_task_type(sptr, &task_defn);
  //printf("Setting up %s with task-type %u\n", task_defn.name, fft_task_type);
  register_accel_can_exec_task(sptr, cpu_accel_id,     fft_task_type, &execute_cpu_fft_accelerator);
  register_accel_can_exec_task(sptr, fft_hwr_accel_id, fft_task_type, &execute_hwr_fft_accelerator);
  if (NUM_FFT_ACCEL > 0) {
    // Add the new Policy-v0 HW_Threshold values for FFT tasks
    p0_hw_threshold[fft_task_type][fft_hwr_accel_id] = 25; // ~75% chance to use FFT HWR for FFT Tasks
    printf("Set p0_hw_threshold[%s][%s] = %u\n", sptr->task_name_str[fft_task_type], sptr->accel_name_str[fft_hwr_accel_id], p0_hw_threshold[fft_task_type][fft_hwr_accel_id]);
  }
  
  // Opotionally add the "Test Task" (to test flexibility in the scheduler, etc.
  if ((num_Crit_test_tasks + num_Base_test_tasks) > 0) {
    sprintf(task_defn.name, "TEST-Task");
    sprintf(task_defn.description, "A TESTING task to execute");
    task_defn.print_metadata_block_contents  = &print_test_metadata_block_contents;
    task_defn.output_task_type_run_stats  = &output_test_task_type_run_stats;
    test_task_type = register_task_type(sptr, &task_defn);
    //printf("Setting up %s with task-type %u\n", task_defn.name, test_task_type);
    register_accel_can_exec_task(sptr, cpu_accel_id,     test_task_type, &execute_on_cpu_test_accelerator);
    if ((NUM_VIT_ACCEL > 0) && (test_on_hwr_vit_run_time_in_usec > 0)) {
      register_accel_can_exec_task(sptr, vit_hwr_accel_id, test_task_type, &execute_on_hwr_vit_test_accelerator);
      p0_hw_threshold[test_task_type][vit_hwr_accel_id] = 75; // ~25% chance to use VIT HWR for Test Tasks in P0
      printf("Set p0_hw_threshold[%s][%s] = %u\n", sptr->task_name_str[test_task_type], sptr->accel_name_str[vit_hwr_accel_id], p0_hw_threshold[test_task_type][vit_hwr_accel_id]);
      //printf("Set p0_hw_threshold[%u][%u] = %u\n", test_task_type, vit_hwr_accel_id, p0_hw_threshold[test_task_type][vit_hwr_accel_id]);
    }
    if ((NUM_FFT_ACCEL > 0) && (test_on_hwr_fft_run_time_in_usec > 0)) {
      register_accel_can_exec_task(sptr, fft_hwr_accel_id, test_task_type, &execute_on_hwr_fft_test_accelerator);
      p0_hw_threshold[test_task_type][fft_hwr_accel_id] = 50; // ~25% chance to use FFT HWR for Test Tasks in P0
      printf("Set p0_hw_threshold[%s][%s] = %u\n", sptr->task_name_str[test_task_type], sptr->accel_name_str[fft_hwr_accel_id], p0_hw_threshold[test_task_type][fft_hwr_accel_id]);
      //printf("Set p0_hw_threshold[%u][%u] = %u\n", test_task_type, fft_hwr_accel_id, p0_hw_threshold[test_task_type][fft_hwr_accel_id]);
    }
    if ((NUM_CV_ACCEL > 0) && (test_on_hwr_cv_run_time_in_usec > 0)) {
      register_accel_can_exec_task(sptr, cv_hwr_accel_id, test_task_type, &execute_on_hwr_cv_test_accelerator);
      p0_hw_threshold[test_task_type][cv_hwr_accel_id] = 25; // ~25% chance to use CV HWR for Test Tasks in P0
      printf("Set p0_hw_threshold[%s][%s] = %u\n", sptr->task_name_str[test_task_type], sptr->accel_name_str[cv_hwr_accel_id], p0_hw_threshold[test_task_type][cv_hwr_accel_id]);
    }
  }

  printf("Done Setting up/Registering Accelerators and Task Types...\n\n");
}



void set_up_task_on_accel_profile_data()
{
  for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
    fft_profile[0][ai] = ACINFPROF;
    fft_profile[1][ai] = ACINFPROF;
    vit_profile[0][ai] = ACINFPROF;
    vit_profile[1][ai] = ACINFPROF;
    vit_profile[2][ai] = ACINFPROF;
    vit_profile[3][ai] = ACINFPROF;
    cv_profile[ai]   = ACINFPROF;
    test_profile[ai] = ACINFPROF;
  }
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  fft_profile[0][cpu_accel_id]     =  23000;
  fft_profile[0][fft_hwr_accel_id] =   6000;
  fft_profile[1][cpu_accel_id]     = 600000;
  fft_profile[1][fft_hwr_accel_id] = 143000;
  
  vit_profile[0][cpu_accel_id]     =  120000;
  vit_profile[0][vit_hwr_accel_id] =    5950;
  vit_profile[1][cpu_accel_id]     = 1700000;
  vit_profile[1][vit_hwr_accel_id] =   67000;
  vit_profile[2][cpu_accel_id]     = 3400000;
  vit_profile[2][vit_hwr_accel_id] =  135000;
  vit_profile[3][cpu_accel_id]     = 4800000;
  vit_profile[3][vit_hwr_accel_id] =  191000;
  
  cv_profile[cpu_accel_id]     = 5000000;
  cv_profile[cv_hwr_accel_id]  =  150000;

  if ((num_Crit_test_tasks + num_Base_test_tasks) > 0) {
    if (test_on_cpu_run_time_in_usec > 0) {
      test_profile[cpu_accel_id] = test_on_cpu_run_time_in_usec;
    }
    if (test_on_hwr_vit_run_time_in_usec > 0) {
      test_profile[vit_hwr_accel_id] = test_on_hwr_vit_run_time_in_usec;
    }
    if (test_on_hwr_fft_run_time_in_usec > 0) {
      test_profile[fft_hwr_accel_id] = test_on_hwr_fft_run_time_in_usec;
    }
    if (test_on_hwr_cv_run_time_in_usec > 0) {
      test_profile[cv_hwr_accel_id]  =  test_on_hwr_cv_run_time_in_usec;
    }
  }
  DEBUG(printf("\n%15s : %18s %18s %18s %18s\n", "PROFILES", "CPU", "VIT-HWR", "FFT-HWR", "CV-HWR");
	printf("%15s :", "fft_profile[0]");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", fft_profile[0][ai]);
	} printf("\n");
	printf("%15s :", "fft_profile[1]");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", fft_profile[1][ai]);
	} printf("\n");
	printf("%15s :", "vit_profile[0]");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", vit_profile[0][ai]);
	} printf("\n");
	printf("%15s :", "vit_profile[1]");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", vit_profile[1][ai]);
	} printf("\n");
	printf("%15s :", "vit_profile[2]");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", vit_profile[2][ai]);
	} printf("\n");
	printf("%15s :", "vit_profile[3]");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", vit_profile[3][ai]);
	} printf("\n");
	printf("%15s :", "cv_profile");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", cv_profile[ai]);
	} printf("\n");
	printf("%15s :", "test_profile");
	for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	  printf(" 0x%016lx", test_profile[ai]);
	} printf("\n");
	printf("\n"));

}


int main(int argc, char *argv[])
{
  
  vehicle_state_t vehicle_state;
  label_t label;
  distance_t distance;
  message_t message;
  test_res_t test_res;
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

  unsigned additional_cv_tasks_per_time_step = 0;
  unsigned additional_fft_tasks_per_time_step = 0;
  unsigned additional_vit_tasks_per_time_step = 0;
  unsigned max_additional_tasks_per_time_step = 0;

  unsigned sched_holdoff_usec = 0;
  char policy[256];
  unsigned num_MBs_to_use      = GLOBAL_METADATA_POOL_BLOCKS;
  unsigned num_maxTasks_to_use = my_num_task_types;
  
  // STOMP-viz tracing parameters
  task_type_t viz_task_start_type  = NO_Task;
  int32_t     viz_task_start_count = -1;
  int32_t     viz_task_stop_count  = -1;


  //printf("SIZEOF pthread_t : %lu\n", sizeof(pthread_t));
  
  // put ':' in the starting of the
  // string so that program can
  // distinguish between '?' and ':'
  while((opt = getopt(argc, argv, ":hcAbot:v:s:r:W:R:V:C:H:f:p:F:M:P:S:N:d:D:u:L:B:T:X:i:I:e:")) != -1) {
    switch(opt) {
    case 'h':
      print_usage(argv[0]);
      exit(0);
    case 'A':
      all_obstacle_lanes_mode = true;
      break;
    case 'c':
      no_crit_cnn_task= true;
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
    case 'u':
      sched_holdoff_usec = atoi(optarg);
      break;
    case 's':
      max_time_steps = atoi(optarg);
      break;
    case 'p':
      pandc_repeat_factor = atoi(optarg);
      break;
    case 'f':
      crit_fft_samples_set = atoi(optarg);
      break;
    case 'r':
     #ifdef USE_SIM_ENVIRON
      rand_seed = atoi(optarg);
     #endif
      break;
    case 't':
     #ifndef USE_SIM_ENVIRON
      trace_file = optarg;
     #endif
      break;
    case 'v':
      vit_msgs_size = atoi(optarg);
      if (vit_msgs_size >= VITERBI_MSG_LENGTHS) {
	printf("ERROR: Specified viterbi message size (%u) is larger than max (%u) : from the -v option\n", vit_msgs_size, VITERBI_MSG_LENGTHS);
	print_usage(argv[0]);
	exit(-1);
      }
      break;
    case 'S':
      task_size_variability = atoi(optarg);
      break;
    case 'W':
     #ifdef USE_SIM_ENVIRON
      world_desc_file_name = optarg;
     #endif
      break;
    case 'F':
      additional_fft_tasks_per_time_step = atoi(optarg);
      break;
    case 'M':
      additional_vit_tasks_per_time_step = atoi(optarg);
      break;
    case 'N':
      additional_cv_tasks_per_time_step = atoi(optarg);
      break;
    case 'P':
      snprintf(policy, 255, "%s", optarg);
      //global_scheduler_selection_policy = atoi(optarg);
      break;

    case 'd':
     #ifdef FAKE_HW_CV
      cv_fake_hwr_run_time_in_usec = atoi(optarg);
     #else
      printf("ERROR : I don't understand option '-d'\n");
      print_usage(argv[0]);
      exit(-1);
     #endif
      break;
    case 'D':
      cv_cpu_run_time_in_usec = atoi(optarg);
      break;

    case 'B':
      num_MBs_to_use = atoi(optarg);
      break;

    case 'T':
      num_maxTasks_to_use = atoi(optarg);
      break;

    case 'L': // Accelerator Limits for this run : CPU/CV/FFT/VIT
    {
      unsigned in_cpu = 0;
      unsigned in_cv = 0;
      unsigned in_fft = 0;
      unsigned in_vit = 0;
      if (sscanf(optarg, "%u,%u,%u,%u", &in_cpu, &in_fft, &in_vit, &in_cv) != 4) {
        printf("ERROR : Accelerator Limits (-L) argument didn't specify proper format: #CPU,#FFT,#VIT,#CV\n");
	exit(-1);
      }
      accel_limit_cpu = in_cpu;
      accel_limit_fft = in_fft;
      accel_limit_vit = in_vit;
      accel_limit_cv  = in_cv;
    }
    break;

    case 'X': // Add an X-tra task type (with "fake" execution usec times
      {
	unsigned nCrit = 0;
	unsigned nBase = 0;
	unsigned on_cpu = 0;
	unsigned on_fft = 0;
	unsigned on_vit = 0;
	unsigned on_cv  = 0;
	int sres = sscanf(optarg, "%u,%u,%u,%u,%u,%u", &nCrit, &nBase, &on_cpu, &on_fft, &on_vit, &on_cv);
	if ((sres != 2) && (sres != 6)) {
	  printf("ERROR : -X option (Add Xtra Test-Task) argument didn't specify proper format: Crit,Base<,CPU,FFT,VIT,CV>\n");
	  exit(-1);
	}
	DEBUG(printf("From -X option, sres = %u\n", sres););
	num_Crit_test_tasks = nCrit;
	num_Base_test_tasks = nBase;
	if (sres == 6) {
	  test_on_cpu_run_time_in_usec        = on_cpu;
	  test_on_hwr_fft_run_time_in_usec    = on_fft;
	  test_on_hwr_vit_run_time_in_usec    = on_vit;
	  test_on_hwr_cv_run_time_in_usec     = on_cv;
	  DEBUG(printf("     -X option set CPU %u FFT %u VIT %u CV %u\n", test_on_cpu_run_time_in_usec, test_on_hwr_fft_run_time_in_usec, test_on_hwr_vit_run_time_in_usec, test_on_hwr_cv_run_time_in_usec));
	}
      }
      break;

    case 'I':
     #ifdef SL_VIZ
      viz_task_start_type = atol(optarg);
     #endif
      break;

    case 'i':
     #ifdef SL_VIZ
      viz_task_start_count = atol(optarg);
     #endif
      break;

    case 'e':
     #ifdef SL_VIZ
      viz_task_stop_count = atol(optarg);
     #endif
      break;

    case ':':
      printf("option %c needs a value\n", optopt);
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

  /**#ifdef SL_VIZ
     if (viz_task_stop_count == 0){
     printf("ERROR - Task count must be >= 1 : %u specified (with '-e' option)\n", viz_task_stop_count);
     print_usage(argv[0]);
     exit(-1);
     }
     #endif
  **/
  
  if (pandc_repeat_factor == 0) {
    printf("ERROR - Plan-and-Control repeat factor must be >= 1 : %u specified (with '-p' option)\n", pandc_repeat_factor);
    print_usage(argv[0]);
    exit(-1);
  }

  // Get a scheduler_datastate_block
  scheduler_get_datastate_in_parms_t* sched_inparms = malloc(sizeof(scheduler_get_datastate_in_parms_t));
  if( sched_inparms == NULL) {
    printf("ERROR: Couldn't allocate the sched_inparms memory\n");
    exit(-99);
  }
  // Copy the scheduler's default parms into the new sched_inparms values
  copy_scheduler_datastate_defaults_into_parms(sched_inparms);
  // Alter the default parms to those values we want for this run...
  sched_inparms->max_metadata_pool_blocks = num_MBs_to_use;
  sched_inparms->max_task_types = num_maxTasks_to_use;

  // Now get a new scheduler datastate space
  scheduler_datastate_block_t* sptr = get_new_scheduler_datastate_pointer(sched_inparms);
  // Set the scheduler state values we need to for this run
  sptr->scheduler_holdoff_usec = sched_holdoff_usec;
  snprintf(sptr->policy, 255, "%s", policy);
  sptr->visualizer_task_start_count = viz_task_start_count;
  sptr->visualizer_task_stop_count  = viz_task_stop_count;
  sptr->visualizer_task_enable_type = viz_task_start_type;

  printf("LIMITS: Max Tasks %u Accels %u MB_blocks %u DSp_bytes %u Tsk_times %u Num_Acc_of_ty %u\n", sptr->limits.max_task_types, sptr->limits.max_accel_types, sptr->limits.max_metadata_pool_blocks, sptr->limits.max_data_space_bytes, sptr->limits.max_task_timing_sets, sptr->limits.max_accel_of_any_type);

  // Set up the task_on_accel profiles...
  p0_hw_threshold = calloc(num_maxTasks_to_use, sizeof(unsigned*));
  if (p0_hw_threshold == NULL) {
    printf("ERROR: main couldn't allocate memory for p0_hw_threshold\n");
    exit(-1);
  }
  for (int ti = 0; ti < num_maxTasks_to_use; ti++) {
    p0_hw_threshold[ti] = malloc(MAX_ACCEL_TYPES * sizeof(unsigned));
    if (p0_hw_threshold[ti] == NULL) {
      printf("ERROR: main couldn't allocate memory for p0_hw_threshold[%u]\n", ti);
      exit(-1);
    }
    for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
      p0_hw_threshold[ti][ai] = 101; // Pre-set all to be 0% chance of using any HWR
    }
  }

  printf("Run using scheduling policy %s using %u CPU accel %u HWR FFT %u HWR VIT and %u HWR CV and hold-off %u (of %u %u %u %u )\n",
	 sptr->policy,
	 accel_limit_cpu, accel_limit_fft, accel_limit_vit, accel_limit_cv, 
	 sptr-> scheduler_holdoff_usec, NUM_CPU_ACCEL, NUM_FFT_ACCEL, NUM_VIT_ACCEL, NUM_CV_ACCEL);

  if ((num_Crit_test_tasks + num_Base_test_tasks) > 0) {
    printf("Added Test-Task with %u Crit and %u Base, Timings: CPU %u FFT %u VIT %u CV %u\n", num_Crit_test_tasks, num_Base_test_tasks, test_on_cpu_run_time_in_usec, test_on_hwr_fft_run_time_in_usec, test_on_hwr_vit_run_time_in_usec, test_on_hwr_cv_run_time_in_usec);
  }
  
  #ifdef HW_FFT
  printf("Run has enabled Hardware-FFT : Device base is %s\n", FFT_DEV_BASE);
 #else
  printf("Run is using ONLY-CPU-FFT\n");
 #endif
 #ifdef HW_VIT
  printf("Run has enabled Hardware-Viterbi : Device base is %s\n", VIT_DEV_BASE);
 #else
  printf("Run is using ONLY-CPU-Viterbi\n");
 #endif
  {
    char* cv0_txt[3] = { "ONLY-CPU-", "CPU-And-", "ONLY-"};
    char* cv1_txt[3] = { "", "Fake-", "Hardware-" };
    int i = 0;
    int is = 0;
    int ie = 0;
   #ifdef HW_ONLY_CV
    i = 2;
   #endif
   #ifdef FAKE_HW_CV
    if (i == 0) { i = 1; }
    is = 1;
    ie = 2;
   #endif
   #ifdef HW_CV
    if (i == 0) { i = 1; }
    if (is == 0) { is = 2; }
    ie = 2;
   #endif
    printf("Run is using %s", cv0_txt[i]);
    for (int ix = is; ix <= ie; ix++ ){
      printf("%s", cv1_txt[ix]);
    }
    printf("CV with no-crit-CV = %u\n", no_crit_cnn_task);
  }
 #ifndef HW_ONLY_CV
  printf(" with cv_cpu_run_time_in_usec set to %u\n", cv_cpu_run_time_in_usec);
 #endif
 #ifdef FAKE_HW_CV
  printf("  and cv_fake_hwr_run_time_in_usec set to %u\n", cv_fake_hwr_run_time_in_usec);
 #endif
  
  printf("Using Plan-And-Control repeat factor %u\n", pandc_repeat_factor);
  printf("Using Radar Dictionary samples set %u for the critical FFT tasks\n", crit_fft_samples_set);
  printf("Using viterbi message size %u = %s\n", vit_msgs_size, vit_msgs_size_str[vit_msgs_size]);
  printf("Using task-size variability behavior %u\n", task_size_variability);
  printf("Using %u maximum time steps (simulation)\n", max_time_steps);
 #ifdef USE_SIM_ENVIRON
  printf("Using world description file: %s\n", world_desc_file_name);
  printf("Using random seed of %u\n", rand_seed);
 #else
  printf("Using trace file: %s\n", trace_file);
 #endif
      
  if (rad_dict[0] == '\0') {
    sprintf(rad_dict, "traces/norm_radar_all_dictionary.dfn");
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

 #ifdef SL_VIZ
  if (viz_task_start_type == NO_Task) {
    if (viz_task_start_count < 0) {
      printf("\nSTOMP-viz tracing starts from the very beginning (with the execution)\n");
    } else {
      printf("\nSTOMP-viz tracing starts on task number %d\n", viz_task_start_count);
    }
  } else {
    if (viz_task_start_count < 0) {
      printf("\nSTOMP-viz tracing starts with task type %d\n", viz_task_start_type);
    } else {
      printf("\nSTOMP-viz tracing starts on earlier of task number %d or task type %d\n", viz_task_start_count, viz_task_start_type);
    }
  }
  if (viz_task_stop_count < 0) {
    printf("STOMP-viz tracing continues for the full run (no executed tasks limit)\n");
  } else {
    printf("STOMP-viz tracing stops %d executed task(s) of any type after it starts\n", viz_task_stop_count);
  }
 #endif

  printf("\nDictionaries:\n");
  printf("   CV/CNN : %s\n", cv_dict);
  printf("   Radar  : %s\n", rad_dict);
  printf("   Viterbi: %s\n", vit_dict);

  printf("\n There are %u additional FFT, %u addtional Viterbi and %u Additional CV/CNN tasks per time step\n", additional_fft_tasks_per_time_step, additional_vit_tasks_per_time_step, additional_cv_tasks_per_time_step);
  max_additional_tasks_per_time_step = additional_fft_tasks_per_time_step;
  if (additional_vit_tasks_per_time_step > max_additional_tasks_per_time_step) {
    max_additional_tasks_per_time_step = additional_vit_tasks_per_time_step;
  }
  if (additional_cv_tasks_per_time_step > max_additional_tasks_per_time_step) {
    max_additional_tasks_per_time_step = additional_cv_tasks_per_time_step;
  }
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

  if (sptr->policy[0] == '\0') {
    printf("Error: a policy was not specified. Use -P to define the task scheduling policy to use.\n");
    print_usage(argv[0]);
    return 1;
  }

  printf("Doing initialization tasks...\n");
  initialize_scheduler(sptr);

  // Call the policy initialization, with the HW_THRESHOLD set up (in case we've selected policy 0)
  sptr->initialize_assign_task_to_pe(p0_hw_threshold);

  DEBUG(printf("p0_hw_threshold is @ %p\n", p0_hw_threshold);
	for (int ti = 0; ti < num_maxTasks_to_use; ti++) {
	  printf("p0_hw_threshold[%2u] @ %p :", ti, p0_hw_threshold[ti]);
	  for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
	    printf(" %3u", p0_hw_threshold[ti][ai]);
	  }
	  printf("\n");
	});


  // Set up the Accelerators for this application
  set_up_scheduler_accelerators_and_tasks(sptr);

  // Set up the profiles we will be using... NOTE: This must be AFTER we've added the accelerator and task definitions!
  set_up_task_on_accel_profile_data();

#ifndef USE_SIM_ENVIRON
  /* Trace Reader initialization */
  if (!init_trace_reader(trace_file))
  {
    printf("Error: the trace reader couldn't be initialized properly -- check the '-t' option.\n");
    print_usage(argv[0]);
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
  if (!init_cv_kernel(sptr, cv_py_file, cv_dict)) {
    printf("Error: the computer vision kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Radar kernel...\n");
  if (!init_rad_kernel(sptr, rad_dict)) {
    printf("Error: the radar kernel couldn't be initialized properly.\n");
    return 1;
  }
  printf("Initializing the Viterbi kernel...\n");
  if (!init_vit_kernel(sptr, vit_dict)) {
    printf("Error: the Viterbi decoding kernel couldn't be initialized properly.\n");
    return 1;
  }
  if ((num_Crit_test_tasks + num_Base_test_tasks) > 0) {
    if (!init_test_kernel(sptr, "")) { 
      printf("Error: the testing-task kernel couldn't be initialized properly.\n");
      return 1;
    }
  }

  if (crit_fft_samples_set >= num_radar_samples_sets) {
    printf("ERROR : Selected FFT Tasks from Radar Dictionary Set %u but there are only %u sets in the dictionary %s\n", crit_fft_samples_set, num_radar_samples_sets, rad_dict);
    print_usage(argv[0]);
    cleanup_and_exit(sptr, -1);
  }
    
//  if (global_scheduler_selection_policy > NUM_SELECTION_POLICIES) {
//    printf("ERROR : Selected Scheduler Policy (%u) is larger than number of policies (%u)\n", global_scheduler_selection_policy, NUM_SELECTION_POLICIES);
//    print_usage(argv[0]);
//    cleanup_and_exit(sptr, -1);
//  }
//  printf("Scheduler is using Policy %u : %s\n", global_scheduler_selection_policy, scheduler_selection_policy_str[global_scheduler_selection_policy]);
  
  /* We assume the vehicle starts in the following state:
   *  - Lane: center
   *  - Speed: 50 mph
   */
  vehicle_state.active  = true;
  vehicle_state.lane    = center;
  vehicle_state.speed   = 50;
  DEBUG(printf("\nVehicle starts with the following state: active: %u lane %u speed %.1f\n", vehicle_state.active, vehicle_state.lane, vehicle_state.speed));

  #ifdef USE_SIM_ENVIRON
  // In simulation mode, we could start the main car is a different state (lane, speed)
  if (init_sim_environs(world_desc_file_name, &vehicle_state) == error) {
    cleanup_and_exit(sptr, -1);
  }
  #endif

/*** MAIN LOOP -- iterates until all the traces are fully consumed ***/
  time_step = 0;
 #ifdef TIME
  struct timeval stop_prog, start_prog;

  struct timeval stop_iter_rad, start_iter_rad;
  struct timeval stop_iter_vit, start_iter_vit;
  struct timeval stop_iter_cv , start_iter_cv;
  struct timeval stop_iter_test , start_iter_test;
  struct timeval stop_iter_h264 , start_iter_h264;

  uint64_t iter_rad_sec = 0LL;
  uint64_t iter_vit_sec = 0LL;
  uint64_t iter_cv_sec  = 0LL;
  uint64_t iter_test_sec  = 0LL;
  uint64_t iter_h264_sec  = 0LL;

  uint64_t iter_rad_usec = 0LL;
  uint64_t iter_vit_usec = 0LL;
  uint64_t iter_cv_usec  = 0LL;
  uint64_t iter_test_usec  = 0LL;
  uint64_t iter_h264_usec  = 0LL;

  struct timeval stop_exec_rad, start_exec_rad;
  struct timeval stop_exec_vit, start_exec_vit;
  struct timeval stop_exec_cv , start_exec_cv;
  struct timeval stop_exec_test , start_exec_test;
  struct timeval stop_exec_h264 , start_exec_h264;

  uint64_t exec_rad_sec = 0LL;
  uint64_t exec_vit_sec = 0LL;
  uint64_t exec_cv_sec  = 0LL;
  uint64_t exec_test_sec  = 0LL;
  uint64_t exec_h264_sec  = 0LL;

  uint64_t exec_rad_usec = 0LL;
  uint64_t exec_vit_usec = 0LL;
  uint64_t exec_cv_usec  = 0LL;
  uint64_t exec_test_usec  = 0LL;
  uint64_t exec_h264_usec  = 0LL;

  uint64_t exec_get_rad_sec = 0LL;
  uint64_t exec_get_vit_sec = 0LL;
  uint64_t exec_get_cv_sec  = 0LL;
  uint64_t exec_get_test_sec  = 0LL;
  uint64_t exec_get_h264_sec  = 0LL;

  uint64_t exec_get_rad_usec = 0LL;
  uint64_t exec_get_vit_usec = 0LL;
  uint64_t exec_get_cv_usec  = 0LL;
  uint64_t exec_get_test_usec  = 0LL;
  uint64_t exec_get_h264_usec  = 0LL;

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
  /*init_accelerators_in_use_interval(sptr, start_prog);*/
 #endif

 #ifdef USE_SIM_ENVIRON
  DEBUG(printf("\n\nTime Step %d\n", time_step));
  while (iterate_sim_environs(vehicle_state))
 #else //TRACE DRIVEN MODE
  read_next_trace_record(sptr, vehicle_state);
  DEBUG(printf("Starting main while loop : max_stim_steps %u\n", max_time_steps));
  while ((time_step < max_time_steps) && (!eof_trace_reader()))
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
      hdep = iterate_h264_kernel(sptr, vehicle_state);
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
    label_t cv_tr_label = iterate_cv_kernel(sptr, vehicle_state);
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
    radar_dict_entry_t* rdentry_p = iterate_rad_kernel(sptr, vehicle_state);
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
    vit_dict_entry_t* vdentry_p = iterate_vit_kernel(sptr, vehicle_state);
   #ifdef TIME
    gettimeofday(&stop_iter_vit, NULL);
    iter_vit_sec  += stop_iter_vit.tv_sec  - start_iter_vit.tv_sec;
    iter_vit_usec += stop_iter_vit.tv_usec - start_iter_vit.tv_usec;
   #endif

    test_dict_entry_t* tdentry_p;
    if ((num_Crit_test_tasks + num_Base_test_tasks) > 0) {
     #ifdef TIME
      gettimeofday(&start_iter_test, NULL);
     #endif
      tdentry_p = iterate_test_kernel(sptr, vehicle_state);
     #ifdef TIME
      gettimeofday(&stop_iter_test, NULL);
      iter_test_sec  += stop_iter_test.tv_sec  - start_iter_test.tv_sec;
      iter_test_usec += stop_iter_test.tv_usec - start_iter_test.tv_usec;
     #endif
    }
    
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
    // Request a MetadataBlock (for an CV/CNN task at Critical Level)
    task_metadata_block_t* cv_mb_ptr = NULL;
    if (!no_crit_cnn_task) {
      do {
        cv_mb_ptr = get_task_metadata_block(sptr, cv_task_type, CRITICAL_TASK, cv_profile);
	//usleep(get_mb_holdoff);
     } while (0); // (cv_mb_ptr == NULL);
     #ifdef TIME
      struct timeval got_time;
      gettimeofday(&got_time, NULL);
      exec_get_cv_sec  += got_time.tv_sec  - start_exec_cv.tv_sec;
      exec_get_cv_usec += got_time.tv_usec - start_exec_cv.tv_usec;
     #endif
      if (cv_mb_ptr == NULL) {
        // We ran out of metadata blocks -- PANIC!
        printf("Out of metadata blocks for CV -- PANIC Quit the run (for now)\n");
	dump_all_metadata_blocks_states(sptr);
        exit (-4);
      }
      cv_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
      start_execution_of_cv_kernel(cv_mb_ptr, cv_tr_label); // Critical RADAR task    label = execute_cv_kernel(cv_tr_label);
    }
    if (!no_crit_cnn_task) {
      DEBUG(printf("CV/CNN task Block-ID = %u\n", cv_mb_ptr->block_id));
    }
   #ifdef TIME
    gettimeofday(&start_exec_rad, NULL);
   #endif
    // Request a MetadataBlock (for an FFT task at Critical Level)
      task_metadata_block_t* fft_mb_ptr = NULL;
      do {
        fft_mb_ptr = get_task_metadata_block(sptr, fft_task_type, CRITICAL_TASK, fft_profile[crit_fft_samples_set]);
	//usleep(get_mb_holdoff);
      } while (0); //(fft_mb_ptr == NULL);
     #ifdef TIME
      struct timeval got_time;
      gettimeofday(&got_time, NULL);
      exec_get_rad_sec  += got_time.tv_sec  - start_exec_rad.tv_sec;
      exec_get_rad_usec += got_time.tv_usec - start_exec_rad.tv_usec;
     #endif
    //printf("FFT Crit Profile: %e %e %e %e %e\n", fft_profile[crit_fft_samples_set][0], fft_profile[crit_fft_samples_set][1], fft_profile[crit_fft_samples_set][2], fft_profile[crit_fft_samples_set][3], fft_profile[crit_fft_samples_set][4]);
    if (fft_mb_ptr == NULL) {
      // We ran out of metadata blocks -- PANIC!
      printf("Out of metadata blocks for FFT -- PANIC Quit the run (for now)\n");
      dump_all_metadata_blocks_states(sptr);
      exit (-4);
    }
    fft_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
    start_execution_of_rad_kernel(fft_mb_ptr, radar_log_nsamples_per_dict_set[crit_fft_samples_set], radar_inputs); // Critical RADAR task
    DEBUG(printf("FFT task Block-ID = %u\n", fft_mb_ptr->block_id));
   #ifdef TIME
    gettimeofday(&start_exec_vit, NULL);
   #endif
    //NOTE Removed the num_messages stuff -- need to do this differently (separate invocations of this process per message)
    // Request a MetadataBlock for a Viterbi Task at Critical Level
    task_metadata_block_t* vit_mb_ptr = NULL;
    do {
      vit_mb_ptr = get_task_metadata_block(sptr, vit_task_type, CRITICAL_TASK, vit_profile[vit_msgs_size]);
      //usleep(get_mb_holdoff);
    } while (0); //(vit_mb_ptr == NULL);
   #ifdef TIME
    //struct timeval got_time;
    gettimeofday(&got_time, NULL);
    exec_get_vit_sec  += got_time.tv_sec  - start_exec_vit.tv_sec;
    exec_get_vit_usec += got_time.tv_usec - start_exec_vit.tv_usec;
   #endif
    if (vit_mb_ptr == NULL) {
      // We ran out of metadata blocks -- PANIC!
      printf("Out of metadata blocks for VITERBI -- PANIC Quit the run (for now)\n");
      dump_all_metadata_blocks_states(sptr);
      exit (-4);
    }
    vit_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
    start_execution_of_vit_kernel(vit_mb_ptr, vdentry_p); // Critical VITERBI task
    DEBUG(printf("VIT_TASK_BLOCK: ID = %u\n", vit_mb_ptr->block_id));

    task_metadata_block_t* test_mb_ptr = NULL;
    if (num_Crit_test_tasks > 0) {
     #ifdef TIME
      gettimeofday(&start_exec_test, NULL);
     #endif
      //NOTE Removed the num_messages stuff -- need to do this differently (separate invocations of this process per message)
      // Request a MetadataBlock for a Testerbi Task at Critical Level
      do {
	test_mb_ptr = get_task_metadata_block(sptr, test_task_type, CRITICAL_TASK, test_profile);
	//usleep(get_mb_holdoff);
      } while (0); //(test_mb_ptr == NULL);
     #ifdef TIME
      //struct timeval got_time;
      gettimeofday(&got_time, NULL);
      exec_get_test_sec  += got_time.tv_sec  - start_exec_test.tv_sec;
      exec_get_test_usec += got_time.tv_usec - start_exec_test.tv_usec;
     #endif
      if (test_mb_ptr == NULL) {
	// We ran out of metadata blocks -- PANIC!
	printf("Out of metadata blocks for TESTERBI -- PANIC Quit the run (for now)\n");
	dump_all_metadata_blocks_states(sptr);
	exit (-4);
      }
      test_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
      start_execution_of_test_kernel(test_mb_ptr, tdentry_p); // Critical TESTERBI task
      DEBUG(printf("TEST_TASK_BLOCK: ID = %u\n", test_mb_ptr->block_id));
    }
    
    // Now we add in the additional non-critical tasks...
    for (int i = 0; i < max_additional_tasks_per_time_step; i++) {
      // Aditional CV Tasks
      //for (int i = 0; i < additional_cv_tasks_per_time_step; i++) {
      if (i < additional_cv_tasks_per_time_step) {
       #ifdef TIME
        struct timeval get_time;
        gettimeofday(&get_time, NULL);
       #endif
        task_metadata_block_t* cv_mb_ptr_2 = NULL;
        do {
          cv_mb_ptr_2 = get_task_metadata_block(sptr, cv_task_type, BASE_TASK, cv_profile);
	  //usleep(get_mb_holdoff);
        } while (0); //(cv_mb_ptr_2 == NULL);
       #ifdef TIME
        struct timeval got_time;
        gettimeofday(&got_time, NULL);
        exec_get_cv_sec  += got_time.tv_sec  - get_time.tv_sec;
        exec_get_cv_usec += got_time.tv_usec - get_time.tv_usec;
       #endif
        if (cv_mb_ptr_2 == NULL) {
  	  printf("Out of metadata blocks for Non-Critical CV -- PANIC Quit the run (for now)\n");
  	  dump_all_metadata_blocks_states(sptr);
	  exit (-5);
        }
        cv_mb_ptr_2->atFinish = base_release_metadata_block;
        start_execution_of_cv_kernel(cv_mb_ptr_2, cv_tr_label); // NON-Critical RADAR task
      } // if (i < additional CV tasks)
      //for (int i = 0; i < additional_fft_tasks_per_time_step; i++) {
      if (i < additional_fft_tasks_per_time_step) {
        radar_dict_entry_t* rdentry_p2;
	if (task_size_variability == 0) {
	  rdentry_p2 = select_critical_radar_input(rdentry_p);
	} else {
	  rdentry_p2 = select_random_radar_input();
	  //printf("FFT select: Crit %u rdp2->set %u\n", crit_fft_samples_set, rdentry_p2->set);
	}
	int base_fft_samples_set = rdentry_p2->set;
	//printf("FFT Base Profile: %e %e %e %e %e\n", fft_profile[base_fft_samples_set][0], fft_profile[base_fft_samples_set][1], fft_profile[base_fft_samples_set][2], fft_profile[base_fft_samples_set][3], fft_profile[base_fft_samples_set][4]);
       #ifdef TIME
	struct timeval get_time;
	gettimeofday(&get_time, NULL);
       #endif
	task_metadata_block_t* fft_mb_ptr_2 = NULL;
        do {
	  fft_mb_ptr_2 = get_task_metadata_block(sptr, fft_task_type, BASE_TASK, fft_profile[base_fft_samples_set]);
	  //usleep(get_mb_holdoff);
        } while (0); //(fft_mb_ptr_2 == NULL);
       #ifdef TIME
        //struct timeval got_time;
        gettimeofday(&got_time, NULL);
	exec_get_rad_sec  += got_time.tv_sec  - get_time.tv_sec;
	exec_get_rad_usec += got_time.tv_usec - get_time.tv_usec;
       #endif
        if (fft_mb_ptr_2 == NULL) {
  	  printf("Out of metadata blocks for Non-Critical FFT -- PANIC Quit the run (for now)\n");
	  dump_all_metadata_blocks_states(sptr);
	  exit (-5);
        }
        fft_mb_ptr_2->atFinish = base_release_metadata_block;
	float* addl_radar_inputs = rdentry_p2->return_data;
	start_execution_of_rad_kernel(fft_mb_ptr_2, radar_log_nsamples_per_dict_set[crit_fft_samples_set], addl_radar_inputs); // NON-Critical RADAR task
      } // if (i < additional FFT tasks)

      //for (int i = 0; i < additional_vit_tasks_per_time_step; i++) {
      if (i < additional_vit_tasks_per_time_step) {
        vit_dict_entry_t* vdentry2_p;
	int base_msg_size;
        if (task_size_variability == 0) {
	  base_msg_size = vdentry_p->msg_num / NUM_MESSAGES;
	  int m_id = vdentry_p->msg_num % NUM_MESSAGES;
	  if (m_id != vdentry_p->msg_id) {
	    printf("WARNING: MSG_NUM %u : LNUM %u M_ID %u MSG_ID %u\n", vdentry_p->msg_num, base_msg_size, m_id, vdentry_p->msg_id);
          }
	  if (base_msg_size != vit_msgs_size) {
	    printf("WARNING: MSG_NUM %u : LNUM %u M_ID %u MSG_ID %u\n", vdentry_p->msg_num, base_msg_size, m_id, vdentry_p->msg_id);
	  }
	  vdentry2_p = select_specific_vit_input(base_msg_size, m_id);
        } else {
	  DEBUG(printf("Note: electing a random Vit Message for base-task\n"));
	  vdentry2_p = select_random_vit_input();
	  base_msg_size = vdentry2_p->msg_num / NUM_MESSAGES;
        }
       #ifdef TIME
        struct timeval get_time;
	gettimeofday(&get_time, NULL);
       #endif
        task_metadata_block_t* vit_mb_ptr_2 = NULL;
        do {
          vit_mb_ptr_2 = get_task_metadata_block(sptr, vit_task_type, BASE_TASK, vit_profile[base_msg_size]);
	  //usleep(get_mb_holdoff);
        } while (0); // (vit_mb_ptr_2 == NULL);
       #ifdef TIME
        struct timeval got_time;
	gettimeofday(&got_time, NULL);
	exec_get_vit_sec  += got_time.tv_sec  - get_time.tv_sec;
	exec_get_vit_usec += got_time.tv_usec - get_time.tv_usec;
       #endif
        if (vit_mb_ptr_2 == NULL) {
  	  printf("Out of metadata blocks for Non-Critical VIT -- PANIC Quit the run (for now)\n");
	  dump_all_metadata_blocks_states(sptr);
	  exit (-5);
        }
        vit_mb_ptr_2->atFinish = base_release_metadata_block;
        start_execution_of_vit_kernel(vit_mb_ptr_2, vdentry2_p); // Non-Critical VITERBI task
      } // if (i < Additional VIT tasks)

      // Non-Critical (base) TEST-Tasks
      if (i < num_Base_test_tasks) { 
       #ifdef TIME
        struct timeval get_time;
	gettimeofday(&get_time, NULL);
       #endif
        task_metadata_block_t* test_mb_ptr_2 = NULL;
        do {
          test_mb_ptr_2 = get_task_metadata_block(sptr, test_task_type, BASE_TASK, test_profile);
	  //usleep(get_mb_holdoff);
        } while (0); // (test_mb_ptr_2 == NULL);
       #ifdef TIME
        struct timeval got_time;
	gettimeofday(&got_time, NULL);
	exec_get_test_sec  += got_time.tv_sec  - get_time.tv_sec;
	exec_get_test_usec += got_time.tv_usec - get_time.tv_usec;
       #endif
        if (test_mb_ptr_2 == NULL) {
  	  printf("Out of metadata blocks for Non-Critical TEST -- PANIC Quit the run (for now)\n");
	  dump_all_metadata_blocks_states(sptr);
	  exit (-5);
        }
        test_mb_ptr_2->atFinish = base_release_metadata_block;
        start_execution_of_test_kernel(test_mb_ptr_2, tdentry_p); // Non-Critical TESTERBI task
      } // if (i < Additional TEST tasks)
    } // for (i over MAX_additional_tasks)
   #ifdef TIME
    gettimeofday(&start_wait_all_crit, NULL);
   #endif

    DEBUG(printf("MAIN: Calling wait_all_critical\n"));
    wait_all_critical(sptr);

   #ifdef TIME
    gettimeofday(&stop_wait_all_crit, NULL);
    wait_all_crit_sec  += stop_wait_all_crit.tv_sec  - start_wait_all_crit.tv_sec;
    wait_all_crit_usec += stop_wait_all_crit.tv_usec - start_wait_all_crit.tv_usec;
   #endif
    
    distance = finish_execution_of_rad_kernel(fft_mb_ptr);
    message = finish_execution_of_vit_kernel(vit_mb_ptr);
    if (!no_crit_cnn_task) {
      label = finish_execution_of_cv_kernel(cv_mb_ptr);
    }
    if (num_Crit_test_tasks > 0) {
      test_res = finish_execution_of_test_kernel(test_mb_ptr);
    }
   #ifdef TIME
    gettimeofday(&stop_exec_rad, NULL);
    exec_rad_sec  += stop_exec_rad.tv_sec  - start_exec_rad.tv_sec;
    exec_rad_usec += stop_exec_rad.tv_usec - start_exec_rad.tv_usec;
    exec_vit_sec  += stop_exec_rad.tv_sec  - start_exec_vit.tv_sec;
    exec_vit_usec += stop_exec_rad.tv_usec - start_exec_vit.tv_usec;
    exec_cv_sec   += stop_exec_rad.tv_sec  - start_exec_cv.tv_sec;
    exec_cv_usec  += stop_exec_rad.tv_usec - start_exec_cv.tv_usec;
   #endif

    // POST-EXECUTE each kernel to gather stats, etc.
    /*if (!bypass_h264_functions) {
      post_execute_h264_kernel();
      }*/
    if (!no_crit_cnn_task) {
      post_execute_cv_kernel(cv_tr_label, label);
    }
    post_execute_rad_kernel(rdentry_p->set, rdentry_p->index_in_set, rdict_dist, distance);
    post_execute_vit_kernel(vdentry_p->msg_id, message);
    if (num_Crit_test_tasks > 0) {
      post_execute_test_kernel(TEST_TASK_DONE, test_res);
    }
    
    /* The plan_and_control() function makes planning and control decisions
     * based on the currently perceived information. It returns the new
     * vehicle state.
     */
    DEBUG(printf("Time Step %3u : Calling Plan and Control %u times with message %u and distance %.1f\n", time_step, pandc_repeat_factor, message, distance));
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

    time_step++;

    // TEST - trying this here.
    //wait_all_tasks_finish();

    #ifndef USE_SIM_ENVIRON
    read_next_trace_record(sptr, vehicle_state);
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
  closeout_test_kernel();

 #ifndef USE_SIM_ENVIRON
  closeout_trace_reader();
 #endif

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
    uint64_t exec_get_rad   = (uint64_t) (exec_get_rad_sec) * 1000000 + (uint64_t) (exec_get_rad_usec);
    uint64_t exec_get_vit   = (uint64_t) (exec_get_vit_sec) * 1000000 + (uint64_t) (exec_get_vit_usec);
    uint64_t exec_get_cv    = (uint64_t) (exec_get_cv_sec)  * 1000000 + (uint64_t) (exec_get_cv_usec);
    //uint64_t exec_get_h264  = (uint64_t) (exec_h264_sec) * 1000000 + (uint64_t) (exec_h264_usec);
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
    printf("    GET_MB execute_rad_kernel run time  %lu usec\n", exec_rad);
    printf("    GET_MB execute_vit_kernel run time  %lu usec\n", exec_vit);
    printf("    GET_MB execute_cv_kernel run time   %lu usec\n", exec_cv);
    //printf("  execute_h264_kernel run time      %lu usec\n", exec_h264);
    printf("  plan_and_control run time         %lu usec at %u factor\n", exec_pandc, pandc_repeat_factor);
    printf("  wait_all_critical run time        %lu usec\n", wait_all_crit);
  }
 #endif // TIME
  shutdown_scheduler(sptr);
  free(sched_inparms);
  printf("\nDone.\n");
  return 0;
}
