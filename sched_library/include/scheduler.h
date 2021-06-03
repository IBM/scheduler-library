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

#ifndef H_SCHEDULER_INTERFACE_H
#define H_SCHEDULER_INTERFACE_H

#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>

//#include "base_types.h"
typedef enum { success, error } status_t;

typedef enum {
  SCHED_CPU_ACCEL_T = 0,
  SCHED_EPOCHS_1D_FFT_ACCEL_T,
  SCHED_EPOCHS_VITDEC_ACCEL_T,
  SCHED_EPOCHS_CV_CNN_ACCEL_T,
  SCHED_MAX_ACCEL_TYPES,
} scheduler_accelerator_type;

// A value that stands in for Infinite Time for task profiles.
#define ACINFPROF 0x0f00deadbeeff00d // A recognizable "infinite-time" value

// These are "global" Scheduler-defined fields, etc.
enum { NO_Task = -1 } task_type_enum_t;
typedef int task_type_t;

enum { NO_Accelerator = -1 };
typedef int accelerator_type_t;

typedef enum {
  BASE_TASK = 1,
  ELEVATED_TASK = 2,
  CRITICAL_TASK = 3,
  NUM_TASK_CRIT_LEVELS
} task_criticality_t;

typedef enum {
  TASK_FREE = 0,
  TASK_ALLOCATED,
  TASK_QUEUED,
  TASK_RUNNING,
  TASK_DONE,
  NUM_TASK_STATUS
} task_status_t;

#define MAX_TASK_NAME_LEN 32
#define MAX_TASK_DESC_LEN 256

#define MAX_ACCEL_NAME_LEN 32
#define MAX_ACCEL_DESC_LEN 256

/* typedef struct { */
/*   scheduler_accelerator_type    accel; */
/*   task_type_t                   task; */
/*   sched_execute_task_function_t fptr; */
/* } exec_task_on_accel_tuple_t; */

typedef struct {
  unsigned max_task_types; // The max number of task types that might be used in
                           // this run/usage
  unsigned max_accel_types; // The max number of accelerator type that might be
                            // used in this run/usage

  unsigned max_metadata_pool_blocks; // The max number of Metadata blocks that
                                     // can be used during this run/usage
  unsigned max_data_space_bytes; // The max number of Data (Memory) bytes that
                                 // any task can use this run/usage (i.e.
                                 // Metadata Block In/Out memory size)

  unsigned max_task_timing_sets; // The max number of gettime timing sets the
                                 // MBs can track per this run/usage (per MB) --
                                 // one per task accelerator target...

  unsigned scheduler_holdoff_usec;

  char policy[256];

  bool visualizer_output_enabled;
  int32_t visualizer_task_start_count;
  int32_t visualizer_task_stop_count;
  task_type_t visualizer_task_enable_type;

  char sl_viz_fname[256];

  // Indicates the max number accelerators that should be made available for use
  // for each possible accelerator pool
  //  Note: the default is '0' so one only needs to set those that are used.
  int max_accel_to_use_from_pool[SCHED_MAX_ACCEL_TYPES];

} scheduler_get_datastate_in_parms_t;

// This is a timing analysis structure for the scheduler functions, etc.
typedef struct {
  struct timeval idle_start;
  uint64_t idle_sec, idle_usec;
  struct timeval get_start;
  uint64_t get_sec, get_usec;
  struct timeval queued_start;
  uint64_t queued_sec, queued_usec;
  struct timeval running_start;
  uint64_t *running_sec;  //[MAX_ACCEL_TYPES];
  uint64_t *running_usec; //[MAX_ACCEL_TYPES];
  struct timeval done_start;
  uint64_t done_sec, done_usec;
} sched_timing_data_t;

typedef struct { // This allows each task to track up to 16 total internal task
                 // timings...
                 /*   struct timeval time_val[MAX_TASK_TIMING_SETS]; */
  /*   uint64_t time_sec[MAX_TASK_TIMING_SETS][MAX_ACCEL_TYPES]; */
  /*   uint64_t time_usec[MAX_TASK_TIMING_SETS][MAX_ACCEL_TYPES]; */
  uint64_t timing_data[160];
} task_timing_data_t;

// This is a metadata structure; it is used to hold all information for any task
//  to be invoked through the scheduler.  This includes a description of the
//  task type, and all input/output data space for the task
// The task types are defined when the application registers them.
// The data (i.e. inputs, outputs, etc. ) are transferred here as a "bulk data"
//  memory (of abstract uint8_t or bytes) and a size.  The interpretation of
//  this block of data is task-dependent, and can have an over-laid structure,
//  etc.

struct scheduler_datastate_block_struct;

typedef struct task_metadata_entry_struct {
  // This points to the scheduler datastate structure (defiuned below) to which
  // this metadata block belongs.
  struct scheduler_datastate_block_struct *scheduler_datastate_pointer;

  // This portion is management, control, and scheduler stuff...
  int32_t block_id; // master-pool-index; a unique ID per metadata task
  task_status_t
      status; // -1 = free, 0 = allocated, 1 = queued, 2 = running, 3 = done ?
  pthread_t thread_id; // set when we invoke pthread_create (at least for CPU)
  pthread_mutex_t metadata_mutex; // Used to guard access to altering metadata
                                  // conditional variables
  pthread_cond_t metadata_condv;  // These phthreads conditional variables are
                                  // used to "signal" a thread to do work

  accelerator_type_t
      accelerator_type; // indicates which accelerator type is being used (id's
                        // set at accelerator registration)
  int32_t accelerator_id; // indicates which (of the N of that type) accelerator
                          // this task is executing on
  task_type_t task_type;  // An indication of the task type; defined when tasks
                          // are registeres
  int32_t task_id; // A unique identifier for this task (across the full run)
  int32_t dag_id;  // Indicates which DAG spawns or owns this task
  task_criticality_t
      crit_level; // [0 .. 3] -- see above enumeration ("Base" to "Critical")

  uint64_t *task_on_accel_profile; //[MAX_ACCEL_TYPES];  //Timing profile for
                                   //task (in usec) -- maps task projected time
                                   //on accelerator...

  void (*atFinish)(
      struct task_metadata_entry_struct *); // Call-back Finish-time function

  // Statistics
  uint32_t *gets_by_task_type;  // Count of times this metadata block allocated
                                // per task type.
  uint32_t *frees_by_task_type; // Count of times this metadata block allocated
                                // per task type.

  // The number of tiumes this MB was allocated to use each accelerator in the
  // system
  uint32_t **accelerator_allocated_to_MB; //[MAX_ACCEL_TYPES]; //
                                          //[MAX_ACCEL_OF_ANY_TYPE];

  // These are timing-related storage; currently we keep per-task-type in each
  // metadata to aggregate (per block) over the run
  sched_timing_data_t sched_timings;
  uint32_t **task_computed_on; //[MAX_ACCEL_TYPES]; // array over TASK_TYPES
  task_timing_data_t *task_timings; // array over TASK_TYPES

  // This is the segment for data for the tasks
  int32_t data_size;   // Number of bytes occupied in data (in case the task
                       // evaluation wants to know)
  uint8_t *data_space; // The total data space for the metadata block (holds ALL
                       // data for the task)
} task_metadata_block_t;

// This is the Ready Task Queue -- it holds Metadata Block IDs
typedef struct ready_mb_task_queue_entry_struct {
  short unique_id;
  short block_id;
  struct ready_mb_task_queue_entry_struct *next;
  struct ready_mb_task_queue_entry_struct *prev;
} ready_mb_task_queue_entry_t;

// This is a typedef for the call-back function, called by the scheduler at
// finish time for a task
typedef void (*task_finish_callback_t)(task_metadata_block_t *);

// This is a typedef for an execution function called by the scheduler (e.g. to
// execute a task)
typedef void (*sched_execute_task_function_t)(task_metadata_block_t *);

// These are function pointer prototype declaration types, used for the
// regsiter_task_type routine.
typedef void (*print_metadata_block_contents_t)(
    /*task_metadata_block_t*/ void *);
typedef void (*output_task_type_run_stats_t)(
    /*struct scheduler_datastate_block_struct*/ void *sptr,
    unsigned my_task_type, unsigned total_accel_types);

typedef /*task_metadata_block_t*/ void *(*set_up_task_function_t)(
    /*struct scheduler_datastate_block_struct*/ void *sptr,
    task_type_t the_task_type, task_criticality_t crit_level, bool auto_finish,
    int32_t dag_id, ...);
typedef void (*finish_task_execution_function_t)(
    /*task_metadata_block_t*/ void *the_metadata_block, ...);
typedef void (*auto_finish_task_function_t)(/*task_metadata_block_t*/ void *mb);

// These are function pointer prototype declaration types, used for the
// regsiter_accelerator_type routine.
typedef void (*do_accel_initialization_t)(
    struct scheduler_datastate_block_struct *sptr);
typedef void (*do_accel_closeout_t)(
    struct scheduler_datastate_block_struct *sptr);
typedef void (*output_accel_run_stats_t)(
    struct scheduler_datastate_block_struct *sptr, unsigned my_accel_id,
    unsigned total_task_types);

// This typedef defines a structure used to describe a accelerator (for the
// register_accelerator_type routine)
typedef struct accel_pool_defn_info_struct {
  do_accel_initialization_t do_accel_initialization;
  do_accel_closeout_t do_accel_closeout;
  output_accel_run_stats_t output_accel_run_stats;
  unsigned number_available;
  char name[MAX_ACCEL_NAME_LEN];
  char description[MAX_ACCEL_DESC_LEN];
} accelerator_pool_defn_info_t;

typedef struct bi_ll_struct {
  int clt_block_id;
  struct bi_ll_struct *next;
} blockid_linked_list_t;

typedef struct scheduler_datastate_block_struct {
  // These are limits (e.g. max-task-types) for this instantiation of the
  // scheduler datasatate space
  scheduler_get_datastate_in_parms_t *inparms;

  int max_accel_of_any_type; // The max number of accelerators allocated from
                             // any of the pools

  task_type_t next_avail_task_type;
  accelerator_type_t next_avail_accel_id;

  int32_t next_avail_DAG_id;

  // inparm: bool        visualizer_output_enabled;
  // inparm: int32_t     visualizer_task_start_count;
  // inparm: int32_t     visualizer_task_stop_count;
  // inparm: task_type_t visualizer_task_enable_type;
  bool visualizer_output_started;
  uint64_t visualizer_start_time_usec;

  // unsigned scheduler_holdoff_usec;

  // Handle for the dynamically loaded policy
  void *policy_handle;
  // Function pointer to the policy initialization routine
  void (*initialize_assign_task_to_pe)(void *in_parm_ptr);
  // Function pointer for the policy's assign_task_to_pe() function
  ready_mb_task_queue_entry_t *(*assign_task_to_pe)(
      struct scheduler_datastate_block_struct *sptr,
      ready_mb_task_queue_entry_t *ready_task_entry);
  // inparm: char policy[256];

  // The pool of metadata blocks for use by the tasks, etc.
  unsigned total_metadata_pool_blocks;
  task_metadata_block_t *master_metadata_pool;

  pthread_mutex_t free_metadata_mutex; // Used to guard access to altering the
                                       // free-list metadata information, etc.
  int free_metadata_blocks;
  int *free_metadata_pool;
  uint32_t *allocated_metadata_blocks; // array over TASK_TYPES
  uint32_t *freed_metadata_blocks;     // array over TASK_TYPES

  pthread_mutex_t task_queue_mutex; // Used to guard access to altering the
                                    // ready-task-queue contents
  ready_mb_task_queue_entry_t *ready_mb_task_queue_pool;
  ready_mb_task_queue_entry_t *free_ready_mb_task_queue_entries;
  ready_mb_task_queue_entry_t *ready_mb_task_queue_head;
  ready_mb_task_queue_entry_t *ready_mb_task_queue_tail;
  unsigned num_free_task_queue_entries;
  unsigned num_tasks_in_ready_queue;

  pthread_mutex_t accel_alloc_mutex; // Used to guard access to altering the
                                     // accelerator allocations

  // ASCII trace for Scheduler-Visualization Trace Output
  FILE *sl_viz_fp;
  pthread_mutex_t sl_viz_out_mutex; // Used to guard access to writing the
                                    // sl_viz output entries.

  pthread_t *metadata_threads;

  // pthread_mutex_t schedule_from_queue_mutex;   // Used to guard access to
  // scheduling functionality
  pthread_t scheduling_thread;

  blockid_linked_list_t *critical_live_task_head;
  blockid_linked_list_t *critical_live_tasks_list;
  int *free_critlist_pool;
  int free_critlist_entries;
  int total_critical_tasks;

  char **task_name_str; // array over TASK_TYPES and of size MAX_TASK_NAME_LEN
  char **task_desc_str; // array over TASK_TYPES and of size MAX_TASK_DESC_LEN

  char *
      *accel_name_str; // array over ACCEL_TYPES and of size MAX_ACCEL_NAME_LEN
  char *
      *accel_desc_str; // array over ACCEL_TYPES and of size MAX_ACCEL_DESC_LEN

  char task_criticality_str[NUM_TASK_CRIT_LEVELS][32];
  char task_status_str[NUM_TASK_STATUS][32];

  // This is a table of the execution functions for the various Task Types in
  // the scheduler
  //  We set this up with one "set" of entries per JOB_TYPE
  //   where each set has one execute function per possible TASK TARGET (on
  //   which it can execute) Currently the targets are "CPU" and "HWR" -- this
  //   probably has to change (though this interpretation is only convention).
  sched_execute_task_function_t *
      *scheduler_execute_task_function; //[MAX_ACCEL_TYPES]; // array over
                                        //TASK_TYPES

  print_metadata_block_contents_t
      *print_metablock_contents_function; // array over TASK_TYPES
  output_task_type_run_stats_t
      *output_task_run_stats_function;          // array over TASK_TYPES
  set_up_task_function_t *set_up_task_function; // array over TASK_TYPES
  finish_task_execution_function_t
      *finish_task_execution_function; // array over TASK_TYPES
  auto_finish_task_function_t
      *auto_finish_task_function; // array over TASK_TYPES

  do_accel_initialization_t *do_accel_init_function; //[MAX_ACCEL_TYPES];
  do_accel_closeout_t *do_accel_closeout_function;   //[MAX_ACCEL_TYPES];
  output_accel_run_stats_t
      *output_accel_run_stats_function; //[MAX_ACCEL_TYPES];

  int map_sched_accel_type_to_local_accel_type[SCHED_MAX_ACCEL_TYPES];
  volatile int *
      *accelerator_in_use_by;    //[MAX_ACCEL_TYPES]; //[MAX_ACCEL_OF_ANY_TYPE];
  int *num_accelerators_of_type; //[MAX_ACCEL_TYPES];

  /*struct timeval last_accel_use_update_time;
    uint64_t
    in_use_accel_times_array[NUM_CPU_ACCEL+1][NUM_FFT_ACCEL+1][NUM_VIT_ACCEL+1][NUM_CV_ACCEL+1];*/

  // Scheduler Library statistics
  uint64_t scheduler_decision_time_usec;
  uint64_t scheduler_decisions;
  uint64_t scheduler_decision_checks;

} scheduler_datastate_block_t;

extern scheduler_get_datastate_in_parms_t *
get_scheduler_datastate_input_parms();
extern scheduler_datastate_block_t *
initialize_scheduler_and_return_datastate_pointer(
    scheduler_get_datastate_in_parms_t *inp);
extern scheduler_datastate_block_t *
initialize_scheduler_from_config_file(char *config_file_name);

extern void set_up_scheduler();

extern task_metadata_block_t *
get_task_metadata_block(scheduler_datastate_block_t *sptr, int32_t dag_id,
                        task_type_t of_task_type, task_criticality_t crit_level,
                        uint64_t *task_profile);
extern void free_task_metadata_block(task_metadata_block_t *mb);

extern auto_finish_task_function_t
get_auto_finish_routine(scheduler_datastate_block_t *sptr,
                        task_type_t the_task_type);

extern void request_execution(void *task_metadata_block);
// extern int get_task_status(scheduler_datastate_block_t* sptr, int task_id);
extern void wait_all_critical(scheduler_datastate_block_t *sptr);
extern void wait_all_tasks_finish(scheduler_datastate_block_t *sptr);
extern void wait_on_tasklist(scheduler_datastate_block_t *sptr, int num_tasks,
                             ...);
extern void mark_task_done(task_metadata_block_t *task_metadata_block);

extern void print_base_metadata_block_contents(task_metadata_block_t *mb);
extern void dump_all_metadata_blocks_states(scheduler_datastate_block_t *sptr);

extern void shutdown_scheduler(scheduler_datastate_block_t *sptr);

extern void init_accelerators_in_use_interval(scheduler_datastate_block_t *sptr,
                                              struct timeval start_prog);

extern void cleanup_and_exit(scheduler_datastate_block_t *sptr, int rval);

task_type_t register_task_type(
    void *sptr, char *task_name, char *task_description,
    set_up_task_function_t set_up_task_func,
    finish_task_execution_function_t finish_task_func,
    auto_finish_task_function_t auto_finish_func,
    print_metadata_block_contents_t
        print_metadata_block_contents,                       // function pointer
    output_task_type_run_stats_t output_task_type_run_stats, // function pointer
    int num_accel_task_exec_descriptions, ...);

extern void *set_up_task(void *sptr, task_type_t the_task_type,
                         task_criticality_t crit_level, int use_auto_finish,
                         int32_t dag_id, ...);

extern void finish_task_execution(void *the_metadata_block, ...);

#endif
