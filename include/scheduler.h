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

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

#include "base_types.h"

// Some Profiling Data:
#define ACINFPROF  0x0f00deadbeeff00d    // A recognizable "infinite-time" value

// These are "global" Scheduler-defined fields, etc.
enum { NO_Task = -1} task_id_enum_t;
typedef int task_id_t;

enum {NO_Accelerator = -1};
typedef int accelerator_type_t;

typedef enum { NO_TASK   = 0,
	       BASE_TASK = 1,
	       ELEVATED_TASK = 2,
	       CRITICAL_TASK = 3,
	       NUM_TASK_CRIT_LEVELS} task_criticality_t;


typedef enum { TASK_FREE = 0,
	       TASK_ALLOCATED,
	       TASK_QUEUED,
	       TASK_RUNNING,
	       TASK_DONE,
	       NUM_TASK_STATUS} task_status_t;

#define MAX_TASK_NAME_LEN   32
#define MAX_TASK_DESC_LEN   256

#define MAX_ACCEL_NAME_LEN   32
#define MAX_ACCEL_DESC_LEN   256

// These are fields defined by the application when it gets/sets up a new cheduler datastate block
#define MAX_TASK_TYPES     4
#define MAX_ACCEL_TYPES    5
#define GLOBAL_METADATA_POOL_BLOCKS 32
#define MAX_TASK_TIMING_SETS   16
#define MAX_DATA_SPACE_BYTES   128*1024

typedef struct {
  unsigned max_task_types;	// The max number of task types that might be used in this run/usage
  unsigned max_accel_types;	// The max number of accelerator type that might be used in this run/usage

  unsigned max_metadata_pool_blocks; // The max number of Metadata blocks that can be used during this run/usage
  unsigned max_data_space_bytes; // The max number of Data (Memory) bytes that any task can use this run/usage (i.e. Metadata Block In/Out memory size)

  unsigned max_task_timing_sets; // The max number of gettime timing sets the MBs can track per this run/usage (per MB) -- one per task accelerator target...

  unsigned max_accel_of_any_type; // The max number of accelerators of any type that might be used in this run/usage
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
  uint64_t running_sec[MAX_ACCEL_TYPES];
  uint64_t running_usec[MAX_ACCEL_TYPES];
  struct timeval done_start;
  uint64_t done_sec, done_usec;
} sched_timing_data_t;

typedef struct { // This allows each task to track up to 16 total internal task timings...
  struct timeval time_val[MAX_TASK_TIMING_SETS];
  uint32_t comp_by[MAX_ACCEL_TYPES]; 
  uint64_t time_sec[MAX_TASK_TIMING_SETS*MAX_ACCEL_TYPES];
  uint64_t time_usec[MAX_TASK_TIMING_SETS*MAX_ACCEL_TYPES];
} task_timing_data_t;

// This is a metadata structure; it is used to hold all information for any task
//  to be invoked through the scheduler.  This includes a description of the
//  task type, and all input/output data space for the task
// The task types are defined when the application registers them.
// The data (i.e. inputs, outputs, etc. ) are transferred here as a "bulk data"
//  memory (of abstract uint8_t or bytes) and a size.  The interpretation of this
//  block of data is task-dependent, and can have an over-laid structure, etc.

struct scheduler_datastate_block_struct;

typedef struct task_metadata_entry_struct {
  // This points to the scheduler datastate structure (defiuned below) to which this metadata block belongs.
  struct scheduler_datastate_block_struct* scheduler_datastate_pointer;

  // This portion is management, control, and scheduler stuff...
  int32_t         block_id;       // master-pool-index; a unique ID per metadata task
  task_status_t   status;         // -1 = free, 0 = allocated, 1 = queued, 2 = running, 3 = done ?
  pthread_t       thread_id;      // set when we invoke pthread_create (at least for CPU)
  pthread_mutex_t metadata_mutex; // Used to guard access to altering metadata conditional variables
  pthread_cond_t  metadata_condv; // These phthreads conditional variables are used to "signal" a thread to do work

  accelerator_type_t  accelerator_type; // indicates which accelerator type is being used (id's set at accelerator registration)
  int32_t  accelerator_id;        // indicates which accelerator this task is executing on
  task_id_t task_type;            // An indication of the task type; defined when tasks are registeres
  task_criticality_t crit_level;  // [0 .. 3] -- see above enumeration ("Base" to "Critical")

  uint64_t task_on_accel_profile[MAX_ACCEL_TYPES];  //Timing profile for task (in usec) -- maps task projected time on accelerator...

  void (*atFinish)(struct task_metadata_entry_struct *); // Call-back Finish-time function

  uint32_t gets_by_task_type[MAX_TASK_TYPES]; // Count of times this metadata block allocated per job type.
  uint32_t frees_by_task_type[MAX_TASK_TYPES]; // Count of times this metadata block allocated per job type.

  // These are timing-related storage; currently we keep per-task-type in each metadata to aggregate (per block) over the run
  sched_timing_data_t sched_timings;
  task_timing_data_t  task_timings[MAX_TASK_TYPES];  // This allows for N types of tasks (e.g. FFT, Viterbi, etc.)

  // This is the segment for data for the tasks
  int32_t  data_size;                // Number of bytes occupied in data (NOT USED/NOT NEEDED?)
  uint8_t  data_space[MAX_DATA_SPACE_BYTES];
} task_metadata_block_t;

// This is the Ready Task Queue -- it holds Metadata Block IDs
typedef struct ready_mb_task_queue_entry_struct {
  short  unique_id;
  short  block_id;
  struct ready_mb_task_queue_entry_struct * next;
  struct ready_mb_task_queue_entry_struct * prev;
} ready_mb_task_queue_entry_t;

// This is a typedef for the call-back function, called by the scheduler at finish time for a task
typedef void (*task_finish_callback_t)(task_metadata_block_t*);

// This is a typedef for an execution function called by the scheduler (e.g. to execute a task)
typedef void (*sched_execute_task_function_t)(task_metadata_block_t*);

// These are function pointer prototype declaration types, used for the regsiter_task_type routine.
typedef void (*print_metadata_block_contents_t)(task_metadata_block_t*);
typedef void (*output_task_type_run_stats_t)(struct scheduler_datastate_block_struct* sptr, unsigned my_task_id, unsigned total_accel_types);

// This typedef defines a structure used to describe a task (for the register_task_type routine)
typedef struct task_type_defn_info_struct {
  print_metadata_block_contents_t print_metadata_block_contents;
  output_task_type_run_stats_t    output_task_type_run_stats;
  char                            name[MAX_TASK_NAME_LEN];
  char                            description[MAX_TASK_DESC_LEN];
} task_type_defn_info_t;

// These are function pointer prototype declaration types, used for the regsiter_accelerator_type routine.
typedef void (*do_accel_initialization_t)(struct scheduler_datastate_block_struct* sptr);
typedef void (*do_accel_closeout_t)(struct scheduler_datastate_block_struct* sptr);
typedef void (*output_accel_run_stats_t)(struct scheduler_datastate_block_struct* sptr, unsigned my_accel_id, unsigned total_task_types);

// This typedef defines a structure used to describe a accelerator (for the register_accelerator_type routine)
typedef struct accel_pool_defn_info_struct {
  do_accel_initialization_t       do_accel_initialization;
  do_accel_closeout_t             do_accel_closeout;
  output_accel_run_stats_t        output_accel_run_stats;
  unsigned                        number_available;
  char                            name[MAX_ACCEL_NAME_LEN];
  char                            description[MAX_ACCEL_DESC_LEN];
} accelerator_pool_defn_info_t;


typedef struct bi_ll_struct { int clt_block_id;  struct bi_ll_struct* next; } blockid_linked_list_t;


typedef struct scheduler_datastate_block_struct {
  // These are limits (e.g. max-task-types) for this instantiation of the scheduler datasatate space
  scheduler_get_datastate_in_parms_t limits;

  task_id_t next_avail_task_id;
  accelerator_type_t next_avail_accel_id;

  unsigned int scheduler_holdoff_usec;

  // Handle for the dynamically loaded policy
  void *policy_handle;
  // Function pointer for the policy's assign_task_to_pe() function
  ready_mb_task_queue_entry_t *
  (*assign_task_to_pe)(struct scheduler_datastate_block_struct* sptr, ready_mb_task_queue_entry_t* ready_task_entry);
  char policy[256];

  // The pool of metadata blocks for use by the tasks, etc.
  unsigned total_metadata_pool_blocks;
  task_metadata_block_t master_metadata_pool[GLOBAL_METADATA_POOL_BLOCKS];

  pthread_mutex_t free_metadata_mutex; // Used to guard access to altering the free-list metadata information, etc.
  int free_metadata_blocks;
  int free_metadata_pool[GLOBAL_METADATA_POOL_BLOCKS];
  unsigned allocated_metadata_blocks[MAX_TASK_TYPES];
  unsigned freed_metadata_blocks[MAX_TASK_TYPES];

  pthread_mutex_t task_queue_mutex;   // Used to guard access to altering the ready-task-queue contents
  ready_mb_task_queue_entry_t ready_mb_task_queue_pool[GLOBAL_METADATA_POOL_BLOCKS];
  ready_mb_task_queue_entry_t* free_ready_mb_task_queue_entries;
  ready_mb_task_queue_entry_t* ready_mb_task_queue_head;
  ready_mb_task_queue_entry_t* ready_mb_task_queue_tail;
  unsigned num_free_task_queue_entries;
  unsigned num_tasks_in_ready_queue;

  pthread_mutex_t accel_alloc_mutex;   // Used to guard access to altering the accelerator allocations

  pthread_t metadata_threads[GLOBAL_METADATA_POOL_BLOCKS]; // One thread per metadata block (to exec it in)

  //pthread_mutex_t schedule_from_queue_mutex;   // Used to guard access to scheduling functionality
  pthread_t scheduling_thread;

  blockid_linked_list_t* critical_live_task_head;
  blockid_linked_list_t  critical_live_tasks_list[GLOBAL_METADATA_POOL_BLOCKS];
  int free_critlist_pool[GLOBAL_METADATA_POOL_BLOCKS];
  int free_critlist_entries;
  int total_critical_tasks;

  char task_name_str[MAX_TASK_TYPES][MAX_TASK_NAME_LEN];
  char task_desc_str[MAX_TASK_TYPES][MAX_TASK_DESC_LEN];

  char accel_name_str[MAX_ACCEL_TYPES][MAX_ACCEL_NAME_LEN];
  char accel_desc_str[MAX_ACCEL_TYPES][MAX_ACCEL_DESC_LEN];

  char task_criticality_str[NUM_TASK_CRIT_LEVELS][32];
  char task_status_str[NUM_TASK_STATUS][32];

  // This is a table of the execution functions for the various Task Types in the scheduler
  //  We set this up with one "set" of entries per JOB_TYPE
  //   where each set has one execute function per possible TASK TARGET (on which it can execute)
  //   Currently the targets are "CPU" and "HWR" -- this probably has to change (though this interpretation is only convention).
  sched_execute_task_function_t scheduler_execute_task_function[MAX_TASK_TYPES][MAX_ACCEL_TYPES];

  print_metadata_block_contents_t print_metablock_contents_function[MAX_TASK_TYPES];
  output_task_type_run_stats_t output_task_run_stats_function[MAX_TASK_TYPES];

  do_accel_initialization_t do_accel_init_function[MAX_ACCEL_TYPES];
  do_accel_closeout_t do_accel_closeout_function[MAX_ACCEL_TYPES];
  output_accel_run_stats_t output_accel_run_stats_function[MAX_ACCEL_TYPES];

  volatile int accelerator_in_use_by[MAX_ACCEL_TYPES-1][MAX_ACCEL_OF_EACH_TYPE];
  unsigned int accelerator_allocated_to_MB[MAX_ACCEL_TYPES-1][MAX_ACCEL_OF_EACH_TYPE][GLOBAL_METADATA_POOL_BLOCKS];
  int num_accelerators_of_type[MAX_ACCEL_TYPES];

  /*struct timeval last_accel_use_update_time;
    uint64_t in_use_accel_times_array[NUM_CPU_ACCEL+1][NUM_FFT_ACCEL+1][NUM_VIT_ACCEL+1][NUM_CV_ACCEL+1];*/

  // Scheduler Library statistics
  uint64_t scheduler_decision_time_usec;
  uint64_t scheduler_decisions;
  uint64_t scheduler_decision_checks;

} scheduler_datastate_block_t;

//extern scheduler_datastate_block_t sched_state;

scheduler_get_datastate_in_parms_t* get_scheduler_datastate_default_parms_pointer();
scheduler_datastate_block_t* get_new_scheduler_datastate_pointer(scheduler_get_datastate_in_parms_t* inp);

extern status_t initialize_scheduler(scheduler_datastate_block_t* sptr);

extern task_metadata_block_t* get_task_metadata_block(scheduler_datastate_block_t* sptr, task_id_t of_task_type, task_criticality_t crit_level, uint64_t * task_profile);
extern void free_task_metadata_block(task_metadata_block_t* mb);

extern void request_execution(task_metadata_block_t* task_metadata_block);
extern int get_task_status(scheduler_datastate_block_t* sptr, int task_id);
extern void wait_all_critical(scheduler_datastate_block_t* sptr);
extern void wait_all_tasks_finish(scheduler_datastate_block_t* sptr);
void mark_task_done(task_metadata_block_t* task_metadata_block);

extern void print_base_metadata_block_contents(task_metadata_block_t* mb);
extern void dump_all_metadata_blocks_states(scheduler_datastate_block_t* sptr);

extern void shutdown_scheduler(scheduler_datastate_block_t* sptr);

extern void init_accelerators_in_use_interval(scheduler_datastate_block_t* sptr, struct timeval start_prog);

extern void cleanup_and_exit(scheduler_datastate_block_t* sptr, int rval);

extern task_id_t register_task_type(scheduler_datastate_block_t* sptr, task_type_defn_info_t*);

extern accelerator_type_t register_accelerator_pool(scheduler_datastate_block_t* sptr, accelerator_pool_defn_info_t*);

extern void register_accel_can_exec_task(scheduler_datastate_block_t* sptr, accelerator_type_t acid, task_id_t tid, sched_execute_task_function_t fptr);

#endif
