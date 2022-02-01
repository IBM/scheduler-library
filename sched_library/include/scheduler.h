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

#include <functional>
#include <queue>
#include <list>
#include <vector>
#include <iostream>

#include "boost/graph/adjacency_list.hpp"
#include "boost/graph/topological_sort.hpp"
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/graphviz.hpp>

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

typedef enum {
  ACTIVE_DAG = 0,
  ACTIVE_QUEUED_DAG,
  ACTIVE_NOTREADY_DAG,
  COMPLETED_DAG,
  NUM_DAG_STATUS
} dag_status_t;

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

  //Meta policy
  char meta_policy[256];
  //Task policy
  char task_policy[256];

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
  uint64_t assign_sec, assign_usec;
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

class scheduler_datastate;

class task_metadata_entry {
 public:
  // This points to the scheduler datastate structure (defiuned below) to which
  // this metadata block belongs.
  scheduler_datastate *scheduler_datastate_pointer;

  // This portion is management, control, and scheduler stuff...
  int32_t block_id; // master-pool-index; a unique ID per metadata task
  task_status_t status; // -1 = free, 0 = allocated, 1 = queued, 2 = running, 3 = done ?
  pthread_t thread_id; // set when we invoke pthread_create (at least for CPU)
  pthread_mutex_t metadata_mutex; // Used to guard access to altering metadata
  // conditional variables
  pthread_cond_t metadata_condv;  // These phthreads conditional variables are
  // used to "signal" a thread to do work

  accelerator_type_t accelerator_type; // indicates which accelerator type is being used (id's
  // set at accelerator registration)
  int32_t accelerator_id; // indicates which (of the N of that type) accelerator
  // this task is executing on
  task_type_t task_type;  // An indication of the task type; defined when tasks
  // are registeres
  int32_t task_id; // A unique identifier for this task (across the full run)
  int32_t dag_id;  // Indicates which DAG spawns or owns this task
  task_criticality_t
  crit_level; // [0 .. 3] -- see above enumeration ("Base" to "Critical")
  float rank_hom;
  uint8_t rank_het;
  uint64_t deadline_time;


  uint64_t *task_on_accel_profile; //[MAX_ACCEL_TYPES];  //Timing profile for
  //task (in usec) -- maps task projected time
  //on accelerator...
  uint64_t task_max_time;
  uint64_t task_min_time;

  void (*atFinish)(task_metadata_entry *); // Call-back Finish-time function

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
};

struct dag_vertex_t {
  int32_t vertex_id;
  task_status_t vertex_status = TASK_FREE;
  task_type_t task_type;
  task_metadata_entry * task_mb_ptr;
  void * input_ptr;
  void * output_ptr;
};

struct dag_edge_t {
  uint64_t data_movement_time_usec[SCHED_MAX_ACCEL_TYPES][SCHED_MAX_ACCEL_TYPES];
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::bidirectionalS, dag_vertex_t, dag_edge_t >
Graph;
typedef Graph::vertex_descriptor vertex_t;
typedef Graph::edge_descriptor edge_t;

class dag_metadata_entry {
 public:
  // This points to the scheduler datastate structure (defiuned below) to which
  // this metadata block belongs.
  scheduler_datastate *scheduler_datastate_pointer;
  int32_t dag_id;  // Indicates unique DAG ID based on arrival of DAG
  Graph * graph_ptr;
  // boost::adjacency_list<> * graph_ptr;
  task_metadata_entry * task_mb_ptr;
  struct timeval dag_arrival_time; //Time at which the DAG arrived
  uint64_t dag_deadline_time_usec; //Time before which DAG should be completed
  uint64_t dag_slack_usec; //Time until deadline
  task_criticality_t crit_level; // [0 .. 3] -- see above enumeration ("Base" to "Critical")
  // dag_type_t dag_type; //Type of DAG
  dag_status_t dag_status; // =-1 active, 0 active and queued, 1 completed

  //Stats calculation
  uint64_t dag_response_time;

  dag_metadata_entry(scheduler_datastate *scheduler_datastate_pointer, int32_t dag_id,
                     Graph * graph_ptr, uint64_t dag_deadline_time,
                     task_criticality_t crit_level);

};

// This is the Ready Task Queue -- it holds Metadata Block IDs
class ready_mb_task_queue_entry_t {
 public:
  // short unique_id;
  short block_id;
  scheduler_datastate *sptr;
};

// This is a typedef for the call-back function, called by the scheduler at
// finish time for a task
typedef void (*task_finish_callback_t)(task_metadata_entry *);

// This is a typedef for an execution function called by the scheduler (e.g. to
// execute a task)
typedef void (*sched_execute_task_function_t)(task_metadata_entry *);

// These are function pointer prototype declaration types, used for the
// regsiter_task_type routine.
typedef void (*print_metadata_block_contents_t)(/*task_metadata_entry*/ void *);
typedef void (*output_task_type_run_stats_t)(/*scheduler_datastate*/ void *sptr,
    unsigned my_task_type, unsigned total_accel_types);

typedef /*task_metadata_entry*/ void *(*set_up_task_function_t)(/*scheduler_datastate*/ void *sptr,
    task_type_t the_task_type, task_criticality_t crit_level, bool auto_finish,
    int32_t dag_id, int32_t task_id, void* args);
typedef void (*finish_task_execution_function_t)(/*task_metadata_entry*/ void *the_metadata_block,
    void* args);
typedef void (*auto_finish_task_function_t)(/*task_metadata_entry*/ void *mb);

// These are function pointer prototype declaration types, used for the
// regsiter_accelerator_type routine.
typedef void (*do_accel_initialization_t)(scheduler_datastate *sptr);
typedef void (*do_accel_closeout_t)(scheduler_datastate *sptr);
typedef void (*output_accel_run_stats_t)(scheduler_datastate *sptr, unsigned my_accel_id,
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

class  scheduler_datastate {
 public:
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

  //Handle for dynamically loaded META policy
  void *meta_policy_handle;
  // Function pointer to the meta policy static rank assignment
  void (*assign_static_rank)(scheduler_datastate *sptr, dag_metadata_entry *);
  // Function pointer to the meta policy static rank assignment
  void (*assign_dynamic_rank)(scheduler_datastate *sptr, task_metadata_entry *);


  // Handle for the dynamically loaded TASK policy
  void *task_policy_handle;
  // Function pointer to the policy initialization routine
  void (*initialize_assign_task_to_pe)(void *in_parm_ptr);
  // Function pointer for the policy's assign_task_to_pe() function
  ready_mb_task_queue_entry_t *(*assign_task_to_pe)(scheduler_datastate *sptr);
  // inparm: char policy[256];

  // The pool of metadata blocks for use by the tasks, etc.
  unsigned total_metadata_pool_blocks;
  task_metadata_entry *master_metadata_pool;

  pthread_mutex_t free_metadata_mutex; // Used to guard access to altering the
  // free-list metadata information, etc.
  int free_metadata_blocks;
  int *free_metadata_pool;
  uint32_t *allocated_metadata_blocks; // array over TASK_TYPES
  uint32_t *freed_metadata_blocks;     // array over TASK_TYPES

  pthread_mutex_t task_queue_mutex; // Used to guard access to altering the
  // ready-task-queue contents
  unsigned num_free_task_queue_entries;
  unsigned num_tasks_in_ready_queue;

  pthread_mutex_t dag_queue_mutex; // Used to guard access to altering the
  // ative-dag-queue contents

  //DAG based queues
  std::list<dag_metadata_entry *> active_dags;
  std::list<dag_metadata_entry *> completed_dags;

  //Task based queues
  std::list<ready_mb_task_queue_entry_t*> ready_mb_task_queue_pool;
  std::list<ready_mb_task_queue_entry_t*> free_ready_mb_task_queue_entries;


  pthread_mutex_t accel_alloc_mutex; // Used to guard access to altering the
  // accelerator allocations

  // ASCII trace for Scheduler-Visualization Trace Output
  FILE *sl_viz_fp;
  pthread_mutex_t sl_viz_out_mutex; // Used to guard access to writing the
  // sl_viz output entries.

  pthread_t *metadata_threads;

  // pthread_mutex_t schedule_from_queue_mutex;   // Used to guard access to
  // scheduling functionality
  pthread_t metasched_thread;
  pthread_t tasksched_thread;


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

  print_metadata_block_contents_t *print_metablock_contents_function; // array over TASK_TYPES
  output_task_type_run_stats_t *output_task_run_stats_function;          // array over TASK_TYPES
  set_up_task_function_t *set_up_task_function; // array over TASK_TYPES
  finish_task_execution_function_t *finish_task_execution_function; // array over TASK_TYPES
  auto_finish_task_function_t *auto_finish_task_function; // array over TASK_TYPES

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
};

//Custom function to sort entries of ready queue
struct rank_ordering {
  bool operator() (ready_mb_task_queue_entry_t const * lhs,
                   ready_mb_task_queue_entry_t const * rhs) {
    //First sort by rank_het
    if (((lhs->sptr)->master_metadata_pool[lhs->block_id]).rank_het >
        ((rhs->sptr)->master_metadata_pool[rhs->block_id]).rank_het) {
      return true;
    } else if (((lhs->sptr)->master_metadata_pool[lhs->block_id]).rank_het ==
               ((rhs->sptr)->master_metadata_pool[rhs->block_id]).rank_het) {
      //And then sort by rank_hom
      return ((lhs->sptr)->master_metadata_pool[lhs->block_id]).rank_hom >=
             ((rhs->sptr)->master_metadata_pool[rhs->block_id]).rank_hom;
    } else return false;
  }
};


extern scheduler_get_datastate_in_parms_t *
get_scheduler_datastate_input_parms();
extern scheduler_datastate *
initialize_scheduler_and_return_datastate_pointer(
  scheduler_get_datastate_in_parms_t *inp);
extern scheduler_datastate *
initialize_scheduler_from_config_file(char *config_file_name);

extern void set_up_scheduler();

extern task_metadata_entry *
get_task_metadata_block(scheduler_datastate *sptr, int32_t dag_id, int32_t task_id,
                        task_type_t of_task_type, task_criticality_t crit_level,
                        uint64_t *task_profile);
extern void free_task_metadata_block(task_metadata_entry *mb);

extern auto_finish_task_function_t
get_auto_finish_routine(scheduler_datastate *sptr,
                        task_type_t the_task_type);

extern void request_execution(void *dag_ptr);
// extern int get_task_status(scheduler_datastate* sptr, int task_id);

extern void mark_task_done(task_metadata_entry *task_metadata_block);
extern void update_dag(task_metadata_entry *task_metadata_block);

extern void print_base_metadata_block_contents(task_metadata_entry *mb);
extern void dump_all_metadata_blocks_states(scheduler_datastate *sptr);

extern void shutdown_scheduler(scheduler_datastate *sptr);

extern void init_accelerators_in_use_interval(scheduler_datastate *sptr,
    struct timeval start_prog);

extern void cleanup_and_exit(int rval, void *sptr);

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
                         int32_t dag_id, int32_t task_id, void * args);

extern void finish_task_execution(void *the_metadata_block, void * args);

extern void print_ready_tasks_queue(scheduler_datastate *sptr);
#endif
