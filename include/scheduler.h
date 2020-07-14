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

//#include "verbose.h"

#include "base_types.h"

#define MAX_LIVE_METADATA_BLOCKS  32  // Must be <= total_metadata_pool_blocks 
#define MAX_ACCEL_OF_EACH_TYPE    10

#define MAX_RADAR_LOGN            14        // Max we allow is 16k samples
#define MAX_RADAR_N     (1<<MAX_RADAR_LOGN) // Max we allow is 16k samples


typedef enum { NO_TASK_JOB = 0,
	       FFT_TASK,
	       VITERBI_TASK,
	       CV_TASK,
	       NUM_JOB_TYPES } scheduler_jobs_t;

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

typedef enum { cpu_accel_t = 0,
	       fft_hwr_accel_t,
	       vit_hwr_accel_t,
	       cv_hwr_accel_t,
	       no_accelerator_t,
	       NUM_ACCEL_TYPES} accelerator_type_t;

typedef enum { SELECT_ACCEL_AND_WAIT_POLICY = 0,
	       FAST_TO_SLOW_FIRST_AVAIL_POLICY,
	       FASTEST_FINISH_TIME_FIRST_POLICY,
               NUM_SELECTION_POLICIES } accel_selct_policy_t;


extern const char* task_job_str[NUM_JOB_TYPES];
extern const char* task_criticality_str[NUM_TASK_CRIT_LEVELS];
extern const char* task_status_str[NUM_TASK_STATUS];
extern const char* accel_type_str[NUM_ACCEL_TYPES];
extern const char* scheduler_selection_policy_str[NUM_SELECTION_POLICIES];

// This is a structure that defines the "FFT" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "FFT" Task view of "data"
  label_t object_label; // The deteremined label of the object in the image
}  cv_data_struct_t;


// This is a structure that defines the "FFT" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "FFT" Task view of "data"
  int32_t log_nsamples;       // The Log2 of the number of samples in this FFT
  float   theData[2* (1<<14)]; // MAx supported samples (2^14) * 2 float per complex input/output
}  fft_data_struct_t;

// This is a struutre that defines the "Viterbi" job's "view" of the data (in the metadata structure)
//  Each job can define a specific "view" of data, and use that in interpreting the data space.
typedef struct { // The "Viterbi" view of "data"
  int32_t n_data_bits;
  int32_t n_cbps;
  int32_t n_traceback;
  int32_t psdu_size;
  int32_t inMem_size;    // The first inMem_size bytes of theData are the inMem (input memories)
  int32_t inData_size;   // The next inData_size bytes of theData are the inData (input data)
  int32_t outData_size;  // The next outData_size bytes of theData are the outData (output data)
  uint8_t theData[64*1024]; // This is larger than needed (~24780 + 18585) but less than FFT requires (so okay)
}  viterbi_data_struct_t;

// This is a timing analysis structure for the scheduler functions, etc.
typedef struct {
  struct timeval idle_start;
  uint64_t idle_sec, idle_usec;
  struct timeval get_start;
  uint64_t get_sec, get_usec;
  struct timeval queued_start;
  uint64_t queued_sec, queued_usec;
  struct timeval running_start;
  uint64_t running_sec[NUM_ACCEL_TYPES], running_usec[NUM_ACCEL_TYPES];
  struct timeval done_start;
  uint64_t done_sec, done_usec;
} sched_timing_data_t;

// The following structures are for timing analysis (per job type)
typedef struct {
  struct timeval calc_start;
  struct timeval fft_start;
  struct timeval fft_br_start;
  struct timeval bitrev_start;
  struct timeval fft_cvtin_start;
  struct timeval fft_cvtout_start;
  struct timeval cdfmcw_start;
  // 0 = timings for cpu_accel_T and 1 = fft_hwr_accel_t
  unsigned comp_by[2];
  uint64_t calc_sec[2], calc_usec[2];
  uint64_t fft_sec[2], fft_usec[2];
  uint64_t fft_br_sec[2], fft_br_usec[2];
  uint64_t bitrev_sec[2], bitrev_usec[2];
  uint64_t fft_cvtin_sec[2], fft_cvtin_usec[2];
  uint64_t fft_cvtout_sec[2], fft_cvtout_usec[2];
  uint64_t cdfmcw_sec[2], cdfmcw_usec[2];
} fft_timing_data_t;

typedef struct {
  struct timeval dodec_start;
  struct timeval depunc_start;
  // 0 = timings for cpu_accel_T and 1 = vit_hwr_accel_t
  unsigned comp_by[2];
  uint64_t dodec_sec[2],  dodec_usec[2];
  uint64_t depunc_sec[2], depunc_usec[2];
} vit_timing_data_t;

typedef struct {
  struct timeval call_start;
  struct timeval parse_start;
  // 0 = timings for cpu_accel_T and 1 = cv_hwr_accel_t
  unsigned comp_by[2];
  uint64_t call_sec[2],  call_usec[2];
  uint64_t parse_sec[2], parse_usec[2];
} cv_timing_data_t;

// This is a metatdata structure; it is used to hold all information for any job
//  to be invoked through the scheduler.  This includes a description of the
//  job type, and all input/output data space for the task
// The job types are defined above in the scheduler_jobs_t enumeration
// The data (i.e. inputs, outputs, etc. ) are transferred here as a "bulk data"
//  memory (of abstract uint8_t or bytes) and a size.  The interpretation of this
//  block of data is task-dependent, and can have an over-laid structure, etc.

typedef struct task_metadata_entry_struct {
  // This portion is management, control, and scheduler stuff...
  int32_t  block_id;              // master-pool-index; a unique ID per metadata task
  task_status_t  status;          // -1 = free, 0 = allocated, 1 = queued, 2 = running, 3 = done ?
  pthread_t       thread_id;      // set when we invoke pthread_create (at least for CPU)
  pthread_mutex_t metadata_mutex; // Used to guard access to altering metadata conditional variables
  pthread_cond_t  metadata_condv; // These phthreads conditional variables are used to "signal" a thread to do work

  accelerator_type_t  accelerator_type; // indicates which accelerator this task is executing on
  int32_t  accelerator_id;        // indicates which accelerator this task is executing on
  scheduler_jobs_t job_type;      // see above enumeration
  task_criticality_t crit_level;  // [0 .. ?] ?

  uint64_t task_profile[NUM_ACCEL_TYPES];  //Timing profile for task (in usec) -- maps job to accelerator projected time on accelerator...
  
  void (*atFinish)(struct task_metadata_entry_struct *); // Call-back Finish-time function

  unsigned gets_by_type[NUM_JOB_TYPES]; // Count of times this metadata block allocated per job type.
  unsigned frees_by_type[NUM_JOB_TYPES]; // Count of times this metadata block allocated per job type.
  
  // These are timing-related storage; currently we keep per-job-type in each metadata to aggregate (per block) over the run
  sched_timing_data_t sched_timings;
  fft_timing_data_t   fft_timings;  
  vit_timing_data_t   vit_timings; 
  cv_timing_data_t    cv_timings;  

  // This is the segment for data for the jobs
  int32_t  data_size;                // Number of bytes occupied in data (NOT USED/NOT NEEDED?)
  union { // This union holds job-specific "views" of the data (input/ouput memory for job accelerators)
    uint8_t  raw_data[128*1024];     // 128 KB is the current MAX data size for all jobs
    cv_data_struct_t      cv_data;   // CV/CNN view of data -- see strucutre typedef above
    fft_data_struct_t     fft_data;  // FFT view of data -- see strucutre typedef above
    viterbi_data_struct_t vit_data;  // Viterbi view of data -- see strucutre typedef above
  } data_view;
} task_metadata_block_t;

// This is a typedef for the call-back function, called by the scheduler at finish time for a task
typedef void (*task_finish_callback_t)(task_metadata_block_t*);

// This is the accelerator selection policy used by the scheduler
extern accel_selct_policy_t global_scheduler_selection_policy;

// These are some "fake" times (models the execution of CV timing)
extern unsigned cv_cpu_run_time_in_usec;
extern unsigned cv_fake_hwr_run_time_in_usec;

// This is the number of fft samples (the log of the samples, e.g. 10 = 1024 samples, 14 = 16k-samples)
extern unsigned crit_fft_samples_set;

extern status_t initialize_scheduler();

extern task_metadata_block_t* get_task_metadata_block(scheduler_jobs_t task_type, task_criticality_t crit_level, uint64_t * task_profile);
extern void free_task_metadata_block(task_metadata_block_t* mb);

extern void request_execution(task_metadata_block_t* task_metadata_block);
extern int get_task_status(int task_id);
extern void wait_all_critical();
extern void wait_all_tasks_finish();
void mark_task_done(task_metadata_block_t* task_metadata_block);

extern void print_base_metadata_block_contents(task_metadata_block_t* mb);
extern void print_fft_metadata_block_contents(task_metadata_block_t* mb);
extern void print_viterbi_metadata_block_contents(task_metadata_block_t* mb);

extern void shutdown_scheduler();

#endif
