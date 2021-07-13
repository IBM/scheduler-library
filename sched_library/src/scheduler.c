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

#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

//#include "utils.h"
//#define VERBOSE
#include "verbose.h"

/*#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif
*/
#include "scheduler.h"

// These are global "invariants" (determined by the physical hardware)
//  Mostly this is the pool of available hardware accelerator types...
struct global_hardware_state_block_struct {
  char accel_name_str[SCHED_MAX_ACCEL_TYPES][MAX_ACCEL_NAME_LEN];
  char accel_desc_str[SCHED_MAX_ACCEL_TYPES][MAX_ACCEL_DESC_LEN];

  // This is a table of the execution functions for the various Task Types in
  // the scheduler
  //  We set this up with one "set" of entries per JOB_TYPE
  //   where each set has one execute function per possible TASK TARGET (on
  //   which it can execute) Currently the targets are "CPU" and "HWR" -- this
  //   probably has to change (though this interpretation is only convention).
  sched_execute_task_function_t
      *scheduler_execute_task_function[SCHED_MAX_ACCEL_TYPES]; // array over
                                                               // TASK_TYPES

  do_accel_initialization_t do_accel_init_function[SCHED_MAX_ACCEL_TYPES];
  do_accel_closeout_t do_accel_closeout_function[SCHED_MAX_ACCEL_TYPES];
  output_accel_run_stats_t
      output_accel_run_stats_function[SCHED_MAX_ACCEL_TYPES];

  int num_accelerators_of_type[SCHED_MAX_ACCEL_TYPES];
} global_hardware_state_block;

int32_t global_task_id_counter = 0;
int32_t global_finished_task_id_counter = 0;

scheduler_get_datastate_in_parms_t sched_state_default_parms = {
    .max_task_types = 8, // MAX_TASK_TYPES,
    .max_accel_types = SCHED_MAX_ACCEL_TYPES,

    //.max_accel_of_any_type = 4, // Some random initial value - was
    // MAX_ACCEL_OF_ANY_TYPE,

    .max_metadata_pool_blocks =
        32, // Some default value - was GLOBAL_METADATA_POOL_BLOCKS,
    .max_data_space_bytes =
        (256 * 1024), // Some default value - was MAX_DATA_SPACE_BYTES,

    .max_task_timing_sets =
        16, // Some default value - was MAX_TASK_TIMING_SETS,

    .scheduler_holdoff_usec = 1,

    .visualizer_output_enabled = 0,
    .visualizer_task_start_count = -1,
    .visualizer_task_stop_count = 0,
    .visualizer_task_enable_type = -1,
};

// This now will hold all the state for an instance of the scheduler.
//  This can be shared with the policies, etc. using a single pointer parameter
//  This also could allow multiple scheduler states to be in effect
//  simultaneously...?
// scheduler_datastate_block_t sched_state;

// Forward declarations
void release_accelerator_for_task(task_metadata_block_t *task_metadata_block);

void *schedule_executions_from_queue(void *void_parm_ptr);
status_t initialize_scheduler(scheduler_datastate_block_t *sptr);
void register_accel_can_exec_task(scheduler_datastate_block_t *sptr,
                                  scheduler_accelerator_type sl_acid,
                                  task_type_t tid,
                                  sched_execute_task_function_t fptr);

void print_ready_tasks_queue(scheduler_datastate_block_t *sptr) {
  int ti = 0;
  ready_mb_task_queue_entry_t *t_te = sptr->ready_mb_task_queue_head;
  while (t_te != NULL) {
    printf(
        " RTQ: Ready_Task_Entry %2u of %2u : ID %2u MB%d P: %p T: %p N: %p\n",
        ti, sptr->num_tasks_in_ready_queue, t_te->unique_id, t_te->block_id,
        t_te->prev, t_te, t_te->next);
    ti++;
    t_te = t_te->next;
  }
}

void print_free_ready_tasks_list(scheduler_datastate_block_t *sptr) {
  int i = 0;
  ready_mb_task_queue_entry_t *t_te = sptr->free_ready_mb_task_queue_entries;
  while (t_te != NULL) {
    printf(" FRTL: Entry %2u of %2u : ID %2u MB%d P: %p T: %p N: %p\n", i,
           sptr->num_free_task_queue_entries, t_te->unique_id, t_te->block_id,
           t_te->prev, t_te, t_te->next);
    i++;
    t_te = t_te->next;
  }
}

/*void init_accelerators_in_use_interval(scheduler_datastate_block_t* sptr,
  struct timeval start_time) { sptr->last_accel_use_update_time = start_time;
  }*/

void account_accelerators_in_use_interval(scheduler_datastate_block_t *sptr) {
  // Count which accelerators are in use
  int acc_in_use[sptr->inparms->max_accel_types - 1];
  for (int i = 0; i < sptr->inparms->max_accel_types - 1; i++) {
    acc_in_use[i] = 0;
    for (int j = 0; j < sptr->num_accelerators_of_type[i]; j++) {
      if (sptr->accelerator_in_use_by[i][j] != -1) {
        acc_in_use[i]++;
      }
    }
  }
  // Now we have the array of curr-in-use
  struct timeval now;
  gettimeofday(&now, NULL);
  /* // This is a 4-Dim by ACCEL type : [CPU][FFT][VIT][CV] */
  /* sptr->in_use_accel_times_array[acc_in_use[0]][acc_in_use[1]][acc_in_use[2]][acc_in_use[3]]
  += 1000000*(now.tv_sec - sptr->last_accel_use_update_time.tv_sec) +
  (now.tv_usec - sptr->last_accel_use_update_time.tv_usec);
  sptr->last_accel_use_update_time = now;*/
}

void print_base_metadata_block_contents(task_metadata_block_t *mb) {
  printf("MB%u: block_id = %d @ %p\n", mb->block_id, mb->block_id, mb);
  scheduler_datastate_block_t *sptr = mb->scheduler_datastate_pointer;
  unsigned status = mb->status;
  if (status < NUM_TASK_STATUS) {
    printf("MB%u ** status = %s\n", mb->block_id,
           sptr->task_status_str[status]);
  } else {
    printf("MB%u ** status = %d <= NOT a legal value!\n", mb->block_id,
           mb->status);
  }
  unsigned task_type = mb->task_type;
  if ((task_type >= 0) && (task_type < sptr->inparms->max_task_types)) {
    printf("MB%u    task_type = %s\n", mb->block_id,
           sptr->task_name_str[task_type]);
  } else {
    printf("MB%u ** task_type = %d <= NOT a legal value!\n", mb->block_id,
           mb->task_type);
  }
  printf("MB%u    task_id = %d\n", mb->block_id, mb->task_id);
  unsigned crit_level = mb->crit_level;
  if (crit_level < NUM_TASK_CRIT_LEVELS) {
    printf("MB%u    crit_level = %s\n", mb->block_id,
           sptr->task_criticality_str[crit_level]);
  } else {
    printf("MB%u ** crit_level = %d <= NOT a legal value!\n", mb->block_id,
           mb->crit_level);
  }
  unsigned accelerator_type = mb->accelerator_type;
  if (accelerator_type < sptr->inparms->max_accel_types) {
    printf("MB%u    accelerator_type = %s\n", mb->block_id,
           sptr->accel_name_str[accelerator_type]);
  } else {
    printf("MB%u ** accelerator_type = %d <= NOT a legal value!\n",
           mb->block_id, mb->accelerator_type);
  }
  printf("MB%u    accelerator_id = %d\n", mb->block_id, mb->accelerator_id);
  printf("MB%u    data_size  = %d\n", mb->block_id, mb->data_size);
  printf("MB%u    data_space @ %p\n", mb->block_id, mb->data_space);
}

void print_critical_task_list_ids(scheduler_datastate_block_t *sptr) {
  blockid_linked_list_t *cli = sptr->critical_live_task_head;
  if (cli == NULL) {
    printf("Critical task list is EMPTY\n");
  } else {
    printf("Critical task list : ");
    while (cli != NULL) {
      printf(" %u",
             cli->clt_block_id); //, free_critlist_pool[cli->clt_block_id]);
      cli = cli->next;
    }
    printf("\n");
  }
}

void do_accelerator_type_closeout(scheduler_datastate_block_t *sptr) {
  // Clean up any hardware accelerator stuff
  DEBUG(printf("Doing accelerator type closeout for %u accelerators\n",
               sptr->next_avail_accel_id));
  for (int ai = 0; ai < sptr->next_avail_accel_id; ai++) {
    if (sptr->do_accel_closeout_function[ai] != NULL) {
      sptr->do_accel_closeout_function[ai](NULL);
    } else {
      printf("Note: do_accel_closeout_function for accel %u = %s is NULL\n", ai,
             sptr->accel_name_str[ai]);
    }
  }
}

void output_task_and_accel_run_stats(scheduler_datastate_block_t *sptr) {
  printf("\nPer-MetaData-Block Job Timing Data:\n");
  for (int ti = 0; ti < sptr->next_avail_task_type; ti++) {
    if (sptr->output_task_run_stats_function[ti] != NULL) {
      sptr->output_task_run_stats_function[ti](sptr, ti,
                                               sptr->next_avail_accel_id);
    }
  }

  for (int ai = 0; ai < sptr->next_avail_accel_id; ai++) {
    if (sptr->output_accel_run_stats_function[ai] != NULL) {
      sptr->output_accel_run_stats_function[ai](sptr, ai,
                                                sptr->next_avail_task_type);
    }
  }
}

// There is an open question as to whether we should "Wait" for an available
// Metadata Block
//  or return if there are none available to the program (and let IT decide what
//  to do next) Currently, we have to return, or else the scheduler task cannot
//  make progress and free additional metablocks.... so we require the caller to
//  do the "while" loop...

task_metadata_block_t *
get_task_metadata_block(scheduler_datastate_block_t *sptr, int32_t in_dag_id,
                        task_type_t in_task_type, task_criticality_t crit_level,
                        uint64_t *task_profile) {
  pthread_mutex_lock(&(sptr->free_metadata_mutex));
  TDEBUG(printf("in get_task_metadata_block with %u free_metadata_blocks\n",
                sptr->free_metadata_blocks));
  if (sptr->free_metadata_blocks < 1) {
    // Out of metadata blocks -- all in use, cannot enqueue new tasks!
    printf("No free metadata blocks: %d\n", sptr->free_metadata_blocks);
    return NULL;
  }
  int bi = sptr->free_metadata_pool[sptr->free_metadata_blocks - 1];
  TDEBUG(printf(" BEFORE_GET : MB%d : free_metadata_pool : ", bi);
         for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
           printf("%d ", sptr->free_metadata_pool[i]);
         } printf("\n"));
  if ((bi < 0) || (bi > sptr->total_metadata_pool_blocks)) {
    printf("ERROR : free_metadata_pool[%u -1] = %d   with %d "
           "free_metadata_blocks\n",
           sptr->free_metadata_blocks, bi, sptr->free_metadata_blocks);
    for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
      printf("  free_metadata_pool[%2u] = %d\n", i,
             sptr->free_metadata_pool[i]);
    }
    exit( -16);
  }
  sptr->free_metadata_pool[sptr->free_metadata_blocks - 1] = -1;
  sptr->free_metadata_blocks -= 1;
  // For neatness (not "security") we'll clear the meta-data in the block (not
  // the data data,though)
  sptr->master_metadata_pool[bi].task_type = in_task_type;
  sptr->master_metadata_pool[bi].dag_id = in_dag_id;
  // TASKID: sptr->master_metadata_pool[bi].task_id   = -1; // Unset as yet ---
  // but we could track get_task_metadata_block order rather than
  // request_execution order..
  sptr->master_metadata_pool[bi].task_id =
      global_task_id_counter++; // Set a task id for this task (which is the
                                // global request_execution order, for now)
  sptr->master_metadata_pool[bi].gets_by_task_type[in_task_type]++;
  sptr->master_metadata_pool[bi].status = TASK_ALLOCATED;
  sptr->master_metadata_pool[bi].crit_level = crit_level;
  for (int i = 0; i < sptr->inparms->max_accel_types; ++i) {
    sptr->master_metadata_pool[bi].task_on_accel_profile[i] = task_profile[i];
  }
  sptr->master_metadata_pool[bi].data_size = 0;
  sptr->master_metadata_pool[bi].accelerator_type = NO_Accelerator;
  sptr->master_metadata_pool[bi].accelerator_id = NO_Task;
  sptr->master_metadata_pool[bi].atFinish =
      NULL; // NO finish call-back function

  gettimeofday(&sptr->master_metadata_pool[bi].sched_timings.get_start, NULL);
  sptr->master_metadata_pool[bi].sched_timings.idle_sec +=
      sptr->master_metadata_pool[bi].sched_timings.get_start.tv_sec -
      sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_sec;
  sptr->master_metadata_pool[bi].sched_timings.idle_usec +=
      sptr->master_metadata_pool[bi].sched_timings.get_start.tv_usec -
      sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_usec;

  if (crit_level > 1) { // is this a "critical task"
    // Select the next available critical-live-task-list entry ID
    int li = sptr->free_critlist_pool[sptr->free_critlist_entries - 1];
    sptr->free_critlist_pool[sptr->free_critlist_entries - 1] =
        -1; // clear it a(as it is no longer free)
    sptr->free_critlist_entries -= 1;
    // Now li indicates the critical_live_tasks_list[] index to use
    // Now set up the revisions to the critical live tasks list
    sptr->critical_live_tasks_list[li].clt_block_id =
        bi; // point this entry to the sptr->master_metadata_pool block id
    sptr->critical_live_tasks_list[li].next =
        sptr->critical_live_task_head; // Insert as head of critical tasks list
    sptr->critical_live_task_head = &(sptr->critical_live_tasks_list[li]);
    sptr->total_critical_tasks += 1;
  }
  DEBUG(printf("  returning block %u\n", bi);
        print_critical_task_list_ids(sptr));
  TDEBUG(printf(" AFTER_GET : MB%u : free_metadata_pool : ", bi);
         for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
           printf("%d ", sptr->free_metadata_pool[i]);
         } printf("\n"));
  sptr->allocated_metadata_blocks[in_task_type]++;
  // printf("MB%u got allocated : %u %u\n", bi, task_type, crit_level);
  pthread_mutex_unlock(&(sptr->free_metadata_mutex));

  return &(sptr->master_metadata_pool[bi]);
}

void free_task_metadata_block(task_metadata_block_t *mb) {
  scheduler_datastate_block_t *sptr = mb->scheduler_datastate_pointer;
  pthread_mutex_lock(&(sptr->free_metadata_mutex));

  int bi = mb->block_id;
  // printf("MB%u getting freed : %u %u\n", bi, mb->task_type, mb->crit_level);
  TDEBUG(printf("in free_task_metadata_block for block %u with %u "
                "free_metadata_blocks\n",
                bi, sptr->free_metadata_blocks); //);
         printf(" BEFORE_FREE : MB%u : free_metadata_pool : ", bi);
         for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
           printf("%d ", sptr->free_metadata_pool[i]);
         } printf("\n"));

  if (sptr->free_metadata_blocks < sptr->total_metadata_pool_blocks) {
    sptr->master_metadata_pool[bi].frees_by_task_type[mb->task_type]++;
    sptr->free_metadata_pool[sptr->free_metadata_blocks] = bi;
    sptr->free_metadata_blocks += 1;
    if (sptr->master_metadata_pool[bi].crit_level >
        1) { // is this a critical tasks?
      // Remove task form critical list, free critlist entry, etc.
      blockid_linked_list_t *lcli = NULL;
      blockid_linked_list_t *cli = sptr->critical_live_task_head;
      // while ((cli != NULL) &&
      // (critical_live_tasks_list[cli->clt_block_id].clt_block_id != bi)) {
      while ((cli != NULL) && (cli->clt_block_id != bi)) {
        lcli = cli; // The "previous" block; NULL == "head"
        cli = cli->next;
      }
      if (cli == NULL) {
        printf("ERROR: Critical task NOT on the critical_live_task_list :\n");
        print_base_metadata_block_contents(mb);
        exit( -6);
      }
      // We've found the critical task in critical_live_tasks_list - cli points
      // to it
      int cti = cli->clt_block_id;
      // printf(" freeing critlist_pool %u to %u\n", free_critlist_entries - 1,
      // cti);
      sptr->free_critlist_pool[sptr->free_critlist_entries] =
          cti; // Enable this crit-list entry for new use
      sptr->free_critlist_entries +=
          1; // Update the count of available critlist entries in the pool
      cli->clt_block_id =
          -1; // clear the clt_block_id indicator (we're done with it)
      // And remove the cli entry from the critical_lvet_tasks linked list
      if (lcli == NULL) {
        sptr->critical_live_task_head = cli->next;
      } else {
        lcli->next = cli->next;
      }
      cli->next = NULL;
      sptr->total_critical_tasks -= 1;
    }
    sptr->master_metadata_pool[bi].atFinish =
        NULL; // Ensure this is now set to NULL (safety safety)
    // For neatness (not "security") we'll clear the meta-data in the block (not
    // the data data, though)
    sptr->freed_metadata_blocks[sptr->master_metadata_pool[bi].task_type]++;
    sptr->master_metadata_pool[bi].task_type = NO_Task; // unset
    sptr->master_metadata_pool[bi].status = TASK_FREE;  // free
    gettimeofday(&sptr->master_metadata_pool[bi].sched_timings.idle_start,
                 NULL);
    sptr->master_metadata_pool[bi].sched_timings.done_sec +=
        sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_sec -
        sptr->master_metadata_pool[bi].sched_timings.done_start.tv_sec;
    sptr->master_metadata_pool[bi].sched_timings.done_usec +=
        sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_usec -
        sptr->master_metadata_pool[bi].sched_timings.done_start.tv_usec;
    sptr->master_metadata_pool[bi].crit_level = BASE_TASK; // lowest/free?
    sptr->master_metadata_pool[bi].data_size = 0;
  } else {
    printf("ERROR : We are freeing a metadata block when we already have max "
           "metadata blocks free...\n");
    printf("   THE FREE Metadata Blocks list:\n");
    for (int ii = 0; ii < sptr->free_metadata_blocks; ii++) {
      printf("        free[%2u] = %u\n", ii, sptr->free_metadata_pool[ii]);
    }
    DEBUG(printf("    THE Being-Freed Meta-Data Block:\n");
          print_base_metadata_block_contents(mb));
    exit( -5);
  }
  TDEBUG(printf(" AFTER_FREE : MB%u : free_metadata_pool : ", bi);
         for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
           printf("%d ", sptr->free_metadata_pool[i]);
         } printf("\n"));
  pthread_mutex_unlock(&(sptr->free_metadata_mutex));
}

void mark_task_done(task_metadata_block_t *task_metadata_block) {
  scheduler_datastate_block_t *sptr =
      task_metadata_block->scheduler_datastate_pointer;
  DEBUG(printf("MB%u entered mark_task_done...\n",
               task_metadata_block->block_id));
  // printf("MB%u in mark_task_done\n", task_metadata_block->block_id);

  gettimeofday(&task_metadata_block->sched_timings.done_start, NULL);
  int idx = task_metadata_block->accelerator_type;
  task_metadata_block->sched_timings.running_sec[idx] +=
      task_metadata_block->sched_timings.done_start.tv_sec -
      task_metadata_block->sched_timings.running_start.tv_sec;
  task_metadata_block->sched_timings.running_usec[idx] +=
      task_metadata_block->sched_timings.done_start.tv_usec -
      task_metadata_block->sched_timings.running_start.tv_usec;
  DEBUG(printf("Set MB%u sched_timings.done_start to %lu %lu\n",
               task_metadata_block->block_id,
               task_metadata_block->sched_timings.done_start.tv_sec,
               task_metadata_block->sched_timings.done_start.tv_usec));

  // First release the accelerator
  release_accelerator_for_task(task_metadata_block);

  // IF we should output Visualizer info for this task, output it now...
  if (sptr->inparms->visualizer_output_enabled) {
    DEBUG(printf("  Glob_Fin_Tasks %d : START_CT %d  :  STOP_CT %d  :  en_task "
                 "%d vs curr_task %d\n",
                 global_finished_task_id_counter,
                 sptr->inparms->visualizer_task_start_count,
                 sptr->inparms->visualizer_task_stop_count,
                 sptr->inparms->visualizer_task_enable_type,
                 task_metadata_block->task_type));
    pthread_mutex_lock(&(sptr->sl_viz_out_mutex));
    if (!sptr->visualizer_output_started) {
      int start_logging = false;
      if (sptr->inparms->visualizer_task_start_count < 0) {
        DEBUG(printf(" start_logging : no_start_count : task_en_ty = %d vs "
                     "curr_type %d\n",
                     sptr->inparms->visualizer_task_enable_type,
                     task_metadata_block->task_type));
        if ((sptr->inparms->visualizer_task_enable_type == NO_Task) ||
            (task_metadata_block->task_type ==
             sptr->inparms->visualizer_task_enable_type)) {
          start_logging = true;
        }
      } else {
        DEBUG(printf(" start_logging : start_count %d vs curr_count %d : "
                     "task_en_ty = %d vs curr_type %d\n",
                     global_finished_task_id_counter,
                     sptr->inparms->visualizer_task_start_count,
                     sptr->inparms->visualizer_task_enable_type,
                     task_metadata_block->task_type));
        if ((global_finished_task_id_counter >=
             sptr->inparms->visualizer_task_start_count) ||
            (task_metadata_block->task_type ==
             sptr->inparms->visualizer_task_enable_type)) {
          start_logging = true;
        }
      }
      if (start_logging) {
        sptr->visualizer_output_started = true;
        sptr->visualizer_start_time_usec =
            1000000 * task_metadata_block->sched_timings.queued_start.tv_sec +
            task_metadata_block->sched_timings.queued_start.tv_usec;
        if (sptr->inparms->visualizer_task_stop_count >= 0) {
          sptr->inparms->visualizer_task_stop_count +=
              global_finished_task_id_counter; // This means we get stop_count
                                               // starting from here...
          DEBUG(printf("Starting SL_VIZ logging at %d .. stop at count %d\n",
                       global_finished_task_id_counter,
                       sptr->inparms->visualizer_task_stop_count));
        } else {
          DEBUG(printf(
              "Starting SL_VIZ logging at %d .. stop at count end of run\n",
              global_finished_task_id_counter));
        }
      }
    }

    if (sptr->visualizer_output_started &&
        ((sptr->inparms->visualizer_task_stop_count < 0) ||
         (global_finished_task_id_counter <
          sptr->inparms->visualizer_task_stop_count))) {
      int64_t curr_time =
          (1000000 * task_metadata_block->sched_timings.done_start.tv_sec +
           task_metadata_block->sched_timings.done_start.tv_usec) -
          sptr->visualizer_start_time_usec;
      int64_t arr_time =
          (1000000 * task_metadata_block->sched_timings.queued_start.tv_sec +
           task_metadata_block->sched_timings.queued_start.tv_usec) -
          sptr->visualizer_start_time_usec;
      int64_t start_time =
          (1000000 * task_metadata_block->sched_timings.running_start.tv_sec +
           task_metadata_block->sched_timings.running_start.tv_usec) -
          sptr->visualizer_start_time_usec;
      if (curr_time < 0) {
        curr_time = 0;
      }
      if (arr_time < 0) {
        arr_time = 0;
      }
      if (start_time < 0) {
        start_time = 0;
      }
      uint64_t end_time = curr_time;
      DEBUG(
          printf("   printing SL_VIZ line for MB%u Task %d %s on %d %d %s\n",
                 task_metadata_block->block_id, task_metadata_block->task_type,
                 sptr->task_name_str[task_metadata_block->task_type],
                 task_metadata_block->accelerator_type,
                 task_metadata_block->accelerator_id,
                 sptr->accel_name_str[task_metadata_block->accelerator_type]);
          printf(
              "     MB%u : arr_time = %lu start_time = %lu curr_time = %lu\n",
              task_metadata_block->block_id, arr_time, start_time, curr_time));
      // First a line for the "Queue" PE
      /*printf(" MB%u 4_SL_VIZ: full MB:\n", task_metadata_block->block_id);
        print_base_metadata_block_contents(task_metadata_block);
        printf(" MB%u 4_SL_VIZ: start_time : %lu\n",
        task_metadata_block->block_id, start_time);  //  sim_time,   ( pretend
        this was reported at start_time) printf(" MB%u 4_SL_VIZ: dag_id     :
        %d\n", task_metadata_block->block_id,  task_metadata_block->dag_id);  //
        task_dag_id printf(" MB%u 4_SL_VIZ: task_id    : %d\n",
        task_metadata_block->block_id,  task_metadata_block->task_id); //
        task_tid printf(" MB%u 4_SL_VIZ: task_type  : %d\n",
        task_metadata_block->block_id,  task_metadata_block->task_type);
        //task_type ?, printf(" MB%u 4_SL_VIZ: task_name  : %s\n",
        task_metadata_block->block_id,
        sptr->task_name_str[task_metadata_block->task_type]); //task_type ?,
        printf(" MB%u 4_SL_VIZ: crit_level : %u\n",
        task_metadata_block->block_id,  task_metadata_block->crit_level); //
        task_tid printf(" MB%u 4_SL_VIZ: dat_dtime  : %d\n",
        task_metadata_block->block_id,  0); // dag_dtime printf(" MB%u 4_SL_VIZ:
        accel_id   : %d\n", task_metadata_block->block_id,
        sptr->inparms->max_accel_types+1); // accelerator_id  - use a number
        that cannot be a legal accel_id, printf(" MB%u 4_SL_VIZ: accel_type :
        %s\n", task_metadata_block->block_id,  "Rdy_Que");  //accelerator_type
        ?, printf(" MB%u 4_SL_VIZ: t_prnt_ids : %s\n",
        task_metadata_block->block_id,  "nan"); //task_parent_ids printf(" MB%u
        4_SL_VIZ: acc_time   : %lu\n", task_metadata_block->block_id, arr_time);
        //task_arrival_time printf(" MB%u 4_SL_VIZ: start_time : %lu\n",
        task_metadata_block->block_id, start_time); //curr_job_start_time
        (Should chart only the "Arraival" period printf(" MB%u 4_SL_VIZ:
        end_time   : %lu\n", task_metadata_block->block_id, start_time);
        //curr_job_end_time    up to the start time, at which point it is moved
        into the actual PE) printf(" MB%u 4_SL_VIZ: sl_viz_fp  : %p\n",
        task_metadata_block->block_id, sptr->sl_viz_fp);*/
      fprintf(
          sptr->sl_viz_fp, "%lu,%d,%d,%s,%u,%d,%d,%s,%s,%lu,%lu,%lu\n",
          start_time, //  sim_time,   ( pretend this was reported at start_time)
          task_metadata_block->dag_id,                         // task_dag_id
          task_metadata_block->task_id,                        // task_tid
          sptr->task_name_str[task_metadata_block->task_type], // task_type ?,
          task_metadata_block->crit_level,                     // task_tid
          0,                                                   // dag_dtime
          sptr->inparms->max_accel_types +
              1,       // accelerator_id  - use a number that cannot be a legal
                       // accel_id,
          "Rdy_Que",   // accelerator_type ?,
          "nan",       // task_parent_ids
          arr_time,    // task_arrival_time
          start_time,  // curr_job_start_time  (Should chart only the "Arraival"
                       // period
          start_time); // curr_job_end_time    up to the start time, at which
                       // point it is moved into the actual PE)
      fprintf(
          sptr->sl_viz_fp, "%lu,%d,%d,%s,%u,%d,%d,%s,%s,%lu,%lu,%lu\n",
          curr_time,                                           //  sim_time,
          task_metadata_block->dag_id,                         // task_dag_id
          task_metadata_block->task_id,                        // task_tid
          sptr->task_name_str[task_metadata_block->task_type], // task_type ?,
          task_metadata_block->crit_level,                     // task_tid
          0,                                                   // dag_dtime
          task_metadata_block->accelerator_id, // accelerator_id ?,
          sptr->accel_name_str[task_metadata_block
                                   ->accelerator_type], // accelerator_type
                                                        // ?,
          "nan",                                        // task_parent_ids
          start_time, // task_arrival_time  (now this is in the Rdy_Que above)
          start_time, // curr_job_start_time
          end_time);  // curr_job_end_time
      pthread_mutex_unlock(&(sptr->sl_viz_out_mutex));
    } else {
      DEBUG(
          printf("          skip : NOT printing SL_VIZ line for MB%u Task %d "
                 "%s on %d %d %s\n",
                 task_metadata_block->block_id, task_metadata_block->task_type,
                 sptr->task_name_str[task_metadata_block->task_type],
                 task_metadata_block->accelerator_type,
                 task_metadata_block->accelerator_id,
                 sptr->accel_name_str[task_metadata_block->accelerator_type]));
    }
    pthread_mutex_unlock(&(sptr->sl_viz_out_mutex));
  }

  global_finished_task_id_counter++;

  // Then, mark the task as "DONE" with execution
  task_metadata_block->status = TASK_DONE;

  // And finally, call the call-back if there is one... (which might clear out
  // the metadata_block entirely)
  if (task_metadata_block->atFinish != NULL) {
    // And finally, call the atFinish call-back routine specified in the
    // MetaData Block
    task_metadata_block->atFinish(task_metadata_block);
  }
}

// NOTE: This is executed by a metadata_block pthread
void execute_task_on_accelerator(task_metadata_block_t *task_metadata_block) {
  scheduler_datastate_block_t *sptr =
      task_metadata_block->scheduler_datastate_pointer;
  DEBUG(printf("In execute_task_on_accelerator for MB%d with Accel Type %s and "
               "Number %u\n",
               task_metadata_block->block_id,
               sptr->accel_name_str[task_metadata_block->accelerator_type],
               task_metadata_block->accelerator_id));
  if (task_metadata_block->accelerator_type != NO_Accelerator) {
    if ((task_metadata_block->task_type != NO_Task) &&
        (task_metadata_block->task_type < sptr->inparms->max_task_types)) {
      DEBUG(
          printf("Executing Task for MB%d : Type %u %s on %u %s\n",
                 task_metadata_block->block_id, task_metadata_block->task_type,
                 sptr->task_name_str[task_metadata_block->task_type],
                 task_metadata_block->accelerator_type,
                 sptr->accel_name_str[task_metadata_block->accelerator_type]));
      sptr->scheduler_execute_task_function[task_metadata_block
                                                ->accelerator_type]
                                           [task_metadata_block->task_type](
                                               task_metadata_block);
    } else {
      printf("ERROR: execute_task_on_accelerator called for unknown task type: "
             "%u\n",
             task_metadata_block->task_type);
      print_base_metadata_block_contents(task_metadata_block);
      exit( -13);
    }
  } else {
    printf("ERROR -- called execute_task_on_accelerator for NO_ACCELERATOR "
           "with block:\n");
    print_base_metadata_block_contents(task_metadata_block);
    exit( -11);
  }
  TDEBUG(
      printf("DONE Executing Task for MB%d\n", task_metadata_block->block_id));
}

void *metadata_thread_wait_for_task(void *void_parm_ptr) {
  // Set up the pthread_cancel conditions for this thread
  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
  task_metadata_block_t *task_metadata_block = (task_metadata_block_t *)void_parm_ptr;

  int bi = task_metadata_block->block_id;
  DEBUG(printf( "In metadata_thread_wait_for_task for thread for metadata block %d\n", bi));
  // I think we do this once, then can wait_cond many times
  pthread_mutex_lock(&(task_metadata_block->metadata_mutex));
  do {
    TDEBUG(printf("MB%d calling pthread_cond_wait\n", bi));
    // This will cause the thread to wait for a triggering signal through
    // metadata_condv[bi]
    pthread_cond_wait(&(task_metadata_block->metadata_condv), &(task_metadata_block->metadata_mutex));

    TDEBUG(printf("MB%d calling execute_task_on_accelerator...\n", bi));
    // Now we have been "triggered" -- so we invoke the appropriate accelerator
    execute_task_on_accelerator(task_metadata_block);
    TDEBUG(printf("MB%d done with execution...\n", bi));
  } while (1); // We will loop here forever, until the main program exits....

  // We should never fall out, but in case we do, clean up
  pthread_mutex_unlock(&(task_metadata_block->metadata_mutex));
}

/* Input parameters: */
scheduler_get_datastate_in_parms_t *get_scheduler_datastate_input_parms() {
  scheduler_get_datastate_in_parms_t *pptr =
      malloc(sizeof(scheduler_get_datastate_in_parms_t));
  if (pptr == NULL) {
    printf("ERROR: Couldn't allocate the sched_inparms memory\n");
    exit(-99);
  }
  pptr->max_task_types = sched_state_default_parms.max_task_types;
  pptr->max_accel_types = sched_state_default_parms.max_accel_types;
  // pptr->max_accel_of_any_type     =
  // sched_state_default_parms.max_accel_of_any_type;
  pptr->max_metadata_pool_blocks =
      sched_state_default_parms.max_metadata_pool_blocks;
  pptr->max_task_timing_sets = sched_state_default_parms.max_task_timing_sets;
  pptr->max_data_space_bytes = sched_state_default_parms.max_data_space_bytes;

  pptr->scheduler_holdoff_usec =
      sched_state_default_parms.scheduler_holdoff_usec;
  pptr->policy[0] = '\0';

  pptr->visualizer_output_enabled =
      sched_state_default_parms.visualizer_output_enabled;
  pptr->visualizer_task_start_count =
      sched_state_default_parms.visualizer_task_start_count;
  pptr->visualizer_task_stop_count =
      sched_state_default_parms.visualizer_task_stop_count;
  pptr->visualizer_task_enable_type =
      sched_state_default_parms.visualizer_task_enable_type;
  pptr->sl_viz_fname[0] = '\0';

  // Note: the default here is '0' so one only needs to set those that are used.
  for (int i = 0; i < SCHED_MAX_ACCEL_TYPES; i++) {
    pptr->max_accel_to_use_from_pool[i] = 0;
  }
  return pptr;
}

scheduler_datastate_block_t *initialize_scheduler_and_return_datastate_pointer(scheduler_get_datastate_in_parms_t *inp)
{
  scheduler_datastate_block_t *sptr = malloc(sizeof(scheduler_datastate_block_t));
  on_exit(cleanup_and_exit, sptr);
  size_t sched_state_size = sizeof(scheduler_datastate_block_t);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for base scheduler_datastate_block_t sptr\n");
    exit(-99);
  }

  sptr->inparms = inp;
  sptr->visualizer_output_started = false;

  // Determine the max_acceleratores_of_any_type (used for some allocations
  // below, etc.)
  unsigned max_of_any_type = 0;
  for (int i = 0; i < SCHED_MAX_ACCEL_TYPES; i++) {
    int desired_num = inp->max_accel_to_use_from_pool[i];
    // printf("Requested to allocate %d Accel from pool %u\n", desired_num, i);
    if (desired_num < 0) {
      if (max_of_any_type <
          global_hardware_state_block.num_accelerators_of_type[i]) {
        max_of_any_type =
            global_hardware_state_block.num_accelerators_of_type[i];
      }
    } else {
      if (max_of_any_type < desired_num) {
        max_of_any_type = desired_num;
      }
    }
  }
  sptr->max_accel_of_any_type = max_of_any_type;
  DEBUG(printf("max_of_any_type = %u\n", sptr->max_accel_of_any_type));

  // Allocate the scheduler_datastate_block_t dynamic (per-task-type) elements
  sptr->allocated_metadata_blocks =
      malloc(inp->max_task_types * sizeof(uint32_t));
  sched_state_size += inp->max_task_types * sizeof(uint32_t);
  if (sptr->allocated_metadata_blocks == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->allocated_metadata_blocks\n");
    exit(-99);
  }
  sptr->freed_metadata_blocks = malloc(inp->max_task_types * sizeof(uint32_t));
  sched_state_size += inp->max_task_types * sizeof(uint32_t);
  if (sptr->freed_metadata_blocks == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->freed_metadata_blocks\n");
    exit(-99);
  }

  { // Set up the accelerator name and description strings
    sptr->accel_name_str =
        calloc(inp->max_accel_types,
               sizeof(char *)); //[MAX_ACCEL_TYPES][MAX_ACCEL_NAME_LEN];
    sched_state_size += inp->max_accel_types * sizeof(char *);
    if (sptr->accel_name_str == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->accel_name_str\n");
      exit(-99);
    }
    char *tptr = malloc(inp->max_accel_types * MAX_ACCEL_NAME_LEN);
    sched_state_size += inp->max_accel_types * MAX_ACCEL_NAME_LEN;
    if (tptr == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->accel_name_str char-pool\n");
      exit(-99);
    }
    for (int ti = 0; ti < inp->max_accel_types; ti++) {
      sptr->accel_name_str[ti] = tptr;
      tptr += MAX_ACCEL_NAME_LEN;
    }
    // Set up the accel_desc_str
    sptr->accel_desc_str =
        calloc(inp->max_accel_types,
               sizeof(char *)); //[MAX_ACCEL_TYPES][MAX_ACCEL_DESC_LEN];
    sched_state_size += inp->max_accel_types * sizeof(char *);
    if (sptr->accel_desc_str == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->accel_desc_str\n");
      exit(-99);
    }
    tptr = malloc(inp->max_accel_types * MAX_ACCEL_DESC_LEN);
    sched_state_size += inp->max_accel_types * MAX_ACCEL_DESC_LEN;
    if (tptr == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->accel_desc_str char-pool\n");
      exit(-99);
    }
    for (int ti = 0; ti < inp->max_accel_types; ti++) {
      sptr->accel_desc_str[ti] = tptr;
      tptr += MAX_ACCEL_NAME_LEN;
    }
  }

  sptr->scheduler_execute_task_function =
      malloc(inp->max_accel_types * sizeof(sched_execute_task_function_t));
  sched_state_size +=
      inp->max_accel_types * sizeof(sched_execute_task_function_t);
  if (sptr->scheduler_execute_task_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->scheduler_execute_task_function\n");
    exit(-99);
  }
  for (int ai = 0; ai < inp->max_accel_types; ai++) {
    sptr->scheduler_execute_task_function[ai] = calloc(
        inp->max_task_types,
        sizeof(
            sched_execute_task_function_t)); //[MAX_ACCEL_TYPES][MAX_TASK_TYPES];
    sched_state_size +=
        inp->max_task_types * sizeof(sched_execute_task_function_t);
    if (sptr->scheduler_execute_task_function[ai] == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->scheduler_execute_task_function[%u]\n",
             ai);
      exit(-99);
    }
  }

  sptr->do_accel_init_function =
      malloc(inp->max_accel_types * sizeof(do_accel_initialization_t));
  sched_state_size += inp->max_accel_types * sizeof(do_accel_initialization_t);
  if (sptr->do_accel_init_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->scheduler_execute_task_function\n");
    exit(-99);
  }

  sptr->do_accel_closeout_function =
      malloc(inp->max_accel_types * sizeof(do_accel_closeout_t));
  sched_state_size += inp->max_accel_types * sizeof(do_accel_closeout_t);
  if (sptr->do_accel_closeout_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->scheduler_execute_task_function\n");
    exit(-99);
  }

  sptr->output_accel_run_stats_function =
      malloc(inp->max_accel_types * sizeof(output_accel_run_stats_t));
  sched_state_size += inp->max_accel_types * sizeof(output_accel_run_stats_t);
  if (sptr->output_accel_run_stats_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->scheduler_execute_task_function\n");
    exit(-99);
  }

  { // Set up task name and description strings
    // Set up the task_name_str
    sptr->task_name_str =
        calloc(inp->max_task_types,
               sizeof(char *)); //[MAX_TASK_TYPES][MAX_TASK_NAME_LEN];
    sched_state_size += inp->max_task_types * sizeof(char *);
    if (sptr->task_name_str == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->task_name_str\n");
      exit(-99);
    }
    char *tptr = malloc(inp->max_task_types * MAX_TASK_NAME_LEN);
    sched_state_size += inp->max_task_types * MAX_TASK_NAME_LEN;
    if (tptr == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->task_name_str char-pool\n");
      exit(-99);
    }
    for (int ti = 0; ti < inp->max_task_types; ti++) {
      sptr->task_name_str[ti] = tptr;
      tptr += MAX_TASK_NAME_LEN;
    }
    // Set up the task_desc_str
    sptr->task_desc_str =
        calloc(inp->max_task_types,
               sizeof(char *)); //[MAX_TASK_TYPES][MAX_TASK_DESC_LEN];
    sched_state_size += inp->max_task_types * sizeof(char *);
    if (sptr->task_desc_str == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->task_desc_str\n");
      exit(-99);
    }
    tptr = malloc(inp->max_task_types * MAX_TASK_DESC_LEN);
    sched_state_size += inp->max_task_types * MAX_TASK_DESC_LEN;
    if (tptr == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->task_desc_str char-pool\n");
      exit(-99);
    }
    for (int ti = 0; ti < inp->max_task_types; ti++) {
      sptr->task_desc_str[ti] = tptr;
      tptr += MAX_TASK_NAME_LEN;
    }
  }

  sptr->print_metablock_contents_function =
      malloc(inp->max_task_types * sizeof(print_metadata_block_contents_t));
  sched_state_size +=
      inp->max_task_types * sizeof(print_metadata_block_contents_t);
  if (sptr->print_metablock_contents_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->print_metablock_contents_function\n");
    exit(-99);
  }
  sptr->output_task_run_stats_function =
      malloc(inp->max_task_types * sizeof(output_task_type_run_stats_t));
  sched_state_size +=
      inp->max_task_types * sizeof(output_task_type_run_stats_t);
  if (sptr->output_task_run_stats_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->output_task_run_stats_function\n");
    exit(-99);
  }
  sptr->set_up_task_function =
      malloc(inp->max_task_types * sizeof(set_up_task_function_t));
  sched_state_size += inp->max_task_types * sizeof(set_up_task_function_t);
  if (sptr->set_up_task_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->set_up_task_function\n");
    exit(-99);
  }
  sptr->finish_task_execution_function =
      malloc(inp->max_task_types * sizeof(finish_task_execution_function_t));
  sched_state_size +=
      inp->max_task_types * sizeof(finish_task_execution_function_t);
  if (sptr->finish_task_execution_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->finish_task_execution_function\n");
    exit(-99);
  }
  sptr->auto_finish_task_function =
      malloc(inp->max_task_types * sizeof(auto_finish_task_function_t));
  sched_state_size += inp->max_task_types * sizeof(auto_finish_task_function_t);
  if (sptr->auto_finish_task_function == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->auto_finish_task_function\n");
    exit(-99);
  }

  // sptr->accelerator_in_use_by = malloc(MAX_ACCEL_TYPES
  // /*inp->max_accel_types*/ * sizeof(volatile int*));
  sptr->accelerator_in_use_by =
      malloc(inp->max_accel_types * sizeof(volatile int *));
  sched_state_size += inp->max_accel_types * sizeof(volatile int *);
  if (sptr->accelerator_in_use_by == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->accelerator_in_use_by\n");
    exit(-99);
  }
  for (int ti = 0; ti < inp->max_accel_types; ti++) {
    sptr->accelerator_in_use_by[ti] =
        malloc(sptr->max_accel_of_any_type * sizeof(volatile int));
    sched_state_size += sptr->max_accel_of_any_type * sizeof(volatile int);
    if (sptr->accelerator_in_use_by[ti] == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->accelerator_in_use_by[%u]\n",
             ti);
      exit(-99);
    }
  }

  sptr->num_accelerators_of_type = malloc(inp->max_accel_types * sizeof(int));
  sched_state_size += inp->max_accel_types * sizeof(int);
  if (sptr->num_accelerators_of_type == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->num_accelerators_of_type\n");
    exit(-99);
  }

  sptr->free_metadata_pool = calloc(inp->max_metadata_pool_blocks, sizeof(int));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(int);
  if (sptr->free_metadata_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->free_metadata_pool\n");
    exit(-99);
  }
  sptr->ready_mb_task_queue_pool = calloc(inp->max_metadata_pool_blocks,
                                          sizeof(ready_mb_task_queue_entry_t));
  sched_state_size +=
      inp->max_metadata_pool_blocks * sizeof(ready_mb_task_queue_entry_t);
  if (sptr->ready_mb_task_queue_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->ready_mb_task_queue_pool\n");
    exit(-99);
  }
  sptr->metadata_threads =
      calloc(inp->max_metadata_pool_blocks, sizeof(pthread_t));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(pthread_t);
  if (sptr->metadata_threads == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->metadata_threads\n");
    exit(-99);
  }
  sptr->critical_live_tasks_list =
      calloc(inp->max_metadata_pool_blocks, sizeof(blockid_linked_list_t));
  sched_state_size +=
      inp->max_metadata_pool_blocks * sizeof(blockid_linked_list_t);
  if (sptr->critical_live_tasks_list == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->critical_live_tasks_list\n");
    exit(-99);
  }
  sptr->free_critlist_pool = calloc(inp->max_metadata_pool_blocks, sizeof(int));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(int);
  if (sptr->free_critlist_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->free_critlist_pool\n");
    exit(-99);
  }

  // Now allocate the metadata block entries...
  sptr->master_metadata_pool =
      calloc(inp->max_metadata_pool_blocks, sizeof(task_metadata_block_t));
  sched_state_size +=
      inp->max_metadata_pool_blocks * sizeof(task_metadata_block_t);
  if (sptr->master_metadata_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
           "for sptr->master_metadata_pool\n");
    exit(-99);
  }

  for (int mi = 0; mi < inp->max_metadata_pool_blocks; mi++) {
    sptr->master_metadata_pool[mi].gets_by_task_type =
        malloc(inp->max_task_types * sizeof(uint32_t));
    sched_state_size += inp->max_task_types * sizeof(uint32_t);
    if (sptr->master_metadata_pool[mi].gets_by_task_type == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->master_metadata_pool[%u].gets_by_task_type\n",
             mi);
      exit(-99);
    }

    sptr->master_metadata_pool[mi].frees_by_task_type =
        malloc(inp->max_task_types * sizeof(uint32_t));
    sched_state_size += inp->max_task_types * sizeof(uint32_t);
    if (sptr->master_metadata_pool[mi].frees_by_task_type == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->master_metadata_pool[%u].frees_by_task_type\n",
             mi);
      exit(-99);
    }

    sptr->master_metadata_pool[mi].sched_timings.running_sec =
        calloc(inp->max_accel_types, sizeof(uint64_t)); //[MAX_ACCEL_TYPES]
    sptr->master_metadata_pool[mi].sched_timings.running_usec =
        calloc(inp->max_accel_types, sizeof(uint64_t)); //[MAX_ACCEL_TYPES]
    sched_state_size += inp->max_accel_types * 2 * sizeof(uint64_t);
    if (sptr->master_metadata_pool[mi].sched_timings.running_sec == NULL) {
      printf(
          "ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
          "for sptr->master_metadata_pool[%u].sched_timings.running_sec\n",
          mi);
      exit(-99);
    }
    if (sptr->master_metadata_pool[mi].sched_timings.running_usec == NULL) {
      printf(
          "ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
          "for sptr->master_metadata_pool[%u].sched_timings.running_usec\n",
          mi);
      exit(-99);
    }

    sptr->master_metadata_pool[mi].task_on_accel_profile =
        calloc(inp->max_accel_types, sizeof(uint64_t)); //[MAX_ACCEL_TYPES]
    sched_state_size += inp->max_accel_types * sizeof(uint64_t);
    if (sptr->master_metadata_pool[mi].task_on_accel_profile == NULL) {
      printf(
          "ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
          "for sptr->master_metadata_pool[%u].task_on_accel_profile\n",
          mi);
      exit(-99);
    }

    sptr->master_metadata_pool[mi].task_computed_on =
        calloc(inp->max_accel_types, sizeof(uint64_t)); //[MAX_ACCEL_TYPES]
    sched_state_size += inp->max_accel_types * sizeof(uint64_t);
    if (sptr->master_metadata_pool[mi].task_computed_on == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->master_metadata_pool[%u].task_computed_on\n",
             mi);
      exit(-99);
    }
    for (int ai = 0; ai < inp->max_accel_types; ai++) {
      sptr->master_metadata_pool[mi].task_computed_on[ai] =
          malloc(inp->max_task_types * sizeof(uint32_t));
      sched_state_size += inp->max_task_types * sizeof(uint32_t);
      if (sptr->master_metadata_pool[mi].task_computed_on[ai] == NULL) {
        printf(
            "ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
            "for sptr->master_metadata_pool[%u].task_computed_on[%u]\n",
            mi, ai);
        exit(-99);
      }
    }

    sptr->master_metadata_pool[mi].task_timings =
        calloc(inp->max_task_types, sizeof(task_timing_data_t));
    sched_state_size += inp->max_task_types * sizeof(task_timing_data_t);
    if (sptr->master_metadata_pool[mi].task_timings == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->master_metadata_pool[%u].task_timings\n",
             mi);
      exit(-99);
    }

    sptr->master_metadata_pool[mi].accelerator_allocated_to_MB =
        malloc(inp->max_accel_types * sizeof(uint32_t *));
    sched_state_size += inp->max_accel_types * sizeof(uint32_t *);
    if (sptr->master_metadata_pool[mi].accelerator_allocated_to_MB == NULL) {
      printf(
          "ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
          "for sptr->master_metadata_pool[%u].accelerator_allocated_to_MB\n",
          mi);
      exit(-99);
    }
    for (int ti = 0; ti < inp->max_accel_types; ti++) {
      sptr->master_metadata_pool[mi].accelerator_allocated_to_MB[ti] =
          malloc(sptr->max_accel_of_any_type * sizeof(uint32_t));
      sched_state_size += sptr->max_accel_of_any_type * sizeof(uint32_t);
      if (sptr->master_metadata_pool[mi].accelerator_allocated_to_MB[ti] ==
          NULL) {
        printf(
            "ERROR: get_new_scheduler_datastate_pointer cannot allocate memory "
            "for "
            "sptr->master_metadata_pool[%u].accelerator_allocated_to_MB[%u]\n",
            mi, ti);
        exit(-99);
      }
    }

    sptr->master_metadata_pool[mi].data_space =
        malloc(inp->max_data_space_bytes);
    sched_state_size += inp->max_data_space_bytes;
    if (sptr->master_metadata_pool[mi].data_space == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate "
             "memory for sptr->master_metadata_pool[%u].data_space\n",
             mi);
      exit(-99);
    }
  }

  printf("TOTAL Schedule DataState Size is %lu bytes\n", sched_state_size);
  printf("Now calling initialize_scheduler...\n");
  status_t init_status = initialize_scheduler(sptr);
  if (init_status != success) {
    printf(
        "ERROR : Scheduiler initialization returned an error indication...\n");
    exit( -1);
  }
  return sptr;
}

// This function proviudes for a user to presenta  configuration file (of a
// particular format) which
//  contains the desired scheduler datastate input parms (to be changed from
//  default) and thus invoke the full initialization in a single call
// Under the covers it uses the above routines and lower-level API interface...
scheduler_datastate_block_t *
initialize_scheduler_from_config_file(char *config_file_name) {
  DEBUG(printf("In the initialize_scheduler_from_config_file routine...\n"));
  FILE *cf = fopen(config_file_name, "r");
  if (cf == NULL) {
    printf("ERROR : Could not open the Configuration input file `%s`\n",
           config_file_name);
    exit(-1);
  }

  // This gets the scheduler datastate input parms set to default values
  scheduler_get_datastate_in_parms_t *sched_inparms =
      get_scheduler_datastate_input_parms();
  DEBUG(printf("  got the sched_inparms\n"));

  // Scan the file and apply the updates into the input parms
  //  Note: this uses very rudimentary string parsing, etc.
  char sched_accel_enum_names[SCHED_MAX_ACCEL_TYPES][64];
  snprintf(sched_accel_enum_names[SCHED_CPU_ACCEL_T], 64, "SCHED_CPU_ACCEL_T");
  snprintf(sched_accel_enum_names[SCHED_EPOCHS_1D_FFT_ACCEL_T], 64,
           "SCHED_EPOCHS_1D_FFT_ACCEL_T");
  snprintf(sched_accel_enum_names[SCHED_EPOCHS_VITDEC_ACCEL_T], 64,
           "SCHED_EPOCHS_VITDEC_ACCEL_T");
  snprintf(sched_accel_enum_names[SCHED_EPOCHS_CV_CNN_ACCEL_T], 64,
           "SCHED_EPOCHS_CV_CNN_ACCEL_T");

  DEBUG(printf("  Scanning the configuration file `%s`\n", config_file_name));
  char parm_id[256];
  char setting[256];
  while ((fscanf(cf, "%s = %s\n", parm_id, setting) == 2) || (!feof(cf))) {
    if (strcmp(parm_id, "MAX_TASK_TYPES") == 0) {
      sched_inparms->max_task_types = atoi(setting);
      DEBUG(printf("  Set inparms->max_task_types = %d\n",
                   sched_inparms->max_task_types));
    } else if (strcmp(parm_id, "MAX_ACCEL_TYPES") == 0) {
      sched_inparms->max_accel_types = atoi(setting);
      DEBUG(printf("  Set inparms->max_accel_types = %d\n",
                   sched_inparms->max_accel_types));
    } else if (strcmp(parm_id, "MAX_METADATA_POOL_BLOCKS") == 0) {
      sched_inparms->max_metadata_pool_blocks = atoi(setting);
      DEBUG(printf("  Set inparms->max_metadata_pool_blocks = %d\n",
                   sched_inparms->max_metadata_pool_blocks));
    } else if (strcmp(parm_id, "MAX_DATA_SPACE_BYTES") == 0) {
      sched_inparms->max_data_space_bytes = atoi(setting);
      DEBUG(printf("  Set inparms->max_data_space_bytes = %d\n",
                   sched_inparms->max_data_space_bytes));
    } else if (strcmp(parm_id, "MAX_TASK_TIMING_SETS") == 0) {
      sched_inparms->max_task_timing_sets = atoi(setting);
      DEBUG(printf("  Set inparms-> = %d\n",
                   sched_inparms->max_task_timing_sets));
    } else if (strcmp(parm_id, "SCHEDULER_HOLDOFF_USEC") == 0) {
      sched_inparms->scheduler_holdoff_usec = atoi(setting);
      DEBUG(printf("  Set inparms->scheduler_holdoff_usec = %d\n",
                   sched_inparms->scheduler_holdoff_usec));
    } else if (strcmp(parm_id, "POLICY") == 0) {
      snprintf(sched_inparms->policy, 255, "%s", setting);
      DEBUG(printf("  Set inparms->policy = %s\n", sched_inparms->policy));
    } else if (strcmp(parm_id, "VISUALIZER_OUTPUT_ENABLED") == 0) {
      sched_inparms->visualizer_output_enabled = atoi(setting);
      DEBUG(printf("  Set inparms->visualizer_output_enabled = %d\n",
                   sched_inparms->visualizer_output_enabled));
    } else if (strcmp(parm_id, "VISUALIZER_TASK_START_COUNT") == 0) {
      sched_inparms->visualizer_task_start_count = atoi(setting);
      DEBUG(printf("  Set inparms->visualizer_task_start_count = %d\n",
                   sched_inparms->visualizer_task_start_count));
    } else if (strcmp(parm_id, "VISUALIZER_TASK_STOP_COUNT") == 0) {
      sched_inparms->visualizer_task_stop_count = atoi(setting);
      DEBUG(printf("  Set inparms->visualizer_task_stop_count = %d\n",
                   sched_inparms->visualizer_task_stop_count));
    } else if (strcmp(parm_id, "VISUALIZER_TASK_ENABLE_TYPE") == 0) {
      sched_inparms->visualizer_task_enable_type = atoi(setting);
      DEBUG(printf("  Set inparms->visualizer_task_enable_type = %d\n",
                   sched_inparms->visualizer_task_enable_type));
    } else if (strcmp(parm_id, "SL_VIZ_FNAME") == 0) {
      snprintf(sched_inparms->sl_viz_fname, 255, "%s", setting);
      DEBUG(printf("  Set inparms->sl_viz_fname = %s\n",
                   sched_inparms->sl_viz_fname));
    } else {
      char *search_str = "MAX_ACCEL_TO_USE_FROM_POOL_";
      if (strncmp(parm_id, search_str, strlen(search_str)) == 0) {
        for (int i = 0; i < SCHED_MAX_ACCEL_TYPES; i++) {
          if (strncmp(&(parm_id[strlen(search_str)]), sched_accel_enum_names[i],
                      64) == 0) {
            sched_inparms->max_accel_to_use_from_pool[i] = atoi(setting);
            DEBUG(printf("  Set inparms->max_accel_to_use_from_pool[%d] = %d\n",
                         i, sched_inparms->max_accel_to_use_from_pool[i]));
            i = SCHED_MAX_ACCEL_TYPES;
          }
        }
      } else {
        printf("Scheduler ignoring parm `%s` with value `%s`\n", parm_id,
               setting);
      }
    }
  } // while (scan through config file)

  // Now initialize the scheduler and return a datastate space pointer
  printf("Calling get_new_scheduler_datastate_pointer...\n");
  scheduler_datastate_block_t *sptr =
      initialize_scheduler_and_return_datastate_pointer(sched_inparms);

  return sptr;
}

// This function is really only executed once, at the very start of scheduler
// lifetime,
//  to set up global scheduler state related to the hardware environment
#include "cpu_accel.h"
#include "cv_accel.h"
#include "fft_accel.h"
#include "vit_accel.h"

void set_up_scheduler() {
  printf( "Setting up the Global Scheduler Hardware State (System Accelerators)\n");
  // Set up the "CPU" (threada/accelerators)
  printf("Setting up the Accel %u CPU (thread) Accelerators...\n", SCHED_CPU_ACCEL_T);
  sprintf(global_hardware_state_block.accel_name_str[SCHED_CPU_ACCEL_T], "CPU-Acc");
  sprintf(global_hardware_state_block.accel_desc_str[SCHED_CPU_ACCEL_T], "Run task on a RISC-V CPU thread");
  global_hardware_state_block.num_accelerators_of_type[SCHED_CPU_ACCEL_T] = NUM_CPU_ACCEL;
  global_hardware_state_block.do_accel_init_function[SCHED_CPU_ACCEL_T] = &do_cpu_accel_type_initialization;
  global_hardware_state_block.do_accel_closeout_function[SCHED_CPU_ACCEL_T] = &do_cpu_accel_type_closeout;
  global_hardware_state_block.output_accel_run_stats_function[SCHED_CPU_ACCEL_T] = &output_cpu_accel_type_run_stats;
  // Now initialize this accelerator
  if (global_hardware_state_block.do_accel_init_function[SCHED_CPU_ACCEL_T] != NULL) {
    // DEBUG(
    printf(" Calling the accelerator initialization function...\n"); //);
    global_hardware_state_block.do_accel_init_function[SCHED_CPU_ACCEL_T](NULL);
  } else {
    printf("Note: accelerator initialization function is NULL\n");
  }

  // Set up the Viterbi Decoder HWR accelerators
  printf("Setting up the Accel %u  Viterbi Decoder Hardware Accelerators...\n", SCHED_EPOCHS_VITDEC_ACCEL_T);
  sprintf(global_hardware_state_block.accel_name_str[SCHED_EPOCHS_VITDEC_ACCEL_T], "VIT-HW-Acc");
  sprintf(global_hardware_state_block.accel_desc_str[SCHED_EPOCHS_VITDEC_ACCEL_T], "Run task on the Viterbi-Decode Hardware Accelerator");
  global_hardware_state_block.num_accelerators_of_type[SCHED_EPOCHS_VITDEC_ACCEL_T] = NUM_VIT_ACCEL;
  global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_VITDEC_ACCEL_T] = &do_vit_accel_type_initialization;
  global_hardware_state_block.do_accel_closeout_function[SCHED_EPOCHS_VITDEC_ACCEL_T] = &do_vit_accel_type_closeout;
  global_hardware_state_block.output_accel_run_stats_function[SCHED_EPOCHS_VITDEC_ACCEL_T] = &output_vit_accel_type_run_stats;
  // Now initialize this accelerator
  if (global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_VITDEC_ACCEL_T] != NULL) {
    // DEBUG(
    printf(" Calling the accelerator initialization function...\n"); //);
    global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_VITDEC_ACCEL_T](NULL);
  } else {
    printf("Note: accelerator initialization function is NULL\n");
  }

  printf("Setting up the Accel %u 1-D FFT Hardware Accelerators...\n", SCHED_EPOCHS_1D_FFT_ACCEL_T);
  sprintf(global_hardware_state_block.accel_name_str[SCHED_EPOCHS_1D_FFT_ACCEL_T], "FFT-HW-Acc");
  sprintf(global_hardware_state_block.accel_desc_str[SCHED_EPOCHS_1D_FFT_ACCEL_T], "Run task on the 1-D FFT Hardware Accelerator");
  global_hardware_state_block.num_accelerators_of_type[SCHED_EPOCHS_1D_FFT_ACCEL_T] = NUM_FFT_ACCEL;
  global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_1D_FFT_ACCEL_T] = &do_fft_accel_type_initialization;
  global_hardware_state_block.do_accel_closeout_function[SCHED_EPOCHS_1D_FFT_ACCEL_T] = &do_fft_accel_type_closeout;
  global_hardware_state_block.output_accel_run_stats_function[SCHED_EPOCHS_1D_FFT_ACCEL_T] = &output_fft_accel_type_run_stats;
  // Now initialize this accelerator
  if (global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_1D_FFT_ACCEL_T] != NULL) {
    // DEBUG(
    printf(" Calling the accelerator initialization function...\n"); //);
    global_hardware_state_block .do_accel_init_function[SCHED_EPOCHS_1D_FFT_ACCEL_T](NULL);
  } else {
    printf("Note: accelerator initialization function is NULL\n");
  }

  printf("Setting up the Accel %u NVDLA CV/CNN Hardware Accelerators...\n", SCHED_EPOCHS_CV_CNN_ACCEL_T);
  sprintf(global_hardware_state_block.accel_name_str[SCHED_EPOCHS_CV_CNN_ACCEL_T], "CV-HW-Acc");
  sprintf(global_hardware_state_block.accel_desc_str[SCHED_EPOCHS_CV_CNN_ACCEL_T], "Run task on the CV/CNN NVDLA Hardware Accelerator");
  global_hardware_state_block.num_accelerators_of_type[SCHED_EPOCHS_CV_CNN_ACCEL_T] = NUM_CV_ACCEL;
  global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_CV_CNN_ACCEL_T] = &do_cv_accel_type_initialization;
  global_hardware_state_block.do_accel_closeout_function[SCHED_EPOCHS_CV_CNN_ACCEL_T] = &do_cv_accel_type_closeout;
  global_hardware_state_block.output_accel_run_stats_function[SCHED_EPOCHS_CV_CNN_ACCEL_T] = &output_cv_accel_type_run_stats;
  // Now initialize this accelerator
  if (global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_CV_CNN_ACCEL_T] != NULL) {
    // DEBUG(
    printf(" Calling the accelerator initialization function...\n"); //);
    global_hardware_state_block.do_accel_init_function[SCHED_EPOCHS_CV_CNN_ACCEL_T](NULL);
  } else {
    printf("Note: accelerator initialization function is NULL\n");
  }
}

status_t
initialize_scheduler(scheduler_datastate_block_t *sptr) //, char* sl_viz_fname)
{
  DEBUG(printf("In initialize...\n"));
  // Currently we set this to a fixed a-priori number...
  sptr->total_metadata_pool_blocks = sptr->inparms->max_metadata_pool_blocks;

  sptr->next_avail_task_type = 0;
  sptr->next_avail_accel_id = 0;
  sptr->next_avail_DAG_id = 0;

  sptr->free_metadata_blocks = sptr->total_metadata_pool_blocks;
  sptr->num_free_task_queue_entries = 0;
  sptr->num_tasks_in_ready_queue = 0;
  sptr->critical_live_task_head = NULL;
  sptr->free_critlist_entries = sptr->total_metadata_pool_blocks;
  sptr->total_critical_tasks = 0;

  snprintf(sptr->task_criticality_str[0], 32, "%s", "NO-TASK");
  snprintf(sptr->task_criticality_str[1], 32, "%s", "NBASETASK");
  snprintf(sptr->task_criticality_str[2], 32, "%s", "ELEVATED-TASK");
  snprintf(sptr->task_criticality_str[3], 32, "%s", "CRITICAL-TASK");

  snprintf(sptr->task_status_str[0], 32, "%s", "TASK-FREE");
  snprintf(sptr->task_status_str[1], 32, "%s", "TASK-ALLOCATED");
  snprintf(sptr->task_status_str[2], 32, "%s", "TASK-QUEUED");
  snprintf(sptr->task_status_str[3], 32, "%s", "TASK-RUNING");
  snprintf(sptr->task_status_str[4], 32, "%s", "TASK-DONE");

  // Initialize scheduler-decision statistics
  sptr->scheduler_decision_time_usec = 0;
  sptr->scheduler_decisions = 0;
  sptr->scheduler_decision_checks = 0;

  sptr->sl_viz_fp = NULL;

  if (sptr->inparms->visualizer_output_enabled) {
    // sptr->sl_viz_fp = fopen("./sl_viz.trace", "w");
    sptr->sl_viz_fp = fopen(sptr->inparms->sl_viz_fname, "w");
    if (sptr->sl_viz_fp == NULL) {
      printf("ERROR: Cannot open the output vizualizer file '%s' - exiting\n",
             sptr->inparms->sl_viz_fname);
      exit( -1);
    }
    fprintf(sptr->sl_viz_fp,
            "sim_time,task_dag_id,task_tid,task_name,task_crit,dag_dtime,id,"
            "type,task_parent_ids,task_arrival_time,curr_job_start_time,curr_"
            "job_end_time\n");
  }

  // Dynamically load the scheduling policy (plug-in) to use, and initialize it
  char policy_filename[300];
  snprintf(policy_filename, 270, "%s", sptr->inparms->policy);
  if ((sptr->policy_handle = dlopen(policy_filename, RTLD_LAZY)) == NULL) {
    printf("Could not open plug-in scheduling policy: %s\n", dlerror());
    exit( -1);
  }

  if (dlerror() != NULL) {
    dlclose(sptr->policy_handle);
    printf("Function initialize_policy() not found in scheduling policy %s\n",
           policy_filename);
    exit( -1);
  }

  sptr->initialize_assign_task_to_pe =
      dlsym(sptr->policy_handle, "initialize_assign_task_to_pe");
  if (dlerror() != NULL) {
    dlclose(sptr->policy_handle);
    printf("Function initialize_assign_task_to_pe() not found in scheduling "
           "policy %s\n",
           policy_filename);
    exit( -1);
  }

  sptr->assign_task_to_pe = dlsym(sptr->policy_handle, "assign_task_to_pe");
  if (dlerror() != NULL) {
    dlclose(sptr->policy_handle);
    printf("Function assign_task_to_pe() not found in scheduling policy %s\n",
           policy_filename);
    exit( -1);
  }

  pthread_mutex_init(&(sptr->free_metadata_mutex), NULL);
  pthread_mutex_init(&(sptr->accel_alloc_mutex), NULL);
  pthread_mutex_init(&(sptr->task_queue_mutex), NULL);
  pthread_mutex_init(&(sptr->sl_viz_out_mutex), NULL);

  struct timeval init_time;
  gettimeofday(&init_time, NULL);
  /*sptr->last_accel_use_update_time = init_time; // Start accounting at init
   * time... ? */
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    sptr->master_metadata_pool[i].scheduler_datastate_pointer = sptr;
    sptr->master_metadata_pool[i].block_id =
        i; // Set the master pool's block_ids
    for (int ji = 0; ji < sptr->inparms->max_task_types; ji++) {
      sptr->master_metadata_pool[i].gets_by_task_type[ji] = 0;
      sptr->master_metadata_pool[i].frees_by_task_type[ji] = 0;
    }
    // Clear the (full-run, aggregate) timing data spaces
    gettimeofday(&(sptr->master_metadata_pool[i].sched_timings.idle_start),
                 NULL);
    // Scheduler timings
    sptr->master_metadata_pool[i].sched_timings.idle_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.idle_usec = 0;
    sptr->master_metadata_pool[i].sched_timings.get_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.get_usec = 0;
    sptr->master_metadata_pool[i].sched_timings.queued_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.queued_usec = 0;
    for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
      sptr->master_metadata_pool[i].sched_timings.running_sec[ti] = 0;
      sptr->master_metadata_pool[i].sched_timings.running_usec[ti] = 0;
    }
    sptr->master_metadata_pool[i].sched_timings.done_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.done_usec = 0;
    // Reset all the per-task type and targets timing data, too.
    for (int ai = 0; ai < sptr->inparms->max_accel_types; ai++) {
      for (int ti = 0; ti < sptr->inparms->max_task_types; ti++) {
        sptr->master_metadata_pool[i].task_computed_on[ai][ti] = 0;
      }
    }

    // And some allocation stats stuff:
    for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
      for (int ai = 0; ai < sptr->max_accel_of_any_type; ai++) {
        sptr->master_metadata_pool[i].accelerator_allocated_to_MB[ti][ai] = 0;
      }
    }

    pthread_mutex_init(&(sptr->master_metadata_pool[i].metadata_mutex), NULL);
    pthread_cond_init(&(sptr->master_metadata_pool[i].metadata_condv), NULL);

    sptr->free_metadata_pool[i] = i; // Set up all blocks are free
    sptr->free_critlist_pool[i] = i; // Set up all critlist blocks are free
  }

  // Now set up the task ready queue
  DEBUG(printf("Setting up the task ready queue...\n"));
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    sptr->ready_mb_task_queue_pool[i].unique_id = i;
    sptr->ready_mb_task_queue_pool[i].block_id = -1; // id -1 = unassigned
    if (i > 0) {
      sptr->ready_mb_task_queue_pool[i].prev =
          &(sptr->ready_mb_task_queue_pool[i - 1]);
    } else {
      sptr->ready_mb_task_queue_pool[i].prev = NULL;
    }
    if (i < (sptr->total_metadata_pool_blocks - 1)) {
      sptr->ready_mb_task_queue_pool[i].next =
          &(sptr->ready_mb_task_queue_pool[i + 1]);
    } else {
      sptr->ready_mb_task_queue_pool[i].next = NULL;
    }
    DEBUG(printf("  set pool[%2u] @ %p id %i prev %p next %p\n", i,
                 &(sptr->ready_mb_task_queue_pool[i]),
                 sptr->ready_mb_task_queue_pool[i].block_id,
                 sptr->ready_mb_task_queue_pool[i].prev,
                 sptr->ready_mb_task_queue_pool[i].next));
  } // for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
  sptr->free_ready_mb_task_queue_entries = &(sptr->ready_mb_task_queue_pool[0]);
  sptr->ready_mb_task_queue_head = NULL;
  sptr->ready_mb_task_queue_tail = NULL;
  sptr->num_free_task_queue_entries = sptr->total_metadata_pool_blocks;
  DEBUG(printf(" AND free_ready_mb_task_queue_entries = %p\n",
               sptr->free_ready_mb_task_queue_entries));

  // Now initialize the per-metablock threads
  // For portability (as per POSIX documentation) explicitly create threads in
  // joinable state
  pthread_attr_t pt_attr;
  pthread_attr_init(&pt_attr);
  pthread_attr_setdetachstate(&pt_attr, PTHREAD_CREATE_JOINABLE);
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    if (pthread_create(&(sptr->metadata_threads[i]), &pt_attr,
                       metadata_thread_wait_for_task,
                       &(sptr->master_metadata_pool[i]))) {
      printf( "ERROR: Scheduler failed to create thread for metadata block: %d\n", i);
      exit( -10);
    }
    sptr->master_metadata_pool[i].thread_id = sptr->metadata_threads[i];
  }

  for (int ti = 0; ti < sptr->inparms->max_task_types; ti++) {
    sptr->allocated_metadata_blocks[ti] = 0;
    sptr->freed_metadata_blocks[ti] = 0;
  }

  for (int i = 0; i < sptr->inparms->max_accel_types; i++) {
    sptr->num_accelerators_of_type[i] = 0;
  }

  for (int i = 0; i < sptr->inparms->max_accel_types; i++) {
    for (int j = 0; j < sptr->max_accel_of_any_type; j++) {
      sptr->accelerator_in_use_by[i][j] =
          -1; // NOT a valid metadata block ID; -1 indicates "Not in Use"
    }
  }

  /*  for (int i = 0; i < NUM_CPU_ACCEL+1; i++) {
    for (int j = 0; j < NUM_FFT_ACCEL+1; j++) {
      for (int k = 0; k < NUM_VIT_ACCEL+1; k++) {
        for (int l = 0; l < NUM_CV_ACCEL+1; l++) {
          sptr->in_use_accel_times_array[i][j][k][l] = 0;
        }
      }
    }
    }*/

  // Set up the Scheduler's Execution-Task-Function Table (for now by hand)
  for (int i = 0; i < sptr->inparms->max_task_types; i++) {
    for (int j = 0; j < sptr->inparms->max_accel_types; j++) {
      sptr->scheduler_execute_task_function[j][i] =
          NULL; // Set all to default to NULL
    }
    sptr->print_metablock_contents_function[i] = NULL;
    sptr->output_task_run_stats_function[i] = NULL;
  }

  for (int j = 0; j < sptr->inparms->max_accel_types; j++) {
    sptr->do_accel_init_function[j] = NULL;
    sptr->do_accel_closeout_function[j] = NULL;
    sptr->output_accel_run_stats_function[j] = NULL;
  }

  // Now start the "schedule_executions_from_queue() pthread -- using the
  // DETACHED pt_attr
  int pt_ret = pthread_create(&(sptr->scheduling_thread), &pt_attr, schedule_executions_from_queue, (void *)(sptr));
  if (pt_ret != 0) {
    printf("Could not start the scheduler pthread... return value %d\n", pt_ret);
    exit( -1);
  }

  // Now set up the pool of accelerators that this applicaiton will use (from
  // their
  printf("\nRegistering the ACCELERATOR Usage...\n");
  unsigned total_accelerators_allocated = 0;
  for (int i = 0; i < SCHED_MAX_ACCEL_TYPES; i++) {
    int desired_num = sptr->inparms->max_accel_to_use_from_pool[i];
    // printf("Requested to allocate %d Accel from pool %u\n", desired_num, i);
    if (desired_num != 0) {
      if (global_hardware_state_block.num_accelerators_of_type[i] == 0) {
        if (desired_num < 0) {
          printf(
              "WARNING: Requested all-avail Accel for empty Pool %u : %s %s\n",
              i, global_hardware_state_block.accel_name_str[i],
              global_hardware_state_block.accel_desc_str[i]);
        } else {
          printf("WARNING: Requested %u Accel for empty Pool %u : %s %s\n",
                 desired_num, i, global_hardware_state_block.accel_name_str[i],
                 global_hardware_state_block.accel_desc_str[i]);
        }
      }
      accelerator_type_t acid = sptr->next_avail_accel_id;
      if (acid >= sptr->inparms->max_accel_types) {
        printf("ERROR: Ran out of Accel IDs: MAX_ACCEL_ID = %u and we are "
               "adding id %u\n",
               sptr->inparms->max_accel_types, acid);
        exit( -32);
      }
      sptr->map_sched_accel_type_to_local_accel_type[i] = acid;
      printf("map_sched_accel_type_to_local_accel_type[%u] = %u\n", i, acid);
      sptr->next_avail_accel_id += 1;
      printf("sptr->next_avail_accel_id now = %u\n", sptr->next_avail_accel_id);
      if (desired_num >
          global_hardware_state_block.num_accelerators_of_type[i]) {
        printf("ERROR: Specified desired number of accelerators ( %u ) is more "
               "than available in the hardware ( %u )\n",
               desired_num,
               global_hardware_state_block.num_accelerators_of_type[i]);
        exit( -33);
      }
      if (desired_num > sptr->max_accel_of_any_type) {
        printf("ERROR: Specified desired number of accelerators ( %u ) is more "
               "than specified max_accel_of_any_type ( %u )\n",
               desired_num, sptr->max_accel_of_any_type);
        exit( -34);
      }
      unsigned alloc_num;
      if (desired_num < 0) {
        // This means all available
        alloc_num = global_hardware_state_block.num_accelerators_of_type[i];
      } else {
        // Set the max number ot the desired number
        alloc_num = desired_num;
      }
      sptr->num_accelerators_of_type[acid] = alloc_num;
      total_accelerators_allocated += alloc_num;
      printf(
          "Setting to use %d from Accelerator Pool %u : SL Accel %u %s : %s\n",
          alloc_num, acid, i, global_hardware_state_block.accel_name_str[i],
          global_hardware_state_block.accel_desc_str[i]);
      sptr->do_accel_init_function[acid] =
          global_hardware_state_block.do_accel_init_function[i];
      sptr->do_accel_closeout_function[acid] =
          global_hardware_state_block.do_accel_closeout_function[i];
      sptr->output_accel_run_stats_function[acid] =
          global_hardware_state_block.output_accel_run_stats_function[i];
      sprintf(sptr->accel_name_str[acid], "%s", global_hardware_state_block.accel_name_str[i]);
      sprintf(sptr->accel_desc_str[acid], "%s", global_hardware_state_block.accel_desc_str[i]);
    } // if (desired_num != 0)
  }   // for (itn i = 0 .. SCHED_MAX_ACCEL_TYPES)
  if (total_accelerators_allocated == 0) {
    printf("WARNING: No scheduler accelerator (pools) were requested -- no "
           "accelerators allocated for scheduler to schedule\n");
  }

  /**
   // Let's make sure these are detached?
   for (int i = 0; i < total_metadata_pool_blocks; i++) {
   pthread_detach(metadata_threads[i]);
   }
   pthread_detach(scheduling_thread);
  **/

  DEBUG(printf("DONE with initialize -- returning success\n"));
  return success;
}

void release_accelerator_for_task(task_metadata_block_t *task_metadata_block) {
  scheduler_datastate_block_t *sptr =
      task_metadata_block->scheduler_datastate_pointer;
  unsigned mdb_id = task_metadata_block->block_id;
  unsigned accel_type = task_metadata_block->accelerator_type;
  unsigned accel_id = task_metadata_block->accelerator_id;
  pthread_mutex_lock(&(sptr->accel_alloc_mutex));

  // printf("MB%u RELEASE  accelerator %u %u for %d cl %u\n", mdb_id,
  // accel_type, accel_id, accelerator_in_use_by[accel_type][accel_id],
  // task_metadata_block->crit_level);
  DEBUG(
      printf(" RELEASE accelerator %u  %u  = %d  : ", accel_type, accel_id,
             sptr->accelerator_in_use_by[accel_type][accel_id]);
      for (int ai = 0; ai < sptr->num_accelerators_of_type[accel_type]; ai++) {
        printf("%u %d : ", ai, sptr->accelerator_in_use_by[accel_type][ai]);
      } printf("\n"));
  if (sptr->accelerator_in_use_by[accel_type][accel_id] != mdb_id) {
    printf("ERROR - in release_accelerator_for_task for ACCEL %s Num %d but "
           "BLOCK_ID Mismatch: %d vs %d\n",
           sptr->accel_name_str[accel_type], accel_id,
           sptr->accelerator_in_use_by[accel_type][accel_id], mdb_id);
    printf("  this occurred on finish of block:\n");
    print_base_metadata_block_contents(task_metadata_block);
    printf("Accelerators Info:\n");
    for (int ai = 0; ai < sptr->num_accelerators_of_type[accel_type]; ai++) {
      printf(" accelerator_in_use_by[ %u ][ %u ] = %d\n", accel_type, ai,
             sptr->accelerator_in_use_by[accel_type][ai]);
    }
  } else {
    account_accelerators_in_use_interval(sptr);
    sptr->accelerator_in_use_by[accel_type][accel_id] =
        -1; // Indicates "Not in Use"
  }
  pthread_mutex_unlock(&(sptr->accel_alloc_mutex));
}

// This routine schedules (the first) ready task from the ready task queue
// The input parm is a pointer to a scheduler_datastate_block_t structure
void *schedule_executions_from_queue(void *void_parm_ptr) {
  DEBUG(printf("SCHED: starting execution of schedule_executions_from_queue thread...\n"));
  // Set up the pthread_cancel behaviors
  pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t *)void_parm_ptr;
  // This will now be an eternally-running scheduler process, I think.
  // pthread_mutex_lock(&schedule_from_queue_mutex);
  while (1) {
    // If there is something in the task ready queue;
    if (sptr->num_tasks_in_ready_queue > 0) {
      DEBUG(printf("SCHED: num_tasks_in_ready_queue = %u\n",
                   sptr->num_tasks_in_ready_queue);
            int ti = 0;
            ready_mb_task_queue_entry_t *t_te = sptr->ready_mb_task_queue_head;
            while (t_te != NULL) {
              printf("SCHED:    Ready_Task_Entry %2u : MB%u  %p %p\n", ti,
                     t_te->block_id, t_te->prev, t_te->next);
              ti++;
              t_te = t_te->next;
            });

      // Get pointer to the first task on the ready queue
      ready_mb_task_queue_entry_t *ready_task_entry =
          sptr->ready_mb_task_queue_head;
      ready_mb_task_queue_entry_t *selected_task_entry = NULL;
      task_metadata_block_t *task_metadata_block = NULL;

      // Select the target accelerator to execute the task
      DEBUG(printf("SCHED: calling assign_task_to_pe\n"));
      // Pass the head of the ready queue to parse entries in the queue
      selected_task_entry = sptr->assign_task_to_pe(sptr, ready_task_entry);
      if (selected_task_entry == NULL) {
        // No schedulable task
        continue;
      } else {
        task_metadata_block =
            &(sptr->master_metadata_pool[selected_task_entry->block_id]);
      }
      unsigned int accel_type = task_metadata_block->accelerator_type;
      unsigned int accel_id = task_metadata_block->accelerator_id;
      DEBUG(printf("SCHED: MB%u Selected accel type: %d id: accel_id: %d\n",
                   task_metadata_block->block_id,
                   task_metadata_block->accelerator_type,
                   task_metadata_block->accelerator_id));

      if (accel_type == NO_Accelerator) {
        printf("SCHED: ERROR : Selected Task has no accelerator assigned\n");
        // pthread_mutex_unlock(&schedule_from_queue_mutex);
        print_base_metadata_block_contents(task_metadata_block);
        exit( -19);
      } else {
        // Mark the requested accelerator as "In-USE" by this metadata block
        if (sptr->accelerator_in_use_by[accel_type][accel_id] != -1) {
          printf("ERROR : schedule_executions_from_queue is trying to allocate "
                 "ACCEL %s %u which is already allocated to Block %u\n",
                 sptr->accel_name_str[accel_type], accel_id,
                 sptr->accelerator_in_use_by[accel_type][accel_id]);
          exit( -14);
        }
        account_accelerators_in_use_interval(sptr);
        int bi = task_metadata_block->block_id; // short name for the block_id
        sptr->accelerator_in_use_by[accel_type][accel_id] = bi;
        sptr->master_metadata_pool[bi]
            .accelerator_allocated_to_MB[accel_type][accel_id] += 1;
        // Okay -- we can allocate to the accelerator -- remove from the queue
        // printf("MB%u ALLOCATE accelerator %u %u to  %d cl %u\n", bi,
        // accel_type, accel_id, bi, task_metadata_block->crit_level);
        DEBUG(printf("SCHED: MB%u ALLOC accelerator %u  %u to %d  : ", bi,
                     accel_type, accel_id, bi);
              for (int ai = 0; ai < sptr->num_accelerators_of_type[accel_type];
                   ai++) {
                printf("%u %d : ", ai,
                       sptr->accelerator_in_use_by[accel_type][ai]);
              } printf("\n"));
        // Update the ready task queue... Connect selected_task_entry.prev->next
        // = selected_task_entry.next

        DEBUG(printf("SCHED: Updating the task ready queue...\n"));
        pthread_mutex_lock(&(sptr->task_queue_mutex));

        if (selected_task_entry->prev == NULL) {
          // This was the HEAD of the ready queue
          sptr->ready_mb_task_queue_head = selected_task_entry->next;
          DEBUG(printf("SCHED:   Removed HEAD of queue\n"));
        } else {
          selected_task_entry->prev->next = selected_task_entry->next;
          DEBUG(printf("SCHED:   Removed internal entry of queue\n"));
        }
        if (selected_task_entry->next == NULL) {
          // This is the TAIL of the ready queue
          sptr->ready_mb_task_queue_tail = selected_task_entry->prev;
          DEBUG(printf("SCHED:   Removed TAIL of queue\n"));
        } else {
          selected_task_entry->next->prev = selected_task_entry->prev;
          DEBUG(printf("SCHED:   Removed internal entry of queue\n"));
        }
        sptr->num_tasks_in_ready_queue--;
        DEBUG(printf("SCHED:   Set num_tasks_in_ready_queue to %u\n",
                     sptr->num_tasks_in_ready_queue));

        DEBUG(printf("SCHED:   Adding back the ready task entry to the free "
                     "list pre: %u entries\n",
                     sptr->num_free_task_queue_entries));
        // Prepend to the free_ready_mb_task_queue_entries;
        if (sptr->free_ready_mb_task_queue_entries != NULL) {
          // There are other free task queue entries
          sptr->free_ready_mb_task_queue_entries->prev = selected_task_entry;
          selected_task_entry->next = sptr->free_ready_mb_task_queue_entries;
        }
        selected_task_entry->prev =
            NULL; // As head of the list, the prev should be NULL
        sptr->free_ready_mb_task_queue_entries = selected_task_entry;
        sptr->num_free_task_queue_entries++;
        DEBUG(printf("SCHED:   Prepended to FREE ready task queue, with %u "
                     "entries now\n",
                     sptr->num_free_task_queue_entries));
        SDEBUG(print_free_ready_tasks_list(sptr));
        /* // And clean up the ready task storage... */
        /* ready_task_entry->block_id = -1; */
        /* ready_task_entry->next = NULL; */
        /* ready_task_entry->prev = NULL; */
        // And unlock the task-queue mutex
        pthread_mutex_unlock(&(sptr->task_queue_mutex));

        // Set the task to "RUNNING" State and account the times...
        task_metadata_block->status = TASK_RUNNING; // running

        gettimeofday(
            &sptr->master_metadata_pool[bi].sched_timings.running_start, NULL);
        sptr->master_metadata_pool[bi].sched_timings.queued_sec +=
            sptr->master_metadata_pool[bi].sched_timings.running_start.tv_sec -
            sptr->master_metadata_pool[bi].sched_timings.queued_start.tv_sec;
        sptr->master_metadata_pool[bi].sched_timings.queued_usec +=
            sptr->master_metadata_pool[bi].sched_timings.running_start.tv_usec -
            sptr->master_metadata_pool[bi].sched_timings.queued_start.tv_usec;

        TDEBUG(
            printf("Kicking off accelerator task for Metadata Block %u : Task "
                   "%s %s on Accel %s %u\n",
                   bi, sptr->task_name_str[task_metadata_block->task_type],
                   sptr->task_criticality_str[task_metadata_block->crit_level],
                   sptr->accel_name_str[task_metadata_block->accelerator_type],
                   task_metadata_block->accelerator_id));

        // Lock the mutex associated to the conditional variable
        pthread_mutex_lock(&(task_metadata_block->metadata_mutex));

        // Signal the conditional variable -- triggers the target thread
        // execution of accelerator
        pthread_cond_signal(&(task_metadata_block->metadata_condv));

        // And now we unlock because we are done here...
        pthread_mutex_unlock(&(task_metadata_block->metadata_mutex));
      }
    } else { // if (num_tasks_in_queue > 0)
      // I think perhaps we should add a short delay here to avoid this being
      // such a busy spin loop...
      //   If we are using the 78MHz FPGA, then one clock cycle is ~12.82 ns ?
      // DEBUG(printf("Waiting for ready task in the queue...\n"));
      usleep(sptr->inparms->scheduler_holdoff_usec); // This defaults to 1 usec (about 78 FPGA clock cycles)
    }
  } // while (1)
  // pthread_mutex_unlock(&schedule_from_queue_mutex);
  return NULL;
}

void request_execution(
    /*task_metadata_block_t*/ void *task_metadata_block_ptr) {
  task_metadata_block_t *task_metadata_block =
      (task_metadata_block_t *)task_metadata_block_ptr;
  DEBUG(printf("APP: in request_execution for MB%u\n",
               task_metadata_block->block_id));
  // TASKID: task_metadata_block->task_id = global_task_id_counter++; // Set a
  // task id for this task (which is the global request_execution order, for
  // now)
  scheduler_datastate_block_t *sptr =
      task_metadata_block->scheduler_datastate_pointer;
  task_metadata_block->status = TASK_QUEUED; // queued
  gettimeofday(&task_metadata_block->sched_timings.queued_start, NULL);
  task_metadata_block->sched_timings.get_sec +=
      task_metadata_block->sched_timings.queued_start.tv_sec -
      task_metadata_block->sched_timings.get_start.tv_sec;
  task_metadata_block->sched_timings.get_usec +=
      task_metadata_block->sched_timings.queued_start.tv_usec -
      task_metadata_block->sched_timings.get_start.tv_usec;

  // Put this into the ready-task-queue
  //   Get a ready_task_queue_entry
  pthread_mutex_lock(&(sptr->task_queue_mutex));
  DEBUG(printf(
      "APP: there are currently %u free task queue entries in the list\n",
      sptr->num_free_task_queue_entries));
  SDEBUG(print_free_ready_tasks_list(sptr));
  ready_mb_task_queue_entry_t *my_queue_entry =
      sptr->free_ready_mb_task_queue_entries;
  sptr->free_ready_mb_task_queue_entries =
      sptr->free_ready_mb_task_queue_entries->next;
  sptr->free_ready_mb_task_queue_entries->prev =
      NULL; // disconnect the prev pointer
  sptr->num_free_task_queue_entries--;
  DEBUG(
      printf("APP: and now there are %u free task queue entries in the list\n",
             sptr->num_free_task_queue_entries));
  SDEBUG(print_free_ready_tasks_list(sptr));
  //   Now fill it in
  my_queue_entry->block_id = task_metadata_block->block_id;
  DEBUG(printf("APP: got a free_task_ready_queue_entry, leaving %u free\n",
               sptr->num_free_task_queue_entries));
  //   And add to the tail of the task queue
  if (sptr->ready_mb_task_queue_head == NULL) {
    my_queue_entry->prev = NULL;
    my_queue_entry->next = NULL;
    sptr->ready_mb_task_queue_head = my_queue_entry;
    DEBUG(printf("APP: inserted this as the HEAD of the ready-task-queue\n"));
  } else {
    my_queue_entry->prev = sptr->ready_mb_task_queue_tail;
    my_queue_entry->next = NULL;
    sptr->ready_mb_task_queue_tail->next = my_queue_entry;
    DEBUG(printf("APP: inserted this as the TAIL of the ready-task-queue\n"));
  }
  sptr->ready_mb_task_queue_tail = my_queue_entry;
  sptr->num_tasks_in_ready_queue++;
  DEBUG(printf("APP: and now there are %u ready tasks in the queue\n",
               sptr->num_tasks_in_ready_queue);
        print_ready_tasks_queue(sptr));
  pthread_mutex_unlock(&(sptr->task_queue_mutex));

  return;
}

/********************************************************************************
 * Here are the wait routines -- for critical tasks or all tasks to finish
 ********************************************************************************/
void wait_all_critical(scheduler_datastate_block_t *sptr) {
  struct timeval stop_wait_all_crit, start_wait_all_crit;
  uint64_t wait_all_crit_sec = 0LL;
  uint64_t wait_all_crit_usec = 0LL;

  gettimeofday(&start_wait_all_crit, NULL);
  // Loop through the critical tasks list and check whether they are all in
  // status "done"
  blockid_linked_list_t *cli = sptr->critical_live_task_head;
  while (cli != NULL) {
    if (sptr->master_metadata_pool[cli->clt_block_id].status != TASK_DONE) {
      // This task is not finished yet.. wait for it
      //  So start polling from the start of the list again.
      cli = sptr->critical_live_task_head;
    } else {
      cli = cli->next;
    }
  }
  gettimeofday(&stop_wait_all_crit, NULL);
  wait_all_crit_sec += stop_wait_all_crit.tv_sec - start_wait_all_crit.tv_sec;
  wait_all_crit_usec +=
      stop_wait_all_crit.tv_usec - start_wait_all_crit.tv_usec;
  if (sptr->visualizer_output_started &&
      ((sptr->inparms->visualizer_task_stop_count < 0) ||
       (global_finished_task_id_counter <
        sptr->inparms->visualizer_task_stop_count))) {
    int64_t wait_start = 1000000 * start_wait_all_crit.tv_sec +
                         start_wait_all_crit.tv_usec -
                         sptr->visualizer_start_time_usec;
    int64_t wait_stop = 1000000 * stop_wait_all_crit.tv_sec +
                        stop_wait_all_crit.tv_usec -
                        sptr->visualizer_start_time_usec;
    if (wait_start < 0) {
      wait_start = 0;
    }
    pthread_mutex_lock(&(sptr->sl_viz_out_mutex));
    fprintf(
        sptr->sl_viz_fp, "%lu,%d,%d,%s,%d,%d,%d,%s,%s,%lu,%lu,%lu\n",
        wait_start, //  sim_time,   ( pretend this was reported at start_time)
        (sptr->next_avail_DAG_id - 1), // task_dag_id
        0, // task_tid (This is a "fake" one, as there is no real single task
           // here)
        "Waiting", 0,
        0, // dag_dtime
        sptr->inparms->max_accel_types +
            2,       // accelerator_id  - use a number that cannot be a legal
                     // accel_id, isnt Rdy_Que
        "Wait_Crit", // accelerator_type ?,
        "nan",       // task_parent_ids
        wait_start, // task_arrival_time    (Make arrival and start the same, as
                    // we really only have start time?
        wait_start, // curr_job_start_time  (Make arrival and start the same, as
                    // we really only have start time?
        wait_stop); // curr_job_end_time
    pthread_mutex_unlock(&(sptr->sl_viz_out_mutex));
  }
}

void wait_on_tasklist(/* scheduler_datastate_block_t */ void *_sptr, int num_tasks, ...) {
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t*)_sptr;
  struct timeval stop_wait_tasklist, start_wait_tasklist;
  uint64_t wait_tasklist_sec = 0LL;
  uint64_t wait_tasklist_usec = 0LL;

  gettimeofday(&start_wait_tasklist, NULL);
  // Loop through the critical tasks list and check whether they are all in
  // status "done"
  int task_block_ids[num_tasks];
  va_list var_list;
  va_start(var_list, num_tasks);
  for (int i = 0; i < num_tasks; i++) {
    task_metadata_block_t *mb_ptr = va_arg(var_list, task_metadata_block_t *);
    if (mb_ptr == NULL) {
      printf("ERROR: wait_on_tasklist provided a NULL task pointer\n");
      exit( -30);
    }
    task_block_ids[i] = mb_ptr->block_id;
  }
  va_end(var_list);
  int idx = 0;
  while (idx < num_tasks) {
    int bid = task_block_ids[idx];
    task_status_t status = sptr->master_metadata_pool[bid].status;

    if ((status == TASK_DONE) || (status == TASK_FREE)) {
      // This task is finished move on to the next task
      idx++;
    } else {
      // I think perhaps we should add a short delay here to avoid this being
      // such a busy spin loop...
      //   If we are using the 78MHz FPGA, then one clock cycle is ~12.82 ns ?
      usleep(sptr->inparms
                 ->scheduler_holdoff_usec); // This defaults to 1 usec (about 78
                                            // FPGA clock cycles)
    }
  }
  gettimeofday(&stop_wait_tasklist, NULL);
  wait_tasklist_sec += stop_wait_tasklist.tv_sec - start_wait_tasklist.tv_sec;
  wait_tasklist_usec +=
      stop_wait_tasklist.tv_usec - start_wait_tasklist.tv_usec;
  if (sptr->visualizer_output_started &&
      ((sptr->inparms->visualizer_task_stop_count < 0) ||
       (global_finished_task_id_counter <
        sptr->inparms->visualizer_task_stop_count))) {
    int64_t wait_start = 1000000 * start_wait_tasklist.tv_sec +
                         start_wait_tasklist.tv_usec -
                         sptr->visualizer_start_time_usec;
    int64_t wait_stop = 1000000 * stop_wait_tasklist.tv_sec +
                        stop_wait_tasklist.tv_usec -
                        sptr->visualizer_start_time_usec;
    if (wait_start < 0) {
      wait_start = 0;
    }
    pthread_mutex_lock(&(sptr->sl_viz_out_mutex));
    fprintf(
        sptr->sl_viz_fp, "%lu,%d,%d,%s,%d,%d,%d,%s,%s,%lu,%lu,%lu\n",
        wait_start, //  sim_time,   ( pretend this was reported at start_time)
        (sptr->next_avail_DAG_id - 1), // task_dag_id
        0, // task_tid (This is a "fake" one, as there is no real single task
           // here)
        "Waiting", 0,
        0, // dag_dtime
        sptr->inparms->max_accel_types +
            2,       // accelerator_id  - use a number that cannot be a legal
                     // accel_id, isnt Rdy_Que
        "Wait_Crit", // accelerator_type ?,
        "nan",       // task_parent_ids
        wait_start, // task_arrival_time    (Make arrival and start the same, as
                    // we really only have start time?
        wait_start, // curr_job_start_time  (Make arrival and start the same, as
                    // we really only have start time?
        wait_stop); // curr_job_end_time
    pthread_mutex_unlock(&(sptr->sl_viz_out_mutex));
  }
}

void wait_all_tasks_finish(scheduler_datastate_block_t *sptr) {
  // Spin loop : check whether all blocks are free...
  printf("Waiting for ALL tasks to finish: free = %u and total = %u\n",
         sptr->free_metadata_blocks, sptr->total_metadata_pool_blocks);
  while (sptr->free_metadata_blocks != sptr->total_metadata_pool_blocks) {
    ; // Nothing really to do, but wait.
  }
}

// This cleans up the state (pthreads, etc.) before exit
void cleanup_state(scheduler_datastate_block_t *sptr) {
  DEBUG(printf("In the cleanup-state routine...\n"); fflush(stdout));

  // Dynamically unload the scheduling policy (plug-in)
  dlclose(sptr->policy_handle);

  if (sptr->sl_viz_fp != NULL) {
    fclose(sptr->sl_viz_fp);
  }

  // Cancel all the created pthreads...
  //printf("Cancelling the metadata block threads...\n"); fflush(stdout);
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    //printf("  cancelling MB%u pthread\n", i); fflush(stdout);
    pthread_cancel(sptr->metadata_threads[i]);
  }
  //printf("Cancelling the Schedule-From-Ready-Queue pthread\n"); fflush(stdout);
  pthread_cancel(sptr->scheduling_thread);
  sleep(1);

  // Clean out the pthread mutex and conditional variables
  pthread_mutex_destroy(&(sptr->free_metadata_mutex));
  pthread_mutex_destroy(&(sptr->accel_alloc_mutex));
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    pthread_mutex_destroy(&(sptr->master_metadata_pool[i].metadata_mutex));
    pthread_cond_destroy(&(sptr->master_metadata_pool[i].metadata_condv));
  }

  // Clean up any hardware accelerator stuff
  do_accelerator_type_closeout(sptr);

  //printf("Doing the free calls...\n"); fflush(stdout);
  free(sptr->free_metadata_pool);
  free(sptr->ready_mb_task_queue_pool);
  free(sptr->metadata_threads);
  free(sptr->critical_live_tasks_list);
  free(sptr->free_critlist_pool);
  free(sptr->master_metadata_pool);
  free(sptr->inparms);
  free(sptr);
}

// This is called at the end of run/life to shut down the scheduler
//  This will also output a bunch of stats abdout timings, etc.

void output_run_statistics(scheduler_datastate_block_t *sptr) {

  // NOW output some overall full-run statistics, etc.
  printf("\nOverall Accelerator allocation/usage statistics:\n");
  printf("\nTotal Scheduler Decision-Making Time was %lu usec for %lu "
         "decisions spanning %lu checks\n",
         sptr->scheduler_decision_time_usec, sptr->scheduler_decisions,
         sptr->scheduler_decision_checks);

  printf("\nScheduler block allocation/free statistics:\n");
  uint32_t total_alloc_MBs = 0;
  uint32_t total_freed_MBs = 0;
  for (int ti = 0; ti < sptr->inparms->max_task_types; ti++) {
    printf("  For %12s Scheduler allocated %9u blocks and freed %9u blocks\n",
           sptr->task_name_str[ti], sptr->allocated_metadata_blocks[ti],
           sptr->freed_metadata_blocks[ti]);
    total_alloc_MBs += sptr->allocated_metadata_blocks[ti];
    total_freed_MBs += sptr->freed_metadata_blocks[ti];
  }
  printf(" During FULL run,  Scheduler allocated %9u blocks and freed %9u "
         "blocks in total\n",
         total_alloc_MBs, total_freed_MBs);

  printf("\nPer-MetaData-Block Scheduler Allocation/Frees by Job-Type Data:\n");
  printf("%6s ", "Block");
  for (int ji = 1; ji < sptr->inparms->max_task_types; ji++) {
    printf("%12s_G %12s_F ", sptr->task_name_str[ji], sptr->task_name_str[ji]);
  }
  printf("\n");
  unsigned type_gets[sptr->inparms->max_task_types];
  unsigned type_frees[sptr->inparms->max_task_types];
  for (int ji = 0; ji < sptr->inparms->max_task_types; ji++) {
    type_gets[ji] = 0;
    type_frees[ji] = 0;
  }
  for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
    printf("%6u ", bi);
    for (int ji = 1; ji < sptr->inparms->max_task_types; ji++) {
      type_gets[ji] += sptr->master_metadata_pool[bi].gets_by_task_type[ji];
      type_frees[ji] += sptr->master_metadata_pool[bi].frees_by_task_type[ji];
      printf("%14u %14u ", sptr->master_metadata_pool[bi].gets_by_task_type[ji],
             sptr->master_metadata_pool[bi].frees_by_task_type[ji]);
    }
    printf("\n");
  }
  printf("%6s ", "Total");
  for (int ji = 1; ji < sptr->inparms->max_task_types; ji++) {
    printf("%14u %14u ", type_gets[ji], type_frees[ji]);
  }
  printf("\n");

  printf("\nPer-MetaData-Block Scheduler Timing Data:\n");
  {
    unsigned total_blocks_used = 0;
    uint64_t total_idle_usec = 0;
    uint64_t total_get_usec = 0;
    uint64_t total_queued_usec = 0;
    uint64_t total_running_usec[sptr->inparms->max_accel_types];
    for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
      total_running_usec[ti] = 0;
    }
    uint64_t total_done_usec = 0;
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      uint64_t this_idle_usec =
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.idle_sec) *
              1000000 +
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.idle_usec);
      uint64_t this_get_usec =
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.get_sec) *
              1000000 +
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.get_usec);
      uint64_t this_queued_usec =
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.queued_sec) *
              1000000 +
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.queued_usec);
      uint64_t this_total_run_usec = 0;
      uint64_t this_running_usec[sptr->inparms->max_accel_types];
      for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
        this_running_usec[ti] =
            (uint64_t)(
                sptr->master_metadata_pool[bi].sched_timings.running_sec[ti]) *
                1000000 +
            (uint64_t)(
                sptr->master_metadata_pool[bi].sched_timings.running_usec[ti]);
        this_total_run_usec += this_running_usec[ti];
      }
      uint64_t this_done_usec =
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.done_sec) *
              1000000 +
          (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.done_usec);
      printf(" Block %3u : IDLE %15lu GET %15lu QUE %15lu RUN %15lu DONE %15lu "
             "usec :",
             bi, this_idle_usec, this_get_usec, this_queued_usec,
             this_total_run_usec, total_done_usec);
      for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
        printf(" %15lu", this_running_usec[ti]);
      }
      printf("\n");
      if (this_idle_usec != 0) {
        total_blocks_used++;
      }
      total_idle_usec += this_idle_usec;
      total_get_usec += this_get_usec;
      total_queued_usec += this_queued_usec;
      for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
        total_running_usec[ti] += this_running_usec[ti];
      }
      total_done_usec += this_done_usec;
    }
    double avg;
    printf("\nScheduler Timings: Aggregate Across all Metadata Blocks with %u "
           "used blocks\n",
           total_blocks_used);
    avg = (double)total_idle_usec / (double)total_blocks_used;
    printf("  Metablocks_IDLE total run time:                %15lu usec : "
           "%16.2lf (average)\n",
           total_idle_usec, avg);
    avg = (double)total_get_usec / (double)total_blocks_used;
    printf("  Metablocks_GET total run time:                 %15lu usec : "
           "%16.2lf (average)\n",
           total_get_usec, avg);
    avg = (double)total_queued_usec / (double)total_blocks_used;
    printf("  Metablocks_QUEUED total run time:              %15lu usec : "
           "%16.2lf (average)\n",
           total_queued_usec, avg);
    for (int ti = 0; ti < sptr->inparms->max_accel_types; ti++) {
      avg = (double)total_running_usec[ti] / (double)total_blocks_used;
      printf("  Metablocks_RUNNING %u %15s run time: %15lu usec : %16.2lf "
             "(average)\n",
             ti, sptr->accel_name_str[ti], total_running_usec[ti], avg);
    }
    avg = (double)total_done_usec / (double)total_blocks_used;
    printf("  Metablocks_DONE total run time:                %15lu usec : "
           "%16.2lf (average)\n",
           total_done_usec, avg);
  }

  output_task_and_accel_run_stats(sptr);

  /*printf("\nACU_HIST: Aggregated In-Use Accelerator Time Histogram...\n");
  {
    printf("ACU_HIST:  CPU  FFT  VIT  CNN : TACC TFFT TVIT TCNN :
  Time-in-usec\n"); for (int i0 = 0; i0 <= sptr->num_accelerators_of_type[0];
  i0++) { for (int i1 = 0; i1 <= sptr->num_accelerators_of_type[1]; i1++) { for
  (int i2 = 0; i2 <= sptr->num_accelerators_of_type[2]; i2++) { for (int i3 = 0;
  i3 <= sptr->num_accelerators_of_type[3]; i3++) { printf("ACU_HIST: %4u %4u %4u
  %4u : %4u : %lu\n", i0, i1, i2, i3, (i1+i2+i3),
  sptr->in_use_accel_times_array[i0][i1][i2][i3]);
          }
        }
      }
    }
    }*/

  printf("\nAccelerator Usage Statistics:\n");
  {
    unsigned totals[sptr->inparms->max_accel_types - 1]
                   [sptr->max_accel_of_any_type];
    unsigned top_totals[sptr->inparms->max_accel_types - 1];
    for (int ti = 0; ti < sptr->inparms->max_accel_types - 1; ti++) {
      top_totals[ti] = 0;
      for (int ai = 0; ai < sptr->max_accel_of_any_type; ai++) {
        totals[ti][ai] = 0;
        for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
          totals[ti][ai] += sptr->master_metadata_pool[bi]
                                .accelerator_allocated_to_MB[ti][ai];
        }
      }
    }
    printf("\nPer-Accelerator allocation/usage statistics:\n");
    for (int ti = 0; ti < sptr->inparms->max_accel_types - 1; ti++) {
      for (int ai = 0; ai < sptr->max_accel_of_any_type; ai++) {
        if (ai < sptr->num_accelerators_of_type[ti]) {
          printf(" Acc_Type %u %s : Accel %2u Allocated %6u times\n", ti,
                 sptr->accel_name_str[ti], ai, totals[ti][ai]);
        } else {
          if (totals[ti][ai] != 0) {
            printf("ERROR : We have use of non-existent Accelerator %u %s : "
                   "index %u = %u\n",
                   ti, sptr->accel_name_str[ti], ai, totals[ti][ai]);
          }
        }
        top_totals[ti] += totals[ti][ai];
      }
    }
    printf("\nPer-Accelerator-Type allocation/usage statistics:\n");
    for (int ti = 0; ti < sptr->inparms->max_accel_types - 1; ti++) {
      printf(" Acc_Type %u %s Allocated %6u times\n", ti,
             sptr->accel_name_str[ti], top_totals[ti]);
    }
    printf("\nPer-Meta-Block Accelerator allocation/usage statistics:\n");
    for (int ti = 0; ti < sptr->inparms->max_accel_types - 1; ti++) {
      for (int ai = 0; ai < sptr->num_accelerators_of_type[ti]; ai++) {
        for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
          if (sptr->master_metadata_pool[bi]
                  .accelerator_allocated_to_MB[ti][ai] != 0) {
            printf(" Per-MB Acc_Type %u %s : Accel %2u Allocated %6u times for "
                   "MB%u\n",
                   ti, sptr->accel_name_str[ti], ai,
                   sptr->master_metadata_pool[bi]
                       .accelerator_allocated_to_MB[ti][ai],
                   bi);
          }
        }
      }
    }
  }
}

void shutdown_scheduler(scheduler_datastate_block_t *sptr) {
  output_run_statistics(sptr);
  // DON'T call this here -- there is an "on_exit" call that does this automatically now!
  //printf("Calling cleanup_stats...\n"); fflush(stdout);
  // cleanup_state(sptr);
}

void cleanup_and_exit(int rval, void *sptr_ptr) {
  if (sptr_ptr != NULL) {
    scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t *) sptr_ptr;
    cleanup_state(sptr);
  }
  exit(rval);
}

void dump_all_metadata_blocks_states(scheduler_datastate_block_t *sptr) {
  if (sptr->free_metadata_blocks == 0) {
    printf("FREE_MBS: { }\n");
  } else {
    for (int i = 0; i < sptr->free_metadata_blocks; i++) {
      if (i > 0) {
        printf(",");
      };
      printf("%u", sptr->free_metadata_pool[i]);
    }
    printf("\n");
  }
  // unsigned allocated_metadata_blocks[sptr->inparms->max_task_types];
  printf("Total Allocated MBs:  ");
  for (int i = 0; i < sptr->inparms->max_task_types; i++) {
    printf("( %s, %u ) ", sptr->task_name_str[i],
           sptr->allocated_metadata_blocks[i]);
  }
  printf("\n");
  // unsigned freed_metadata_blocks[sptr->inparms->max_task_types];
  printf("Total Freed MBs:  ");
  for (int i = 0; i < sptr->inparms->max_task_types; i++) {
    printf("( %s, %u ) ", sptr->task_name_str[i],
           sptr->freed_metadata_blocks[i]);
  }
  printf("\n");
  printf("\nData for EACH MB:\n");
  for (int mbi = 0; mbi < sptr->total_metadata_pool_blocks; mbi++) {
    printf("MB%u : Status %u %s\n", mbi, sptr->master_metadata_pool[mbi].status,
           sptr->task_status_str[sptr->master_metadata_pool[mbi].status]);
    printf(
        "  MB%u : Acc_ty %u   Acc_id %d   Job %u   Crit_Lvl %u\n", mbi,
        sptr->master_metadata_pool[mbi].accelerator_type,
        sptr->master_metadata_pool[mbi].accelerator_id,
        sptr->master_metadata_pool[mbi]
            .task_type, // task_name_str[sptr->master_metadata_pool[mbi].task_type],
        sptr->master_metadata_pool[mbi].crit_level);
    printf("  MB%u GETS:  ", mbi);
    for (int i = 0; i < sptr->inparms->max_task_types; i++) {
      printf("( %s, %u ) ", sptr->task_name_str[i],
             sptr->master_metadata_pool[mbi].gets_by_task_type[i]);
    }
    printf("\n");
    printf("  MB%u FREES:  ", mbi);
    for (int i = 0; i < sptr->inparms->max_task_types; i++) {
      printf("( %s, %u ) ", sptr->task_name_str[i],
             sptr->master_metadata_pool[mbi].frees_by_task_type[i]);
    }
    printf("\n");
  } // for (mbi loop over Metablocks)
}

task_type_t register_task_type(
    /*scheduler_datastate_block_t*/ void *sptr_ptr, char *task_name,
    char *task_description, set_up_task_function_t set_up_task_func,
    finish_task_execution_function_t finish_task_func,
    auto_finish_task_function_t auto_finish_func,
    print_metadata_block_contents_t
        print_metadata_block_contents,                       // function pointer
    output_task_type_run_stats_t output_task_type_run_stats, // function pointer
    int num_accel_task_exec_descriptions, ...) {
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t *)sptr_ptr;
  DEBUG(printf("In register_task_type with inputs:\n");
        printf("  name  = %s\n", task_name);
        printf("  description  = %s\n", task_description);
        printf("  print_metadata_block_contents = %p\n",
               print_metadata_block_contents);
        printf("  output_task_type_run_stats_t  = %p\n",
               output_task_type_run_stats);
        printf(" and num_accel_task_exeC_descr  = %u\n",
               num_accel_task_exec_descriptions));

  printf("Registering Task %s : %s\n", task_name, task_description);

  if (set_up_task_func == NULL) {
    printf(
        "ERROR: Must set set_up_task_func function -- can use base routine\n");
    exit( -31);
  }
  if (finish_task_func == NULL) {
    printf(
        "ERROR: Must set finish_task_func function -- can use base routine\n");
    exit( -31);
  }
  if (auto_finish_func == NULL) {
    printf(
        "ERROR: Must set auto_finish_func function -- can use base routine\n");
    exit( -31);
  }
  if (print_metadata_block_contents == NULL) {
    printf("ERROR: Must set print_metadata_block_contents function -- can use "
           "base routine\n");
    exit( -31);
  }
  // Okay, so here is where we "fill in" the scheduler's task-type information
  // for this task
  task_type_t tid = sptr->next_avail_task_type;
  if (tid < sptr->inparms->max_task_types) {
    sptr->next_avail_task_type++;
  } else {
    printf("ERROR: Ran out of Task IDs: MAX_TASK_TYPE = %u and we are adding "
           "id %u\n",
           (sptr->inparms->max_task_types - 1), tid);
    exit( -35);
  }
  snprintf(sptr->task_name_str[tid], MAX_TASK_NAME_LEN, "%s", task_name);
  snprintf(sptr->task_desc_str[tid], MAX_TASK_DESC_LEN, "%s", task_description);
  sptr->output_task_run_stats_function[tid] = output_task_type_run_stats;
  sptr->set_up_task_function[tid] = set_up_task_func;
  sptr->finish_task_execution_function[tid] = finish_task_func;
  sptr->auto_finish_task_function[tid] = auto_finish_func;

  sptr->print_metablock_contents_function[tid] = print_metadata_block_contents;

  printf("Starting the variable arguments processing for %d tuples\n",
         num_accel_task_exec_descriptions);
  va_list var_list;
  va_start(var_list, /*2 **/ num_accel_task_exec_descriptions);
  for (int i = 0; i < num_accel_task_exec_descriptions; i++) {
    scheduler_accelerator_type sched_accel =
        va_arg(var_list, scheduler_accelerator_type);
    sched_execute_task_function_t exec_fptr =
        va_arg(var_list, sched_execute_task_function_t);
    printf(" Call %d to Register Accel %u for task %u with fptr %p\n", i,
           sched_accel, tid, exec_fptr);
    register_accel_can_exec_task(sptr, sched_accel, tid, exec_fptr);
  }
  va_end(var_list);
  return tid;
}

void register_accel_can_exec_task(scheduler_datastate_block_t *sptr,
                                  scheduler_accelerator_type sl_acid,
                                  task_type_t tid,
                                  sched_execute_task_function_t fptr) {
  if (sl_acid > SCHED_MAX_ACCEL_TYPES) {
    printf("In register_accel_can_exec_task specified an illegal accelerator "
           "id: %u vs %u (MAX)\n",
           sl_acid, (SCHED_MAX_ACCEL_TYPES - 1));
    exit( -36);
  }
  DEBUG(printf("In register_accel_can_exec_task for accel %u %s and task %u "
               "with fptr %p\n",
               sl_acid, sptr->accel_name_str[sl_acid], tid, fptr));
  int acid = sptr->map_sched_accel_type_to_local_accel_type[sl_acid];
  if (acid < 0) {
    printf("In register_accel_can_exec_task specified an un-allocated "
           "accelerator id: %u %s\n",
           sl_acid, sptr->accel_name_str[sl_acid]);
    exit( -38);
  }
  if (tid >= sptr->next_avail_task_type) {
    printf("In register_task_can_exec_task specified an illegal taskerator id: "
           "%u vs %u currently defined\n",
           tid, sptr->next_avail_task_type);
    exit( -37);
  }
  if (sptr->scheduler_execute_task_function[acid][tid] != NULL) {
    printf("In register_accel_can_exec_task for accel_type %u and task_type %u "
           "- Already have a registered execution (%p)\n",
           acid, tid, sptr->scheduler_execute_task_function[tid][acid]);
    exit( -39);
  }
  sptr->scheduler_execute_task_function[acid][tid] = fptr;
  DEBUG(printf(
      "  Set scheduler_execute_task_function[acid = %u ][tid = %u ]  to %p\n",
      acid, tid, fptr));
  printf("Set scheduler_execute_task_function for Task %s on Accelerator Type "
         "%s\n",
         sptr->task_name_str[tid], sptr->accel_name_str[acid]);
}

/*task_metadata_block_t*/ void *
set_up_task(/*scheduler_datastate_block_t*/ void *sptr_ptr,
            task_type_t the_task_type, task_criticality_t crit_level,
            int use_auto_finish, int32_t dag_id, ...) {
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t *)sptr_ptr;
  va_list args;
  va_start(args, dag_id);

  task_metadata_block_t *task_mb = sptr->set_up_task_function[the_task_type](
      sptr, the_task_type, crit_level, use_auto_finish, dag_id, &args);

  va_end(args);
  return task_mb;
}

void finish_task_execution(
    /*task_metadata_block_t*/ void *the_metadata_block_ptr, ...) {
  task_metadata_block_t *the_metadata_block =
      (task_metadata_block_t *)the_metadata_block_ptr;
  scheduler_datastate_block_t *sptr =
      the_metadata_block->scheduler_datastate_pointer;
  task_type_t task_type = the_metadata_block->task_type;

  va_list args;
  va_start(args, the_metadata_block_ptr);

  sptr->finish_task_execution_function[task_type](the_metadata_block, &args);

  va_end(args);
}

auto_finish_task_function_t
get_auto_finish_routine(scheduler_datastate_block_t *sptr,
                        task_type_t the_task_type) {
  return sptr->auto_finish_task_function[the_task_type];
}
