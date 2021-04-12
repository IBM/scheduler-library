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
#include <limits.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <dlfcn.h>

#include "utils.h"
//#define VERBOSE
#include "verbose.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "scheduler.h"

scheduler_get_datastate_in_parms_t sched_state_def_parms = {
  .max_task_types   = MAX_TASK_TYPES,
  .max_accel_types  = MAX_ACCEL_TYPES,
  .max_accel_of_any_type = MAX_ACCEL_OF_EACH_TYPE,

  .max_metadata_pool_blocks = GLOBAL_METADATA_POOL_BLOCKS,
  .max_data_space_bytes = MAX_DATA_SPACE_BYTES,

  .max_task_timing_sets = MAX_TASK_TIMING_SETS,
};


// This now will hold all the state for an instance of the scheduler.
//  This can be shared with the policies, etc. using a single pointer parameter
//  This also could allow multiple scheduler states to be in effect simultaneously...?
//scheduler_datastate_block_t sched_state; 


// Forward declarations
void release_accelerator_for_task(task_metadata_block_t* task_metadata_block);

void* schedule_executions_from_queue(void* void_parm_ptr);


void print_ready_tasks_queue(scheduler_datastate_block_t* sptr) {
  int ti = 0;
  ready_mb_task_queue_entry_t* t_te = sptr->ready_mb_task_queue_head;
  while(t_te != NULL) {
    printf(" RTQ: Ready_Task_Entry %2u of %2u : ID %2u MB%d P: %p T: %p N: %p\n", ti, sptr->num_tasks_in_ready_queue, t_te->unique_id, t_te->block_id, t_te->prev, t_te, t_te->next);
    ti++;
    t_te = t_te->next;
  }
}

void print_free_ready_tasks_list(scheduler_datastate_block_t* sptr) {
  int i = 0;
  ready_mb_task_queue_entry_t* t_te = sptr->free_ready_mb_task_queue_entries;
  while(t_te != NULL) {
    printf(" FRTL: Entry %2u of %2u : ID %2u MB%d P: %p T: %p N: %p\n", i, sptr->num_free_task_queue_entries, t_te->unique_id, t_te->block_id, t_te->prev, t_te, t_te->next);
    i++;
    t_te = t_te->next;
  }
}


/*void init_accelerators_in_use_interval(scheduler_datastate_block_t* sptr, struct timeval start_time) {
  sptr->last_accel_use_update_time = start_time;
  }*/


void account_accelerators_in_use_interval(scheduler_datastate_block_t* sptr)
{
  // Count which accelerators are in use
  int acc_in_use[sptr->limits.max_accel_types-1];
  for (int i = 0; i < sptr->limits.max_accel_types-1; i++) {
    acc_in_use[i] = 0;
    for (int j = 0; j < sptr->num_accelerators_of_type[i]; j++) {
      if (sptr->accelerator_in_use_by[i][j] != -1) {
        acc_in_use[i]++;
      }
    }
  }
  //Now we have the array of curr-in-use
  struct timeval now;
  gettimeofday(&now, NULL);
  /* // This is a 4-Dim by ACCEL type : [CPU][FFT][VIT][CV] */
  /* sptr->in_use_accel_times_array[acc_in_use[0]][acc_in_use[1]][acc_in_use[2]][acc_in_use[3]] += 1000000*(now.tv_sec - sptr->last_accel_use_update_time.tv_sec) + (now.tv_usec - sptr->last_accel_use_update_time.tv_usec);
  sptr->last_accel_use_update_time = now;*/
}

void print_base_metadata_block_contents(task_metadata_block_t* mb)
{
  printf("block_id = %d @ %p\n", mb->block_id, mb);
  scheduler_datastate_block_t* sptr = mb->scheduler_datastate_pointer;
  unsigned status = mb->status;
  if (status < NUM_TASK_STATUS) {
    printf(" ** status = %s\n", sptr->task_status_str[status]);
  } else {
    printf(" ** status = %d <= NOT a legal value!\n",  mb->status);
  }
  unsigned task_type = mb->task_type;
  if (task_type < sptr->limits.max_task_types) {
    printf("    task_type = %s\n", sptr->task_name_str[task_type]);
  } else {
    printf(" ** task_type = %d <= NOT a legal value!\n", mb->task_type);
  }
  unsigned crit_level = mb->crit_level;
  if (crit_level < NUM_TASK_CRIT_LEVELS) {
    printf("    crit_level = %s\n",  sptr->task_criticality_str[crit_level]);
  } else {
    printf(" ** crit_level = %d <= NOT a legal value!\n",  mb->crit_level);
  }
  printf("    data_size  = %d\n",  mb->data_size);
  printf("    data_space @ %p\n", &(mb->data_space));
}



void print_critical_task_list_ids(scheduler_datastate_block_t* sptr) {
  blockid_linked_list_t* cli = sptr->critical_live_task_head;
  if (cli == NULL) {
    printf("Critical task list is EMPTY\n");
  } else {
    printf("Critical task list : ");
    while (cli != NULL) {
      printf(" %u", cli->clt_block_id); //, free_critlist_pool[cli->clt_block_id]);
      cli = cli->next;
    }
    printf("\n");
  }
}



void
do_accelerator_type_closeout(scheduler_datastate_block_t* sptr)
{
  // Clean up any hardware accelerator stuff
  DEBUG(printf("Doing accelerator type closeout for %u accelerators\n", sptr->next_avail_accel_id));
  for (int ai = 0; ai < sptr->next_avail_accel_id; ai++) {
    if (sptr->do_accel_closeout_function[ai] != NULL) {
      sptr->do_accel_closeout_function[ai](NULL);
    } else {
      printf("Note: do_accel_closeout_function for accel %u = %s is NULL\n", ai, sptr->accel_name_str[ai]);
    }
  }
}


void
output_task_and_accel_run_stats(scheduler_datastate_block_t* sptr)
{
  printf("\nPer-MetaData-Block Job Timing Data:\n");
  for (int ti = 0; ti < sptr->next_avail_task_id; ti++) {
    if (sptr->output_task_run_stats_function[ti] != NULL) {
      sptr->output_task_run_stats_function[ti](sptr, ti, sptr->next_avail_accel_id);
    }
  }

  for (int ai = 0; ai < sptr->next_avail_accel_id; ai++) {
    if (sptr->output_accel_run_stats_function[ai] != NULL) {
      sptr->output_accel_run_stats_function[ai](sptr, ai, sptr->next_avail_task_id);
    }
  }
}


// There is an open question as to whether we should "Wait" for an available Metadata Block
//  or return if there are none available to the program (and let IT decide what to do next)
//  Currently, we have to return, or else the scheduler task cannot make progress and free
//  additional metablocks.... so we require the caller to do the "while" loop...

task_metadata_block_t* get_task_metadata_block(scheduler_datastate_block_t* sptr, task_id_t in_task_type, task_criticality_t crit_level, uint64_t * task_profile)
{
  pthread_mutex_lock(&(sptr->free_metadata_mutex));
  TDEBUG(printf("in get_task_metadata_block with %u free_metadata_blocks\n", sptr->free_metadata_blocks));
  if (sptr->free_metadata_blocks < 1) {
    // Out of metadata blocks -- all in use, cannot enqueue new tasks!
    printf("No free metadata blocks: %d\n", sptr->free_metadata_blocks);
    return NULL;
  }
  int bi = sptr->free_metadata_pool[sptr->free_metadata_blocks - 1];
  TDEBUG(printf(" BEFORE_GET : MB%d : free_metadata_pool : ", bi);
	 for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
	   printf("%d ", sptr->free_metadata_pool[i]);
	 }
	 printf("\n"));
  if ((bi < 0) || (bi > sptr->total_metadata_pool_blocks)) {
    printf("ERROR : free_metadata_pool[%u -1] = %d   with %d free_metadata_blocks\n", sptr->free_metadata_blocks, bi, sptr->free_metadata_blocks);
    for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
      printf("  free_metadata_pool[%2u] = %d\n", i, sptr->free_metadata_pool[i]);
    }
    cleanup_and_exit(sptr, -16);
  }
  sptr->free_metadata_pool[sptr->free_metadata_blocks - 1] = -1;
  sptr->free_metadata_blocks -= 1;
  // For neatness (not "security") we'll clear the meta-data in the block (not the data data,though)
  sptr->master_metadata_pool[bi].task_type = in_task_type;
  sptr->master_metadata_pool[bi].gets_by_task_type[in_task_type]++;
  sptr->master_metadata_pool[bi].status = TASK_ALLOCATED;
  sptr->master_metadata_pool[bi].crit_level = crit_level;
  for (int i = 0; i < sptr->limits.max_accel_types; ++i) {
    sptr->master_metadata_pool[bi].task_on_accel_profile[i] = task_profile[i];
  }
  sptr->master_metadata_pool[bi].data_size = 0;
  sptr->master_metadata_pool[bi].accelerator_type = NO_Accelerator;
  sptr->master_metadata_pool[bi].accelerator_id   = NO_Task;
  sptr->master_metadata_pool[bi].atFinish = NULL;  // NO finish call-back function

  gettimeofday(&sptr->master_metadata_pool[bi].sched_timings.get_start, NULL);
  sptr->master_metadata_pool[bi].sched_timings.idle_sec += sptr->master_metadata_pool[bi].sched_timings.get_start.tv_sec - sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_sec;
  sptr->master_metadata_pool[bi].sched_timings.idle_usec += sptr->master_metadata_pool[bi].sched_timings.get_start.tv_usec - sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_usec;

  if (crit_level > 1) { // is this a "critical task"
    /* int ci = total_critical_tasks; // Set index for crit_task block_id in pool */
    /* critical_live_tasks_list[ci].clt_block_id = bi;  // Set the critical task block_id indication */
    // Select the next available critical-live-task-list entry ID 
    int li = sptr->free_critlist_pool[sptr->free_critlist_entries - 1];
    sptr->free_critlist_pool[sptr->free_critlist_entries - 1] = -1; // clear it a(as it is no longer free)
    sptr->free_critlist_entries -= 1;
    // Now li indicates the critical_live_tasks_list[] index to use
    // Now set up the revisions to the critical live tasks list
    sptr->critical_live_tasks_list[li].clt_block_id = bi;   // point this entry to the sptr->master_metadata_pool block id
    sptr->critical_live_tasks_list[li].next = sptr->critical_live_task_head;     // Insert as head of critical tasks list
    sptr->critical_live_task_head = &(sptr->critical_live_tasks_list[li]);
    sptr->total_critical_tasks += 1;
  }
  DEBUG(printf("  returning block %u\n", bi);
	print_critical_task_list_ids(sptr));
  TDEBUG(printf(" AFTER_GET : MB%u : free_metadata_pool : ", bi);
	 for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
	   printf("%d ", sptr->free_metadata_pool[i]);
	 }
	 printf("\n"));
  sptr->allocated_metadata_blocks[in_task_type]++;
  //printf("MB%u got allocated : %u %u\n", bi, task_type, crit_level);
  pthread_mutex_unlock(&(sptr->free_metadata_mutex));
  
  return &(sptr->master_metadata_pool[bi]);
}





void free_task_metadata_block(task_metadata_block_t* mb)
{
  scheduler_datastate_block_t* sptr = mb->scheduler_datastate_pointer;
  pthread_mutex_lock(&(sptr->free_metadata_mutex));

  int bi = mb->block_id;
  //printf("MB%u getting freed : %u %u\n", bi, mb->task_type, mb->crit_level);
  TDEBUG(printf("in free_task_metadata_block for block %u with %u free_metadata_blocks\n", bi, sptr->free_metadata_blocks);//);
	 printf(" BEFORE_FREE : MB%u : free_metadata_pool : ", bi);
	 for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
	   printf("%d ", sptr->free_metadata_pool[i]);
	 }
	 printf("\n"));

  if (sptr->free_metadata_blocks < sptr->total_metadata_pool_blocks) {
    sptr->master_metadata_pool[bi].frees_by_task_type[mb->task_type]++;
    sptr->free_metadata_pool[sptr->free_metadata_blocks] = bi;
    sptr->free_metadata_blocks += 1;
    if (sptr->master_metadata_pool[bi].crit_level > 1) { // is this a critical tasks?
      // Remove task form critical list, free critlist entry, etc.
      blockid_linked_list_t* lcli = NULL;
      blockid_linked_list_t* cli = sptr->critical_live_task_head;
      //while ((cli != NULL) && (critical_live_tasks_list[cli->clt_block_id].clt_block_id != bi)) {
      while ((cli != NULL) && (cli->clt_block_id != bi)) {
        lcli = cli;  // The "previous" block; NULL == "head"
        cli = cli->next;  
      }
      if (cli == NULL) {
        printf("ERROR: Critical task NOT on the critical_live_task_list :\n");
        print_base_metadata_block_contents(mb);
        cleanup_and_exit(sptr, -6);
      }
      // We've found the critical task in critical_live_tasks_list - cli points to it
      int cti = cli->clt_block_id;
      //printf(" freeing critlist_pool %u to %u\n", free_critlist_entries - 1, cti);
      sptr->free_critlist_pool[sptr->free_critlist_entries] = cti; // Enable this crit-list entry for new use
      sptr->free_critlist_entries += 1; // Update the count of available critlist entries in the pool
      cli->clt_block_id = -1; // clear the clt_block_id indicator (we're done with it)
      // And remove the cli entry from the critical_lvet_tasks linked list
      if (lcli == NULL) {
        sptr->critical_live_task_head = cli->next;
      } else {
        lcli->next = cli->next;
      }
      cli->next = NULL;
      sptr->total_critical_tasks -= 1;
    }
    sptr->master_metadata_pool[bi].atFinish = NULL; // Ensure this is now set to NULL (safety safety)
    // For neatness (not "security") we'll clear the meta-data in the block (not the data data, though)
    sptr->freed_metadata_blocks[sptr->master_metadata_pool[bi].task_type]++;
    sptr->master_metadata_pool[bi].task_type = NO_Task; // unset
    sptr->master_metadata_pool[bi].status = TASK_FREE;   // free
    gettimeofday(&sptr->master_metadata_pool[bi].sched_timings.idle_start, NULL);
    sptr->master_metadata_pool[bi].sched_timings.done_sec += sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_sec - sptr->master_metadata_pool[bi].sched_timings.done_start.tv_sec;
    sptr->master_metadata_pool[bi].sched_timings.done_usec += sptr->master_metadata_pool[bi].sched_timings.idle_start.tv_usec - sptr->master_metadata_pool[bi].sched_timings.done_start.tv_usec;
    sptr->master_metadata_pool[bi].crit_level = NO_TASK; // lowest/free?
    sptr->master_metadata_pool[bi].data_size = 0;
  } else {
    printf("ERROR : We are freeing a metadata block when we already have max metadata blocks free...\n");
    printf("   THE FREE Metadata Blocks list:\n");
    for (int ii = 0; ii < sptr->free_metadata_blocks; ii++) {
      printf("        free[%2u] = %u\n", ii, sptr->free_metadata_pool[ii]);
    }
    DEBUG(printf("    THE Being-Freed Meta-Data Block:\n");
	  print_base_metadata_block_contents(mb));
    cleanup_and_exit(sptr, -5);
  }
  TDEBUG(printf(" AFTER_FREE : MB%u : free_metadata_pool : ", bi);
	 for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
	   printf("%d ", sptr->free_metadata_pool[i]);
	 }
	 printf("\n"));
  pthread_mutex_unlock(&(sptr->free_metadata_mutex));
}




int
get_task_status(scheduler_datastate_block_t* sptr, int task_id) {
  return sptr->master_metadata_pool[task_id].status;
}


void mark_task_done(task_metadata_block_t* task_metadata_block)
{
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  //printf("MB%u in mark_task_done\n", task_metadata_block->block_id);
  // First release the accelerator
  release_accelerator_for_task(task_metadata_block);

  // Then, mark the task as "DONE" with execution
  task_metadata_block->status = TASK_DONE;
  gettimeofday(&task_metadata_block->sched_timings.done_start, NULL);
  int idx = task_metadata_block->accelerator_type;
  task_metadata_block->sched_timings.running_sec[idx] += task_metadata_block->sched_timings.done_start.tv_sec - task_metadata_block->sched_timings.running_start.tv_sec;
  task_metadata_block->sched_timings.running_usec[idx] += task_metadata_block->sched_timings.done_start.tv_usec - task_metadata_block->sched_timings.running_start.tv_usec;

  // And finally, call the call-back if there is one... (which might clear out the metadata_block entirely)
  if (task_metadata_block->atFinish != NULL) {
    // And finally, call the atFinish call-back routine specified in the MetaData Block
    task_metadata_block->atFinish(task_metadata_block);
  }
}


// NOTE: This is executed by a metadata_block pthread
void
execute_task_on_accelerator(task_metadata_block_t* task_metadata_block)
{
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  DEBUG(printf("In execute_task_on_accelerator for MB%d with Accel Type %s and Number %u\n", task_metadata_block->block_id, sptr->accel_name_str[task_metadata_block->accelerator_type], task_metadata_block->accelerator_id));
  if (task_metadata_block->accelerator_type != NO_Accelerator) {
    if ((task_metadata_block->task_type > 0) && (task_metadata_block->task_type < sptr->limits.max_task_types)) {
      DEBUG(printf("Executing Task for MB%d : Type %u on %u\n", task_metadata_block->block_id, task_metadata_block->task_type, task_metadata_block->accelerator_type));
      sptr->scheduler_execute_task_function[task_metadata_block->accelerator_type][task_metadata_block->task_type](task_metadata_block);
    } else {
      printf("ERROR : execute_task_on_accelerator called for unknown task type: %u\n", task_metadata_block->task_type);
      cleanup_and_exit(sptr, -13);
    }
  } else {
    printf("ERROR -- called execute_task_on_accelerator for NO_ACCELERATOR with block:\n");
    print_base_metadata_block_contents(task_metadata_block);
    cleanup_and_exit(sptr, -11);
  }
  TDEBUG(printf("DONE Executing Task for MB%d\n", task_metadata_block->block_id));
}


void*
metadata_thread_wait_for_task(void* void_parm_ptr)
{
  task_metadata_block_t* task_metadata_block = (task_metadata_block_t*)void_parm_ptr;
  int bi = task_metadata_block->block_id;
  DEBUG(printf("In metadata_thread_wait_for_task for thread for metadata block %d\n", bi));
  // I think we do this once, then can wait_cond many times
  pthread_mutex_lock(&(task_metadata_block->metadata_mutex));
  do {
    TDEBUG(printf("MB_THREAD %d calling pthread_cond_wait\n", bi));
    // This will cause the thread to wait for a triggering signal through metadata_condv[bi]
    pthread_cond_wait(&(task_metadata_block->metadata_condv), &(task_metadata_block->metadata_mutex));

    TDEBUG(printf("MB_THREAD %d calling execute_task_on_accelerator...\n", bi));
    // Now we have been "triggered" -- so we invoke the appropriate accelerator
    execute_task_on_accelerator(task_metadata_block);
  } while (1); // We will loop here forever, until the main program exits....

  // We should never fall out, but in case we do, clean up
  pthread_mutex_unlock(&(task_metadata_block->metadata_mutex));
}


/* Input parameters: */

void copy_scheduler_datastate_defaults_into_parms(scheduler_get_datastate_in_parms_t* pptr)
{
  pptr->max_task_types            = sched_state_def_parms.max_task_types;
  pptr->max_accel_types           = sched_state_def_parms.max_accel_types;
  pptr->max_accel_of_any_type     = sched_state_def_parms.max_accel_of_any_type;
  pptr->max_metadata_pool_blocks  = sched_state_def_parms.max_metadata_pool_blocks;
  pptr->max_task_timing_sets      = sched_state_def_parms.max_task_timing_sets;
  pptr->max_data_space_bytes      = sched_state_def_parms.max_data_space_bytes;
}

scheduler_datastate_block_t* get_new_scheduler_datastate_pointer(scheduler_get_datastate_in_parms_t* inp)
{
  scheduler_datastate_block_t* sptr = malloc(sizeof(scheduler_datastate_block_t));
  size_t sched_state_size = sizeof(scheduler_datastate_block_t);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for base scheduler_datastate_block_t sptr\n");
    exit(-99);
  }
  
  sptr->limits.max_task_types            = inp->max_task_types;
  sptr->limits.max_accel_types           = inp->max_accel_types;
  sptr->limits.max_accel_of_any_type     = inp->max_accel_of_any_type;
  sptr->limits.max_metadata_pool_blocks  = inp->max_metadata_pool_blocks;
  sptr->limits.max_task_timing_sets      = inp->max_task_timing_sets;
  sptr->limits.max_data_space_bytes      = inp->max_data_space_bytes;

  // Allocate the scheduler_datastate_block_t dynamic (per-task-type) elements
  sptr->allocated_metadata_blocks = malloc(inp->max_task_types * sizeof(uint32_t));
  sched_state_size += inp->max_task_types * sizeof(uint32_t);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->allocated_metadata_blocks\n");
    exit(-99);
  }
  sptr->freed_metadata_blocks = malloc(inp->max_task_types * sizeof(uint32_t));
  sched_state_size += inp->max_task_types * sizeof(uint32_t);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->freed_metadata_blocks\n");
    exit(-99);
  }

  sptr->task_name_str = calloc(inp->max_task_types, sizeof(char*)); //[MAX_TASK_TYPES][MAX_TASK_NAME_LEN];
  sched_state_size += inp->max_task_types * sizeof(char*);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->task_name_str\n");
    exit(-99);
  }
  char * tptr = malloc(inp->max_task_types * MAX_TASK_NAME_LEN);
  sched_state_size += inp->max_task_types * MAX_TASK_NAME_LEN;
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->task_name_str char-pool\n");
    exit(-99);
  }
  for (int ti = 0; ti < inp->max_task_types; ti++) {
    sptr->task_name_str[ti] = tptr;
    tptr += MAX_TASK_NAME_LEN;
  }
  sptr->task_desc_str = calloc(inp->max_task_types, sizeof(char*)); //[MAX_TASK_TYPES][MAX_TASK_DESC_LEN];
  sched_state_size += inp->max_task_types * sizeof(char*);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->task_desc_str\n");
    exit(-99);
  }
  tptr = malloc(inp->max_task_types * MAX_TASK_DESC_LEN);
  sched_state_size += inp->max_task_types * MAX_TASK_DESC_LEN;
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->task_desc_str char-pool\n");
    exit(-99);
  }
  for (int ti = 0; ti < inp->max_task_types; ti++) {
    sptr->task_desc_str[ti] = tptr;
    tptr += MAX_TASK_NAME_LEN;
  }

  for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
    sptr->scheduler_execute_task_function[ai] = calloc(inp->max_task_types, sizeof(sched_execute_task_function_t)); //[MAX_ACCEL_TYPES][MAX_TASK_TYPES];
    sched_state_size += inp->max_task_types * sizeof(sched_execute_task_function_t);
    if (sptr == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->scheduler_execute_task_function[%u]\n", ai);
      exit(-99);
    }
  }

  sptr->print_metablock_contents_function = malloc(inp->max_task_types * sizeof(print_metadata_block_contents_t));
  sched_state_size += inp->max_task_types * sizeof(print_metadata_block_contents_t);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->print_metablock_contents_function\n");
    exit(-99);
  }
  sptr->output_task_run_stats_function = malloc(inp->max_task_types * sizeof(output_task_type_run_stats_t));
  sched_state_size += inp->max_task_types * sizeof(output_task_type_run_stats_t);
  if (sptr == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->output_task_run_stats_function\n");
    exit(-99);
  }
  
  sptr->free_metadata_pool = calloc(inp->max_metadata_pool_blocks, sizeof(int));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(int);
  if (sptr->free_metadata_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->free_metadata_pool\n");
    exit(-99);
  }
  sptr->ready_mb_task_queue_pool = calloc(inp->max_metadata_pool_blocks, sizeof(ready_mb_task_queue_entry_t));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(ready_mb_task_queue_entry_t);
  if (sptr->ready_mb_task_queue_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->ready_mb_task_queue_pool\n");
    exit(-99);
  }
  sptr->metadata_threads = calloc(inp->max_metadata_pool_blocks, sizeof(pthread_t));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(pthread_t);
  if (sptr->metadata_threads == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->metadata_threads\n");
    exit(-99);
  }
  sptr->critical_live_tasks_list = calloc(inp->max_metadata_pool_blocks, sizeof(blockid_linked_list_t));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(blockid_linked_list_t);
  if (sptr->critical_live_tasks_list == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->critical_live_tasks_list\n");
    exit(-99);
  }
  sptr->free_critlist_pool = calloc(inp->max_metadata_pool_blocks, sizeof(int));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(int);
  if (sptr->free_critlist_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->free_critlist_pool\n");
    exit(-99);
  }

  // Now allocate the metadata block entries...
  sptr->master_metadata_pool = calloc(inp->max_metadata_pool_blocks, sizeof(task_metadata_block_t));
  sched_state_size += inp->max_metadata_pool_blocks * sizeof(task_metadata_block_t);
  if (sptr->master_metadata_pool == NULL) {
    printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->master_metadata_pool\n");
    exit(-99);
  }

  /*
    uint32_t gets_by_task_type[MAX_TASK_TYPES]; // Count of times this metadata block allocated per job type.
    uint32_t frees_by_task_type[MAX_TASK_TYPES]; // Count of times this metadata block allocated per job type.
    uint32_t            task_computed_on[MAX_ACCEL_TYPES][MAX_TASK_TYPES];
    task_timing_data_t  task_timings[MAX_TASK_TYPES];  // This allows for N types of tasks (e.g. FFT, Viterbi, etc.)
  */
  for (int mi = 0; mi < inp->max_metadata_pool_blocks; mi++) {
    sptr->master_metadata_pool[mi].gets_by_task_type = malloc(inp->max_task_types * sizeof(uint32_t));
    sched_state_size += inp->max_task_types * sizeof(uint32_t);
    if (sptr->master_metadata_pool[mi].gets_by_task_type == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->master_metadata_pool[%u].gets_by_task_type\n", mi);
      exit(-99);
    }
    
    sptr->master_metadata_pool[mi].frees_by_task_type = malloc(inp->max_task_types * sizeof(uint32_t));
    sched_state_size += inp->max_task_types * sizeof(uint32_t);
    if (sptr->master_metadata_pool[mi].frees_by_task_type == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->master_metadata_pool[%u].frees_by_task_type\n", mi);
      exit(-99);
    }
    
    for (int ai = 0; ai < MAX_ACCEL_TYPES; ai++) {
      sptr->master_metadata_pool[mi].task_computed_on[ai] = malloc(inp->max_task_types * sizeof(uint32_t));
      sched_state_size += inp->max_task_types * sizeof(uint32_t);
      if (sptr->master_metadata_pool[mi].task_computed_on[ai] == NULL) {
	printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->master_metadata_pool[%u].task_computed_on[%u]\n", mi, ai);
	exit(-99);
      }
    }

    sptr->master_metadata_pool[mi].task_timings = malloc(inp->max_task_types * sizeof(task_timing_data_t));
    sched_state_size += inp->max_task_types * sizeof(task_timing_data_t);
    if (sptr->master_metadata_pool[mi].task_timings == NULL) {
      printf("ERROR: get_new_scheduler_datastate_pointer cannot allocate memory for sptr->master_metadata_pool[%u].task_timings\n", mi);
      exit(-99);
    }
  }


  printf("TOTAL Schedule DataState Size is %lu bytes\n", sched_state_size);
  return sptr;
}


status_t initialize_scheduler(scheduler_datastate_block_t* sptr)
{
  DEBUG(printf("In initialize...\n"));
  // Currently we set this to a fixed a-priori number...
  sptr->total_metadata_pool_blocks = sptr->limits.max_metadata_pool_blocks;
  
  sptr->next_avail_task_id  = 0;
  sptr->next_avail_accel_id = 0;

  sptr->scheduler_holdoff_usec      = 1;
  sptr->free_metadata_blocks        = sptr->total_metadata_pool_blocks;
  sptr->num_free_task_queue_entries = 0;
  sptr->num_tasks_in_ready_queue    = 0;
  sptr->critical_live_task_head     = NULL;
  sptr->free_critlist_entries       = sptr->total_metadata_pool_blocks;
  sptr->total_critical_tasks        = 0;

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
  sptr->scheduler_decisions          = 0;
  sptr->scheduler_decision_checks    = 0;


  // Dynamically load the scheduling policy (plug-in) to use, and initialize it
  char policy_filename[300];
  snprintf(policy_filename, 270, "./obj_p/lib%s.so", sptr->policy);
  if ( (sptr->policy_handle = dlopen(policy_filename, RTLD_LAZY)) == NULL) {
    printf("Could not open plug-in scheduling policy: %s\n", dlerror());
    cleanup_and_exit(sptr, -1);
  }

  if (dlerror() != NULL) {
    dlclose(sptr->policy_handle);
    printf("Function initialize_policy() not found in scheduling policy %s\n", policy_filename);
    cleanup_and_exit(sptr, -1);
  }

  sptr->initialize_assign_task_to_pe = dlsym(sptr->policy_handle, "initialize_assign_task_to_pe");
  if (dlerror() != NULL) {
    dlclose(sptr->policy_handle);
    printf("Function initialize_assign_task_to_pe() not found in scheduling policy %s\n", policy_filename);
    cleanup_and_exit(sptr, -1);
  }

  sptr->assign_task_to_pe = dlsym(sptr->policy_handle, "assign_task_to_pe");
  if (dlerror() != NULL) {
    dlclose(sptr->policy_handle);
    printf("Function assign_task_to_pe() not found in scheduling policy %s\n", policy_filename);
    cleanup_and_exit(sptr, -1);
  }

  int parms_error = 0;
  if (MAX_ACCEL_OF_EACH_TYPE < NUM_CPU_ACCEL) {
    printf("INIT-SCHED: ERROR : MAX_ACCEL_OF_EACH_TYPE < NUM_CPU_ACCEL : %u < %u\n", MAX_ACCEL_OF_EACH_TYPE, NUM_CPU_ACCEL);
    parms_error = 1;
  }
  if (MAX_ACCEL_OF_EACH_TYPE < NUM_FFT_ACCEL) {
    printf("INIT-SCHED: ERROR : MAX_ACCEL_OF_EACH_TYPE < NUM_FFT_ACCEL : %u < %u\n", MAX_ACCEL_OF_EACH_TYPE, NUM_FFT_ACCEL);
    parms_error = 1;
  }
  if (MAX_ACCEL_OF_EACH_TYPE < NUM_VIT_ACCEL) {
    printf("INIT-SCHED: ERROR : MAX_ACCEL_OF_EACH_TYPE < NUM_VIT_ACCEL : %u < %u\n", MAX_ACCEL_OF_EACH_TYPE, NUM_VIT_ACCEL);
    parms_error = 1;
  }
  if (MAX_ACCEL_OF_EACH_TYPE < NUM_CV_ACCEL) {
    printf("INIT-SCHED: ERROR : MAX_ACCEL_OF_EACH_TYPE < NUM_CV_ACCEL : %u < %u\n", MAX_ACCEL_OF_EACH_TYPE, NUM_CV_ACCEL);
    parms_error = 1;
  }
  if (parms_error != 0) {
    cleanup_and_exit(sptr, -2);
  }
  pthread_mutex_init(&(sptr->free_metadata_mutex), NULL);
  pthread_mutex_init(&(sptr->accel_alloc_mutex), NULL);
  pthread_mutex_init(&(sptr->task_queue_mutex), NULL);
  //pthread_mutex_init(&(sptr->schedule_from_queue_mutex), NULL);

  struct timeval init_time;
  gettimeofday(&init_time, NULL);
  /*sptr->last_accel_use_update_time = init_time; // Start accounting at init time... ? */
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    sptr->master_metadata_pool[i].scheduler_datastate_pointer = sptr;
    sptr->master_metadata_pool[i].block_id = i; // Set the master pool's block_ids
    for (int ji = 0; ji < sptr->limits.max_task_types; ji++) {
      sptr->master_metadata_pool[i].gets_by_task_type[ji] = 0;
      sptr->master_metadata_pool[i].frees_by_task_type[ji] = 0;
    }
    // Clear the (full-run, aggregate) timing data spaces
    gettimeofday( &(sptr->master_metadata_pool[i].sched_timings.idle_start), NULL);
    // Scheduler timings 
    sptr->master_metadata_pool[i].sched_timings.idle_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.idle_usec = 0;
    sptr->master_metadata_pool[i].sched_timings.get_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.get_usec = 0;
    sptr->master_metadata_pool[i].sched_timings.queued_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.queued_usec = 0;
    for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
      sptr->master_metadata_pool[i].sched_timings.running_sec[ti] = 0;
      sptr->master_metadata_pool[i].sched_timings.running_usec[ti] = 0;
    }
    sptr->master_metadata_pool[i].sched_timings.done_sec = 0;
    sptr->master_metadata_pool[i].sched_timings.done_usec = 0;
    // Reset all the per-task type and targets timing data, too.
    for (int ai = 0; ai < sptr->limits.max_accel_types; ai++) {
      for (int ti = 0; ti < sptr->limits.max_task_types; ti++) {
	sptr->master_metadata_pool[i].task_computed_on[ai][ti] = 0;
      }
    }
    for (int ti = 0; ti < sptr->limits.max_task_types; ti++) {
      for (int tii = 0; tii < sptr->limits.max_task_timing_sets; tii++) {
	//sptr->master_metadata_pool[i].task_timings[ti].time_val[tii] = 0;
	for (int ai = 0; ai < sptr->limits.max_accel_types; ai++) {
	  sptr->master_metadata_pool[i].task_timings[ti].time_sec[tii][ai] = 0;
	  sptr->master_metadata_pool[i].task_timings[ti].time_usec[tii][ai] = 0;
	}
      }
    }

    // And some allocation stats stuff:
    for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
      for (int ai = 0; ai < MAX_ACCEL_OF_EACH_TYPE; ai++) {
	sptr->master_metadata_pool[i].accelerator_allocated_to_MB[ti][ai] = 0;
      }
    }

    pthread_mutex_init(&(sptr->master_metadata_pool[i].metadata_mutex), NULL);
    pthread_cond_init(&(sptr->master_metadata_pool[i].metadata_condv), NULL);

    sptr->free_metadata_pool[i] = i;    // Set up all blocks are free
    sptr->free_critlist_pool[i] = i;    // Set up all critlist blocks are free
  }

  // Now set up the task ready queue
  DEBUG(printf("Setting up the task ready queue...\n"));
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    sptr->ready_mb_task_queue_pool[i].unique_id = i;
    sptr->ready_mb_task_queue_pool[i].block_id = -1; // id -1 = unassigned
    if (i > 0) { 
      sptr->ready_mb_task_queue_pool[i].prev = &(sptr->ready_mb_task_queue_pool[i-1]);
    } else {
      sptr->ready_mb_task_queue_pool[i].prev = NULL;
    }
    if (i < (sptr->total_metadata_pool_blocks - 1)) {
      sptr->ready_mb_task_queue_pool[i].next = &(sptr->ready_mb_task_queue_pool[i+1]);
    } else {
      sptr->ready_mb_task_queue_pool[i].next = NULL;
    }
    DEBUG(printf("  set pool[%2u] @ %p id %i prev %p next %p\n", i, &(sptr->ready_mb_task_queue_pool[i]), sptr->ready_mb_task_queue_pool[i].block_id, sptr->ready_mb_task_queue_pool[i].prev, sptr->ready_mb_task_queue_pool[i].next));
  } // for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
  sptr->free_ready_mb_task_queue_entries = &(sptr->ready_mb_task_queue_pool[0]);
  sptr->ready_mb_task_queue_head = NULL;
  sptr->ready_mb_task_queue_tail = NULL;
  sptr->num_free_task_queue_entries = sptr->total_metadata_pool_blocks;
  DEBUG(printf(" AND free_ready_mb_task_queue_entries = %p\n", sptr->free_ready_mb_task_queue_entries));

  // Now initialize the per-metablock threads
  // For portability (as per POSIX documentation) explicitly create threads in joinable state
  pthread_attr_t  pt_attr;
  pthread_attr_init(&pt_attr);
  pthread_attr_setdetachstate(&pt_attr, PTHREAD_CREATE_JOINABLE);
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    if (pthread_create(&(sptr->metadata_threads[i]), &pt_attr, metadata_thread_wait_for_task, &(sptr->master_metadata_pool[i]))) {
      printf("ERROR: Scheduler failed to create thread for metadata block: %d\n", i);
      cleanup_and_exit(sptr, -10);
    }
    sptr->master_metadata_pool[i].thread_id = sptr->metadata_threads[i];
  }

  for (int ti = 0; ti < sptr->limits.max_task_types; ti++) {
    sptr->allocated_metadata_blocks[ti] = 0;
    sptr->freed_metadata_blocks[ti] = 0;
  }

  for (int i = 0; i < sptr->limits.max_accel_types; i++) {
    sptr->num_accelerators_of_type[i] = 0;
  }
  
  for (int i = 0; i < sptr->limits.max_accel_types; i++) {
    for (int j = 0; j < MAX_ACCEL_OF_EACH_TYPE; j++) {
      sptr->accelerator_in_use_by[i][j] = -1; // NOT a valid metadata block ID; -1 indicates "Not in Use"
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

  /** Moving this to just as the accelerator is registered...
     printf(" Calling do_accelerator_type_initialization...\n");
     do_accelerator_type_initialization();
  **/

  // Set up the Scheduler's Execution-Task-Function Table (for now by hand)
  for (int i = 0; i < sptr->limits.max_task_types; i++) {
    for (int j = 0; j < sptr->limits.max_accel_types; j++) {
      sptr->scheduler_execute_task_function[j][i] = NULL; // Set all to default to NULL
    }
    sptr->print_metablock_contents_function[i] = NULL;
    sptr->output_task_run_stats_function[i] = NULL;
  }

  for (int j = 0; j < sptr->limits.max_accel_types; j++) {
    sptr->do_accel_init_function[j] = NULL;
    sptr->do_accel_closeout_function[j] = NULL;
    sptr->output_accel_run_stats_function[j] = NULL;
  }

  // Now start the "schedule_executions_from_queue() pthread -- using the DETACHED pt_attr
  int pt_ret = pthread_create(&(sptr->scheduling_thread), &pt_attr, schedule_executions_from_queue, (void*)(sptr));
  if (pt_ret != 0) {
    printf("Could not start the scheduler pthread... return value %d\n", pt_ret);
    cleanup_and_exit(sptr, -1);
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



void
release_accelerator_for_task(task_metadata_block_t* task_metadata_block)
{
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  unsigned mdb_id     = task_metadata_block->block_id;
  unsigned accel_type = task_metadata_block->accelerator_type;
  unsigned accel_id   = task_metadata_block->accelerator_id;
  pthread_mutex_lock(&(sptr->accel_alloc_mutex));

  //printf("MB%u RELEASE  accelerator %u %u for %d cl %u\n", mdb_id, accel_type, accel_id, accelerator_in_use_by[accel_type][accel_id], task_metadata_block->crit_level);
  DEBUG(printf(" RELEASE accelerator %u  %u  = %d  : ", accel_type, accel_id, sptr->accelerator_in_use_by[accel_type][accel_id]);
	for (int ai = 0; ai < sptr->num_accelerators_of_type[accel_type]; ai++) {
	  printf("%u %d : ", ai, sptr->accelerator_in_use_by[accel_type][ai]);
	}
	printf("\n"));
  if (sptr->accelerator_in_use_by[accel_type][accel_id] != mdb_id) {
    printf("ERROR - in release_accelerator_for_task for ACCEL %s Num %d but BLOCK_ID Mismatch: %d vs %d\n", sptr->accel_name_str[accel_type], accel_id, sptr->accelerator_in_use_by[accel_type][accel_id], mdb_id);
    printf("  this occurred on finish of block:\n");
    print_base_metadata_block_contents(task_metadata_block);
    printf("Accelerators Info:\n");
    for (int ai = 0; ai < sptr->num_accelerators_of_type[accel_type]; ai++) {
      printf(" accelerator_in_use_by[ %u ][ %u ] = %d\n", accel_type, ai, sptr->accelerator_in_use_by[accel_type][ai]);
    }
  } else {
    account_accelerators_in_use_interval(sptr);
    sptr->accelerator_in_use_by[accel_type][accel_id] = -1; // Indicates "Not in Use"
  }
  pthread_mutex_unlock(&(sptr->accel_alloc_mutex));
}





 // This routine schedules (the first) ready task from the ready task queue
 // The input parm is a pointer to a scheduler_datastate_block_t structure 
void* schedule_executions_from_queue(void* void_parm_ptr) {
  DEBUG(printf("SCHED: starting execution of schedule_executions_from_queue thread...\n"));
  scheduler_datastate_block_t* sptr = (scheduler_datastate_block_t*)void_parm_ptr;
  // This will now be an eternally-running scheduler process, I think.
  //pthread_mutex_lock(&schedule_from_queue_mutex);
  while(1) {
    // If there is something in the task ready queue;
    if (sptr->num_tasks_in_ready_queue > 0) {
      DEBUG(printf("SCHED: num_tasks_in_ready_queue = %u\n", sptr->num_tasks_in_ready_queue);
	    int ti = 0;
	    ready_mb_task_queue_entry_t* t_te = sptr->ready_mb_task_queue_head;
	    while(t_te != NULL) {
	      printf("SCHED:    Ready_Task_Entry %2u : MB%u  %p %p\n", ti, t_te->block_id, t_te->prev, t_te->next);
	      ti++;
	      t_te = t_te->next;
	    });

      // Get pointer to the first task on the ready queue
      ready_mb_task_queue_entry_t* ready_task_entry = sptr->ready_mb_task_queue_head;
      ready_mb_task_queue_entry_t* selected_task_entry = NULL;
      task_metadata_block_t* task_metadata_block = NULL;

      // Select the target accelerator to execute the task
      DEBUG(printf("SCHED: calling assign_task_to_pe\n"));
      //Pass the head of the ready queue to parse entries in the queue
      selected_task_entry = sptr->assign_task_to_pe(sptr, ready_task_entry);
      if (selected_task_entry == NULL) {
        //No schedulable task
        continue;
      } else {
	task_metadata_block = &(sptr->master_metadata_pool[selected_task_entry->block_id]);
      }
      unsigned int accel_type = task_metadata_block->accelerator_type;
      unsigned int accel_id = task_metadata_block->accelerator_id;
      DEBUG(printf("SCHED: MB%u Selected accel type: %d id: accel_id: %d\n", task_metadata_block->block_id, task_metadata_block->accelerator_type, task_metadata_block->accelerator_id));

      if (accel_type == NO_Accelerator) {
        printf("SCHED: ERROR : Selected Task has no accelerator assigned\n");
        //pthread_mutex_unlock(&schedule_from_queue_mutex);
	print_base_metadata_block_contents(task_metadata_block);
        cleanup_and_exit(sptr, -19);
      } else {
	// Mark the requested accelerator as "In-USE" by this metadata block
	if (sptr->accelerator_in_use_by[accel_type][accel_id] != -1) {
	  printf("ERROR : schedule_executions_from_queue is trying to allocate ACCEL %s %u which is already allocated to Block %u\n", sptr->accel_name_str[accel_type], accel_id, sptr->accelerator_in_use_by[accel_type][accel_id]);
	  cleanup_and_exit(sptr, -14);
	}
	account_accelerators_in_use_interval(sptr);
	int bi = task_metadata_block->block_id; // short name for the block_id
	sptr->accelerator_in_use_by[accel_type][accel_id] = bi;
	sptr->master_metadata_pool[bi].accelerator_allocated_to_MB[accel_type][accel_id] += 1;
	// Okay -- we can allocate to the accelerator -- remove from the queue
	//printf("MB%u ALLOCATE accelerator %u %u to  %d cl %u\n", bi, accel_type, accel_id, bi, task_metadata_block->crit_level);
	DEBUG(printf("SCHED: MB%u ALLOC accelerator %u  %u to %d  : ", bi, accel_type, accel_id, bi);
	      for (int ai = 0; ai < sptr->num_accelerators_of_type[accel_type]; ai++) {
		printf("%u %d : ", ai, sptr->accelerator_in_use_by[accel_type][ai]);
	      }
	      printf("\n"));
	// Update the ready task queue... Connect selected_task_entry.prev->next = selected_task_entry.next

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
	DEBUG(printf("SCHED:   Set num_tasks_in_ready_queue to %u\n", sptr->num_tasks_in_ready_queue));

	DEBUG(printf("SCHED:   Adding back the ready task entry to the free list pre: %u entries\n", sptr->num_free_task_queue_entries));
	// Prepend to the free_ready_mb_task_queue_entries;
	if (sptr->free_ready_mb_task_queue_entries != NULL) {
	  // There are other free task queue entries
	  /* if (free_ready_mb_task_queue_entries->next != NULL) { */
	  /*   sptr->free_ready_mb_task_queue_entries->next->prev = ready_task_entry; */
	  /* }  */
	  sptr->free_ready_mb_task_queue_entries->prev = selected_task_entry;
	  selected_task_entry->next = sptr->free_ready_mb_task_queue_entries;
	}
	selected_task_entry->prev = NULL; // As head of the list, the prev should be NULL
	sptr->free_ready_mb_task_queue_entries = selected_task_entry;  
	sptr->num_free_task_queue_entries++;
	DEBUG(printf("SCHED:   Prepended to FREE ready task queue, with %u entries now\n", sptr->num_free_task_queue_entries));
	SDEBUG(print_free_ready_tasks_list(sptr));
	/* // And clean up the ready task storage... */
	/* ready_task_entry->block_id = -1; */
	/* ready_task_entry->next = NULL; */
	/* ready_task_entry->prev = NULL; */
	// And unlock the task-queue mutex
	pthread_mutex_unlock(&(sptr->task_queue_mutex));

	// Set the task to "RUNNING" State and account the times...
	task_metadata_block->status = TASK_RUNNING; // running

	gettimeofday(&sptr->master_metadata_pool[bi].sched_timings.running_start, NULL);
	sptr->master_metadata_pool[bi].sched_timings.queued_sec += sptr->master_metadata_pool[bi].sched_timings.running_start.tv_sec - sptr->master_metadata_pool[bi].sched_timings.queued_start.tv_sec;
	sptr->master_metadata_pool[bi].sched_timings.queued_usec += sptr->master_metadata_pool[bi].sched_timings.running_start.tv_usec - sptr->master_metadata_pool[bi].sched_timings.queued_start.tv_usec;

	TDEBUG(printf("Kicking off accelerator task for Metadata Block %u : Task %s %s on Accel %s %u\n", bi, sptr->task_name_str[task_metadata_block->task_type], sptr->task_criticality_str[task_metadata_block->crit_level], sptr->accel_name_str[task_metadata_block->accelerator_type], task_metadata_block->accelerator_id));

	// Lock the mutex associated to the conditional variable
	pthread_mutex_lock(&(task_metadata_block->metadata_mutex));

	// Signal the conditional variable -- triggers the target thread execution of accelerator
	pthread_cond_signal(&(task_metadata_block->metadata_condv));

	// And now we unlock because we are done here...
	pthread_mutex_unlock(&(task_metadata_block->metadata_mutex));
      }
    } else { // if (num_tasks_in_queue > 0)
      // I think perhaps we should add a short delay here to avoid this being such a busy spin loop...
      //   If we are using the 78MHz FPGA, then one clock cycle is ~12.82 ns ?
      //DEBUG(printf("Waiting for ready task in the queue...\n"));
      usleep(sptr->scheduler_holdoff_usec); // This defaults to 1 usec (about 78 FPGA clock cycles)
    }
  } // while (1)
  //pthread_mutex_unlock(&schedule_from_queue_mutex);
  return NULL;
}


void
request_execution(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("APP: in request_execution for MB%u\n", task_metadata_block->block_id));
  scheduler_datastate_block_t* sptr = task_metadata_block->scheduler_datastate_pointer;
  task_metadata_block->status = TASK_QUEUED; // queued
  gettimeofday(&task_metadata_block->sched_timings.queued_start, NULL);
  task_metadata_block->sched_timings.get_sec += task_metadata_block->sched_timings.queued_start.tv_sec - task_metadata_block->sched_timings.get_start.tv_sec;
  task_metadata_block->sched_timings.get_usec += task_metadata_block->sched_timings.queued_start.tv_usec - task_metadata_block->sched_timings.get_start.tv_usec;

  // Put this into the ready-task-queue
  //   Get a ready_task_queue_entry
  pthread_mutex_lock(&(sptr->task_queue_mutex));
  DEBUG(printf("APP: there are currently %u free task queue entries in the list\n", sptr->num_free_task_queue_entries));
  SDEBUG(print_free_ready_tasks_list(sptr));
  ready_mb_task_queue_entry_t* my_queue_entry = sptr->free_ready_mb_task_queue_entries;
  sptr->free_ready_mb_task_queue_entries = sptr->free_ready_mb_task_queue_entries->next;
  sptr->free_ready_mb_task_queue_entries->prev = NULL; // disconnect the prev pointer
  sptr->num_free_task_queue_entries--;
  DEBUG(printf("APP: and now there are %u free task queue entries in the list\n", sptr->num_free_task_queue_entries));
  SDEBUG(print_free_ready_tasks_list(sptr));
  //   Now fill it in
  my_queue_entry->block_id = task_metadata_block->block_id;
  DEBUG(printf("APP: got a free_task_ready_queue_entry, leaving %u free\n", sptr->num_free_task_queue_entries));
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
  DEBUG(printf("APP: and now there are %u ready tasks in the queue\n", sptr->num_tasks_in_ready_queue);
	print_ready_tasks_queue(sptr));
  pthread_mutex_unlock(&(sptr->task_queue_mutex));

  // NOW we should return -- we can later schedule tasks off the queue...
  /** This was for single-thread testing; now trying multi-thread mode
      schedule_executions_from_queue();
  **/
  return;
}



/********************************************************************************
 * Here are the wait routines -- for critical tasks or all tasks to finish
 ********************************************************************************/
void wait_all_critical(scheduler_datastate_block_t* sptr)
{
  // Loop through the critical tasks list and check whether they are all in status "done"
  blockid_linked_list_t* cli = sptr->critical_live_task_head;
  while (cli != NULL) {
    if (sptr->master_metadata_pool[cli->clt_block_id].status != TASK_DONE) {
      // This task is not finished yet.. wait for it
      //  So start polling from the start of the list again.
      cli = sptr->critical_live_task_head;
    } else {
      cli = cli->next;
    }
  }
}

void wait_all_tasks_finish(scheduler_datastate_block_t* sptr)
{
  // Spin loop : check whether all blocks are free...
  printf("Waiting for ALL tasks to finish: free = %u and total = %u\n", sptr->free_metadata_blocks, sptr->total_metadata_pool_blocks);
  while (sptr->free_metadata_blocks != sptr->total_metadata_pool_blocks) {
    ; // Nothing really to do, but wait.
  }
}


// This cleans up the state (pthreads, etc.) before exit
void cleanup_state(scheduler_datastate_block_t* sptr) {
  // Kill (cancel) the per-metablock threads
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    pthread_cancel(sptr->metadata_threads[i]);
  }
  pthread_cancel(sptr->scheduling_thread);
  // Clean out the pthread mutex and conditional variables
  pthread_mutex_destroy(&(sptr->free_metadata_mutex));
  pthread_mutex_destroy(&(sptr->accel_alloc_mutex));
  for (int i = 0; i < sptr->total_metadata_pool_blocks; i++) {
    pthread_mutex_destroy(&(sptr->master_metadata_pool[i].metadata_mutex));
    pthread_cond_destroy(&(sptr->master_metadata_pool[i].metadata_condv));
  }

  // Clean up any hardware accelerator stuff
  do_accelerator_type_closeout(sptr);
}


// This is called at the end of run/life to shut down the scheduler
//  This will also output a bunch of stats abdout timings, etc.

void output_run_statistics(scheduler_datastate_block_t* sptr)
{

  // NOW output some overall full-run statistics, etc.
  printf("\nOverall Accelerator allocation/usage statistics:\n");
  printf("\nTotal Scheduler Decision-Making Time was %lu usec for %lu decisions spanning %lu checks\n", sptr->scheduler_decision_time_usec, sptr->scheduler_decisions, sptr->scheduler_decision_checks);

  printf("\nScheduler block allocation/free statistics:\n");
  for (int ti = 0; ti < sptr->limits.max_task_types; ti++) {
    printf("  For %12s Scheduler allocated %9u blocks and freed %9u blocks\n", sptr->task_name_str[ti], sptr->allocated_metadata_blocks[ti], sptr->freed_metadata_blocks[ti]);
  }
  printf(" During FULL run,  Scheduler allocated %9u blocks and freed %9u blocks in total\n", sptr->allocated_metadata_blocks[NO_Task], sptr->freed_metadata_blocks[NO_Task]);

  printf("\nPer-MetaData-Block Scheduler Allocation/Frees by Job-Type Data:\n");
  printf("%6s ", "Block");
  for (int ji = 1; ji < sptr->limits.max_task_types; ji++) {
    printf("%12s_G %12s_F ", sptr->task_name_str[ji], sptr->task_name_str[ji]);
  }
  printf("\n");
  unsigned type_gets[sptr->limits.max_task_types];
  unsigned type_frees[sptr->limits.max_task_types];
  for (int ji = 0; ji < sptr->limits.max_task_types; ji++) {
    type_gets[ji] = 0;
    type_frees[ji] = 0;
  }
  for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
    printf("%6u ", bi);
    for (int ji = 1; ji < sptr->limits.max_task_types; ji++) {
      type_gets[ji]  += sptr->master_metadata_pool[bi].gets_by_task_type[ji];
      type_frees[ji] += sptr->master_metadata_pool[bi].frees_by_task_type[ji];
      printf("%14u %14u ", sptr->master_metadata_pool[bi].gets_by_task_type[ji], sptr->master_metadata_pool[bi].frees_by_task_type[ji]);
    }
    printf("\n");
  }
  printf("%6s ", "Total");
  for (int ji = 1; ji < sptr->limits.max_task_types; ji++) {
    printf("%14u %14u ", type_gets[ji], type_frees[ji]);
  }
  printf("\n");

  printf("\nPer-MetaData-Block Scheduler Timing Data:\n");
  {
    unsigned total_blocks_used  = 0;
    uint64_t total_idle_usec    = 0;
    uint64_t total_get_usec     = 0;
    uint64_t total_queued_usec  = 0;
    uint64_t total_running_usec[sptr->limits.max_accel_types];
    for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
      total_running_usec[ti] = 0;
    }
    uint64_t total_done_usec    = 0;
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      uint64_t this_idle_usec = (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.idle_sec) * 1000000 + (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.idle_usec);
      uint64_t this_get_usec = (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.get_sec) * 1000000 + (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.get_usec);
      uint64_t this_queued_usec = (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.queued_sec) * 1000000 + (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.queued_usec);
      uint64_t this_total_run_usec = 0;
      uint64_t this_running_usec[sptr->limits.max_accel_types];
      for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
	this_running_usec[ti] = (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.running_sec[ti]) * 1000000 + (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.running_usec[ti]);
	this_total_run_usec += this_running_usec[ti];
      }
      uint64_t this_done_usec = (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.done_sec) * 1000000 + (uint64_t)(sptr->master_metadata_pool[bi].sched_timings.done_usec);
      printf(" Block %3u : IDLE %15lu GET %15lu QUE %15lu RUN %15lu DONE %15lu usec :", bi, this_idle_usec, this_get_usec, this_queued_usec, this_total_run_usec,  total_done_usec);
      for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
	printf(" %15lu", this_running_usec[ti]);
      }
      printf("\n");
      if (this_idle_usec != 0) { 
	total_blocks_used++; 
      }
      total_idle_usec    += this_idle_usec;
      total_get_usec     += this_get_usec;
      total_queued_usec  += this_queued_usec;
      for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
	total_running_usec[ti] += this_running_usec[ti];
      }
      total_done_usec    += this_done_usec;
    }
    double avg;
    printf("\nScheduler Timings: Aggregate Across all Metadata Blocks with %u used blocks\n", total_blocks_used);
    avg = (double)total_idle_usec/(double)total_blocks_used;
    printf("  Metablocks_IDLE total run time:                %15lu usec : %16.2lf (average)\n", total_idle_usec, avg);
    avg = (double)total_get_usec/(double)total_blocks_used;
    printf("  Metablocks_GET total run time:                 %15lu usec : %16.2lf (average)\n", total_get_usec, avg);
    avg = (double)total_queued_usec/(double)total_blocks_used;
    printf("  Metablocks_QUEUED total run time:              %15lu usec : %16.2lf (average)\n", total_queued_usec, avg);
    for (int ti = 0; ti < sptr->limits.max_accel_types; ti++) {
      avg = (double)total_running_usec[ti]/(double)total_blocks_used;
      printf("  Metablocks_RUNNING %u %15s run time: %15lu usec : %16.2lf (average)\n", ti, sptr->accel_name_str[ti], total_running_usec[ti], avg);
    }
    avg = (double)total_done_usec/(double)total_blocks_used;
    printf("  Metablocks_DONE total run time:                %15lu usec : %16.2lf (average)\n", total_done_usec, avg);
  }

  output_task_and_accel_run_stats(sptr);
    
  /*printf("\nACU_HIST: Aggregated In-Use Accelerator Time Histogram...\n");
  {
    printf("ACU_HIST:  CPU  FFT  VIT  CNN : TACC TFFT TVIT TCNN : Time-in-usec\n");
    for (int i0 = 0; i0 <= sptr->num_accelerators_of_type[0]; i0++) {
      for (int i1 = 0; i1 <= sptr->num_accelerators_of_type[1]; i1++) {
	for (int i2 = 0; i2 <= sptr->num_accelerators_of_type[2]; i2++) {
	  for (int i3 = 0; i3 <= sptr->num_accelerators_of_type[3]; i3++) {
	    printf("ACU_HIST: %4u %4u %4u %4u : %4u : %lu\n", i0, i1, i2, i3, (i1+i2+i3), sptr->in_use_accel_times_array[i0][i1][i2][i3]);
	  }
	}
      }
    }
    }*/

  printf("\nAccelerator Usage Statistics:\n");
  {
    unsigned totals[sptr->limits.max_accel_types-1][MAX_ACCEL_OF_EACH_TYPE];
    unsigned top_totals[sptr->limits.max_accel_types-1];
    for (int ti = 0; ti < sptr->limits.max_accel_types-1; ti++) {
      top_totals[ti] = 0;
      for (int ai = 0; ai < MAX_ACCEL_OF_EACH_TYPE; ai++) {
	totals[ti][ai] = 0;
	for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
	  totals[ti][ai] += sptr->master_metadata_pool[bi].accelerator_allocated_to_MB[ti][ai];
	}
      }
    }
    printf("\nPer-Accelerator allocation/usage statistics:\n");
    for (int ti = 0; ti < sptr->limits.max_accel_types-1; ti++) {
      for (int ai = 0; ai < MAX_ACCEL_OF_EACH_TYPE; ai++) {
	if (ai < sptr->num_accelerators_of_type[ti]) { 
	  printf(" Acc_Type %u %s : Accel %2u Allocated %6u times\n", ti, sptr->accel_name_str[ti], ai, totals[ti][ai]);
	} else {
	  if (totals[ti][ai] != 0) {
	    printf("ERROR : We have use of non-existent Accelerator %u %s : index %u = %u\n", ti, sptr->accel_name_str[ti], ai, totals[ti][ai]);
	  }
	}
	top_totals[ti]+= totals[ti][ai];
      }
    }
    printf("\nPer-Accelerator-Type allocation/usage statistics:\n");
    for (int ti = 0; ti < sptr->limits.max_accel_types-1; ti++) {
      printf(" Acc_Type %u %s Allocated %6u times\n", ti, sptr->accel_name_str[ti], top_totals[ti]);
    }
    printf("\nPer-Meta-Block Accelerator allocation/usage statistics:\n");
    for (int ti = 0; ti < sptr->limits.max_accel_types-1; ti++) {
      for (int ai = 0; ai < sptr->num_accelerators_of_type[ti]; ai++) {
	for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
	  if (sptr->master_metadata_pool[bi].accelerator_allocated_to_MB[ti][ai] != 0) {
	    printf(" Per-MB Acc_Type %u %s : Accel %2u Allocated %6u times for MB%u\n", ti, sptr->accel_name_str[ti], ai, sptr->master_metadata_pool[bi].accelerator_allocated_to_MB[ti][ai], bi);
	  }
	}
      }
    }
  }
}

void shutdown_scheduler(scheduler_datastate_block_t* sptr)
{
  output_run_statistics(sptr);

  // Dynamically unload the scheduling policy (plug-in)
  dlclose(sptr->policy_handle);

  cleanup_state(sptr);

  free(sptr->free_metadata_pool);
  free(sptr->ready_mb_task_queue_pool);
  free(sptr->metadata_threads);
  free(sptr->critical_live_tasks_list);
  free(sptr->free_critlist_pool);
  free(sptr->master_metadata_pool);
  free(sptr);
}



void cleanup_and_exit(scheduler_datastate_block_t* sptr, int rval) {
  cleanup_state(sptr);
  exit (rval);
}




void dump_all_metadata_blocks_states(scheduler_datastate_block_t* sptr)
{
  if (sptr->free_metadata_blocks == 0) {
    printf("FREE_MBS: { }\n");
  } else {
    for (int i = 0; i < sptr->free_metadata_blocks; i++) {
      if (i > 0) { printf(","); };
      printf("%u", sptr->free_metadata_pool[i]);
    }
    printf("\n");
  }
  //unsigned allocated_metadata_blocks[sptr->limits.max_task_types];
  printf("Total Allocated MBs:  ");
  for (int i = 0; i < sptr->limits.max_task_types; i++) {
    printf("( %s, %u ) ", sptr->task_name_str[i], sptr->allocated_metadata_blocks[i]);
  }
  printf("\n");
  //unsigned freed_metadata_blocks[sptr->limits.max_task_types];
  printf("Total Freed MBs:  ");
  for (int i = 0; i < sptr->limits.max_task_types; i++) {
    printf("( %s, %u ) ", sptr->task_name_str[i], sptr->freed_metadata_blocks[i]);
  }
  printf("\n");
  printf("\nData for EACH MB:\n");
  for (int mbi = 0; mbi < sptr->total_metadata_pool_blocks; mbi++) {
    printf("MB%u : Status %u %s\n", mbi, sptr->master_metadata_pool[mbi].status, sptr->task_status_str[sptr->master_metadata_pool[mbi].status]);
    printf("  MB%u : Acc_ty %u   Acc_id %d   Job %u   Crit_Lvl %u\n", mbi, 
	   sptr->master_metadata_pool[mbi].accelerator_type, sptr->master_metadata_pool[mbi].accelerator_id, 
	   sptr->master_metadata_pool[mbi].task_type, //task_name_str[sptr->master_metadata_pool[mbi].task_type],
	   sptr->master_metadata_pool[mbi].crit_level);
    printf("  MB%u GETS:  ", mbi);
    for (int i = 0; i < sptr->limits.max_task_types; i++) {
      printf("( %s, %u ) ", sptr->task_name_str[i], sptr->master_metadata_pool[mbi].gets_by_task_type[i]);
    }
    printf("\n");
    printf("  MB%u FREES:  ", mbi);
    for (int i = 0; i < sptr->limits.max_task_types; i++) {
      printf("( %s, %u ) ", sptr->task_name_str[i], sptr->master_metadata_pool[mbi].frees_by_task_type[i]);
    }
    printf("\n");
  } // for (mbi loop over Metablocks)
}



task_id_t
register_task_type(scheduler_datastate_block_t* sptr, task_type_defn_info_t* tinfo)
{
  DEBUG(printf("In register_task_type with inputs:\n");
	printf("  name  = %s\n", tinfo->name);
	printf("  description  = %s\n", tinfo->description);
	printf("  print_metadata_block_contents = %p\n", tinfo->print_metadata_block_contents);
	/* printf("  do_task_type_initialization   = %p\n", tinfo->do_task_type_initialization); */
	/* printf("  do_task_type_closeout_t       = %p\n", tinfo->do_task_type_closeout); */
	printf("  output_task_type_run_stats_t  = %p\n", tinfo->output_task_type_run_stats));

  printf("Registering Task %s : %s\n", tinfo->name, tinfo->description);
  
  if (tinfo->print_metadata_block_contents == NULL) {
    printf("ERROR: Must set print_metadata_block_contents function -- can use base routine\n");
    cleanup_and_exit(sptr, -30);
  }
  // Okay, so here is where we "fill in" the scheduler's task-type information for this task
  task_id_t tid = sptr->next_avail_task_id;
  if (tid < sptr->limits.max_task_types) {
    sptr->next_avail_task_id++;
  } else {
    printf("ERROR: Ran out of Task IDs: MAX_TASK_ID = %u and we are adding %u\n", sptr->limits.max_task_types, tid);
    cleanup_and_exit(sptr, -31);
  }
  snprintf(sptr->task_name_str[tid], MAX_TASK_NAME_LEN, "%s", tinfo->name);
  snprintf(sptr->task_desc_str[tid], MAX_TASK_DESC_LEN, "%s", tinfo->description);
  sptr->output_task_run_stats_function[tid] =  tinfo->output_task_type_run_stats;
  
  return tid;
}


accelerator_type_t
register_accelerator_pool(scheduler_datastate_block_t* sptr, accelerator_pool_defn_info_t* info)
{
  DEBUG(printf("In register_accelerator_pool with inputs:\n");
	printf("  name  = %s\n", info->name);
	printf("  description  = %s\n", info->description);
	printf("  do_accel_initialization   = %p\n", info->do_accel_initialization);
	printf("  do_accel_closeout_t       = %p\n", info->do_accel_closeout);
	printf("  output_accel_run_stats_t  = %p\n", info->output_accel_run_stats));
  printf("Registering Accelerator %s : %s\n", info->name, info->description);
	
  // Okay, so here is where we "fill in" the scheduler's accel-type information for this accel
  accelerator_type_t acid = sptr->next_avail_accel_id;
  if (acid < sptr->limits.max_accel_types) {
    sptr->next_avail_accel_id++;
  } else {
    printf("ERROR: Ran out of Accel IDs: MAX_ACCEL_ID = %u and we are adding %u\n", sptr->limits.max_accel_types, acid);
    cleanup_and_exit(sptr, -32);
  }
  snprintf(sptr->accel_name_str[acid], MAX_ACCEL_NAME_LEN, "%s", info->name);
  snprintf(sptr->accel_desc_str[acid], MAX_ACCEL_DESC_LEN, "%s", info->description);
  sptr->num_accelerators_of_type[acid]   = info->number_available;
  sptr->do_accel_init_function[acid]     = info->do_accel_initialization;
  sptr->do_accel_closeout_function[acid] = info->do_accel_closeout;
  sptr->output_accel_run_stats_function[acid] =  info->output_accel_run_stats;
  // Now initialize this accelerator
  if (sptr->do_accel_init_function[acid] != NULL) {
    DEBUG(printf(" Calling the accelerator initialization function...\n"));
    sptr->do_accel_init_function[acid](NULL);
  } else {
    printf("Note: accelerator initialization function is NULL\n");
  }
  if (MAX_ACCEL_OF_EACH_TYPE < sptr->num_accelerators_of_type[acid]) {
    printf("ERROR: MAX_ACCEL_OF_EACH_TYPE < sptr->num_accelerators_of_type[%u] : %u < %u\n", acid, MAX_ACCEL_OF_EACH_TYPE, sptr->num_accelerators_of_type[acid]);
    cleanup_and_exit(sptr, -33);
  }
  return acid;
}


void
register_accel_can_exec_task(scheduler_datastate_block_t* sptr, accelerator_type_t acid, task_id_t tid, sched_execute_task_function_t fptr)
{
  DEBUG(printf("In register_accel_can_exec_task for accel %u and task %u with fptr %p\n", acid, tid, fptr));
  if (acid >= sptr->next_avail_accel_id) {
    printf("In register_accel_can_exec_task specified an illegal accelerator id: %u vs %u currently defined\n", acid, sptr->next_avail_accel_id);
    cleanup_and_exit(sptr, -36);
  }
  if (tid >= sptr->next_avail_task_id) {
    printf("In register_task_can_exec_task specified an illegal taskerator id: %u vs %u currently defined\n", tid, sptr->next_avail_task_id);
    cleanup_and_exit(sptr, -37);
  }
  if (sptr->scheduler_execute_task_function[acid][tid] != NULL) {
    printf("In register_accel_can_exec_task for accel_type %u and task_type %u - Already have a registered execution (%p)\n", acid, tid, sptr->scheduler_execute_task_function[tid][acid]);
    cleanup_and_exit(sptr, -38);
  }
  sptr->scheduler_execute_task_function[acid][tid] = fptr;
  DEBUG(printf("  Set scheduler_execute_task_function[acid = %u ][tid = %u ]  to %p\n", acid, tid, fptr));
  printf("Set scheduler_execute_task_function for Task %s on Accelerator Type %s\n", sptr->task_name_str[tid], sptr->accel_name_str[acid]);
}
