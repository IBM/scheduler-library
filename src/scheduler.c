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

#include "utils.h"
//#define VERBOSE
#include "verbose.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "scheduler.h"
#include "accelerators.h" // include AFTER scheduler.h -- needs types form scheduler.h


unsigned int scheduler_holdoff_usec = 1;

// Forward declarations
void release_accelerator_for_task(task_metadata_block_t* task_metadata_block);

accel_select_policy_t global_scheduler_selection_policy = 1;

unsigned cv_cpu_run_time_in_usec      = 10000;
unsigned cv_fake_hwr_run_time_in_usec =  1000;

task_metadata_block_t master_metadata_pool[total_metadata_pool_blocks];

pthread_mutex_t free_metadata_mutex; // Used to guard access to altering the free-list metadata information, etc.
int free_metadata_pool[total_metadata_pool_blocks];
int free_metadata_blocks = total_metadata_pool_blocks;
unsigned allocated_metadata_blocks[NUM_JOB_TYPES];
unsigned freed_metadata_blocks[NUM_JOB_TYPES];

// This is the Ready Task Queue -- it holds Metadata Block IDs
//typedef struct ready_mb_task_queue_entry_struct {
//  short  unique_id;
//  short  block_id;
//  struct ready_mb_task_queue_entry_struct * next;
//  struct ready_mb_task_queue_entry_struct * prev;
//} ready_mb_task_queue_entry_t;

pthread_mutex_t task_queue_mutex;   // Used to guard access to altering the ready-task-queue contents
ready_mb_task_queue_entry_t ready_mb_task_queue_pool[total_metadata_pool_blocks];
ready_mb_task_queue_entry_t* free_ready_mb_task_queue_entries;
ready_mb_task_queue_entry_t* ready_mb_task_queue_head;
ready_mb_task_queue_entry_t* ready_mb_task_queue_tail;
unsigned num_free_task_queue_entries = 0;
unsigned num_tasks_in_ready_queue = 0;


pthread_mutex_t accel_alloc_mutex;   // Used to guard access to altering the accelerator allocations

pthread_t metadata_threads[total_metadata_pool_blocks]; // One thread per metadata block (to exec it in)

pthread_mutex_t schedule_from_queue_mutex;   // Used to guard access to scheduling functionality
pthread_t scheduling_thread;

typedef struct bi_ll_struct { int clt_block_id;  struct bi_ll_struct* next; } blockid_linked_list_t;

blockid_linked_list_t critical_live_tasks_list[total_metadata_pool_blocks];
blockid_linked_list_t* critical_live_task_head = NULL;
//blockid_linked_list_t* critical_live_task_tail = NULL;
int free_critlist_pool[total_metadata_pool_blocks];
int free_critlist_entries = total_metadata_pool_blocks;
int total_critical_tasks = 0;


const char* task_job_str[NUM_JOB_TYPES] = { "NO-JOB",
					    "FFT-TASK",
					    "VITERBI-TASK",
					    "CV-CNN-TASK" };

const char* task_criticality_str[NUM_TASK_CRIT_LEVELS] = { "NO-TASK",
							   "BASE-TASK",
							   "ELEVATED-TASK",
							   "CRITICAL-TASK" };

const char* task_status_str[NUM_TASK_STATUS] = {"TASK-FREE",
						"TASK-ALLOCATED",
						"TASK-QUEUED",
						"TASK-RUNNING",
						"TASK-DONE"};

const char* accel_type_str[NUM_ACCEL_TYPES] = { "CPU-ACCELERATOR",
						"FFT-HWR-ACCEL",
						"VITERBI-HWR-ACCEL",
						"VISION-HWR-ACCEL",
						"NO-ACCELERATOR"};

const char* scheduler_selection_policy_str[NUM_SELECTION_POLICIES] = { "Select_Accelerator_Type_and_Wait_Available",
								       "Fastest_to_Slowest_First_Available",
								       "Fastest_Finish_Time_First",
								       "Fastest_Finish_Time_First_Queued" } ;

// This is a table of the execution functions for the various Task Types in the scheduler
//  We set this up with one "set" of entries per JOB_TYPE
//   where each set has one execute function per possible TASK TARGET (on which it can execute)
//   Currently the targets are "CPU" and "HWR" -- this probably has to change (though this interpretation is only convention).
sched_execute_task_function_t scheduler_execute_task_function[NUM_JOB_TYPES][NUM_ACCEL_TYPES];

//#define  MAX_ACCEL_OF_EACH_TYPE     8

volatile int accelerator_in_use_by[NUM_ACCEL_TYPES-1][MAX_ACCEL_OF_EACH_TYPE];
unsigned int accelerator_allocated_to_MB[NUM_ACCEL_TYPES-1][MAX_ACCEL_OF_EACH_TYPE][total_metadata_pool_blocks];
int num_accelerators_of_type[NUM_ACCEL_TYPES-1];

struct timeval last_accel_use_update_time;
uint64_t in_use_accel_times_array[NUM_CPU_ACCEL+1][NUM_FFT_ACCEL+1][NUM_VIT_ACCEL+1][NUM_CV_ACCEL+1];

uint64_t scheduler_decision_time_usec = 0;
uint32_t scheduler_decisions = 0;
uint32_t scheduler_decision_checks = 0;

// This is defined per accelerator type            CPU            FFT            VIT            CV        None
unsigned input_accel_limit[NUM_ACCEL_TYPES] = {NUM_CPU_ACCEL, NUM_FFT_ACCEL, NUM_VIT_ACCEL, NUM_CV_ACCEL, 0};


void* schedule_executions_from_queue(void* void_parm_ptr);


void print_ready_tasks_queue() {
	int ti = 0;
	ready_mb_task_queue_entry_t* t_te = ready_mb_task_queue_head;
	while(t_te != NULL) {
		printf(" RTQ: Ready_Task_Entry %2u of %2u : ID %2u MB%d P: %p T: %p N: %p\n", ti, num_tasks_in_ready_queue, t_te->unique_id, t_te->block_id, t_te->prev, t_te, t_te->next);
		ti++;
		t_te = t_te->next;
	}
}

void print_free_ready_tasks_list() {
	int i = 0;
	ready_mb_task_queue_entry_t* t_te = free_ready_mb_task_queue_entries;
	while(t_te != NULL) {
		printf(" FRTL: Entry %2u of %2u : ID %2u MB%d P: %p T: %p N: %p\n", i, num_free_task_queue_entries, t_te->unique_id, t_te->block_id, t_te->prev, t_te, t_te->next);
		i++;
		t_te = t_te->next;
	}
}


void init_accelerators_in_use_interval(struct timeval start_time) {
	last_accel_use_update_time = start_time;
}


void account_accelerators_in_use_interval()
{
	// Count which accelerators are in use
	int acc_in_use[NUM_ACCEL_TYPES-1];
	for (int i = 0; i < NUM_ACCEL_TYPES-1; i++) {
		acc_in_use[i] = 0;
		for (int j = 0; j < num_accelerators_of_type[i]; j++) {
			if (accelerator_in_use_by[i][j] != -1) {
				acc_in_use[i]++;
			}
		}
	}
	//Now we have the array of curr-in-use
	struct timeval now;
	gettimeofday(&now, NULL);
	// This is a 4-Dim by ACCEL type : [CPU][FFT][VIT][CV]
	in_use_accel_times_array[acc_in_use[0]][acc_in_use[1]][acc_in_use[2]][acc_in_use[3]] += 1000000*(now.tv_sec - last_accel_use_update_time.tv_sec) + (now.tv_usec - last_accel_use_update_time.tv_usec);
	last_accel_use_update_time = now;
}

void print_base_metadata_block_contents(task_metadata_block_t* mb)
{
	printf("block_id = %d @ %p\n", mb->block_id, mb);
	unsigned status = mb->status;
	if (status < NUM_TASK_STATUS) {
		printf(" ** status = %s\n", task_status_str[status]);
	} else {
		printf(" ** status = %d <= NOT a legal value!\n",  mb->status);
	}
	unsigned job_type = mb->job_type;
	if (job_type < NUM_JOB_TYPES) {
		printf("    job_type = %s\n", task_job_str[job_type]);
	} else {
		printf(" ** job_type = %d <= NOT a legal value!\n", mb->job_type);
	}
	unsigned crit_level = mb->crit_level;
	if (crit_level < NUM_TASK_CRIT_LEVELS) {
		printf("    crit_level = %s\n",  task_criticality_str[crit_level]);
	} else {
		printf(" ** crit_level = %d <= NOT a legal value!\n",  mb->crit_level);
	}
	printf("    data_size  = %d\n",  mb->data_size);
	printf("    data_space @ %p\n", &(mb->data_space));
}



void print_critical_task_list_ids() {
	blockid_linked_list_t* cli = critical_live_task_head;
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


// There is an open question as to whether we should "Wait" for an available Metadata Block
//  or return if there are none available to the program (and let IT decide what to do next)
//  Currently, we have to return, or else the scheduler task cannot make progress and free
//  additional metablocks.... so we require the caller to do the "while" loop...

task_metadata_block_t* get_task_metadata_block(scheduler_jobs_t task_type, task_criticality_t crit_level, uint64_t * task_profile)
{
	pthread_mutex_lock(&free_metadata_mutex);
	TDEBUG(printf("in get_task_metadata_block with %u free_metadata_blocks\n", free_metadata_blocks));
	if (free_metadata_blocks < 1) {
		// Out of metadata blocks -- all in use, cannot enqueue new tasks!
		return NULL;
	}
	int bi = free_metadata_pool[free_metadata_blocks - 1];
	TDEBUG(printf(" BEFORE_GET : MB%d : free_metadata_pool : ", bi);
		for (int i = 0; i < total_metadata_pool_blocks; i++) {
			printf("%d ", free_metadata_pool[i]);
		}
		printf("\n"));
	if ((bi < 0) || (bi > total_metadata_pool_blocks)) {
		printf("ERROR : free_metadata_pool[%u -1] = %d   with %d free_metadata_blocks\n", free_metadata_blocks, bi, free_metadata_blocks);
		for (int i = 0; i < total_metadata_pool_blocks; i++) {
			printf("  free_metadata_pool[%2u] = %d\n", i, free_metadata_pool[i]);
		}
		cleanup_and_exit(-16);
	}
	free_metadata_pool[free_metadata_blocks - 1] = -1;
	free_metadata_blocks -= 1;
	// For neatness (not "security") we'll clear the meta-data in the block (not the data data,though)
	master_metadata_pool[bi].job_type = task_type;
	master_metadata_pool[bi].gets_by_type[task_type]++;
	master_metadata_pool[bi].status = TASK_ALLOCATED;
	master_metadata_pool[bi].crit_level = crit_level;
	for (int i = 0; i < NUM_ACCEL_TYPES; ++i) {
		master_metadata_pool[bi].task_profile[i] = task_profile[i];
	}
	master_metadata_pool[bi].data_size = 0;
	master_metadata_pool[bi].accelerator_type = no_accelerator_t;
	master_metadata_pool[bi].accelerator_id   = -1;
	master_metadata_pool[bi].atFinish = NULL;  // NO finish call-back function
  
	gettimeofday(&master_metadata_pool[bi].sched_timings.get_start, NULL);
	master_metadata_pool[bi].sched_timings.idle_sec += master_metadata_pool[bi].sched_timings.get_start.tv_sec - master_metadata_pool[bi].sched_timings.idle_start.tv_sec;
	master_metadata_pool[bi].sched_timings.idle_usec += master_metadata_pool[bi].sched_timings.get_start.tv_usec - master_metadata_pool[bi].sched_timings.idle_start.tv_usec;
  
	if (crit_level > 1) { // is this a "critical task"
		/* int ci = total_critical_tasks; // Set index for crit_task block_id in pool */
		/* critical_live_tasks_list[ci].clt_block_id = bi;  // Set the critical task block_id indication */
		// Select the next available critical-live-task-list entry ID 
		int li = free_critlist_pool[free_critlist_entries - 1];
		free_critlist_pool[free_critlist_entries - 1] = -1; // clear it a(as it is no longer free)
		free_critlist_entries -= 1;
		// Now li indicates the critical_live_tasks_list[] index to use
		// Now set up the revisions to the critical live tasks list
		critical_live_tasks_list[li].clt_block_id = bi;   // point this entry to the master_metadata_pool block id
		critical_live_tasks_list[li].next = critical_live_task_head;     // Insert as head of critical tasks list
		critical_live_task_head = &(critical_live_tasks_list[li]);
		total_critical_tasks += 1;
	}
	DEBUG(printf("  returning block %u\n", bi);
		print_critical_task_list_ids());
	TDEBUG(printf(" AFTER_GET : MB%u : free_metadata_pool : ", bi);
		for (int i = 0; i < total_metadata_pool_blocks; i++) {
			printf("%d ", free_metadata_pool[i]);
		}
		printf("\n"));
	allocated_metadata_blocks[task_type]++;
	//printf("MB%u got allocated : %u %u\n", bi, task_type, crit_level);
	pthread_mutex_unlock(&free_metadata_mutex);
  
	return &(master_metadata_pool[bi]);
}





void free_task_metadata_block(task_metadata_block_t* mb)
{
	pthread_mutex_lock(&free_metadata_mutex);

	int bi = mb->block_id;
	//printf("MB%u getting freed : %u %u\n", bi, mb->job_type, mb->crit_level);
	TDEBUG(printf("in free_task_metadata_block for block %u with %u free_metadata_blocks\n", bi, free_metadata_blocks);//);
		printf(" BEFORE_FREE : MB%u : free_metadata_pool : ", bi);
		for (int i = 0; i < total_metadata_pool_blocks; i++) {
			printf("%d ", free_metadata_pool[i]);
		}
		printf("\n"));

	if (free_metadata_blocks < total_metadata_pool_blocks) {
		master_metadata_pool[bi].frees_by_type[mb->job_type]++;
		free_metadata_pool[free_metadata_blocks] = bi;
		free_metadata_blocks += 1;
		if (master_metadata_pool[bi].crit_level > 1) { // is this a critical tasks?
			// Remove task form critical list, free critlist entry, etc.
			blockid_linked_list_t* lcli = NULL;
			blockid_linked_list_t* cli = critical_live_task_head;
			//while ((cli != NULL) && (critical_live_tasks_list[cli->clt_block_id].clt_block_id != bi)) {
			while ((cli != NULL) && (cli->clt_block_id != bi)) {
				lcli = cli;  // The "previoud" block; NULL == "head"
				cli = cli->next;  
			}
			if (cli == NULL) {
				printf("ERROR: Critical task NOT on the critical_live_task_list :\n");
				print_base_metadata_block_contents(mb);
				cleanup_and_exit(-6);
			}
			// We've found the critical task in critical_live_tasks_list - cli points to it
			int cti = cli->clt_block_id;
			//printf(" freeing critlist_pool %u to %u\n", free_critlist_entries - 1, cti);
			free_critlist_pool[free_critlist_entries] = cti; // Enable this crit-list entry for new use
			free_critlist_entries += 1; // Update the count of available critlist entries in the pool
			cli->clt_block_id = -1; // clear the clt_block_id indicator (we're done with it)
			// And remove the cli entry from the critical_lvet_tasks linked list
			if (lcli == NULL) {
				critical_live_task_head = cli->next;
			} else {
				lcli->next = cli->next;
			}
			cli->next = NULL;
			total_critical_tasks -= 1;
		}
		master_metadata_pool[bi].atFinish = NULL; // Ensure this is now set to NULL (safety safety)
		// For neatness (not "security") we'll clear the meta-data in the block (not the data data, though)
		freed_metadata_blocks[master_metadata_pool[bi].job_type]++;
		master_metadata_pool[bi].job_type = NO_TASK_JOB; // unset
		master_metadata_pool[bi].status = TASK_FREE;   // free
		gettimeofday(&master_metadata_pool[bi].sched_timings.idle_start, NULL);
		master_metadata_pool[bi].sched_timings.done_sec += master_metadata_pool[bi].sched_timings.idle_start.tv_sec - master_metadata_pool[bi].sched_timings.done_start.tv_sec;
		master_metadata_pool[bi].sched_timings.done_usec += master_metadata_pool[bi].sched_timings.idle_start.tv_usec - master_metadata_pool[bi].sched_timings.done_start.tv_usec;
		master_metadata_pool[bi].crit_level = NO_TASK; // lowest/free?
		master_metadata_pool[bi].data_size = 0;
	} else {
		printf("ERROR : We are freeing a metadata block when we already have max metadata blocks free...\n");
		printf("   THE FREE Metadata Blocks list:\n");
		for (int ii = 0; ii < free_metadata_blocks; ii++) {
			printf("        free[%2u] = %u\n", ii, free_metadata_pool[ii]);
		}
		DEBUG(printf("    THE Being-Freed Meta-Data Block:\n");
			print_base_metadata_block_contents(mb));
		cleanup_and_exit(-5);
	}
	TDEBUG(printf(" AFTER_FREE : MB%u : free_metadata_pool : ", bi);
		for (int i = 0; i < total_metadata_pool_blocks; i++) {
			printf("%d ", free_metadata_pool[i]);
		}
		printf("\n"));
	pthread_mutex_unlock(&free_metadata_mutex);
}




int
get_task_status(int task_id) {
	return master_metadata_pool[task_id].status;
}


void mark_task_done(task_metadata_block_t* task_metadata_block)
{
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
	DEBUG(printf("In execute_task_on_accelerator for MB%d with Accel Type %s and Number %u\n", task_metadata_block->block_id, accel_type_str[task_metadata_block->accelerator_type], task_metadata_block->accelerator_id));
	if (task_metadata_block->accelerator_type != no_accelerator_t) {
		if ((task_metadata_block->job_type > 0) && (task_metadata_block->job_type < NUM_JOB_TYPES)) {
			DEBUG(printf("Executing Task for MB%d : Type %u on %u\n", task_metadata_block->block_id, task_metadata_block->job_type, task_metadata_block->accelerator_type));
			scheduler_execute_task_function[task_metadata_block->job_type][task_metadata_block->accelerator_type](task_metadata_block);
		} else {
			printf("ERROR : execute_task_on_accelerator called for unknown task type: %u\n", task_metadata_block->job_type);
			cleanup_and_exit(-13);
		}
	} else {
		printf("ERROR -- called execute_task_on_accelerator for NO_ACCELERATOR_T with block:\n");
		print_base_metadata_block_contents(task_metadata_block);
		cleanup_and_exit(-11);
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



status_t initialize_scheduler()
{
	DEBUG(printf("In initialize...\n"));
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
	for (int i = 0; i < NUM_ACCEL_TYPES-1; i++) {
		if (MAX_ACCEL_OF_EACH_TYPE < input_accel_limit[i]) {
			printf("INIT-SCHED: ERROR : MAX_ACCEL_OF_EACH_TYPE < input_accel_limit[%u] : %u < %u\n", i, MAX_ACCEL_OF_EACH_TYPE, input_accel_limit[i]);
			parms_error = 1;
		}
		if (NUM_CPU_ACCEL < input_accel_limit[i]) {
			printf("INIT-SCHED: ERROR : NUM_CPU_ACCEL < input_accel_limit[%u] : %u < %u\n", i, NUM_CPU_ACCEL, input_accel_limit[i]);
			parms_error = 1;
		}
	}
	if (parms_error > 0) {
		printf("... Exiting run...\n");
		exit(-16);
	}
	pthread_mutex_init(&free_metadata_mutex, NULL);
	pthread_mutex_init(&accel_alloc_mutex, NULL);
	pthread_mutex_init(&task_queue_mutex, NULL);
	pthread_mutex_init(&schedule_from_queue_mutex, NULL);

	struct timeval init_time;
	gettimeofday(&init_time, NULL);
	last_accel_use_update_time = init_time; // Start accounting at init time... ?
	for (int i = 0; i < total_metadata_pool_blocks; i++) {
		master_metadata_pool[i].block_id = i; // Set the master pool's block_ids
		for (int ji = 0; ji < NUM_JOB_TYPES; ji++) {
			master_metadata_pool[i].gets_by_type[ji] = 0;
			master_metadata_pool[i].frees_by_type[ji] = 0;
		}
		// Clear the (full-run, aggregate) timing data spaces
		gettimeofday( &(master_metadata_pool[i].sched_timings.idle_start), NULL);
		// Scheduler timings 
		master_metadata_pool[i].sched_timings.idle_sec = 0;
		master_metadata_pool[i].sched_timings.idle_usec = 0;
		master_metadata_pool[i].sched_timings.get_sec = 0;
		master_metadata_pool[i].sched_timings.get_usec = 0;
		master_metadata_pool[i].sched_timings.queued_sec = 0;
		master_metadata_pool[i].sched_timings.queued_usec = 0;
		for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
			master_metadata_pool[i].sched_timings.running_sec[ti] = 0;
			master_metadata_pool[i].sched_timings.running_usec[ti] = 0;
		}
		master_metadata_pool[i].sched_timings.done_sec = 0;
		master_metadata_pool[i].sched_timings.done_usec = 0;
		// Reset all the per-task type and targets timing data, too.
		for (int ti = 0; ti < MAX_TASK_TYPES; ti++) {
			for (int tii = 0; tii < MAX_TASK_TARGETS; tii++) {
				master_metadata_pool[i].task_timings[ti].comp_by[tii] = 0;
				master_metadata_pool[i].task_timings[ti].time_sec[tii] = 0;
				master_metadata_pool[i].task_timings[ti].time_usec[tii] = 0;
			}
		}

		pthread_mutex_init(&(master_metadata_pool[i].metadata_mutex), NULL);
		pthread_cond_init(&(master_metadata_pool[i].metadata_condv), NULL);

		free_metadata_pool[i] = i;    // Set up all blocks are free
		free_critlist_pool[i] = i;    // Set up all critlist blocks are free
	}

	// Now set up the task ready queue
	DEBUG(printf("Setting up the task ready queue...\n"));
	for (int i = 0; i < total_metadata_pool_blocks; i++) {
		ready_mb_task_queue_pool[i].unique_id = i;
		ready_mb_task_queue_pool[i].block_id = -1; // id -1 = unassigned
		if (i > 0) { 
			ready_mb_task_queue_pool[i].prev = &(ready_mb_task_queue_pool[i-1]);
		} else {
			ready_mb_task_queue_pool[i].prev = NULL;
		}
		if (i < (total_metadata_pool_blocks - 1)) {
			ready_mb_task_queue_pool[i].next = &(ready_mb_task_queue_pool[i+1]);
		} else {
			ready_mb_task_queue_pool[i].next = NULL;
		}
		DEBUG(printf("  set pool[%2u] @ %p id %i prev %p next %p\n", i, &(ready_mb_task_queue_pool[i]), ready_mb_task_queue_pool[i].block_id, ready_mb_task_queue_pool[i].prev, ready_mb_task_queue_pool[i].next));
	}
	free_ready_mb_task_queue_entries = &(ready_mb_task_queue_pool[0]);
	ready_mb_task_queue_head = NULL;
	ready_mb_task_queue_tail = NULL;
	num_free_task_queue_entries = total_metadata_pool_blocks;
	DEBUG(printf(" AND free_ready_mb_task_queue_entries = %p\n", free_ready_mb_task_queue_entries));

	// Now initialize the per-metablock threads
	// For portability (as per POSIX documentation) explicitly create threads in joinable state
	pthread_attr_t  pt_attr;
	pthread_attr_init(&pt_attr);
	pthread_attr_setdetachstate(&pt_attr, PTHREAD_CREATE_JOINABLE);
	for (int i = 0; i < total_metadata_pool_blocks; i++) {
		if (pthread_create(&metadata_threads[i], &pt_attr, metadata_thread_wait_for_task, &(master_metadata_pool[i]))) {
			printf("ERROR: Scheduler failed to create thread for metadata block: %d\n", i);
			cleanup_and_exit(-10);
		}
		master_metadata_pool[i].thread_id = metadata_threads[i];
	}

	for (int ti = 0; ti < NUM_JOB_TYPES; ti++) {
		allocated_metadata_blocks[ti] = 0;
		freed_metadata_blocks[ti] = 0;
	}

	// These are brought in at compile time via config parameters
	num_accelerators_of_type[cpu_accel_t]        = input_accel_limit[cpu_accel_t];
	num_accelerators_of_type[fft_hwr_accel_t]    = input_accel_limit[fft_hwr_accel_t];
	num_accelerators_of_type[vit_hwr_accel_t]    = input_accel_limit[vit_hwr_accel_t];
	num_accelerators_of_type[cv_hwr_accel_t]     = input_accel_limit[cv_hwr_accel_t];

	for (int i = 0; i < NUM_ACCEL_TYPES-1; i++) {
		for (int j = 0; j < MAX_ACCEL_OF_EACH_TYPE; j++) {
			accelerator_in_use_by[i][j] = -1; // NOT a valid metadata block ID; -1 indicates "Not in Use"
		}
	}

	for (int i = 0; i < NUM_CPU_ACCEL+1; i++) {
		for (int j = 0; j < NUM_FFT_ACCEL+1; j++) {
			for (int k = 0; k < NUM_VIT_ACCEL+1; k++) {
				for (int l = 0; l < NUM_CV_ACCEL+1; l++) {
					in_use_accel_times_array[i][j][k][l] = 0;
				}
			}
		}
	}

	do_task_type_initialization();

	// And some stats stuff:
	for (int ti = 0; ti < NUM_ACCEL_TYPES-1; ti++) {
		for (int ai = 0; ai < MAX_ACCEL_OF_EACH_TYPE; ai++) {
			for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
				accelerator_allocated_to_MB[ti][ai][bi] = 0;
			}
		}
	}

	// Set up the Scheduler's Execution-Task-Function Table (for now by hand)
	for (int i = 0; i < NUM_JOB_TYPES; i++) {
		for (int j = 0; j < NUM_ACCEL_TYPES; j++) {
			scheduler_execute_task_function[i][j] = NULL; // Set all to default to NULL
		}
	}
	// Now set up those that "make sense"
	scheduler_execute_task_function[FFT_TASK][cpu_accel_t]     = &execute_cpu_fft_accelerator;
	scheduler_execute_task_function[FFT_TASK][fft_hwr_accel_t] = &execute_hwr_fft_accelerator;

	scheduler_execute_task_function[VITERBI_TASK][cpu_accel_t]     = &execute_cpu_viterbi_accelerator;
	scheduler_execute_task_function[VITERBI_TASK][vit_hwr_accel_t] = &execute_hwr_viterbi_accelerator;

	scheduler_execute_task_function[CV_TASK][cpu_accel_t]    = &execute_cpu_cv_accelerator;
	scheduler_execute_task_function[CV_TASK][cv_hwr_accel_t] = &execute_hwr_cv_accelerator;

	// Now start the "schedule_executions_from_queue() pthread -- using the DETACHED pt_attr
	int pt_ret = pthread_create(&scheduling_thread, &pt_attr, schedule_executions_from_queue, NULL);
	if (pt_ret != 0) {
		printf("Could not start the scheduler pthread... return value %d\n", pt_ret);
		cleanup_and_exit(-1);
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
	unsigned mdb_id     = task_metadata_block->block_id;
	unsigned accel_type = task_metadata_block->accelerator_type;
	unsigned accel_id   = task_metadata_block->accelerator_id;
	pthread_mutex_lock(&accel_alloc_mutex);

	//printf("MB%u RELEASE  accelerator %u %u for %d cl %u\n", mdb_id, accel_type, accel_id, accelerator_in_use_by[accel_type][accel_id], task_metadata_block->crit_level);
	DEBUG(printf(" RELEASE accelerator %u  %u  = %d  : ", accel_type, accel_id, accelerator_in_use_by[accel_type][accel_id]);
		for (int ai = 0; ai < num_accelerators_of_type[fft_hwr_accel_t]; ai++) {
			printf("%u %d : ", ai, accelerator_in_use_by[accel_type][ai]);
		}
		printf("\n"));
	if (accelerator_in_use_by[accel_type][accel_id] != mdb_id) {
		printf("ERROR - in release_accelerator_for_task for ACCEL %s Num %d but BLOCK_ID Mismatch: %d vs %d\n", accel_type_str[accel_type], accel_id, accelerator_in_use_by[accel_type][accel_id], mdb_id);
		printf("  this occurred on finish of block:\n");
		print_base_metadata_block_contents(task_metadata_block);
		printf("Accelerators Info:\n");
		for (int ai = 0; ai < num_accelerators_of_type[accel_type]; ai++) {
			printf(" accelerator_in_use_by[ %u ][ %u ] = %d\n", accel_type, ai, accelerator_in_use_by[accel_type][ai]);
		}
	} else {
		account_accelerators_in_use_interval();
		accelerator_in_use_by[accel_type][accel_id] = -1; // Indicates "Not in Use"
	}
	pthread_mutex_unlock(&accel_alloc_mutex);
}




unsigned HW_THRESHOLD[NUM_JOB_TYPES][NUM_ACCEL_TYPES-1] = { {101, 101, 101, 101},   // NO_JOB : 0% chance of using any HWR
#ifdef HW_FFT
							    {101,  25, 101, 101},   // FFT : 75% chance on HWR on FFT_HWR
#else
							    {101, 101, 101, 101},   // NO_HWR_FFT : 0% chance of using any HWR
#endif
#ifdef HW_FFT
							    {101, 101,  25, 101},   // VIT : 75% chance on HWR on VIT_HWR
#else
							    {101, 101, 101, 101},   // NO_HWR_VIT : 0% chance of using any HWR
#endif
#if (defined(HW_CV) || defined(FAKE_HW_CV))
							    {101, 101, 101,  25} }; // CV  : 75% chance on HWR on CV_HWR
#else
							    {101, 101, 101, 101} }; // NO_HWR_CV : 0% chance of using any HWR
#endif

// This is a basic accelerator selection policy:
//   This one selects an accelerator type (HWR or CPU) randomly
//   If an accelerators of that type is not available, it waits until it is.

ready_mb_task_queue_entry_t*
pick_accel_and_wait_for_available(ready_mb_task_queue_entry_t* ready_task_entry)
{
	//TODO: Make function to get task block from head of ready queue
	//Choose head of ready queue to be scheduled
	ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
	task_metadata_block_t * task_metadata_block = NULL;
	if (selected_task_entry != NULL) {
		task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
	}
	if (selected_task_entry == NULL) {
		printf("Ready queue empty\n");
	}
	if (task_metadata_block == NULL) {
		printf("ERROR : First Ready Task Queue entry is NULL?\n");
		pthread_mutex_unlock(&schedule_from_queue_mutex);
		cleanup_and_exit(-19);
	}
  
	DEBUG(printf("THE-SCHED: In pick_accel_and_wait_for_available policy for MB%u\n", task_metadata_block->block_id));
#ifdef INT_TIME
	struct timeval current_time;
	gettimeofday(&current_time, NULL);
#endif
	int proposed_accel = cpu_accel_t; //no_accelerator_t;
	int accel_type     = no_accelerator_t;
	int accel_id       = -1;
	if (task_metadata_block->job_type > 0) {
		// Scheduler should now run this either on CPU or HWR
		int num = (rand() % (100)); // Return a value from [0,99]
		for (int i = 1; i < NUM_ACCEL_TYPES-1; i++) {
			if (num >= HW_THRESHOLD[task_metadata_block->job_type][i]) {
				// Execute on hardware
				proposed_accel = i; // hwr_accel_t;
			}
			scheduler_decision_checks++;
		}
	} else {
		printf("ERROR : pick_accel_and_wait_for_available called for unknown task type: %u\n", task_metadata_block->job_type);
		cleanup_and_exit(-15);
	}
	// Okay, here we should have a good task to schedule...
	// Creating a "busy spin loop" where we constantly try to allocate
	//  This metablock to an accelerator, until one gets free...
#ifdef INT_TIME
	struct timeval decis_time;
	gettimeofday(&decis_time, NULL);
	scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
#endif
	scheduler_decisions++;
	do {
		int i = 0;
		while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) {
			if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available
				accel_type = proposed_accel;
				accel_id = i;
			}
			i++;
		}
	} while (accel_type == no_accelerator_t);
	task_metadata_block->accelerator_type = accel_type;
	task_metadata_block->accelerator_id = accel_id;

	return selected_task_entry;
}


// This is a basic accelerator selection policy:
//   This one scans through all the potential accelerators, and if the accelerator can
//    execute this type of job, AND the proposed accelerator is FASTER than any
//    prior (selected) accelerator, then it prefers that accelerator.
//   The seeking for an accelerator repeats until an eligible is found.
// This is "blocking" in that it spins until this task is allocated to an accelerator.
ready_mb_task_queue_entry_t*
fastest_to_slowest_first_available(ready_mb_task_queue_entry_t* ready_task_entry)
{
	//TODO: Make function to get task block from head of ready queue
	//Choose head of ready queue to be scheduled
	ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
	task_metadata_block_t * task_metadata_block = NULL;
	if (selected_task_entry != NULL) {
		task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
	}
	if (task_metadata_block == NULL) {
		printf("ERROR : First Ready Task Queue entry is NULL?\n");
		pthread_mutex_unlock(&schedule_from_queue_mutex);
		cleanup_and_exit(-19);
	}
	DEBUG(printf("THE-SCHED: In fastest_to_slowest_first_available policy for MB%u\n", task_metadata_block->block_id));
#ifdef INT_TIME
	struct timeval current_time;
	gettimeofday(&current_time, NULL);
#endif
	int proposed_accel = no_accelerator_t;
	int accel_type     = no_accelerator_t;
	int accel_id       = -1;
	uint64_t prop_time = ACINFPROF;
	if (task_metadata_block->job_type != NO_TASK_JOB) {
		do { // We will spin (in this policy) until we find an available accelerator...
			// Find an acceptable accelerator for this task (job_type)
			for (int check_accel = NUM_ACCEL_TYPES-2; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
				DEBUG(printf("F2S_FA: job %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->job_type, task_job_str[task_metadata_block->job_type], check_accel, accel_type_str[check_accel], scheduler_execute_task_function[task_metadata_block->job_type][check_accel]));
				if (scheduler_execute_task_function[task_metadata_block->job_type][check_accel] != NULL) {
					DEBUG(printf("F2S_FA: job %u check_accel = %u Tprof 0x%016llx prop_time 0x%016llx : %u\n", task_metadata_block->job_type, check_accel, task_metadata_block->task_profile[check_accel], prop_time, (task_metadata_block->task_profile[check_accel] < prop_time)));
					if (task_metadata_block->task_profile[check_accel] < prop_time) {
						int i = 0;
						DEBUG(printf("F2S_FA:  Checking from i = %u : num_acc = %u\n", i, num_accelerators_of_type[check_accel]));
						while ((i < num_accelerators_of_type[check_accel]) && (accel_id < 0)) {
							DEBUG(printf("F2S_FA:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, accel_type_str[check_accel], check_accel, i, accelerator_in_use_by[check_accel][i]));
							if (accelerator_in_use_by[check_accel][i] == -1) { // Not in use -- available
								proposed_accel = check_accel;
								accel_type = proposed_accel;
								accel_id = i;
								prop_time = task_metadata_block->task_profile[proposed_accel];
								DEBUG(printf("F2S_FA:   SELECT: prop_acc %u acc_ty %u acc_id %u prop_time %lu\n", proposed_accel, accel_type, accel_id, prop_time));
							}
							i++;
							scheduler_decision_checks += i;
						}
					} // if (accelerator is currently available)
				} // if (accelerator can execute this job_type)
			} // for (int check_accel = ...
		} while(accel_type == no_accelerator_t);
	} else {
		printf("ERROR : fastest_to_slowest_first_available called for unknown task type: %u\n", task_metadata_block->job_type);
		cleanup_and_exit(-15);
	}

#ifdef INT_TIME
	struct timeval decis_time;
	gettimeofday(&decis_time, NULL);
	scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
#endif
	scheduler_decisions++;
	// Okay, here we should have a good task to schedule...
	// Creating a "busy spin loop" where we constantly try to allocate
	//  This metablock to an accelerator, until one gets free...
	/* do { */
	/* 	int i = 0; */
	/* 	while ((i < num_accelerators_of_type[proposed_accel]) && (accel_id < 0)) { */
	/* 		if (accelerator_in_use_by[proposed_accel][i] == -1) { // Not in use -- available */
	/* 			accel_type = proposed_accel; */
	/* 			accel_id = i; */
	/* 		} */
	/* 		i++; */
	/* 	} */
	/* } while (accel_type == no_accelerator_t); */
	task_metadata_block->accelerator_type = accel_type;
	task_metadata_block->accelerator_id = accel_id;
	printf("F2SFA : task %u accel_ty %u accel %u\n", task_metadata_block->job_type, accel_type, accel_id);
	return selected_task_entry;
}



// This is an accelerator selection policy that prefers the accelerator target that results in earliest projected finish time.
//   This one scans through all the potential accelerators, and if the accelerator can
//    execute this type of job, AND the proposed accelerator's finish time is earlier than any
//    prior (selected) accelerator, then it prefers that accelerator.
////   The seeking for an accelerator repeats until an eligible is found.
//// This is "blocking" in that it spins until this task is allocated to an accelerator.
/* #undef DEBUG */
/* #define DEBUG(x) x */

ready_mb_task_queue_entry_t*
fastest_finish_time_first(ready_mb_task_queue_entry_t* ready_task_entry)
{
	//TODO: Make function to get task block from head of ready queue
	//Choose head of ready queue to be scheduled
	ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
	task_metadata_block_t * task_metadata_block = NULL;
	if (selected_task_entry != NULL) {
		task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
	}
	if (task_metadata_block == NULL) {
		printf("ERROR : First Ready Task Queue entry is NULL?\n");
		pthread_mutex_unlock(&schedule_from_queue_mutex);
		cleanup_and_exit(-19);
	}
	DEBUG(printf("THE-SCHED: In fastest_to_slowest_first_available policy for MB%u : job %u %s \n", task_metadata_block->block_id, task_metadata_block->job_type, task_job_str[task_metadata_block->job_type]));
#ifdef INT_TIME
	struct timeval current_time;
	gettimeofday(&current_time, NULL);
#endif
	int proposed_accel = no_accelerator_t;
	int accel_type     = no_accelerator_t;
	int accel_id       = -1;
	uint64_t proj_finish_time = ACINFPROF;
	if (task_metadata_block->job_type != NO_TASK_JOB) {
		// Find an acceptable accelerator for this task (job_type)
		for (int check_accel = NUM_ACCEL_TYPES-2; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
			DEBUG(printf("SCHED_FF: job %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->job_type, task_job_str[task_metadata_block->job_type], check_accel, accel_type_str[check_accel], scheduler_execute_task_function[task_metadata_block->job_type][check_accel]));
			if (scheduler_execute_task_function[task_metadata_block->job_type][check_accel] != NULL) {
				DEBUG(printf("SCHED_FF: job %u check_accel = %u Tprof 0x%016llx proj_finish_time 0x%016llx : %u\n", task_metadata_block->job_type, check_accel, task_metadata_block->task_profile[check_accel], proj_finish_time, (task_metadata_block->task_profile[check_accel] < proj_finish_time)));
				//if (task_metadata_block->task_profile[check_accel] < proj_finish_time)
				{
					uint64_t new_proj_finish_time;
					int i = 0;
					DEBUG(printf("SCHED_FF:  Checking from i = %u : num_acc = %u\n", i, num_accelerators_of_type[check_accel]));
					while ((i < num_accelerators_of_type[check_accel])) { // && (accel_id < 0)) {
						DEBUG(printf("SCHED_FF:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, accel_type_str[check_accel], check_accel, i, accelerator_in_use_by[check_accel][i]));
						int bi = accelerator_in_use_by[check_accel][i];
						if (bi == -1) { // Not in use -- available
							new_proj_finish_time = task_metadata_block->task_profile[check_accel];
							DEBUG(printf("SCHED_FFF:     So AcTy %u Acc %u projected finish_time = %lu\n", check_accel, i, new_proj_finish_time));
						} else {
							// Compute the remaining execution time (estimate) for job currently on accelerator
							uint64_t elapsed_sec = current_time.tv_sec - master_metadata_pool[bi].sched_timings.running_start.tv_sec;
							uint64_t elapsed_usec = current_time.tv_usec - master_metadata_pool[bi].sched_timings.running_start.tv_usec;
							uint64_t total_elapsed_usec = elapsed_sec*1000000 + elapsed_usec;
							uint64_t remaining_time = master_metadata_pool[bi].task_profile[check_accel] - total_elapsed_usec;
							// and add that to the projected task run time to get the estimated finish time.
							new_proj_finish_time = task_metadata_block->task_profile[check_accel] + remaining_time;
							DEBUG(printf("SCHED_FFF:     So AcTy %u Acc %u projected finish_time = %lu = %lu + %lu\n", check_accel, i, new_proj_finish_time, task_metadata_block->task_profile[check_accel], remaining_time));
						}
						if (new_proj_finish_time < proj_finish_time) {
							proposed_accel = check_accel;
							accel_type = proposed_accel;
							accel_id = i;
							proj_finish_time = new_proj_finish_time;
							DEBUG(printf("SCHED_FF:   SELECT: prop_acc %u acc_ty %u acc_id %u proj_finish_time %lu\n", proposed_accel, accel_type, accel_id, proj_finish_time));
						}
						i++;
						scheduler_decision_checks += i;
					}
				} //
			} // if (accelerator can execute this job_type)
		} // for (int check_accel = ...
	} else {
		printf("ERROR : fastest_to_slowest_first_available called for unknown task type: %u\n", task_metadata_block->job_type);
		cleanup_and_exit(-15);
	}

#ifdef INT_TIME
	struct timeval decis_time;
	gettimeofday(&decis_time, NULL);
	scheduler_decision_time_usec += 1000000*(decis_time.tv_sec - current_time.tv_sec) + (decis_time.tv_usec - current_time.tv_usec);
#endif
	scheduler_decisions++;
	// Okay, here we should have selected a target accelerator
	// Creating a "busy spin loop" where we constantly try to allocate
	//  This metablock to an accelerator, until one gets free...
	unsigned wait_iter = 0;
	while (accelerator_in_use_by[accel_type][accel_id] != -1) { // Not in use -- available
		wait_iter++;
	}
	task_metadata_block->accelerator_type = accel_type;
	task_metadata_block->accelerator_id = accel_id;
	DEBUG(printf("FFTFP : task %u accel_ty %u accel %u : wait_iter %u\n", task_metadata_block->job_type, accel_type, accel_id, wait_iter));

	return selected_task_entry;
}
/* #undef DEBUG */
/* #define DEBUG(x) */

ready_mb_task_queue_entry_t *
fastest_finish_time_first_queued(ready_mb_task_queue_entry_t* ready_task_entry)
{
	//Choose task out of order to be scheduled based on least finish time and available accelerator
	ready_mb_task_queue_entry_t* selected_task_entry = ready_task_entry;
	task_metadata_block_t * task_metadata_block = NULL;
	for (int i = 0; i < num_tasks_in_ready_queue; ++i)
	{
		task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
		if (task_metadata_block == NULL) {
			printf("SCHED-FFFQ: ERROR : Ready Task Queue entry %u is NULL even though num_tasks_in_ready_queue = %d depicts otherwise?\n", i, num_tasks_in_ready_queue);
			pthread_mutex_unlock(&schedule_from_queue_mutex);
			cleanup_and_exit(-19);
		}
		if (task_metadata_block->job_type == NO_TASK_JOB) {
			printf("SCHED-FFFQ: ERROR : Ready Task Queue entry %u Job Type is NO_TASK_JOB?\n", i);
			pthread_mutex_unlock(&schedule_from_queue_mutex);
			cleanup_and_exit(-20);
		}

		DEBUG(printf("SCHED-FFFQ: In fastest_finish_time_first_queued for Entry %u : MB%d Task %s\n", i, task_metadata_block->block_id, task_job_str[task_metadata_block->job_type]));
               #ifdef INT_TIME
		struct timeval current_time;
		gettimeofday(&current_time, NULL);
               #endif

		int accel_type     = task_metadata_block->accelerator_type; //no_accelerator_t;
		int accel_id       = task_metadata_block->accelerator_id;   //-1;
		//if (task_metadata_block->accelerator_type == no_accelerator_t || task_metadata_block->accelerator_id == -1) {
		if ((accel_type == no_accelerator_t) || (accel_id == -1)) {
			DEBUG(printf("FFFQ: In fastest_finish_time_first_queued policy for MB%u\n", task_metadata_block->block_id));
			uint64_t proj_finish_time = ACINFPROF;

			// Find an acceptable accelerator for this task (job_type)
			for (int check_accel = NUM_ACCEL_TYPES-2; check_accel >= 0; check_accel--) { // Last accel is "no-accelerator"
				DEBUG(printf("SCHED-FFFQ: job %u %s : check_accel = %u %s : SchedFunc %p\n", task_metadata_block->job_type, task_job_str[task_metadata_block->job_type], check_accel, accel_type_str[check_accel], scheduler_execute_task_function[task_metadata_block->job_type][check_accel]));
				if (scheduler_execute_task_function[task_metadata_block->job_type][check_accel] != NULL) {
					DEBUG(printf("SCHED-FFFQ: job %u check_accel = %u Tprof 0x%016llx proj_finish_time 0x%016llx : %u\n", task_metadata_block->job_type, check_accel, task_metadata_block->task_profile[check_accel], proj_finish_time, (task_metadata_block->task_profile[check_accel] < proj_finish_time)));
					uint64_t new_proj_finish_time;
					int i = 0;
					DEBUG(printf("SCHED-FFFQ:  Checking from i = %u : num_acc = %u\n", i, num_accelerators_of_type[check_accel]));
					while ((i < num_accelerators_of_type[check_accel])) { // && (accel_id < 0)) {
						DEBUG(printf("SCHED-FFFQ:  Checking i = %u %s : acc_in_use[%u][%u] = %d\n", i, accel_type_str[check_accel], check_accel, i, accelerator_in_use_by[check_accel][i]));
						int bi = accelerator_in_use_by[check_accel][i];
						if (bi == -1) { // Not in use -- available
							new_proj_finish_time = task_metadata_block->task_profile[check_accel];
							DEBUG(printf("SCHED-FFFQ:     So AcTy %u Acc %u projected finish_time = %lu\n", check_accel, i, new_proj_finish_time));
						} else {
							// Compute the remaining execution time (estimate) for job currently on accelerator
							uint64_t elapsed_sec = current_time.tv_sec - master_metadata_pool[bi].sched_timings.running_start.tv_sec;
							uint64_t elapsed_usec = current_time.tv_usec - master_metadata_pool[bi].sched_timings.running_start.tv_usec;
							uint64_t total_elapsed_usec = elapsed_sec*1000000 + elapsed_usec;
							uint64_t remaining_time = master_metadata_pool[bi].task_profile[check_accel] - total_elapsed_usec;
							// and add that to the projected task run time to get the estimated finish time.
							new_proj_finish_time = task_metadata_block->task_profile[check_accel] + remaining_time;
							DEBUG(printf("SCHED-FFFQ:     So AcTy %u Acc %u projected finish_time = %lu = %lu + %lu\n", check_accel, i, new_proj_finish_time, task_metadata_block->task_profile[check_accel], remaining_time));
						}
						if (new_proj_finish_time < proj_finish_time) {
							accel_type = check_accel;
							accel_id = i;
							proj_finish_time = new_proj_finish_time;
							DEBUG(printf("SCHED-FFFQ:   SELECT: acc_ty %u acc_id %u proj_finish_time %lu\n", accel_type, accel_id, proj_finish_time));
						}
						i++;
						scheduler_decision_checks += i;
					} // while (i < num_accelerators_of_type
				} // if (accelerator can execute this job_type)
			} // for (int check_accel = ...
			// At this point, we must have a "best" accelerator selected for this task
			scheduler_decisions++;
			if ((accel_type == no_accelerator_t) || (accel_id == -1)) {
				printf("SCHED-FFFQ: ERROR : Ready Task Queue entry %u Job Type %u %s couldn't find an accelerator: acc_ty %u id %d\n", i, task_metadata_block->job_type, task_job_str[task_metadata_block->job_type], accel_type, accel_id);
				pthread_mutex_unlock(&schedule_from_queue_mutex);
				cleanup_and_exit(-21);
			}
			//task_metadata_block->accelerator_type = accel_type;
			//task_metadata_block->accelerator_id   = accel_id;
		} // if (task not already assigned to an accelerator

		// Check if best accelerator is available
		//if (accelerator_in_use_by[task_metadata_block->accelerator_type][task_metadata_block->accelerator_id] == -1) {
		if (accelerator_in_use_by[accel_type][accel_id] == -1) {
			// Task is schedulable on the best accelerator
			DEBUG(printf("SCHED-FFFQ: Best accel type: %d id: accel_id: %d tid: %d\n", task_metadata_block->accelerator_type, task_metadata_block->accelerator_id, task_metadata_block->thread_id));
			task_metadata_block->accelerator_type = accel_type;
			task_metadata_block->accelerator_id   = accel_id;
			return selected_task_entry;
		}

		selected_task_entry = selected_task_entry->next;
	}
	// No task found that can be scheduled on its best accelerator
	return NULL;
}



// This routine selects an available accelerator for the given job, 
//  The accelerator is selected according to a policy
//  The policies are implemented in separate functions.
ready_mb_task_queue_entry_t *
select_task_and_target_accelerator(accel_select_policy_t policy, ready_mb_task_queue_entry_t* ready_task_entry)
{
	ready_mb_task_queue_entry_t* selected_task_entry = NULL;
	switch(policy) { 
	case SELECT_ACCEL_AND_WAIT_POLICY:
		selected_task_entry = pick_accel_and_wait_for_available(ready_task_entry);
		break;
	case FAST_TO_SLOW_FIRST_AVAIL_POLICY:
		selected_task_entry = fastest_to_slowest_first_available(ready_task_entry);
		break;
	case FASTEST_FINISH_TIME_FIRST_POLICY:
		selected_task_entry = fastest_finish_time_first(ready_task_entry);
		break;
	case FASTEST_FINISH_TIME_FIRST_QUEUED_POLICY:
		selected_task_entry = fastest_finish_time_first_queued(ready_task_entry);
		break;
	default:
		printf("ERROR : unknown scheduler accelerator selection policy: %u\n", policy);
		cleanup_and_exit(-15);
	}
	return selected_task_entry;
}



// This routine schedules (the first) ready task from the ready task queue
// The input parm is currently ignored -- there are no parms here.
// The output
void* schedule_executions_from_queue(void* void_parm_ptr) {
	DEBUG(printf("SCHED: starting execution of schedule_executions_from_queue thread...\n"));
	// This will now be an eternally-running scheduler process, I think.
	pthread_mutex_lock(&schedule_from_queue_mutex);
	while(1) {
		// If there is nothing on the queue -- return;
		if (num_tasks_in_ready_queue > 0) {
			DEBUG(printf("SCHED: num_tasks_in_ready_queue = %u\n", num_tasks_in_ready_queue);
				int ti = 0;
				ready_mb_task_queue_entry_t* t_te = ready_mb_task_queue_head;
				while(t_te != NULL) {
					printf("SCHED:    Ready_Task_Entry %2u : MB%u  %p %p\n", ti, t_te->block_id, t_te->prev, t_te->next);
					ti++;
					t_te = t_te->next;
				});

			// Get pointer to the first task on the ready queue
			ready_mb_task_queue_entry_t* ready_task_entry = ready_mb_task_queue_head;
			ready_mb_task_queue_entry_t* selected_task_entry = NULL;
			task_metadata_block_t* task_metadata_block = NULL;

			// Select the target accelerator to execute the task
			DEBUG(printf("SCHED: calling select_task_and_target_accelerator\n"));
			//Pass the head of the ready queue to parse entries in the queue
			selected_task_entry = select_task_and_target_accelerator(global_scheduler_selection_policy, ready_task_entry);
			if (selected_task_entry == NULL) {
				//No schedulable task
				continue;
			} else {
				task_metadata_block = &(master_metadata_pool[selected_task_entry->block_id]);
			}
			unsigned int accel_type = task_metadata_block->accelerator_type;
			unsigned int accel_id = task_metadata_block->accelerator_id;
			DEBUG(printf("SCHED: Selected accel type: %d id: accel_id: %d tid: %d\n", task_metadata_block->accelerator_type, task_metadata_block->accelerator_id, task_metadata_block->thread_id));

			if (accel_type == no_accelerator_t) {
				printf("SCHED: ERROR : Selected Task has no accelerator assigned\n");
				pthread_mutex_unlock(&schedule_from_queue_mutex);
				cleanup_and_exit(-19);
			}
			if (accel_type < no_accelerator_t) {
				// Mark the requested accelerator as "In-USE" by this metadata block
				if (accelerator_in_use_by[accel_type][accel_id] != -1) {
					printf("ERROR : schedule_executions_from_queue is trying to allocate ACCEL %s %u which is already allocated to Block %u\n", accel_type_str[accel_type], accel_id, accelerator_in_use_by[accel_type][accel_id]);
					cleanup_and_exit(-14);
				}
				account_accelerators_in_use_interval();
				int bi = task_metadata_block->block_id; // short name for the block_id
				accelerator_in_use_by[accel_type][accel_id] = bi;
				accelerator_allocated_to_MB[accel_type][accel_id][bi] += 1;
				// Okay -- we can allocate to the accelerator -- remove from the queue
				//printf("MB%u ALLOCATE accelerator %u %u to  %d cl %u\n", bi, accel_type, accel_id, bi, task_metadata_block->crit_level);
				DEBUG(printf("SCHED: MB%u ALLOC accelerator %u  %u to %d  : ", bi, accel_type, accel_id, bi);
					for (int ai = 0; ai < num_accelerators_of_type[fft_hwr_accel_t]; ai++) {
						printf("%u %d : ", ai, accelerator_in_use_by[accel_type][ai]);
					}
					printf("\n"));
				// Update the ready task queue... Connect selected_task_entry.prev->next = selected_task_entry.next

				DEBUG(printf("SCHED: Updating the task ready queue...\n"));
				pthread_mutex_lock(&task_queue_mutex); 

				if (selected_task_entry->prev == NULL) {
					// This was the HEAD of the ready queue
					ready_mb_task_queue_head = selected_task_entry->next;
					DEBUG(printf("SCHED:   Removed HEAD of queue\n"));
				} else {
					selected_task_entry->prev->next = selected_task_entry->next;
					DEBUG(printf("SCHED:   Removed internal entry of queue\n"));
				}
				if (selected_task_entry->next == NULL) {
					// This is the TAIL of the ready queue
					ready_mb_task_queue_tail = selected_task_entry->prev;
					DEBUG(printf("SCHED:   Removed TAIL of queue\n"));
				} else {
					selected_task_entry->next->prev = selected_task_entry->prev;
					DEBUG(printf("SCHED:   Removed internal entry of queue\n"));
				}
				num_tasks_in_ready_queue--;
				DEBUG(printf("SCHED:   Set num_tasks_in_ready_queue to %u\n", num_tasks_in_ready_queue));

				DEBUG(printf("SCHED:   Adding back the ready task entry to the free list pre: %u entries\n", num_free_task_queue_entries));
				// Prepend to the free_ready_mb_task_queue_entries;
				if (free_ready_mb_task_queue_entries != NULL) {
					// There are other free task queue entries
					/* if (free_ready_mb_task_queue_entries->next != NULL) { */
					/*   free_ready_mb_task_queue_entries->next->prev = ready_task_entry; */
					/* }  */
					free_ready_mb_task_queue_entries->prev = selected_task_entry;
					selected_task_entry->next = free_ready_mb_task_queue_entries;
				}
				selected_task_entry->prev = NULL; // As head of the list, the prev should be NULL
				free_ready_mb_task_queue_entries = selected_task_entry;  
				num_free_task_queue_entries++;
				DEBUG(printf("SCHED:   Prepended to FREE ready task queue, with %u entries now\n", num_free_task_queue_entries);
					print_free_ready_tasks_list());
				/* // And clean up the ready task storage... */
				/* ready_task_entry->block_id = -1; */
				/* ready_task_entry->next = NULL; */
				/* ready_task_entry->prev = NULL; */
				// And unlock the task-queue mutex
				pthread_mutex_unlock(&task_queue_mutex);

				// Set the task to "RUNNING" State and account the times...
				task_metadata_block->status = TASK_RUNNING; // running

				gettimeofday(&master_metadata_pool[bi].sched_timings.running_start, NULL);
				master_metadata_pool[bi].sched_timings.queued_sec += master_metadata_pool[bi].sched_timings.running_start.tv_sec - master_metadata_pool[bi].sched_timings.queued_start.tv_sec;
				master_metadata_pool[bi].sched_timings.queued_usec += master_metadata_pool[bi].sched_timings.running_start.tv_usec - master_metadata_pool[bi].sched_timings.queued_start.tv_usec;

				TDEBUG(printf("Kicking off accelerator task for Metadata Block %u : Task %s %s on Accel %s %u\n", bi, task_job_str[task_metadata_block->job_type], task_criticality_str[task_metadata_block->crit_level], accel_type_str[task_metadata_block->accelerator_type], task_metadata_block->accelerator_id));

				// Lock the mutex associated to the conditional variable
				pthread_mutex_lock(&(task_metadata_block->metadata_mutex));

				// Signal the conditional variable -- triggers the target thread execution of accelerator
				pthread_cond_signal(&(task_metadata_block->metadata_condv));

				// And now we unlock because we are done here...
				pthread_mutex_unlock(&(task_metadata_block->metadata_mutex));
			} else {
				printf("Cannot allocate execution resources for metadata block:\n");
				print_base_metadata_block_contents(task_metadata_block);
			}
		} else { // if (num_tasks_in_queue > 0)
			// I think perhaps we should add a short delay here to avoid this being such a busy spin loop...
			//   If we are using the 78MHz FPGA, then one clock cycle is ~12.82 ns ?
			usleep(scheduler_holdoff_usec); // This defaults to 1 usec (about 78 FPGA clock cycles)
		}
	} // while (1)
	pthread_mutex_unlock(&schedule_from_queue_mutex);
	return NULL;
}


void
request_execution(task_metadata_block_t* task_metadata_block)
{
	DEBUG(printf("APP: in request_execution for MB%u\n", task_metadata_block->block_id));
	task_metadata_block->status = TASK_QUEUED; // queued
	gettimeofday(&task_metadata_block->sched_timings.queued_start, NULL);
	task_metadata_block->sched_timings.get_sec += task_metadata_block->sched_timings.queued_start.tv_sec - task_metadata_block->sched_timings.get_start.tv_sec;
	task_metadata_block->sched_timings.get_usec += task_metadata_block->sched_timings.queued_start.tv_usec - task_metadata_block->sched_timings.get_start.tv_usec;

	// Put this into the ready-task-queue
	//   Get a ready_task_queue_entry
	pthread_mutex_lock(&task_queue_mutex);
	DEBUG(printf("APP: there are currently %u free task queue entries in the list\n", num_free_task_queue_entries);
		print_free_ready_tasks_list());
	ready_mb_task_queue_entry_t* my_queue_entry = free_ready_mb_task_queue_entries;
	free_ready_mb_task_queue_entries = free_ready_mb_task_queue_entries->next;
	free_ready_mb_task_queue_entries->prev = NULL; // disconnect the prev pointer
	num_free_task_queue_entries--;
	DEBUG(printf("APP: and now there are %u free task queue entries in the list\n", num_free_task_queue_entries);
		print_free_ready_tasks_list());
	//   Now fill it in
	my_queue_entry->block_id = task_metadata_block->block_id;
	DEBUG(printf("APP: got a free_task_ready_queue_entry, leaving %u free\n", num_free_task_queue_entries));
	//   And add to the tail of the task queue
	if (ready_mb_task_queue_head == NULL) {
		my_queue_entry->prev = NULL;
		my_queue_entry->next = NULL;
		ready_mb_task_queue_head = my_queue_entry;
		DEBUG(printf("APP: inserted this as the HEAD of the ready-task-queue\n"));
	} else {
		my_queue_entry->prev = ready_mb_task_queue_tail;
		my_queue_entry->next = NULL;
		ready_mb_task_queue_tail->next = my_queue_entry;
		DEBUG(printf("APP: inserted this as the TAIL of the ready-task-queue\n"));
	}
	ready_mb_task_queue_tail = my_queue_entry;
	num_tasks_in_ready_queue++;
	DEBUG(printf("APP: and now there are %u ready tasks in the queue\n", num_tasks_in_ready_queue);
		print_ready_tasks_queue(););
	pthread_mutex_unlock(&task_queue_mutex);

	// NOW we should return -- we can later schedule tasks off the queue...
	/** This was for single-thread testing; now trying multi-thread mode
	    schedule_executions_from_queue();
	**/
	return;
}



/********************************************************************************
 * Here are the wait routines -- for critical tasks or all tasks to finish
 ********************************************************************************/
void wait_all_critical()
{
	// Loop through the critical tasks list and check whether they are all in status "done"
	blockid_linked_list_t* cli = critical_live_task_head;
	while (cli != NULL) {
		if (master_metadata_pool[cli->clt_block_id].status != TASK_DONE) {
			// This task is not finished yet.. wait for it
			//  So start polling from the start of the list again.
			cli = critical_live_task_head;
		} else {
			cli = cli->next;
		}
	}
}

void wait_all_tasks_finish()
{
	// Spin loop : check whether all blocks are free...
	printf("Waiting for ALL tasks to finish: free = %u and total = %u\n", free_metadata_blocks, total_metadata_pool_blocks);
	while (free_metadata_blocks != total_metadata_pool_blocks) {
		; // Nothing really to do, but wait.
	}
}


// This cleans up the state (pthreads, etc.) before exit
void cleanup_state() {
	// Kill (cancel) the per-metablock threads
	for (int i = 0; i < total_metadata_pool_blocks; i++) {
		pthread_cancel(metadata_threads[i]);
	}
	pthread_cancel(scheduling_thread);
	// Clean out the pthread mutex and conditional variables
	pthread_mutex_destroy(&free_metadata_mutex);
	pthread_mutex_destroy(&accel_alloc_mutex);
	for (int i = 0; i < total_metadata_pool_blocks; i++) {
		pthread_mutex_destroy(&(master_metadata_pool[i].metadata_mutex));
		pthread_cond_destroy(&(master_metadata_pool[i].metadata_condv));
	}

	// Clean up any hardware accelerator stuff
	do_task_type_closeout();
}


// This is called at the end of run/life to shut down the scheduler
//  This will also output a bunch of stats abdout timings, etc.

void output_run_statistics()
{

	// NOW output some overall full-run statistics, etc.
	printf("\nOverall Accelerator allocation/usage statistics:\n");
	printf("\nTotal Scheduler Decision-Making Time was %lu usec for %u decisions spanning %u checks\n", scheduler_decision_time_usec, scheduler_decisions, scheduler_decision_checks);

	printf("\nScheduler block allocation/free statistics:\n");
	for (int ti = 0; ti < NUM_JOB_TYPES; ti++) {
		printf("  For %12s Scheduler allocated %9u blocks and freed %9u blocks\n", task_job_str[ti], allocated_metadata_blocks[ti], freed_metadata_blocks[ti]);
	}
	printf(" During FULL run,  Scheduler allocated %9u blocks and freed %9u blocks in total\n", allocated_metadata_blocks[NO_TASK_JOB], freed_metadata_blocks[NO_TASK_JOB]);

	printf("\nPer-MetaData-Block Scheduler Allocation/Frees by Job-Type Data:\n");
	printf("%6s ", "Block");
	for (int ji = 1; ji < NUM_JOB_TYPES; ji++) {
		printf("%12s_G %12s_F ", task_job_str[ji], task_job_str[ji]);
	}
	printf("\n");
	unsigned type_gets[NUM_JOB_TYPES];
	unsigned type_frees[NUM_JOB_TYPES];
	for (int ji = 0; ji < NUM_JOB_TYPES; ji++) {
		type_gets[ji] = 0;
		type_frees[ji] = 0;
	}
	for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
		printf("%6u ", bi);
		for (int ji = 1; ji < NUM_JOB_TYPES; ji++) {
			type_gets[ji]  += master_metadata_pool[bi].gets_by_type[ji];
			type_frees[ji] += master_metadata_pool[bi].frees_by_type[ji];
			printf("%14u %14u ", master_metadata_pool[bi].gets_by_type[ji], master_metadata_pool[bi].frees_by_type[ji]);
		}
		printf("\n");
	}
	printf("%6s ", "Total");
	for (int ji = 1; ji < NUM_JOB_TYPES; ji++) {
		printf("%14u %14u ", type_gets[ji], type_frees[ji]);
	}
	printf("\n");

	printf("\nPer-MetaData-Block Scheduler Timing Data:\n");
	{
		unsigned total_blocks_used  = 0;
		uint64_t total_idle_usec    = 0;
		uint64_t total_get_usec     = 0;
		uint64_t total_queued_usec  = 0;
		uint64_t total_running_usec[NUM_ACCEL_TYPES];
		for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
			total_running_usec[ti] = 0;
		}
		uint64_t total_done_usec    = 0;
		for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
			uint64_t this_idle_usec = (uint64_t)(master_metadata_pool[bi].sched_timings.idle_sec) * 1000000 + (uint64_t)(master_metadata_pool[bi].sched_timings.idle_usec);
			uint64_t this_get_usec = (uint64_t)(master_metadata_pool[bi].sched_timings.get_sec) * 1000000 + (uint64_t)(master_metadata_pool[bi].sched_timings.get_usec);
			uint64_t this_queued_usec = (uint64_t)(master_metadata_pool[bi].sched_timings.queued_sec) * 1000000 + (uint64_t)(master_metadata_pool[bi].sched_timings.queued_usec);
			uint64_t this_total_run_usec = 0;
			uint64_t this_running_usec[NUM_ACCEL_TYPES];
			for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
				this_running_usec[ti] = (uint64_t)(master_metadata_pool[bi].sched_timings.running_sec[ti]) * 1000000 + (uint64_t)(master_metadata_pool[bi].sched_timings.running_usec[ti]);
				this_total_run_usec += this_running_usec[ti];
			}
			uint64_t this_done_usec = (uint64_t)(master_metadata_pool[bi].sched_timings.done_sec) * 1000000 + (uint64_t)(master_metadata_pool[bi].sched_timings.done_usec);
			printf(" Block %3u : IDLE %15lu GET %15lu QUE %15lu RUN %15lu DONE %15lu usec :", bi, this_idle_usec, this_get_usec, this_queued_usec, this_total_run_usec,  total_done_usec);
			for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
				printf(" %15lu", this_running_usec[ti]);
			}
			printf("\n");
			if (this_idle_usec != 0) { 
				total_blocks_used++; 
			}
			total_idle_usec    += this_idle_usec;
			total_get_usec     += this_get_usec;
			total_queued_usec  += this_queued_usec;
			for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
				total_running_usec[ti] += this_running_usec[ti];
			}
			total_done_usec    += this_done_usec;
		}
		double avg;
		printf("\nScheduler Timings: Aggregate Across all Metadata Blocks with %u used blocks\n", total_blocks_used);
		avg = (double)total_idle_usec/(double)total_blocks_used;
		printf("  Metablocks_IDLE total run time:    %15lu usec : %16.2lf (average)\n", total_idle_usec, avg);
		avg = (double)total_get_usec/(double)total_blocks_used;
		printf("  Metablocks_GET total run time:     %15lu usec : %16.2lf (average)\n", total_get_usec, avg);
		avg = (double)total_queued_usec/(double)total_blocks_used;
		printf("  Metablocks_QUEUED total run time:  %15lu usec : %16.2lf (average)\n", total_queued_usec, avg);
		for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
			avg = (double)total_running_usec[ti]/(double)total_blocks_used;
			printf("  Metablocks_RUNNING total %u %s run time: %15lu usec : %16.2lf (average)\n", ti, accel_type_str[ti], total_running_usec[ti], avg);
		}
		avg = (double)total_done_usec/(double)total_blocks_used;
		printf("  Metablocks_DONE total run time:    %15lu usec : %16.2lf (average)\n", total_done_usec, avg);
	}

	output_task_type_run_stats();
    
	printf("\nACU_HIST: Aggregated In-Use Accelerator Time Histogram...\n");
	{
		printf("ACU_HIST:  CPU  FFT SFFT  VIT SVIT  CNN SCNN : TACC TFFT TVIT TCNN : Time-in-usec\n");
		for (int i0 = 0; i0 <= num_accelerators_of_type[0]; i0++) {
			for (int i1 = 0; i1 <= num_accelerators_of_type[1]; i1++) {
				for (int i2 = 0; i2 <= num_accelerators_of_type[2]; i2++) {
					for (int i3 = 0; i3 <= num_accelerators_of_type[3]; i3++) {
						printf("ACU_HIST: %4u %4u %4u %4u : %4u : %lu\n", i0, i1, i2, i3, (i1+i2+i3), in_use_accel_times_array[i0][i1][i2][i3]);
					}
				}
			}
		}
	}

	printf("\nAccelerator Usage Statistics:\n");
	{
		unsigned totals[NUM_ACCEL_TYPES-1][MAX_ACCEL_OF_EACH_TYPE];
		unsigned top_totals[NUM_ACCEL_TYPES-1];
		for (int ti = 0; ti < NUM_ACCEL_TYPES-1; ti++) {
			top_totals[ti] = 0;
			for (int ai = 0; ai < MAX_ACCEL_OF_EACH_TYPE; ai++) {
				totals[ti][ai] = 0;
				for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
					totals[ti][ai] += accelerator_allocated_to_MB[ti][ai][bi];
				}
			}
		}
		printf("\nPer-Accelerator allocation/usage statistics:\n");
		for (int ti = 0; ti < NUM_ACCEL_TYPES-1; ti++) {
			for (int ai = 0; ai < MAX_ACCEL_OF_EACH_TYPE; ai++) {
				if (ai < num_accelerators_of_type[ti]) { 
					printf(" Acc_Type %u %s : Accel %2u Allocated %6u times\n", ti, accel_type_str[ti], ai, totals[ti][ai]);
				} else {
					if (totals[ti][ai] != 0) {
						printf("ERROR : We have use of non-existent Accelerator %u %s : index %u = %u\n", ti, accel_type_str[ti], ai, totals[ti][ai]);
					}
				}
				top_totals[ti]+= totals[ti][ai];
			}
		}
		printf("\nPer-Accelerator-Type allocation/usage statistics:\n");
		for (int ti = 0; ti < NUM_ACCEL_TYPES-1; ti++) {
			printf(" Acc_Type %u %s Allocated %6u times\n", ti, accel_type_str[ti], top_totals[ti]);
		}
		printf("\nPer-Meta-Block Accelerator allocation/usage statistics:\n");
		for (int ti = 0; ti < NUM_ACCEL_TYPES-1; ti++) {
			for (int ai = 0; ai < num_accelerators_of_type[ti]; ai++) {
				for (int bi = 0; bi < total_metadata_pool_blocks; bi++) {
					if (accelerator_allocated_to_MB[ti][ai][bi] != 0) {
						printf(" Per-MB Acc_Type %u %s : Accel %2u Allocated %6u times for MB%u\n", ti, accel_type_str[ti], ai, accelerator_allocated_to_MB[ti][ai][bi], bi);
					}
				}
			}
		}
	}
}

void shutdown_scheduler()
{
	output_run_statistics();
	cleanup_state();
}



void cleanup_and_exit(int rval) {
	cleanup_state();
	exit (rval);
}




void dump_all_metadata_blocks_states()
{
	//int free_metadata_pool[total_metadata_pool_blocks];
	//int free_metadata_blocks = total_metadata_pool_blocks;
	if (free_metadata_blocks == 0) {
		printf("FREE_MBS: { }\n");
	} else {
		for (int i = 0; i < free_metadata_blocks; i++) {
			if (i > 0) { printf(","); };
			printf("%u", free_metadata_pool[i]);
		}
		printf("\n");
	}
	//unsigned allocated_metadata_blocks[NUM_JOB_TYPES];
	printf("Total Allocated MBs:  ");
	for (int i = 0; i < NUM_JOB_TYPES; i++) {
		printf("( %s, %u ) ", task_job_str[i], allocated_metadata_blocks[i]);
	}
	printf("\n");
	//unsigned freed_metadata_blocks[NUM_JOB_TYPES];
	printf("Total Freed MBs:  ");
	for (int i = 0; i < NUM_JOB_TYPES; i++) {
		printf("( %s, %u ) ", task_job_str[i], freed_metadata_blocks[i]);
	}
	printf("\n");
	printf("\nData for EACH MB:\n");
	for (int mbi = 0; mbi < total_metadata_pool_blocks; mbi++) {
		printf("MB%u : Status %u %s\n", mbi, master_metadata_pool[mbi].status, task_status_str[master_metadata_pool[mbi].status]);
		printf("  MB%u : Acc_ty %u   Acc_id %d   Job %u   Crit_Lvl %u\n", mbi, 
			master_metadata_pool[mbi].accelerator_type, master_metadata_pool[mbi].accelerator_id, 
			master_metadata_pool[mbi].job_type, //task_job_str[master_metadata_pool[mbi].job_type],
			master_metadata_pool[mbi].crit_level);
		printf("  MB%u GETS:  ", mbi);
		for (int i = 0; i < NUM_JOB_TYPES; i++) {
			printf("( %s, %u ) ", task_job_str[i], master_metadata_pool[mbi].gets_by_type[i]);
		}
		printf("\n");
		printf("  MB%u FREES:  ", mbi);
		for (int i = 0; i < NUM_JOB_TYPES; i++) {
			printf("( %s, %u ) ", task_job_str[i], master_metadata_pool[mbi].frees_by_type[i]);
		}
		printf("\n");
		// Scheduler timings 
		/*
		  master_metadata_pool[mbi].sched_timings.idle_sec = 0;
		  master_metadata_pool[mbi].sched_timings.idle_usec = 0;
		  master_metadata_pool[mbi].sched_timings.get_sec = 0;
		  master_metadata_pool[mbi].sched_timings.get_usec = 0;
		  master_metadata_pool[mbi].sched_timings.queued_sec = 0;
		  master_metadata_pool[mbi].sched_timings.queued_usec = 0;
		  for (int ti = 0; ti < NUM_ACCEL_TYPES; ti++) {
		  master_metadata_pool[mbi].sched_timings.running_sec[ti] = 0;
		  master_metadata_pool[mbi].sched_timings.running_usec[ti] = 0;
		  }
		  master_metadata_pool[mbi].sched_timings.done_sec = 0;
		  master_metadata_pool[mbi].sched_timings.done_usec = 0;
		  for (int ti = 0; ti < 2; ti++) {
		  // FFT task timings
		  master_metadata_pool[mbi].fft_timings.comp_by[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.call_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.call_usec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_usec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_br_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_br_usec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_cvtin_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_cvtin_usec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_comp_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_comp_usec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_cvtout_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.fft_cvtout_usec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.cdfmcw_sec[ti] = 0;
		  master_metadata_pool[mbi].fft_timings.cdfmcw_usec[ti] = 0;
		  // Viterbi task timings
		  master_metadata_pool[mbi].vit_timings.comp_by[ti] = 0;
		  master_metadata_pool[mbi].vit_timings.dodec_sec[ti] = 0;
		  master_metadata_pool[mbi].vit_timings.dodec_usec[ti] = 0;
		  master_metadata_pool[mbi].vit_timings.depunc_sec[ti] = 0;
		  master_metadata_pool[mbi].vit_timings.depunc_usec[ti] = 0;
		  // CV/CNN task timings
		  master_metadata_pool[mbi].cv_timings.comp_by[ti] = 0;
		  master_metadata_pool[mbi].cv_timings.call_sec[ti] = 0;
		  master_metadata_pool[mbi].cv_timings.call_usec[ti] = 0;
		  master_metadata_pool[mbi].cv_timings.parse_sec[ti] = 0;
		  master_metadata_pool[mbi].cv_timings.parse_usec[ti] = 0;
		  }*/


	} // for (mbi loop over Metablocks)
}

