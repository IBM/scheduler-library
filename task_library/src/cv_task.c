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

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "viterbi_utils.h"
//#define VERBOSE
#include "verbose.h"

#include "cv_task.h"
#include "scheduler.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#ifndef BYPASS_KERAS_CV_CODE
#include <Python.h>
#include <pythonrun.h>
#endif
#ifndef BYPASS_KERAS_CV_CODE
PyObject *pName, *pModule, *pFunc, *pFunc_load;
PyObject *pArgs, *pValue, *pretValue;
#define PY_SSIZE_T_CLEAN

char *python_module = "mio";
char *python_func = "predict";
char *python_func_load = "loadmodel";
#endif

#ifdef FAKE_HW_CV
unsigned cv_cpu_run_time_in_usec = 10000;
unsigned cv_fake_hwr_run_time_in_usec = 1000;
#else
#ifdef COMPILE_TO_ESP
unsigned cv_cpu_run_time_in_usec = 10000;
// unsigned cv_fake_hwr_run_time_in_usec =  1000;
#else
unsigned cv_cpu_run_time_in_usec = 50;
// unsigned cv_fake_hwr_run_time_in_usec =     1;
#endif
#endif

void print_cv_metadata_block_contents(/*task_metadata_block_t*/void *mb_ptr) {
  task_metadata_block_t* mb = (task_metadata_block_t*) mb_ptr;
  print_base_metadata_block_contents(mb);
}

void output_cv_task_type_run_stats(/*scheduler_datastate_block_t*/void *sptr_ptr,
                                   unsigned my_task_type,
                                   unsigned total_accel_types) {
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t*) sptr_ptr;

  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u "
         "accelerators\n",
         my_task_type, sptr->task_name_str[my_task_type],
         sptr->freed_metadata_blocks[my_task_type], total_accel_types);
  // The CV/CNN Task Timing Info
  unsigned total_cv_comp_by[total_accel_types + 1];
  uint64_t total_cv_call_usec[total_accel_types + 1];
  uint64_t total_parse_usec[total_accel_types + 1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_cv_comp_by[ai] = 0;
    total_cv_call_usec[ai] = 0;
    total_parse_usec[ai] = 0;
  }
  for (int ai = 0; ai < total_accel_types; ai++) {
    if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u "
             "%s\n",
             my_task_type, sptr->task_name_str[my_task_type], ai,
             sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      cv_timing_data_t *cv_timings_p = (cv_timing_data_t *)&(
          sptr->master_metadata_pool[bi].task_timings[my_task_type]);
      unsigned this_comp_by =
          (unsigned)(sptr->master_metadata_pool[bi]
                         .task_computed_on[ai][my_task_type]);
      uint64_t this_cv_call_usec =
          (uint64_t)(cv_timings_p->call_sec[ai]) * 1000000 +
          (uint64_t)(cv_timings_p->call_usec[ai]);
      uint64_t this_parse_usec =
          (uint64_t)(cv_timings_p->parse_sec[ai]) * 1000000 +
          (uint64_t)(cv_timings_p->parse_usec[ai]);
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu parse %15lu "
               "usec\n",
               bi, ai, sptr->accel_name_str[ai], this_comp_by,
               this_cv_call_usec, this_parse_usec);
      } else {
        if ((this_comp_by + this_cv_call_usec + this_parse_usec) != 0) {
          printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu parse "
                 "%15lu usec\n",
                 bi, ai, sptr->accel_name_str[ai], this_comp_by,
                 this_cv_call_usec, this_parse_usec);
        }
      }
      // Per acceleration (CPU, HWR)
      total_cv_comp_by[ai] += this_comp_by;
      total_cv_call_usec[ai] += this_cv_call_usec;
      total_parse_usec[ai] += this_parse_usec;
      // Overall Total
      total_cv_comp_by[total_accel_types] += this_comp_by;
      total_cv_call_usec[total_accel_types] += this_cv_call_usec;
      total_parse_usec[total_accel_types] += this_parse_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  }   // for (ai = 0 .. total_accel_types)

  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type,
         sptr->task_name_str[my_task_type]);
  printf("     CNN-call  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_cv_call_usec[ai] /
                 (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
           sptr->accel_name_str[ai], total_cv_comp_by[ai],
           total_cv_call_usec[ai], avg);
  }
  {
    double avg = (double)total_cv_call_usec[total_accel_types] /
                 (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
           total_cv_comp_by[total_accel_types],
           total_cv_call_usec[total_accel_types], avg);
  }

  printf("     get_label run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_parse_usec[ai] /
                 (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
           sptr->accel_name_str[ai], total_cv_comp_by[ai], total_parse_usec[ai],
           avg);
  }
  {
    double avg = (double)total_parse_usec[total_accel_types] /
                 (double)sptr->freed_metadata_blocks[my_task_type];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
           total_cv_comp_by[total_accel_types],
           total_parse_usec[total_accel_types], avg);
  }
}

static inline label_t parse_output_dimg() {
  FILE *file_p = fopen("./output.dimg", "r");
  const size_t n_classes = 5;
  float probs[n_classes];
  for (size_t i = 0; i < n_classes; i++) {
    if (fscanf(file_p, "%f", &probs[i]) != 1) {
      printf("Didn't parse the probs[%ld] from output.dimg\n", i);
    }
  }
  float max_val = 0.0f;
  size_t max_idx = -1;
  for (size_t i = 0; i < n_classes; i++) {
    if (probs[i] > max_val) {
      max_val = probs[i], max_idx = i;
    }
  }
  fclose(file_p);
  return (label_t)max_idx;
}

void execute_hwr_cv_accelerator(/*task_metadata_block_t*/void *task_metadata_block_ptr) {
  task_metadata_block_t* task_metadata_block = (task_metadata_block_t*) task_metadata_block_ptr;
  int fn = task_metadata_block->accelerator_id;
  int aidx = task_metadata_block->accelerator_type;
  cv_timing_data_t *cv_timings_p = (cv_timing_data_t *)&(
      task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  TDEBUG(printf(
      "In execute_hwr_cv_accelerator on CV_HWR Accel %u : MB%d  CL %d\n", fn,
      task_metadata_block->block_id, task_metadata_block->crit_level));
#ifdef HW_CV
  // Add the call to the NVDLA stuff here.
  printf("Doing the system call : './nvdla_runtime --loadable hpvm-mod.nvdla "
         "--image 2004_2.jpg --rawdump'\n");
// printf("Doing the system call : './nvdla_runtime --loadable
// mio_loadable.nvdla --image three.jpg'\n");
#ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
#endif

  int sret = system(
      "./nvdla_runtime --loadable hpvm-mod.nvdla --image 0003_0.jpg --rawdump");
  // int sret = system("./nvdla_runtime --loadable mio_loadable.nvdla --image
  // three.jpg");
  if (sret == -1) {
    printf(" The system call returned -1 -- an error occured?\n");
  }
#ifdef INT_TIME
  gettimeofday(&(cv_timings_p->parse_start), NULL);
  cv_timings_p->call_sec[aidx] +=
      cv_timings_p->parse_start.tv_sec - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[aidx] +=
      cv_timings_p->parse_start.tv_usec - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("REAL_HW_CV: Set Call_Sec[%u] to %llu %llu\n",
               cv_timings_p->call_sec[aidx], cv_timings_p->call_usec[aidx]));
#endif
  label_t pred_label = parse_output_dimg();
#ifdef INT_TIME
  struct timeval stop;
  gettimeofday(&(stop), NULL);
  cv_timings_p->parse_sec[aidx] +=
      stop.tv_sec - cv_timings_p->parse_start.tv_sec;
  cv_timings_p->parse_usec[aidx] +=
      stop.tv_usec - cv_timings_p->parse_start.tv_usec;
#endif
  TDEBUG(printf("---> Predicted label = %d\n", pred_label));
  // Set result into the metatdata block
  task_metadata_block->data_view.cv_data.object_label = pred_label;

#else // Of #ifdef HW_CV
#ifdef FAKE_HW_CV
#ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
#endif
  // This usleep call stands in as the "Fake" CNN accelerator
  usleep(cv_fake_hwr_run_time_in_usec);

#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[aidx] +=
      stop_time.tv_sec - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[aidx] +=
      stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_CV: Set Call_Sec[%u] to %lu %lu\n", aidx,
               cv_timings_p->call_sec[aidx], cv_timings_p->call_usec[aidx]));
#endif
#endif
#endif
  TDEBUG(printf("MB%u CV_HWR calling mark_task_done...\n",
                task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

label_t run_object_classification_syscall(unsigned tr_val) {
  DEBUG(printf("Entered run_object_classification_syscall...\n"));
  label_t object;
#ifdef BYPASS_KERAS_CV_CODE
  object = (label_t)tr_val;
#else
  char shell_cmd[100];
  snprintf(shell_cmd, sizeof(shell_cmd), "sh utils/cnn_shell.sh %u", tr_val);
  DEBUG(printf("  Invoking CV CNN using `%s`\n", shell_cmd));
  FILE *testing = popen(shell_cmd, "r");
  if (testing == NULL) {
    printf("FAIL to open CV kernel !\n");
    return 1;
  }
  char pbuffer[100];
  while (fgets(pbuffer, 100, testing) != NULL) {
    // printf(pbuffer);
  }
  DEBUG(printf("Label Prediction done \n"));
  DEBUG(printf("pbuffer : %s\n", pbuffer));
  int val = atoi(pbuffer); // the last thing printed by the Keras code is the
                           // predicted label
  object = (label_t)val;
  pclose(testing);
  DEBUG(printf("run_object_classification_syscall returning %u = %u\n", val,
               object));
#endif
  return object;
}

label_t run_object_classification(unsigned tr_val) {
  DEBUG(printf("Entered run_object_classification... tr_val = %u\n", tr_val));
  label_t object = (label_t)tr_val;
#ifndef BYPASS_KERAS_CV_CODE
  if (pModule != NULL) {
    DEBUG(printf("  Starting call to pModule...\n"));
    pFunc = PyObject_GetAttrString(pModule, python_func);

    if (pFunc && PyCallable_Check(pFunc)) {
      pArgs = PyTuple_New(1);
      pValue = PyLong_FromLong(tr_val);
      if (!pValue) {
        Py_DECREF(pArgs);
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        fprintf(stderr, "Trying to run CNN kernel: Cannot convert C argument "
                        "into python\n");
        return 1;
      }
      PyTuple_SetItem(pArgs, 0, pValue);
      pretValue = PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      if (pretValue != NULL) {
        DEBUG(printf("Predicted label from Python program: %ld\n",
                     PyLong_AsLong(pretValue)));
        int val = PyLong_AsLong(pretValue);
        object = (label_t)val;
        DEBUG(printf("run_object_classification returning %u = %u\n", val,
                     object));
        Py_DECREF(pretValue);
      } else {
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        printf("Trying to run CNN kernel : Python function call failed\n");
        return 1;
      }
    } else {
      if (PyErr_Occurred())
        PyErr_Print();
      printf("Cannot find python function");
    }
    Py_XDECREF(pFunc);
    // Py_DECREF(pModule);
  }
#endif
  return object;
}

void execute_cpu_cv_accelerator(/*task_metadata_block_t*/void *task_metadata_block_ptr) {
  task_metadata_block_t *task_metadata_block = (task_metadata_block_t*) task_metadata_block_ptr;
  DEBUG(printf("In execute_cpu_cv_accelerator: MB %d  CL %d\n",
               task_metadata_block->block_id, task_metadata_block->crit_level));
  int aidx = task_metadata_block->accelerator_type;
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  cv_timing_data_t *cv_timings_p = (cv_timing_data_t *)&(
      task_metadata_block->task_timings[task_metadata_block->task_type]);

#ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
#endif

  usleep(cv_cpu_run_time_in_usec);

#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[aidx] +=
      stop_time.tv_sec - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[aidx] +=
      stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
#endif

  TDEBUG(printf("MB%u CV_CPU calling mark_task_done...\n",
                task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}

uint64_t cv_profile[SCHED_MAX_ACCEL_TYPES];
void set_up_cv_task_on_accel_profile_data() {
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    cv_profile[ai] = ACINFPROF;
  }
 #ifdef COMPILE_TO_ESP
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz

  cv_profile[SCHED_CPU_ACCEL_T] = cv_cpu_run_time_in_usec; // Specified in the run - was 5000000
  #ifdef FAKE_HW_CV
  cv_profile[SCHED_EPOCHS_CV_CNN_ACCEL_T] = cv_fake_hwr_run_time_in_usec; // Specified in the run
  #else
  cv_profile[SCHED_EPOCHS_CV_CNN_ACCEL_T] = 150000;
  #endif
 #else
  cv_profile[SCHED_CPU_ACCEL_T] = cv_cpu_run_time_in_usec; // Specified in the run - was 50
 #endif
  DEBUG(printf("%15s :", "cv_profile");
        for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES;
             ai++) { printf(" 0x%016lx", cv_profile[ai]); } printf("\n");
        printf("\n"));
}

/*task_metadata_block_t*/void *
set_up_cv_task(/*scheduler_datastate_block_t*/void *sptr_ptr, task_type_t cv_task_type,
               task_criticality_t crit_level,
               bool use_auto_finish, int32_t dag_id, ...) // label_t in_label) {
{
  va_list var_list;
  va_start(var_list, dag_id);
  
  scheduler_datastate_block_t *sptr = (scheduler_datastate_block_t*) sptr_ptr;

#ifdef TIME
  gettimeofday(&start_exec_cv, NULL);
#endif
  label_t in_label = va_arg(var_list, label_t);

  // Request a MetadataBlock (for an CV task at Critical Level)
  task_metadata_block_t *cv_mb_ptr = NULL;
  DEBUG(printf("Calling get_task_metadata_block for Critical CV-Task %u\n", cv_task_type));
  do {
    cv_mb_ptr = get_task_metadata_block(sptr, dag_id, cv_task_type, crit_level, cv_profile);
    // usleep(get_mb_holdoff);
  } while (0); //(*mb_ptr == NULL);
#ifdef TIME
  struct timeval got_time;
  gettimeofday(&got_time, NULL);
  exec_get_cv_sec += got_time.tv_sec - start_exec_cv.tv_sec;
  exec_get_cv_usec += got_time.tv_usec - start_exec_cv.tv_usec;
#endif
  // printf("CV Crit Profile: %e %e %e %e %e\n",
  // cv_profile[crit_cv_samples_set][0], cv_profile[crit_cv_samples_set][1],
  // cv_profile[crit_cv_samples_set][2], cv_profile[crit_cv_samples_set][3],
  // cv_profile[crit_cv_samples_set][4]);
  if (cv_mb_ptr == NULL) {
    // We ran out of metadata blocks -- PANIC!
    printf("Out of metadata blocks for CV -- PANIC Quit the run (for now)\n");
    dump_all_metadata_blocks_states(sptr);
    exit(-4);
  }
  DEBUG(printf("MB%u In set_up_cv_task\n", cv_mb_ptr->block_id));

  if (use_auto_finish) {
    cv_mb_ptr->atFinish = sptr->auto_finish_task_function[cv_task_type]; // get_auto_finish_routine(sptr, cv_task_type);
  } else {
    cv_mb_ptr->atFinish = NULL;
  }

  cv_timing_data_t *cv_timings_p = (cv_timing_data_t *)&(cv_mb_ptr->task_timings[cv_mb_ptr->task_type]);
  cv_data_struct_t *cv_data_p = (cv_data_struct_t *)(cv_mb_ptr->data_space);
  // Handle the input data to the task
  cv_data_p->object_label = in_label;

#ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
#endif
  return cv_mb_ptr;
  //  schedule_cv(data);
}

// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
void cv_auto_finish_routine(/*task_metadata_block_t*/void *mb_ptr) {
  task_metadata_block_t *mb = (task_metadata_block_t*) mb_ptr;
  TDEBUG(scheduler_datastate_block_t *sptr = mb->scheduler_datastate_pointer;
         printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",mb->block_id, sptr->task_name_str[mb->task_type],
                sptr->task_criticality_str[mb->crit_level], sptr->accel_name_str[mb->accelerator_type], mb->accelerator_id));
  DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
  free_task_metadata_block(mb);
}

// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the CV results (As we used to)
//   but this seems un-necessary since we only want the final "distance" really.
void finish_cv_execution(/*task_metadata_block_t*/void *cv_metadata_block_ptr, ...)
{
  va_list var_list;
  va_start(var_list, cv_metadata_block_ptr);
  task_metadata_block_t* cv_metadata_block = (task_metadata_block_t*) cv_metadata_block_ptr;
  // label_t *obj_label)
  label_t* obj_label = va_arg(var_list, label_t*);

  int tidx = cv_metadata_block->accelerator_type;
  cv_timing_data_t *cv_timings_p = (cv_timing_data_t *)&(cv_metadata_block->task_timings[cv_metadata_block->task_type]);
  cv_data_struct_t *cv_data_p = (cv_data_struct_t *)(cv_metadata_block->data_space);
#ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[tidx] += stop_time.tv_sec - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[tidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
#endif // INT_TIME

  *obj_label = cv_data_p->object_label;

  // We've finished the execution and lifetime for this task; free its metadata
  DEBUG(printf("  MB%u Calling free_task_metadata_block\n", cv_metadata_block->block_id));
  free_task_metadata_block(cv_metadata_block);
}
