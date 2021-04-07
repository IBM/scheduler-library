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

#include "scheduler.h"
#include "cv_task.h"

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


unsigned cv_cpu_run_time_in_usec      = 10000;
unsigned cv_fake_hwr_run_time_in_usec =  1000;


void print_cv_metadata_block_contents(task_metadata_block_t* mb) {
  print_base_metadata_block_contents(mb);
}


void
output_cv_task_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_task_id, unsigned total_accel_types)
{
  
  printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n", my_task_id, sptr->task_name_str[my_task_id], sptr->freed_metadata_blocks[my_task_id], total_accel_types);
  // The CV/CNN Task Timing Info
  unsigned total_cv_comp_by[total_accel_types+1];
  uint64_t total_cv_call_usec[total_accel_types+1];
  uint64_t total_parse_usec[total_accel_types+1];
  for (int ai = 0; ai <= total_accel_types; ai++) {
    total_cv_comp_by[ai] = 0;
    total_cv_call_usec[ai] = 0;
    total_parse_usec[ai] = 0;
  }
  for (int ai = 0; ai < total_accel_types; ai++) {
    if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[my_task_id][ai] != NULL)) {
      printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_id, sptr->task_name_str[my_task_id], ai, sptr->accel_name_str[ai]);
    }
    for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
      cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(sptr->master_metadata_pool[bi].task_timings[my_task_id]);
      unsigned this_comp_by = (unsigned)(sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_id]);
      uint64_t this_cv_call_usec = (uint64_t)(cv_timings_p->call_sec[ai]) * 1000000 + (uint64_t)(cv_timings_p->call_usec[ai]);
      uint64_t this_parse_usec = (uint64_t)(cv_timings_p->parse_sec[ai]) * 1000000 + (uint64_t)(cv_timings_p->parse_usec[ai]);
      if ((ai == total_accel_types-1) || (sptr->scheduler_execute_task_function[my_task_id][ai] != NULL)) {
	printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu parse %15lu usec\n", bi, ai, sptr->accel_name_str[my_task_id], this_comp_by, this_cv_call_usec, this_parse_usec);
      } else {
	if ((this_comp_by + this_cv_call_usec + this_parse_usec) != 0) {
	  printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu parse %15lu usec\n", bi, ai, sptr->accel_name_str[my_task_id], this_comp_by, this_cv_call_usec, this_parse_usec);
	}
      }
      // Per acceleration (CPU, HWR)
      total_cv_comp_by[ai] += this_comp_by;
      total_cv_call_usec[ai]  += this_cv_call_usec;
      total_parse_usec[ai] += this_parse_usec;
      // Overall Total
      total_cv_comp_by[total_accel_types] += this_comp_by;
      total_cv_call_usec[total_accel_types]  += this_cv_call_usec;
      total_parse_usec[total_accel_types] += this_parse_usec;
    } // for (bi = 1 .. numMetatdataBlocks)
  } // for (ai = 0 .. total_accel_types)

  printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_id, sptr->task_name_str[my_task_id]);
  printf("     CNN-call  run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_cv_call_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_cv_comp_by[ai], total_cv_call_usec[ai], avg);
  }
  {
    double avg = (double)total_cv_call_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_cv_comp_by[total_accel_types], total_cv_call_usec[total_accel_types], avg);
  }

  printf("     get_label run time   ");
  for (int ai = 0; ai < total_accel_types; ai++) {
    double avg = (double)total_parse_usec[ai] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai, sptr->accel_name_str[ai], total_cv_comp_by[ai], total_parse_usec[ai], avg);
  }
  {
    double avg = (double)total_parse_usec[total_accel_types] / (double)sptr->freed_metadata_blocks[my_task_id];
    printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL", total_cv_comp_by[total_accel_types], total_parse_usec[total_accel_types], avg);
  }
}


static inline label_t parse_output_dimg() {
  FILE *file_p = fopen("./output.dimg", "r");
  const size_t n_classes = 5;
  float probs[n_classes];
  for (size_t i = 0; i< n_classes; i++) {
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


void
execute_hwr_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  int fn = task_metadata_block->accelerator_id;
  int aidx = task_metadata_block->accelerator_type;
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  TDEBUG(printf("In execute_hwr_cv_accelerator on CV_HWR Accel %u : MB%d  CL %d\n", fn, task_metadata_block->block_id, task_metadata_block->crit_level));
#ifdef HW_CV
  // Add the call to the NVDLA stuff here.
  printf("Doing the system call : './nvdla_runtime --loadable hpvm-mod.nvdla --image 2004_2.jpg --rawdump'\n");
  //printf("Doing the system call : './nvdla_runtime --loadable mio_loadable.nvdla --image three.jpg'\n");
  #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
  #endif

  int sret = system("./nvdla_runtime --loadable hpvm-mod.nvdla --image 0003_0.jpg --rawdump");
  //int sret = system("./nvdla_runtime --loadable mio_loadable.nvdla --image three.jpg");
  if (sret == -1) {
    printf(" The system call returned -1 -- an error occured?\n");
  }
 #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->parse_start), NULL);
  cv_timings_p->call_sec[aidx]  += cv_timings_p->parse_start.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[aidx]  += cv_timings_p->parse_start.tv_usec  - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("REAL_HW_CV: Set Call_Sec[%u] to %llu %llu\n", cv_timings_p->call_sec[aidx], cv_timings_p->call_usec[aidx]));
 #endif
  label_t pred_label = parse_output_dimg();
 #ifdef INT_TIME
  struct timeval stop;
  gettimeofday(&(stop), NULL);
  cv_timings_p->parse_sec[aidx]  += stop.tv_sec  - cv_timings_p->parse_start.tv_sec;
  cv_timings_p->parse_usec[aidx] += stop.tv_usec - cv_timings_p->parse_start.tv_usec;
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
  cv_timings_p->call_sec[aidx]  += stop_time.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[aidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
  DEBUG(printf("FAKE_HW_CV: Set Call_Sec[%u] to %lu %lu\n", aidx, cv_timings_p->call_sec[aidx], cv_timings_p->call_usec[aidx]));
  #endif

 #else
  printf("ERROR : This executable DOES NOT support Hardware-CV execution!\n");
  cleanup_and_exit(-2);
 #endif
#endif
  TDEBUG(printf("MB%u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}


label_t run_object_classification_syscall(unsigned tr_val)
{
  DEBUG(printf("Entered run_object_classification_syscall...\n"));
  label_t object;
#ifdef BYPASS_KERAS_CV_CODE
  object = (label_t)tr_val;
#else
  char shell_cmd[100];
  snprintf(shell_cmd, sizeof(shell_cmd), "sh utils/cnn_shell.sh %u", tr_val);
  DEBUG(printf("  Invoking CV CNN using `%s`\n", shell_cmd));
  FILE *testing = popen(shell_cmd, "r");
  if (testing == NULL)
  {
    printf("FAIL to open CV kernel !\n");
    return 1;
  }
  char pbuffer[100];
  while (fgets(pbuffer, 100, testing) != NULL)
  {
    //printf(pbuffer);
  }
  DEBUG(printf("Label Prediction done \n"));
  DEBUG(printf("pbuffer : %s\n", pbuffer));
  int val = atoi(pbuffer);   //the last thing printed by the Keras code is the predicted label
  object = (label_t)val;
  pclose(testing);
  DEBUG(printf("run_object_classification_syscall returning %u = %u\n", val, object));
#endif
  return object;
}

label_t run_object_classification(unsigned tr_val)
{
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
	fprintf(stderr, "Trying to run CNN kernel: Cannot convert C argument into python\n");
	return 1;
      }
      PyTuple_SetItem(pArgs, 0, pValue);
      pretValue = PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      if (pretValue != NULL) {
	DEBUG(printf("Predicted label from Python program: %ld\n", PyLong_AsLong(pretValue)));
	int val = PyLong_AsLong(pretValue);
	object = (label_t)val;
	DEBUG(printf("run_object_classification returning %u = %u\n", val, object));
	Py_DECREF(pretValue);
      }
      else {
	Py_DECREF(pFunc);
	Py_DECREF(pModule);
	PyErr_Print();
	printf("Trying to run CNN kernel : Python function call failed\n");
	return 1;
      }
    }
    else {
      if (PyErr_Occurred())
	PyErr_Print();
      printf("Cannot find python function");
    }
    Py_XDECREF(pFunc);
    //Py_DECREF(pModule);
  }
#endif
  return object;
}


void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block)
{
  DEBUG(printf("In execute_cpu_cv_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level ));
  int aidx = task_metadata_block->accelerator_type;
  task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
  cv_timing_data_t * cv_timings_p = (cv_timing_data_t*)&(task_metadata_block->task_timings[task_metadata_block->task_type]);

 #ifdef INT_TIME
  gettimeofday(&(cv_timings_p->call_start), NULL);
 #endif

  usleep(cv_cpu_run_time_in_usec);

 #ifdef INT_TIME
  struct timeval stop_time;
  gettimeofday(&stop_time, NULL);
  cv_timings_p->call_sec[aidx]  += stop_time.tv_sec  - cv_timings_p->call_start.tv_sec;
  cv_timings_p->call_usec[aidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
 #endif

  TDEBUG(printf("MB_THREAD %u calling mark_task_done...\n", task_metadata_block->block_id));
  mark_task_done(task_metadata_block);
}




