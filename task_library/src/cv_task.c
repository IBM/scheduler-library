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
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "viterbi_types.h"
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
PyObject * pName, * pModule, * pFunc, * pFunc_load;
PyObject * pArgs, * pValue, * pretValue;
#define PY_SSIZE_T_CLEAN

char * python_module = "yolo";
char * python_func = "predict";
char * python_func_load = "loadmodel";
#endif

#ifdef ENABLE_NVDLA
extern "C" void initNVDLA();
extern "C" void runImageonNVDLAWrapper(char * Image);
#endif

#ifdef COMPILE_TO_ESP
// This is using the hardware (RISC-V FPGA) numbers
unsigned cv_cpu_run_time_in_usec = 3500000;
unsigned cv_fake_hwr_run_time_in_usec = 320000;
#else
// This is using the local (presume X86) numbers
//unsigned cv_cpu_run_time_in_usec = 10000;
//unsigned cv_fake_hwr_run_time_in_usec = 1000;
unsigned cv_cpu_run_time_in_usec = 3500000;
unsigned cv_fake_hwr_run_time_in_usec = 320000;
#endif

std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> cv_profile;
unsigned image_index = 0;

extern "C" {
  void print_cv_metadata_block_contents(/*task_metadata_entry*/ void * mb_ptr) {
    task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
    print_base_metadata_block_contents(mb);
  }
}

extern "C" {
  void output_cv_task_type_run_stats(/*scheduler_datastate*/ void * sptr_ptr, unsigned my_task_type,
    unsigned total_accel_types) {
    scheduler_datastate * sptr = (scheduler_datastate *) sptr_ptr;

    printf("\n  Per-MetaData-Block %u %s Timing Data: %u finished tasks over %u accelerators\n",
      my_task_type, sptr->task_name_str[my_task_type],
      sptr->freed_metadata_blocks[my_task_type], total_accel_types);
    // The CV/CNN Task Timing Info
    unsigned total_cv_comp_by[total_accel_types + 1];
    uint64_t total_cv_call_usec[total_accel_types + 1];
    uint64_t total_nvdla_usec[total_accel_types + 1];
    uint64_t total_parse_usec[total_accel_types + 1];
    for (int ai = 0; ai <= total_accel_types; ai++) {
      total_cv_comp_by[ai] = 0;
      total_cv_call_usec[ai] = 0;
      total_nvdla_usec[ai] = 0;
      total_parse_usec[ai] = 0;
    }
    for (int ai = 0; ai < total_accel_types; ai++) {
      if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
        printf("\n  Per-MetaData-Block-Timing for Task  %u %s on Accelerator %u %s\n", my_task_type,
          sptr->task_name_str[my_task_type], ai, sptr->accel_name_str[ai]);
      }
      for (int bi = 0; bi < sptr->total_metadata_pool_blocks; bi++) {
        cv_timing_data_t * cv_timings_p = (cv_timing_data_t *) &
          (sptr->master_metadata_pool[bi].task_timings[my_task_type]);
        unsigned this_comp_by = (unsigned) (
          sptr->master_metadata_pool[bi].task_computed_on[ai][my_task_type]);
        uint64_t this_cv_call_usec = (uint64_t) (cv_timings_p->call_sec[ai]) * 1000000 + (uint64_t) (
          cv_timings_p->call_usec[ai]);
        uint64_t this_nvdla_usec = (uint64_t) (cv_timings_p->nvdla_sec[ai]) * 1000000 + (uint64_t) (
          cv_timings_p->nvdla_usec[ai]);
        uint64_t this_parse_usec = (uint64_t) (cv_timings_p->parse_sec[ai]) * 1000000 + (uint64_t) (
          cv_timings_p->parse_usec[ai]);
        if (sptr->scheduler_execute_task_function[ai][my_task_type] != NULL) {
          printf("    Block %3u : %u %s : CmpBy %8u call-time %15lu parse %15lu usec\n", bi, ai,
            sptr->accel_name_str[ai], this_comp_by, this_cv_call_usec,
            this_parse_usec);
        }
        else {
          if ((this_comp_by + this_cv_call_usec + this_parse_usec) != 0) {
            printf("  ERROR: Block %3u : %u %s : CmpBy %8u call-time %15lu parse %15lu usec\n", bi, ai,
              sptr->accel_name_str[ai], this_comp_by, this_cv_call_usec,
              this_parse_usec);
          }
        }
        // Per acceleration (CPU, HWR)
        total_cv_comp_by[ai] += this_comp_by;
        total_cv_call_usec[ai] += this_cv_call_usec;
        total_nvdla_usec[ai] += this_nvdla_usec;
        total_parse_usec[ai] += this_parse_usec;
        // Overall Total
        total_cv_comp_by[total_accel_types] += this_comp_by;
        total_cv_call_usec[total_accel_types] += this_cv_call_usec;
        total_nvdla_usec[total_accel_types] += this_nvdla_usec;
        total_parse_usec[total_accel_types] += this_parse_usec;
      } // for (bi = 1 .. numMetatdataBlocks)
    }   // for (ai = 0 .. total_accel_types)

    printf("\nAggregate TID %u %s Tasks Total Timing Data:\n", my_task_type,
      sptr->task_name_str[my_task_type]);
    printf("     CNN-call  run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_cv_call_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_cv_comp_by[ai], total_cv_call_usec[ai], avg);
    }
    {
      double avg = (double) total_cv_call_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_cv_comp_by[total_accel_types], total_cv_call_usec[total_accel_types], avg);
    }

    printf("     CNN-NVDLA run time\n                          ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_nvdla_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_cv_comp_by[ai], total_nvdla_usec[ai], avg);
    }
    {
      double avg = (double) total_nvdla_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_cv_comp_by[total_accel_types], total_nvdla_usec[total_accel_types], avg);
    }

    printf("     get_label (parse) run time\n                  ");
    for (int ai = 0; ai < total_accel_types; ai++) {
      double avg = (double) total_parse_usec[ai] / (double) sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n                          ", ai,
        sptr->accel_name_str[ai], total_cv_comp_by[ai], total_parse_usec[ai], avg);
    }
    {
      double avg = (double) total_parse_usec[total_accel_types] / (double)
        sptr->freed_metadata_blocks[my_task_type];
      printf("%u %20s %8u %15lu usec %16.3lf avg\n", total_accel_types, "TOTAL",
        total_cv_comp_by[total_accel_types], total_parse_usec[total_accel_types], avg);
    }
  }}

static inline label_t parse_output_dimg() {
  FILE * file_p = fopen("./output.dimg", "r");
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
  return (label_t) max_idx;
}

extern "C" {
  // void execute_hwr_cv_accelerator( /*task_metadata_entry*/ void * cv_io_ptr) {
  void execute_hwr_cv_accelerator (size_t in_size, label_t in_label, label_t * obj_label, size_t obj_label_size) {
    // cv_io_t * cv_data_p = (cv_io_t *) (cv_io_ptr);
#ifdef HW_CV
    //printf("  We have HW_CV defined...\n");
#ifdef ENABLE_NVDLA
  //printf("  We have ENABLE_NVDLA defined...\n");
  // Add the call to the NVDLA stuff here.
    label_t tr_label = in_label;
    char image_name[32];
    switch (tr_label) {
    case no_label:
      sprintf(image_name, "cnn_data/empty_%02u.jpg", (image_index & 0x1f));
      break;
    case bicycle:
      sprintf(image_name, "cnn_data/bike_%02u.jpg", (image_index & 0x1f));
      break;
    case car:
      sprintf(image_name, "cnn_data/car_%02u.jpg", (image_index & 0x1f));
      break;
    case pedestrian:
      sprintf(image_name, "cnn_data/person_%02u.jpg", (image_index & 0x1f));
      break;
    case truck:
      sprintf(image_name, "cnn_data/truck_%02u.jpg", (image_index & 0x1f));
      break;
    default:
      printf("ERROR : unknown input object type %u\n", tr_label);
    }
    DEBUG(printf("Calling NVDLA for idx %u image %s\n", image_index, image_name));
    runImageonNVDLAWrapper(image_name);
    DEBUG(printf("   DONE with NVDLA call...\n"));
    image_index++;
    DEBUG(printf("Setting pred_label from parse_output_dimg call...\n"));
    label_t pred_label = parse_output_dimg();
    TDEBUG(printf("---> Predicted label = %d\n", pred_label));
    // Set result into the metatdata block
    //task_metadata_block->data_view.cv_data.object_label = pred_label;
    *(obj_label) = pred_label;
#endif // ifdef ENABLE_NVDLA

#else // Of #ifdef HW_CV
#ifdef FAKE_HW_CV
    // This usleep call stands in as the "Fake" CNN accelerator
    usleep(cv_fake_hwr_run_time_in_usec);
    // cv_data_p->object_label = run_object_classification(tr_val);

#endif // #fdef FAKE_HE_CV
#endif // else of #ifdef HW_CV
  }}

label_t run_object_classification(unsigned tr_val) {
  DEBUG(printf("Entered run_object_classification... tr_val = %u\n", tr_val));
  label_t object = (label_t) tr_val;
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
        return (label_t) 1;
      }
      PyTuple_SetItem(pArgs, 0, pValue);
      pretValue = PyObject_CallObject(pFunc, pArgs);
      Py_DECREF(pArgs);
      if (pretValue != NULL) {
        DEBUG(printf("Predicted label from Python program: %ld\n",
          PyLong_AsLong(pretValue)));
        int val = PyLong_AsLong(pretValue);
        object = (label_t) val;
        DEBUG(printf("run_object_classification returning %u = %u\n", val,
          object));
        Py_DECREF(pretValue);
      }
      else {
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        printf("Trying to run CNN kernel : Python function call failed\n");
        return (label_t) 1;
      }
    }
    else {
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

extern "C" {
  void execute_cpu_cv_accelerator( /*task_metadata_entry*/ void * cv_io_ptr) {
    cv_io_t * cv_data_p = (cv_io_t *) cv_io_ptr;

    label_t tr_val = cv_data_p->in_label;

    *(cv_data_p->obj_label) = run_object_classification((unsigned) tr_val);
  }
}

// void execute_cpu_cv_accelerator( /*task_metadata_entry*/ void *task_metadata_block_ptr) {
//   task_metadata_entry *task_metadata_block = (task_metadata_entry *)task_metadata_block_ptr;
//   DEBUG(printf("In execute_cpu_cv_accelerator: MB %d  CL %d\n", task_metadata_block->block_id, task_metadata_block->crit_level));
//   int aidx = task_metadata_block->accelerator_type;
//   task_metadata_block->task_computed_on[aidx][task_metadata_block->task_type]++;
//   cv_timing_data_t *cv_timings_p = (cv_timing_data_t *) & (task_metadata_block->task_timings[task_metadata_block->task_type]);

// #ifdef INT_TIME
//   gettimeofday(&(cv_timings_p->call_start), NULL);
// #endif

//   usleep(cv_cpu_run_time_in_usec);

// #ifdef INT_TIME
//   struct timeval stop_time;
//   gettimeofday(&stop_time, NULL);
//   cv_timings_p->call_sec[aidx] += stop_time.tv_sec - cv_timings_p->call_start.tv_sec;
//   cv_timings_p->call_usec[aidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
// #endif

//   TDEBUG(printf("MB%u CV_CPU calling mark_task_done...\n", task_metadata_block->block_id));
//   mark_task_done(task_metadata_block);
// }


void set_up_cv_task_on_accel_profile_data() {
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    cv_profile[sizeof(label_t)][ai] = ACINFPROF;
  }
#ifndef CONFIG_CV_ONLY_HWR
  // NOTE: The following data is for the RISCV-FPGA environment @ ~78MHz
  cv_profile[sizeof(label_t)][SCHED_CPU_ACCEL_T] =
    cv_cpu_run_time_in_usec; // Specified in the run - was 5000000
  DEBUG(printf("CV_PROFILE: NOT_HW_ONLY : cv_profile[sizeof(label_t)][CPU] = %lu\n",
    cv_profile[sizeof(label_t)][SCHED_CPU_ACCEL_T]));
#endif

#ifdef COMPILE_TO_ESP
  //printf("CV_PROFILE: COMPILE_TO_ESP : \n");
#ifdef FAKE_HW_CV
  cv_profile[sizeof(label_t)][SCHED_EPOCHS_CV_CNN_ACCEL_T] =
    cv_fake_hwr_run_time_in_usec; // Specified in the run
  DEBUG(printf("CV_PROFILE: ESP_and_FAKE_HW : cv_profile[sizeof(label_t)][NVDLA] = %u\n",
    cv_profile[sizeof(label_t)][SCHED_EPOCHS_CV_CNN_ACCEL_T]));
#else
  cv_profile[sizeof(label_t)][SCHED_EPOCHS_CV_CNN_ACCEL_T] = 150000;
  DEBUG(printf("CV_PROFILE: ESP_and_REAL_HW : cv_profile[sizeof(label_t)][NVDLA] = %u\n",
    cv_profile[sizeof(label_t)][SCHED_EPOCHS_CV_CNN_ACCEL_T]));
#endif
#endif
  DEBUG(printf("\n%18s : %18s %18s %18s %18s\n", "CV-PROFILES", "CPU", "FFT-HWR", "VIT-HWR", "CV-HWR");
  printf("%15s :", "cv_profile[sizeof(label_t)]");
  for (int ai = 0; ai < SCHED_MAX_ACCEL_TYPES; ai++) {
    printf(" 0x%016lx", cv_profile[sizeof(label_t)][ai]);
  }
  printf("\n\n"););
}


// void set_up_cv_task(task_metadata_entry * cv_mb_ptr, void * args) {
//   label_t in_label = ((cv_io_t *) args)->in_label;
//   cv_data_struct_t *cv_data_p = (cv_data_struct_t *)(cv_mb_ptr->data_space);
//   // Handle the input data to the task
//   cv_data_p->object_label = in_label;
// }

// This is a default "finish" routine that can be included in the
// start_executiond call for a task that is to be executed, but whose results
// are not used...
//
extern "C" {
  void cv_auto_finish_routine(/*task_metadata_entry*/ void * mb_ptr) {
    task_metadata_entry * mb = (task_metadata_entry *) mb_ptr;
    TDEBUG(scheduler_datastate * sptr = mb->scheduler_datastate_pointer;
    printf("Releasing Metadata Block %u : Task %s %s from Accel %s %u\n",
      mb->block_id, sptr->task_name_str[mb->task_type],
      sptr->task_criticality_str[mb->crit_level],
      sptr->accel_name_str[mb->accelerator_type],
      mb->accelerator_id));
    DEBUG(printf("  MB%u auto Calling free_task_metadata_block\n", mb->block_id));
    free_task_metadata_block(mb);
  }}

// NOTE: This routine DOES NOT copy out the data results -- a call to
//   calculate_peak_distance_from_fmcw now results in alteration ONLY
//   of the metadata task data; we could send in the data pointer and
//   over-write the original input data with the CV results (As we used to)
//   but this seems un-necessary since we only want the final "distance" really.
extern "C" {
  void finish_cv_execution(/*task_metadata_entry*/ void * cv_metadata_block_ptr, void * args) {
    task_metadata_entry * cv_metadata_block = (task_metadata_entry *) cv_metadata_block_ptr;
    // label_t *obj_label)
    label_t * obj_label = ((cv_io_t *) args)->obj_label;

    int tidx = cv_metadata_block->accelerator_type;
    cv_timing_data_t * cv_timings_p = (cv_timing_data_t *) &
      (cv_metadata_block->task_timings[cv_metadata_block->task_type]);
    cv_data_struct_t * cv_data_p = (cv_data_struct_t *) (cv_metadata_block->data_space);
#ifdef INT_TIME
    struct timeval stop_time;
    gettimeofday(&stop_time, NULL);
    cv_timings_p->call_sec[tidx] += stop_time.tv_sec - cv_timings_p->call_start.tv_sec;
    cv_timings_p->call_usec[tidx] += stop_time.tv_usec - cv_timings_p->call_start.tv_usec;
#endif // INT_TIME

    * obj_label = cv_data_p->object_label;

    // We've finished the execution and lifetime for this task; free its metadata
    DEBUG(printf("  MB%u Calling free_task_metadata_block\n", cv_metadata_block->block_id));
    free_task_metadata_block(cv_metadata_block);
  }}
