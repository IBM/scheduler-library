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

//#define VERBOSE
#include "verbose.h"

#include "scheduler.h"
#include "cv_accel.h"

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

void
do_cv_accel_type_initialization(scheduler_datastate* sptr) {
#ifndef BYPASS_KERAS_CV_CODE
   Py_Initialize();
   pName = PyUnicode_DecodeFSDefault(python_module);
   pModule = PyImport_Import(pName);
   Py_DECREF(pName);

   if (pModule == NULL) {
      PyErr_Print();
      printf("Failed to load Python program, perhaps pythonpath needs to be set; export PYTHONPATH=your_mini_era_dir/cv/CNN_MIO_KERAS");
      return;
   } else {
      pFunc_load = PyObject_GetAttrString(pModule, python_func_load);

      if (pFunc_load && PyCallable_Check(pFunc_load)) {
         PyObject_CallObject(pFunc_load, NULL);
      } else {
         if (PyErr_Occurred())
            PyErr_Print();
         printf("Cannot find python function - loadmodel");
      }
      Py_XDECREF(pFunc_load);
   }
   DEBUG(printf("CV Kernel Init done\n"));
#endif

#ifdef HW_CV
#endif
}


void
do_cv_accel_type_closeout(scheduler_datastate* sptr) {
#ifndef BYPASS_KERAS_CV_CODE
   Py_DECREF(pModule);
   Py_Finalize();
#endif
   // Clean up any hardware accelerator stuff
#ifdef HW_CV
#endif
}


void
output_cv_accel_type_run_stats(scheduler_datastate* sptr, unsigned my_accel_id, unsigned total_task_types) {
}

