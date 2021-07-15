# The Scheduler's Task Library

This is part of a Software Development Environment for initial development, deployment, and testing of a *smart scheduler*. 
This code provides the underlying bas functions of the EPOCHS Task Scheduler's Task Library.

The Task Library is a collection of Task definitions, where a task is a logically complete, off-loadable (ie. can be run on a separate accelerator) packet of workload functionality wiht a well-defined interface (i.e. known, limited, non-global inputs/outputs). 


## The "Task" API

The Task Library uses a relatively simple API.  Each task is requierd to express some "data-structure" (overlay/specialization) and several interface functions.

The Tasks are executed in a off-load format, which means that they copy their input set into a localized (off-load) memory, and produce their outputs into that off-load memory.  At invocation, the full set of inputs must be spcified in a task-specified memory space (accessible by the off-load engine) and at completion, the results will be placed similarly into that space.

Under the covers, this is managed by using a "Task Metadata Block" which is defined with a general data transfer space (of some maxmimal size).  The Task Library API expects that a task will specify a task-specific "view" of this data space, which the task-spcific functions in the task library can utilize.  One can see an example of such a data space definition, e.g. in the ```include/vit_task.h``` file.

```
typedef struct { // The "Viterbi" view of "data"
  int32_t n_data_bits;
  int32_t n_cbps;
  int32_t n_traceback;
  int32_t psdu_size;
  int32_t inMem_size;   // The first inMem_size bytes of theData are the inMem (input memories)
  int32_t inData_size;  // The next inData_size bytes of theData are the inData (input data)
  int32_t outData_size; // The next outData_size bytes of theData are the outData (output data)
  uint8_t theData[64 * 1024]; // Larger than needed (~24780 + 18585) but less than FFT (so okay)
} viterbi_data_struct_t
```

This strucutre is then "overlaid" on the underlying Metedata Block "data" space, to give the interpretation that the fft tasks will use.  There is a similar overlay for run-time timing information (from ```include/vit_task.h```):

```
typedef struct {
  struct timeval call_start;
  struct timeval dodec_start;
  struct timeval depunc_start;
  struct timeval depunc_stop;

  uint64_t call_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t dodec_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t dodec_usec[SCHED_MAX_ACCEL_TYPES];

  uint64_t call_usec[SCHED_MAX_ACCEL_TYPES];
  uint64_t depunc_sec[SCHED_MAX_ACCEL_TYPES];
  uint64_t depunc_usec[SCHED_MAX_ACCEL_TYPES];
} vit_timing_data_t;
```

The API functions that must be provided are illustrated in the ```task_lib.config``` file, and include:
 - TaskRunStats : This is a function that outputs any final stats gathered for this type of task during the run, including the gathered timing information.
 - TaskFinishExecution : This routine is called by the application after it recognizs the computation is done; this should copy out output values, etc. and then release the Task Metadata Block (for other tasks to use).
 - TaskAutoFinish : This is a version of the finish routine that can be called "automatically" at the finish of the computation; it does not return output,e tc. but does clean up after the task is done.  This is most useful for tasks that do not return outputs to a dependent computation, but are strictly side-effect tasks.
 - TaskDevice1 CPU_ACCEL : This specifies the first device (type) on which the task can execute; typically we use CPU_ACCEL as the first target (as we assume all workload content can be executed on a CPU).
 - TaskExecuteOnDevice1 execute_cpu_fft_accelerator : This indicates the function (name) used to execute the Task on the first device (i.e. the CPU_ACCEL in this case).
 - TaskDevice2 : This indicates the second device type (name) that can execute this type of task (e.g. VITDEC_ACCEL, or CV_CNN_ACCEL)
 - TaskExecuteOnDevice2 : This is the function (name) used to embody the workload execution on that second device

One can specify as many devices (and their associated functions) as are supported by the Scheduler.
The ```TaskExecuteOnDevice``` function is used to set up the specific execution; in the case of the CPU_ACCEL it includes the workload functionality, and in the case of a hardware accelerator, any operations needed to prepare the workload inputs, etc. for execution on that harware (e.g. the FFT task includes conversion from ```float``` to a ```fixed-point``` representation).

## Status

This platform is meant for development and integration, and is under active development, so it is changing over time.

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
