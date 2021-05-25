# Mini-ERA Scheduler Library Example Driver

This is a Mini-EAR example driver application for (i.e. use of) the Software Development Environment.
his code provides the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application (FFT and Viterbi kernels -- i.e. the C-Subset of Mini-ERA) implemented atop a Scheduler Library (SL).  Calls to execute the FFT or Viterbi (accelerator functions) are now turned into requests for the Scheduler to execute the task, and the Scheduler will then schedule these tasks across the available function-execution hardware (e.g. on a CPU via a pthread, or on a hardware accelerator where those are implemented).

## Requirements

Mini-ERA SL has been successfully built and executed using the following set-up:
 - Ubuntu 18.04
 - Ubuntu 16.04
 - Centos7
 
Other platforms should also work.
DTthis implementation currently does NOT support invocation of the hardware CV/CNN model code, or the (external, python-based) software CV/CNN tensorflow model code,
and thus does not require that support, etc.  This code does require gcc or similar/compatible compiler, an up to date pthreads library, and the C99 standard.

## Installation and Execution
The installation and execution are fairly standard, via github clone and makefiles:

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library
make clean
make
```

Once the scheduler library has been made (once), one can also make the Mini-ERA driver application from the ```examples/mini-era``` diretory.
```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library/library
make
cd ../examples/mini-era
make
```

The `clean` and `clobber` targets can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc.
The `make` command should produce the `test-scheduler*` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

The `make` process uses the contents of a `hardware.config` file to set various compile-time parameters; the contents of this file must be "compatible" with the scheduler library description of the hardware, e.g. application cannot request or expect hardware accelerators not present in the bvuild of the scheduler-library (or on the hardware platform).

The current version of this software produces an executable target that includes some representation of these configuration parameters.  Thus, the output executable will likely have a name like `test-scheduler-CF-P3V0F0N0` which indicates:
 - this is the `test-scheduler` trace-driven (as opposed to `sim-test-scheduler` simulation-driven) run-type
 - the 'S' indicates it is all-software (no hardware accelerators; only uses CPUs)
 - the build allows for 'P3' three CPU accelerators (processors), 'V0' and zero hardware Viterbi accelerators, 'F0' and zero FFT accelerators, and 'N0' zero CV/CNN
When building for hardware that includes hardware accelerators (e.g. 3 FFT, 2 Viterbi, and 1 CV/CNN) then the name would reflect that, e.g. the target executable could read `test-scheduler-RV-F2VCHo-P3V2F3N1` which indicates:
 - it was cross-compiled for `RV` (RISC-V) target architecture
 - the build uses the `F2` fft2 hardware accelerator, 'V' Viterbi hardware accelerator, and `CHo` Hardware (NVDLA) only (i.e. no processor-based execution of CV/CNN tasks)
 - and the build allows for 'P3' three CPU accelerators (processors), 'V2' and two hardware Viterbi accelerators, 'F3' and three hardware FFT accelerators, and 'N1' one CV/CNN hardware NVDLA accelerator

This new naming convention is primarily used to make clearer the target plaftforms on which the executable can be expected to function.  The _all-software_ executable can execute on any platform.  The example above that uses hardware accelerators (`test-scheduler-RV-F2VCHo-P3V2F3N1`) can therefore be executed on a RISC-V based system that includes at least 2 hardware Viterbi, 3 hardware FFT, and 1 CV/CNN NVDLA accelerator.

### Targets

The standard ```make``` of the scheduler-library code will produce two executables:
 - `test-scheduler*`: corresponds to the trace-driven `mini-era` executable.
 - `sim-test-scheduler*`: corresponds to the simulation version of `mini-era` (where the inputs are derived by simulation of arrival events, guided by thresholds and random number selection).  For more details on the simulation versus trae-driven `mini-era` programs, please see the documentation in the mini-era github repository.

### Configuration

The Mini-ERA + SL build supports more than just a basic Linux platform; there is provision to tie this same application into the EPOCHS <a href="https://esp.cs.columbia.edu" target="_blank">ESP-based</a> SoC platform (currently propotyped on an FPGA). As such, there are options to compile this application code within the ESP SoC design environment to cross-compile an output target for the EPOCHS native RISC-V Linux SoC environment. There are also some additional configuration capabilites tied in to the overall make process, and these are controlled in two ways:

1. Build using an explicit Makefile, e.g. to build the local (native to this system) version, invoke `make -f Makefile.local` which will use the native gcc and compile to a version that does not include the use of ESP-based SoC hardware; to compile explicitly to the ESP-based SoC RISC-V system, build with `make -f Makefile.riscv`

2. Alter the contents of the `.config` file. The config file contains a set of defines used by the Makefile to produce the proper build style by default.

### The `.config` File Contents

The config file contains a series of definitions, like #define macros or environment variables, used to guide the default make behavior. These contents are:

- `DO_CROSS_COMPILATION=y`    indicates we are cross-compiling (uses the cross-compiler defined in Makefile)
- `COMPILE_TO_ESP=y`  	      indicates we are compiling to target the ESP RISC-V SoC environment
- `CONFIG_ESP_INTERFACE=y`    this should always be set -- historical control to choose between some function interfaces.
- `CONFIG_FFT_EN=y`	      enable FFT Hardware Accelerators
- `CONFIG_FFT_FX=32`	      indicates FFT accelerators use 32-bit FXP format (can specify 64)
- `CONFIG_FFT_BITREV=y`	      indicates FFT accelerators include the bit-reverse operation
- `CONFIG_VITERBI_EN=y`	      enable Viterbi Decode Hardware Accelerators
- `CONFIG_KERAS_CV_BYPASS=y`  turns off the Tensorflow code, etc. -- Leave this enabled!
- `CONFIG_FAKE_CV_EN=y`	      turns on a "simulation" of teh CV hardfware accelerator (faking its use by allocating scheduler resources for a specified execution time)
- `CONFIG_CV_ONLY_HWR=y`      turns on a mode where the CV can only be run using the hardware accelerator (i.e. if CPU is insufficient to run the CV model)
- `CONFIG_VERBOSE=y`	      turns on a LOT of debugging output
- `CONFIG_DBG_THREADS=y`      turns on debugging output for the threads 
- `CONFIG_GDB=y`	      indicates compilation should iclude the "-g" flag to retain symbols, etc. which provides for greater debugger support, etc.

- `CONFIG_NUM_CPU=3`	indicates the number of `CPU` accelerators that can be used/scheduled for this build
- `CONFIG_NUM_VIT=0`	indicates the number of Viterbi-Decoder hardware accelerators that can be used/scheduled for this build
- `CONFIG_NUM_FFT=0`	indicates the number of 1D-FFT hardware accelerators that can be used/scheduled for this build
- `CONFIG_NUM_CV=0`	indicates the number of CV/CNN hardware accelerators that can be used/scheduled for this build

- `CONFIG_GDB=y`	 indicates the build should include symbols, to support gdb/debugger use
- `CONFIG_VERBOSE=y`	 indicates the build will include verbose debugging output
- `CONFIG_DBG_THREADS=y` indicates the build will include thread-debug messaging

There may be others, and these may be modified as we continue development.

### Usage

As indicated, there are two executables that will execute the Mini-ERA functionality on the SL, and their usage is very similar,
but with a few distinctions owing to the differences in running with a trace input as versus a simulated world  providing inputs.

#### Trace-Driven Version: `test-scheduler*`
```
./test-scheduler-S-P3V0F0N0 -h
Usage: ./test-scheduler-S-P3V0F0N0 <OPTIONS>
 OPTIONS:
    -h          : print this helpful usage info
    -o          : print the Visualizer output traace information during the run
    -R <file>   : defines the input Radar dictionary file <file> to use
    -V <file>   : defines the input Viterbi dictionary file <file> to use
    -C <file>   : defines the input CV/CNN dictionary file <file> to use
    -s <N>      : Sets the max number of time steps to simulate
    -t <trace>  : defines the input trace file <trace> to use
    -p <N>      : defines the plan-and-control repeat factor (calls per time step -- default is 1)
    -f <N>      : defines which Radar Dictionary Set is used for Critical FFT Tasks
                :      Each Set of Radar Dictionary Entries Can use a different sample size, etc.
    -N <N>      : Adds <N> additional (non-critical) CV/CNN tasks per time step.
    -D <N>      : Delay (in usec) of CPU CV Tasks (faked execution)
    -F <N>      : Adds <N> additional (non-critical) FFT tasks per time step.
    -v <N>      : defines Viterbi message size:
                :      0 = Short messages (4 characters)
                :      1 = Medium messages (500 characters)
                :      2 = Long messages (1000 characters)
                :      3 = Max-sized messages (1500 characters)
    -M <N>      : Adds <N> additional (non-critical) Viterbi message tasks per time step.
    -S <N>      : Task-Size Variability: Varies the sizes of input tasks where appropriate
                :      0 = No variability (e.g. all messages same size, etc.)
    -u <N>      : Sets the hold-off usec for checks on work in the scheduler queue
                :   This reduces the busy-spin-loop rate for the scheduler thread
    -B <N>      : Sets the number of Metadata Blocks (max) to <N>
    -P <policy> : defines the task scheduling policy <policy> to use (<policy> is a string)
                :   <policy> needs to exist as a dynamic shared object (DSO) with filename lib<policy>.so
    -L <tuple>  : Sets the limits on number of each accelerator type available in this run.
                :      tuple = #CPU,#FFT,#VIT,#CV (string interpreted internally)
    -X <tuple>  : Sets the Test-Task parameters for this run; default is NO Test-Tasks.
                :   Two tuple formats are acceptable:
                :      tuple = #Crit,#Base : Number of per-time-step Critical and Base Test-tasks injected
                :      tuple = #Crit,#Base,tCPU,tFFT,tVIT,tCV : Num Crit and Base tasks, and usec exec time
                :              per each accelerator type

 Options for the Scheduler-Visualizer tool (enable tracing to be visualized):
    -O <fn>     : Output scheduler visualization trace information to file <fn>
    -i <N>      : Number of executed tasks (of any type) before starting task logging
                :   If not specified, then logging starts with the execution
    -I <type>   : Task type that triggers task logging for the Schedule-Visualization tool
                :   If not specified, then logging starts with the execution
                :  NOTE: If -i and -I are specified, then logging starts when either condition is satisfied
    -e <N>      : Number of executed tasks (of any type) before stopping task logging
                :   This parameter is mandatory to keep control of the trace file size
```

To actually execute a trace, one must point to the trace in the trace repository (subdirectory ```traces```) using the ```-t``` option.
Note that the user must also specify a Scheduler Library policy to be used.
The Scheduler policies are defined in the Scheduler Library code, but are produced as separate shared-object files (i.e. lib*.so) whcih are dynamically loaded at execution time.  These libraries reside in the top-level's ```library/policies``` directory.

#### Simulation-Driven Version: ```sim-test-scheduler*```
```
./sim-test-scheduler-S-P3V0F0N0 -h
Usage: ./sim-test-scheduler-S-P3V0F0N0 <OPTIONS>
 OPTIONS:
    -h          : print this helpful usage info
    -o          : print the Visualizer output traace information during the run
    -R <file>   : defines the input Radar dictionary file <file> to use
    -V <file>   : defines the input Viterbi dictionary file <file> to use
    -C <file>   : defines the input CV/CNN dictionary file <file> to use
    -s <N>      : Sets the max number of time steps to simulate
    -r <N>      : Sets the rand random number seed to N
    -A          : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)
    -W <wfile>  : defines the world environment parameters description file <wfile> to use
    -p <N>      : defines the plan-and-control repeat factor (calls per time step -- default is 1)
    -f <N>      : defines which Radar Dictionary Set is used for Critical FFT Tasks
                :      Each Set of Radar Dictionary Entries Can use a different sample size, etc.
    -N <N>      : Adds <N> additional (non-critical) CV/CNN tasks per time step.
    -D <N>      : Delay (in usec) of CPU CV Tasks (faked execution)
    -F <N>      : Adds <N> additional (non-critical) FFT tasks per time step.
    -v <N>      : defines Viterbi message size:
                :      0 = Short messages (4 characters)
                :      1 = Medium messages (500 characters)
                :      2 = Long messages (1000 characters)
                :      3 = Max-sized messages (1500 characters)
    -M <N>      : Adds <N> additional (non-critical) Viterbi message tasks per time step.
    -S <N>      : Task-Size Variability: Varies the sizes of input tasks where appropriate
                :      0 = No variability (e.g. all messages same size, etc.)
    -u <N>      : Sets the hold-off usec for checks on work in the scheduler queue
                :   This reduces the busy-spin-loop rate for the scheduler thread
    -B <N>      : Sets the number of Metadata Blocks (max) to <N>
    -P <policy> : defines the task scheduling policy <policy> to use (<policy> is a string)
                :   <policy> needs to exist as a dynamic shared object (DSO) with filename lib<policy>.so
    -L <tuple>  : Sets the limits on number of each accelerator type available in this run.
                :      tuple = #CPU,#FFT,#VIT,#CV (string interpreted internally)
    -X <tuple>  : Sets the Test-Task parameters for this run; default is NO Test-Tasks.
                :   Two tuple formats are acceptable:
                :      tuple = #Crit,#Base : Number of per-time-step Critical and Base Test-tasks injected
                :      tuple = #Crit,#Base,tCPU,tFFT,tVIT,tCV : Num Crit and Base tasks, and usec exec time
                :              per each accelerator type

 Options for the Scheduler-Visualizer tool (enable tracing to be visualized):
    -O <fn>     : Output scheduler visualization trace information to file <fn>
    -i <N>      : Number of executed tasks (of any type) before starting task logging
                :   If not specified, then logging starts with the execution
    -I <type>   : Task type that triggers task logging for the Schedule-Visualization tool
                :   If not specified, then logging starts with the execution
                :  NOTE: If -i and -I are specified, then logging starts when either condition is satisfied
    -e <N>      : Number of executed tasks (of any type) before stopping task logging
                :   This parameter is mandatory to keep control of the trace file size
```

For execution in simulation mode (e.g. using ```sim-test-scheduler*```) no trace is necessary, and the simulation provides the inputs.
One must have the ```default_world.desc``` (or equivalent) file to provide the simulation parameters.

## Code/Layout

This code provides a "Scheduler Library" and an example application (Mini-ERA) that uses it.
The "Scheduler-Library" code is contained under the ```sched_lib``` subdirectory.  There is a brief README there as well, which focuses more on the library portion, adn it has its own Makefiole.  Essentially, the "scheduler Library" should be seen as an entirely separate body of code, that produces a "service platform" to which an application can connect and utilize those services.

The remaining code, at the top level (and in the ```include``` and ```src``` directories) contains the application that is making use of the scheduler, along with some additional code that provides definitions (for the application) of the ```tasks``` that the application will employ (e.g. see ```include/vit_task.h``` and ```src/vit_task.c``` for the Viterbi Decoder task definitions).

## Using the Scheduler Library from an Application

This distribution contains an example that uses the Scheduler (via the Scheduler-Library), and it can provide a bit of a tutorial on how to integrate the Scheduler into an application.
Currently, the Scheduler Library API is undergoing some major re-worki to simplify and claen up the API calls required to interface an application to the Scheduler.
The foillowing description is somewhat out of date, and will be revised when the API (next release) is finalized.

The basic Scheduler interface is defined in the 
<a href="https://github.com/IBM/scheduler-library/tree/master/sched_lib">Scheduler Library</a> in the file
<a href="https://github.com/IBM/scheduler-library/tree/master/sched_lib/include/scheduler.h">scheduler.h</a>.
We encourage you to look at that file for a genreal description and overview of the Scheduler API.


The basic use of the Scheduler Library is illustrated in the
<a href="https://github.com/IBM/scheduler-library/tree/master/src/main.c">main.c</a> file, though additional portions appear in the
<a href="https://github.com/IBM/scheduler-library/tree/master/src/main.c">kernels_api.c</a> file as well.

In the Mini-ERA based example application, most of the Scheduler interfacing ins managed in the ```main.c``` file, and in the ```main()``` routine.
In ```main()``` the application starts with the ```set_up_scheduler``` call to initialize the Scheduler.
After that, the code does a ```scheduler_get_datastate_in_parms_t* sched_inparms = malloc(sizeof(scheduler_get_datastate_in_parms_t));```
to create a Scheduler get_datastate input parameters space, and a call to ```copy_scheduler_datastate_defaults_into_parms(sched_inparms);```
to initialize the iScheduler datastate parameters with the (Scheduler-provided) default values.
These input parameter defaults are then adjusted to correspond to the use by this application:
```
  // Alter the default parms to those values we want for this run...
  sched_inparms->max_metadata_pool_blocks = num_MBs_to_use;
  sched_inparms->max_task_types = num_maxTasks_to_use;
  sched_inparms->max_accel_types = MY_APP_ACCEL_TYPES;
  // keep the default sched_inparms->max_accel_of_any_type = 4;
  sched_inparms->max_data_space_bytes = (128*1024 + 64);
  
  sched_inparms->enable_sched_viz_trace = enable_sl_viz_output;
```
and then the ```scheduler_datastate_block_t* sptr = get_new_scheduler_datastate_pointer(sched_inparms);```
call is made to use these input parms to establish the desired Scheduler Datastate space (i.e. one that supports the required sizes and dimensions of this Application).

With the Scheduler Datastate Block set up, the Application now does a ```initialize_scheduler(sptr, my_sl_viz_fname);```
call to initialize the Scheduler, using this Datastate Block space.
Now the Application has to indicate which Accelerators it will us, whcih is does by a series of calls like
```cpu_accel_id = register_using_accelerator_pool(sptr, SCHED_CPU_ACCEL_T, input_accel_limit_cpu);```
which in this case registers with the Scheduler that this Application will use a pool of up to ```input_accel_limit_cpu```
accelerators of type ```SCHED_CPU_ACCEL_T``` during the run.  There are similar calls for the other accelerators that this Mini-ERA application will use (e.g. ```SCHED_EPOCHS_VITDEC_ACCEL_T```).

Once the Accelerators to be used are enumerated, the Application then needs to define the Task types that the Scheduler will have to manage.  Each application defines its own Task types to the scheduler, and this is done here by filling in a task defninition structure and then passing that through to the Scheduler:
```
  task_type_defn_info_t task_defn;

  sprintf(task_defn.name, "VIT-Task");
  sprintf(task_defn.description, "A Viterbi Decoding task to execute");
  task_defn.print_metadata_block_contents  = &print_viterbi_metadata_block_contents;
  task_defn.output_task_type_run_stats  = &output_vit_task_type_run_stats;
  vit_task_type = register_task_type(sptr, &task_defn);
```

At this point, we now have defined the "VIT-Task" which is a Viterbi Decoder task.  The actual Viterbi Decoder Task definition contents appear in th e```vit_task.h``` and ```vit_task.c``` files, and similarly for the others (e.g. "CV_Task", "FFT_Task" etc.).  All that is left in the setup portion, is to identify which Accelerators (from those we earlier registered for use) can execute the Viterbi Decoder Tasks.  This is done with another API call:
```
  register_accel_can_exec_task(sptr, cpu_accel_id,     vit_task_type, &exec_vit_task_on_cpu_accel);
  register_accel_can_exec_task(sptr, vit_hwr_accel_id, vit_task_type, &exec_vit_task_on_vit_hwr_accel);
```
And here we see that the CPU accerators (i.e. the CPU running a task in a separate thread) can execute the Viterbi Decoder tasks, as can the Viterbi Hardware Acceleratror.  Note that the Application provides the function to use to "initiate" the execution on the accelerator, e.g. the ```&exec_vit_task_on_vit_hwr_accel``` (which in this case is in ```vit_task.c``` in this repository.

Now, the Mini-ERA application is ready to start normal executi9on, using the Scheduler to schedule the Tasks (i.e .the FFT, Vit Decode, CV/CNN, and plan-and--control tasks).  The execution of a task involves several steps as well.  First, the Application must acquire a Metadata Block for that Task to use:
```
      vit_mb_ptr = get_task_metadata_block(sptr, time_step, vit_task_type, CRITICAL_TASK, vit_profile[vit_msgs_size]);
```
Here the Application gets a metadata block to use for a Critical (```CRITICAL_TASK```) Viterbi Decoder task (```vit_task_type```).  The last argument is a set of profile data which estimates the execution time of this Viterbi Decoder Task on each of the Accelerator (types) in this run, which may be used by the Scheduler in sleecting the accelerator to whcih this Task is allocated.

The next step is to start execution of the task.
```
    vit_mb_ptr->atFinish = NULL; // Just to ensure it is NULL
    start_execution_of_vit_kernel(vit_mb_ptr, vdentry_p); // Critical VITERBI task
```
The first line sets the ```atFinish``` function pointer to NULL.  The ```atFinish``` function is used when no outputs are required from the execution of the Task, and the Application simply wants to "fire and forget" the execution; in that case the ```atFinish``` routine can be automatically called at the end of the execution (by the Scheduler) to clean up after execution, and return the Metadata Block to the free pool.

The next line calls the ```start_execution_of_vit_kernel``` routine, which is in ```kernels_api.c``` (the original Mini-ERA used the kernel API to provide a logical separation of the main-line software code and the accelkerated kernels).  Looking at the ```start_execution_of_vit_kernel``` code, we see it simply calls the ```start_decode``` function, which is in the ```viterbi_flat.c``` file.  Looking at the ```start_decode``` file, we find the code to fill in the Metadata Block with input data:
```
  // Set up the task_metadata scope block
  vit_metadata_block->data_size = 43365; // MAX size?
  // Copy over our task data to the MetaData Block
  // Get a viterbi_data_struct_t "View" of the metablock data pointer.
  viterbi_data_struct_t* vdsptr = (viterbi_data_struct_t*)(vit_metadata_block->data_space);
  // Copy inputs into the vdsptr data view of the metadata_block metadata data segment
  vdsptr->n_data_bits = frame->n_data_bits;
  vdsptr->n_cbps      = ofdm->n_cbps;
  vdsptr->n_traceback = d_ntraceback;
  vdsptr->psdu_size   = frame->psdu_size;
  vdsptr->inMem_size = 72; // fixed -- always (add the 2 padding bytes)
  uint8_t* in_Mem   = &(vdsptr->theData[0]);
  uint8_t* in_Data  = &(vdsptr->theData[vdsptr->inMem_size]);
  uint8_t* out_Data = &(vdsptr->theData[vdsptr->inMem_size + vdsptr->inData_size]);
```
Once the Metadata Block contains all the inputs required to execute the Task, the Application calls the Scheduler API to request execution of that Task:

```
  // Call the do_decoding routine
  request_execution(vit_metadata_block);
```

This call instructs the Scheduler to add the Task (specified/described in the Metadata Block) to the "Ready Task Queue" as a ready-to-be-scheduled Task.
An independent Scheduler thread is continuously examining the "Ready Task Queue" and trying to schedule the ready tasks to accelerators.
Once a task is scheduled (i.e. allocated to a specific acceleratr) it enters the "Running" state and is "sent" to that accelerator, where it will do the actual execution.

At this point, the Application has effectively "forked" the Task to the Scheduler, and can continue on in it's main-line processing (e.g. to set up and ```request_execution``` of other Tasks for this Application run).  Eventually, the Tasks will finish execution, and enter the "Done" state, at wihch time they will have completed execution and released the accelerator (for other Task' usage).

There are several ways for the Application to identify when tasks are "done" executing.  First, the Metadata Block conatins a field that is the current Task status, and one can simply read out the Task status from there.  There are also a couple of additional Scheduler routines that provide some simple "synchronization barrier" type semantics:
```
 void wait_all_critical(scheduler_datastate_block_t* sptr);
 void wait_all_tasks_finish(scheduler_datastate_block_t* sptr);
```
The first of these waits until all "Crtiical" priority tasks in the Scheduler are in the "Done" state, and then returns to the caller.  This is useful for some simple DAGs, and is used in the Mini-ERA Application example, where each time step one Viterbi, one FFT and one CV/CNN task are marked "Critical" and their outputs are used in the Plan-and-Control function.  The Mini-ERA kicks off these critical tasks, then uses ```wait_all_critical``` to delay reading those Tasks' outputs and using them for the Plan-and-Control Task execution.   The scond routine, ```wait_all_tasks_finish``` is similar to ```wait_all_critical``` but waits for ALL task to be done (not just the Critical ones).

After the ```wait_all_critical`` call, the Mini-ERA Application reads out the outputs from the FFT (Radar), Viterbi, Fetc. Tasks:
```
    distance = finish_execution_of_rad_kernel(fft_mb_ptr);
    message = finish_execution_of_vit_kernel(vit_mb_ptr);
```
In this Application, it uses those outputs to drive inputs to the Plan-and-Control task in a manner analogous to the set-up and execution of these tasks (e.g. it gets a Metadata Block, fills in the inputs, etc.).  The finish_execution code is also in ```kernels_api.c``` and looking at the code for ```finish_execution_of_vit_kernel``` we see that adt the end, that routine frees the Metadata Block:
```
  // We've finished the execution and lifetime for this task; free its metadata
  free_task_metadata_block(mb_ptr);
```
which makes the Metadata Block available (returns it to the free-list of currently unassigned Metadata Blocks) for use by other Tasks.


## Status

This platform is meant for SL development and integration, so it is expected to change over time. Currently, this is a relatively complete but bare-bones trace version Mini-ERA implementation. Additional features of the Mini-ERA code, and extensions thereto should also be developed over time.

There are currently some example traces in the `traces` subdirectory. Note that most development of Mini-ERA and its off-shoots to date has focused around the `tt02.new` trace.

 - `tt00.new` is a 5000 record illustrative trace.
 - `tt01.new` is a 5000 record illustrative trace.
 - `tt02.new` is a 5000 record trace that includes multiple obstacle vehicles per lane (at times).
 - `tt03.new` is a 5000 record trace that includes multiple obstacle vehicles per lane (at times).

There is a default set of world parameter setings for the simulation-driven mode (```sim-test-scheduler*```) as well.
This file (```default-world.desc```) describes the thresholds for various input occurrences, etc.

For additional information on all aspects of the Mini-ERA baseline workload,
please see the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> main-line README.


## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
