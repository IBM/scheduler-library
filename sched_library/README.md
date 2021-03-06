# The Scheduler Library (SL)

This is a Software Development Environment for initial development, deployment, and testing of a *smart scheduler*. This code provides the underlying bas functions of the EPOCHS Task Scheduler in a linkable library format, along with the header file to provide important data type definitions, etc.

## Requirements

The Scheduler Library has been successfully built and executed using the following set-up:
 - Ubuntu 18.04
 - Ubuntu 16.04
 - CentOs 7

Other platforms should also work. 
This code does require gcc or similar/compatible compiler, an up to date pthreads library, and the C99 standard.
This code has also been cross-compiled for a linux RISC-V environment, and to target specific implementations of a RISC-V based SoC including a variety of hardware accelerators (to which th scheduler can schedule tasks).

## Installation and Execution

This code is the Scheduler library portion; it is usually obtained as part of the full 
<a href="https://github.com/IBM/scheduler-library" target="_blank">Scheduler-Library Github Repository</a>
along with the example-of-use the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application
implementation.

Installation and execution are fairly standard, via github clone and makefiles.  To make the full repository:

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library
make
```
or to make just the Task-Scheduler Library portion:

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library/library
make
```

The `make clean` can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc. There is also a `make clobber` which removes the object-file directories, executables, etc. The `make` command should produce the `test-scheduler*` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

The `make` process uses the contents of a `.config` file to set various compile-time parameters.  The current version of this configuration is taken from the parent directory (i.e. the usage application in this case).

The resulting output is ```libscheduler.a``` which is a library containing the task-scheduler functionality.
The buidl also produces a directory ```policies``` which is populated with a set of shared-object libraries that each define a scheduler policy.
The Scheduler does not provide a default policy, and one must indicate which scheduler (shared-object library) should be used by the Scheduler for the run.

## The Scheduler API

The basic Scheudler interface is defined in the 
<a href="https://github.com/IBM/scheduler-library/tree/master/sched_lib/include/scheduler.h">scheduler.h</a> file.

NOTE: This API is undergoing major revision, and the follwing description is somewhat outdated.  This will be updated when we have finalized the nmewest revisions to the API.

At the top is a definition of various types (structures) used in the scheduler and in interfacing with the scheduler.
The basic flow is that an application will
1 "Connect" to the scheduler
2 Register the set of scheduler resources this application will use
3 Execution the Application's workload
4 closeout the use of the scheduler

### "Connecting" to the Scheduler

Currently, the scheduler is implemented as a linked-=in library, so it runs in the same process space, etc. as the application.  In the longer term, we plan to have thge scheduler act more as a service of the system, but for now, "connecting" to the scheduler really means doing the initial set-up and initialization of the scheduler state, etc.  When the scheduler is a service, this process will look similar, but for the initial set-up step.

This requires use of several API functions:
1. ```status_t set_up_scheduler()``` - this does the one-time initial set up of the internal Scheduler (global, hidden) state.  Functionally, this sets up the full set ("dictionary") of hardware accelerators (and physical limits) in the hardware system.  Eventually this mshould be a boot-time type of process.
2. ```scheduler_get_datastate_in_parms_t* sched_inparms = malloc(sizeof(scheduler_get_datastate_in_parms_t))``` - this creates a space in whcih to put scheduler initialization parameters, for the following call to ```get_new_scheduler_datastate_pointe```
3. ```void copy_scheduler_datastate_defaults_into_parms(scheduler_get_datastate_in_parms_t* parms_ptr)``` - this copies default values into the provided parms_ptr.  This is not necessary, but does allow an application to run without setting any parm values, guaranteeing all input parm,s are set to legal values.
4. ```scheduler_datastate_block_t* get_new_scheduler_datastate_pointer(scheduler_get_datastate_in_parms_t* inp)``` - this generates a new Scheduler data-state space (i.e. the internal working state used by an instance of the scheduler for a given application).  The returned pointer is used in most subsequent API calls.

At this point, the application has now "connected" to the Scheduler, and has an application-specific Scheduler Datastate Block allocated for it (and pointed to by the returned pointer).

### Register the Resources the Application will Use

Once the Application has a Scheduler Datastate Block, it can now register the resources that it will try to utilize during execution.  This process defines the set of accelerators the Application would like to have available (i.e. the accelerator "pools") and the kinds of Tasks (i.e. jobs or operations) to be run under scheduler control/scheduling during the Application execution.

This also requires several API functions:
1. ```accelerator_type_t register_using_accelerator_pool(scheduler_datastate_block_t* sptr, scheduler_accelerator_type acid, int desired_number_in_pool)``` - this is used to indicate to the Scheduler that this application will be using a given (type of) acceleratror (```acid```). The list of supported accelerators types is defined at the top of the ```scheduler.h``` file.  This is done for each accelerator that the application will be using; the final input parameterindicates the total number of such accelerators the application is requesting be in the accelerator pool.
2. ```task_type_t register_task_type(scheduler_datastate_block_t* sptr, task_type_defn_info_t*)``` - this sets up an application Task -- it defines a task type that will be used in the execution.  Tasks are the element taht the Scheduler will assign to accelerators, and these Tasks are defined by the application itself.  The ```task_type_defn_t``` is also defined in ```scheduler.h``` and indicates the name and description strings for the task, and some function-pointers used to print task-specific information, and to output final run data, etc. for that task.
3. ```void register_accel_can_exec_task(scheduler_datastate_block_t* sptr, accelerator_type_t acid, task_type_t tid, sched_execute_task_function_t fptr)``` - this indicates to the Scheduler that the given accelerator type (```acid```) can execute Tasks of the given task type (```tid```) using the provided execute-on-accelerator task function (```fptr```).

At this point, the Application has defined (1) the set of accelerators that shoudl be allocated to it for this run, (2) the set of Task types that may be being scheduld during the run, and (3) the accelerators that can execute each type of Task.  At this point, the Scheduler and Application are ready to execute the application's work.

### Execute the Application Tasks

Once the Scheduler is initialized and knows what Tasks to expect, and how they can be executed across the accelerators, the workload can then star the execution process, using Scheduler API calls to execute the registered Tasks.  To do this, the Application makes use of a ```Metadata Block```.  The Metadata Block (MB) is used to contain all the relevant information about a given Task (instance) so that this MB can be handed off to the Scheduler, queued up (e.g. to wait for an acceleartor to become available), executed, and to contain any returned data.  The concept of the Metadata Block is used to communicate in a safe manner across threads and/or physical hardware components (hardware accelerators) the input and output values of the Task (placed in a contiguous shared-memory segment amenable to DMA transfer) along twith a variety of other Task state information, and debug and statistics information.

The execution process therefor involves use of several API functions:
1. ```task_metadata_block_t* get_task_metadata_block(scheduler_datastate_block_t* sptr, int32_t dag_id, task_type_t of_task_type, task_criticality_t crit_level, uint64_t * task_profile)``` - this gets a new Metadata Block (from a pre-allocated pool in the Scheduler) for use by this Application to track and execute this Task.
2. ```void request_execution(task_metadata_block_t* task_metadata_block)``` - this indicates that the Metadata Block is fully initialized for this Task, and the Task is ready to start execution; the Scheduler can now scheduler this Task onto an accelerator (or perhaps enqueue it for later execution).
3.```Metadata Block "status" field``` - this field indicates the current execution/status of the Task in the Scheduler (see ```scheduler.h```) and can be used to check the current status of a Task that has been submitrted via ```request_execution`` into the Scheuduler.  The list of Task status results is defined in the ```scheduler.h``` include file.
4. ```void wait_all_critical(scheduler_datastate_block_t* sptr)``` - this pauses the Application (busy-wait here) until all the Tasks that were submitted as Critical tasks (there are multiple Task criticality levels, all defined in the ```scheduler.h``` include file) have reached the "Done" status.  This is an execution synchronization barrier.
5. ```void wait_all_tasks_finish(scheduler_datastate_block_t* sptr)``` - this pauses the Application (busy-wait here) until all the Tasks that were submitted have reached the "Done" status.  This is an execution synchronization barrier.
6. ```void mark_task_done(task_metadata_block_t* task_metadata_block)``` - This is used in the Application's Task execution function (used as a parameter to the ```register_accel_can_exec_task``` API function) to indicate that the task has finished execution; it effectively changes the state of the Tasks execution (in the scheduler) to "Done" to indicate execution is completed.
7. ```void free_task_metadata_block(task_metadata_block_t* mb)```- This is used when the Application is done with a Task -- after it has copied out any output results, etc. -- to return the Metadata Block to the global Metadata Block pool (free list) for future use.

### Closeout the Scheduler

When the execution is done, the Application should call ```void shutdown_scheduler(scheduler_datastate_block_t* sptr)``` to closeout the Scheduler's resources, deallocate memory, free the threads, etc.

## Status

This platform is meant for development and integration, and is under active development, so it is changing over time.

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
