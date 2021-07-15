# Scheduler Library (SL)

This is a Software Development Environment for the development, deployment, and testing of a *smart scheduler*. 
The code provides the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application implemented using the EPOCHS Task-based Scheduler Library (SL).  
Rather than explicit management of the accelerators in the application, the application is written to use Tasks from a task library, where these tasks correspond to the underlying conceptal operaion, e.g. a Viterbi Decode.
The application therefore generates a series of requests to the Scheduler to execute these various tasks, 
and the Scheduler will then schedule these tasks across the available function-execution hardware (e.g. on a CPU via a pthread, or on a hardware accelerator where those are implemented).
The Scheduler determines a "best-fit" accelerator (for the current status of the system, etc.) using a policy; several policies are provided, and  new policies can be generated in simple C code and are compiled into dynamically-shared libraries, so the scheduler policy to employ can be specifed at run time.
Upon the task finishing on an accelerator, the Scheduler releases the accelerator back into the "available accelerators" pool, and the applicaiton will be xwable to determine that the task is finisheddetermine 
Calls pplication can then read out the outputs, etc.
to execute the FFT or Viterbi (accelerator functions) are now turned into requests for the Scheduler to execute the task, and the Scheduler will then schedule these tasks across the available function-execution hardware (e.g. on a CPU via a pthread, or on a hardware accelerator where those are implemented).

The Scheduler code is in the ```sched_library``` subdir, the Task library code is in the ```task_library``` subdir, and the Mini-ERA example driver application is under ```examples/mini-era```.
This code also supports compilation using the UIUC HPVM compiler framework, which has provided an intefgrated EPOCHS back-end that automatically implements the use of the task and scheduler library code for properly labeled portions of the code.  
Note, however, that his code is also compilable using a traditional C/C++ compiler (e.g. gcc).

## Requirements

This project has been successfully built and executed on the following linux platforms:
 - Ubuntu 18.04 (on x86-64 hardware)
 - Ubuntu 16.04 (on x86-64 hardware)
 - Centos7 (on x86-64 hardware)
 
Other platforms and Linux distributions should also work.
```

For the utilization of the HPVM compilation, one must conform to the HPVM requirements.


## Installation and Execution
The installation and execution are fairly standard, via github clone and makefiles.
For the most basic build (e.g. using the native C compiler to generate the default executable)"
```
git clone -b hpvm-integration https://github.com/IBM/scheduler-library.git
cd scheduler-library
```
At this point you should have all the viles you need, but note that there is a file named ```setup_paths.sh``` which contains the definitions of three paths that may need to be set up for your system.
These three paths are:
 - HPVM_DIR : This is the root directory of the HPVM compiler distribution/installation.  This path must be set if you intend to use the HPVM compiler. More details on setting up the appropriate HPVM environment are at the bottom of this README file.
 - RISCV_BIN_DIR : This should be set to the ```bin``` directory of your RISC-V toolchain.  This path is require if you intend to cross-compile to RISC-V targets.
 - ESP_ROOT : This shoudl point to the root of your ESP environment; this path is required to interface to the ESP interface for hardware accelerators, adn thus is required if you intend to compile to target hardware accelerators.

Once these paths are set, you shoudl ```source ./setup_paths.sh``` to make them active in your shell environment.
Then it is a simple matter to call ```make``` at the top-level (```scheduler-library```) to make all the required components.

There is a `make clean` that can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc. There is also a `make clobber` which removes the object-file directories, executables, etc. 
The `make` command should produce the ```test-scheduler*``` target (in the ```examples/mini-era``` subdir), which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application using the Scheduler.

The `make` process moves therough the sub-components and makes each in turn; this is a global top-level make to make not only the
Scheduler Library, but also the example driver applications.
One can also re-make individual components by moving into that directory (e.g. ```task_library``` oe ```sched_library```) and doing a ```make``` there.  Similarly, the examples directory has a top-level Makefile that shoudl build all examples (though currently we only have the one released in the distribution).

### HPVM Compilation

To make the example mini-era application using the Scheduler, compiled under HPVM, one must invoke an ```hpvm-``` compiler target.
The base build (equivalent to the above) is 
```
make hpvm-epochs
```
This should invoke the HPVM compiler process, build the mini-era applicaiton, and result in an executabel with a name that starts with ```hpvm-test-scheduler-*```.

### Hardware Configuration Files

The ```sched_library``` contains a set of "hardware configuration files" (with names like ```build_local.config``` and ```build_riscv.config```).  
These files provide information about which hardware acceleratos are "enabled" (i.e. available/implemented) in the hardware, and logically correspond to information the system would have acquired at "boot time" to reflect the physical hardware.
There is also a soft-link named ```hardware_config``` which is used to indicate which of these is the "current" reflection of the hardware, i.e. the configuration to eb used for compilation, etc.  
ANY time you change this link (e.g. to point to a different hardware configuration target) you should do a```make clean``` or ```make clobber``` to ensure that all code is re-built using the new configuration.

## Code/Layout

This code provides a "Scheduler" a "Task Library" and example applications (i.e. Mini-ERA) that use it.

The Task-Library consists of logical execution Tasks, described in C code, that conform to the Task-Library interface (API).  This task API provides some data-strucutre views (used to "interpret" the shared general task memory space) and a set of functions that implement operations, e.g. a task set-up routine to prepare the task execution (e.g. set up the task inputs), a task finish routine to closout the task (e.g. to recover the computed outputs), andso forth.

The "Scheduler" code is contained under the ```sched_library``` subdirectory.  
This code provides the scheduler engine, and a set of accelerator defninitions (used by the scheduelr to interface with the "hardeware accelerators).  
There is a README there as which focuses more on the Scheduler Library directly.
The Scheduler should be seen logically as an entirely separate body of code that produces a "service platform" to which an application can connect and utilize those services. 

The example drivers each reside in a subdirectory under the ```examples``` directory; currently the primary example driver is the ```mini-era``` workload, whcih is provided therein.  Each example driver application should include its own makefile, README, etc. and is expected to contain the full application that uses the Scheduler. See the ```mini-era``` README for more background information about the Mini-ERA workload, etc.

## Some make Targets and their Meanings

This distribution c$is a development platform and as such provides a large variety of compilation options/targets.  To provide some insights into the possibilities, we are providing a bit more description of some make targets, their compatible hardware_config options, and the general description of that system.  

While the Scheduler uses a compile-time configration file (to describe the physical hardware), the Mini-ERA workload also has configuration parameters that alter/limit the available hardware to the Mini-ERA run. 
Mini-ERA does have an option to provide a run-time configuration file (required with the HPVM compilation) that provides much of the configuration information for the given run.

### Non-HPVM Compilation
```make``` will generate the Scheduler and example application according to the current hardware_config file.
The effective resulting machine depends on the selected hardware_config
 - build_local.config : this results in an all-software (x86) executable that uses only "CPU" accelerators (i.e. separate parallel thread execution) to execute the Tasks.
 - build_riscv_SW.config : this results in a all-software cross-compiled to RISC-V executable that uses only "CPU" accelerators (i.e. separate parallel thread execution) to execute the Tasks.
 - build_riscv.config : this builds a RISC-V cross-compiled executable that includes hardware accelerators for the FFT and Viterbi Tasks (and uses a simple delay-loop for the CV/CNN).  This pre-dates the final integration of the CV/CNN NVDLA accelerator into the Scheduler environment.
 - build_riscv_nvdla.config : this builds a RISC-V cross-compiled executable that includes hardware accelerators for all three of the main Mini-ERA Tasks (Viterbi decode, FFT/Radar distancing, and CV/CNN Object classification).
 - build_epochs0.config : this is a historic copy of build_riscv.config that includes HWR accelerator limits related to our EPOCHS0 Asic implementation; it simply ahs different limits on the numbers of HWR accelerators of each type.

More description of the Scheduler's "hardware_config" file contents, etc. is provided in the Scheduler README.  
Note taht these various files can also be modified, and are effectively just saved copies of specific configuration value sets that we've used at various times in development.

### HPVM-Compilation

HPVM compilation sues some additiona make target indications to control some aspects of the HPVM compilation.
 - The ```make hpvm-cpu``` target will compile the mini-era application without use of the Task or Scheduler library.  This provides a capability not currently included inthe non-HPVM make (i.e. the non-HPVM make always targets use of the Scheduler).
 - The ```make hpvm-epochs``` target will compile the mini-era application to use the Task library and the Scheduler.  This is interpreted as make using HPVM and target the EPOCHS Scheduler API/environment.

The HPVM compile uses the hardware_config file to identify the available hardware components.  In addition, the HPVM compiler is "aware" of the Task Library; it actually uses the Task Library definition to identy the tasks, and to provide Scheduler interfacing to the application automatically.
Because of this, HPVM uses a Task Library configuration file in the ```task_library``` subdir, and by default it uses the file ```task_lib.config``` to provide this information.
Tehre is a second version of the Task Library configuration that only includes the "CPU Accelerators" version of the Task Library content, which effectively indicates to HPVM that there are no hardware accelerators avaialble for this applciation (so it will only compile to CPU Accelerated tasks).  

The various harwdware configuration files and these task-library configuration files can also be used together in combinations with HPVM compilation.
- build_local.config : this defiens a hardware configuration that only allows for CPU accelerators; compilation under HPVM (for either Task Library configuration) should result in an all-software (x86) executable that uses only "CPU" accelerators (i.e. separate parallel thread execution) to execute the Tasks.
 - build_riscv_SW.config : similar to ```build_local.config``` this should produce an all-software (CPU accelerators) verison, but cross-compiled to RISCS-V execution
 - build_riscv_nvdla.config : this builds a hardware-accelerator enabled RISC-V cross-compiled executable that includes hardware accelerators for all three of tcehe main Mini-ERA Tasks (Viterbi decode, FFT/Radar distancing, and CV/CNN Object classification). If this is built using ```make hpvm-epochs``` then it should include execution of the hardware and/or CPU accelerators.  If this is compiled using ```make TASK_CONFIG_FILE=~/scheduler-library/task_library/task_lib_cpuonly.config hpvm-epochs``` then HPVM will use the "only CPU Accelerators" Task Library configuration and limite the executable to only using CPU Accelerators (not the hardware accelerators).
- and the other hardware configurations are similar to the above.


## Status

This platform is meant for SL development and integration, so it is expected to change over time. Currently, this is a relatively complete version of the Task Scheduling
functiuonality, and the Mini-ERA driver application is similarly complete.

The Scheduler Library is under constant development (and hopefully improvement).

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)



# MINIERA-HPVM

## HPVM Setup

-   HPVM (internal repository branch  `hpvm-release-epochs0`  located  [here](https://gitlab.engr.illinois.edu/llvm/hpvm-release/-/tree/hpvm-release-epochs0)
    -   Refer to  [HPVM Build Instructions](https://hpvm.readthedocs.io/en/latest/build-hpvm.html)  for detailed set up instructions.  _Note: During installation, make sure target is set to X86;RISCV to be able to target the EPOCHS-0 RISC-V processor._

-   GCC cross compiler for RISC-V, can be installed using ESP as follows:
    -   Clone ESP repository using:  `git clone --recursive https://github.com/sld-columbia/esp.git`
    -   Checkout the epochs branch:  `cd esp && git checkout epochs`
    -   Invoke the cross-compiler installation script:  `./utils/scripts/build_riscv_toolchain.sh`

## HPVM Installation and Execution

First we must clone and set up the hpvm repository:

    git clone --branch hpvm-release-epochs0 https://gitlab.engr.illinois.edu/llvm/hpvm-release/-/tree/hpvm-release-epochs0

Install HPVM via the provided install script and follow the provided prompts:

    cd ${HPVM_SRC_ROOT}/hpvm/hpvm/
    ./install.sh

## Setting up required paths
After installation, you will need to source the `set_paths.sh` script to export the environment variables needed for running the hpvm scheduler backend for MiniERA.
Specifically you will have to provide definitions for the root of your hpvm clone which contains the build directory (`HPVM_DIR`), as well as the
path to the RISCV bin directory (`RISCV_BIN_DIR`).

Provide these variable definitions in `${SCHED_LIB_SRC_ROOT}/setup_paths.sh`. After which:

    cd ${SCHED_LIB_SRC_ROOT}
    source setup_paths.sh

## Build

We provided two hpvm compiled versions of the MiniERA application:
1. `hpvm-cpu` : Using the CPU backend in HPVM (without using the task library and the scheduler library).
2. `hpvm-epochs`: Using the EPOCHs backend in HPVM (which generates the api calls for the scheduler library).

### HPVM-CPU
To build MiniERA using the HPVM CPU backend:
1. `cd` to the root of the scheduler library repository.
2. run `make hpvm-cpu`
	-   _Note: The `setup_paths.sh` scripts must be sourced using  `source`  because it sets up environment variables that will be needed by the Makefiles._
3. To clean the build run `make clobber`

### HPVM-EPOCHS
To build MiniERA using the HPVM CPU backend:
1. `cd` to the root of the scheduler library repository.
2. run `make hpvm-epochs`
	-   _Note: The `setup_paths.sh` scripts must be sourced using  `source`  because it sets up environment variables that will be needed by the Makefiles._

3. To clean the build run `make clobber`
