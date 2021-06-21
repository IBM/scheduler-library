# Scheduler Library (SL)

This is a Software Development Environment for the development, deployment, and testing of a *smart scheduler*. This code provides the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application (FFT and Viterbi kernels -- i.e. the C-Subset of Mini-ERA) implemented atop a Scheduler Library (SL).  Calls to execute the FFT or Viterbi (accelerator functions) are now turned into requests for the Scheduler to execute the task, and the Scheduler will then schedule these tasks across the available function-execution hardware (e.g. on a CPU via a pthread, or on a hardware accelerator where those are implemented).

NOTE: The code has been a bit reorganized, and this README is currently a bit out of date.

The Scheduler Library code is in the ```sched_lib``` subdir, the Mini-ERA example driver application is now under ```examples/mini-era``` and
we have worked up some new Makefiles to allow builds to proceed relatively normally, but this re-org is on-going for the moment...

The rest of this information still holds, but covers elements of both teh SL and the Mini-ERA driver example intermixed, for now...

## Requirements

SL has been successfully built and executed using the following set-up:
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

The `make clean` can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc. There is also a `make clobber` which removes the object-file directories, executables, etc. The `make` command should produce the `test-scheduler*` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

The `make` process moves therough the sub-components and makes each in turn; this is a global top-level make to make not only the
Scheduler Librady, but also teh example driver applications.
The make for the individual components is described in their own directories.

## Code/Layout

This code provides a "Scheduler Library" and example usage applications that use it.

The "Scheduler-Library" code is contained under the ```library``` subdirectory.  There is a README there as which focuses more on the Scheduler Library directly.
The "scheduler Library" should be seen as an entirely separate body of code that produces a "service platform" to which an application can connect and utilize those services.  This is the direction we are heading this project.

The example drivers each reside in a subdirectory under the ```examples``` directory; currently the primary example driver is the ```mini-era``` workload, whcih is providede therein.  Each example driver application should include its own makefile, README, etc. and is expected to contain the full application that uses the Scheduler Library. See the ```mini-era``` README for more information about the Mini-ERA workload, etc.

## Using the Scheduler Library from an Application

This distribution contains examples that use the Scheduler (via the Scheduler-Library), and they can provide a bit of a tutorial on integrating to the Scheduler.

The basic Scheudler interface is defined in the 
<a href="https://github.com/IBM/scheduler-library/tree/master/library">Scheduler Library</a> in the file
<a href="https://github.com/IBM/scheduler-library/tree/master/library/include/scheduler.h">scheduler.h</a>.
We encourage you to look at that file for a genreal description and overview of the Scheduler API.


The basic use of the Scheduler Library is illustrated in the
<a href="https://github.com/IBM/scheduler-library/tree/master/examples/mini-era/src/main.c">main.c</a> file, with additional contrent (e.g. the
definition of tasks, etc.) in the ```*_task.[ch]``` files, e.g. 
<a href="https://github.com/IBM/scheduler-library/tree/master/examples/mini-era/src/fft_task.c">fft_task.c</a> file as well.

This section needs revision; we are simplifying the Scheduler Library API, merging older API calls, etc.
We will add a high-level descriptiuon of its use here once we've stabilized the latest revisions.


## Status

This platform is meant for SL development and integration, so it is expected to change over time. Currently, this is a relatively complete version of the Task Scheduling
functiuonality, and the Mini-ERA driver application is similarly complete.

The Scheduler Library is under constant development (and hopefully improvement).

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)



# MINIERA-HPVM

## HPVM Setup

-   HPVM (internal repository branch  `hpvm-epochs0-backend`  located  [here](https://gitlab.engr.illinois.edu/llvm/hpvm/-/tree/hpvm-epochs0-backend))
    -   Refer to  [HPVM Build Instructions](https://hpvm.readthedocs.io/en/latest/build-hpvm.html)  for detailed set up instructions.  _Note: During installation, make sure target is set to X86;RISCV to be able to target the EPOCHS-0 RISC-V processor._

-   GCC cross compiler for RISC-V, can be installed using ESP as follows:
    -   Clone ESP repository using:  `git clone --recursive https://github.com/sld-columbia/esp.git`
    -   Checkout the epochs branch:  `cd esp && git checkout epochs`
    -   Invoke the cross-compiler installation script:  `./utils/scripts/build_riscv_toolchain.sh`

## HPVM Installation and Execution

First we must clone and set up the hpvm repository:

    git clone --branch hpvm-epochs0-backend https://gitlab.engr.illinois.edu/llvm/hpvm.git

Install HPVM via the provided install script and follow the provided prompts:

    cd ${HPVM_SRC_ROOT}/hpvm/hpvm/
    ./install.sh

## Setting up required paths
After installation source the `set_paths.sh` script to export the environment variables needed for running the hpvm scheduler backend for MiniERA.

    cd ${HPVM_SRC_ROOT}/hpvm/hpvm/
    source set_paths.sh
Then update the `HPVM_DIR` variable definition in `${SCHED_LIB_SRC_ROOT}/setup_paths.sh` to be the path to the directory containing the build directory for HPVM. After which:

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
