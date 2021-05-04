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
make clean
make
```
or to make just the Task-Scheduler Library portion:

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library/sched_lib
make clean
make
```

The `make clean` can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc. There is also a `make clobber` which removes the object-file directories, executables, etc. The `make` command should produce the `test-scheduler*` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

The `make` process uses the contents of a `.config` file to set various compile-time parameters.  The current version of this configuration is taken from the parent directory (i.e. the usage application in this case).

The resulting output is ```libscheduler.a``` which is a library containing the task-scheduler functionality.


## Status

This platform is meant for development and integration, and is under active development, so it is changing overt time.

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
