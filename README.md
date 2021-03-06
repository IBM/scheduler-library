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
