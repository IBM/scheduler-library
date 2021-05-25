# Scheduler Library Example Driver Applications

This directory holds a set of example Scheduler-Library using/driving applications.
Each application is a separate application, and should compile independently of the Scheduler Library (linking in the ```libscheduler.a``` at this time from the parent's ```libreary``` directory).

For more information on individual driver example,s look for a README in each of the example driver directories.

This directory includes a top-level (driver examples) Makefile, whcih can be use3d to compile all the examples.

## Requirements

SL has been successfully built and executed using the following set-up:
 - Ubuntu 18.04
 - Ubuntu 16.04
 - Centos7
 
Other platforms should also work.
DTthis implementation currently does NOT support invocation of the hardware CV/CNN model code, or the (external, python-based) software CV/CNN tensorflow model code,
and thus does not require that support, etc.  This code does require gcc or similar/compatible compiler, an up to date pthreads library, and the C99 standard.

## Compilation
The installation is part of the Scheduler Library project.  One can make the example using the top-level makefile, or using this makefile.

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library/library
make
cd ../examples
make
```

## Status

This platform is meant for SL development and integration, so it is expected to change over time. Currently, this is a relatively complete version of the Task Scheduling
functiuonality, and the Mini-ERA driver application is similarly complete.

The Scheduler Library is under constant development (and hopefully improvement).

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
