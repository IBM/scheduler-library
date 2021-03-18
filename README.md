# Scheduler Library (SL)

This is a Software Development Environment for initial development, deployment, and testing of a *smart scheduler*. This code provides the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> (FFT and Viterbi) kernels (i.e. the C-Subset of Mini-ERA) implemented atop a Scheduler Library (SL).  Calls to execute the FFT or Viterbi (accelerator functions) are now turned into calls to SL to `request_execution` of a task (either an `FFT_TASK` or a `VITERBI_TASK`) and SL will then schedule these tasks across the available function-execution hardware (e.g. on a CPU via a pthread, or on a hardware accelerator where those are implemented).

## Requirements

SL has been successfully built and executed using the following set-up:
 - Ubuntu 18.04
 - Ubuntu 16.04

Other platforms should also work; this implementation does NOT support (yet) the CV/CNN tensorflow model code, and thus does not require that support, etc.  This code does require gcc or similar/compatible compiler, an up to date pthreads library, and the C99 standard.

## Installation and Execution
The installation and execution are fairly standard, via github clone and makefiles:

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library
make clean
make
```

The `make clean` can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc. There is also a `make clobber` which removes the object-file directories, executables, etc. The `make` command should produce the `test-scheduler*.exe` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

The `make` process uses the contents of a `.config` file to set various compile-time parameters.  The current version of this software produces an executable target that includes some representation of these configuration parameters.  Thus, the output executable will likely have a name like `test-scheduler-CF-P3V0F0N0` which indicates:
 - this is the `test-scheduler` trace-driven (as opposed to `test-scheduler-sim` simulation-driven) run-type
 - the build uses the "fake" CV/CNN accelerators (`CF`)
 - the build allows for 'P3' three CPU accelerators (processors), 'V0' and zero hardware Viterbi accelerators, 'F0' and zero FFT accelerators, and 'N0' zero CV/CNN
When building for hardware that includes hardware accelerators (e.g. 3 FFT, 2 Viterbi, and 1 CV/CNN) then the name would reflect that, e.g. the target executable could read `test-scheduler-RV-F2VCHo-P3V2F3N1` which indicates:
 - it was cross-compiled for `RV` (RISC-V) target architecture
 - the build uses the `F2` fft2 hardware accelerator, 'V' Viterbi hardware accelerator, and `CHo` Hardware (NVDLA) only (i.e. no processor-based execution of CV/CNN tasks)
 - and the build allows for 'P3' three CPU accelerators (processors), 'V2' and two hardware Viterbi accelerators, 'F3' and three hardware FFT accelerators, and 'N1' one CV/CNN hardware NVDLA accelerator

This new naming convention is primarily used to make clearer the target plaftforms on which the executable can be expected to function.  The _all-software_ executable can execute on any platform.  The example above that uses hardware accelerators (`test-scheduler-RV-F2VCHo-P3V2F3N1`) can therefore be executed on a RISC-V based system that includes at least 2 hardware Viterbi, 3 hardware FFT, and 1 CV/CNN NVDLA accelerator.

### Targets

The standard ```make``` of the scheduler-library code will produce two executables:
 - `test-scheduler.exe`: corresponds to the trace-driven `mini-era` executable.
 - `test-scheduler-sim.exe`: corresponds to the simulation version of `mini-era` (where the inputs are derived by simulation of arrival events, guided by thresholds and random number selection).  For more details on the simulation versus trae-driven `mini-era` programs, please see the documentation in the mini-era github repository.

### Configuration

The Mini-ERA + SL build supports more than just a basic Linux platform; there is provision to tie this same application into the EPOCHS <a href="https://esp.cs.columbia.edu" target="_blank">ESP-based</a> SoC platform (currently propotyped on an FPGA). As such, there are options to compile this application code within the ESP SoC design environment to cross-compile an output target for the EPOCHS native RISC-V Linux SoC environment. There are also some additional configuration capabilites tied in to the overall make process, and these are controlled in two ways:

1. Build using an explicit Makefile, e.g. to build the local (native to this system) version, invoke `make -f Makefile.local` which will use the native gcc and compile to a version that does not include the use of ESP-based SoC hardware; to compile explicitly to the ESP-based SoC RISC-V system, build with `make -f Makefile.riscv`

2. Alter the contents of the `.config` file. The config file contains a set of defines used by the Makefile to produce the proper build style by default.

### The `.config` File Contents

The config file contains a series of definitions, like #define macros or environment variables, used to guide the default make behavior. These contents are:

- `DO_CROSS_COMPILATION=y`  indicates we are cross-compiling (uses the cross-compiler defined in Makefile)
- `COMPILE_TO_ESP=y`	  indicates we are compiling to target the ESP RISC-V SoC environment
- `CONFIG_ESP_INTERFACE=y`  this should always be set -- historical control to choose between some function interfaces.
- `CONFIG_FFT_EN=y`	  enable FFT Hardware Accelerators
- `CONFIG_FFT_FX=32`	  indicates FFT accelerators use 32-bit FXP format (can specify 64)
- `CONFIG_FFT_BITREV=y`	  indicates FFT accelerators include the bit-reverse operation
- `CONFIG_VITERBI_EN=y`	  enable Viterbi Decode Hardware Accelerators
- `CONFIG_KERAS_CV_BYPASS=y`	 turns off the Tensorflow code, etc. -- Leave this enabled!
- `CONFIG_VERBOSE=y`	  turns on a LOT of debugging output
- `CONFIG_DBG_THREADS=y`	  turns on debugging output for the threads 
- `CONFIG_GDB=y`		  indicates compilation should iclude the "-g" flag to retain symbols, etc. which provides for greater debugger support, etc.

### Usage

As indicated, there are two executables that will execute the Mini-ERA functionality on the SL, and their usage is very similar,
but with a few distinctions owing to the differences in running with a trace input as versus a simulated world  providing inputs.

#### Trace-Driven Version: `test-scheduler.exe`
```
./test-scheduler.exe -h
Usage: ./cmain.exe <OPTIONS>
 OPTIONS:
    -h         : print this helpful usage info
    -o         : print the Visualizer output traace information during the run
    -R <file>  : defines the input Radar dictionary file <file> to use
    -V <file>  : defines the input Viterbi dictionary file <file> to use
    -C <file>  : defines the input CV/CNN dictionary file <file> to use
    -t <trace> : defines the input trace file <trace> to use
    -p <N>     : defines the plan-and-control repeat factor (calls per time step -- default is 1)
    -f <N>     : defines which Radar Dictionary Set is used for Critical FFT Tasks
               :      Each Set ofRadar Dictionary Entries Can use a different sample size, etc.
    -F <N>     : Adds <N> additional (non-critical) FFT tasks per time step.
    -v <N>     : defines Viterbi message size:
               :      0 = Short messages (4 characters)
               :      1 = Medium messages (500 characters)
               :      2 = Long messages (1000 characters)
               :      3 = Max-sized messages (1500 characters)
    -M <N>     : Adds <N> additional (non-critical) Viterbi message tasks per time step.
    -S <N>     : Task-Size Variability: Varies the sizes of input tasks where appropriate
               :      0 = No variability (e.g. all messages same size, etc.)
    -P <N>     : defines the Scheduler Accelerator Selection Policy:
               :      0 = Select_Accelerator_Type_And_Wait
               :      1 = Fastest_to_Slowest_First_Available
               :      2 = Fastest_Finish_Time_First
               :      3 = Fastest_Finish_Time_First_Queued
```

To actually execute a trace, one must point to the trace in the trace repository (subdirectory ```traces```) using the ```-t``` option.

#### Simulation-Driven Version: ```test-scheduler-sim.exe```
```
./test-scheduler-sim.exe  -h
Usage: ./test-scheduler-sim.exe <OPTIONS>
 OPTIONS:
    -h         : print this helpful usage info
    -o         : print the Visualizer output traace information during the run
    -R <file>  : defines the input Radar dictionary file <file> to use
    -V <file>  : defines the input Viterbi dictionary file <file> to use
    -C <file>  : defines the input CV/CNN dictionary file <file> to use
    -s <N>     : Sets the max number of time steps to simulate
    -r <N>     : Sets the rand random number seed to N
    -A         : Allow obstacle vehciles in All lanes (otherwise not in left or right hazard lanes)
    -W <wfile> : defines the world environment parameters description file <wfile> to use
    -p <N>     : defines the plan-and-control repeat factor (calls per time step -- default is 1)
    -f <N>     : defines which Radar Dictionary Set is used for Critical FFT Tasks
               :      Each Set ofRadar Dictionary Entries Can use a different sample size, etc.
    -F <N>     : Adds <N> additional (non-critical) FFT tasks per time step.
    -v <N>     : defines Viterbi message size:
               :      0 = Short messages (4 characters)
               :      1 = Medium messages (500 characters)
               :      2 = Long messages (1000 characters)
               :      3 = Max-sized messages (1500 characters)
    -M <N>     : Adds <N> additional (non-critical) Viterbi message tasks per time step.
    -S <N>     : Task-Size Variability: Varies the sizes of input tasks where appropriate
               :      0 = No variability (e.g. all messages same size, etc.)
    -P <N>     : defines the Scheduler Accelerator Selection Policy:
               :      0 = Select_Accelerator_Type_And_Wait
               :      1 = Fastest_to_Slowest_First_Available
               :      2 = Fastest_Finish_Time_First
               :      3 = Fastest_Finish_Time_First_Queued
```

For execution in simulation mode (e.g. using ```test-scheduler-sim.exe```) no trace is necessary, and the simulation provides the inputs.

## Status

This platform is meant for SL development and integration, so it is expected to change over time. Currently, this is a relatively complete but bare-bones trace version Mini-ERA implementation. Additional features of the Mini-ERA code, and extensions thereto should also be developed over time.

There are currently some example traces in the `traces` subdirectory. Note that most development of Mini-ERA and its off-shoots to date has focused around the `tt02.new` trace.

 - `tt00.new` is a 5000 record illustrative trace.
 - `tt001new` is a 5000 record illustrative trace.
 - `tt002new` is a 5000 record trace that includes multiple obstacle vehicles per lane (at times).
 - `tt003new` is a 5000 record trace that includes multiple obstacle vehicles per lane (at times).

There is a default set of world parameter setings for the simulation-driven mode (```test-scheduler-sim.exe```) as well.
This file (```default-world.desc```) describes the thresholds for various input occurrences, etc.

For additional information on all aspects of the Mini-ERA baseline workload,
please see the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> main-line README.


## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
