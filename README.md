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
git clone https://github.com/IBM/mini-era.git
make clean
make
```

The `make clean` can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc. The `make` command should produce the `test-scheduler.exe` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

### Configuration

The Mini-ERA + SL build supports more than just a basic Linux platform; there is provision to tie this same application into the EPOCHS <a href="https://esp.cs.columbia.edu" target="_blank">ESP-based</a> Soc platform (currently propotyped on an FPGA). As such, there are options to compile this application code within the ESP SoC design environment to cross-compile an output target or the EPOCHS native RISC-V Linux SoC environment. There are also some additional configuration capabilites tied in to the overall make process, and these are controlled in two ways:

1. Build using an explicit Makefile, e.g. to build the local (native to this system) version, invoke `make -f Makefile.local` which will use the native gcc and compile to a version that does not include the use of ESP-based SoC hardware; to compile explicitly to the ESP-based SoC RISC-V system, build with `make -f Makefile.riscv`

2. Alter the contents of the `.config` file. The config file contains a set of defines used by the Makefile to produce the proper build style by default.

### The `.config` File Contents

The config file contains a series of definitions, like #define macros or environment variables, wused to guide the default make behavior. These contents are:

- DO_CROSS_COMPILATION=y  indicates we are cross-compiling (uses the cross-compiler defined in Makefile)
- COMPILE_TO_ESP=y	  indicates we are compiling to target the ESP RISC-V SoC environment
- CONFIG_ESP_INTERFACE=y  this should always be set -- historical control to choose between some function interfaces.
- CONFIG_FFT_EN=y	  enable FFT Hardware Accelerators
- CONFIG_FFT_FX=32	  indicates FFT accelerators use 32-bit FXP format (can specify 64)
- CONFIG_FFT_BITREV=y	  indicates FFT accelerators include the bit-reverse operation
- CONFIG_VITERBI_EN=y	  enable Viterbi Decode Hardware Accelerators
- CONFIG_KERAS_CV_BYPASS=y	 turns off the Tensorflow code, etc. -- Leave this enabled!
- CONFIG_VERBOSE=y	  turns on a LOT of debugging output
- CONFIG_DBG_THREADS=y	  turns on debugging output for the threads 
- CONFIG_GDB=y		  indicates compilation should iclude the "-g" flag to retain symbols, etc. which provides for greater debugger support, etc.

### Usage
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
```

To actually execute a trace, one must point to the trace repository. SL does not include a trace directory itself, but instead uses the one from <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a>.

## Status

This platform is meant for SL development and integration, so it is expected to change over time. Currently, this is a relatively complete but bare-bones trace version Mini-ERA implementation. Additional features of the Mini-ERA code, and extensions thereto should also be developed over time.

There are currently some example traces in the `traces` subdirectory. Note that most development of Mini-ERA and its off-shoots to date has focused around the `tt02.new` trace.

 - `tt00.new` is a 5000 record illustrative trace.
 - `tt001new` is a 5000 record illustrative trace.
 - `tt002new` is a 5000 record trace that includes multiple obstacle vehicles per lane (at times).
 - `tt003new` is a 5000 record trace that includes multiple obstacle vehicles per lane (at times).

For additional information, please see the <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> main-line README.


## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
