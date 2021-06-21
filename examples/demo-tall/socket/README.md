# Mini-ERA Wi-Fi Stand-In Socket Server

This code defines socket servers used as stand-in for the Mini-ERA example demo driver application.
These socket servers accept input messages from the Mini-ERA application, and then run them through the GnuRadio
IEEE 802.11p encode and decode pipelines to produce the Viterbi Decoder inputs (as the previous Mini-ERA did voa the Viterbi Dictionary),
whcih they then send to the "other" car.

## Requirements

Mini-ERA SL has been successfully built and executed using the following set-up:
 - Ubuntu 18.04
 
Other platforms should also work.

## Installation and Execution
This code is part of the demo-me example of the EPOCHS Scheduler-Library, and install as part of that code, 

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library
make
```

Once the scheduler library and task libraries have been made (once), one can also make the Mini-ERA driver application from the ```examples/mini-era``` diretory.
```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library/sched_library
make
cd ../task_library
make
cd ../examples/demo-me
make
```

The `clean` and `clobber` targets can be used to ensure that all code is re-built, i.e. in case there are odd time-stamps on the files, etc.
The `make` command should produce the `test-scheduler*` target, which is the core executable that will run the C-mode <a href="https://github.com/IBM/mini-era" target="_blank">Mini-ERA</a> application atop SL.

The `make` process uses the contents of a `hardware.config` file to set various compile-time parameters; the contents of this file must be "compatible" with the scheduler library description of the hardware, e.g. application cannot request or expect hardware accelerators not present in the build of the scheduler-library (or on the hardware platform).  The default configuration is the scheduler library "local" configuration (via a soft-link).

The make produces two executables, 
This new naming convention is primarily used to make clearer the target plaftforms on which the executable can be expected to function.  The _all-software_ executable can execute on any platform.  The example above that uses hardware accelerators (`test-scheduler-RV-F2VCHo-P3V2F3N1`) can therefore be executed on a RISC-V based system that includes at least 2 hardware Viterbi, 3 hardware FFT, and 1 CV/CNN NVDLA accelerator.

### Targets

The standard ```make``` of the scheduler-library code will produce two sets of executables:
 - `wifi1_100_100_2.0.exe` : This is the Wi-Fi socket server for car 1 (see the parent demo-me README)
 - `wifi2_100_100_2.0.exe` : This is the Wi-Fi socket server for car 2 (see the parent demo-me README)
 - `t_wifi1_100_100_2.0.exe` : This is the Wi-Fi socket server for car 1 with internal timing information reporting
 - `t_wifi2_100_100_2.0.exe` : This is the Wi-Fi socket server for car 2 with internal timing information reporting

### Configuration

The make takes in configuration information, primarily about the grid-map (size) being communicated.


### Usage


#### Trace-Driven Version: `test-scheduler*`
```
./wifi1_100_100_2.0.exe -h
Usage: ./wifi1_100_100_2.0.exe <OPTIONS>
 OPTIONS:
    -h         : print this helpful usage info
    -W <str>   : set the internet-address for the WiFi server to <str>

## Status

This example is in development, and this socket-server Wi-Fi stand-in is siumilarly under current development.


## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
