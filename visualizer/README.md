# Scheduler Library (SL) Visualizer

This is a simple Gantt-Chart type Visualization tool for use with the Scheudler-Library.
The Scheduler supports the output of a visualization trace file (see option ```-O``` et. al.) which can be used as the input to the
Scheduler Visualizer tool.  

## Requirements

The SL-Visualizer used Python-3, and has been successfully run on 
 - Ubuntu 18.04
 - Ubuntu 16.04
 - Centos-7

Other platforms should also work; this is basic Python-3 code, etc.
Most of the needed code is resident in the ```vusalizer/python-gantt``` directory, but this also makes use of the python ```pandas``` package.


## Installation and Execution

Once the Scheduler-Library distribution is downloaded, no further installation should be required.
Execution, once a trace file has been obtained from an application run using the Scheduler, simply requires the invocation of the sl-0viz.py program under Pytho9n-3, passing in the location of the trace file.  Note that the "directory" path portion of the trace file location is required, so for an soutput SL-Visualizer trace file named ```sl-viz/trace``` located in the current directory, the invocation is illustrated below.

```
git clone https://github.com/IBM/scheduler-library.git
cd scheduler-library/
python3 visualizer/sl-viz.py ./sl_viz/trace
```


### Outputs

The output of the Scheduler-Library Visualizer is a set of ```.svg``` files:
 - no_group.svg : this provides visualization output that displays the Task "lifetime" in their finish order.
 - criticality_group.svg : this provides visualization output that groups the tasks by criticality level (Base, Critical, etc.) and within each such group they are represented in finish order.
 - server_type_group.svg : this provides visualization output that groups the tasks by the type of server, with tasks represented in each server group by the order of their finish.
 - server_type_flat_group.svg : this provides visualization output that groups the tasks by the type of server, but "flattens" the representation (e.g. so all four PEs of a given type are a single row).  

### Viewing the Outputs

The output visualization is readily viewable with any tool that displays SVG images.  The simplest method is usually to open the file in a web browser.

## Status

As with the entire Shceduler-Library, the visualizer is under constant development.

## Contacts and Current Maintainers

 - J-D Wellman (wellman@us.ibm.com)
 - Augusto Vega (ajvega@us.ibm.com)
