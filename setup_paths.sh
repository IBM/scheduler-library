#!/bin/bash
####### THESE VARIABLES NEED TO BE SET! #########
export HPVM_DIR=/home/espuser/hpvm-release/hpvm
export RISCV_BIN_DIR=/home/espuser/riscv/bin
export ESP_ROOT=/home/espuser/esp


####### THESE VARIABLES SHOULD NOT NEED ANY MODIFICATION! #########
SH="$(readlink -f /proc/$$/exe)"
if [[ "$SH" == "/bin/zsh" ]]; then
  CUR_DIR="${0:A:h}"
else
  CUR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
fi
export SCHED_LIB_DIR=$CUR_DIR/sched_library
export TASK_LIB_DIR=$CUR_DIR/task_library
export HPVM_BUILD_DIR=$HPVM_DIR/build
export LD_LIBRARY_PATH=$HPVM_DIR/build/lib:$LD_LIBRARY_PATH

export PATH=$PATH:/$RISCV_BIN_DIR
