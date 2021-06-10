#!/bin/bash
####### THESE VARIABLES NEED TO BE SET! #########
export HPVM_DIR=$HOME/work_dir/hpvm-dssoc/hpvm
#export APPROXHPVM_DIR=$HOME/work_dir/approxhpvm-nvdla
#export RISCV_BIN_DIR=$HOME/work_dir/riscv/bin

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
#export HPVM_BENCH_DIR=$HPVM_DIR/test/benchmarks
#export LLVM_SRC_ROOT=$APPROXHPVM_DIR/llvm
#export LLVM_BUILD_ROOT=$APPROXHPVM_DIR/build
##export LIBRARY_PATH=/software/cuda-9.1/lib64/:$LIBRARY_PATH
#export LD_LIBRARY_PATH=$LLVM_SRC_ROOT/lib/Transforms/HPVM2NVDLA/nvdla:$LD_LIBRARY_PATH
##export LD_LIBRARY_PATH=$LLVM_SRC_ROOT/lib/Transforms/HPVM2NVDLA/nvdla:/software/cuda-9.1/lib64/:$LD_LIBRARY_PATH
#export PATH=$HPVM_DIR/build/bin:$PATH
#export TOP=$MINIERA_DIR/sw/umd
