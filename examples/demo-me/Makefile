include ../../hardware.config

CPU ?= ariane
ARCH ?= riscv
ifdef DO_CROSS_COMPILATION
 CROSS_COMPILE ?= riscv64-unknown-linux-gnu-
 TOOLCHAIN_PREFIX ?= $(RISCV_BIN_DIR)/riscv64-unknown-linux-gnu-
 EXE_EXTENSION=-RV
 CROSS_COMPILE_FLAGS = --target=riscv64 -march=rv64g -mabi=lp64d
endif

ifdef COMPILE_TO_ESP
 ESP_ROOT ?= $(realpath ../../../../esp)
 ESP_DRIVERS ?= $(ESP_ROOT)/soft/common/drivers
 ESP_DRV_LINUX  = $(ESP_DRIVERS)/linux
endif

CC = gcc -std=c99

HPVM_BIN_PATH = $(HPVM_BUILD_DIR)/bin
OPT = $(HPVM_BIN_PATH)/opt
LLVM_LINK = $(HPVM_BIN_PATH)/llvm-link
HPVMCC = $(HPVM_BIN_PATH)/clang
HPVM_RT_PATH = $(HPVM_BUILD_DIR)/tools/hpvm/projects/hpvm-rt
HPVM_RT_LIB = $(HPVM_RT_PATH)/hpvm-rt.bc

YEL='\033[0;33m'
NC='\033[0m'



INCDIR ?=
INCDIR += -I./include -I../../sched_library/include -I../../task_library/include 
INCDIR += -I./socket/include 
ifdef COMPILE_TO_ESP
 INCDIR += -I$(ESP_DRIVERS)/common/include
 INCDIR += -I$(ESP_DRIVERS)/linux/include
endif

CFLAGS ?= -O2 #-g
CFLAGS += $(INCDIR)
CFLAGS += -DINT_TIME
ifdef COMPILE_TO_ESP
 CFLAGS += -DCOMPILE_TO_ESP
endif
#  -- ALWAYS use this one! --   ifdef CONFIG_ESP_INTERFACE
CFLAGS += -DUSE_ESP_INTERFACE
#   endif

# This sets the maximum number of any Acceleartor type in this build
#  This should be equal to the largest number of any of FFT, VIT, CV, etc.
#  But it could be a larger value as well
#ifdef CONFIG_MAX_ACCEL_ANY_TYPE
# CFLAGS += -DMAX_ACCEL_OF_EACH_TYPE=$(CONFIG_MAX_ACCEL_ANY_TYPE)
#endif

# This selects the number of each Acceleartor type:
# MAX for the CPU is up to you -- that is "spawned CPU threads"
# MAX for others is determined by the hardware run on
ifdef CONFIG_NUM_CPU
 CFLAGS += -DNUM_CPU_ACCEL=$(CONFIG_NUM_CPU)
endif
ifdef CONFIG_NUM_VIT
 CFLAGS += -DNUM_VIT_ACCEL=$(CONFIG_NUM_VIT)
endif
ifdef CONFIG_NUM_FFT
 CFLAGS += -DNUM_FFT_ACCEL=$(CONFIG_NUM_FFT)
endif
ifdef CONFIG_NUM_CV
 CFLAGS += -DNUM_CV_ACCEL=$(CONFIG_NUM_CV)
endif

# This adds the code to allow the user to specify (at run-time)
#  some additional base-level (not-critical) tasks of Vit, CV, FFT
ifdef ALLOW_ADD_BASE_TASKS
 CFLAGS += -DHPVM_BASE_CRIT
endif

CFLAGS += -DMAX_RADAR_LOGN=$(LOG2_MAX_FFT_SAMPLES)

SW_STR = S
FA_STR =
VA_STR =
CA_STR =
CB_STR =
ifdef CONFIG_FFT_EN
 SW_STR =
 FA_STR = F$(CONFIG_FFT_ACCEL_VER)
 CFLAGS += -DHW_FFT
 CFLAGS += -DUSE_FFT_FX=$(CONFIG_FFT_FX)
 CFLAGS += -DUSE_FFT_ACCEL_VERSION=$(CONFIG_FFT_ACCEL_VER)
 CFLAGS += -DFFT_DEV_BASE='"$(FFT_DEVICE_BASE)"'
endif
ifdef CONFIG_FFT_BITREV
 CFLAGS += -DHW_FFT_BITREV
endif

ifdef CONFIG_VITERBI_EN
 SW_STR =
 VA_STR = V
 CFLAGS += -DHW_VIT
 CFLAGS += -DVIT_DEV_BASE='"$(VIT_DEVICE_BASE)"'
endif

ifdef CONFIG_KERAS_CV_BYPASS
 CFLAGS += -DBYPASS_KERAS_CV_CODE
else
 CFLAGS += -I/usr/include/python2.7
endif

ifdef CONFIG_CV_EN
 SW_STR =
 CA_STR = CH
 CFLAGS += -DHW_CV -DENABLE_NVDLA
 #CFLAGS += -DCNN_DEV_BASE='"$(CNN_DEVICE_BASE)"' -I/usr/include/python2.7
else # Not using the HWR accelerator
 ifdef CONFIG_FAKE_CV_EN
  SW_STR =
  CA_STR = CF
  CFLAGS += -DFAKE_HW_CV
 endif
endif

ifdef CONFIG_CV_ONLY_HWR
 SW_STR =
 CB_STR = o
 CFLAGS += -DHW_ONLY_CV
endif

ifdef SL_VIZ
 CFLAGS += -DSL_VIZ
endif

ifdef CONFIG_VERBOSE
 CFLAGS += -DVERBOSE
 #OPTFLAGS += -debug
endif

ifdef CONFIG_DBG_THREADS
 CFLAGS += -DDBG_THREADS
endif

ifdef CONFIG_SUPER_VERBOSE
 CFLAGS += -DSUPER_VERBOSE
endif

ifdef CONFIG_FUSED_MAP
 CFLAGS += -DWRITE_FUSED_MAPS
endif
ifdef CONFIG_GDB
 CFLAGS += -g
endif



SCHED_LIB = $(SCHED_LIB_DIR)/libscheduler.a
TASK_LIB = $(TASK_LIB_DIR)/libtasks.a
TASK_LIB_W = $(TASK_LIB_DIR)/task_library/libtasksW.a

SCHED_MODULE = $(SCHED_LIB_DIR)/libscheduler.bc
TASK_MODULE = $(TASK_LIB_DIR)/libtasks.bc
TASK_MODULE_W = $(TASK_LIB_DIR)/libtasksW.bc

LDLIBS ?=
ifdef COMPILE_TO_ESP
 ESP_BUILD_DRIVERS=../../sched_library/esp-build/drivers
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/contig_alloc
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/test
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/libesp
endif
#ifndef CONFIG_KERAS_CV_BYPASS
# LDLIBS += 
#endif
LDLIBS += -L../../task_library -L../../sched_library

MYLIBS   += -ltasks -lscheduler
MYLIBS_W += -ltasksW -lscheduler

LDFLAGS ?=
LDFLAGS += -lm
LDFLAGS += -lpthread
LDFLAGS += -ldl -rdynamic
ifdef COMPILE_TO_ESP
 LDFLAGS += -lrt
 LDFLAGS += -lesp
 LDFLAGS += -ltest
 LDFLAGS += -lcontig
endif
ifndef CONFIG_KERAS_CV_BYPASS
 LDFLAGS += -lpython2.7
endif

SRC = $(foreach f, $(wildcard src/*.c), $(shell basename $(f)))
SRC_T = $(filter-out sim_environs.c, $(SRC))
SRC_S = $(filter-out read_trace.c, $(SRC))
SRC_U = sim_environs.c gen_trace.c
SRC_D = $(wildcard src/*.c)
HDR_T = $(wildcard include/*.h)
OBJ_T1 = $(SRC_T:%.c=obj_t1/%.o)
OBJ_S1 = $(SRC_S:%.c=obj_s1/%.o)
OBJ_T2 = $(SRC_T:%.c=obj_t2/%.o)
OBJ_S2 = $(SRC_S:%.c=obj_s2/%.o)
OBJ_T1_W = $(SRC_T:%.c=obj_t1w/%.o)
OBJ_S1_W = $(SRC_S:%.c=obj_s1w/%.o)
OBJ_T2_W = $(SRC_T:%.c=obj_t2w/%.o)
OBJ_S2_W = $(SRC_S:%.c=obj_s2w/%.o)
OBJ_U = $(SRC_U:%.c=obj_u/%.o)
OBJ_U_W = $(SRC_U:%.c=obj_uw/%.o)

$(info $$SRC is [${SRC}])
$(info $$SRC_T is [${SRC_T}])

OBJ_LL1 = $(filter-out obj_hpvm1/hpvm_tasks.ll, $(SRC_T:%.c=obj_hpvm1/%.ll))
OBJ_H1 = obj_hpvm1/hpvm_tasks.host.ll
OBJ_LINKED1 = obj_hpvm1/hpvm_tasks.linked.ll

OBJ_LL2 = $(filter-out obj_hpvm2/hpvm_tasks.ll, $(SRC_T:%.c=obj_hpvm2/%.ll))
OBJ_H2 = obj_hpvm2/hpvm_tasks.host.ll
OBJ_LINKED2 = obj_hpvm2/hpvm_tasks.linked.ll

OBJ_LL1_W = $(filter-out obj_hpvm1_W/hpvm_tasks.ll, $(SRC_T:%.c=obj_hpvm1_W/%.ll))
OBJ_H1_W = obj_hpvm1_W/hpvm_tasks.host.ll
OBJ_LINKED1_W = obj_hpvm1_W/hpvm_tasks.linked.ll

OBJ_LL2_W = $(filter-out obj_hpvm2_W/hpvm_tasks.ll, $(SRC_T:%.c=obj_hpvm2_W/%.ll))
OBJ_H2_W = obj_hpvm2_W/hpvm_tasks.host.ll
OBJ_LINKED2_W = obj_hpvm2_W/hpvm_tasks.linked.ll

VPATH = ./src ./src/readers ./src/utils

TARGET1=car1-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
STARGET1=sim-car1-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)

TARGET2=car2-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
STARGET2=sim-car2-scheduler$(EXE_EXTENSION)-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)

TARGET1_W=car1-scheduler$(EXE_EXTENSION)-W-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
STARGET1_W=sim-car1-scheduler$(EXE_EXTENSION)-W-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)

TARGET2_W=car2-scheduler$(EXE_EXTENSION)-W-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)
STARGET2_W=sim-car2-scheduler$(EXE_EXTENSION)-W-$(SW_STR)$(FA_STR)$(VA_STR)$(CA_STR)$(CB_STR)-P$(CONFIG_NUM_CPU)V$(CONFIG_NUM_VIT)F$(CONFIG_NUM_FFT)N$(CONFIG_NUM_CV)

#$(info $$EXE_EXTENSION is [${EXE_EXTENSION}])
#$(info $$TARGET1 is [${TARGET1}])
#$(info $$TARGET1_W is [${TARGET1_W}])

HPVMTARGET1=hpvm-$(TARGET1)
HPVMTARGET2=hpvm-$(TARGET2)
HPVMTARGET1_W=hpvm-$(TARGET1_W)
HPVMTARGET2_W=hpvm-$(TARGET2_W)
$(info $$HPVMTARGET1 is [${HPVMTARGET1}])
$(info $$HPVMTARGET1_W is [${HPVMTARGET1_W}])

#OBJ_H1 = obj_hpvm1/hpvm_tasks.host.ll
#OBJ_LINKED1 = obj_hpvm1/hpvm_tasks.linked.ll
#OBJ_H2 = obj_hpvm2/hpvm_tasks.host.ll
#OBJ_LINKED2 = obj_hpvm2/hpvm_tasks.linked.ll
#OBJ_H1_W = obj_hpvm1_W/hpvm_tasks.host.ll
#OBJ_LINKED1_W = obj_hpvm1_W/hpvm_tasks.linked.ll
#OBJ_H2_W = obj_hpvm2_W/hpvm_tasks.host.ll
#OBJ_LINKED2_W = obj_hpvm2_W/hpvm_tasks.linked.ll


#all: $(TARGET1) $(STARGET1) $(TARGET2) $(STARGET2) $(TARGET1_W) $(STARGET1_W) $(TARGET2_W) $(STARGET2_W) sockets gen_new_trace gen_new_wide_trace
all: $(TARGET1) $(TARGET2) $(TARGET1_W) $(TARGET2_W) $(STARGET1) $(STARGET2) $(STARGET1_W) $(STARGET2_W) sockets gen_new_trace gen_new_wide_trace

$(OBJ_T1): $(HDR_T)

$(OBJ_T2): $(HDR_T)

$(OBJ_S1): $(HDR_S)

$(OBJ_S2): $(HDR_S)

$(OBJ_T1_W): $(HDR_T)

$(OBJ_T2_W): $(HDR_T)

$(OBJ_S1_W): $(HDR_S)

$(OBJ_S2_W): $(HDR_S)


$(TARGET1): $(TASK_LIB) $(SCHED_LIB) obj_t1 $(OBJ_T1) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_T1) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(STARGET1): $(TASK_LIB) $(SCHED_LIB) obj_s1 $(OBJ_S1) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_S1) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(TARGET2): $(TASK_LIB) $(SCHED_LIB) obj_t2 $(OBJ_T2) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_T2) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(STARGET2): $(TASK_LIB) $(SCHED_LIB) obj_s2 $(OBJ_S2) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_S2) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(TARGET1_W): $(TASK_LIB) $(SCHED_LIB) obj_t1w $(OBJ_T1_W) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_T1_W) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(STARGET1_W): $(TASK_LIB) $(SCHED_LIB) obj_s1w $(OBJ_S1_W) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_S1_W) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(TARGET2_W): $(TASK_LIB) $(SCHED_LIB) obj_t2w $(OBJ_T2_W)$(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_T2_W) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o

$(STARGET2_W): $(TASK_LIB) $(SCHED_LIB) obj_s2w $(OBJ_S2_W) $(NVDLA_MODULE) $(ALLMODULE_OBJS)
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) $(OBJ_S2_W) -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	$(RM) -f wnvdla_test.o


sockets:
	(cd socket; $(MAKE))


gen_new_trace: obj_u $(OBJ_U)
	$(CROSS_COMPILE)$(CC) $(LDLIBS) $(OBJ_U) -o $@ $(MYLIBS) $(LDFLAGS)

gen_new_wide_trace: obj_uw $(OBJ_U_W)
	$(CROSS_COMPILE)$(CC) $(LDLIBS) $(OBJ_U_W) -o $@ $(MYLIBS_W) $(LDFLAGS)


hpvm-epochs: CFLAGS += -DHPVM -DDEVICE=EPOCHS_TARGET
#hpvm-epochs: CFLAGS := $(filter-out -I../../sched_library/include -I../../task_library/include, $(CFLAGS))
hpvm-epochs: CONFIG_FILE ?= config_files/base_me_p2.config 
hpvm-epochs: TASK_CONFIG_FILE ?= $(TASK_LIB_DIR)/task_lib.config 
hpvm-epochs: BACKEND_LOAD = -load LLVMDFG2LLVM_EPOCHS.so
hpvm-epochs: BACKEND_FLAG = -dfg2llvm-epochs -sched-lib-path=$(SCHED_MODULE) -sched-config=$(CONFIG_FILE) -task-config=$(TASK_CONFIG_FILE) -task-lib-path=$(TASK_MODULE)
hpvm-epochs: HPVM = hpvm

hpvm-cpu: CFLAGS += -DHPVM -DDEVICE=CPU_TARGET
#hpvm-cpu: CFLAGS := $(filter-out -I../../sched_library/include -I../../task_library/include, $(CFLAGS))
hpvm-cpu: CFLAGS := $(filter-out -I../../sched_library/include , $(CFLAGS))
hpvm-cpu: BACKEND_LOAD = -load LLVMDFG2LLVM_CPU.so
hpvm-cpu: BACKEND_FLAG = -dfg2llvm-cpu 
hpvm-cpu: LDFLAGS := $(filter-out -ltasks -lscheduler, $(LDFLAGS))
hpvm-cpu: LDLIBS := $(filter-out -L../../task_library -L../../sched_library, $(LDLIBS))


hpvm-epochs: check_env $(SCHED_MODULE) $(TASK_MODULE) $(NVDLA_MODULE) obj_hpvm1 $(HPVMTARGET1) 
#hpvm-epochs: check_env obj_hpvm1 obj_hpvm2 obj_hpvm1_W obj_hpvm2_W $(SCHED_MODULE) $(TASK_MODULE) $(NVDLA_MODULE) $(HPVMTARGET1) $(HPVMTARGET2) $(HPVMTARGET1_W) $(HPVMTARGET2_W)
hpvm-cpu: check_env obj_hpvm1 obj_hpvm2 obj_hpvm1_W obj_hpvm2_W $(HPVMTARGET1) $(HPVMTARGET2) $(HPVMTARGET1_W) $(HPVMTARGET2_W)


#------------------------------------------------------------------------------------------

#------------------------------------------------------------------------------------------


CUR_DIR = $(dir $(realpath $(firstword $(MAKEFILE_LIST))))
$(info $$CUR_DIR is [${CUR_DIR}])

ROOT := $(CUR_DIR)/sw/umd
TOP := $(ROOT)
$(info $$ROOT is [${ROOT}])

ESP_NVDLA_DIR = esp_hardware/nvdla
#INCLUDES +=  -I$(SRC_DIR) -I$(ESP_NVDLA_DIR) -I$(TOP)/core/include
INC_DIR +=  -I$(ESP_NVDLA_DIR) -I$(ROOT)/core/include

NVDLA_MAKE_DIR = $(HPVM_DIR)/test/epoch_dnn
NVDLA_RES_DIR = $(NVDLA_MAKE_DIR)/res/miniera

NVDLA_RUNTIME_DIR = $(ROOT)
NVDLA_RUNTIME = $(NVDLA_RUNTIME_DIR)/out
$(info $$NVDLA_RUNTIME is [${NVDLA_RUNTIME}])
$(info $$NVDLA_RUNTIME_DIR is [${NVDLA_RUNTIME_DIR}])

ifdef CONFIG_CV_EN

MODULE := nvdla_runtime

include $(ROOT)/make/macros.mk

BUILDOUT ?= $(ROOT)/out/apps/runtime
BUILDDIR := $(BUILDOUT)/$(MODULE)
TEST_BIN := $(BUILDDIR)/$(MODULE)

MODULE_COMPILEFLAGS := -W -Wall -Wno-multichar -Wno-unused-parameter -Wno-unused-function -Werror-implicit-function-declaration
MODULE_CFLAGS := --std=c99
MODULE_CPPFLAGS := --std=c++11 -fexceptions -fno-rtti
NVDLA_FLAGS := -pthread -L$(ROOT)/external/ -ljpeg -L$(ROOT)/out/core/src/runtime/libnvdla_runtime -lnvdla_runtime  -Wl,-rpath=.

include esp_hardware/nvdla/rules.mk
endif

$(info $$ALLMODULE_OBJS is [${ALLMODULE_OBJS}])
$(info $$MODULE_OBJS is [${MODULE_OBJS}])
#-----------------------------------------------------------------------------------------------

$(NVDLA_MODULE):
	@echo -e ${YEL}Compiling NVDLA Runtime Library${NC}
	@cd $(NVDLA_RUNTIME_DIR) && make ROOT=$(ROOT) runtime
	@echo -e ${YEL}Compiling HPVM Module for NVDLA${NC}
	cd $(NVDLA_MAKE_DIR) && ln -sf $(CUR_DIR)/user_scripts/gen_me_hpvm_mod.py . && python gen_me_hpvm_mod.py && cp $(NVDLA_RES_DIR)/$(NVDLA_MODULE) $(CUR_DIR)
	#@cd $(NVDLA_MAKE_DIR) && python gen_me_hpvm_mod.py && cp $(NVDLA_RES_DIR)/$(NVDLA_MODULE) $(CUR_DIR)

#-----------------------------------------------------------------------------------------------

$(HPVMTARGET1): $(OBJ_LINKED1) $(TASK_LIB) $(SCHED_LIB) $(ALLMODULE_OBJS)
	$(HPVMCC) $(CROSS_COMPILE_FLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=5 -fPIC $< -c -o dme_test.o
	$(CROSS_COMPILE)$(LD) -r $(ALLMODULE_OBJS) dme_test.o -o wnvdla_test.o
	$(CROSS_COMPILE)$(CXX) $(LDLIBS) wnvdla_test.o -o $@ $(MYLIBS) $(LDFLAGS) $(NVDLA_FLAGS)
	rm dme_test.o wnvdla_test.o

#-----------------------------------------------------------------------------------------------

obj_t1/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=5 -c $< -o $@

obj_s1/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DUSE_SIM_ENVIRON -DBUILD_WITH_N_LANES=5 -c $< -o $@


obj_t2/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DBUILD_WITH_N_LANES=5 -c $< -o $@

obj_s2/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DUSE_SIM_ENVIRON -DBUILD_WITH_N_LANES=5 -c $< -o $@

obj_u/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DUSE_SIM_ENVIRON -DBUILD_WITH_N_LANES=5 -c $< -o $@


obj_t1w/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=9 -c $< -o $@

obj_s1w/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DUSE_SIM_ENVIRON -DBUILD_WITH_N_LANES=9 -c $< -o $@


obj_t2w/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DBUILD_WITH_N_LANES=9 -c $< -o $@

obj_s2w/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DUSE_SIM_ENVIRON -DBUILD_WITH_N_LANES=9 -c $< -o $@

obj_uw/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DUSE_SIM_ENVIRON -DBUILD_WITH_N_LANES=9 -c $< -o $@


obj_t1:
	mkdir $@

obj_s1:
	mkdir $@


obj_t2:
	mkdir $@

obj_s2:
	mkdir $@

obj_u:
	mkdir $@


obj_t1w:
	mkdir $@

obj_s1w:
	mkdir $@


obj_t2w:
	mkdir $@

obj_s2w:
	mkdir $@

obj_uw:
	mkdir $@

obj_hpvm1:
	mkdir $@

obj_hpvm2:
	mkdir $@

obj_hpvm1_W:
	mkdir $@

obj_hpvm2_W:
	mkdir $@

obj_hpvm1/hpvm_tasks.ll : src/hpvm_tasks.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=5 -emit-llvm -S -o $@ $<

obj_hpvm1/hpvm_tasks.hpvm.ll : obj_hpvm1/hpvm_tasks.ll
	$(OPT) -load LLVMGenHPVM.so -genhpvm -globaldce $< -S -o $@ $(OPTFLAGS)

obj_hpvm2/hpvm_tasks.ll : src/hpvm_tasks.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DBUILD_WITH_N_LANES=5 -emit-llvm -S -o $@ $<

obj_hpvm2/hpvm_tasks.hpvm.ll : obj_hpvm2/hpvm_tasks.ll
	$(OPT) -load LLVMGenHPVM.so -genhpvm -globaldce $< -S -o $@ $(OPTFLAGS)

obj_hpvm1_W/hpvm_tasks.ll : src/hpvm_tasks.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=9 -emit-llvm -S -o $@ $<

obj_hpvm1_W/hpvm_tasks.hpvm.ll : obj_hpvm1_W/hpvm_tasks.ll
	$(OPT) -load LLVMGenHPVM.so -genhpvm -globaldce $< -S -o $@ $(OPTFLAGS)

obj_hpvm2_w/hpvm_tasks.ll : src/hpvm_tasks.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DBUILD_WITH_N_LANES=9 -emit-llvm -S -o $@ $<

obj_hpvm2_w/hpvm_tasks.hpvm.ll : obj_hpvm2_W/hpvm_tasks.ll
	$(OPT) -load LLVMGenHPVM.so -genhpvm -globaldce $< -S -o $@ $(OPTFLAGS)

$(OBJ_H1) : obj_hpvm1/hpvm_tasks.hpvm.ll
	$(OPT) -load LLVMBuildDFG.so $(BACKEND_LOAD) -load LLVMClearDFG.so $(BACKEND_FLAG) -clearDFG -S $< -o $@ $(OPTFLAGS)

$(OBJ_LL1):obj_hpvm1/%.ll : src/%.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=5 -emit-llvm -S -o $@ $<

$(OBJ_H2) : obj_hpvm2/hpvm_tasks.hpvm.ll
	$(OPT) -load LLVMBuildDFG.so $(BACKEND_LOAD) -load LLVMClearDFG.so $(BACKEND_FLAG) -clearDFG -S $< -o $@ $(OPTFLAGS)

$(OBJ_LL2):obj_hpvm2/%.ll : src/%.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DBUILD_WITH_N_LANES=5 -emit-llvm -S -o $@ $<

$(OBJ_LINKED1) : $(OBJ_H1) $(OBJ_LL1) $(HPVM_RT_LIB)
	$(LLVM_LINK) $^ -S -o $@

$(OBJ_LINKED2) : $(OBJ_H2) $(OBJ_LL2) $(HPVM_RT_LIB)
	$(LLVM_LINK) $^ -S -o $@


$(OBJ_H1_W) : obj_hpvm1_W/hpvm_tasks.hpvm.ll
	$(OPT) -load LLVMBuildDFG.so $(BACKEND_LOAD) -load LLVMClearDFG.so $(BACKEND_FLAG) -clearDFG -S $< -o $@ $(OPTFLAGS)

$(OBJ_LL1_W):obj_hpvm1_W/%.ll : src/%.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5558 -DRECV_PORT=5559 -DBUILD_WITH_N_LANES=9 -emit-llvm -S -o $@ $<

$(OBJ_H2_W) : obj_hpvm2_W/hpvm_tasks.hpvm.ll
	$(OPT) -load LLVMBuildDFG.so $(BACKEND_LOAD) -load LLVMClearDFG.so $(BACKEND_FLAG) -clearDFG -S $< -o $@ $(OPTFLAGS)

$(OBJ_LL2_W):obj_hpvm2_W/%.ll : src/%.c
	$(HPVMCC) $(CFLAGS) -DXMIT_PORT=5560 -DRECV_PORT=5561 -DBUILD_WITH_N_LANES=9 -emit-llvm -S -o $@ $<

$(OBJ_LINKED1_W) : $(OBJ_H1_W) $(OBJ_LL1_W) $(HPVM_RT_LIB)
	$(LLVM_LINK) $^ -S -o $@

$(OBJ_LINKED2_W) : $(OBJ_H2_W) $(OBJ_LL2_W) $(HPVM_RT_LIB)
	$(LLVM_LINK) $^ -S -o $@


$(SCHED_LIB) $(SCHED_MODULE):
	@echo "***********************************"
	@echo "*** Compiling Scheduler Library ***"
	@echo "***********************************"
	(cd ../../sched_library; $(MAKE) $(HPVM))
	@echo "***********************************"
	@echo "************** DONE! **************"
	@echo "***********************************"

$(TASK_LIB) $(TASK_MODULE):
	@echo "***********************************"
	@echo "****** Compiling Task Library *****"
	@echo "***********************************"
	(cd ../../task_library; $(MAKE) $(HPVM))
	@echo "***********************************"
	@echo "************** DONE! **************"
	@echo "***********************************"

$(TASK_LIB_W) $(TASK_MODULE_W):
	@echo "***********************************"
	@echo "****** Compiling Task Library *****"
	@echo "***********************************"
	(cd ../../task_library; $(MAKE) $(HPVM))
	@echo "***********************************"
	@echo "************** DONE! **************"
	@echo "***********************************"

clean:
	$(RM) $(OBJ_T1) $(OBJ_S1)
	$(RM) $(OBJ_T2) $(OBJ_S2)
	$(RM) $(OBJ_T1_W) $(OBJ_S1_W)
	$(RM) $(OBJ_T2_W) $(OBJ_S2_W)
	$(RM) $(OBJ_H1)
	$(RM) $(OBJ_H2)
	$(RM) $(OBJ_H1_W)
	$(RM) $(OBJ_H2_W)
	$(RM) $(OBJ_U)
	$(RM) $(OBJ_U_W)
	if [ -f "$(NVDLA_MODULE)" ]; then rm $(NVDLA_MODULE); fi
	if [ -d "$(NVDLA_RUNTIME)" ]; then rm -rf $(NVDLA_RUNTIME); fi
	(cd socket; make clean)

clobber: clean
	$(RM) -r obj_t1 obj_s1
	$(RM) -r obj_t2 obj_s2
	$(RM) -r obj_u
	$(RM) -r obj_t1w obj_s1w
	$(RM) -r obj_t2w obj_s2w
	$(RM) -r obj_uw
	$(RM) -r obj_hpvm1 obj_hpvm1_W obj_hpvm2 obj_hpvm2_w
	$(RM) $(TARGET1) $(TARGET2)
	$(RM) $(STARGET1) $(STARGET2)
	$(RM) $(TARGET1_W) $(TARGET2_W)
	$(RM) $(STARGET1_W) $(STARGET2_W)
	$(RM) $(HPVMTARGET1) $(HPVMTARGET2)
	$(RM) $(HPVMTARGET1_W) $(HPVMTARGET2_W)
	$(RM) gen_new_trace
	$(RM) gen_new_wide_trace
	$(RM) $(HPVMTARGET)
	(cd socket; make clobber)


check_env:
ifndef HPVM_DIR
	$(error HPVM_DIR is undefined! setup_paths.sh needs to be sourced before running make!)
endif


.PHONY: all clean


#depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(SRC_D)
# DO NOT DELETE THIS LINE -- make depend depends on it.

src/read_trace.o: ./include/kernels_api.h ./include/verbose.h
src/read_trace.o: ./include/base_types.h ./include/calc_fmcw_dist.h
src/read_trace.o: ./include/utils.h
src/read_trace.o: ./include/sim_environs.h
src/getopt.o: ./include/getopt.h
src/descrambler_function.o: ./include/base.h ./include/utils.h
src/descrambler_function.o: ./include/viterbi_standalone.h
src/viterbi_flat.o: ./include/base.h ./include/utils.h
src/viterbi_flat.o: ./include/viterbi_flat.h ./include/verbose.h
src/viterbi_flat.o: ./include/viterbi_standalone.h ./include/scheduler.h
src/viterbi_flat.o: ./include/base_types.h
src/scheduler.o: ./include/getopt.h ./include/utils.h
src/scheduler.o: ./include/verbose.h
src/scheduler.o: ./include/scheduler.h ./include/base_types.h
src/scheduler.o: ./include/calc_fmcw_dist.h
src/sim_environs.o: ./include/kernels_api.h ./include/verbose.h
src/sim_environs.o: ./include/base_types.h ./include/calc_fmcw_dist.h
src/sim_environs.o: ./include/utils.h
src/sim_environs.o: ./include/sim_environs.h
src/cpu_vit_accel.o: ./include/base.h ./include/utils.h
src/cpu_vit_accel.o: ./include/viterbi_flat.h ./include/verbose.h
src/cpu_vit_accel.o: ./include/viterbi_standalone.h
src/viterbi_standalone.o: ./include/base.h ./include/utils.h
src/viterbi_standalone.o: ./include/viterbi_flat.h
src/viterbi_standalone.o: ./include/verbose.h
src/main.o: ./include/getopt.h ./include/verbose.h ./include/scheduler.h
src/main.o: ./include/base_types.h
src/main.o: ./include/kernels_api.h ./include/calc_fmcw_dist.h
src/main.o: ./include/utils.h ./include/sim_environs.h
src/cpu_fft_accel.o: ./include/scheduler.h
src/cpu_fft_accel.o: ./include/base_types.h ./include/fft-1d.h
src/cpu_fft_accel.o: ./include/calc_fmcw_dist.h
src/timer.o: ./include/timer.h
src/kernels_api.o: ./include/kernels_api.h
src/kernels_api.o: ./include/verbose.h ./include/base_types.h
src/kernels_api.o: ./include/calc_fmcw_dist.h
src/kernels_api.o: ./include/utils.h ./include/read_trace.h
src/kernels_api.o: ./include/viterbi_flat.h ./include/base.h
src/hpvm_tasks.o: ./include/hpvm_tasks.h ./include/base_types.h
