include ../hardware.config

CPU ?= ariane
ARCH ?= riscv
ifdef DO_CROSS_COMPILATION
 CROSS_COMPILE ?= riscv64-unknown-linux-gnu-
 EXE_EXTENSION=-RV
endif

ifdef COMPILE_TO_ESP
 ESP_ROOT ?= $(realpath ../../../esp)
 ESP_DRIVERS ?= $(ESP_ROOT)/soft/common/drivers
 ESP_DRV_LINUX  = $(ESP_DRIVERS)/linux

 #$(info $$ESP_ROOT is [${ESP_ROOT}])
 #$(info $$ESP_DRIVERS is [${ESP_DRIVERS}])
 #$(info $$ESP_DRV_LINUX is [${ESP_DRV_LINUX}])
endif

HPVM_BIN_PATH = $(HPVM_BUILD_DIR)/bin
OPT = $(HPVM_BIN_PATH)/opt
LLVM_LINK = $(HPVM_BIN_PATH)/llvm-link
HPVMCC = $(HPVM_BIN_PATH)/clang
HPVM_RT_PATH = $(HPVM_BUILD_DIR)/tools/hpvm/projects/hpvm-rt
HPVM_RT_LIB = $(HPVM_RT_PATH)/hpvm-rt.bc


CC = gcc -std=c99

SCHED_ROOT = $(realpath ../sched_library)
INCDIR ?=
INCDIR += -I./include -I$(SCHED_ROOT)/include
ifdef COMPILE_TO_ESP
 INCDIR += -I$(ESP_DRIVERS)/common/include
 INCDIR += -I$(ESP_DRIVERS)/linux/include
endif

CFLAGS ?= -O2 -g
#CFLAGS += -DNUM_ACCEL_TYPES=$(CONFIG_NUM_ACCEL_TYPES)
CFLAGS += $(INCDIR)
CFLAGS += -DINT_TIME
ifdef COMPILE_TO_ESP
 CFLAGS += -DCOMPILE_TO_ESP
endif
#  -- ALWAYS use this one! --   ifdef CONFIG_ESP_INTERFACE
CFLAGS += -DUSE_ESP_INTERFACE
#   endif

# # This sets the maximum number of any Acceleartor type in this build
# #  This should be equal to the largest number of any of FFT, VIT, CV, etc.
# #  But it could be a larger value as well
# ifdef CONFIG_MAX_ACCEL_ANY_TYPE
# CFLAGS += -DMAX_ACCEL_OF_EACH_TYPE=$(CONFIG_MAX_ACCEL_ANY_TYPE)
# endif

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
 CFLAGS += -I/home/espuser/anaconda3/include/python3.7m
endif

ifdef CONFIG_CV_EN
 SW_STR =
 CA_STR = CH
 #CFLAGS += -DHW_CV -DENABLE_NVDLA
 CFLAGS += -DCNN_DEV_BASE='"$(CNN_DEVICE_BASE)"' -I/home/espuser/anaconda3/include/python3.7m
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

LDLIBS ?=
ifdef COMPILE_TO_ESP
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/contig_alloc
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/test
 LDLIBS += -L$(ESP_BUILD_DRIVERS)/libesp
endif
#ifndef CONFIG_KERAS_CV_BYPASS
# LDLIBS += 
#endif
LDLIBS += -L./

LDFLAGS ?=
LDFLAGS += -lm
LDFLAGS += -lpthread
#LDFLAGS += -ldl -rdynamic
ifdef COMPILE_TO_ESP
 LDFLAGS += -lrt
 LDFLAGS += -lesp
 LDFLAGS += -ltest
 LDFLAGS += -lcontig
endif
ifndef CONFIG_KERAS_CV_BYPASS
 LDFLAGS += -lpython3.7m
endif

SRC = $(foreach f, $(wildcard src/*.c), $(shell basename $(f)))
HDR = $(wildcard include/*.h)
OBJ = $(SRC:%.c=obj/%.o)
OBJ_W = $(SRC:%.c=obj_w/%.o)
LL_OBJ = $(SRC:%.c=obj/%.bc)
LL_OBJ_W = $(SRC:%.c=obj_w/%.bc)

VPATH = ./src

TARGET=libtasks.a
TARGET_W=libtasksW.a
HPVM_TARGET=libtasks.bc

#$(info $$TARGET is [${TARGET}])


all: esp-libs $(TARGET) $(TARGET_W)

ESP_BUILD_DRIVERS     = esp-build/drivers

esp-build:
	@mkdir -p $(ESP_BUILD_DRIVERS)/contig_alloc
	@mkdir -p $(ESP_BUILD_DRIVERS)/esp
	@mkdir -p $(ESP_BUILD_DRIVERS)/esp_cache
	@mkdir -p $(ESP_BUILD_DRIVERS)/libesp
	@mkdir -p $(ESP_BUILD_DRIVERS)/probe
	@mkdir -p $(ESP_BUILD_DRIVERS)/test
	@mkdir -p $(ESP_BUILD_DRIVERS)/utils/baremetal
	@mkdir -p $(ESP_BUILD_DRIVERS)/utils/linux
	@ln -sf $(ESP_DRV_LINUX)/contig_alloc/* $(ESP_BUILD_DRIVERS)/contig_alloc
	@ln -sf $(ESP_DRV_LINUX)/esp/* $(ESP_BUILD_DRIVERS)/esp
	@ln -sf $(ESP_DRV_LINUX)/esp_cache/* $(ESP_BUILD_DRIVERS)/esp_cache
	@ln -sf $(ESP_DRV_LINUX)/driver.mk $(ESP_BUILD_DRIVERS)
	@ln -sf $(ESP_DRV_LINUX)/include $(ESP_BUILD_DRIVERS)
	@ln -sf $(ESP_DRV_LINUX)/../common $(ESP_BUILD_DRIVERS)/../common

esp-build-clean:

esp-build-distclean: esp-build-clean
	$(QUIET_CLEAN)$(RM) -rf esp-build

ifdef COMPILE_TO_ESP
esp-libs: esp-build
	  CROSS_COMPILE=$(CROSS_COMPILE) DRIVERS=$(ESP_DRV_LINUX) $(MAKE) -C $(ESP_BUILD_DRIVERS)/contig_alloc/ libcontig.a
	  cd $(ESP_BUILD_DRIVERS)/test; CROSS_COMPILE=$(CROSS_COMPILE) BUILD_PATH=$$PWD $(MAKE) -C $(ESP_DRV_LINUX)/test
	  cd $(ESP_BUILD_DRIVERS)/libesp; CROSS_COMPILE=$(CROSS_COMPILE) BUILD_PATH=$$PWD $(MAKE) -C $(ESP_DRV_LINUX)/libesp
	  cd $(ESP_BUILD_DRIVERS)/utils; CROSS_COMPILE=$(CROSS_COMPILE) BUILD_PATH=$$PWD $(MAKE) -C $(ESP_DRV_LINUX)/utils
else
esp-libs: 

endif
.PHONY: esp-build-clean esp-build-distclean esp-libs

obj/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DBUILD_WITH_N_LANES=5 -c $< -o $@

$(OBJ): $(HDR)

$(TARGET): obj $(OBJ)
	$(CROSS_COMPILE)$(AR) rcsv $@ $(OBJ)
#	$(CROSS_COMPILE)$(LD) -shared $(LDLIBS) $(OBJ) -o $@ $(LDFLAGS)


obj_w/%.o: %.c
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -DBUILD_WITH_N_LANES=9 -c $< -o $@

$(OBJ_W): $(HDR)

$(TARGET_W): obj_w $(OBJ_W)
	$(CROSS_COMPILE)$(AR) rcsv $@ $(OBJ_W)
#	$(CROSS_COMPILE)$(LD) -shared $(LDLIBS) $(OBJ_W) -o $@ $(LDFLAGS)


clean:
	$(RM) $(OBJ) $(LL_OBJ) $(TARGET)
	$(RM) $(OBJ_W) $(LL_OBJ_W) $(TARGET_W)

clobber: clean
	$(RM) -rf obj obj_w
	$(RM) -rf esp-build


obj:
	mkdir $@

obj_w:
	mkdir $@

hpvm: obj $(HPVM_TARGET) esp-libs $(TARGET)

$(HPVM_TARGET): $(LL_OBJ)
	$(LLVM_LINK) $^ -o $@ 

obj/%.bc: %.c
	$(HPVMCC) -O3 -S $(CFLAGS) -emit-llvm $< -o $@


.PHONY: all clean


#depend:;	makedepend -fMakefile -- $(CFLAGS) -- $(SRC_D)
# DO NOT DELETE THIS LINE -- make depend depends on it.

