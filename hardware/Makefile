# VOX — Daisy firmware Makefile (robust Phase B parity)
TARGET = firmware

CPP_SOURCES = \
  main.cpp \
  ../src/dsp/VoxCore.cpp

-include Makefile.local

CANDIDATE_LIBDAISY_DIRS := \
  $(abspath ../../libDaisy) \
  $(abspath ../libDaisy) \
  $(abspath ../../../libDaisy) \
  $(LIBDAISY_DIR)

CANDIDATE_DAISYSP_DIRS := \
  $(abspath ../../DaisySP) \
  $(abspath ../DaisySP) \
  $(abspath ../../../DaisySP) \
  $(DAISYSP_DIR)

LIBDAISY_DIR := $(firstword $(foreach d,$(CANDIDATE_LIBDAISY_DIRS),$(if $(wildcard $(d)/core/Makefile),$(d))))
DAISYSP_DIR  := $(firstword $(foreach d,$(CANDIDATE_DAISYSP_DIRS),$(if $(wildcard $(d)/Makefile),$(d))))

ifeq ($(LIBDAISY_DIR),)
$(error Could not find libDaisy. Set LIBDAISY_DIR=/absolute/path/to/libDaisy or add it as submodule.)
endif
ifeq ($(wildcard $(LIBDAISY_DIR)/core/Makefile),)
$(error $(LIBDAISY_DIR)/core/Makefile not found. Is libDaisy checked out correctly?)
endif

ifneq ($(DAISYSP_DIR),)
C_INCLUDES += -I$(DAISYSP_DIR)/Source
endif

C_INCLUDES += -I../src -I../src/dsp

C_DEFS += -DDEBUG -DDAISY_USE_FPU=1
OPT ?= -O3

SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

.PHONY: flash size deps
flash: program-dfu
size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) -B $<
	$(SIZE) -A $<
deps:
	@echo "LIBDAISY_DIR=$(LIBDAISY_DIR)"
	@echo "DAISYSP_DIR=$(DAISYSP_DIR)"
