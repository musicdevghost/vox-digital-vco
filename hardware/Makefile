# VOX — Daisy firmware Makefile (robust)
# Usage:
#   make deps          # show resolved LIBDAISY_DIR / DAISYSP_DIR
#   make               # build
#   make flash         # DFU flash
#   make clean         # clean
#   make size          # show size

TARGET = firmware

CPP_SOURCES = \
  main.cpp \
  ../src/dsp/VoxCore.cpp

# Allow local overrides without editing this file
-include Makefile.local

# Candidate locations (first matching wins)
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

# Resolve directories
LIBDAISY_DIR := $(firstword $(foreach d,$(CANDIDATE_LIBDAISY_DIRS),$(if $(wildcard $(d)/core/Makefile),$(d))))
DAISYSP_DIR  := $(firstword $(foreach d,$(CANDIDATE_DAISYSP_DIRS),$(if $(wildcard $(d)/Makefile),$(d))))

# Friendly errors if libDaisy is missing
ifeq ($(LIBDAISY_DIR),)
$(error Could not find libDaisy. Set LIBDAISY_DIR=/absolute/path/to/libDaisy or run `git submodule update --init --recursive` at the repo root.)
endif
ifeq ($(wildcard $(LIBDAISY_DIR)/core/Makefile),)
$(error $(LIBDAISY_DIR)/core/Makefile not found. Is libDaisy checked out correctly?)
endif

# Optional DaisySP (only if present)
ifneq ($(DAISYSP_DIR),)
C_INCLUDES += -I$(DAISYSP_DIR)/Source
endif

# Project includes
C_INCLUDES += -I../src -I../src/dsp

# Flags
C_DEFS += -DDEBUG -DDAISY_USE_FPU=1
OPT ?= -O3

# Daisy core rules
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
