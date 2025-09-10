# Minimal Daisy Seed firmware using shared VCV Rack DSP (C++17 only)
# Run from inside hardware/:
#   make         # build
#   make flash   # DFU (alias to Daisy's program-dfu)
#   make clean
#   make size    # section sizes
#   make verify  # nm + DSP-only heap check (no operator new/delete CALLS in ../src/dsp/*.cpp)

TARGET = firmware

# App + your shared DSP sources
# Include both top-level dsp/*.cpp and dsp/cores/*.cpp (new location for cores)
CPP_SOURCES = \
  main.cpp \
  Hw.cpp \
  $(wildcard ../src/dsp/*.cpp) \
  $(wildcard ../src/dsp/cores/*.cpp)

# Submodules at repo root
LIBDAISY_DIR = ../libdaisy
DAISYSP_DIR  = ../DaisySP

# Use C++17 (override Daisy default)
CPP_STANDARD = -std=gnu++17

# Include paths
# -I../src so that SelectedCore.hpp can #include "dsp/VoxVcoCore.hpp"
# -I../src/dsp so we can include "SelectedCore.hpp" directly
C_INCLUDES += -I.. -I../src -I../src/dsp -I.

# Extra flags (Daisy core already sets M7 hard-float, linker script, etc.)
CFLAGS   += -ffunction-sections -fdata-sections -fno-builtin -fno-math-errno -Wno-psabi
CXXFLAGS += -fno-exceptions -fno-rtti
LDFLAGS  += -Wl,--gc-sections -Wl,--print-memory-usage

# Build-time defines (override with DEFS_EXTRA=...)
# Default to your selector alias vm::SelectedCore
C_DEFS += \
  -DVM_HARDWARE=1 \
  -DVM_USE_SHARED_CORE=1 \
  -DVM_CORE_TYPE='vm::SelectedCore' \
  -DVM_CORE_PROCESS_NAME=processBlock \
  -DVM_CORE_HAS_INPUTS=1 \
  -DVM_SR=48000 -DVM_BLOCKSIZE=48 \
  -DTARGET_DAISY

# Optional: force-include a header if you ever need to
# make HEADERS_FORCE=../src/dsp/SelectedCore.hpp
ifdef HEADERS_FORCE
CXXFLAGS += -include $(HEADERS_FORCE)
endif

# Pull in Daisy’s core Makefile (like official examples)
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile

# Convenience aliases & checks
.PHONY: flash size verify

flash: program-dfu

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) -B $<
	$(SIZE) -A $<

# DSP object list (for heap-check). Include cores/*.cpp as well.
DSP_SRCS := $(wildcard ../src/dsp/*.cpp) $(wildcard ../src/dsp/cores/*.cpp)
DSP_OBJS := $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.cpp,%.o,$(DSP_SRCS))))

# Verify:
#  1) No unresolved externals after link (nm -u empty).
#  2) No CALLS or tail-calls to operator new/delete inside your DSP objects.
#     (We scan only $(DSP_OBJS) with objdump; this avoids false positives from the C++ runtime.)
verify: $(BUILD_DIR)/$(TARGET).elf
	@echo "---- unresolved externals (should be empty):"
	@arm-none-eabi-nm -u -C $< || true
	@arm-none-eabi-nm -u -C $< | awk 'END{ if (NR>0){ print "*** ERROR: Unresolved externals present."; exit 1 }}'
	@echo "---- heap check in DSP objects ($(DSP_OBJS)):"
	@ok=1; \
	for f in $(DSP_OBJS); do \
	  if [ -f $$f ]; then \
	    if arm-none-eabi-objdump -drwC $$f | grep -E '\b(bl|blx)\b.*(_Znwm|_Znwj|_Znay|_Znaj|_ZdlPv|_ZdlPvm|_ZdaPv)\b|\bb\.\w+\b.*(_Znwm|_Znwj|_Znay|_Znaj|_ZdlPv|_ZdlPvm|_ZdaPv)\b' >/dev/null; then \
	      echo "*** ERROR: heap call in $$f"; \
	      arm-none-eabi-objdump -drwC $$f | grep -nE '\b(bl|blx|b\.\w+)\b.*(_Znwm|_Znwj|_Znay|_Znaj|_ZdlPv|_ZdlPvm|_ZdaPv)\b' | head -n 20; \
	      ok=0; \
	    fi; \
	  fi; \
	done; \
	if [ $$ok -eq 1 ]; then echo "OK (no operator new/delete calls in DSP objects)"; else exit 1; fi
