# ============================================
# Daisy Seed Hardware Build — vox-digital-vco
# STM32H750, libDaisy + DaisySP (submodules)
# ============================================

# ---------- Project ----------
PROJECT_NAME ?= vox_digital_vco_hw
BUILD_DIR    ?= build

# Build toggles
BUILD        ?= release      # release|debug
USE_LTO      ?= 1            # 1|0
HW_FAST_MATH ?= 0            # 1|0 (adds -ffast-math)
V            ?= 0            # 1 = verbose

# ---------- Repo / Submodules ----------
REPO_ROOT       := $(abspath ..)
SUB_DAISY_DIR   := $(abspath $(REPO_ROOT)/libdaisy)
SUB_DAISYSP_DIR := $(abspath $(REPO_ROOT)/DaisySP)

# Prefer submodules (no env vars needed)
ifneq ("$(wildcard $(SUB_DAISY_DIR)/core/STM32H750IB_flash.lds)","")
  DAISY_DIR := $(SUB_DAISY_DIR)
endif
ifneq ("$(wildcard $(SUB_DAISYSP_DIR)/Source)","")
  DAISYSP_DIR := $(SUB_DAISYSP_DIR)
endif

# Optional local override file (not required)
-include .daisy_paths.mk

# Sanity checks
ifeq ("$(wildcard $(DAISY_DIR)/core/STM32H750IB_flash.lds)","")
  $(error DAISY_DIR invalid or missing. Expected submodule at '$(SUB_DAISY_DIR)' or set in .daisy_paths.mk)
endif
ifeq ("$(wildcard $(DAISYSP_DIR)/Source)","")
  $(error DAISYSP_DIR invalid or missing. Expected submodule at '$(SUB_DAISYSP_DIR)' or set in .daisy_paths.mk)
endif

# Helpers
.PHONY: submodules hw_print_paths hw_write_paths diagnose_headers
submodules:
	git -C "$(REPO_ROOT)" submodule update --init --recursive

hw_print_paths:
	@echo "DAISY_DIR=$(DAISY_DIR)"
	@echo "DAISYSP_DIR=$(DAISYSP_DIR)"

hw_write_paths:
	@echo 'DAISY_DIR=$(DAISY_DIR)'     > .daisy_paths.mk
	@echo 'DAISYSP_DIR=$(DAISYSP_DIR)' >> .daisy_paths.mk
	@echo 'Wrote .daisy_paths.mk'

diagnose_headers:
	@test -f "$(DAISY_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include/stm32h7xx.h" -o \
	      -f "$(DAISY_DIR)/Drivers/CMSIS-Device/ST/STM32H7xx/Include/stm32h7xx.h" && \
	 echo "OK: stm32h7xx.h present" || (echo "MISSING: stm32h7xx.h — build libdaisy to fetch Cube HAL"; exit 1)

# ---------- Toolchain ----------
PREFIX  ?= arm-none-eabi
CC      := $(PREFIX)-gcc
CXX     := $(PREFIX)-g++
AS      := $(PREFIX)-gcc
AR      := $(PREFIX)-ar
OBJCOPY := $(PREFIX)-objcopy
OBJDUMP := $(PREFIX)-objdump
NM      := $(PREFIX)-nm
SIZE    := $(PREFIX)-size

Q := $(if $(filter 1,$(V)),,@)

# ---------- MCU / Flags ----------
DEVICE_DEFS := -DSTM32H750xx -DUSE_HAL_DRIVER -DTARGET_DAISY=1 -DARM_MATH_CM7 -DVM_HARDWARE=1
CPU_FLAGS   := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
COMMON_WARN := -Wall -Wextra -Wno-unused-parameter -Wno-psabi

SANITY_NOHEAP := -fno-exceptions -fno-rtti -fno-math-errno -fno-asynchronous-unwind-tables -fno-unwind-tables -ffunction-sections -fdata-sections -fno-builtin

ifeq ($(BUILD),debug)
  OPT := -O1 -g
else
  OPT := -O3
endif

ifeq ($(USE_LTO),1)
  LTO := -flto
else
  LTO :=
endif

ifeq ($(HW_FAST_MATH),1)
  FAST := -ffast-math -funsafe-math-optimizations
else
  FAST :=
endif

CFLAGS   := $(CPU_FLAGS) $(OPT) $(LTO) $(FAST) $(SANITY_NOHEAP) $(DEVICE_DEFS) $(COMMON_WARN) -std=gnu11
CXXFLAGS := $(CPU_FLAGS) $(OPT) $(LTO) $(FAST) $(SANITY_NOHEAP) $(DEVICE_DEFS) $(COMMON_WARN) -std=gnu++17
ASFLAGS  := $(CPU_FLAGS)

# ---------- Includes ----------
INCLUDES := \
  -I$(REPO_ROOT)/src \
  -I$(REPO_ROOT)/src/dsp \
  -I$(DAISY_DIR)/src \
  -I$(DAISY_DIR)/src/sys \
  -I$(DAISY_DIR)/src/usbd \
  -I$(DAISY_DIR)/src/usbh \
  -I$(DAISY_DIR)/src/sys/fatfs \
  -I$(DAISY_DIR)/Drivers/STM32H7xx_HAL_Driver/Inc \
  -I$(DAISY_DIR)/Drivers/CMSIS/Include \
  -I$(DAISY_DIR)/Drivers/CMSIS/Device/ST/STM32H7xx/Include \
  -I$(DAISY_DIR)/Drivers/CMSIS_5/CMSIS/Core/Include \
  -I$(DAISY_DIR)/Drivers/CMSIS-Device/ST/STM32H7xx/Include \
  -I$(DAISY_DIR)/Drivers/CMSIS-DSP/Include \
  -I$(DAISY_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
  -I$(DAISY_DIR)/Middlewares/ST/STM32_USB_Host_Library/Core/Inc \
  -I$(DAISY_DIR)/Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc \
  -I$(DAISY_DIR)/Middlewares/ST/STM32_USB_Host_Library/Class/MIDI/Inc \
  -I$(DAISY_DIR)/Middlewares/Third_Party/FatFs/src \
  -I$(DAISYSP_DIR)/Source \
  -I$(DAISYSP_DIR)/Source/Utility

# ---------- Sources ----------
SRC_CXX := \
  $(abspath $(REPO_ROOT)/src/dsp/*.cpp) \
  $(abspath $(CURDIR)/main.cpp)
SRC_CXX := $(wildcard $(SRC_CXX))

OBJ_CXX := $(patsubst $(REPO_ROOT)/%,$(BUILD_DIR)/%,$(patsubst %.cpp,%.o,$(SRC_CXX)))

# ---------- Link ----------
LIB_DIRS  := -L$(DAISY_DIR)/build -L$(DAISYSP_DIR)/build
LIBS      := -Wl,--start-group -ldaisy -ldaisysp -lc -lm -lnosys -Wl,--end-group
LINKMAP   := $(BUILD_DIR)/$(PROJECT_NAME).map
LDSCRIPT  := $(DAISY_DIR)/core/STM32H750IB_flash.lds
LDFLAGS   := $(CPU_FLAGS) $(OPT) $(LTO) \
             -Wl,--gc-sections \
             -Wl,-Map,$(LINKMAP),--cref \
             -T$(LDSCRIPT) \
             -Wl,--print-memory-usage \
             -Wl,--no-warn-rwx-segments

# SDK libs (building these also fetches STM32Cube HAL headers & Middlewares)
LIB_DAISY_A   := $(DAISY_DIR)/build/libdaisy.a
LIB_DAISYSP_A := $(DAISYSP_DIR)/build/libdaisysp.a
SDK_LIBS      := $(LIB_DAISY_A) $(LIB_DAISYSP_A)

# ---------- Public Targets ----------
.PHONY: hw hw_clean flash hw_size hw_size_verbose hw_nm hw_check_symbols hw_run_silence_test hw_bootstrap_sdks

hw: submodules $(BUILD_DIR)/$(PROJECT_NAME).bin hw_size hw_nm hw_check_symbols
	@echo "== Build OK"

hw_clean:
	rm -rf $(BUILD_DIR)

flash: $(BUILD_DIR)/$(PROJECT_NAME).bin
	@if command -v dfu-util >/dev/null 2>&1; then \
	  echo "== Flashing via dfu-util"; \
	  dfu-util -a 0 -s 0x08000000:leave -D $(BUILD_DIR)/$(PROJECT_NAME).bin; \
	else \
	  echo "dfu-util not found. Trying openocd..."; \
	  if command -v openocd >/dev/null 2>&1; then \
	    openocd -f interface/stlink.cfg -f target/stm32h7x.cfg -c "program $(BUILD_DIR)/$(PROJECT_NAME).elf verify reset exit"; \
	  else \
	    echo "Neither dfu-util nor openocd available. Install one."; exit 1; \
	  fi; \
	fi

hw_bootstrap_sdks: submodules $(SDK_LIBS)
	@echo "== SDKs ready"

# ---------- Build Rules ----------
$(LIB_DAISY_A):
	$(Q)$(MAKE) -C $(DAISY_DIR)

$(LIB_DAISYSP_A):
	$(Q)$(MAKE) -C $(DAISYSP_DIR)

# Ensure SDK libs are built BEFORE compiling ANY of our objects
$(OBJ_CXX): | $(SDK_LIBS)

$(BUILD_DIR)/%.o: $(REPO_ROOT)/%.cpp
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/$(PROJECT_NAME).elf: $(OBJ_CXX) $(SDK_LIBS)
	@mkdir -p $(dir $@)
	$(Q)$(CXX) $(OBJ_CXX) $(LIB_DIRS) $(LIBS) $(LDFLAGS) -o $@

$(BUILD_DIR)/$(PROJECT_NAME).bin: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(Q)$(OBJCOPY) -O binary $< $@

# ---------- Post-Build Checks ----------
hw_size: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(SIZE) --format=berkeley $<

# Per-section view (and totals) like the old verbose style
hw_size_verbose: $(BUILD_DIR)/$(PROJECT_NAME).elf
	$(SIZE) --format=sysv $<

hw_nm: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "== Checking unresolved externals (should be none):"
	@out="$$( $(NM) -u $< )"; \
	if [ -n "$$out" ]; then echo "$$out"; echo "ERROR: Unresolved symbols found"; exit 2; \
	else echo "OK: no unresolved externals"; fi

hw_check_symbols: $(BUILD_DIR)/$(PROJECT_NAME).elf
	@echo "== Checking for operator new/delete (should be none):"
	@$(NM) $< | grep -E ' _Z(nw|na|dl|da)(j|m)|operator new|operator delete' >/dev/null && \
	  (echo "ERROR: C++ heap operators detected"; exit 2) || echo "OK"
	@echo "== Checking for common EH/RTTI helpers (advisory):"
	@$(NM) $< | grep -E '__cxa|typeinfo|__gxx_personality' >/dev/null && \
	  (echo "WARN: EH/RTTI markers present (ensure not in audio path)"; exit 0) || echo "OK"

# ---------- Host-side Denorm/NaN Smoke Test (optional) ----------
hw_run_silence_test:
	@mkdir -p $(BUILD_DIR)
	@c++ -std=c++17 -O2 -I$(REPO_ROOT)/src -I$(REPO_ROOT)/src/dsp -o $(BUILD_DIR)/silence_test $(REPO_ROOT)/tests/silence_test.cpp && $(BUILD_DIR)/silence_test
