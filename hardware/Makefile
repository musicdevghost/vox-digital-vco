# ---------- Project ----------
TARGET      := VoxAudioHW
BUILD_DIR   := build

BUILD      ?= release
USE_LTO    ?= 1

# ---------- Toolchain ----------
CROSS   := arm-none-eabi-
CC      := $(CROSS)gcc
CXX     := $(CROSS)g++
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size

# ---------- MCU ----------
CPU_FLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
LD_SCRIPT := ../libdaisy/core/STM32H750IB_flash.lds
SPECFLAGS := --specs=nano.specs --specs=nosys.specs

# ---------- LTO / Opt ----------
ifeq ($(USE_LTO),1)
LTO_FLAGS := -flto
endif

ifeq ($(BUILD),debug)
OPT_FLAGS := -Og
DBG_FLAGS := -g -ggdb
C_DEFS_EXTRA :=
else
OPT_FLAGS := -Os
DBG_FLAGS := -g
C_DEFS_EXTRA := -DNDEBUG
endif

# ---------- Macros & Includes ----------
C_DEFS := \
  -DUSE_HAL_DRIVER -DSTM32H750xx -DHSE_VALUE=16000000 \
  -DCORE_CM7 -DSTM32H750IB -DARM_MATH_CM7 -DUSE_FULL_LL_DRIVER \
  $(C_DEFS_EXTRA)

INCLUDES := \
  -I../src \
  -I../src/dsp \
  -I../libdaisy \
  -I../libdaisy/src \
  -I../libdaisy/src/sys \
  -I../libdaisy/src/usbd \
  -I../libdaisy/src/usbh \
  -I../libdaisy/Drivers/CMSIS_5/CMSIS/Core/Include \
  -I../libdaisy/Drivers/CMSIS-DSP/Include \
  -I../libdaisy/Drivers/CMSIS-Device/ST/STM32H7xx/Include \
  -I../libdaisy/Drivers/STM32H7xx_HAL_Driver/Inc \
  -I../libdaisy/Middlewares/ST/STM32_USB_Device_Library/Core/Inc \
  -I../libdaisy/Middlewares/ST/STM32_USB_Host_Library/Core/Inc \
  -I../libdaisy/Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc \
  -I../libdaisy/Middlewares/ST/STM32_USB_Host_Library/Class/MIDI/Inc \
  -I../libdaisy/Middlewares/Third_Party/FatFs/src \
  -I../libdaisy/core \
  -I../DaisySP/Source

# Force include ST top header for CMSIS
# This is needed to get the correct #defines for the specific MCU
# being used (e.g. STM32H750xx)
FORCE_INCLUDE := -include stm32h7xx.h

# ---------- Sources ----------
SRCS_C   := \
  ../libdaisy/core/startup_stm32h750xx.c

SRCS_CPP := \
  main.cpp \
  ../src/dsp/VoxAudioCore.cpp

OBJS := \
  $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_C:.c=.o))) \
  $(addprefix $(BUILD_DIR)/,$(notdir $(SRCS_CPP:.cpp=.o)))

DEPS := $(OBJS:.o=.d)

# ---------- Flags ----------
WARN_FLAGS := -Wall -Wno-missing-attributes -Wno-stringop-overflow

CFLAGS   := $(CPU_FLAGS) $(OPT_FLAGS) $(DBG_FLAGS) $(LTO_FLAGS) \
            $(WARN_FLAGS) -fdata-sections -ffunction-sections \
            -fno-asynchronous-unwind-tables -fno-unwind-tables \
            -fomit-frame-pointer -std=gnu11

CXXFLAGS := $(CPU_FLAGS) $(OPT_FLAGS) $(DBG_FLAGS) $(LTO_FLAGS) \
            $(WARN_FLAGS) -fdata-sections -ffunction-sections \
            -fno-asynchronous-unwind-tables -fno-unwind-tables \
            -fomit-frame-pointer -fno-exceptions -fno-rtti \
            -finline -finline-functions-called-once -fshort-enums \
            -fno-move-loop-invariants -Wno-register -std=gnu++14

LDFLAGS := $(CPU_FLAGS) $(SPECFLAGS) $(LTO_FLAGS) \
           -T$(LD_SCRIPT) \
           -L../libdaisy/build -L../DaisySP/build \
           -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections -Wl,--print-memory-usage -Wl,--no-warn-rwx-segments

LIBS    := -ldaisy -ldaisysp -lc -lm -lnosys

# ---------- Targets ----------
.PHONY: all clean flash program-dfu size libs
.DEFAULT_GOAL := all

all: libs $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

libs:
	$(MAKE) -C ../libdaisy -j
	$(MAKE) -C ../DaisySP  -j

$(BUILD_DIR):
	@mkdir -p $@

# Objects
# Add -mgeneral-regs-only for the startup file to silence FP clobber warning
$(BUILD_DIR)/%.o: ../libdaisy/core/%.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -mgeneral-regs-only $(C_DEFS) $(INCLUDES) $(FORCE_INCLUDE) -MMD -MP -MF"$(@:.o=.d)" -c $< -o $@

$(BUILD_DIR)/%.o: %.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(C_DEFS) $(INCLUDES) $(FORCE_INCLUDE) -MMD -MP -MF"$(@:.o=.d)" -c $< -o $@

$(BUILD_DIR)/%.o: ../src/dsp/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(C_DEFS) $(INCLUDES) $(FORCE_INCLUDE) -MMD -MP -MF"$(@:.o=.d)" -c $< -o $@

# Link
$(BUILD_DIR)/$(TARGET).elf: $(OBJS) | $(BUILD_DIR)
	$(CXX) $(OBJS) $(LDFLAGS) $(LIBS) -o $@

# Hex / Bin
$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Flash via DFU bootloader (hold BOOT, tap RESET, release BOOT)
flash: $(BUILD_DIR)/$(TARGET).bin
	dfu-util -a 0 -s 0x08000000:leave -D $(BUILD_DIR)/$(TARGET).bin

program-dfu: flash

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) --format=berkeley $<

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
