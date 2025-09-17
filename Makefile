RACK_DIR ?= $(RACK_SDK)

CXXFLAGS += -std=c++11 -O3 -fPIC -Wall -Wextra -Wno-unused-parameter
CXXFLAGS += -Isrc

SOURCES := \
  src/rack/plugin.cpp \
  src/rack/VoxSimModule.cpp \
  src/dsp/VoxCore.cpp \
  src/hw/sim_hal.cpp

DISTRIBUTABLES += plugin.json README.md
include $(RACK_DIR)/plugin.mk
