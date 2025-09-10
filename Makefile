RACK_DIR ?= $(RACK_SDK)

FLAGS += -std=c++11 -stdlib=libc++ -mmacosx-version-min=10.9 -march=nehalem          -O3 -fPIC -fno-omit-frame-pointer -funsafe-math-optimizations          -Wall -Wextra -Wno-unused-parameter
FLAGS += -Isrc
FLAGS += -DTARGET_RACK

SOURCES += $(wildcard src/*.cpp) $(wildcard src/*/*.cpp) $(wildcard src/*/*/*.cpp)

DISTRIBUTABLES += res plugin.json README.md
include $(RACK_DIR)/plugin.mk

# Strip noisy Homebrew path from LDFLAGS
LDFLAGS := $(filter-out -L/opt/homebrew/opt/postgresql@14/lib,$(LDFLAGS))