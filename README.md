# Vox Digital VCO — Hardware (Daisy Seed) + VCV Rack v2

A clean setup for sharing DSP between **VCV Rack v2** and **Daisy Seed (STM32H750 Rev B)**.  
Builds are small, **C++17-only**, and run from **inside `hardware/`** with Daisy’s core Makefile.

---

## 1) Requirements

### Toolchain (hardware)
- **Arm GNU Toolchain** (`arm-none-eabi`) — put its `bin` in your `PATH`.
  - Needed tools: `arm-none-eabi-gcc`, `g++`, `size`, `objcopy`, `nm`, `objdump`.
- **dfu-util** for flashing (or OpenOCD/ST-Link if you prefer).

### Submodules (repo root)
This repo assumes submodules are checked out at the root:
```
libdaisy/   (Electro-Smith libDaisy)
DaisySP/    (DSP helper library)
src/dsp/    (shared DSP used by Rack + hardware)
hardware/   (firmware: Makefile + main.cpp)
```
Initialize once:
```bash
git submodule update --init --recursive
```

### VCV Rack SDK (plugin)
Install the Rack **v2** SDK (or use a built Rack repo) and set exactly one:
```bash
export RACK_SDK=/path/to/Rack-SDK
# or
export RACK_DIR=/path/to/Rack   # built Rack source tree
```

---

## 2) Shared DSP layout

Your DSP lives in `src/dsp/`. Important files:
```
src/dsp/
├─ IDspCore.hpp
├─ Platform.hpp
├─ VoxVcoCore.hpp / .cpp
└─ SelectedCore.hpp     # selects the core type for hardware
```
`SelectedCore.hpp` keeps hardware glue trivial:
```cpp
#pragma once
#include "dsp/VoxVcoCore.hpp"
namespace vm { using SelectedCore = VoxVcoCore; }
```

> Note: because `SelectedCore.hpp` uses `#include "dsp/VoxVcoCore.hpp"`, the **repo root** and **src** are on the include path in the hardware Makefile (`-I.. -I../src -I../src/dsp`). No relative `../` includes needed in your DSP headers.

---

## 3) Hardware build (Daisy Seed)

All commands run from **inside** `hardware/`.

### Quick start
```bash
make clean
make          # builds with C++17, links with Daisy core; prints memory usage
make flash    # DFU flash (BOOT + RESET on the Seed to enter DFU)
make size     # Berkeley + SysV section sizes
make verify   # checks unresolveds + no heap CALLS inside src/dsp/*.cpp
```

### What the Makefile does
- Includes Daisy’s **core** Makefile (like official examples) → correct HAL/CMSIS paths, startup, and linker script for **STM32H750**.
- Forces **C++17** via `CPP_STANDARD = -std=gnu++17` (no C++20/Concepts).
- Builds **directly from your shared DSP**: `../src/dsp/*.cpp` (no duplication).
- Prints a Daisy-style **region usage table** after link (`--print-memory-usage`).
- `make verify`:
  - Fails on **unresolved externals** in the final ELF.
  - Scans **only your DSP objects** (`../src/dsp/*.cpp`) with `objdump` and fails **only if there are call sites** to `operator new/delete` (no false positives from libsupc++ being *present* but unused).

### Hardware wiring

`hardware/main.cpp` initializes Seed at **48 kHz**, **stereo**, **blocksize ~48**, and calls your core through the selector:

```cpp
#include "SelectedCore.hpp"
using CoreT = vm::SelectedCore;
static CoreT core;

// default signature: processBlock(inL,inR,outL,outR,n)
```

Two compile-time options:
- **Default** (core expects inputs):
  ```bash
  make
  ```
- **No inputs** (core has `process(outL,outR,n)`):
  ```bash
  make DEFS_EXTRA="-DVM_CORE_PROCESS_NAME=process -DVM_CORE_HAS_INPUTS=0"
  ```

Your four parameters (pitch, timbre, morph, spread) are available in `main.cpp` to map ADCs later (currently stubbed). A soft-saturation path is available for a minimal “alive” sound if you ever compile without the core.

---

## 4) VCV Rack v2 build

From repo root (with `RACK_SDK` or `RACK_DIR` set):
```bash
make clean
make -j$(sysctl -n hw.ncpu 2>/dev/null || nproc)
make install     # copies plugin to Rack user plugins folder
make dist        # packages dist/<slug>-<version>-<platform>.vcvplugin
```

---

## 5) Overriding the core/type/signature (optional)

You can override the defaults **without editing code**:
```bash
# Different type (fully-qualified ok)
make DEFS_EXTRA="-DVM_CORE_TYPE='my::ns::MyCore'"

# Different process name
make DEFS_EXTRA="-DVM_CORE_PROCESS_NAME=process"

# No inputs + different name
make DEFS_EXTRA="-DVM_CORE_PROCESS_NAME=process -DVM_CORE_HAS_INPUTS=0"

# Force-include a specific header (if your filename differs)
make HEADERS_FORCE=../src/dsp/SelectedCore.hpp
```

Defaults used by the Makefile:
```
-DVM_USE_SHARED_CORE=1
-DVM_CORE_TYPE='vm::SelectedCore'
-DVM_CORE_PROCESS_NAME=processBlock
-DVM_CORE_HAS_INPUTS=1
-DVM_SR=48000 -DVM_BLOCKSIZE=48
```

---

## 6) Memory and size visibility

After link, the toolchain prints a region table like:
```
Memory region   Used Size  Region Size  %age Used
FLASH:             95800 B       128 KB    73.1%
SRAM:              19928 B       512 KB     3.8%
RAM_D2:               16 KB      288 KB     5.6%
...
```
Use `make size` for detailed section sizes (Berkeley + SysV).

---

## 7) “No heap in audio” policy

- `make verify` **passes** unless your **DSP objects** actually call `operator new/delete`.  
- Typical culprits if it fails: `std::vector`, `std::string`, `std::function`, dynamic `new`/`delete`.  
- Fix by switching to `std::array`, fixed pools, ring buffers, or static workspaces.

---

## 8) Troubleshooting

**Header not found / type not found (`'vox' does not name a type`)**  
- Ensure your header is included (we include `SelectedCore.hpp` directly).
- Verify the **include paths** (`-I.. -I../src -I../src/dsp`) and the **type macro**:
  ```bash
  make DEFS_EXTRA="-DVM_CORE_TYPE='vm::SelectedCore'"
  ```

**SelectedCore.hpp can’t find `dsp/VoxVcoCore.hpp`**  
- That path is relative to repo root; keep `-I../src` in the Makefile (already added).

**Verify flags `_ZdlPv` / `_Znwm`**  
- That means an actual call site exists **inside your DSP TU(s)**. Replace dynamic allocations and STL containers that allocate. Re-run `make verify` to confirm.

**DFU not detected**  
- Enter DFU (hold **BOOT**, tap **RESET**, release **BOOT**). Try a different cable/port. On Windows, install WinUSB for the DFU device via Zadig.

**Toolchain warnings like `_read/_write will always fail`**  
- Benign with `--specs=nosys.specs`; they’re GC’d if unused.

**Homebrew adds odd linker flags**  
- Run `LDFLAGS= make` to clear inherited env flags.

---

## 9) File map (quick reference)

```
repo-root/
├─ libdaisy/             (submodule)
├─ DaisySP/              (submodule)
├─ src/
│  └─ dsp/
│     ├─ IDspCore.hpp
│     ├─ VoxVcoCore.hpp / .cpp
│     └─ SelectedCore.hpp
└─ hardware/
   ├─ Makefile           (uses Daisy core Makefile; C++17; shared DSP)
   └─ main.cpp           (48kHz, stereo, block 48; calls SelectedCore)
```

---

## 10) Credits / License

- Daisy platform © Electro-Smith (libDaisy, DaisySP).  
- This template: MIT (unless specified otherwise in the repo).
