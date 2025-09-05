# VoxAudio — Hardware & VCV Rack Template

A clean template for building **Daisy Seed (STM32H750)** firmware and a **VCV Rack v2** plugin with modular DSP cores.

This doc covers:

- Environment setup  
- Building & flashing the **hardware** target  
- Building & installing the **VCV Rack** plugin  
- How to create a **new module** by only touching the DSP core + metadata  
- What `USE_LTO` does (0 vs 1)  
- What the `all` target builds  
- Common pitfalls / troubleshooting

---

## 1) Requirements

### Toolchain (hardware)
- **Arm GNU Toolchain** (`arm-none-eabi`) — ensure the `bin` dir is in your `PATH`  
  You should have: `arm-none-eabi-gcc`, `arm-none-eabi-g++`, `arm-none-eabi-objcopy`, `arm-none-eabi-size`.
- **dfu-util** (for flashing Daisy Seed)  
  - macOS: `brew install dfu-util`  
  - Debian/Ubuntu: `sudo apt-get install dfu-util`  
  - Windows: use Zadig to install the **WinUSB** driver for the Daisy DFU device.

### Submodules (hardware)
This template expects **git submodules** for Daisy:

```ini
[submodule "DaisySP"]
    path = DaisySP
    url  = https://github.com/electro-smith/DaisySP
[submodule "libdaisy"]
    path = libdaisy
    url  = https://github.com/electro-smith/libdaisy
```

Initialize them once:

```bash
git submodule update --init --recursive
```

> The hardware Makefile includes headers from `../libdaisy` and `../DaisySP` and links to `../libdaisy/build` and `../DaisySP/build`.

### VCV Rack SDK (plugin)
- VCV Rack **v2** SDK (or full Rack source). Set one of:
  - `RACK_SDK=/path/to/Rack-SDK` **or**
  - `RACK_DIR=/path/to/Rack` (built Rack repo)

---

## 2) First-time setup

```bash
# From repo root — build Daisy dependencies (once, or when submodules update)
make -C libdaisy
make -C DaisySP
```

> The hardware Makefile also has a `libs` target and will build these automatically on first `make`.

---

## 3) Building the hardware target (Daisy Seed)

From the `hardware/` folder:

```bash
# Default build = release + LTO
make
```

Debug build (easier to step through, no LTO):

```bash
make BUILD=debug USE_LTO=0 all
```

Other handy targets:

```bash
make clean     # wipe build/
make size      # code size report (after linking)
make flash     # flash to Daisy via dfu-util
```

### Flashing the Daisy
1. Put the Seed in DFU mode: hold **BOOT**, tap **RESET**, release **BOOT**.  
2. Then:

```bash
make flash
```

This runs:

```
dfu-util -a 0 -s 0x08000000:leave -D build/VoxAudioHW.bin
```

**Hardware entry point** is `hardware/main.cpp`. It calls your selected DSP core with **non-interleaved** buffers:
```cpp
core.processBlock(in[0], in[1], out[0], out[1], size);
```

---

## 4) Building the VCV Rack plugin

From repo root:

```bash
# Either set RACK_SDK (recommended)...
export RACK_SDK=/path/to/Rack-SDK
# ...or set RACK_DIR to a built Rack repo
# export RACK_DIR=/path/to/Rack

make clean
make -j$(nproc)
make install
```

Create a distributable ZIP:
```bash
make dist
# Produces: dist/<slug>-<version>-<platform>.vcvplugin
```

> If you see a stray Homebrew `-L/opt/homebrew/opt/postgresql@14/lib` warning, you can run:
> `LDFLAGS= make` (clears inherited LDFLAGS) — purely cosmetic.

---

## 5) Template structure (Rack side)

```
src/
├─ dsp/
│  ├─ IDspCore.hpp           # tiny interface used by all modules
│  ├─ VoxAudioCore.{hpp,cpp} # example core
│  └─ SelectedCore.hpp       # (optional) alias for hardware builds
├─ framework/
│  ├─ TemplateIO.hpp         # shared enums (Params/Inputs/Outputs/Lights)
│  ├─ TemplateModule.hpp     # minimal glue: smoothing + DSP call
│  ├─ TemplateWidget.hpp     # generic widget (no hardcoded positions)
│  └─ PepperOverlay.hpp      # optional overlay (reused by all modules)
├─ Components.hpp            # knobs/ports/screws from your project
├─ Layout12HP.hpp            # ALL positions centralized here
├─ ui.hpp                    # SVG helpers
├─ plugin.cpp/.hpp           # model registration via template
res/
└─ plugin.json               # module entries (slug,name,desc,tags)
hardware/
└─ Makefile, main.cpp        # Daisy Seed build + entry-point
```

**UI is generic and layout-driven:**  
Edit **only** `Layout12HP.hpp` if you want to move controls. Pepper overlay is controlled by:

```cpp
inline math::Vec PEPPER_POS()       { return rack::mm2px({10.8f, 14.7f}); }
inline float     PEPPER_BRIGHTNESS(){ return 0.60f; } // 0..1
```

---

## 6) Create a new module (minimal steps)

> Goal: Only implement the DSP core + add minimal metadata. No UI/layout changes.

### Step A — Add your DSP core
Create `src/dsp/VoxFilterCore.{hpp,cpp}` implementing `IDspCore`:

```cpp
// src/dsp/VoxFilterCore.hpp
#pragma once
#include "IDspCore.hpp"
#include <cmath>

struct VoxFilterCore : public IDspCore {
    void init(double sr) override { sr_ = sr; reset(); }
    void reset() override { lpL_ = lpR_ = 0.f; params_ = {0.5f, 1.0f, 0.5f, 0.0f}; }
    void setParams(const Params& p) override { params_ = p; }

    void processBlock(const float* inL, const float* inR,
                      float* outL, float* outR, size_t n) override {
        float fc = 200.f * std::pow(60.f, params_.tone);             // 200..12k
        float a  = 1.f - std::exp(-6.2831853f * fc / (float)sr_);
        float dry = 1.f - params_.dryWet, wet = params_.dryWet;
        float drive = 1.f + 2.f * params_.macro;

        for (size_t i=0;i<n;++i) {
            float L = inL ? inL[i] : 0.f, R = inR ? inR[i] : 0.f;
            lpL_ += a * (L - lpL_); lpR_ += a * (R - lpR_);
            float WL = std::tanh(drive * lpL_) * params_.gain;
            float WR = std::tanh(drive * lpR_) * params_.gain;
            outL[i] = dry * L + wet * WL;
            outR[i] = dry * R + wet * WR;
        }
    }
private:
    double sr_ = 48000.0;
    float  lpL_ = 0.f, lpR_ = 0.f;
    Params params_{};
};
```

```cpp
// src/dsp/VoxFilterCore.cpp
#include "VoxFilterCore.hpp" // header-only; cpp exists for some build globbing
```

### Step B — Register the module (2 lines in `src/plugin.cpp`)
```cpp
#include "dsp/VoxAudioCore.hpp"
#include "dsp/VoxFilterCore.hpp"   // add this

#define REGISTER_VOX_MODULE(ClassName, CoreType, slug)     using ClassName = VoxTemplateModule<CoreType>;          Model* model##ClassName = createModel<ClassName, VoxTemplateWidget>(slug)

Plugin* pluginInstance = nullptr;
REGISTER_VOX_MODULE(VoxAudio,  VoxAudioCore,  "vox-audio");
REGISTER_VOX_MODULE(VoxFilter, VoxFilterCore, "vox-filter");  // add this

void init(Plugin* p) {
    pluginInstance = p;
    p->addModel(modelVoxAudio);
    p->addModel(modelVoxFilter);   // add this
}
```

### Step C — Add a module entry to `plugin.json`
```json
{
  "slug": "vox-filter",
  "name": "VoxFilter",
  "description": "Stereo filter with tone/macro.",
  "tags": ["Filter","Stereo"]
}
```

### Build for Rack
```bash
make clean && make && make install
```

**That’s it.** UI/layout/overlay all reuse the template.

---

## 7) Hardware: select a core without touching `main.cpp`

Keep `hardware/main.cpp` fixed by using an alias:

```cpp
// src/dsp/SelectedCore.hpp
#pragma once
#include "VoxFilterCore.hpp"   // or VoxAudioCore, etc.
using SelectedCore = VoxFilterCore;
```

```cpp
// hardware/main.cpp
#include "../src/dsp/SelectedCore.hpp"
SelectedCore core;   // never changes again
```

Build & flash:

```bash
cd hardware
make clean
make
make flash   # DFU (hold BOOT, tap RESET, release BOOT)
```

---

## 8) What does `USE_LTO` do?

**LTO = Link Time Optimization.** Optimizes across files at link time.

- `USE_LTO=1` (default in release)
  - **Pros:** smaller/faster firmware via cross-file inlining & DCE.
  - **Cons:** slower links; debugging optimized code is harder.
- `USE_LTO=0`
  - **Pros:** faster builds; friendlier debugging (symbols map neatly).
  - **Cons:** bigger/slightly slower binaries.

**Recommendation:** use `release + LTO` for day-to-day, `debug + no LTO` for tricky debugging.

---

## 9) What does the `all` target build?

`make` (or `make all`) produces:

- `build/VoxAudioHW.elf` — linked ELF (with symbols)  
- `build/VoxAudioHW.hex` — Intel HEX  
- `build/VoxAudioHW.bin` — raw binary (used by `make flash`)

---

## 10) Troubleshooting

**“No such file or directory: `/Makefile`” when running `make` in `hardware/`**  
You’re including libDaisy’s makefile without `LIBDAISY_DIR` set.  
This template **does not** include libDaisy’s makefile; it links to prebuilt archives and supplies the includes directly.

**`usbh_def.h` or other USB/FatFs headers missing**  
This template mirrors Illusions’ includes:
```
../libdaisy/src/usbd
../libdaisy/src/usbh
../libdaisy/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
../libdaisy/Middlewares/ST/STM32_USB_Host_Library/Core/Inc
../libdaisy/Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc
../libdaisy/Middlewares/ST/STM32_USB_Host_Library/Class/MIDI/Inc
../libdaisy/Middlewares/Third_Party/FatFs/src
```
If you’re on a different libDaisy revision, adjust these paths to match.

**Link warnings `_write/_read/_close` “will always fail”**  
Benign with `--specs=nosys.specs`. Ignorable unless you’re calling POSIX I/O.

**DFU can’t find device**  
Re-enter DFU (hold **BOOT**, tap **RESET**, release **BOOT**), try a different USB-C cable/port, avoid hubs. On Windows, install WinUSB driver for DFU using Zadig.

**Rack build inherits noisy Homebrew `LDFLAGS`**  
Run `LDFLAGS= make` (clears inherited flags) or filter in your Rack Makefile.

---

## 11) Make this a GitHub “Template repository”

On GitHub: **Settings → General → Template repository → Enable**.  
Description: *“Voxmachina Rack + Daisy template. Add modules by writing a Core.”*

---

## 12) Optional helper: scaffold a module

Add `scripts/new_module.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
NAME_PASCAL="$1"                               # e.g. VoxFilter
CORE="${NAME_PASCAL#Vox}Core"                  # FilterCore
SLUG="vox-$(echo "$NAME_PASCAL" | sed 's/^Vox//; s/\([A-Z]\)/-\L\1/g; s/^-//')"

HPP="src/dsp/${CORE}.hpp"; CPP="src/dsp/${CORE}.cpp"
mkdir -p src/dsp

cat > "$HPP" <<'EOF'
#pragma once
#include "IDspCore.hpp"
#include <cmath>
struct __CLASS__ : public IDspCore {
  void init(double sr) override { sr_ = sr; reset(); }
  void reset() override { zL_=zR_=0.f; params_={0.5f,1.0f,0.5f,0.0f}; }
  void setParams(const Params& p) override { params_ = p; }
  void processBlock(const float* inL, const float* inR, float* outL, float* outR, size_t n) override {
    float fc = 200.f * std::pow(60.f, params_.tone);
    float a  = 1.f - std::exp(-6.2831853f * fc / (float)sr_);
    for (size_t i=0;i<n;++i) {
      float L = inL ? inL[i] : 0.f, R = inR ? inR[i] : 0.f;
      zL_ += a * (L - zL_); zR_ += a * (R - zR_);
      float WL = zL_ * params_.gain, WR = zR_ * params_.gain;
      outL[i] = (1.f-params_.dryWet)*L + params_.dryWet*WL;
      outR[i] = (1.f-params_.dryWet)*R + params_.dryWet*WR;
    }
  }
private:
  double sr_=48000.0; float zL_=0.f,zR_=0.f; Params params_{};
};
EOF
sed -i '' "s/__CLASS__/${CORE}/g" "$HPP"

cat > "$CPP" <<EOF
#include "${CORE}.hpp"
EOF

# Register in plugin.cpp and plugin.json
perl -0777 -pe '
  s!(#include "dsp/.*VoxAudioCore\.hpp".*)!$1\n#include "dsp/'"$CORE"'.hpp'!s;
  s!(REGISTER_VOX_MODULE\(VoxAudio.*\);)!$1\nREGISTER_VOX_MODULE('"$NAME_PASCAL"', '"$CORE"', "'"$SLUG"'");!s;
  s!(p->addModel\(modelVoxAudio\);)!$1\n    p->addModel(model'"$NAME_PASCAL"');!s;
' -i '' src/plugin.cpp

python3 - <<PY
import json,sys
p="plugin.json"
data=json.load(open(p))
data.setdefault("modules",[]).append({
  "slug":"$SLUG","name":"$NAME_PASCAL",
  "description":"$NAME_PASCAL module.","tags":["Stereo"]
})
json.dump(data,open(p,"w"),indent=2); print("Updated plugin.json")
PY

echo "Scaffolded $NAME_PASCAL → $CORE (slug: $SLUG)."
```

Make it executable and use:
```bash
chmod +x scripts/new_module.sh
scripts/new_module.sh VoxFilter
make -j$(nproc) && make install
# Firmware:
sed -i '' 's/using SelectedCore = .*/using SelectedCore = VoxFilterCore;/' src/dsp/SelectedCore.hpp
( cd hardware && make clean && make && make flash )
```

---

Happy patching!
