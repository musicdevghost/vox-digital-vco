# Vox DIY Hardware Template — Reusable Hardware + Shared DSP

This template freezes the **Daisy Seed** hardware/pin map and scanning logic so you can reuse the same codebase for many Eurorack modules. You only change:
- **Module name / panel labels** (graphics, docs),
- **DSP core** (drop a new `MyNewCore.*` in `src/dsp/cores/` and point `SelectedCore.hpp` to it).

Everything else (audio I/O, ADCs, dual 4051 muxes, LED, DAC env out, smoothing, env follower) stays identical.

## Fixed hardware

- **Audio:** stereo in/out via codec (Seed defaults), 48 kHz, blocksize ≈ 48.
- **Direct CV ADCs (post attenuverter bias, normalized 0..1):**  
  A0=CV1→Timbre, A1=CV2→Pitch (V/Oct), A2=CV3→Spread, A3=CV4→Morph.
- **MUX 1 (IC8: raw pots), COM=A5, S0=D6, S1=D8, S2=D9**  
  ch0=FILTER(Tone), ch1=SIZE(Pitch), ch2=FEEDB(Morph), ch3=DIFF(Spread).
- **MUX 2 (IC9: attenuverters), COM=A6, S0=D1, S1=D2, S2=D3**  
  ch0=FILTER_AT, ch1=SIZE_AT, ch2=FEEDB_AT, ch3=DIFF_AT.
- **Panel LED:** D29 (GPIO).
- **ENV / CV OUT:** DAC OUT1 (PA4). Write 0..1; external stage lifts to 0–8 V.

## Build

You build only from `hardware/` using Daisy core Makefile.

Environment (typical):

```bash
export DAISY_DIR=../libdaisy
export DAISYSP_DIR=../DaisySP
```

Build (default core, inputs enabled):

```bash
make -C hardware clean
make -C hardware
```

**Flash (DFU):**

```bash
make -C hardware program-dfu
```

### Useful `DEFS_EXTRA` combinations

- **Test mode (blink LED, 440 Hz tone, DAC ramp):**
  ```bash
  make -C hardware DEFS_EXTRA="-DHW_TEST=1"
  ```
- **Core without inputs (self oscillators):** feeds zeros into core
  ```bash
  make -C hardware DEFS_EXTRA="-DVM_CORE_HAS_INPUTS=0"
  ```
- **Change blocksize/sample rate (defaults shown):**
  ```bash
  make -C hardware DEFS_EXTRA="-DVM_BLOCKSIZE=48 -DVM_SR=48000"
  ```

## Adding a new module / core

1. **Copy the template core**:
   ```
   cp src/dsp/cores/VoxTemplateCore.hpp src/dsp/cores/MyNewCore.hpp
   cp src/dsp/cores/VoxTemplateCore.cpp src/dsp/cores/MyNewCore.cpp
   ```
2. Implement your DSP in `MyNewCore.*` using the shared API in `IDspCore.hpp`.
3. Point the selector to your core: edit `src/dsp/SelectedCore.hpp` and replace:
   ```cpp
   #include "cores/VoxTemplateCore.hpp"
   using SelectedCore = vm::VoxTemplateCore;
   ```
   with:
   ```cpp
   #include "cores/MyNewCore.hpp"
   using SelectedCore = vm::MyNewCore;
   ```
4. Build/flash as usual. You **do not** touch `hardware/` files.

## DSP API (stable)

```cpp
namespace vm {
  struct Params { float pitch, timbre, morph, spread; float pitchVolts; };
  struct Block  { const float* inL; const float* inR; float* outL; float* outR; size_t n; };
  struct Meta   { const char* name; const char* version; };
  struct Aux    { /* optional buffer for ENV/etc. */ };

  struct IDspCore {
    virtual ~IDspCore() {}
    virtual void init(float sr) {}
    virtual void reset() {}
    virtual void setParams(const Params&) {}
    virtual void processBlock(const Block&) = 0;                 // main
    virtual bool fillAux(float* dst, size_t n) { return false; } // optional
  };
}
```

- Always stereo I/O. If you compile with `-DVM_CORE_HAS_INPUTS=0`, the hardware layer feeds **zero** inputs to your core.
- `pitchVolts` exposes the **raw Pitch CV** path in volts (see scaling below).

## Control mapping (normalized)

After smoothing (α≈0.12), params are computed per block:

```
pitch  = SIZE_KNOB   + CV2 * SIZE_AT;    // clamp 0..1
timbre = FILTER_KNOB + CV1 * FILTER_AT;  // clamp 0..1
morph  = FEEDB_KNOB  + CV4 * FEEDB_AT;   // clamp 0..1
spread = DIFF_KNOB   + CV3 * DIFF_AT;    // clamp 0..1
```

- Attenuverter readings are mapped to **[-1..+1]** (center at 0.5).
- `pitchVolts` is derived from the **direct CV2** path using `VM_VOCT_RANGE_VOLTS` (default `10.0f`). You can change that in `ParamMap.hpp`.

## ADC scaling notes

- ADCs are normalized to 0..1 by Daisy. We assume:
  - Direct CVs A0..A3 already include any biasing to be ADC-safe.
  - V/Oct (CV2) nominal range is **0..10 V** → normalized 0..1. Edit `ParamMap::normToVolts()` if your hardware is 0..5 V or 0..8 V.
- The mux scanners run with **1 block latency** (we read the previous channel while setting the next one). This is documented and harmless for UI.

## Verifications / guards

- **No heap in DSP:** run
  ```bash
  make -C hardware verify
  ```
  This fails the build if `new`/`delete` appear anywhere under `../src/dsp/`.

## Design choices

- **SelectedCore.hpp** is a thin alias. Swapping a module is a one-line change.
- **IDspCore** keeps the surface tiny and stable. Audio block IO, a single `Params`, and an optional `fillAux()` for sidechain/env.
- **HAL vs Mixer:** `Hw.cpp` isolates Seed init, ADC/mux, LED/DAC, smoothing. `Hw` exposes only `poll()`, `params()`, `pitchVolts()`, and `writeEnv()`.
- **Smoothing:** single-pole with α≈0.12 by default for every control to avoid zipper.
- **ENV path:** If the core does not fill `Aux`, we fall back to the output audio. Either way, the LED and DAC get a useable envelope.

Happy reusing—drop a new core file, point the selector, and ship.
