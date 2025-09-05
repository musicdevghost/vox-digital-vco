#pragma once
#include <cstddef>

namespace vm {

// Allocation-free DSP interface shared by Rack and Daisy.
class IDspCore {
public:
    struct Params {
        // ---- Framework-expected fields (TemplateModule.hpp interpolates these) ----
        float dryWet;  // [0..1]
        float gain;    // [0..1]
        float tone;    // [0..1]
        float macro;   // [0..1]

        // ---- Extra fields for advanced cores (optional; wrapper can fill) ----
        double pitchVolts;      // 1V/Oct in volts
        double macros[4];       // Macro A..D [0..1] if your wrapper provides them
        bool   fmCablePresent;  // True if FM jack patched

        // Default ctor used by framework
        Params()
            : dryWet(0.5f), gain(1.0f), tone(0.5f), macro(0.0f),
              pitchVolts(0.0), macros{0.0, 0.0, 0.0, 0.0}, fmCablePresent(false) {}

        // 4-float ctor REQUIRED by TemplateModule.hpp aggregate init
        Params(float d, float g, float t, float m)
            : dryWet(d), gain(g), tone(t), macro(m),
              pitchVolts(0.0), macros{0.0, 0.0, 0.0, 0.0}, fmCablePresent(false) {}
    };

    virtual ~IDspCore() {}

    virtual void init(double sampleRate) = 0;
    virtual void reset() = 0;
    virtual void setParams(const Params& p) = 0;

    // inL: Sync (gate), inR: Linear FM. outs must be valid, n > 0.
    virtual void processBlock(const float* inL, const float* inR,
                              float* outL, float* outR, int n) = 0;
};

} // namespace vm
