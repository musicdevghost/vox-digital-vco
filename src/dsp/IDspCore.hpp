#pragma once
#include <cstddef>

namespace vm {

// Allocation-free DSP interface shared by Rack and Daisy.
class IDspCore {
public:
    struct Params {
        // ---- Framework-expected fields (TemplateModule.hpp interpolates these) ----
        float pitch;  // [0..1]
        float timbre;    // [0..1]
        float morph;    // [0..1]
        float spread;   // [0..1]

        // ---- Extra fields for advanced cores (optional; wrapper can fill) ----
        double pitchVolts;      // 1V/Oct in volts
        double macros[4];       // Macro A..D [0..1] if your wrapper provides them
        bool   fmCablePresent;  // True if FM jack patched

        // Default ctor used by framework
        Params()
            : pitch(0.5f), timbre(1.0f), morph(0.5f), spread(0.0f),
              pitchVolts(0.0), macros{0.0, 0.0, 0.0, 0.0}, fmCablePresent(false) {}

        // 4-float ctor REQUIRED by TemplateModule.hpp aggregate init
        Params(float d, float g, float t, float m)
            : pitch(d), timbre(g), morph(t), spread(m),
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
