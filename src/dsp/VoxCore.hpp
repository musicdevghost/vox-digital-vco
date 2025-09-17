#pragma once
#include <cmath>
#include <cstdint>

namespace vm { namespace vox {

struct CoreParams {
    double sampleRate = 48000.0;
    double baseA4 = 440.0;
    int    kPitchMacroOctaves = 5; // +/- octaves range via knob
};

struct Controls {
    double pitchKnob01 = 0.5; // 0..1
};

struct Mods {
    // reserved for Phase B (v/oct, fm, sync...)
};

struct State {
    double phase = 0.0; // 0..1
};

class VoxCore {
public:
    void setup(const CoreParams& p) { params_ = p; }
    void reset() { st_.phase = 0.0; }

    void processBlock(const CoreParams& p,
                      const Controls& k,
                      const Mods& m,
                      State& s,
                      float* outL, float* outR, int nframes) {
        (void)m;
        params_ = p;

        const double semisSpan = params_.kPitchMacroOctaves * 12.0;
        const double knobSemis = (clamp01(k.pitchKnob01) * 2.0 - 1.0) * semisSpan;
        const double freq = hzFromSemis(knobSemis, params_.baseA4);
        const double dt = 1.0 / params_.sampleRate;

        for (int i = 0; i < nframes; ++i) {
            s.phase += freq * dt;
            if (s.phase >= 1.0) s.phase -= std::floor(s.phase);
            const float y = (s.phase < 0.5) ? 1.0f : -1.0f;
            outL[i] = y;
            outR[i] = y;
        }
    }

private:
    static inline double clamp01(double x) {
        return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
    }
    static inline double hzFromSemis(double semis, double f0) {
        return f0 * std::pow(2.0, semis / 12.0);
    }

    CoreParams params_;
    State st_;
};

}} // namespace vm::vox
