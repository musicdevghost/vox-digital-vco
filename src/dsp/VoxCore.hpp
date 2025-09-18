#pragma once
#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vm { namespace vox {

static inline double clamp01(double x) {
    return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
}

struct CoreParams {
    double sampleRate = 48000.0;
    double baseA4 = 440.0;
    int    kPitchMacroOctaves = 5; // +/- octaves range via knob
};

struct Controls {
    double pitchKnob01 = 0.5; // 0..1
    double morph01     = 0.0; // 0=square, 1=sine
    double timbre01    = 0.5; // PWM depth center
    double spread01    = 1.0; // amplitude scalar
};

struct Mods {
    // reserved for Phase C (v/oct, fm, sync...)
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

        const double morph = clamp01(k.morph01);
        const double pwmDepth = clamp01(k.timbre01);
        const double amp = clamp01(k.spread01);

        // Map timbre to duty with safe clamp (5%..95%).
        const double duty = 0.5 + (pwmDepth - 0.5) * 0.9; // +/-45% around 50%
        const double dutySafe = clampDuty(duty);

        for (int i = 0; i < nframes; ++i) {
            s.phase += freq * dt;
            if (s.phase >= 1.0) s.phase -= std::floor(s.phase);

            // PWM square
            double pulse = (s.phase < dutySafe) ? 1.0 : -1.0;
            // Sine
            double sine = std::sin(2.0 * M_PI * s.phase);
            // Morph
            double y = (1.0 - morph) * pulse + morph * sine;
            // Spread as amplitude
            y *= amp;

            outL[i] = static_cast<float>(y);
            outR[i] = static_cast<float>(y);
        }
    }

private:
    static inline double hzFromSemis(double semis, double f0) {
        return f0 * std::pow(2.0, semis / 12.0);
    }
    static inline double clampDuty(double d) {
        const double lo = 0.05, hi = 0.95;
        if (d < lo) return lo;
        if (d > hi) return hi;
        return d;
    }

    CoreParams params_;
    State st_;
};

}} // namespace vm::vox
