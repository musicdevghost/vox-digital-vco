#pragma once
#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vm { namespace vox {

static inline double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
static inline double fract(double x)   { return x - std::floor(x); }

struct CoreParams {
    double sampleRate = 48000.0;
    double baseA4 = 440.0;
    int    kPitchMacroOctaves = 5;
};

// Knob-level controls (sampled once per 48 frames)
struct Controls {
    double pitchKnob01 = 0.5;
    double morph01     = 0.0; // selects waveform family (discrete switch for now)
    double timbre01    = 0.5; // PWM duty for square
    double spread01    = 1.0; // amplitude
};

// Audio-rate mods (48-sample arrays)
struct Mods {
    const float* hsync = nullptr; // hard sync source (use rising 0-crossing)
    const float* fm    = nullptr; // linear FM (-1..+1)
    const float* ssync = nullptr; // soft sync gate (>0.5 resets phase)
    double fmDepthHz   = 0.0;     // amount of FM in Hz
};

struct State {
    double phase = 0.0;
};

class VoxCore {
public:
    enum class Wave : int { SINE = 0, TRIANGLE = 1, SAW = 2, SQUARE = 3, COUNT = 4 };

    void setup(const CoreParams& p) { params_ = p; }
    void reset() { st_.phase = 0.0; }

    void processBlock(const CoreParams& p,
                      const Controls& k,
                      const Mods& m,
                      State& s,
                      float* outL, float* outR, int nframes);

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

    // --- PolyBLEP helpers to reduce aliasing on discontinuities ---
    static inline double polyblep(double t, double dt) {
        // 2-sample wide polynomial band-limited step
        if (t < dt) {
            t /= dt;
            return t + t - t*t - 1.0;
        } else if (t > 1.0 - dt) {
            t = (t - 1.0) / dt;
            return t*t + t + t + 1.0;
        } else {
            return 0.0;
        }
    }

    // Basic waveforms (with BLEP where needed)
    static inline double osc_sine(double ph) {
        return std::sin(2.0 * M_PI * ph);
    }
    static inline double osc_triangle(double ph) {
        // Triangle from saw with abs: linear, DC-free
        double s = 2.0 * ph - 1.0;
        return 2.0 * (1.0 - std::fabs(s)) - 1.0;
    }
    static inline double osc_saw(double ph, double dt) {
        double y = 2.0 * ph - 1.0;   // naive saw
        y -= polyblep(ph, dt);       // BLEP at wrap
        return y;
    }
    static inline double osc_square(double ph, double duty, double dt) {
        double y = (ph < duty) ? 1.0 : -1.0; // naive square/PWM
        // BLEP at both edges: phase 0 and duty crossing
        y += polyblep(ph, dt);
        double t2 = fract(ph - duty + 1.0);
        y -= polyblep(t2, dt);
        return y;
    }

    CoreParams params_;
    State st_;
    float hs_prev_ = 0.f;
};

}} // namespace vm::vox
