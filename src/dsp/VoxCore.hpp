
#pragma once
#include <cmath>
#include <cstdint>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vm { namespace vox {

static inline double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
static inline double fract(double x)   { return x - std::floor(x); }
static inline double mix(double a, double b, double t) { return a + (b - a) * t; }

// Equal-power crossfade to keep loudness stable
static inline double xfade_equal_power(double a, double b, double t) {
    t = clamp01(t);
    double wA = std::sin((1.0 - t) * (M_PI * 0.5));
    double wB = std::sin(t * (M_PI * 0.5));
    return a * wA + b * wB;
}

struct CoreParams {
    double sampleRate = 48000.0;
    double baseA4 = 440.0;
    int    kPitchMacroOctaves = 5;
};

// Knob-level controls (sampled once per 48 frames)
struct Controls {
    double pitchKnob01 = 0.5;
    double morph01     = 0.0; // Sine/Tri fold -> Tri -> Saw -> Square -> Additive
    double timbre01    = 0.5; // folding near low morph; PWM near square; brightness near additive
    double spread01    = 1.0; // (temporarily ignored; amplitude normalized internally)
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
    // slow-drift state for analog "warmth"
    double drift = 0.0;
    double driftTarget = 0.0;
    int    driftCountdown = 0;
    uint32_t rng = 22222u;
    // amplitude normalization state
    double rms = 0.0;
    double outGain = 1.0;
};

class VoxCore {
public:
    void setup(const CoreParams& p) { params_ = p; }
    void reset() { st_.phase = 0.0; st_.drift = st_.driftTarget = 0.0; st_.driftCountdown = 0; st_.rng = 22222u; st_.rms = 0.0; st_.outGain = 1.0; }

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
    static inline double osc_triangle_linear(double ph) {
        double s = 2.0 * ph - 1.0;
        return 2.0 * (1.0 - std::fabs(s)) - 1.0;
    }
    static inline double osc_saw_blep(double ph, double dt) {
        double y = 2.0 * ph - 1.0;
        y -= polyblep(ph, dt);
        return y;
    }
    static inline double osc_square_pwm_blep(double ph, double duty, double dt) {
        double y = (ph < duty) ? 1.0 : -1.0;
        y += polyblep(ph, dt);
        double t2 = fract(ph - duty + 1.0);
        y -= polyblep(t2, dt);
        return y;
    }

    // Triangle curvature shaping (0.0 linear; >0 rounded; <0 sharper)
    static inline double tri_shape(double ph, double shape) {
        double x = osc_triangle_linear(ph);
        double s = std::tanh(x * (1.0 + 4.0 * shape));
        return mix(x, s, std::fabs(shape));
    }

    // Soft saturation with roughly constant output gain
    static inline double soft_sat(double x, double amt) {
        double drive = 1.0 + 5.0 * clamp01(amt);
        double y = std::tanh(x * drive);
        double norm = std::tanh(drive);
        return (norm > 0.0) ? (y / norm) : y;
    }

    // Deterministic tiny RNG for drift
    static inline uint32_t lcg_next(uint32_t s) { return 1664525u * s + 1013904223u; }
    static inline double   lcg_unip(uint32_t &s) { s = lcg_next(s); return (double)(s) / 4294967296.0; }
    static inline double   lcg_bip(uint32_t &s) { return lcg_unip(s) * 2.0 - 1.0; }

    // Symmetric wavefolder, x in [-1,1], fold in [0..1]
    static inline double fold_sym(double x, double fold) {
        double g = 1.0 + 9.0 * clamp01(fold);
        double y = x * g;
        y = std::fabs(std::fmod(y + 3.0, 4.0) - 2.0) - 1.0;
        return y;
    }

    // Additive blocks with dynamic harmonic limit (Nyquist-safe)
    static inline double additive_sine(double ph, double freq, double sr, double bright) {
        int maxH = (int)std::floor((sr * 0.45) / std::max(freq, 1.0));
        int baseH = 4 + (int)std::floor(12.0 * clamp01(bright));
        int H = std::max(1, std::min(baseH, maxH));
        double y = 0.0, norm = 0.0;
        for (int k = 1; k <= H; ++k) {
            double amp = 1.0 / (double)k;
            y += amp * std::sin(2.0 * M_PI * (double)k * ph);
            norm += amp;
        }
        return (norm > 0.0) ? (y / norm) : y;
    }
    static inline double additive_triangle(double ph, double freq, double sr, double bright) {
        int maxH = (int)std::floor((sr * 0.45) / std::max(freq, 1.0));
        int baseH = 3 + (int)std::floor(10.0 * clamp01(bright));
        int H = std::max(1, std::min(baseH, maxH));
        double y = 0.0, norm = 0.0;
        int added = 0;
        for (int n = 1; added < H; n += 2) { // odd only
            double amp = 1.0 / ((double)n * (double)n);
            y += amp * std::sin(2.0 * M_PI * (double)n * ph) * ((n % 4 == 1) ? 1.0 : -1.0);
            norm += amp;
            added++;
        }
        return (norm > 0.0) ? (y / norm) : y;
    }

    CoreParams params_;
    State st_;
    float hs_prev_ = 0.f;
};

}} // namespace vm::vox
