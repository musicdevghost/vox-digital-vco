#pragma once
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>
#include "IDspCore.hpp"   // your template interface

namespace vm {

// ===== Helpers ===============================================================

static inline double fast_clip(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// PolyBLEP (normalized phase t in [0,1), dt = phase increment per sample)
static inline double poly_blep(double t, double dt) {
    if (dt <= 0.0) return 0.0;
    if (t < dt) {
        const double x = t / dt;
        return x + x - x * x - 1.0;
    }
    if (t > 1.0 - dt) {
        const double x = (t - 1.0) / dt;
        return x * x + x + x + 1.0;
    }
    return 0.0;
}

// Simple one-pole HP for DC blocking (for FM input and fold feedback)
struct OnePoleHP {
    double a = 0.0, z = 0.0;
    void setup(double sr, double fc = 5.0) {
        const double x = std::exp(-2.0 * M_PI * fc / sr);
        a = x;
        z = 0.0;
    }
    inline double process(double x) {
        const double y = x - z;
        z = x * a + y * a;
        return y;
    }
};

// Fast PRNG (LCG)
struct Lcg {
    uint32_t s = 0x1234567u;
    inline void seed(uint32_t v) { s = v ? v : 1u; }
    inline uint32_t next() { s = 1664525u * s + 1013904223u; return s; }
    inline double unipolar() { return (next() & 0x00FFFFFFu) / double(0x01000000u); }
    inline double bipolar()  { return unipolar() * 2.0 - 1.0; }
};

// ===== VoxVcoCore ============================================================

class VoxVcoCore final : public IDspCore {
public:
    VoxVcoCore() { reset(); }

    // Template's interface
    void init(double sampleRate) override;
    void reset() override;
    void setParams(const IDspCore::Params& p) override;
    void processBlock(const float* inL, const float* inR,
                      float* outL, float* outR, int n) override;

private:
    // ---- Constants ----
    static constexpr int kMaxUnison = 7;
    static constexpr double kC4 = 261.6255653006; // Hz
    static constexpr double kMinHz = 0.1;
    static constexpr double kMaxHz = 19000.0;

#if defined(TARGET_DAISY)
    static constexpr double kFmMaxHz = 4000.0;
#else
    static constexpr double kFmMaxHz = 8000.0;
#endif

#if defined(TARGET_DAISY)
    static constexpr double kOutGain = 1.0;  // hardware path [-1..+1]
#else
    static constexpr double kOutGain = 5.0;  // Rack path ±5 V
#endif

    // ---- State ----
    double sr_ = 48000.0;
    double invSr_ = 1.0 / 48000.0;

    struct Voice {
        double phase = 0.0;
        double triI = 0.0;
        double parabPrev = 0.0;
        double drift = 0.0;
        Lcg rng;
    };
    std::array<Voice, kMaxUnison> voices_{};

    Lcg rng_;
    float prevSync_ = 0.0f;
    OnePoleHP fmHp_;

    // ---- Parameters (cached per block) ----
    struct Cached {
        double pitchVolts = 0.0;
        double macroA = 0.0;
        double macroB = 0.0;
        double macroC = 0.0;
        double macroD = 0.0;
        bool   fmCable = false;
    } P_;

    // derived per-block settings
    int    unisonCount_ = 1;
    double detuneSpreadCents_ = 0.0;
    double stereoWidth_ = 0.0;
    double pwmWidth_ = 0.5;
    double foldAmt_ = 0.0;
    double chaosAmt_ = 0.0;
    double fmIndex_ = 0.0;
    bool   fmEnabled_ = false;

    // ---- Stereo lookahead limiter (linked) ----
    struct LookaheadLimiter {
        static constexpr int kMax = 256;  // max ring size
        // ring buffers
        double bufL[kMax]{}, bufR[kMax]{}, absBuf[kMax]{};
        int w = 0;     // write index
        int r = 0;     // read index (delayed sample)
        int size = 64; // lookahead in samples (<= kMax-1)

        double target = 5.0; // ceiling in volts (Rack=5.0, Daisy=1.0)
        double gr = 1.0;     // current gain reduction (<= 1)
        double atkA = 0.0;   // attack smoothing coeff
        double relA = 0.0;   // release smoothing coeff

        void setup(double sr, double look_ms = 0.75, double atk_ms = 0.10, double rel_ms = 80.0, double targetVolts = 5.0) {
            target = targetVolts;
            // clamp sizes
            int req = (int)std::lround(sr * (look_ms * 0.001));
            size = std::min(kMax - 1, std::max(8, req));
            w = 0;
            // place read pointer 'size' samples behind (so we can look ahead by 'size')
            r = (kMax - size) % kMax;
            gr = 1.0;
            // 1-pole coefficients
            atkA = std::exp(-1.0 / (sr * (atk_ms * 0.001))); // fast toward lower gr
            relA = std::exp(-1.0 / (sr * (rel_ms * 0.001))); // slow toward higher gr
            for (int i = 0; i < kMax; ++i) { bufL[i] = bufR[i] = absBuf[i] = 0.0; }
        }

        inline void process(double inL, double inR, float& outL, float& outR) {
            // write newest sample
            bufL[w] = inL;
            bufR[w] = inR;
            absBuf[w] = std::max(std::abs(inL), std::abs(inR));

            // look ahead: find peak over next 'size' samples (O(size), small)
            double peak = 1e-12;
            int idx = w;
            for (int i = 0; i < size; ++i) {
                double a = absBuf[idx];
                if (a > peak) peak = a;
                if (--idx < 0) idx = kMax - 1;
            }

            // desired gain reduction to hit 'target'
            double desired = (peak > 1e-12) ? std::min(1.0, target / peak) : 1.0;

            // smooth: fast when reducing (attack), slow when releasing
            if (desired < gr)
                gr = desired + (gr - desired) * atkA;
            else
                gr = desired + (gr - desired) * relA;

            // read delayed sample and apply current GR
            double yL = bufL[r] * gr;
            double yR = bufR[r] * gr;
            outL = (float)yL;
            outR = (float)yR;

            // advance indices
            if (++w >= kMax) w = 0;
            if (++r >= kMax) r = 0;
        }
    };

    LookaheadLimiter limiter_;

    // helpers
    inline void updateUnisonFromMacro(double mc);
    inline double detuneCentsToRatio(double cents) const {
        return std::pow(2.0, cents / 1200.0);
    }
    inline double equalPowerL(double pan01) const { return std::cos(0.5 * M_PI * pan01); }
    inline double equalPowerR(double pan01) const { return std::sin(0.5 * M_PI * pan01); }
    inline double softclip3(double x) const {
        const double a = std::fabs(x);
        return (a < 1.0) ? x * (1.0 - (x*x)/3.0) : ((x > 0.0) ? 2.0/3.0 : -2.0/3.0);
    }

    inline double renderCore(double phase, double dt, double morph01,
                             double timbre01, double& triState,
                             double& outSquareForTri) const;

    inline void advanceDrift(Voice& v, double depthCents);
}; // class

} // namespace vm
