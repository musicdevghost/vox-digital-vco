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
    static constexpr int    kMaxUnison = 7;     // fixed internal layout: 7 slots
    static constexpr double kC4        = 261.6255653006; // Hz
    static constexpr double kMinHz     = 0.1;
    static constexpr double kMaxHz     = 19000.0;

    // Spread smoothing: crossfade bandwidth around the historical thresholds
    static constexpr double kPairXfadeBW = 0.08;   // ±band around each threshold

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
        double macroC = 0.0; // SPREAD [0..1]
        double macroD = 0.0;
        bool   fmCable = false;
    } P_;

    // derived per-block settings
    int    unisonCount_ = 1;         // legacy (kept for compatibility), not used for mixing
    double detuneSpreadCents_ = 0.0; // scales with macroC (unchanged)
    double stereoWidth_ = 0.0;       // 0..1
    double pwmWidth_ = 0.5;
    double foldAmt_ = 0.0;
    double chaosAmt_ = 0.0;
    double fmIndex_ = 0.0;
    bool   fmEnabled_ = false;

    // ---- Stereo lookahead limiter (linked) ----
    struct LookaheadLimiter {
        static constexpr int kMax = 256;  // max ring size
        double bufL[kMax]{}, bufR[kMax]{}, absBuf[kMax]{};
        int w = 0, r = 0, size = 64;
        double target = 5.0, gr = 1.0, atkA = 0.0, relA = 0.0;

        void setup(double sr, double look_ms = 0.75, double atk_ms = 0.10, double rel_ms = 80.0, double targetVolts = 5.0) {
            target = targetVolts;
            int req = (int)std::lround(sr * (look_ms * 0.001));
            size = std::min(kMax - 1, std::max(8, req));
            w = 0;
            r = (kMax - size) % kMax;
            gr = 1.0;
            atkA = std::exp(-1.0 / (sr * (atk_ms * 0.001)));
            relA = std::exp(-1.0 / (sr * (rel_ms * 0.001)));
            for (int i = 0; i < kMax; ++i) bufL[i] = bufR[i] = absBuf[i] = 0.0;
        }

        inline void process(double inL, double inR, float& outL, float& outR) {
            bufL[w] = inL; bufR[w] = inR;
            absBuf[w] = std::max(std::abs(inL), std::abs(inR));

            double peak = 1e-12;
            int idx = w;
            for (int i = 0; i < size; ++i) {
                double a = absBuf[idx];
                if (a > peak) peak = a;
                if (--idx < 0) idx = kMax - 1;
            }

            double desired = (peak > 1e-12) ? std::min(1.0, target / peak) : 1.0;
            if (desired < gr) gr = desired + (gr - desired) * atkA;
            else              gr = desired + (gr - desired) * relA;

            outL = float(bufL[r] * gr);
            outR = float(bufR[r] * gr);

            if (++w >= kMax) w = 0;
            if (++r >= kMax) r = 0;
        }
    };

    LookaheadLimiter limiter_;

    // ----- Helpers -----
    inline void updateUnisonFromMacro(double mc); // legacy thresholds (kept)
    inline double detuneCentsToRatio(double cents) const {
        return std::pow(2.0, cents / 1200.0);
    }
    inline double equalPowerL(double pan01) const { return std::cos(0.5 * M_PI * pan01); }
    inline double equalPowerR(double pan01) const { return std::sin(0.5 * M_PI * pan01); }
    inline double softclip3(double x) const {
        const double a = std::fabs(x);
        return (a < 1.0) ? x * (1.0 - (x*x)/3.0) : ((x > 0.0) ? 2.0/3.0 : -2.0/3.0);
    }

    // smoothstep 0..1
    static inline double sstep(double x) {
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    // Compute per-voice activations for 7 slots given SPREAD a∈[0,1]
    void computeVoiceActivations_(double a, double act[7]) const;

    inline double renderCore(double phase, double dt, double morph01,
                             double timbre01, double& triState,
                             double& outSquareForTri) const;

    inline void advanceDrift(Voice& v, double depthCents);
}; // class

} // namespace vm
