// VoxVcoCore.hpp
#pragma once
#include <cmath>
#include <cstdint>
#include <array>
#include <algorithm>
#include "IDspCore.hpp"

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

// Simple one-pole HP for DC blocking (for FM input)
struct OnePoleHP {
    double a = 0.0, z = 0.0;
    void setup(double sr, double fc = 5.0) {
        const double x = std::exp(-2.0 * M_PI * fc / sr);
        a = x; z = 0.0;
    }
    inline double process(double x) {
        const double y = x - z;
        z = x * a + y * a;
        return y;
    }
};

// Simple one-pole LP for output bandwidth shaping
struct OnePoleLP {
    double a = 0.0, b = 0.0, y = 0.0;
    void setup(double sr, double fc) {
        const double x = std::exp(-2.0 * M_PI * fc / sr);
        a = x; b = 1.0 - x; y = 0.0;
    }
    inline double process(double x) {
        y = a * y + b * x;
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

    void init(double sampleRate) override;
    void reset() override;
    void setParams(const IDspCore::Params& p) override;
    void processBlock(const float* inL, const float* inR,
                      float* outL, float* outR, int n) override;

private:
    // ---- Constants ----
    static constexpr int    kMaxUnison = 7;
    static constexpr double kC4        = 261.6255653006; // Hz
    static constexpr double kMinHz     = 0.1;
    static constexpr double kMaxHz     = 19000.0;

    static constexpr double kPairXfadeBW = 0.08;   // Spread xfade BW

#if defined(TARGET_DAISY)
    static constexpr double kFmMaxHz = 4000.0;
    static constexpr double kOutGain = 1.0;
#else
    static constexpr double kFmMaxHz = 8000.0;
    static constexpr double kOutGain = 5.0;        // Rack ±5 V
#endif

    // ---- Analog vibe tunables ----
    static constexpr bool   kAnalogModel     = true;
    static constexpr double kAnalogJitterRelPpm = 200.0;
    static constexpr double kAnalogJitterAbsHz  = 0.15;
    static constexpr double kAnalogPwmBiasStd   = 0.015;
    static constexpr double kAnalogShapeStd     = 0.06;
    static constexpr double kAnalogSoftSatMix   = 0.02;
    static constexpr double kAnalogSlewVPerMs   = 800.0;
    static constexpr double kAnalogEdgeJitterUs = 1.2;
    static constexpr double kAnalogPwmBiasHF    = 0.010;
    static constexpr double kOutputLP_Hz        = 22000.0;
    static constexpr double kHum50_Level        = 0.00025;
    static constexpr double kHum60_Level        = 0.00020;
    static constexpr double kHum2H_Level        = 0.00010;
    static constexpr double kNoiseRms_V         = 5.0 * 1e-4;

    // Time smoothing for spread & detune span
    static constexpr double kSpreadSlewMs = 10.0;
    static constexpr double kDetuneSlewMs = 25.0;

    // ---- Sub-oscillator behavior ----
    static constexpr double kSubOnThreshold = 0.15; // add sub when spread < 0.15
    static constexpr double kSubFadeBW      = 0.25; // smooth fade up to threshold
    static constexpr double kSubGain        = 0.90; // sub level before normalization

    // ---- State ----
    double sr_ = 48000.0;
    double invSr_ = 1.0 / 48000.0;

    struct Voice {
        double phase = 0.0;
        double triI = 0.0;
        double parabPrev = 0.0;
        double drift = 0.0;
        double pwmBias = 0.0;
        double shapeSkew = 0.0;
        double jitMF = 0.0;
        double jitLF = 0.0;
        Lcg rng;
    };
    std::array<Voice, kMaxUnison> voices_{};

    double aJitMF_ = 0.0; // jitter filters
    double aJitLF_ = 0.0;

    Lcg rng_;
    float prevSync_ = 0.0f;
    OnePoleHP fmHp_;

    // Sub oscillator state (mono sub triangle)
    double subPhase_ = 0.0;
    double subTriI_  = 0.0;

    // ---- Cached params ----
    struct Cached {
        double pitchVolts = 0.0;
        double macroA = 0.0;
        double macroB = 0.0;
        double macroC = 0.0; // Spread 0..1
        double macroD = 0.0;
        bool   fmCable = false;
    } P_;

    // Derived (targets)
    int    unisonCount_ = 1;
    double detuneSpreadCents_ = 0.0;
    double stereoWidth_ = 0.0;
    double pwmWidth_ = 0.5;
    double foldAmt_ = 0.0;
    double chaosAmt_ = 0.0;
    double fmIndex_ = 0.0;
    bool   fmEnabled_ = false;

    // Smoothed versions
    double stereoWidthZ_ = 0.0;
    double detuneSpreadCentsZ_ = 0.0;
    double aWidth_  = 0.0;
    double aDetune_ = 0.0;

    // ---- Stereo look-ahead limiter (safe) ----
    struct LookaheadLimiter {
        static constexpr int kMax = 256;
        double bufL[kMax]{}, bufR[kMax]{}, absBuf[kMax]{};
        int w = 0, r = 0, size = 36;  // default ~0.75ms @48k
        int prime = 0;                 // priming samples left
        double target = 5.0, gr = 1.0, atkA = 0.0, relA = 0.0;

        void setup(double sr, double look_ms = 0.75, double atk_ms = 0.10, double rel_ms = 80.0, double targetVolts = 5.0) {
            target = targetVolts;
            int req = (int)std::lround(sr * (look_ms * 0.001));
            size = std::min(kMax - 1, std::max(8, req));
            w = 0;
            r = (kMax - size) % kMax;     // read trails write by <size>
            prime = size;                  // until filled, bypass delay path
            gr = 1.0;
            atkA = std::exp(-1.0 / (sr * (atk_ms * 0.001)));
            relA = std::exp(-1.0 / (sr * (rel_ms * 0.001)));
            for (int i = 0; i < kMax; ++i) bufL[i] = bufR[i] = absBuf[i] = 0.0;
        }

        inline void process(double inL, double inR, float& outL, float& outR) {
            if (!std::isfinite(inL)) inL = 0.0;
            if (!std::isfinite(inR)) inR = 0.0;

            // Priming: pass-through (soft-ceiling) until ring is ready
            if (prime > 0) {
                --prime;
                const double peak = std::max(1e-12, std::max(std::abs(inL), std::abs(inR)));
                double desired = target / peak;
                if (!std::isfinite(desired)) desired = 1.0;
                if (desired > 1.0) desired = 1.0;
                if (desired < 1e-4) desired = 1e-4;
                if (desired < gr) gr = desired + (gr - desired) * atkA;
                else              gr = desired + (gr - desired) * relA;
                if (!std::isfinite(gr) || gr < 1e-4) gr = 1e-4; else if (gr > 1.0) gr = 1.0;

                outL = float(inL * gr);
                outR = float(inR * gr);

                // still write into the ring so it fills
                bufL[w] = inL; bufR[w] = inR;
                absBuf[w] = peak;
                if (++w >= kMax) w = 0;
                if (++r >= kMax) r = 0;
                return;
            }

            // Normal look-ahead path
            bufL[w] = inL; bufR[w] = inR;
            double ab = std::max(std::abs(inL), std::abs(inR));
            if (!std::isfinite(ab)) ab = 0.0;
            absBuf[w] = ab;

            double peak = 1e-12;
            int idx = w;
            for (int i = 0; i < size; ++i) {
                const double a = absBuf[idx];
                if (a > peak) peak = a;
                if (--idx < 0) idx = kMax - 1;
            }

            double desired = target / peak;
            if (!std::isfinite(desired)) desired = 1.0;
            if (desired > 1.0) desired = 1.0;
            if (desired < 1e-4) desired = 1e-4;

            if (desired < gr) gr = desired + (gr - desired) * atkA;
            else              gr = desired + (gr - desired) * relA;

            if (!std::isfinite(gr) || gr < 1e-4) gr = 1e-4;
            else if (gr > 1.0) gr = 1.0;

            outL = float(bufL[r] * gr);
            outR = float(bufR[r] * gr);

            if (++w >= kMax) w = 0;
            if (++r >= kMax) r = 0;
        }
    };

    LookaheadLimiter limiter_;

    // Output slew limiter (pre-limiter)
    struct SlewLimiter {
        double step = 1e9; // volts/sample
        double y = 0.0;
        void setup(double sr, double v_per_ms) {
            step = (v_per_ms > 0.0) ? ((v_per_ms * 1000.0) / sr) : 1e9;
            // v_per_ms [V/ms] * 1000 = [V/s]; divide by sr -> [V/sample]
            y = 0.0;
        }
        inline double process(double x) {
            if (!std::isfinite(x)) x = 0.0;
            double d = x - y;
            if (d > step) d = step;
            else if (d < -step) d = -step;
            y += d;
            return y;
        }
    };
    SlewLimiter slewL_, slewR_;

    // Output analog path (LP + hum)
    OnePoleLP outLpL_, outLpR_;
    double humPhi50_ = 0.0, humPhi60_ = 0.0;

    // ----- Helpers -----
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

    static inline double sstep(double x) { // smoothstep 0..1
        x = std::max(0.0, std::min(1.0, x));
        return x * x * (3.0 - 2.0 * x);
    }

    void computeVoiceActivations_(double a, double act[7]) const;

    inline double renderCore(double phase, double dt, double morph01,
                             double timbre01, double& triState,
                             double& outSquareForTri,
                             double shapeSkew, double pwmBias) const;

    inline void advanceDrift(Voice& v, double depthCents);
};

} // namespace vm
