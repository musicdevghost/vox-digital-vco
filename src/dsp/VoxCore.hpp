#pragma once
#include <cmath>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vm { namespace vox {

static inline double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }

struct CoreParams {
    double sampleRate = 48000.0;
    double baseA4 = 440.0;
    int    kPitchMacroOctaves = 5;
};

struct Controls {
    double pitchKnob01 = 0.5;
    double morph01     = 0.0; // 0=square,1=sine
    double timbre01    = 0.5; // PWM depth
    double spread01    = 1.0; // amplitude
};

struct Mods {
    // Audio-rate inputs (may be nullptr if unused)
    const float* hsync = nullptr; // use rising zero-crossings for hard sync
    const float* fm    = nullptr; // linear FM input (-1..+1 suggested)
    const float* ssync = nullptr; // soft sync gate ( >0.5 triggers reset )
    double fmDepthHz = 0.0;       // FM depth in Hz (0 to disable)
};

struct State {
    double phase = 0.0;
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
        params_ = p;

        const double semisSpan = params_.kPitchMacroOctaves * 12.0;
        const double knobSemis = (clamp01(k.pitchKnob01) * 2.0 - 1.0) * semisSpan;
        const double f0 = hzFromSemis(knobSemis, params_.baseA4);
        const double dt = 1.0 / params_.sampleRate;

        const double morph = clamp01(k.morph01);
        const double pwmDepth = clamp01(k.timbre01);
        const double amp = clamp01(k.spread01);

        const double duty = clampDuty(0.5 + (pwmDepth - 0.5) * 0.9);

        float prev_hs = 0.f;
        if (m.hsync) prev_hs = m_hsPrev; // keep continuity across blocks

        for (int i = 0; i < nframes; ++i) {
            // Hard/soft sync handling
            if (m.hsync) {
                float hs = m.hsync[i];
                if (prev_hs <= 0.f && hs > 0.f) {
                    s.phase = 0.0;
                }
                prev_hs = hs;
            }
            if (m.ssync && m.ssync[i] > 0.5f) {
                s.phase = 0.0;
            }

            // Linear FM (Hz) at audio rate
            double fmHz = 0.0;
            if (m.fm && m.fmDepthHz != 0.0) {
                fmHz = (double)m.fm[i] * m.fmDepthHz;
            }

            double freq = f0 + fmHz;
            if (freq < 0.0) freq = 0.0;

            s.phase += freq * dt;
            if (s.phase >= 1.0) {
                s.phase -= std::floor(s.phase);
            }

            double pulse = (s.phase < duty) ? 1.0 : -1.0;
            double sine  = std::sin(2.0 * M_PI * s.phase);
            double y = (1.0 - morph) * pulse + morph * sine;
            y *= amp;

            outL[i] = (float)y;
            outR[i] = (float)y;
        }
        m_hsPrev = prev_hs;
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
    float m_hsPrev = 0.f;
};

}} // namespace vm::vox
