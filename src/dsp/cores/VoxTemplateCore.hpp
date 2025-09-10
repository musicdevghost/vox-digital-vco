#pragma once
#include <cmath>
#include "dsp/IDspCore.hpp"
#include "dsp/shared/DcBlock.hpp"
#include "dsp/shared/FastMath.hpp"

// Minimal, allocation-free example core implementing your IDspCore interface.
// Purpose: Give you a tiny, readable starting point for new modules.
namespace vm {

class VoxTemplateCore final : public IDspCore
{
public:
    void init(double sampleRate) override
    {
        sr_ = (sampleRate > 0.0) ? sampleRate : 48000.0;
        reset();
    }

    void reset() override
    {
        phase_ = 0.0;
        freq_  = 220.0;
        width_ = 0.0;
        drive_ = 1.0;
        mix_   = 0.0;
        dcL_.reset();
        dcR_.reset();
    }

    void setParams(const IDspCore::Params& p) override
    {
        // Pitch: prefer true 1V/oct if provided, otherwise map 0..1 ~ 20..8kHz
        if (std::isfinite(p.pitchVolts) && p.pitchVolts != 0.0)
        {
            double hz = 16.35 * std::pow(2.0, p.pitchVolts); // C0 base
            if (!(hz > 0.0)) hz = 16.35;
            freq_ = clamp(hz, 20.0, 8000.0);
        }
        else
        {
            double hz = 20.0 * std::pow(2.0, 10.0 * clamp(p.pitch, 0.0f, 1.0f));
            freq_ = clamp(hz, 20.0, 8000.0);
        }

        // Stereo width (0..~1), Timbre → saturation drive, Morph → in/out mix
        width_ = clamp(p.spread, 0.0f, 1.0f) * 0.95;
        drive_ = 0.2 + 1.8 * clamp(p.timbre, 0.0f, 1.0f);
        mix_   = clamp(p.morph, 0.0f, 1.0f);
    }

    // inL: Sync/Gate (unused here), inR: FM (unused here). outs must be valid, n > 0.
    void processBlock(const float* inL, const float* inR,
                      float* outL, float* outR, int n) override
    {
        (void)inL; (void)inR;
        if (!outL || !outR || n <= 0) return;

        const double dt = 1.0 / sr_;

        for (int i = 0; i < n; ++i)
        {
            // Simple tri+sine hybrid → soft saturation by "timbre"
            phase_ += freq_ * dt;
            if (phase_ >= 1.0) phase_ -= 1.0;

            // Triangle
            double tri = 2.0 * std::fabs(2.0 * (phase_ - std::floor(phase_ + 0.5))) - 1.0;
            // Sine
            double sine = std::sin(2.0 * M_PI * phase_);
            // Hybrid
            double sig = 0.5 * (sine + tri);

            // Timbre saturation
            sig = fast_tanh(float(sig * drive_));

            // No external input in this example; crossfade against silence by morph
            const double x = mix_ * sig; // (1-mix)*0 + mix*sig

            // Stereo width (simple M/S)
            double L = x * (1.0 + width_);
            double R = x * (1.0 - width_);

            // DC protection
            L = dcL_.process(float(L));
            R = dcR_.process(float(R));

            outL[i] = float(L);
            outR[i] = float(R);
        }
    }

private:
    template <typename T>
    static inline T clamp(T x, T lo, T hi) { return x < lo ? lo : (x > hi ? hi : x); }

    double sr_{48000.0};
    double phase_{0.0};
    double freq_{220.0};
    double width_{0.0};
    double drive_{1.0};
    double mix_{0.0};
    DcBlock dcL_{}, dcR_{};
};

} // namespace vm
