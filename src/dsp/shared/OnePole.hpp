#pragma once
#include <algorithm>
#include <cmath>

namespace vm {

// Simple one-pole low-pass smoother for controls (allocation-free).
struct OnePole
{
    float y = 0.f;   // state
    float a = 0.12f; // alpha in [0..1]; larger = faster

    inline void setAlpha(float alpha)
    {
        a = std::max(0.f, std::min(alpha, 1.f));
    }

    // Set smoothing by time constant in milliseconds (SR-aware).
    // Resulting alpha: a = 1 - exp(-1 / (tau * sr))
    inline void setTauMs(float sr, float ms)
    {
        const float tau = std::max(0.0001f, ms * 1e-3f);
        a = 1.f - std::exp(-1.f / (tau * sr));
        if (a < 0.f) a = 0.f;
        if (a > 1.f) a = 1.f;
    }

    inline float process(float x)
    {
        y += a * (x - y);
        return y;
    }

    // Back-compat alias used in existing HAL
    inline float proc(float x) { return process(x); }
};

} // namespace vm
