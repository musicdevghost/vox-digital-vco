#pragma once
#include <algorithm>

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

    inline float process(float x)
    {
        y += a * (x - y);
        return y;
    }
};

} // namespace vm
