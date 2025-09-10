#pragma once
#include <cmath>

namespace vm {

// One-pole DC blocker:
//   y[n] = x[n] - x[n-1] + R * y[n-1]
// Choose R close to 1.0 for low cutoff (e.g., 0.995 at 48 kHz ~ 15–20 Hz).
struct DcBlock
{
    float x1 = 0.f; // previous input
    float y1 = 0.f; // previous output
    float R  = 0.995f;

    inline float process(float x)
    {
        const float y = x - x1 + R * y1;
        x1 = x;
        y1 = y;
        return y;
    }

    inline void reset(float r = 0.995f)
    {
        x1 = 0.f;
        y1 = 0.f;
        R  = r;
    }
};

} // namespace vm
