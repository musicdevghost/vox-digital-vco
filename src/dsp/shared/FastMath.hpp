#pragma once
#include <cmath>

namespace vm {

// Lightweight helpers useful in cores without big tables.

inline float fast_tanh(float x)
{
    // Cubic rational approximation with soft clip.
    // Accurate enough for musical saturation, branchless.
    const float x2 = x * x;
    float y = x * (27.f + x2) / (27.f + 9.f * x2);
    if (y > 1.f)  y = 1.f;
    if (y < -1.f) y = -1.f;
    return y;
}

inline float saturate01(float x)
{
    return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}

} // namespace vm
