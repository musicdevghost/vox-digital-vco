#pragma once
#include <algorithm>

namespace vm {

// Simple one-pole low-pass smoother for controls (allocation-free).
struct OnePole {
    float y=0.f, a=0.12f;
    inline void setAlpha(float alpha){ a = alpha < 0.f ? 0.f : (alpha > 1.f ? 1.f : alpha); }
    inline void setTauMs(float sr, float ms){
        const float tau = std::max(0.01f, ms * 1e-3f);
        a = 1.f - std::exp(-1.f / (tau * sr));
    }
    inline float process(float x){ y += a*(x-y); return y; }
};

} // namespace vm
