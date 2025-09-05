#pragma once
#include <cmath>
#include <cfloat>

#ifndef TA_ENABLE_SOFTCLIP
#define TA_ENABLE_SOFTCLIP 1
#endif

inline void flushDenorm(float& x) {
#if defined(__arm__) || defined(__aarch64__)
    if (std::fabs(x) < 1.0e-30f) x = 0.f;
#else
    if (std::fpclassify(x) == FP_SUBNORMAL) x = 0.f;
#endif
}

inline float softClip(float x) {
#if TA_ENABLE_SOFTCLIP
    return x * (27.f + x*x) / (27.f + 9.f*x*x);
#else
    return x;
#endif
}

inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}
