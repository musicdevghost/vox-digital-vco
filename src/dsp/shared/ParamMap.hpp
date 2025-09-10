#pragma once
#include <algorithm>
#include <cmath>

namespace vm {

// Default assumption for your Pitch CV full-scale.
// Change at build time with: -DVM_VOCT_RANGE_VOLTS=5.0  (if your CV path is 0..5V)
#ifndef VM_VOCT_RANGE_VOLTS
#define VM_VOCT_RANGE_VOLTS 10.0f
#endif

struct ParamMap
{
    // ----- Basic clamps -----
    static inline float clamp01(float x)
    {
        return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
    }

    static inline float clamp(float x, float lo, float hi)
    {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    // ----- Shapes / mappings -----

    // Map 0..1 → -1..+1 (center at 0.5)
    static inline float bipolar(float x01)
    {
        float x = clamp01(x01);
        return x * 2.f - 1.f;
    }

    // Slightly exponential shape in 0..1 (musical feel).
    // k in [0..+inf). k=0 is linear, k>0 increases curvature.
    static inline float expo01(float x01, float k = 4.0f)
    {
        float x = clamp01(x01);
        if (k <= 0.f) return x;
        return (std::exp(k * x) - 1.f) / (std::exp(k) - 1.f);
    }

    // Crossfade a..b by t in 0..1 (no shape)
    static inline float lerp(float a, float b, float t)
    {
        return a + (b - a) * clamp01(t);
    }

    // ----- Volts mapping -----

    // Map normalized CV (0..1) to volts with configurable full-scale.
    static inline float normToVolts(float x01, float fullscale = VM_VOCT_RANGE_VOLTS)
    {
        return clamp01(x01) * fullscale;
    }

    static inline double normToVolts(double x01, double fullscale = double(VM_VOCT_RANGE_VOLTS))
    {
        double x = (x01 < 0.0) ? 0.0 : (x01 > 1.0 ? 1.0 : x01);
        return x * fullscale;
    }

    // 1V/oct: volts → Hz with base frequency f0 (C0 ≈ 16.35 Hz by default).
    static inline float voltsToHz1VPerOct(float volts, float f0 = 16.35f)
    {
        return f0 * std::pow(2.f, volts);
    }

    static inline double voltsToHz1VPerOct(double volts, double f0 = 16.35)
    {
        return f0 * std::pow(2.0, volts);
    }

    // ----- Convenience mixers -----

    // Combine a knob (0..1) with a CV (already mapped to bipolar -1..+1)
    // scaled by a bipolar attenuverter (-1..+1). Result clamped to 0..1.
    static inline float knobPlusCv(float knob01, float cvBipolar, float attBipolar)
    {
        return clamp01(knob01 + cvBipolar * attBipolar);
    }

    // Same but CV given as normalized (0..1). Converts CV to bipolar first.
    static inline float knobPlusCv01(float knob01, float cv01, float attBipolar)
    {
        return clamp01(knob01 + bipolar(cv01) * attBipolar);
    }
};

} // namespace vm
