#pragma once
#include <cmath>
#include <cstdint>

namespace vm { namespace vox { namespace glue {

// ===== Helpers =====
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; }
static inline float bi_to_uni(float b){ return (b * 0.5f) + 0.5f; }

static inline float quantize12(float x01){
    if(x01 < 0.f) x01 = 0.f;
    if(x01 > 1.f) x01 = 1.f;
    int q = int(std::lround(x01 * 4095.f));
    return float(q) / 4095.f;
}

// Rack volts (±10V) to bipolar -1..+1
static inline float voltsToBipolar(float v){
    float n = v / 10.f;
    if(n < -1.f) n = -1.f;
    if(n >  1.f) n =  1.f;
    return n;
}

// Apply attenuverter for Rack path
static inline float apply_cv_att_rack(float knob01, float cvV, float att01){
    float av = uni_to_bi(att01);           // -1..+1
    float mod = av * voltsToBipolar(cvV);  // -1..+1
    return clamp01(knob01 + mod);
}

// Apply attenuverter for Hardware path (cv01 centered around 0.5)
static inline float apply_cv_att_hw(float knob01, float cv01, float att01){
    float av = uni_to_bi(att01);           // -1..+1
    float mod = av * (cv01 - 0.5f);        // -0.5..+0.5
    return clamp01(knob01 + mod);
}

// Simple AR env follower for presence detect
struct Env {
    float a = 0.01f, r = 0.001f, y = 0.f;
    float process(float xabs){
        float c = (xabs > y) ? a : r;
        y = (1.f - c) * y + c * xabs;
        return y;
    }
};

}}} // namespace vm::vox::glue
