#pragma once
#include <cstdint>
#include <cmath>
#include "../dsp/VoxCore.hpp"
#include "../framework/TemplateIO.hpp"
#include <rack.hpp>

namespace vm { namespace vox {

struct SimHal {
    // Raw UI values (0..1 knobs)
    double knobPitch01  = 0.5;
    double knobMorph01  = 0.0;
    double knobTimbre01 = 0.5;
    double knobSpread01 = 1.0;

    // Attenuverters (-1..1 domain after mapping from 0..1)
    double attPitch   = 0.0;
    double attMorph   = 0.0;
    double attTimbre  = 0.0;
    double attSpread  = 0.0;

    // CV inputs in volts (±10 V typical)
    double cvPitchV   = 0.0; // reserved for Phase C expo
    double cvMorphV   = 0.0;
    double cvTimbreV  = 0.0;
    double cvSpreadV  = 0.0;

    static inline double clamp01(double x) { return x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x); }
    static inline double quantize12(double x01) {
        if (x01 < 0.0) x01 = 0.0;
        if (x01 > 1.0) x01 = 1.0;
        const int q = int(std::lround(x01 * 4095.0));
        return double(q) / 4095.0;
    }
    static inline double voltsToBipolar(double v) {
        const double norm = v / 10.0;
        if (norm < -1.0) return -1.0;
        if (norm >  1.0) return  1.0;
        return norm;
    }
    static inline double uni_to_bi(double u) { return (u * 2.0) - 1.0; }

    static inline double apply_cv_att(double knob01, double cvV, double att01) {
        const double av = uni_to_bi(att01);        // -1..+1
        const double mod = av * voltsToBipolar(cvV); // bipolar from volts
        return clamp01(knob01 + mod);
    }

    Controls buildControls() const {
        Controls c;
        // Pitch macro: knob-only macro, quantized (match hardware choice by default)
        c.pitchKnob01 = quantize12(apply_cv_att(knobPitch01, cvPitchV, attPitch));

        // Morph/Timbre/Spread: knob + CV via attenuverters
        c.morph01  = quantize12(apply_cv_att(knobMorph01,  cvMorphV,  attMorph));
        c.timbre01 = quantize12(apply_cv_att(knobTimbre01, cvTimbreV, attTimbre));
        c.spread01 = quantize12(apply_cv_att(knobSpread01, cvSpreadV, attSpread));

        return c;
    }
};

}} // namespace vm::vox
