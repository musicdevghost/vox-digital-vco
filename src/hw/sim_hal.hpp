#pragma once
#include <cstdint>
#include <cmath>
#include "../dsp/VoxCore.hpp"

namespace vm { namespace vox {

struct SimHal {
    double pitchKnob01 = 0.5; // UI value 0..1

    static inline double quantize12(double x01) {
        if (x01 < 0.0) x01 = 0.0;
        if (x01 > 1.0) x01 = 1.0;
        const int q = int(std::lround(x01 * 4095.0));
        return double(q) / 4095.0;
    }

    Controls buildControls() const {
        Controls c;
        c.pitchKnob01 = quantize12(pitchKnob01);
        return c;
    }
};

}} // namespace vm::vox
