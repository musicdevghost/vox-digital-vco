#pragma once
#include <cstddef>

struct VoxAudioCore {
    void init(double sampleRate);
    void reset();

    struct Params {
        float dryWet; // 0..1
        float gain;   // 0..2
        float tone;   // 0..1 (tilt amount)
        float macro;  // free macro (unused in template)
    };

    void setParams(const Params& p);

    // Non-interleaved stereo block process
    void processBlock(const float* inL, const float* inR, float* outL, float* outR, size_t nFrames);

  private:
    double sr_ = 48000.0;
    Params params_{};

    // Tiny tilt EQ state (one-pole HP on "bright" path)
    float hp_zL_ = 0.f, hp_zR_ = 0.f;
    float hp_a_ = 0.f; // coefficient for simple HP (leaky integrator form)
};
