#include "VoxAudioCore.hpp"
#include "Platform.hpp"

void VoxAudioCore::init(double sampleRate) {
    sr_ = sampleRate > 0.0 ? sampleRate : 48000.0;
    reset();

    // Simple HP coeff for tilt (~200 Hz corner)
    const float fc = 200.f;
    hp_a_ = fc / (float)sr_;
}

void VoxAudioCore::reset() {
    hp_zL_ = hp_zR_ = 0.f;
}

void VoxAudioCore::setParams(const Params& p) {
    params_ = p;
}

void VoxAudioCore::processBlock(const float* inL, const float* inR, float* outL, float* outR, size_t nFrames) {
    const float dryWet = clampf(params_.dryWet, 0.f, 1.f);
    const float gain   = clampf(params_.gain,   0.f, 2.f);
    const float tone   = clampf(params_.tone,   0.f, 1.f);

    // Tilt mix: low = (1 - tone), high = tone
    const float lowAmt  = 1.f - tone;
    const float highAmt = tone;

    for (size_t i = 0; i < nFrames; ++i) {
        float dl = inL ? inL[i] : 0.f;
        float dr = inR ? inR[i] : 0.f;

        // High-pass (one-pole leaky integrator form)
        // y = x - z; z += a * y;
        float yL = dl - hp_zL_;
        hp_zL_ += hp_a_ * yL;
        float yR = dr - hp_zR_;
        hp_zR_ += hp_a_ * yR;

        // Tilt: mix lows from input and highs from HP
        float wetL = lowAmt * dl + highAmt * yL;
        float wetR = lowAmt * dr + highAmt * yR;

        // Gain and soft clip
        wetL = softClip(wetL * gain);
        wetR = softClip(wetR * gain);

        // Dry/wet
        float ol = (1.f - dryWet) * dl + dryWet * wetL;
        float or_ = (1.f - dryWet) * dr + dryWet * wetR;

        // Flush denormals and safe clamp
        flushDenorm(ol); flushDenorm(or_);
        ol = clampf(ol, -10.f, 10.f);
        or_ = clampf(or_, -10.f, 10.f);

        outL[i] = ol;
        outR[i] = or_;
    }
}
