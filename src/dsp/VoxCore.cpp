#include "VoxCore.hpp"

namespace vm { namespace vox {

void VoxCore::processBlock(const CoreParams& p,
                           const Controls& k,
                           const Mods& m,
                           State& s,
                           float* outL, float* outR, int nframes) {
    params_ = p;

    const double semisSpan = params_.kPitchMacroOctaves * 12.0;
    const double knobSemis = (clamp01(k.pitchKnob01) * 2.0 - 1.0) * semisSpan;
    const double f0 = hzFromSemis(knobSemis, params_.baseA4);
    const double sr = params_.sampleRate;
    const double dt = 1.0 / sr;

    // Discrete waveform selection (Illusions-style)
    int wsel = (int)std::floor(clamp01(k.morph01) * (int)Wave::COUNT);
    if (wsel >= (int)Wave::COUNT) wsel = (int)Wave::COUNT - 1;
    const Wave wave = static_cast<Wave>(wsel);

    // Timbre→duty for square (5–95%); ignored by other waves for now
    const double duty = clampDuty(0.5 + (clamp01(k.timbre01) - 0.5) * 0.9);
    const double amp  = clamp01(k.spread01);

    float prev_hs = hs_prev_;

    for (int i = 0; i < nframes; ++i) {
        // Hard sync (if provided): reset on rising zero-crossing
        if (m.hsync) {
            const float hs = m.hsync[i];
            if (prev_hs <= 0.f && hs > 0.f) {
                s.phase = 0.0;
            }
            prev_hs = hs;
        }
        // Soft sync (if provided)
        if (m.ssync && m.ssync[i] > 0.5f) {
            s.phase = 0.0;
        }

        // Linear FM in Hz
        double fmHz = 0.0;
        if (m.fm && m.fmDepthHz != 0.0) {
            fmHz = (double)m.fm[i] * m.fmDepthHz;
        }
        double freq = f0 + fmHz;
        if (freq < 0.0) freq = 0.0;
        const double inc = freq * dt;

        // advance phase
        s.phase += inc;
        s.phase -= std::floor(s.phase);

        // waveform
        double y = 0.0;
        switch (wave) {
            case Wave::SINE:     y = osc_sine(s.phase); break;
            case Wave::TRIANGLE: y = osc_triangle(s.phase); break;
            case Wave::SAW:      y = osc_saw(s.phase, inc); break;
            case Wave::SQUARE:   y = osc_square(s.phase, duty, inc); break;
            default:             y = osc_sine(s.phase); break;
        }

        y *= amp;
        outL[i] = (float)y;
        outR[i] = (float)y;
    }

    hs_prev_ = prev_hs;
}

}} // namespace vm::vox
