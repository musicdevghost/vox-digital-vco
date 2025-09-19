
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
    const double f0_nominal = hzFromSemis(knobSemis, params_.baseA4);
    const double sr = params_.sampleRate;
    const double dt = 1.0 / sr;

    // --- slow analog-like drift (deterministic) ---
    if (s.driftCountdown <= 0) {
        double cents = 3.0 * lcg_bip(s.rng);      // -3..+3 cents
        s.driftTarget = std::pow(2.0, cents / 1200.0) - 1.0;
        s.driftCountdown = (int)(sr * 0.5);       // update every ~0.5s
    }
    s.drift += (s.driftTarget - s.drift) * 0.0005;
    s.driftCountdown -= nframes;

    // --- Morph path: 0:Sine/Tri (fold) -> 1:Tri -> 2:Saw -> 3:Square -> 4:Additive ---
    double morph = clamp01(k.morph01);
    double seg   = morph * 4.0;   // 4 segments
    int    idx   = (int)std::floor(seg);
    if (idx > 3) idx = 3;
    double tseg  = seg - (double)idx;

    // Controls
    const double duty = clampDuty(0.5 + (clamp01(k.timbre01) - 0.5) * 0.9);
    const double tim  = clamp01(k.timbre01);

    float prev_hs = hs_prev_;

    // Amplitude normalization targets / smoothing
    const double targetRms = 0.70710678; // ~1/sqrt(2) so ±1 sine ~ ±1 peak
    const double eps = 1e-9;
    const double rmsCoef  = std::exp(-1.0 / (sr * 0.02)); // ~20ms rms window
    const double gainCoef = std::exp(-1.0 / (sr * 0.01)); // ~10ms gain slew

    for (int i = 0; i < nframes; ++i) {
        // Hard sync (rising zero-cross)
        if (m.hsync) {
            const float hs = m.hsync[i];
            if (prev_hs <= 0.f && hs > 0.f)
                s.phase = 0.0;
            prev_hs = hs;
        }
        // Soft sync
        if (m.ssync && m.ssync[i] > 0.5f)
            s.phase = 0.0;

        // Linear FM in Hz
        double fmHz = (m.fm && m.fmDepthHz != 0.0) ? (double)m.fm[i] * m.fmDepthHz : 0.0;

        // Frequency with slow drift
        double f0 = f0_nominal * (1.0 + s.drift);
        double freq = f0 + fmHz;
        if (freq < 0.0) freq = 0.0;
        const double inc = freq * dt;

        // advance phase
        s.phase += inc;
        s.phase -= std::floor(s.phase);

        // Base shapes
        double tri_lin = osc_triangle_linear(s.phase);
        double wsine   = osc_sine(s.phase);
        double wtri    = tri_shape(s.phase, (tim - 0.5) * 0.6);
        double wsaw    = osc_saw_blep(s.phase, inc);
        double wsqr    = osc_square_pwm_blep(s.phase, duty, inc);

        // Folded shapes for continuity at seg0/1 boundary
        double fsine = fold_sym(wsine, tim);
        double ftri  = fold_sym(tri_lin, tim);

        // Additive endpoint
        double addTri = additive_triangle(s.phase, freq, sr, tim);
        double addSin = additive_sine    (s.phase, freq, sr, tim);
        double wadd   = xfade_equal_power(addTri, addSin, tseg); // inside seg3

        // Segment outputs with boundary continuity
        double yseg = 0.0;
        if (idx == 0) {
            // Sine->Triangle with folding (both folded)
            yseg = xfade_equal_power(fsine, ftri, tseg);
        } else if (idx == 1) {
            // Start from the EXACT folded triangle (continuity), then drift to shaped tri, then to saw
            double triBridge = xfade_equal_power(ftri, wtri, tseg);
            yseg = xfade_equal_power(triBridge, wsaw, tseg);
        } else if (idx == 2) {
            yseg = xfade_equal_power(wsaw, wsqr, tseg);
        } else { // idx == 3
            yseg = xfade_equal_power(wsqr, wadd, tseg);
        }

        // Gentle per-wave saturation tied to timbre (kept after morph for consistency)
        double satAmt = tim * 0.25;
        double y = soft_sat(yseg, satAmt);

        // --- Amplitude normalization (Spread ignored) ---
        // Update RMS estimate
        s.rms = rmsCoef * s.rms + (1.0 - rmsCoef) * (y * y);
        // Desired gain to hit target RMS; clamp without std::clamp (use fmin/fmax)
        double desiredGain = std::sqrt((targetRms + eps) / (s.rms + eps));
        desiredGain = std::fmax(0.5, std::fmin(desiredGain, 2.0));
        // Slew the output gain
        s.outGain = gainCoef * s.outGain + (1.0 - gainCoef) * desiredGain;

        double yout = y * s.outGain;

        outL[i] = (float)yout;
        outR[i] = (float)yout;
    }

    hs_prev_ = prev_hs;
}

}} // namespace vm::vox
