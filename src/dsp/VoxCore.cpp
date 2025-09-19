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

    // --- slow analog-like drift (deterministic for repeatability) ---
    // Update target every ~0.5 s
    if (s.driftCountdown <= 0) {
        // target in +/- 3 cents equivalent (very gentle)
        // cents -> ratio ≈ 2^(c/1200) ≈ 1 + c*ln(2)/1200 for small c
        double cents = 3.0 * lcg_bip(s.rng); // -3..+3 cents
        // store as fractional freq offset
        s.driftTarget = std::pow(2.0, cents / 1200.0) - 1.0;
        s.driftCountdown = (int)(sr * 0.5); // 0.5 s
    }
    s.drift += (s.driftTarget - s.drift) * 0.0005; // slow slew
    s.driftCountdown -= nframes;

    // --- Morph path: Sine → Triangle → Saw → Square (equal-power blends) ---
    double morph = clamp01(k.morph01);
    double seg   = morph * 3.0;
    int    idx   = (int)std::floor(seg);
    if (idx > 2) idx = 2;
    double tseg  = seg - (double)idx;

    // Square PWM duty from timbre; for other waves timbre shapes tone via soft saturation/curvature
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

        // Apply slow drift to the nominal frequency
        double f0 = f0_nominal * (1.0 + s.drift);
        double freq = f0 + fmHz;
        if (freq < 0.0) freq = 0.0;
        const double inc = freq * dt;

        // advance phase
        s.phase += inc;
        s.phase -= std::floor(s.phase);

        // --- Wave generation per segment ---
        // Precompute bases
        double wsine = osc_sine(s.phase);
        double wtri  = tri_shape(s.phase, (clamp01(k.timbre01) - 0.5) * 0.6); // subtle curvature
        double wsaw  = osc_saw_blep(s.phase, inc);
        double wsqr  = osc_square_pwm_blep(s.phase, duty, inc);

        // Per-wave subtle soft saturation based on timbre (except square which already uses PWM)
        double satAmt = clamp01(k.timbre01) * 0.35; // gentle
        wsine = soft_sat(wsine, satAmt * 0.6);
        wtri  = soft_sat(wtri,  satAmt * 0.4);
        wsaw  = soft_sat(wsaw,  satAmt * 0.5);
        // square left clean except small saturation for parity
        wsqr  = soft_sat(wsqr,  satAmt * 0.2);

        double y = 0.0;
        if (idx == 0) {
            // Sine -> Triangle
            y = xfade_equal_power(wsine, wtri, tseg);
        } else if (idx == 1) {
            // Triangle -> Saw
            y = xfade_equal_power(wtri, wsaw, tseg);
        } else {
            // Saw -> Square (PWM)
            y = xfade_equal_power(wsaw, wsqr, tseg);
        }

        y *= amp;
        outL[i] = (float)y;
        outR[i] = (float)y;
    }

    hs_prev_ = prev_hs;
}

}} // namespace vm::vox
