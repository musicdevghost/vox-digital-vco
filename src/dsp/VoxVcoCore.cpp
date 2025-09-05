#include "VoxVcoCore.hpp"
#include <cstring> // memset

namespace vm {

void VoxVcoCore::init(double sampleRate) {
    sr_ = (sampleRate > 0.0) ? sampleRate : 48000.0;
    invSr_ = 1.0 / sr_;
    fmHp_.setup(sr_, 5.0);
    rng_.seed(0xC0FFEEu);
    for (int i = 0; i < kMaxUnison; ++i) {
        voices_[i].rng.seed(0xC0FFEEu + 17u * (i + 1));
    }
    reset();
}

void VoxVcoCore::reset() {
    prevSync_ = 0.0f;
    for (auto& v : voices_) {
        v.phase = 0.0;
        v.triI = 0.0;
        v.parabPrev = 0.0;
        v.drift = 0.0;
    }
}

// NOTE: We assume your template's Params provides:
// - p.pitchVolts (V/Oct in volts)
// - p.macros[0..3] in [0..1] (centered macros may be mapped by wrapper to [-1..1])
// If your template differs, remap here (safe centralization).
void VoxVcoCore::setParams(const IDspCore::Params& p) {
    // ---- Pitch (V/Oct) ----
    P_.pitchVolts = p.pitchVolts; // OK if 0.0 when wrapper doesn't provide it

    // Prefer explicit macros[0..3]; if they look defaulted, fall back to dryWet/gain/tone/macro
    double m[4] = { p.macros[0], p.macros[1], p.macros[2], p.macros[3] };
    const bool macrosProvided = (m[0] != 0.0 || m[1] != 0.0 || m[2] != 0.0 || m[3] != 0.0);

    if (!macrosProvided) {
        m[0] = p.dryWet; // Macro A
        m[1] = p.gain;   // Macro B
        m[2] = p.tone;   // Macro C
        m[3] = p.macro;  // Macro D
    }

    // Clamp to [0..1]
    const double mA = fast_clip(m[0], 0.0, 1.0);
    const double mB = fast_clip(m[1], 0.0, 1.0);
    const double mC = fast_clip(m[2], 0.0, 1.0);
    const double mD = fast_clip(m[3], 0.0, 1.0);

    // Map to our internal roles
    P_.macroA = mA * 2.0 - 1.0;  // [-1..+1] → ±12 semis inside process
    P_.macroB = mB;              // morph 0..1
    P_.macroC = mC;              // unison/spread 0..1
    P_.macroD = mD;              // timbre / FM index 0..1

    // Unison / spread
    updateUnisonFromMacro(P_.macroC);

    // FM cable (wrapper may set; otherwise we infer in processBlock())
    P_.fmCable = p.fmCablePresent;
    fmEnabled_ = P_.fmCable;

    // Derived
    pwmWidth_ = 0.05 + 0.90 * P_.macroD;
    foldAmt_  = P_.macroD;
    chaosAmt_ = P_.macroD;
    fmIndex_  = std::sqrt(fast_clip(P_.macroD, 0.0, 1.0)); // FM index (0..1) — make the top half much stronger


    stereoWidth_ = 0.1 + 0.9 * P_.macroC;
    detuneSpreadCents_ = 30.0 * P_.macroC;
}

void VoxVcoCore::updateUnisonFromMacro(double mc) {
    if (mc < 0.125) unisonCount_ = 1;
    else if (mc < 0.375) unisonCount_ = 3;
    else if (mc < 0.625) unisonCount_ = 5;
    else unisonCount_ = 7;
}

void VoxVcoCore::advanceDrift(Voice& v, double depthCents) {
    // Very slow random walk ~0.3 Hz pole
    const double rate = 0.3; // Hz
    const double a = std::exp(-2.0 * M_PI * rate * invSr_);
    const double noise = v.rng.bipolar(); // [-1, 1]
    const double target = noise * (depthCents / 2.0);
    v.drift = a * v.drift + (1.0 - a) * target;
}

inline double morphMix(double a, double b, double x) {
    // equal-power-ish curve for nicer morph
    const double t = fast_clip(x, 0.0, 1.0);
    const double wa = std::cos(0.5 * M_PI * t);
    const double wb = std::sin(0.5 * M_PI * t);
    return a * wa + b * wb;
}

double VoxVcoCore::renderCore(double phase, double dt, double morph01,
                              double timbre01, double& triState,
                              double& outSquareForTri) const
{
    // Map morph01 [0..1] to zones 0..4
    const double morph4 = morph01 * 4.0;
    const int zone = (int)std::floor(morph4);
    const double zf = morph4 - zone; // in-zone morph [0..1]

    // Basic phasor
    double t = phase; // [0,1)

    // --- Primitive waveforms (bandlimited) ---
    // Saw (polyBLEP)
    double saw = 2.0 * t - 1.0;
    saw -= poly_blep(t, std::fabs(dt));

    // Square (PWM, polyBLEP at both edges)
    double w = fast_clip(timbre01 * 0.90 + 0.05, 0.05, 0.95);
    double sq = (t < w ? 1.0 : -1.0);
    sq += poly_blep(t, std::fabs(dt));
    // second edge at w
    double tw = t - w;
    if (tw < 0.0) tw += 1.0;
    sq -= poly_blep(tw, std::fabs(dt));

    outSquareForTri = sq;

    // Triangle: integrate BL square with leaky integrator
    // y[n] = y[n-1] + k * (sq - y[n-1]), with k ~ dt * 2 + small bias
    const double k = fast_clip(std::fabs(dt) * 2.0 + 1e-6, 1e-6, 1.0);
    triState += k * (sq - triState);
    double tri = triState;

    // Sine
    const double sine = std::sin(2.0 * M_PI * t);

    // Folded/chaotic core: start from sine, fold with tanh (or cubic softclip),
    // then add a faint highpass to remove DC.
#if 1
    const double folded = std::tanh( (1.0 + 4.0 * timbre01) * sine );
#else
    const double folded = softclip3((1.0 + 4.0 * timbre01) * sine);
#endif

    // --- Morph zones ---
    double y = 0.0;
    switch (zone) {
        default:
        case 0: // Sine -> Triangle
            y = morphMix(sine, tri, zf);
            break;
        case 1: // Triangle -> Saw
            y = morphMix(tri, saw, zf);
            break;
        case 2: // Saw -> PWM Square (duty uses timbre01 already)
            y = morphMix(saw, sq, zf);
            break;
        case 3: // PWM Square -> Folded/Chaotic (depth from timbre01)
        {
            const double fold = folded;
            // Blend toward a very mild chaotic flavor by adding tiny feedback-ish term
            // The caller can add HP later; here just crossfade.
            y = morphMix(sq, fold, zf);
            break;
        }
    }
    return y;
}

void VoxVcoCore::processBlock(const float* inL, const float* inR,
                              float* outL, float* outR, int n)
{
    if (!outL || !outR || n <= 0) return;

    // Detect FM cable if wrapper doesn't provide it: measure RMS over the block
    // (lightweight). Use this only if wrapper didn't set fmCablePresent.
    bool fmCableInfer = false;
    if (!P_.fmCable && inR) {
        double accum = 0.0;
        for (int i = 0; i < n; i += 8) {
            const float s = inR[i];
            accum += double(s) * double(s);
        }
        fmCableInfer = (accum > 1e-6);
    }
    fmEnabled_ = P_.fmCable || fmCableInfer;

    // Base frequency from V/Oct + MacroA semitone offset
    const double semis = P_.macroA * 12.0; // ±12
    const double baseHz = fast_clip(kC4 * std::pow(2.0, P_.pitchVolts + semis / 12.0), kMinHz, kMaxHz);

    // Precompute per-voice pan positions and detunes
    // Symmetric pan [0..1]
    double panPos[kMaxUnison] = {0};
    // Symmetric detune in cents
    double detC[kMaxUnison] = {0};

    if (unisonCount_ == 1) {
        panPos[0] = 0.5;
        detC[0] = 0.0;
    } else {
        // Spread voices across stereo; center stays near 0.5 for odd counts
        for (int i = 0; i < unisonCount_; ++i) {
            const double idx01 = (unisonCount_ <= 1) ? 0.5 : double(i) / double(unisonCount_ - 1);
            const double centered = (idx01 - 0.5);
            panPos[i] = 0.5 + centered * stereoWidth_;
            // Triangular symmetric detune profile
            const double sgn = (i % 2 == 0) ? -1.0 : +1.0;
            const double mag = (double(i) / double(unisonCount_ - 1));
            detC[i] = sgn * mag * detuneSpreadCents_;
        }
    }

    // Process
    float prevSync = prevSync_;
    prevSync_ = (n > 0 && inL) ? inL[n - 1] : prevSync_;

    for (int i = 0; i < n; ++i) {
        // Rising edge on IN L triggers hard sync
        bool hardSync = false;
        if (inL) {
            const float s = inL[i];
            hardSync = (prevSync <= 1.0f && s > 1.0f);
            prevSync = s;
        }

        // FM input sample
        double fmSample = 0.0;
        if (fmEnabled_ && inR) {
            fmSample = fmHp_.process(double(inR[i])); // DC block
        }

        double L = 0.0, R = 0.0;

        // Each voice
        for (int v = 0; v < unisonCount_; ++v) {
            auto& V = voices_[v];

            // Advance drift (3..7 cents depth scaled with macroC)
            advanceDrift(V, 3.0 + 4.0 * P_.macroC);
            const double cents = detC[v] + V.drift;
            const double ratio = detuneCentsToRatio(cents);

            // Per-voice instantaneous frequency (through-zero FM in Hz)
            double f = baseHz * ratio;
            if (fmEnabled_) {
                double norm = fast_clip(fmSample / 5.0, -1.0, 1.0);
                // Hybrid FM depth: mostly absolute (Hz), plus a bit relative to base
                // - Absolute part gives wide sidebands on low notes
                // - Relative part keeps FM audible at higher notes
                const double absHz = kFmMaxHz * fmIndex_;
                const double relHz = baseHz * (1.5 * fmIndex_);   // up to ~1.5× base at full
                const double devHz = absHz * 0.7 + relHz * 0.3;

                f += devHz * norm;
            }
            f = fast_clip(f, -kMaxHz, kMaxHz); // allow negative (through-zero)

            const double dt = f * invSr_;

            // Hard sync: reset phase with BLEP correction
            if (hardSync) {
                // reset near zero; polyBLEP will smooth on next sample
                V.phase = 0.0;
                // reset triangle integrator to avoid click
                V.triI *= 0.5;
            }

            // Wave render
            double sqForTri = 0.0;
            const double y = renderCore(V.phase, dt,
                                        P_.macroB, P_.macroD,
                                        V.triI, sqForTri);

            // Pan and mix
            const double pan = fast_clip(panPos[v], 0.0, 1.0);
            const double gL = equalPowerL(pan);
            const double gR = equalPowerR(pan);

            L += y * gL;
            R += y * gR;

            // Advance phase (through-zero OK)
            V.phase += dt;
            // Wrap to [0,1)
            V.phase -= std::floor(V.phase);
        }

        // Normalize by voice count and scale to target volts
        const double invN = 1.0 / double(unisonCount_);
        double yL = L * invN;
        double yR = R * invN;
        // simple safety clamp so FM/fold don’t explode; keep linear up to ±1
        yL = fast_clip(yL, -1.0, 1.0);
        yR = fast_clip(yR, -1.0, 1.0);
        outL[i] = float(yL * kOutGain);
        outR[i] = float(yR * kOutGain);
    }

    // store prevSync
    prevSync_ = prevSync;
}

} // namespace vm
