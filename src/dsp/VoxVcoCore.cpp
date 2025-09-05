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

// Params mapping:
// - pitchVolts: true 1 V/oct (0V=C4)
// - Macro A: pitch offset (±12 semis) -> P_.macroA in [-1..+1]
// - Macro B: morph (your UI 0..2) -> normalized to [0..1]
// - Macro C: unison/spread [0..1]
// - Macro D: timbre / FM index [0..1] (sqrt curve for hotter top)
void VoxVcoCore::setParams(const IDspCore::Params& p) {
    // ---- Pitch (V/Oct) ----
    P_.pitchVolts = p.pitchVolts; // OK if 0.0 when wrapper doesn't provide it

    // Prefer explicit macros[0..3]; if they look defaulted, fall back to dryWet/gain/tone/macro
    double m[4] = { p.macros[0], p.macros[1], p.macros[2], p.macros[3] };
    const bool macrosProvided = (m[0] != 0.0 || m[1] != 0.0 || m[2] != 0.0 || m[3] != 0.0);

    if (!macrosProvided) {
        m[0] = p.dryWet; // Macro A
        m[1] = p.gain;   // Macro B (note: your UI uses 0..2)
        m[2] = p.tone;   // Macro C
        m[3] = p.macro;  // Macro D
    }

    // Map & clamp
    const double mA = fast_clip(m[0], 0.0, 1.0);
    const double mB_raw = fast_clip(m[1], 0.0, 2.0); // accept UI's 0..2
    const double mB_norm = mB_raw * 0.5;             // normalize to 0..1 for the core
    const double mC = fast_clip(m[2], 0.0, 1.0);
    const double mD = fast_clip(m[3], 0.0, 1.0);

    P_.macroA = mA * 2.0 - 1.0;  // [-1..+1] → ±12 semis inside process
    P_.macroB = mB_norm;         // morph 0..1
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
    fmIndex_  = std::sqrt(fast_clip(P_.macroD, 0.0, 1.0)); // hotter top

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
    // Map morph01 [0..1] to zones 0..4, clamp to last zone
    const double morph4 = morph01 * 4.0;
    int zone = (int)std::floor(morph4);
    if (zone > 3) zone = 3;          // avoid zone==4 wrap
    const double zf = morph4 - std::floor(morph4); // in-zone morph [0..1)

    // Basic phasor
    double t = phase; // [0,1)
    const double dtAbs = std::fabs(dt);

    // ---------- Primitives ----------
    // Saw (polyBLEP)
    double saw = 2.0 * t - 1.0;
    saw -= poly_blep(t, dtAbs);

    // PWM Square (polyBLEP at both edges); TIMBRE = duty in this region
    const double w = fast_clip(timbre01 * 0.90 + 0.05, 0.05, 0.95);
    double sq = (t < w ? 1.0 : -1.0);
    sq += poly_blep(t, dtAbs);
    double tw = t - w; if (tw < 0.0) tw += 1.0;
    sq -= poly_blep(tw, dtAbs);
    outSquareForTri = sq;

    // 50% duty BL square for triangle integration (amplitude-stable)
    double sq50 = (t < 0.5 ? 1.0 : -1.0);
    sq50 += poly_blep(t, dtAbs);
    double t2 = t - 0.5; if (t2 < 0.0) t2 += 1.0;
    sq50 -= poly_blep(t2, dtAbs);

    // Triangle by exact integration; works with signed dt (TZFM)
    triState += (2.0 * dt) * sq50;
    if (triState > 1.2) triState = 1.2; else if (triState < -1.2) triState = -1.2;
    const double tri = triState;

    // Sine and a phase-distorted sine driven by TIMBRE
    const double theta = 2.0 * M_PI * t;
    const double sine  = std::sin(theta);
    double ph = t + 0.25 * timbre01 * std::sin(theta); // warp phase; 0..1 wrap
    ph -= std::floor(ph);
    const double sinePD = std::sin(2.0 * M_PI * ph);

    // Triangle curvature (TIMBRE increases odd-harm content gently)
    const double triShaped = tri * (1.0 - timbre01) + std::tanh(tri * (1.0 + 3.0 * timbre01)) * timbre01;

    // Folded/chaotic core (depth from TIMBRE)
#if 1
    const double folded = std::tanh( (1.0 + 4.0 * timbre01) * sine );
#else
    const double folded = softclip3((1.0 + 4.0 * timbre01) * sine);
#endif

    // ---------- Morph zones ----------
    double y = 0.0;
    switch (zone) {
        default:
        case 0: // Sine -> Triangle  (timbre = PD on sine)
            y = morphMix(sinePD, tri, zf);
            break;

        case 1: // Triangle -> Saw   (timbre = curvature on triangle)
            y = morphMix(triShaped, saw, zf);
            break;

        case 2: // Saw -> PWM Square (timbre = PWM duty already applied)
            y = morphMix(saw, sq, zf);
            break;

        case 3: // PWM Square -> Folded/Chaotic (timbre = fold depth)
            y = morphMix(sq, folded, zf);
            break;
    }
    return y;
}

void VoxVcoCore::processBlock(const float* inL, const float* inR,
                              float* outL, float* outR, int n)
{
    if (!outL || !outR || n <= 0) return;

    // Detect FM cable if wrapper doesn't provide it: measure RMS over the block
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
    double panPos[kMaxUnison] = {0};
    double detC[kMaxUnison] = {0};

    if (unisonCount_ == 1) {
        panPos[0] = 0.5;
        detC[0] = 0.0;
    } else {
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
        // Sync detection (shared left input):
        // - Hard: rising past ~+2 V
        // - Soft: rising zero-crossing (bipolar signals)
        bool hardSync = false, softSync = false;
        if (inL) {
            const float s = inL[i];
            hardSync = (prevSync <= 2.0f && s > 2.0f);
            softSync = (prevSync <= 0.0f && s > 0.0f);
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
                const double absHz = kFmMaxHz * fmIndex_;
                const double relHz = baseHz * (1.5 * fmIndex_);   // up to ~1.5× base at full
                const double devHz = absHz * 0.7 + relHz * 0.3;
                f += devHz * norm;
            }
            f = fast_clip(f, -kMaxHz, kMaxHz); // allow negative (through-zero)

            const double dt = f * invSr_;

            // Hard/soft sync handling
            if (hardSync) {
                V.phase = 0.0;   // hard reset
                V.triI *= 0.5;   // tame integrator click
            } else if (softSync) {
                // gentle pull of phase toward 0 without discontinuity
                const double k = 0.20;
                V.phase -= k * V.phase;
                if (V.phase < 0.0) V.phase += 1.0;
                else if (V.phase >= 1.0) V.phase -= 1.0;
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
            V.phase -= std::floor(V.phase); // wrap to [0,1)
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

    // store prevSync across blocks
    prevSync_ = prevSync;
}

} // namespace vm
