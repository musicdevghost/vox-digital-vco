#include "VoxVcoCore.hpp"
#include <cstring> // memset
#include <algorithm>
#include <cmath>

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
// - Macro B: morph (UI 0..2) -> normalized to [0..1]
// - Macro C: unison/spread [0..1]
// - Macro D: timbre / FM index [0..1] (sqrt curve for hotter top)
void VoxVcoCore::setParams(const IDspCore::Params& p) {
    // ---- Pitch (V/Oct) ----
    P_.pitchVolts = p.pitchVolts;

    // Prefer explicit macros[0..3]; if they look defaulted, fall back to dryWet/gain/tone/macro
    double m[4] = { p.macros[0], p.macros[1], p.macros[2], p.macros[3] };
    const bool macrosProvided = (m[0] != 0.0 || m[1] != 0.0 || m[2] != 0.0 || m[3] != 0.0);
    if (!macrosProvided) {
        m[0] = p.dryWet; // Macro A
        m[1] = p.gain;   // Macro B (note: UI uses 0..2)
        m[2] = p.tone;   // Macro C
        m[3] = p.macro;  // Macro D
    }

    // Map & clamp
    const double mA      = fast_clip(m[0], 0.0, 1.0);
    const double mB_raw  = fast_clip(m[1], 0.0, 2.0); // accept UI's 0..2
    const double mB_norm = mB_raw * 0.5;              // normalize to 0..1 for the core
    const double mC      = fast_clip(m[2], 0.0, 1.0);
    const double mD      = fast_clip(m[3], 0.0, 1.0);

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

    // IMPORTANT: width and detune scale still follow Macro C (previous behavior)
    stereoWidth_       = P_.macroC;        // 0..1
    detuneSpreadCents_ = 30.0 * P_.macroC;
}

void VoxVcoCore::updateUnisonFromMacro(double mc) {
    if (mc < 0.125)      unisonCount_ = 1;
    else if (mc < 0.375) unisonCount_ = 3;
    else if (mc < 0.625) unisonCount_ = 5;
    else                 unisonCount_ = 7;
}

void VoxVcoCore::advanceDrift(Voice& v, double depthCents) {
    // Very slow random walk ~0.3 Hz pole
    const double rate = 0.3; // Hz
    const double a = std::exp(-2.0 * M_PI * rate * invSr_);
    const double noise = v.rng.bipolar(); // [-1, 1]
    const double target = noise * (depthCents / 2.0);
    v.drift = a * v.drift + (1.0 - a) * target;
}

// Peak-normalized equal-power crossfade: smooth AND keeps peak near 1.0
static inline double morphMix(double a, double b, double x) {
    const double t  = fast_clip(x, 0.0, 1.0);
    const double wa = std::cos(0.5 * M_PI * t);
    const double wb = std::sin(0.5 * M_PI * t);
    double y = a * wa + b * wb;
    const double norm = wa + wb;
    return (norm > 1e-9) ? (y / norm) : y;
}

// NEW: cheap reflect-based wavefolder (branch-light, no loops).
// Folds an input of arbitrary amplitude back into [-1, 1].
static inline double fold_reflect(double x) {
    // Map to [0,2)
    double u = x + 1.0;
    u -= 2.0 * std::floor(u * 0.5);
    // Reflect second half
    if (u > 1.0) u = 2.0 - u;
    return u - 1.0; // back to [-1,1]
}

double VoxVcoCore::renderCore(double phase, double dt, double morph01,
                              double timbre01, double& triState,
                              double& outSquareForTri) const
{
    // Robust mapping of morph to 4 zones with continuous edge at 1.0
    const double m = fast_clip(morph01, 0.0, 1.0);
    double whole = 0.0;
    double zf = std::modf(m * 4.0, &whole);   // [0,1)
    int zone = (int)whole;
    if (zone >= 4) { zone = 3; zf = 1.0; }

    double t = phase;                  // [0,1)
    const double dtAbs = std::fabs(dt);

    // ---------- Primitives ----------
    // Saw (polyBLEP)
    double saw = 2.0 * t - 1.0;
    saw -= poly_blep(t, dtAbs);

    // PWM square (BLEP both edges); TIMBRE is duty here
    const double w = fast_clip(timbre01 * 0.90 + 0.05, 0.05, 0.95);
    double sq = (t < w ? 1.0 : -1.0);
    sq += poly_blep(t, dtAbs);
    double tw = t - w; if (tw < 0.0) tw += 1.0;
    sq -= poly_blep(tw, dtAbs);
    outSquareForTri = sq;

    // 50% BLEP square for triangle integration
    double sq50 = (t < 0.5 ? 1.0 : -1.0);
    sq50 += poly_blep(t, dtAbs);
    double t2 = t - 0.5; if (t2 < 0.0) t2 += 1.0;
    sq50 -= poly_blep(t2, dtAbs);

    // Triangle (correct slope 4*dt); peaks at t ≈ 0.5
    triState += (4.0 * dt) * sq50;
    if (triState > 1.2) triState = 1.2; else if (triState < -1.2) triState = -1.2;
    const double tri = triState;

    // --- Sine family (phase-align to triangle max at t=0.5) ---
    double tSin = t - 0.25; if (tSin < 0.0) tSin += 1.0;
    const double theta = 2.0 * M_PI * tSin;
    const double sine  = std::sin(theta);     // ±1 peak

    // PD-sine (gentle) — original path
    double ph = tSin + 0.25 * timbre01 * std::sin(theta);
    ph -= std::floor(ph);
    const double sinePD = std::sin(2.0 * M_PI * ph);

    // ------ NEW aggressive sine fold when MORPH near ends ------
    // Edge emphasis grows near morph extremes (0 or 1), 0 at center (0.5).
    const double edge = std::pow(std::min(1.0, std::abs(m - 0.5) * 2.0), 0.8);
    // Drive rises with TIMBRE^2, boosted near edges; clamped for stability.
    double drive = 1.0 + (2.0 + 6.0 * edge) * (timbre01 * timbre01);
    if (drive > 9.0) drive = 9.0;

    // Harder fold of sine; bounded in [-1,1].
    const double sineFoldHard = fold_reflect(sine * drive);

    // Blend PD-sine → hard-fold with TIMBRE (keeps low-TIMBRE gentle, high-TIMBRE aggressive).
    const double sineAgg = morphMix(sinePD, sineFoldHard, timbre01);

    // --- Zone-specific timbre shapes (kept normalized) ---

    // Triangle curvature: tanh(g*tri)/tanh(g) keeps peak = 1 for any g
    const double gTri = 1.0 + 3.0 * timbre01;
    const double invTanh_gTri = 1.0 / std::tanh(gTri);
    const double triNL = std::tanh(gTri * tri) * invTanh_gTri;
    const double triShaped = tri * (1.0 - timbre01) + triNL * timbre01;

    // Soft "fold" from sine via tanh (original), kept as the soft baseline
    const double gFold = 1.0 + 4.0 * timbre01;
    const double invTanh_gFold = 1.0 / std::tanh(gFold);
    const double foldedSoft = std::tanh(gFold * sine) * invTanh_gFold;

    // NEW: soft→hard fold crossfade for Zone 3's "folded" endpoint
    const double foldedAgg = morphMix(foldedSoft, sineFoldHard, timbre01);

    // ---------- Smooth crossfades ----------
    auto mixPeakNorm = [](double a, double b, double x) {
        const double u  = fast_clip(x, 0.0, 1.0);
        const double wa = std::cos(0.5 * M_PI * u);
        const double wb = std::sin(0.5 * M_PI * u);
        double y = a * wa + b * wb;
        const double norm = wa + wb;
        return (norm > 1e-9) ? (y / norm) : y;
    };

    double y = 0.0;
    switch (zone) {
        default:
        case 0: y = mixPeakNorm(sineAgg,    tri,    zf); break; // sine side is now PD→hard-fold by TIMBRE
        case 1: y = mixPeakNorm(triShaped,  saw,    zf); break;
        case 2: y = mixPeakNorm(saw,        sq,     zf); break;
        case 3: y = mixPeakNorm(sq,         foldedAgg, zf); break; // folded side becomes soft→hard by TIMBRE
    }

    // Light mid-comp remains to keep peaks flat across the path
    auto midComp = [](double z, double k){ return 1.0 + k * z * (1.0 - z); };
    if (zone == 0) y *= midComp(zf, 0.03);
    if (zone == 1) y *= midComp(zf, 0.02);
    if (zone == 2) y *= midComp(zf, 0.015);

    return y; // ~±1 peak across the morph
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
    double gL[kMaxUnison] = {0};
    double gR[kMaxUnison] = {0};
    double sumGL = 0.0, sumGR = 0.0;

    if (unisonCount_ == 1) {
        panPos[0] = 0.5;
    } else {
        for (int i = 0; i < unisonCount_; ++i) {
            const double idx01 = (unisonCount_ <= 1) ? 0.5 : double(i) / double(unisonCount_ - 1);
            panPos[i] = 0.5 + (idx01 - 0.5) * stereoWidth_; // spread around center
            if (panPos[i] < 0.0) panPos[i] = 0.0;
            if (panPos[i] > 1.0) panPos[i] = 1.0;
        }
    }
    for (int i = 0; i < unisonCount_; ++i) {
        gL[i] = equalPowerL(panPos[i]);
        gR[i] = equalPowerR(panPos[i]);
        sumGL += gL[i];
        sumGR += gR[i];
    }

    const double normFloor = 0.25; // ~-12 dB floor
    const double invSumGL = (sumGL > normFloor) ? (1.0 / sumGL) : 1.0;
    const double invSumGR = (sumGR > normFloor) ? (1.0 / sumGR) : 1.0;

    // Build per-voice detune (symmetric) once per block
    double detC[kMaxUnison] = {0};
    if (unisonCount_ > 1) {
        for (int i = 0; i < unisonCount_; ++i) {
            const double sgn = (i % 2 == 0) ? -1.0 : +1.0;
            const double mag = (double(i) / double(unisonCount_ - 1));
            detC[i] = sgn * mag * detuneSpreadCents_;
        }
    }

    // Process
    float prevSync = prevSync_;
    prevSync_ = (n > 0 && inL) ? inL[n - 1] : prevSync_;

    for (int i = 0; i < n; ++i) {
        // Sync detection
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
                const double absHz = kFmMaxHz * fmIndex_;
                const double relHz = baseHz * (1.5 * fmIndex_);
                const double devHz = absHz * 0.7 + relHz * 0.3;
                f += devHz * norm;
            }
            f = fast_clip(f, -kMaxHz, kMaxHz); // allow negative (through-zero)

            const double dt = f * invSr_;

            // Hard/soft sync handling
            if (hardSync) {
                V.phase = 0.0;
                V.triI *= 0.5;
            } else if (softSync) {
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
            L += y * gL[v];
            R += y * gR[v];

            // Advance phase (through-zero OK)
            V.phase += dt;
            V.phase -= std::floor(V.phase);
        }

        // Normalize by channel pan gains and scale to volts
        double yL = fast_clip(L * invSumGL, -1.0, 1.0);
        double yR = fast_clip(R * invSumGR, -1.0, 1.0);

        outL[i] = float(yL * kOutGain); // ~±5 V per channel
        outR[i] = float(yR * kOutGain);
    }

    // store prevSync across blocks
    prevSync_ = prevSync;
}

} // namespace vm
