#include "VoxVcoCore.hpp"
#include <cstring>
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
        const double r1 = voices_[i].rng.bipolar();
        const double r2 = voices_[i].rng.bipolar();
        voices_[i].pwmBias   = kAnalogModel ? (kAnalogPwmBiasStd * r1) : 0.0;
        voices_[i].shapeSkew = kAnalogModel ? (kAnalogShapeStd * r2) : 0.0;
    }

    aJitMF_ = std::exp(-2.0 * M_PI * 300.0 / sr_);
    aJitLF_ = std::exp(-2.0 * M_PI * 5.0   / sr_);

    const double tWidth  = std::max(0.1, kSpreadSlewMs) * 0.001;
    const double tDetune = std::max(0.1, kDetuneSlewMs) * 0.001;
    aWidth_  = std::exp(-1.0 / (sr_ * tWidth));
    aDetune_ = std::exp(-1.0 / (sr_ * tDetune));

#if defined(TARGET_DAISY)
    limiter_.setup(sr_, 0.75, 0.10, 80.0, 1.0);
    outLpL_.setup(sr_, 18000.0);
    outLpR_.setup(sr_, 18000.0);
#else
    limiter_.setup(sr_, 0.75, 0.10, 80.0, 5.0);
    outLpL_.setup(sr_, kOutputLP_Hz);
    outLpR_.setup(sr_, kOutputLP_Hz);
#endif
    slewL_.setup(sr_, kAnalogModel ? kAnalogSlewVPerMs : 0.0);
    slewR_.setup(sr_, kAnalogModel ? kAnalogSlewVPerMs : 0.0);

    reset();
}

void VoxVcoCore::reset() {
    prevSync_ = 0.0f;
    for (auto& v : voices_) {
        v.phase = 0.0;
        v.triI = 0.0;
        v.parabPrev = 0.0;
        v.drift = 0.0;
        v.jitMF = 0.0;
        v.jitLF = 0.0;
    }
    stereoWidthZ_ = 0.0;
    detuneSpreadCentsZ_ = 0.0;
    humPhi50_ = 0.0;
    humPhi60_ = 0.0;
}

// Params mapping (unchanged)
void VoxVcoCore::setParams(const IDspCore::Params& p) {
    P_.pitchVolts = p.pitchVolts;

    double m[4] = { p.macros[0], p.macros[1], p.macros[2], p.macros[3] };
    const bool macrosProvided = (m[0] != 0.0 || m[1] != 0.0 || m[2] != 0.0 || m[3] != 0.0);
    if (!macrosProvided) {
        m[0] = p.pitch;     // Macro A
        m[1] = p.timbre;    // Macro B (UI 0..2)
        m[2] = p.morph;     // Macro C
        m[3] = p.spread;    // Macro D
    }

    const double mA      = fast_clip(m[0], 0.0, 1.0);
    const double mB_raw  = fast_clip(m[1], 0.0, 2.0);
    const double mB_norm = mB_raw * 0.5; // 0..1
    const double mC      = fast_clip(m[2], 0.0, 1.0);
    const double mD      = fast_clip(m[3], 0.0, 1.0);

    P_.macroA = mA * 2.0 - 1.0;   // ±12 semis
    P_.macroB = mB_norm;          // morph 0..1
    P_.macroC = mC;               // spread 0..1
    P_.macroD = mD;               // timbre/FM 0..1

    updateUnisonFromMacro(P_.macroC);
    P_.fmCable = p.fmCablePresent;
    fmEnabled_ = P_.fmCable;

    pwmWidth_ = 0.05 + 0.90 * P_.macroD;
    foldAmt_  = P_.macroD;
    chaosAmt_ = P_.macroD;
    fmIndex_  = std::sqrt(fast_clip(P_.macroD, 0.0, 1.0));

    stereoWidth_       = P_.macroC;          // targets
    detuneSpreadCents_ = 30.0 * P_.macroC;
}

void VoxVcoCore::updateUnisonFromMacro(double mc) {
    if (mc < 0.125)      unisonCount_ = 1;
    else if (mc < 0.375) unisonCount_ = 3;
    else if (mc < 0.625) unisonCount_ = 5;
    else                 unisonCount_ = 7;
}

// Per-voice activations (center 1.0; pairs fade in smoothly)
void VoxVcoCore::computeVoiceActivations_(double a, double act[7]) const {
    const double t1 = 0.125, t2 = 0.375, t3 = 0.625;
    const double bw = kPairXfadeBW;

    auto ramp = [&](double x, double t) {
        const double lo = t - bw, hi = t + bw;
        return sstep((x - lo) / std::max(1e-9, (hi - lo)));
    };

    const double w1 = ramp(a, t1);
    const double w2 = ramp(a, t2);
    const double w3 = ramp(a, t3);

    act[0] = w3; act[1] = w2; act[2] = w1; act[3] = 1.0; act[4] = w1; act[5] = w2; act[6] = w3;
}

void VoxVcoCore::advanceDrift(Voice& v, double depthCents) {
    const double rate = 0.3; // Hz
    const double a = std::exp(-2.0 * M_PI * rate * invSr_);
    const double noise = v.rng.bipolar();
    const double target = noise * (depthCents / 2.0);
    v.drift = a * v.drift + (1.0 - a) * target;
}

// Peak-normalized equal-power crossfade
static inline double morphMix(double a, double b, double x) {
    const double t  = fast_clip(x, 0.0, 1.0);
    const double wa = std::cos(0.5 * M_PI * t);
    const double wb = std::sin(0.5 * M_PI * t);
    double y = a * wa + b * wb;
    const double norm = wa + wb;
    return (norm > 1e-9) ? (y / norm) : y;
}

// Cheap reflect-based wavefolder
static inline double fold_reflect(double x) {
    double u = x + 1.0;
    u -= 2.0 * std::floor(u * 0.5);
    if (u > 1.0) u = 2.0 - u;
    return u - 1.0;
}

double VoxVcoCore::renderCore(double phase, double dt, double morph01,
                              double timbre01, double& triState,
                              double& outSquareForTri,
                              double shapeSkew, double pwmBias) const
{
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

    // PWM square (BLEP both edges); TIMBRE sets duty with per-voice + HF bias
    double w = fast_clip(timbre01 * 0.90 + 0.05 + pwmBias, 0.05, 0.95);
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

    // Triangle (correct slope 4*dt), with tiny even-harmonic skew
    triState += (4.0 * dt) * sq50;
    if (triState > 1.2) triState = 1.2; else if (triState < -1.2) triState = -1.2;
    double tri = triState + shapeSkew * (triState * triState - 1.0 / 3.0);

    // --- Sine family (phase-align to triangle max at t=0.5) ---
    double tSin = t - 0.25; if (tSin < 0.0) tSin += 1.0;
    const double theta = 2.0 * M_PI * tSin;
    const double sine  = std::sin(theta);

    // PD-sine with a hair of skew
    double ph = tSin + 0.25 * timbre01 * std::sin(theta) + 0.05 * shapeSkew * std::sin(3.0 * theta);
    ph -= std::floor(ph);
    const double sinePD = std::sin(2.0 * M_PI * ph);

    // Aggressive folding near morph ends
    const double edge = std::pow(std::min(1.0, std::abs(m - 0.5) * 2.0), 0.8);
    double drive = 1.0 + (2.0 + 6.0 * edge) * (timbre01 * timbre01);
    if (drive > 9.0) drive = 9.0;
    const double sineFoldHard = fold_reflect(sine * drive);
    const double sineAgg = morphMix(sinePD, sineFoldHard, timbre01);

    // --- Zone-specific timbre shapes (normalized) ---
    const double gTri = 1.0 + 3.0 * timbre01;
    const double invTanh_gTri = 1.0 / std::tanh(gTri);
    const double triNL = std::tanh(gTri * tri) * invTanh_gTri;
    const double triShaped = tri * (1.0 - timbre01) + triNL * timbre01;

    const double gFold = 1.0 + 4.0 * timbre01;
    const double invTanh_gFold = 1.0 / std::tanh(gFold);
    const double foldedSoft = std::tanh(gFold * sine) * invTanh_gFold;
    const double foldedAgg  = morphMix(foldedSoft, sineFoldHard, timbre01);

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
        case 0: y = mixPeakNorm(sineAgg,    tri,    zf); break;
        case 1: y = mixPeakNorm(triShaped,  saw,    zf); break;
        case 2: y = mixPeakNorm(saw,        sq,     zf); break;
        case 3: y = mixPeakNorm(sq,         foldedAgg, zf); break;
    }

    // Light mid-comp to keep peaks flat across the path
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

    // Detect FM cable if wrapper doesn't provide it
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

    // Smoothed width & detune targets
    stereoWidthZ_       = aWidth_  * stereoWidthZ_       + (1.0 - aWidth_)  * stereoWidth_;
    detuneSpreadCentsZ_ = aDetune_ * detuneSpreadCentsZ_ + (1.0 - aDetune_) * detuneSpreadCents_;

    // Activations
    double act[kMaxUnison];
    computeVoiceActivations_(stereoWidthZ_, act);

    // Pans
    double gL[kMaxUnison] = {0.0}, gR[kMaxUnison] = {0.0};
    double sumGL = 0.0, sumGR = 0.0;
    for (int v = 0; v < kMaxUnison; ++v) {
        const double basePos = (double(v) - 3.0) / 3.0; // -1..+1
        const double pan     = basePos * stereoWidthZ_;
        const double pan01   = 0.5 * (pan + 1.0);
        gL[v] = equalPowerL(pan01);
        gR[v] = equalPowerR(pan01);
        sumGL += gL[v] * act[v];
        sumGR += gR[v] * act[v];
    }
    const double invSumGL = (std::isfinite(sumGL) && sumGL > 1e-6) ? (1.0 / sumGL) : 1.0;
    const double invSumGR = (std::isfinite(sumGR) && sumGR > 1e-6) ? (1.0 / sumGR) : 1.0;

    // Detunes
    double detC[kMaxUnison] = {0.0};
    for (int v = 0; v < kMaxUnison; ++v) {
        const double basePos = (double(v) - 3.0) / 3.0;
        const double sgn = (basePos < 0.0) ? -1.0 : (basePos > 0.0 ? 1.0 : 0.0);
        const double mag = std::abs(basePos);
        detC[v] = sgn * mag * detuneSpreadCentsZ_;
    }

    // Jitter scales
    const double jitRel = (kAnalogModel ? (kAnalogJitterRelPpm * 1e-6) : 0.0);
    const double jitAbs = (kAnalogModel ? kAnalogJitterAbsHz : 0.0);

    // Hum phase increments
    const double dPhi50 = 2.0 * M_PI * 50.0 * invSr_;
    const double dPhi60 = 2.0 * M_PI * 60.0 * invSr_;

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

        double fmSample = 0.0;
        if (fmEnabled_ && inR) {
            fmSample = fmHp_.process(double(inR[i]));
            if (!std::isfinite(fmSample)) fmSample = 0.0;
        }

        double L = 0.0, R = 0.0;

        // 7-slot engine
        for (int v = 0; v < kMaxUnison; ++v) {
            auto& V = voices_[v];

            // Slow drift depth
            advanceDrift(V, 3.0 + 4.0 * stereoWidthZ_);
            const double cents = detC[v] + V.drift;
            const double ratio = detuneCentsToRatio(cents);

            // Base + jitter
            double f = baseHz * ratio;
            if (kAnalogModel && (jitRel != 0.0 || jitAbs != 0.0)) {
                V.jitMF = aJitMF_ * V.jitMF + (1.0 - aJitMF_) * V.rng.bipolar();
                V.jitLF = aJitLF_ * V.jitLF + (1.0 - aJitLF_) * V.rng.bipolar();
                const double j = 0.65 * V.jitMF + 0.35 * V.jitLF;
                f += f * (jitRel * j) + (jitAbs * j);
            }

            // External FM
            if (fmEnabled_) {
                const double norm = fast_clip(fmSample / 5.0, -1.0, 1.0);
                const double absHz = kFmMaxHz * fmIndex_;
                const double relHz = baseHz * (1.5 * fmIndex_);
                const double devHz = absHz * 0.7 + relHz * 0.3;
                f += devHz * norm;
            }

            f = fast_clip(f, -kMaxHz, kMaxHz);
            const double dt = f * invSr_;

            // Sync
            if (hardSync) {
                V.phase = 0.0;
                V.triI *= 0.5;
            } else if (softSync) {
                const double k = 0.20;
                V.phase -= k * V.phase;
                if (V.phase < 0.0) V.phase += 1.0;
                else if (V.phase >= 1.0) V.phase -= 1.0;
            }

            // Cycle-edge jitter
            double newPhase = V.phase + dt;
            if (kAnalogModel) {
                const bool wrapped = (newPhase >= 1.0) || (newPhase < 0.0);
                if (wrapped) {
                    const double delay_s = (kAnalogEdgeJitterUs * 1e-6) * V.rng.bipolar();
                    if (std::isfinite(delay_s)) newPhase += f * delay_s;
                }
            }

            // Render (per-voice skew + PWM HF bias)
            double sqForTri = 0.0;
            const double pwmBiasHF = kAnalogModel ? (kAnalogPwmBiasHF * (std::min(std::abs(f), 20000.0) / 20000.0)) : 0.0;
            double y = renderCore(V.phase, dt,
                                  P_.macroB, P_.macroD,
                                  V.triI, sqForTri,
                                  V.shapeSkew, V.pwmBias + pwmBiasHF);
            if (!std::isfinite(y)) y = 0.0;

            // Pan & mix
            const double a = act[v];
            L += y * gL[v] * a;
            R += y * gR[v] * a;

            // Advance phase
            V.phase = newPhase - std::floor(newPhase);
        }

        // Normalize and scale
        double yL = L * invSumGL;
        double yR = R * invSumGR;
        if (!std::isfinite(yL)) yL = 0.0;
        if (!std::isfinite(yR)) yR = 0.0;

        double preL = yL * kOutGain;
        double preR = yR * kOutGain;

        // Soft sat
        if (kAnalogModel && kAnalogSoftSatMix > 0.0) {
            const double m = kAnalogSoftSatMix;
            auto sat = [&](double v){
                double x = fast_clip(v / kOutGain, -1.5, 1.5);
                const double d = 1.0 + 1.5 * m;
                double sh = std::tanh(d * x) / std::tanh(d);
                return kOutGain * ( (1.0 - m) * x + m * sh );
            };
            preL = sat(preL);
            preR = sat(preR);
        }

        // Slew
        if (kAnalogModel && kAnalogSlewVPerMs > 0.0) {
            preL = slewL_.process(preL);
            preR = slewR_.process(preR);
        }

        // Output LP + hum/noise
        if (kAnalogModel) {
            preL = outLpL_.process(preL);
            preR = outLpR_.process(preR);

            const double hum50 = std::sin(humPhi50_);
            const double hum60 = std::sin(humPhi60_);
            const double hum   = kHum50_Level * hum50
                               + kHum2H_Level * std::sin(2.0 * humPhi50_)
                               + kHum60_Level * hum60
                               + kHum2H_Level * std::sin(2.0 * humPhi60_);

            humPhi50_ += dPhi50; if (humPhi50_ > 2.0 * M_PI) humPhi50_ -= 2.0 * M_PI;
            humPhi60_ += dPhi60; if (humPhi60_ > 2.0 * M_PI) humPhi60_ -= 2.0 * M_PI;

            preL += hum * kOutGain;
            preR += hum * kOutGain;

            const double nL = (rng_.bipolar()) * kNoiseRms_V;
            const double nR = (rng_.bipolar()) * kNoiseRms_V;
            if (std::isfinite(nL)) preL += nL;
            if (std::isfinite(nR)) preR += nR;
        }

        // Safe look-ahead limiter to ~±5 V
        limiter_.process(preL, preR, outL[i], outR[i]);
        if (!std::isfinite(outL[i])) outL[i] = 0.0f;
        if (!std::isfinite(outR[i])) outR[i] = 0.0f;
    }

    prevSync_ = prevSync;
}

} // namespace vm
