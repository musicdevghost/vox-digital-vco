#pragma once
#include "plugin.hpp"
#include "TemplateIO.hpp"
#include <algorithm>
#include <cmath>

using rack::clamp;

template <typename CoreT>
struct VoxTemplateModule : rack::engine::Module {
    using ParamsT = typename CoreT::Params;

    VoxTemplateModule() {
        config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

        // Params
        configParam(PARAM_PITCH, 0.f, 1.f, 0.5f, "Pitch");
        configParam(PARAM_MORPH,   0.f, 2.f, 1.0f, "Morph");
        configParam(PARAM_SPREAD,   0.f, 1.f, 0.5f, "Spread");
        configParam(PARAM_TIMBRE,  0.f, 1.f, 0.0f, "Timbre");

        configParam(PARAM_ATT_PITCH, -1.f, 1.f, 1.f, "Pitch CV amount");
        configParam(PARAM_ATT_MORPH,   -1.f, 1.f, 0.f, "Morph CV amount");
        configParam(PARAM_ATT_SPREAD,   -1.f, 1.f, 0.f, "Spread CV amount");
        configParam(PARAM_ATT_TIMBRE,  -1.f, 1.f, 0.f, "Timbre CV amount");

        // Jacks labels (optional)
        configInput(INPUT_CV_PITCH, "CV Pitch");
        configInput(INPUT_CV_MORPH,   "CV Morph");
        configInput(INPUT_CV_SPREAD,   "CV Spread");
        configInput(INPUT_CV_TIMBRE,  "CV Timbre");
        configInput(INPUT_SOFT_SYNC,     "Soft Sync");
        configInput(INPUT_HARD_SYNC,      "Hard Sync");
        configInput(INPUT_FM_LINEAR,      "FM Linear");
        configOutput(OUTPUT_AUX,     "AUX");
        configOutput(OUTPUT_OUT_L,   "Out L");
        configOutput(OUTPUT_OUT_R,   "Out R");
    }

    void onSampleRateChange() override {
        sr_ = APP->engine->getSampleRate() > 0 ? APP->engine->getSampleRate() : 48000.0;
        core_.init(sr_);
        core_.reset();
        ctrlBlock_ = std::max(8, (int)std::lround(sr_ / 1000.0));
        ctrlCountdown_ = 0;
    }

    void process(const ProcessArgs& args) override {
        if (!prepared_) { onSampleRateChange(); prepared_ = true; }

        // --- Control block (@~1kHz) ---
        if (ctrlCountdown_-- <= 0) {
            pNext_ = readParams();
            pDelta_.dryWet = (pNext_.dryWet - pCur_.dryWet) / (float)ctrlBlock_;
            pDelta_.gain   = (pNext_.gain   - pCur_.gain  ) / (float)ctrlBlock_;
            pDelta_.tone   = (pNext_.tone   - pCur_.tone  ) / (float)ctrlBlock_;
            pDelta_.macro  = (pNext_.macro  - pCur_.macro ) / (float)ctrlBlock_;
            ctrlCountdown_ = ctrlBlock_;
        }

        // per-sample ramp and set to core
        pCur_.dryWet += pDelta_.dryWet;
        pCur_.gain   += pDelta_.gain;
        pCur_.tone   += pDelta_.tone;
        pCur_.macro  += pDelta_.macro;

        if (inputs[INPUT_CV_PITCH].isConnected()) {
            const float att = clamp(params[PARAM_ATT_PITCH].getValue(), -1.f, 1.f);
            pCur_.pitchVolts = inputs[INPUT_CV_PITCH].getVoltage() * att;  // true 1V/oct
        } else {
            pCur_.pitchVolts = 0.0;  // C4 base when no cable
        }

        core_.setParams(pCur_);

        // --- Audio IO ---
        // Combine hard-sync (bottom-left) and soft-sync (mid jack) onto left input
        float inL = 0.f;
        if (inputs[INPUT_HARD_SYNC].isConnected())  inL += inputs[INPUT_HARD_SYNC].getVoltage();
        if (inputs[INPUT_SOFT_SYNC].isConnected())  inL += inputs[INPUT_SOFT_SYNC].getVoltage();

        // FM (right input)
        float inR = inputs[INPUT_FM_LINEAR].getVoltage();

        // Let the core know if FM is patched (helps it skip inference)
        pCur_.fmCablePresent = inputs[INPUT_FM_LINEAR].isConnected();

        float outL = 0.f, outR = 0.f;
        core_.processBlock(&inL, &inR, &outL, &outR, 1);

        outputs[OUTPUT_OUT_L].setVoltage(outL);
        outputs[OUTPUT_OUT_R].setVoltage(outR);

        // Envelope follower (0..8 V)
        float amp = std::max(std::fabs(outL), std::fabs(outR)) / 5.f;
        const float atkTc = 0.008f, relTc = 0.160f;
        float atk = 1.f - std::exp(-1.f / (atkTc * (float)sr_));
        float rel = 1.f - std::exp(-1.f / (relTc * (float)sr_));
        float a   = (amp > pepperEnv_) ? atk : rel;
        pepperEnv_ += a * (amp - pepperEnv_);

        float envV = clamp(pepperEnv_ * 8.f, 0.f, 8.f);
        outputs[OUTPUT_AUX].setVoltage(envV);
        lights[LIGHT_PEPPER].setSmoothBrightness(std::pow(pepperEnv_, 0.6f), args.sampleTime);
    }

private:
    double sr_ = 48000.0;
    bool prepared_ = false;

    CoreT core_{};
    int ctrlBlock_ = 48;
    int ctrlCountdown_ = 0;

    ParamsT pCur_{0.5f,1.0f,0.5f,0.0f};
    ParamsT pNext_{0.5f,1.0f,0.5f,0.0f};
    ParamsT pDelta_{};

    float pepperEnv_ = 0.f;

    ParamsT readParams() {
        auto get = [&](int pid) { return params[pid].getValue(); };
        auto sampleCv = [&](int inId) {
            if (!inputs[inId].isConnected()) return 0.f;
            return clamp(inputs[inId].getVoltage() / 10.f, -1.f, 1.f);
        };

        ParamsT p{};

        // --- TRUE 1 V/oct path ---
        // pitchVolts is the raw 1 V/oct CV (scaled by attenuverter).
        // 0 V -> C4 (261.626 Hz); +1 V = +1 octave.
        if (inputs[INPUT_CV_PITCH].isConnected()) {
            const float att = clamp(get(PARAM_ATT_PITCH), -1.f, 1.f);
            p.pitchVolts = inputs[INPUT_CV_PITCH].getVoltage() * att;
        } else {
            p.pitchVolts = 0.0; // C4 when no pitch CV
        }

        // --- Macros ---
        // Macro A (PITCH knob) is *knob only* and becomes the ±12-semitone offset in the core.
        p.dryWet = clamp(get(PARAM_PITCH), 0.f, 1.f); // Macro A (no CV added)

        // Macro B/C/D keep their CV paths via attenuverters (unchanged behavior)
        // Before:
        // p.gain   = clamp(get(PARAM_MORPH) + get(PARAM_ATT_MORPH) * sampleCv(INPUT_CV_MORPH),   0.f, 2.f);

        // After: scale CV by 2× so ±5 V => ±1.0 -> full 0..2 sweep
        p.gain   = clamp(get(PARAM_MORPH) + 2.f * get(PARAM_ATT_MORPH) * sampleCv(INPUT_CV_MORPH), 0.f, 2.f);
        p.tone   = clamp(get(PARAM_SPREAD) + get(PARAM_ATT_SPREAD) * sampleCv(INPUT_CV_SPREAD), 0.f, 1.f);
        p.macro  = clamp(get(PARAM_TIMBRE) + get(PARAM_ATT_TIMBRE) * sampleCv(INPUT_CV_TIMBRE), 0.f, 1.f);

        return p;
    }
};
