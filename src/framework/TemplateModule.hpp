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
        configParam(PARAM_DRYWET, 0.f, 1.f, 0.5f, "Dry/Wet");
        configParam(PARAM_GAIN,   0.f, 2.f, 1.0f, "Gain");
        configParam(PARAM_TONE,   0.f, 1.f, 0.5f, "Tone");
        configParam(PARAM_MACRO,  0.f, 1.f, 0.0f, "Macro");

        configParam(PARAM_ATT_DRYWET, -1.f, 1.f, 0.f, "Dry/Wet CV amount");
        configParam(PARAM_ATT_GAIN,   -1.f, 1.f, 0.f, "Gain CV amount");
        configParam(PARAM_ATT_TONE,   -1.f, 1.f, 0.f, "Tone CV amount");
        configParam(PARAM_ATT_MACRO,  -1.f, 1.f, 0.f, "Macro CV amount");

        // Jacks labels (optional)
        configInput(INPUT_CV_DRYWET, "CV Dry/Wet");
        configInput(INPUT_CV_GAIN,   "CV Gain");
        configInput(INPUT_CV_TONE,   "CV Tone");
        configInput(INPUT_CV_MACRO,  "CV Macro");
        configInput(INPUT_CLOCK,     "Clock");
        configInput(INPUT_IN_L,      "In L");
        configInput(INPUT_IN_R,      "In R");
        configOutput(OUTPUT_ENV,     "Env (0–8 V)");
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
        core_.setParams(pCur_);

        // --- Audio IO ---
        float inL = inputs[INPUT_IN_L].getVoltage();
        float inR = inputs[INPUT_IN_R].getVoltage();
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
        outputs[OUTPUT_ENV].setVoltage(envV);
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
        p.dryWet = clamp(get(PARAM_DRYWET) + get(PARAM_ATT_DRYWET) * sampleCv(INPUT_CV_DRYWET), 0.f, 1.f);
        p.gain   = clamp(get(PARAM_GAIN)   + get(PARAM_ATT_GAIN)   * sampleCv(INPUT_CV_GAIN),   0.f, 2.f);
        p.tone   = clamp(get(PARAM_TONE)   + get(PARAM_ATT_TONE)   * sampleCv(INPUT_CV_TONE),   0.f, 1.f);
        p.macro  = clamp(get(PARAM_MACRO)  + get(PARAM_ATT_MACRO)  * sampleCv(INPUT_CV_MACRO),  0.f, 1.f);
        return p;
    }
};
