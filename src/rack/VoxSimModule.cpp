#include "VoxSimModule.hpp"
#include "plugin.hpp"

using namespace rack;
using namespace vm::vox;
using namespace vm::vox::glue;

VoxSimModule::VoxSimModule() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

    // Params from your TemplateWidget layout
    configParam(PARAM_PITCH,  0.f, 1.f, 0.5f, "Pitch macro");
    configParam(PARAM_MORPH,  0.f, 1.f, 0.0f, "Morph");
    configParam(PARAM_SPREAD, 0.f, 1.f, 1.0f, "Spread");
    configParam(PARAM_TIMBRE, 0.f, 1.f, 0.5f, "Timbre");

    configParam(PARAM_ATT_PITCH,  -1.f, 1.f, 0.0f, "Atten Pitch");
    configParam(PARAM_ATT_MORPH,  -1.f, 1.f, 0.0f, "Atten Morph");
    configParam(PARAM_ATT_SPREAD, -1.f, 1.f, 0.0f, "Atten Spread");
    configParam(PARAM_ATT_TIMBRE, -1.f, 1.f, 0.0f, "Atten Timbre");

    configInput(INPUT_CV_PITCH,  "CV Pitch");
    configInput(INPUT_CV_MORPH,  "CV Morph");
    configInput(INPUT_CV_SPREAD, "CV Spread");
    configInput(INPUT_CV_TIMBRE, "CV Timbre");
    configInput(INPUT_HARD_SYNC, "Hard Sync (audio)");
    configInput(INPUT_FM_LINEAR, "FM Linear (audio)");
    configInput(INPUT_SOFT_SYNC, "Soft Sync (gate)");

    configOutput(OUTPUT_OUT_L, "Left");
    configOutput(OUTPUT_OUT_R, "Right");
    configOutput(OUTPUT_AUX,   "Aux");

    coreParams_.sampleRate = 48000.0;
    core_.setup(coreParams_);
    core_.reset();
}

void VoxSimModule::process(const ProcessArgs& args) {
    if (frameIndex_ >= kBlock) {
        frameIndex_ = 0;

        // --- Controls (once per Daisy tick) ---
        float knobPitch01  = params[PARAM_PITCH].getValue();
        float knobMorph01  = params[PARAM_MORPH].getValue();
        float knobTimbre01 = params[PARAM_TIMBRE].getValue();
        float knobSpread01 = params[PARAM_SPREAD].getValue();

        float attPitch  = params[PARAM_ATT_PITCH].getValue();
        float attMorph  = params[PARAM_ATT_MORPH].getValue();
        float attTimbre = params[PARAM_ATT_TIMBRE].getValue();
        float attSpread = params[PARAM_ATT_SPREAD].getValue();

        float cvPitchV  = inputs[INPUT_CV_PITCH].getVoltageSum();
        float cvMorphV  = inputs[INPUT_CV_MORPH].getVoltageSum();
        float cvTimbreV = inputs[INPUT_CV_TIMBRE].getVoltageSum();
        float cvSpreadV = inputs[INPUT_CV_SPREAD].getVoltageSum();

        Controls c;
        c.pitchKnob01 = quantize12(apply_cv_att_rack(knobPitch01,  cvPitchV,  attPitch));
        c.morph01     = quantize12(apply_cv_att_rack(knobMorph01,  cvMorphV,  attMorph));
        c.timbre01    = quantize12(apply_cv_att_rack(knobTimbre01, cvTimbreV, attTimbre));
        c.spread01    = quantize12(apply_cv_att_rack(knobSpread01, cvSpreadV, attSpread));

        // --- Audio-rate inputs (fill one block) ---
        // FM: sum mono FM input; scale to -1..+1 by 10 V
        float fmV = inputs[INPUT_FM_LINEAR].isConnected() ? inputs[INPUT_FM_LINEAR].getVoltageSum() : 0.f;
        float hsV = inputs[INPUT_HARD_SYNC].isConnected() ? inputs[INPUT_HARD_SYNC].getVoltageSum() : 0.f;
        float ssV = inputs[INPUT_SOFT_SYNC].isConnected() ? inputs[INPUT_SOFT_SYNC].getVoltageSum() : 0.f;

        for (int i = 0; i < kBlock; ++i) {
            fm_[i]    = voltsToBipolar(fmV); // DC sample per tick; simple but deterministic
            hsync_[i] = hsV;                 // use zero-crossing detection inside core
            ssync_[i] = ssV > 1.0f ? 1.0f : 0.0f; // gate
        }

        Mods m;
        m.fm       = fm_;
        m.hsync    = hsync_;
        m.ssync    = ssync_;
        m.fmDepthHz= 440.0; // conservative default; tune as needed

        core_.processBlock(coreParams_, c, m, state_, outL_, outR_, kBlock);
    }

    outputs[OUTPUT_OUT_L].setVoltage(5.f * outL_[frameIndex_]);
    outputs[OUTPUT_OUT_R].setVoltage(5.f * outR_[frameIndex_]);
    frameIndex_++;
}
