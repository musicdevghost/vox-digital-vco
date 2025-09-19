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

    // Attenuuverters are UI -1..+1, convert to 0..1 for ControlMap
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

    // Ensure first block outputs are zeros until rendered
    for (int i = 0; i < kBlock; ++i) {
        outL_[i] = 0.f; outR_[i] = 0.f;
        fm_[i] = 0.f; hsync_[i] = 0.f; ssync_[i] = 0.f;
    }
    frameIndex_ = kBlock; // force control sampling on first call
}

void VoxSimModule::process(const ProcessArgs& args) {
    // Start a new 48-sample "hardware tick" when the block is exhausted
    if (frameIndex_ >= kBlock) {
        frameIndex_ = 0;

        // ---- Sample controls once per kBlock (hardware tick) ----
        const float knobPitch01  = params[PARAM_PITCH].getValue();
        const float knobMorph01  = params[PARAM_MORPH].getValue();
        const float knobTimbre01 = params[PARAM_TIMBRE].getValue();
        const float knobSpread01 = params[PARAM_SPREAD].getValue();

        // Convert attenuverters -1..+1 -> 0..1 for shared glue
        const float attPitchU  = bi_to_uni(params[PARAM_ATT_PITCH].getValue());
        const float attMorphU  = bi_to_uni(params[PARAM_ATT_MORPH].getValue());
        const float attTimbreU = bi_to_uni(params[PARAM_ATT_TIMBRE].getValue());
        const float attSpreadU = bi_to_uni(params[PARAM_ATT_SPREAD].getValue());

        // CVs (volts)
        const float cvPitchV  = inputs[INPUT_CV_PITCH].getVoltageSum();
        const float cvMorphV  = inputs[INPUT_CV_MORPH].getVoltageSum();
        const float cvTimbreV = inputs[INPUT_CV_TIMBRE].getVoltageSum();
        const float cvSpreadV = inputs[INPUT_CV_SPREAD].getVoltageSum();

        Controls c;
        c.pitchKnob01 = quantize12(apply_cv_att_rack(knobPitch01,  cvPitchV,  attPitchU));
        c.morph01     = quantize12(apply_cv_att_rack(knobMorph01,  cvMorphV,  attMorphU));
        c.timbre01    = quantize12(apply_cv_att_rack(knobTimbre01, cvTimbreV, attTimbreU));
        c.spread01    = quantize12(apply_cv_att_rack(knobSpread01, cvSpreadV, attSpreadU));

        // ---- Prepare SSYNC gate for this block ----
        const float ssV = inputs[INPUT_SOFT_SYNC].getVoltageSum();
        const float ssGate = ssV > 1.0f ? 1.0f : 0.0f;
        for (int i = 0; i < kBlock; ++i) ssync_[i] = ssGate;

        // Store the quantized controls into state_ (optional), or simply pass to core when ready.
        // We'll recompute at the end of the block before calling the core to keep it simple.
        // Here we precompute once to keep UX timely; exact value used is recomputed below as well.
    }

    // ---- Per-sample staging of audio-rate inputs ----
    const int i = frameIndex_;
    // Use *current time* sample from the cable (not poly channel index)
    const float fmV = inputs[INPUT_FM_LINEAR].getVoltage();
    const float hsV = inputs[INPUT_HARD_SYNC].getVoltage();
    fm_[i]    = voltsToBipolar(fmV);
    hsync_[i] = hsV; // zero-crossing detection is done in VoxCore

    // When the block is full, run the shared core to render the next 48 outputs
    if (i == kBlock - 1) {
        Mods m;
        m.fm       = fm_;
        m.hsync    = hsync_;
        m.ssync    = ssync_;
        m.fmDepthHz= 440.0; // tune to taste

        // Recompute quantized controls here to ensure we have the latest values
        const float knobPitch01  = params[PARAM_PITCH].getValue();
        const float knobMorph01  = params[PARAM_MORPH].getValue();
        const float knobTimbre01 = params[PARAM_TIMBRE].getValue();
        const float knobSpread01 = params[PARAM_SPREAD].getValue();
        const float attPitchU  = bi_to_uni(params[PARAM_ATT_PITCH].getValue());
        const float attMorphU  = bi_to_uni(params[PARAM_ATT_MORPH].getValue());
        const float attTimbreU = bi_to_uni(params[PARAM_ATT_TIMBRE].getValue());
        const float attSpreadU = bi_to_uni(params[PARAM_ATT_SPREAD].getValue());
        const float cvPitchV  = inputs[INPUT_CV_PITCH].getVoltageSum();
        const float cvMorphV  = inputs[INPUT_CV_MORPH].getVoltageSum();
        const float cvTimbreV = inputs[INPUT_CV_TIMBRE].getVoltageSum();
        const float cvSpreadV = inputs[INPUT_CV_SPREAD].getVoltageSum();
        Controls c;
        c.pitchKnob01 = quantize12(apply_cv_att_rack(knobPitch01,  cvPitchV,  attPitchU));
        c.morph01     = quantize12(apply_cv_att_rack(knobMorph01,  cvMorphV,  attMorphU));
        c.timbre01    = quantize12(apply_cv_att_rack(knobTimbre01, cvTimbreV, attTimbreU));
        c.spread01    = quantize12(apply_cv_att_rack(knobSpread01, cvSpreadV, attSpreadU));

        core_.processBlock(coreParams_, c, m, state_, outL_, outR_, kBlock);
    }

    // ---- Output current sample ----
    outputs[OUTPUT_OUT_L].setVoltage(5.f * outL_[i]);
    outputs[OUTPUT_OUT_R].setVoltage(5.f * outR_[i]);

    frameIndex_++;
}
