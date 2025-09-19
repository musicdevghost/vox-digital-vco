#include "VoxSimModule.hpp"
#include "plugin.hpp"
#include "../shared/Config.hpp"

using namespace rack;
using namespace vm::vox;
using namespace vm::vox::glue;

VoxSimModule::VoxSimModule() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

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

    for (int i = 0; i < kBlock; ++i) {
        outL_[i] = 0.f; outR_[i] = 0.f;
        fm_[i] = 0.f; hsync_[i] = 0.f; ssync_[i] = 0.f;
    }
    frameIndex_ = 0;

    // Seed the first block render so we don't output a zero block
    // Build initial controls and ssync
    const float attPitchU  = bi_to_uni(params[PARAM_ATT_PITCH].getValue());
    const float attMorphU  = bi_to_uni(params[PARAM_ATT_MORPH].getValue());
    const float attTimbreU = bi_to_uni(params[PARAM_ATT_TIMBRE].getValue());
    const float attSpreadU = bi_to_uni(params[PARAM_ATT_SPREAD].getValue());
    const float cvPitchV  = inputs[INPUT_CV_PITCH].getVoltageSum();
    const float cvMorphV  = inputs[INPUT_CV_MORPH].getVoltageSum();
    const float cvTimbreV = inputs[INPUT_CV_TIMBRE].getVoltageSum();
    const float cvSpreadV = inputs[INPUT_CV_SPREAD].getVoltageSum();
    c_block_.pitchKnob01 = quantize12(apply_cv_att_rack(params[PARAM_PITCH].getValue(),  cvPitchV,  attPitchU));
    c_block_.morph01     = quantize12(apply_cv_att_rack(params[PARAM_MORPH].getValue(),  cvMorphV,  attMorphU));
    c_block_.timbre01    = quantize12(apply_cv_att_rack(params[PARAM_TIMBRE].getValue(), cvTimbreV, attTimbreU));
    c_block_.spread01    = quantize12(apply_cv_att_rack(params[PARAM_SPREAD].getValue(), cvSpreadV, attSpreadU));
    const float ssGate = inputs[INPUT_SOFT_SYNC].getVoltageSum() > 1.0f ? 1.0f : 0.0f;
    for (int i = 0; i < kBlock; ++i) ssync_[i] = ssGate;
}

void VoxSimModule::process(const ProcessArgs& args) {
    const int i = frameIndex_;

    // Per-sample capture of audio-rate inputs for the *current* block
    fm_[i]    = inputs[INPUT_FM_LINEAR].isConnected() ? voltsToBipolar(inputs[INPUT_FM_LINEAR].getVoltage()) : 0.f;
    hsync_[i] = inputs[INPUT_HARD_SYNC].isConnected() ? inputs[INPUT_HARD_SYNC].getVoltage() : 0.f;

    // Output current sample BEFORE preparing the next block to avoid buffer clobber
    outputs[OUTPUT_OUT_L].setVoltage(5.f * outL_[i]);
    outputs[OUTPUT_OUT_R].setVoltage(5.f * outR_[i]);

    frameIndex_++;

    // At the end of the block, render the *next* block and refresh controls/ssync
    if (frameIndex_ >= kBlock) {
        // Render next block using the fm_/hsync_/ssync_ we just captured
        Mods m;
        m.fm       = fm_;
        m.hsync    = hsync_;
        m.ssync    = ssync_;
        m.fmDepthHz= vm::vox::cfg::kDefaultFmDepthHz;
        core_.processBlock(coreParams_, c_block_, m, state_, outL_, outR_, kBlock);

        // Prepare controls and ssync for the next capture window
        const float attPitchU  = bi_to_uni(params[PARAM_ATT_PITCH].getValue());
        const float attMorphU  = bi_to_uni(params[PARAM_ATT_MORPH].getValue());
        const float attTimbreU = bi_to_uni(params[PARAM_ATT_TIMBRE].getValue());
        const float attSpreadU = bi_to_uni(params[PARAM_ATT_SPREAD].getValue());
        const float cvPitchV  = inputs[INPUT_CV_PITCH].getVoltageSum();
        const float cvMorphV  = inputs[INPUT_CV_MORPH].getVoltageSum();
        const float cvTimbreV = inputs[INPUT_CV_TIMBRE].getVoltageSum();
        const float cvSpreadV = inputs[INPUT_CV_SPREAD].getVoltageSum();
        c_block_.pitchKnob01 = quantize12(apply_cv_att_rack(params[PARAM_PITCH].getValue(),  cvPitchV,  attPitchU));
        c_block_.morph01     = quantize12(apply_cv_att_rack(params[PARAM_MORPH].getValue(),  cvMorphV,  attMorphU));
        c_block_.timbre01    = quantize12(apply_cv_att_rack(params[PARAM_TIMBRE].getValue(), cvTimbreV, attTimbreU));
        c_block_.spread01    = quantize12(apply_cv_att_rack(params[PARAM_SPREAD].getValue(), cvSpreadV, attSpreadU));
        const float ssGate = inputs[INPUT_SOFT_SYNC].getVoltageSum() > 1.0f ? 1.0f : 0.0f;
        for (int i = 0; i < kBlock; ++i) ssync_[i] = ssGate;

        frameIndex_ = 0;
    }
}
