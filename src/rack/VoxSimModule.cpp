#include "VoxSimModule.hpp"
#include "plugin.hpp"

using namespace rack;

VoxSimModule::VoxSimModule() {
    config(NUM_PARAMS, NUM_INPUTS, NUM_OUTPUTS, NUM_LIGHTS);

    // Params required by your TemplateWidget layout
    configParam(PARAM_PITCH, 0.f, 1.f, 0.5f, "Pitch macro", "%", 0.f, 100.f, 0.f);
    configParam(PARAM_MORPH, 0.f, 1.f, 0.0f, "Morph");
    configParam(PARAM_SPREAD, 0.f, 1.f, 0.0f, "Spread");
    configParam(PARAM_TIMBRE, 0.f, 1.f, 0.0f, "Timbre");
    configParam(PARAM_ATT_PITCH, -1.f, 1.f, 0.0f, "Atten Pitch");
    configParam(PARAM_ATT_MORPH, -1.f, 1.f, 0.0f, "Atten Morph");
    configParam(PARAM_ATT_SPREAD, -1.f, 1.f, 0.0f, "Atten Spread");
    configParam(PARAM_ATT_TIMBRE, -1.f, 1.f, 0.0f, "Atten Timbre");

    configOutput(OUTPUT_OUT_L, "Left");
    configOutput(OUTPUT_OUT_R, "Right");
    configOutput(OUTPUT_AUX, "Aux");

    configInput(INPUT_CV_PITCH, "CV Pitch");
    configInput(INPUT_CV_MORPH, "CV Morph");
    configInput(INPUT_CV_SPREAD, "CV Spread");
    configInput(INPUT_CV_TIMBRE, "CV Timbre");
    configInput(INPUT_HARD_SYNC, "Hard Sync");
    configInput(INPUT_FM_LINEAR, "FM Linear");
    configInput(INPUT_SOFT_SYNC, "Soft Sync");

    coreParams_.sampleRate = 48000.0;
    core_.setup(coreParams_);
    core_.reset();
}

void VoxSimModule::process(const ProcessArgs& args) {
    if (frameIndex_ >= kBlock) {
        frameIndex_ = 0;
        hal_.pitchKnob01 = params[PARAM_PITCH].getValue();
        vm::vox::Controls controls = hal_.buildControls();
        vm::vox::Mods mods;

        core_.processBlock(coreParams_, controls, mods, state_, outL_, outR_, kBlock);
    }
    outputs[OUTPUT_OUT_L].setVoltage(5.f * outL_[frameIndex_]);
    outputs[OUTPUT_OUT_R].setVoltage(5.f * outR_[frameIndex_]);
    frameIndex_++;
}
