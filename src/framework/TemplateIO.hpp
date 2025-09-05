#pragma once

// Shared control/jack/light indices to match your template layout (VoxAudio-style)
enum ParamIds {
    PARAM_PITCH, PARAM_MORPH, PARAM_SPREAD, PARAM_TIMBRE,
    PARAM_ATT_PITCH, PARAM_ATT_MORPH, PARAM_ATT_SPREAD, PARAM_ATT_TIMBRE,
    NUM_PARAMS
};

enum InputIds {
    INPUT_CV_PITCH, INPUT_CV_MORPH, INPUT_CV_SPREAD, INPUT_CV_TIMBRE,
    INPUT_SOFT_SYNC,
    INPUT_HARD_SYNC, INPUT_FM_LINEAR,
    NUM_INPUTS
};

enum OutputIds {
    OUTPUT_AUX,
    OUTPUT_OUT_L, OUTPUT_OUT_R,
    NUM_OUTPUTS
};

enum LightIds {
    LIGHT_PEPPER,
    NUM_LIGHTS
};
