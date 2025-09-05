#pragma once

// Shared control/jack/light indices to match your template layout (VoxAudio-style)
enum ParamIds {
    PARAM_DRYWET, PARAM_GAIN, PARAM_TONE, PARAM_MACRO,
    PARAM_ATT_DRYWET, PARAM_ATT_GAIN, PARAM_ATT_TONE, PARAM_ATT_MACRO,
    NUM_PARAMS
};

enum InputIds {
    INPUT_CV_DRYWET, INPUT_CV_GAIN, INPUT_CV_TONE, INPUT_CV_MACRO,
    INPUT_CLOCK,
    INPUT_IN_L, INPUT_IN_R,
    NUM_INPUTS
};

enum OutputIds {
    OUTPUT_ENV,
    OUTPUT_OUT_L, OUTPUT_OUT_R,
    NUM_OUTPUTS
};

enum LightIds {
    LIGHT_PEPPER,
    NUM_LIGHTS
};
