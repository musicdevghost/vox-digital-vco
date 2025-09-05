#pragma once
#include <rack.hpp>

using namespace rack;

extern Plugin* pluginInstance;

// Generic widget from your template (forward-declared)
struct VoxTemplateWidget;

// Existing module(s)
extern Model* modelVoxAudio;

// NEW: Digital VCO module
extern Model* modelVoxVco;
