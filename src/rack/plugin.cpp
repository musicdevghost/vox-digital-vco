#include "plugin.hpp"
#include "../framework/TemplateWidget.hpp"
#include "../framework/TemplateModule.hpp"

// NEW: digital VCO core
#include "../dsp/VoxVcoCore.hpp"

#define REGISTER_VOX_MODULE(ClassName, CoreType, slug) \
    using ClassName = VoxTemplateModule<CoreType>;      \
    Model* model##ClassName = createModel<ClassName, VoxTemplateWidget>(slug)

Plugin* pluginInstance = nullptr;

// NEW module (note vm:: namespace for VoxVcoCore)
REGISTER_VOX_MODULE(VoxVco, vm::VoxVcoCore, "vox-vco");

void init(Plugin* p) {
    pluginInstance = p;

    // NEW: register the VCO
    p->addModel(modelVoxVco);
}
