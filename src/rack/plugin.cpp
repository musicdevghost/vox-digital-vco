#include "plugin.hpp"
#include "../framework/TemplateWidget.hpp"
#include "../framework/TemplateModule.hpp"

// NEW: digital VCO core
#include "dsp/VoxVcoCore.hpp"

#define REGISTER_VOX_MODULE(ClassName, CoreType, slug) \
    using ClassName = VoxTemplateModule<CoreType>;      \
    Model* model##ClassName = createModel<ClassName, VoxTemplateWidget>(slug)

Plugin* pluginInstance = nullptr;

// NEW module (note vm:: namespace for VoxVcoCore)
REGISTER_VOX_MODULE(VoxVco, vm::VoxVcoCore, "vox-vco");

#if defined(__SSE2__)
#include <xmmintrin.h>
static inline void enableFTZ_DAZ(){
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
}
#endif

void init(Plugin* p) {
    pluginInstance = p;

    // NEW: register the VCO
    p->addModel(modelVoxVco);
}
