#include "plugin.hpp"
#include "framework/TemplateWidget.hpp"
#include "framework/TemplateModule.hpp"
#include "dsp/VoxAudioCore.hpp"

#define REGISTER_VOX_MODULE(ClassName, CoreType, slug)               using ClassName = VoxTemplateModule<CoreType>;                    Model* model##ClassName = createModel<ClassName, VoxTemplateWidget>(slug)

Plugin* pluginInstance = nullptr;
REGISTER_VOX_MODULE(VoxAudio, VoxAudioCore, "vox-audio");

void init(Plugin* p) {
    pluginInstance = p;
    p->addModel(modelVoxAudio);
}
