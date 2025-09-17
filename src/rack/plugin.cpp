#include "plugin.hpp"
#include "../framework/TemplateWidget.hpp"
#include "VoxSimModule.hpp"

Plugin* pluginInstance = nullptr;
Model* modelVOX = createModel<VoxSimModule, VoxTemplateWidget>("vox-vco");

void init(Plugin* p) {
    pluginInstance = p;
    p->addModel(modelVOX);
}
