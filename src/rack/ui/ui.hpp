#pragma once
#include "../plugin.hpp"

inline std::shared_ptr<window::Svg> trySvg(const std::string& relPath, const std::string& systemFallback) {
    std::shared_ptr<window::Svg> svg;
    try {
        svg = APP->window->loadSvg(asset::plugin(pluginInstance, relPath));
    } catch (...) {}
    if (!svg) {
        svg = APP->window->loadSvg(asset::system(systemFallback));
    }
    return svg;
}

struct VoxKnobLarge : app::SvgKnob {
    VoxKnobLarge() {
        setSvg(trySvg("res/knobs/KnobLarge.svg", "res/ComponentLibrary/Rogan1PWhite.svg"));
    }
};

struct VoxKnobSmall : app::SvgKnob {
    VoxKnobSmall() {
        setSvg(trySvg("res/knobs/KnobSmall.svg", "res/ComponentLibrary/BefacoTinyKnob.svg"));
    }
};
