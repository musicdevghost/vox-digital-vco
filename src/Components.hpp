#pragma once
#include "plugin.hpp"
#include "ui.hpp"

struct BananutBlackPort : app::SvgPort {
    BananutBlackPort() {
        setSvg(trySvg("res/blacknut.svg", "res/ComponentLibrary/PJ301M.svg"));
    }
};

struct BananutRedPort : app::SvgPort {
    BananutRedPort() {
        setSvg(trySvg("res/rednut.svg", "res/ComponentLibrary/PJ301M.svg"));
    }
};

struct Davies1900hCreamKnob : rack::componentlibrary::Davies1900hKnob {
    Davies1900hCreamKnob() {
        setSvg(rack::Svg::load(rack::asset::plugin(pluginInstance, "res/Davies1900hCream.svg")));
        bg->setSvg(rack::Svg::load(rack::asset::plugin(pluginInstance, "res/Davies1900hCream_bg.svg")));
    }
};

struct VoxTinyKnob : rack::componentlibrary::BefacoTinyKnob {
    VoxTinyKnob() {
        setSvg(rack::Svg::load(rack::asset::plugin(pluginInstance, "res/blackpointer.svg")));
        bg->setSvg(rack::Svg::load(rack::asset::plugin(pluginInstance, "res/tinyknob_bg.svg")));
    }
};

struct BananutBluePort : app::SvgPort {
    BananutBluePort() {
        setSvg(trySvg("res/bluenut.svg", "res/ComponentLibrary/PJ301M.svg"));
    }
};