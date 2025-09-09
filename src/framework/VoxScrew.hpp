// VoxScrew.hpp
#pragma once
#include "plugin.hpp"

struct VoxScrew : rack::Widget {
    rack::SvgWidget* bg = nullptr;
    rack::SvgWidget* head = nullptr;

    VoxScrew() {
        // Load background
        bg = new rack::SvgWidget();

        // Load head
        head = new rack::SvgWidget();
        head->setSvg(rack::Svg::load(rack::asset::plugin(pluginInstance, "res/knurlie.svg")));
        addChild(head);

        // Container box size = background size (SVG intrinsic)
        box.size = bg->box.size;

        // Center background
        bg->box.pos = rack::Vec(0, 0);

        // Center head relative to background
        rack::Vec center = box.size.minus(head->box.size).div(2.f);
        head->box.pos = center;
    }
};
