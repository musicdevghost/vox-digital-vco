#pragma once
#include "../rack/plugin.hpp"
#include "../rack/ui/ui.hpp"
#include "../rack/ui/Layout12HP.hpp"

using namespace rack;

// Filled pepper overlay drawn in orange with alpha = LIGHT_PEPPER brightness
struct PepperOverlay : widget::TransparentWidget {
    std::shared_ptr<window::Svg> svg;

    PepperOverlay() {
        try { svg = APP->window->loadSvg(asset::plugin(pluginInstance, "res/pepper_fill.svg")); }
        catch (...) { svg.reset(); }
        if (svg) {
            box.pos  = layout::PEPPER_POS();
            box.size = svg->getSize();
        }
    }

    void draw(const DrawArgs& args) override {
        if (!svg) return;
        auto* mw = this->getAncestorOfType<app::ModuleWidget>();
        engine::Module* m = mw ? mw->module : nullptr;
        if (!m) return;

        float b = m->lights[LIGHT_PEPPER].getBrightness() * layout::PEPPER_BRIGHTNESS();
        if (b <= 0.f) return;

        nvgSave(args.vg);
        nvgTranslate(args.vg, box.pos.x, box.pos.y);
        nvgScale(args.vg, box.size.x / svg->getSize().x, box.size.y / svg->getSize().y);

        nvgBeginPath(args.vg);
        nvgFillColor(args.vg, nvgRGBAf(1.f, 0.25f, 0.0f, b)); // Illusions orange
        svg->draw(args.vg);
        nvgFill(args.vg);

        nvgRestore(args.vg);
    }
};
