#pragma once
#include "plugin.hpp"
#include "../Components.hpp"
#include "../Layout12HP.hpp"
#include "TemplateIO.hpp"
#include "PepperOverlay.hpp"
#include "../ui.hpp"
#include <componentlibrary.hpp>
#include "VoxScrew.hpp"

using namespace rack;

struct VoxTemplateWidget : app::ModuleWidget {
    VoxTemplateWidget(engine::Module* module) {
        setModule(module);
        setPanel(trySvg("res/faceplate.svg", "res/ComponentLibrary/Panel.svg"));

        // Screws
        // How much to move (in mm)
        static constexpr float kTopDropMM    = 2.725f;   // move TOP screws down
        static constexpr float kBottomRiseMM = -2.325f;   // move BOTTOM screws up

        // Precompute nudges in px
        const float topDropPx    = mm2px(kTopDropMM);
        const float bottomRisePx = mm2px(kBottomRiseMM);

        // X positions stay the same
        const float xLeft  = RACK_GRID_WIDTH;
        const float xRight = box.size.x - 1 * RACK_GRID_WIDTH;

        // Y bases are the usual grid anchors
        const float yTopBase    = 0.f;
        const float yBottomBase = RACK_GRID_HEIGHT - RACK_GRID_WIDTH;

        // Place with nudges
        addChild(createWidget<VoxScrew>(rack::Vec(xLeft,  yTopBase    + topDropPx)));
        addChild(createWidget<VoxScrew>(rack::Vec(xRight, yTopBase    + topDropPx)));
        addChild(createWidget<VoxScrew>(rack::Vec(xLeft,  yBottomBase - bottomRisePx)));
        addChild(createWidget<VoxScrew>(rack::Vec(xRight, yBottomBase - bottomRisePx)));

        // Pepper overlay (auto-skips if svg missing)
        addChild(new PepperOverlay());

        // Big knobs (Davies white) and trims (Befaco tiny)
        addParam(createParamCentered<Davies1900hCreamKnob>(layout::KNOB_PITCH(), module, PARAM_PITCH));
        addParam(createParamCentered<Davies1900hCreamKnob>(layout::KNOB_MORPH(),   module, PARAM_MORPH));
        addParam(createParamCentered<Davies1900hCreamKnob>(layout::KNOB_SPREAD(),   module, PARAM_SPREAD));
        addParam(createParamCentered<Davies1900hCreamKnob>(layout::KNOB_TIMBRE(),  module, PARAM_TIMBRE));

        addParam(createParamCentered<VoxTinyKnob>(layout::TRIM_PITCH(), module, PARAM_ATT_PITCH));
        addParam(createParamCentered<VoxTinyKnob>(layout::TRIM_MORPH(),   module, PARAM_ATT_MORPH));
        addParam(createParamCentered<VoxTinyKnob>(layout::TRIM_SPREAD(),   module, PARAM_ATT_SPREAD));
        addParam(createParamCentered<VoxTinyKnob>(layout::TRIM_TIMBRE(),  module, PARAM_ATT_TIMBRE));

        // CV inputs
        addInput(createInputCentered<BananutBlackPort>(layout::CV_PITCH(), module, INPUT_CV_PITCH));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_MORPH(),   module, INPUT_CV_MORPH));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_SPREAD(),   module, INPUT_CV_SPREAD));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_TIMBRE(),  module, INPUT_CV_TIMBRE));

        // Mid-row jacks: clock (in) + env (out) at exact template locations
        addInput (createInputCentered<BananutBlackPort>(layout::JACK_SOFT_SYNC(), module, INPUT_SOFT_SYNC));
        addOutput(createOutputCentered<BananutRedPort>(layout::JACK_AUX(),    module, OUTPUT_AUX));

        // Audio I/O bottom row (red outs)
        addInput (createInputCentered<BananutBluePort>(layout::IN_HARD_SYNC(), module, INPUT_HARD_SYNC));
        addInput (createInputCentered<BananutBluePort>(layout::IN_FM_LINEAR(), module, INPUT_FM_LINEAR));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_L(), module, OUTPUT_OUT_L));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_R(), module, OUTPUT_OUT_R));
    }
};
