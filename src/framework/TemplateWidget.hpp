#pragma once
#include "plugin.hpp"
#include "../Components.hpp"
#include "../Layout12HP.hpp"
#include "TemplateIO.hpp"
#include "PepperOverlay.hpp"
#include "../ui.hpp"
#include <componentlibrary.hpp>

using namespace rack;

struct VoxTemplateWidget : app::ModuleWidget {
    VoxTemplateWidget(engine::Module* module) {
        setModule(module);
        setPanel(trySvg("res/faceplate.svg", "res/ComponentLibrary/Panel.svg"));

        // Screws
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
        addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
        addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

        // Pepper overlay (auto-skips if svg missing)
        addChild(new PepperOverlay());

        // Big knobs (Davies white) and trims (Befaco tiny)
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_PITCH(), module, PARAM_PITCH));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_MORPH(),   module, PARAM_MORPH));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_SPREAD(),   module, PARAM_SPREAD));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_TIMBRE(),  module, PARAM_TIMBRE));

        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_PITCH(), module, PARAM_ATT_PITCH));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_MORPH(),   module, PARAM_ATT_MORPH));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_SPREAD(),   module, PARAM_ATT_SPREAD));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_TIMBRE(),  module, PARAM_ATT_TIMBRE));

        // CV inputs
        addInput(createInputCentered<BananutBlackPort>(layout::CV_PITCH(), module, INPUT_CV_PITCH));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_MORPH(),   module, INPUT_CV_MORPH));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_SPREAD(),   module, INPUT_CV_SPREAD));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_TIMBRE(),  module, INPUT_CV_TIMBRE));

        // Mid-row jacks: clock (in) + env (out) at exact template locations
        addInput (createInputCentered<BananutBlackPort>(layout::JACK_SOFT_SYNC(), module, INPUT_SOFT_SYNC));
        addOutput(createOutputCentered<BananutRedPort>(layout::JACK_AUX(),    module, OUTPUT_AUX));

        // Audio I/O bottom row (red outs)
        addInput (createInputCentered<BananutBlackPort>(layout::IN_HARD_SYNC(), module, INPUT_HARD_SYNC));
        addInput (createInputCentered<BananutBlackPort>(layout::IN_FM_LINEAR(), module, INPUT_FM_LINEAR));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_L(), module, OUTPUT_OUT_L));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_R(), module, OUTPUT_OUT_R));
    }
};
