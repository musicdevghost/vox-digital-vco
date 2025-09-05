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

        // Screws at 12HP panel corners
        addChild(createWidget<ScrewSilver>(mm2px(Vec(5.0f, 5.0f))));
        addChild(createWidget<ScrewSilver>(mm2px(Vec(60.96f - 5.0f, 5.0f))));
        addChild(createWidget<ScrewSilver>(mm2px(Vec(5.0f, 128.5f - 5.0f))));
        addChild(createWidget<ScrewSilver>(mm2px(Vec(60.96f - 5.0f, 128.5f - 5.0f))));

        // Pepper overlay (auto-skips if svg missing)
        addChild(new PepperOverlay());

        // Big knobs (Davies white) and trims (Befaco tiny)
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_DRYWET(), module, PARAM_DRYWET));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_GAIN(),   module, PARAM_GAIN));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_TONE(),   module, PARAM_TONE));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_MACRO(),  module, PARAM_MACRO));

        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_DRYWET(), module, PARAM_ATT_DRYWET));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_GAIN(),   module, PARAM_ATT_GAIN));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_TONE(),   module, PARAM_ATT_TONE));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_MACRO(),  module, PARAM_ATT_MACRO));

        // CV inputs
        addInput(createInputCentered<BananutBlackPort>(layout::CV_DRYWET(), module, INPUT_CV_DRYWET));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_GAIN(),   module, INPUT_CV_GAIN));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_TONE(),   module, INPUT_CV_TONE));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_MACRO(),  module, INPUT_CV_MACRO));

        // Mid-row jacks: clock (in) + env (out) at exact template locations
        addInput (createInputCentered<BananutBlackPort>(layout::JACK_CLOCK(), module, INPUT_CLOCK));
        addOutput(createOutputCentered<BananutRedPort>(layout::JACK_ENV(),    module, OUTPUT_ENV));

        // Audio I/O bottom row (red outs)
        addInput (createInputCentered<BananutBlackPort>(layout::IN_L(), module, INPUT_IN_L));
        addInput (createInputCentered<BananutBlackPort>(layout::IN_R(), module, INPUT_IN_R));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_L(), module, OUTPUT_OUT_L));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_R(), module, OUTPUT_OUT_R));
    }
};
