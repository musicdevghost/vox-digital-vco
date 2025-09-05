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
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_DRYWET(), module, PARAM_PITCH));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_GAIN(),   module, PARAM_MORPH));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_TONE(),   module, PARAM_SPREAD));
        addParam(createParamCentered<Davies1900hWhiteKnob>(layout::KNOB_MACRO(),  module, PARAM_TIMBRE));

        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_DRYWET(), module, PARAM_ATT_PITCH));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_GAIN(),   module, PARAM_ATT_MORPH));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_TONE(),   module, PARAM_ATT_SPREAD));
        addParam(createParamCentered<BefacoTinyKnob>(layout::TRIM_MACRO(),  module, PARAM_ATT_TIMBRE));

        // CV inputs
        addInput(createInputCentered<BananutBlackPort>(layout::CV_DRYWET(), module, INPUT_CV_PITCH));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_GAIN(),   module, INPUT_CV_MORPH));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_TONE(),   module, INPUT_CV_SPREAD));
        addInput(createInputCentered<BananutBlackPort>(layout::CV_MACRO(),  module, INPUT_CV_TIMBRE));

        // Mid-row jacks: clock (in) + env (out) at exact template locations
        addInput (createInputCentered<BananutBlackPort>(layout::JACK_CLOCK(), module, INPUT_SOFT_SYNC));
        addOutput(createOutputCentered<BananutRedPort>(layout::JACK_ENV(),    module, OUTPUT_AUX));

        // Audio I/O bottom row (red outs)
        addInput (createInputCentered<BananutBlackPort>(layout::IN_L(), module, INPUT_HARD_SYNC));
        addInput (createInputCentered<BananutBlackPort>(layout::IN_R(), module, INPUT_FM_LINEAR));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_L(), module, OUTPUT_OUT_L));
        addOutput(createOutputCentered<BananutRedPort>(layout::OUT_R(), module, OUTPUT_OUT_R));
    }
};
