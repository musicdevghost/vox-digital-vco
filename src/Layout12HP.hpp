#pragma once
#include "plugin.hpp"

// 12HP = 60.96 mm x 128.5 mm
namespace layout {
    inline Vec PANEL_SIZE()   { return rack::mm2px(Vec(60.96f, 128.5f)); }

    // Big knobs
    inline Vec KNOB_PITCH()  { return rack::mm2px(Vec(29.308f, 16.679f)); }
    inline Vec KNOB_MORPH()    { return rack::mm2px(Vec(49.621f, 32.657f)); }
    inline Vec KNOB_SPREAD()    { return rack::mm2px(Vec(30.337f, 48.892f)); }
    inline Vec KNOB_TIMBRE()   { return rack::mm2px(Vec(11.041f, 32.661f)); }

    // Trims
    inline Vec TRIM_TIMBRE()   { return rack::mm2px(Vec(8.325f, 77.25f)); }
    inline Vec TRIM_PITCH()  { return rack::mm2px(Vec(22.975f, 77.25f)); }
    inline Vec TRIM_SPREAD()    { return rack::mm2px(Vec(37.626f, 77.25f)); }
    inline Vec TRIM_MORPH()    { return rack::mm2px(Vec(52.275f, 77.25f)); }

    // Mid jacks
    inline Vec JACK_SOFT_SYNC()   { return rack::mm2px(Vec( 8.325f, 57.25f)); }
    inline Vec JACK_AUX()     { return rack::mm2px(Vec(52.275f, 57.25f)); }

    // CV row
    inline Vec CV_TIMBRE()     { return rack::mm2px(Vec(8.325f, 95.75f)); }
    inline Vec CV_PITCH()    { return rack::mm2px(Vec( 22.975f, 95.75f)); }
    inline Vec CV_SPREAD()      { return rack::mm2px(Vec(37.626f, 95.75f)); }
    inline Vec CV_MORPH()      { return rack::mm2px(Vec(52.275f, 95.75f)); }

    // Bottom row audio I/O
    inline Vec IN_HARD_SYNC()         { return rack::mm2px(Vec( 8.325f, 110.0f)); }
    inline Vec IN_FM_LINEAR()         { return rack::mm2px(Vec(22.975f, 110.0f)); }
    inline Vec OUT_L()        { return rack::mm2px(Vec(37.626f, 110.0f)); }
    inline Vec OUT_R()        { return rack::mm2px(Vec(52.275f, 110.0f)); }

    // Lights
    inline Vec LIGHT_STATUS() { return rack::mm2px(Vec(30.5f, 31.0f)); }

    // 🔥 Filled pepper overlay anchor (top-left of pepper_fill.svg)
    inline float PEPPER_BRIGHTNESS() { return 0.60f; }
    inline Vec PEPPER_POS()   { return rack::mm2px(Vec(10.8f, 14.7f)); }
}
