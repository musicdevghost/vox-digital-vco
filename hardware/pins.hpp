#pragma once
#include "daisy_seed.h"

namespace hwpins
{
    using daisy::Pin;

    // ---- Direct CV ADCs ----
    static constexpr Pin ADC_CV_TIMBRE = daisy::seed::A0; // CV1
    static constexpr Pin ADC_CV_PITCH  = daisy::seed::A1; // CV2 (V/Oct)
    static constexpr Pin ADC_CV_SPREAD = daisy::seed::A2; // CV3
    static constexpr Pin ADC_CV_MORPH  = daisy::seed::A3; // CV4

    // ---- MUX 1: raw pots (IC8) ----
    static constexpr Pin MUX1_COM = daisy::seed::A5;
    static constexpr Pin MUX1_S0  = daisy::seed::D6; // MUX_ADDRESS_1_0
    static constexpr Pin MUX1_S1  = daisy::seed::D8; // MUX_ADDRESS_1_1
    static constexpr Pin MUX1_S2  = daisy::seed::D9; // MUX_ADDRESS_1_2
    // Channels (per schematic text on IC8):
    // ch0=FILTER (Tone), ch1=SIZE (Pitch), ch2=FEEDB (Morph), ch3=DIFF (Spread)

    // ---- MUX 2: attenuverters (IC9) ----
    static constexpr Pin MUX2_COM = daisy::seed::A6;
    static constexpr Pin MUX2_S0  = daisy::seed::D1; // MUX_ADDRESS_2_0
    static constexpr Pin MUX2_S1  = daisy::seed::D2; // MUX_ADDRESS_2_1
    static constexpr Pin MUX2_S2  = daisy::seed::D3; // MUX_ADDRESS_2_2
    // ch0=FILTER_AT (Timbre), ch1=SIZE_AT (Pitch), ch2=FEEDB_AT (Morph), ch3=DIFF_AT (Spread)

    // ---- Indicators ----
    static constexpr Pin PANEL_LED = daisy::seed::D29;

    // ---- DAC OUT1 (ENV) is PA4; libDaisy handles it via hw.dac ----
}
