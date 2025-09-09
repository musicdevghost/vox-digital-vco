#pragma once
#include "daisy_seed.h"

namespace hwpins
{
    using daisy::Pin;

    // Pots
    static constexpr Pin ADC_SIZE   = daisy::seed::A0; // Size  -> Pitch
    static constexpr Pin ADC_TONE   = daisy::seed::A1; // Tone  -> Timbre
    static constexpr Pin ADC_FEEDB  = daisy::seed::A2; // Feedb -> Morph
    static constexpr Pin ADC_DIFF   = daisy::seed::A3; // Diff  -> Spread

    // CVs
    static constexpr Pin ADC_CV_TIMBRE = daisy::seed::A4; // CV1 → Timbre
    static constexpr Pin ADC_CV_PITCH  = daisy::seed::A5; // CV2 → Pitch (V/Oct)
    static constexpr Pin ADC_CV_SPREAD = daisy::seed::A6; // CV3 → Spread
    static constexpr Pin ADC_CV_MORPH  = daisy::seed::A7; // CV4 → Morph

    // Panel LED (digital)
    static constexpr Pin PANEL_LED = daisy::seed::D29;

    // PWM CV OUT “LED” (override with -DHWPINS_D_CVLED=<D#>)
    static constexpr Pin CVLED_PIN_DEFAULT = daisy::seed::D12;
}
