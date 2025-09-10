#pragma once
#include "daisy_seed.h"
#include <cstddef>
#include "dsp/shared/OnePole.hpp"

namespace hw {

struct Tunables
{
    // If smoothMs > 0, that wins (SR-aware smoothing).
    // Otherwise, if smoothAlpha in [0..1] is provided, that is used.
    // Defaults provide 8 ms smoothing.
    float smoothAlpha = -1.f;  // set < 0 to ignore
    float smoothMs    = 8.f;   // preferred
    float envAtkMs    = 8.f;
    float envRelMs    = 160.f;
};

struct Controls
{
    float pitch      = 0.f; // 0..1
    float timbre     = 0.f; // 0..1
    float morph      = 0.f; // 0..1
    float spread     = 0.f; // 0..1
    float pitchVolts = 0.f; // volts (1V/oct)
};

class Hw
{
  public:
    void init(float sr, size_t blocksize, const Tunables& t);
    void startAudio(void (*cb)(daisy::AudioHandle::InputBuffer,
                               daisy::AudioHandle::OutputBuffer,
                               size_t));
    void poll();
    void writeEnv(float env01);

    const Controls& controls() const { return controls_; }
    daisy::DaisySeed* seed() { return &seed_; }

  private:
    void initAdc_();
    void initMux_(int, int, int);
    void stepMuxes_();
    void updateControls_();

    float              sr_        = 48000.f;
    size_t             blocksize_ = 48;
    Tunables           tun_{};

    // Daisy HW
    daisy::DaisySeed   seed_;
    daisy::AudioHandle audio_;
    daisy::GPIO        led_;
    daisy::DacHandle   dac_;

    // ADC channel indices
    int adc_cv1_ = 0, adc_cv2_ = 1, adc_cv3_ = 2, adc_cv4_ = 3, adc_m1_ = 4, adc_m2_ = 5;

    // MUX select pins
    daisy::GPIO m1_s0_, m1_s1_, m1_s2_;
    daisy::GPIO m2_s0_, m2_s1_, m2_s2_;

    // MUX readback ring (we only use 0..3, sized 8 for completeness)
    float   m1_val_[8] = {0};
    float   m2_val_[8] = {0};
    uint8_t m1_prev_ = 0, m1_curr_ = 0;
    uint8_t m2_prev_ = 0, m2_curr_ = 0;

    // Smoothers
    vm::OnePole sm_knob_[4]; // pots
    vm::OnePole sm_att_[4];  // attenuverters
    vm::OnePole sm_cv_[4];   // direct CVs

    // Mixed, normalized params for cores
    Controls controls_{};
};

} // namespace hw
