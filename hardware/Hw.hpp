#pragma once
#include "daisy_seed.h"
#include <cstddef>
#include <cstdint>

namespace hw {

// --- Tunables ---
struct Tunables {
    float smoothAlpha = 0.12f;   // control smoothing (0..1)
    float envAtkMs    = 8.0f;    // envelope follower attack in ms
    float envRelMs    = 160.0f;  // envelope follower release in ms
};

// Mapped, smoothed controls (0..1), plus pitch volts for 1V/Oct cores.
struct Controls {
    float timbre     = 0.f; // FILTER  (knob + CV1*attenuverter), clamped 0..1
    float pitch      = 0.f; // SIZE    (knob + CV2*attenuverter), clamped 0..1
    float morph      = 0.f; // FEEDB   (knob + CV4*attenuverter), clamped 0..1
    float spread     = 0.f; // DIFF    (knob + CV3*attenuverter), clamped 0..1
    float pitchVolts = 0.f; // derived from CV2 raw (for true 1V/oct)
};

// Hardware wrapper: Seed + ADC + 2×4051 mux + LED + DAC + smoothing.
class Hw
{
  public:
    void init(float sr, size_t blocksize, const Tunables& t = {});
    void startAudio(void (*cb)(daisy::AudioHandle::InputBuffer,
                               daisy::AudioHandle::OutputBuffer,
                               size_t));

    // Call once per audio block to advance mux scanners and update smoothed controls
    void poll();

    // Latest mapped controls
    Controls controls() const { return controls_; }

    // Write 0..1 envelope to LED and DAC
    void writeEnv(float env01);

    // Pointers for audio ISR convenience
    daisy::AudioHandle* audio() { return &audio_; }
    daisy::DaisySeed*   seed()  { return &seed_;  }

  private:
    void initAdc_();
    void initMux_(int s0, int s1, int s2); // reserved; kept for symmetry
    void stepMuxes_();
    void updateControls_();

    // Daisy HW
    daisy::DaisySeed  seed_;
    daisy::AudioHandle audio_;
    daisy::DacHandle   dac_;
    daisy::GPIO        led_; // Panel LED on D29

    // Select pins for both muxes (first: pots, second: attenuverters)
    daisy::GPIO m1_s0_, m1_s1_, m1_s2_; // MUX1 (IC8): D6, D8, D9
    daisy::GPIO m2_s0_, m2_s1_, m2_s2_; // MUX2 (IC9): D1, D2, D3

    // ADC channel indices
    int adc_cv1_ = -1; // A0 Timbre CV
    int adc_cv2_ = -1; // A1 Pitch  CV (V/Oct)
    int adc_cv3_ = -1; // A2 Spread CV
    int adc_cv4_ = -1; // A3 Morph  CV
    int adc_m1_  = -1; // A5 MUX1 COM (raw pots)
    int adc_m2_  = -1; // A6 MUX2 COM (attenuverters)

    // Mux scan state
    uint8_t m1_curr_ = 0, m1_prev_ = 0;
    uint8_t m2_curr_ = 0, m2_prev_ = 0;
    float   m1_val_[8] = {0}, m2_val_[8] = {0};

    // Smoothers per signal (first 4 knob mux channels, first 4 att mux channels, 4 CV)
    struct OnePole { float y=0.f, a=0.12f; inline float proc(float x){ y += a*(x - y); return y; } };
    OnePole sm_knob_[4];
    OnePole sm_att_[4];
    OnePole sm_cv_[4];

    // Tunables
    Tunables tun_;
    float   sr_        = 48000.f;
    size_t  blocksize_ = 48;

    // Aggregated controls
    Controls controls_;
};

} // namespace hw
