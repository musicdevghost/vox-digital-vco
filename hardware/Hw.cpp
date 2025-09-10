#include "Hw.hpp"
#include "dsp/shared/ParamMap.hpp"

using namespace daisy;
using namespace daisy::seed;

namespace hw {

static inline void set4051Pins(GPIO& s0, GPIO& s1, GPIO& s2, uint8_t ch)
{
    s0.Write((ch >> 0) & 0x1);
    s1.Write((ch >> 1) & 0x1);
    s2.Write((ch >> 2) & 0x1);
}

void Hw::init(float sr, size_t blocksize, const Tunables& t)
{
    tun_       = t;
    sr_        = sr;
    blocksize_ = blocksize;

    // Seed bring-up
    seed_.Configure();
    seed_.Init();

    // Audio @ 48k, fixed blocksize
    seed_.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    seed_.SetAudioBlockSize(blocksize_);
    audio_ = seed_.audio_handle;

    // Panel LED on D29
    led_.Init(D29, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);

    // DAC OUT1 (PA4) — 12-bit, polling, one channel
    DacHandle::Config dcfg;
    dcfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dcfg.buff_state = DacHandle::BufferState::ENABLED;
    dcfg.mode       = DacHandle::Mode::POLLING;
    dcfg.chn        = DacHandle::Channel::ONE;
    dac_.Init(dcfg);

    // ADC + mux setup
    initAdc_();
    initMux_(0, 0, 0); // reserved

    // Smoothing: prefer time-constant (ms); fall back to alpha if provided
    auto init_smoother = [&](vm::OnePole& s){
        if (tun_.smoothMs > 0.f) s.setTauMs(sr_, tun_.smoothMs);
        else if (tun_.smoothAlpha >= 0.f) s.setAlpha(tun_.smoothAlpha);
        else s.setTauMs(sr_, 8.f);
    };
    for (auto& s : sm_knob_) init_smoother(s);
    for (auto& s : sm_att_)  init_smoother(s);
    for (auto& s : sm_cv_)   init_smoother(s);
}

void Hw::startAudio(void (*cb)(AudioHandle::InputBuffer,
                               AudioHandle::OutputBuffer,
                               size_t))
{
    seed_.StartAudio(cb);
}

void Hw::initAdc_()
{
    // A0..A3 (direct CVs), A5 (MUX1 COM), A6 (MUX2 COM)
    AdcChannelConfig adc_cfg[6];
    adc_cfg[0].InitSingle(A0); // CV1  → Timbre
    adc_cfg[1].InitSingle(A1); // CV2  → Pitch (V/Oct)
    adc_cfg[2].InitSingle(A2); // CV3  → Spread
    adc_cfg[3].InitSingle(A3); // CV4  → Morph
    adc_cfg[4].InitSingle(A5); // MUX1 → Pots
    adc_cfg[5].InitSingle(A6); // MUX2 → Attenuverters

    seed_.adc.Init(adc_cfg, 6);
    seed_.adc.Start();

    adc_cv1_ = 0;
    adc_cv2_ = 1;
    adc_cv3_ = 2;
    adc_cv4_ = 3;
    adc_m1_  = 4;
    adc_m2_  = 5;

    // MUX1 (IC8): select pins D6, D8, D9
    m1_s0_.Init(D6, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);
    m1_s1_.Init(D8, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);
    m1_s2_.Init(D9, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);

    // MUX2 (IC9): select pins D1, D2, D3
    m2_s0_.Init(D1, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);
    m2_s1_.Init(D2, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);
    m2_s2_.Init(D3, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL, GPIO::Speed::LOW);

    m1_prev_ = m1_curr_ = 0;
    m2_prev_ = m2_curr_ = 0;

    // Pre-clear mux readbacks
    for (int i = 0; i < 8; ++i)
    {
        m1_val_[i] = 0.f;
        m2_val_[i] = 0.f;
    }
}

// NOTE: Match Hw.hpp signature (int,int,int)
void Hw::initMux_(int m1, int m2, int /*reserved*/)
{
    set4051Pins(m1_s0_, m1_s1_, m1_s2_, static_cast<uint8_t>(m1 & 0x7));
    set4051Pins(m2_s0_, m2_s1_, m2_s2_, static_cast<uint8_t>(m2 & 0x7));
    m1_curr_ = static_cast<uint8_t>(m1 & 0x7);
    m2_curr_ = static_cast<uint8_t>(m2 & 0x7);
    m1_prev_ = m1_curr_;
    m2_prev_ = m2_curr_;
}

void Hw::stepMuxes_()
{
    // Read CURRENT ADC value which corresponds to the PREVIOUS select lines
    const float m1_read = seed_.adc.GetFloat(adc_m1_);
    const float m2_read = seed_.adc.GetFloat(adc_m2_);

    m1_val_[m1_prev_] = m1_read;
    m2_val_[m2_prev_] = m2_read;

    // Advance prev indices
    m1_prev_ = m1_curr_;
    m2_prev_ = m2_curr_;

    // Select next channels — use only 0..3 to reduce control latency
    m1_curr_ = (m1_curr_ + 1u) & 0x3u;
    m2_curr_ = (m2_curr_ + 1u) & 0x3u;

    set4051Pins(m1_s0_, m1_s1_, m1_s2_, m1_curr_);
    set4051Pins(m2_s0_, m2_s1_, m2_s2_, m2_curr_);
}

void Hw::poll()
{
    stepMuxes_();
    updateControls_();
}

void Hw::updateControls_()
{
    using vm::ParamMap;

    // --- Direct CVs (normalized 0..1 from Daisy ADC) ---
    // Timbre/Spread/Morph as bipolar (−1..+1) after smoothing; assumes front-end scales ±5V → 0..1.
    const float cv1_b = ParamMap::bipolar(sm_cv_[0].proc(seed_.adc.GetFloat(adc_cv1_))); // Timbre
    const float cv3_b = ParamMap::bipolar(sm_cv_[2].proc(seed_.adc.GetFloat(adc_cv3_))); // Spread
    const float cv4_b = ParamMap::bipolar(sm_cv_[3].proc(seed_.adc.GetFloat(adc_cv4_))); // Morph

    // Pitch as 1V/Oct: convert normalized 0..1 → 0..10V (or your configured full-scale).
    const float pitch_volts = ParamMap::normToVolts(sm_cv_[1].proc(seed_.adc.GetFloat(adc_cv2_)));

    // --- MUX1 raw pots (0..1). We use channels 0..3. ---
    const float k_filter = sm_knob_[0].proc(m1_val_[0]); // FILTER/Tone
    const float k_size   = sm_knob_[1].proc(m1_val_[1]); // SIZE/Pitch (base tune)
    const float k_feedb  = sm_knob_[2].proc(m1_val_[2]); // FEEDB/Morph
    const float k_diff   = sm_knob_[3].proc(m1_val_[3]); // DIFF/Spread

    // --- MUX2 attenuverters (map 0..1 → -1..+1) ---
    const float at_filter = sm_att_[0].proc(ParamMap::bipolar(m2_val_[0]));
    // Pitch attenuverter intentionally ignored for strict 1V/Oct tracking
    // const float at_size   = sm_att_[1].proc(ParamMap::bipolar(m2_val_[1]));
    const float at_feedb  = sm_att_[2].proc(ParamMap::bipolar(m2_val_[2]));
    const float at_diff   = sm_att_[3].proc(ParamMap::bipolar(m2_val_[3]));

    // --- Mixer ---
    // Timbre/Spread/Morph: knob + (bipolar CV * attenuverter) → clamp 0..1.
    controls_.timbre = ParamMap::clamp01(k_filter + cv1_b * at_filter);
    controls_.spread = ParamMap::clamp01(k_diff   + cv3_b * at_diff);
    controls_.morph  = ParamMap::clamp01(k_feedb  + cv4_b * at_feedb);

    // Pitch: base tune from knob (0..1), strict 1V/Oct from pitch_volts
    controls_.pitch      = ParamMap::clamp01(k_size);   // base tune only
    controls_.pitchVolts = pitch_volts;                 // 1V/Oct path for DSP
}

// Envelope/DAC writer used by main.cpp.
// Accepts normalized 0..1 value; clamps and writes to PA4 (12-bit).
void Hw::writeEnv(float norm01)
{
    const float v = vm::ParamMap::clamp01(norm01);
    const uint16_t code = static_cast<uint16_t>(v * 4095.0f + 0.5f);
    dac_.WriteValue(DacHandle::Channel::ONE, code);
}

} // namespace hw
