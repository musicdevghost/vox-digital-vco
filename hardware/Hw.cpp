#include "Hw.hpp"

// Use the shared parameter helpers
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

    // Audio @ 48k, fixed blocksize (use Seed helpers in current libDaisy)
    seed_.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    seed_.SetAudioBlockSize(blocksize_);
    audio_ = seed_.audio_handle;

    // Panel LED on D29 (seed namespace)
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
    initMux_(0, 0, 0); // reserved, ensures symmetry

    // Smoothing alphas
    for (auto& s : sm_knob_) { s.a = tun_.smoothAlpha; }
    for (auto& s : sm_att_)  { s.a = tun_.smoothAlpha; }
    for (auto& s : sm_cv_)   { s.a = tun_.smoothAlpha; }
}

void Hw::startAudio(void (*cb)(AudioHandle::InputBuffer,
                               AudioHandle::OutputBuffer,
                               size_t))
{
    // Prefer Seed helper in current libDaisy
    seed_.StartAudio(cb);
}

void Hw::initAdc_()
{
    // A0..A3 (direct CVs), A5 (MUX1 COM), A6 (MUX2 COM) — seed namespace pins
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
    m1_s0_.Init(D6, GPIO::Mode::OUTPUT);
    m1_s1_.Init(D8, GPIO::Mode::OUTPUT);
    m1_s2_.Init(D9, GPIO::Mode::OUTPUT);

    // MUX2 (IC9): select pins D1, D2, D3
    m2_s0_.Init(D1, GPIO::Mode::OUTPUT);
    m2_s1_.Init(D2, GPIO::Mode::OUTPUT);
    m2_s2_.Init(D3, GPIO::Mode::OUTPUT);

    // Prime both muxes to channel 0; pipeline scheme uses prev/curr
    m1_curr_ = m1_prev_ = 0u;
    m2_curr_ = m2_prev_ = 0u;
    set4051Pins(m1_s0_, m1_s1_, m1_s2_, m1_curr_);
    set4051Pins(m2_s0_, m2_s1_, m2_s2_, m2_curr_);
}

void Hw::initMux_(int, int, int)
{
    // Reserved — already set up in initAdc_().
}

void Hw::stepMuxes_()
{
    // Read CURRENT ADC value which corresponds to the PREVIOUS select lines
    // (one-block latency after channel select update)
    const float m1_read = seed_.adc.GetFloat(adc_m1_);
    const float m2_read = seed_.adc.GetFloat(adc_m2_);

    m1_val_[m1_prev_] = m1_read;
    m2_val_[m2_prev_] = m2_read;

    // Advance prev indices
    m1_prev_ = m1_curr_;
    m2_prev_ = m2_curr_;

    // Select next channels (we only use 0..3 but keep wrap at 0..7)
    m1_curr_ = (m1_curr_ + 1u) & 0x7u;
    m2_curr_ = (m2_curr_ + 1u) & 0x7u;

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

    // Direct CVs (normalized 0..1 from Daisy)
    const float cv1 = sm_cv_[0].proc(seed_.adc.GetFloat(adc_cv1_)); // Timbre
    const float cv2 = sm_cv_[1].proc(seed_.adc.GetFloat(adc_cv2_)); // Pitch (V/Oct)
    const float cv3 = sm_cv_[2].proc(seed_.adc.GetFloat(adc_cv3_)); // Spread
    const float cv4 = sm_cv_[3].proc(seed_.adc.GetFloat(adc_cv4_)); // Morph

    // MUX1 raw pots (0..1). We only use channels 0..3.
    const float k_filter = sm_knob_[0].proc(m1_val_[0]); // FILTER/Tone
    const float k_size   = sm_knob_[1].proc(m1_val_[1]); // SIZE/Pitch
    const float k_feedb  = sm_knob_[2].proc(m1_val_[2]); // FEEDB/Morph
    const float k_diff   = sm_knob_[3].proc(m1_val_[3]); // DIFF/Spread

    // MUX2 attenuverters (map 0..1 → -1..+1)
    const float at_filter = sm_att_[0].proc(ParamMap::bipolar(m2_val_[0]));
    const float at_size   = sm_att_[1].proc(ParamMap::bipolar(m2_val_[1]));
    const float at_feedb  = sm_att_[2].proc(ParamMap::bipolar(m2_val_[2]));
    const float at_diff   = sm_att_[3].proc(ParamMap::bipolar(m2_val_[3]));

    // Mixer: knob + cv * attenuverter, then clamp to 0..1
    controls_.pitch  = ParamMap::clamp01(k_size   + cv2 * at_size);
    controls_.timbre = ParamMap::clamp01(k_filter + cv1 * at_filter);
    controls_.morph  = ParamMap::clamp01(k_feedb  + cv4 * at_feedb);
    controls_.spread = ParamMap::clamp01(k_diff   + cv3 * at_diff);

    // Expose 1V/Oct volts from CV2 path (default assumes 0..10V full-scale)
    controls_.pitchVolts = ParamMap::normToVolts(cv2);
}

void Hw::writeEnv(float env01)
{
    // Clamp to 0..1
    float e = (env01 < 0.f) ? 0.f : (env01 > 1.f ? 1.f : env01);

    // LED with simple hysteresis to avoid chatter
    static float latched = 0.f;
    const float th_on  = 0.08f;
    const float th_off = 0.05f;
    if (latched == 0.f && e > th_on)  latched = 1.f;
    if (latched == 1.f && e < th_off) latched = 0.f;
    led_.Write(latched > 0.5f);

    // DAC 0..1 → 12-bit code
    const uint16_t code = static_cast<uint16_t>(e * 4095.0f + 0.5f);
    dac_.WriteValue(DacHandle::Channel::ONE, code);
}

} // namespace hw
