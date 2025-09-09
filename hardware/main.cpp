#include "daisy_seed.h"
#include "daisysp.h"
#include "SelectedCore.hpp"
#include "pins.hpp"
#include <cmath>

using namespace daisy;
using namespace daisysp;

#ifndef VM_CORE_HAS_INPUTS
#define VM_CORE_HAS_INPUTS 1
#endif
#ifndef VM_CORE_PROCESS_NAME
#define VM_CORE_PROCESS_NAME processBlock
#endif
#ifndef HW_TEST
#define HW_TEST 0
#endif

// Rates / sizes
static constexpr float  kSr    = 48000.0f;
static constexpr size_t kBlock = 48;

static inline float clamp01(float v){ return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
static inline float bipolar(float v01){ float x = v01 * 2.f - 1.f; return (x < -1.f) ? -1.f : (x > 1.f ? 1.f : x); }

struct SmoothLP { float y=0.f; float proc(float x,float a){ y += a*(x-y); return y; } };

// Hardware
DaisySeed        hw;
GPIO             panelLed;

// ADC: 4 direct + 2 mux entries
static const int kAdcEntries = 6;
AdcChannelConfig adc_cfg[kAdcEntries];

// Smoothers
SmoothLP sm_cv_tim, sm_cv_pitch, sm_cv_spread, sm_cv_morph;
SmoothLP sm_p_size, sm_p_tone, sm_p_feedb, sm_p_diff;
SmoothLP sm_at_size, sm_at_tone, sm_at_feedb, sm_at_diff;

// DSP core
vm::SelectedCore core;

// ENV follower state
static float envState = 0.f;

// ----- Audio Callback -----
static void AudioCb(AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    n)
{
    const float a = 0.12f; // control smoothing

    // ---- Read CVs (direct, 0..1) then map to bipolar for modulation ----
    float cv_tim_b = sm_cv_tim   .proc(bipolar(hw.adc.GetFloat(0)), a); // CV1 → Timbre
    float cv_pit_b = sm_cv_pitch .proc(bipolar(hw.adc.GetFloat(1)), a); // CV2 → Pitch
    float cv_spr_b = sm_cv_spread.proc(bipolar(hw.adc.GetFloat(2)), a); // CV3 → Spread
    float cv_mor_b = sm_cv_morph .proc(bipolar(hw.adc.GetFloat(3)), a); // CV4 → Morph

    // ---- Read MUX 1 (pots) and MUX 2 (attenuverters) ----
    // Our adc config order: [0..3]=direct CVs, [4]=MUX1, [5]=MUX2
    // MUX channel indices from schematic: 0=FILTER,1=SIZE,2=FEEDB,3=DIFF
    float pot_filter = sm_p_tone .proc(hw.adc.GetMuxFloat(4, 0), a); // Tone
    float pot_size   = sm_p_size .proc(hw.adc.GetMuxFloat(4, 1), a); // Pitch coarse
    float pot_feedb  = sm_p_feedb.proc(hw.adc.GetMuxFloat(4, 2), a); // Morph
    float pot_diff   = sm_p_diff .proc(hw.adc.GetMuxFloat(4, 3), a); // Spread

    // Attenuverters as bipolar (−1..+1) so they can invert CVs
    float at_filter_b = sm_at_tone .proc(bipolar(hw.adc.GetMuxFloat(5, 0)), a); // Timbre CV amt
    float at_size_b   = sm_at_size .proc(bipolar(hw.adc.GetMuxFloat(5, 1)), a); // Pitch CV amt
    float at_feedb_b  = sm_at_feedb.proc(bipolar(hw.adc.GetMuxFloat(5, 2)), a); // Morph CV amt
    float at_diff_b   = sm_at_diff .proc(bipolar(hw.adc.GetMuxFloat(5, 3)), a); // Spread CV amt

    // ---- Build params: knob + (CV * attenuverter) ----
    vm::IDspCore::Params p{};
    p.pitch  = clamp01(pot_size   + (cv_pit_b * at_size_b));
    p.timbre = clamp01(pot_filter + (cv_tim_b * at_filter_b));
    p.morph  = clamp01(pot_feedb  + (cv_mor_b * at_feedb_b));
    p.spread = clamp01(pot_diff   + (cv_spr_b * at_diff_b));

    core.setParams(p);

#if VM_CORE_HAS_INPUTS
    core.VM_CORE_PROCESS_NAME(in[0], in[1], out[0], out[1], (int)n);
#else
    core.VM_CORE_PROCESS_NAME(out[0], out[1], (int)n);
#endif

    // ---- ENV follower (same feel as your Rack template) ----
    float peak = 0.f;
    for(size_t i = 0; i < n; ++i)
        peak = fmaxf(peak, fmaxf(fabsf(out[0][i]), fabsf(out[1][i])));
    float amp = peak / 5.f;
    const float atk = 1.f - expf(-1.f / (0.008f * kSr));
    const float rel = 1.f - expf(-1.f / (0.160f * kSr));
    float acoef = (amp > envState) ? atk : rel;
    envState += acoef * (amp - envState);
    float env01 = clamp01(envState);

    // Panel LED
    panelLed.Write(env01 > 0.05f);

    // DAC OUT1: write 0..1 to full-scale (external stage lifts to 0..8V)
    uint16_t code = (uint16_t)(env01 * 4095.0f + 0.5f);
    hw.dac.WriteValue(DacHandle::Channel::ONE, code);
}

// ----- Init -----
static void InitAdc()
{
    // 0..3 : direct CVs
    adc_cfg[0].InitSingle(hwpins::ADC_CV_TIMBRE);
    adc_cfg[1].InitSingle(hwpins::ADC_CV_PITCH);
    adc_cfg[2].InitSingle(hwpins::ADC_CV_SPREAD);
    adc_cfg[3].InitSingle(hwpins::ADC_CV_MORPH);

    // 4 : MUX 1 (pots)
    adc_cfg[4].InitMux(hwpins::MUX1_COM, 8, hwpins::MUX1_S0, hwpins::MUX1_S1, hwpins::MUX1_S2);
    // 5 : MUX 2 (attenuverters)
    adc_cfg[5].InitMux(hwpins::MUX2_COM, 8, hwpins::MUX2_S0, hwpins::MUX2_S1, hwpins::MUX2_S2);

    hw.adc.Init(adc_cfg, kAdcEntries);
    hw.adc.Start();
}

static void InitIo()
{
    // Panel LED
    panelLed.Init(hwpins::PANEL_LED, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);

    // DAC OUT1 init (PA4)
    DacHandle::Config dcfg;
    dcfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dcfg.buff_state = DacHandle::BufferState::ENABLED;
    dcfg.mode       = DacHandle::Mode::POLLING; // simple, low-rate updates OK
    dcfg.chn        = DacHandle::Channel::BOTH; // we only use Channel ONE
    hw.dac.Init(dcfg);
    hw.dac.WriteValue(DacHandle::Channel::ONE, 0);
}

int main(void)
{
    hw.Configure();
    hw.Init();

    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(kBlock);

    InitAdc();
    InitIo();

    hw.StartAudio(AudioCb);

#if HW_TEST
    for(;;){ panelLed.Toggle(); System::Delay(150); }
#else
    for(;;){ System::Delay(1000); }
#endif
}
