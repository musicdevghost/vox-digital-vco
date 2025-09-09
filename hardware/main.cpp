#include "daisy_seed.h"
#include "daisysp.h"
#include "SelectedCore.hpp"
#include "pins.hpp"
#include <cmath>

#if __has_include("per/pwmout.h")
  #include "per/pwmout.h"
  #define VM_HAVE_PWMOUT 1
#else
  #define VM_HAVE_PWMOUT 0
#endif

using namespace daisy;
using namespace daisysp;

#ifndef HW_TEST
#define HW_TEST 0
#endif
#ifndef USE_PWM_CVOUT
#define USE_PWM_CVOUT 1           // used only if VM_HAVE_PWMOUT == 1
#endif
#ifndef VM_CORE_HAS_INPUTS
#define VM_CORE_HAS_INPUTS 1
#endif
#ifndef VM_CORE_PROCESS_NAME
#define VM_CORE_PROCESS_NAME processBlock
#endif

static constexpr float  kSr    = 48000.0f;
static constexpr size_t kBlock = 48;

static inline float clamp01(float v){ return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }
static inline float cv_bipolar(float v01){ float x = v01 * 2.f - 1.f; return (x<-1.f)?-1.f:((x>1.f)?1.f:x); }

struct SmoothLP { float y=0.f; float proc(float x,float a){ y += a*(x-y); return y; } };

DaisySeed           hw;
static const int    kAdcCount = 8;
AdcChannelConfig    adc_cfg[kAdcCount];

GPIO                panelLed;
#if USE_PWM_CVOUT && VM_HAVE_PWMOUT
PwmOut              cvLed;
#endif

vm::SelectedCore    core;

SmoothLP sm_size, sm_tone, sm_feedb, sm_diff;
SmoothLP sm_cv_tim, sm_cv_pitch, sm_cv_spread, sm_cv_morph;

static float pepperEnv = 0.f;

static void AudioCb(AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    n)
{
    const float a = 0.12f;

    // Pots (0..1)
    float size_k  = sm_size .proc(hw.adc.GetFloat(0), a);
    float tone_k  = sm_tone .proc(hw.adc.GetFloat(1), a);
    float feedb_k = sm_feedb.proc(hw.adc.GetFloat(2), a);
    float diff_k  = sm_diff .proc(hw.adc.GetFloat(3), a);

    // CVs (post-attenuverter → bipolar)
    float cv_tim_b = sm_cv_tim   .proc(cv_bipolar(hw.adc.GetFloat(4)), a); // CV1
    float cv_pit_b = sm_cv_pitch .proc(cv_bipolar(hw.adc.GetFloat(5)), a); // CV2 (V/Oct)
    float cv_spr_b = sm_cv_spread.proc(cv_bipolar(hw.adc.GetFloat(6)), a); // CV3
    float cv_mor_b = sm_cv_morph .proc(cv_bipolar(hw.adc.GetFloat(7)), a); // CV4

    // Build Params (your core exposes exactly 4 params)
    vm::IDspCore::Params p{};
    p.pitch  = clamp01(size_k  + cv_pit_b);         // coarse + V/Oct CV (normalized)
    p.timbre = clamp01((tone_k * 0.5f) + 0.5f + cv_tim_b * 0.5f); // keep mid-range usable
    p.morph  = clamp01(feedb_k + cv_mor_b);
    p.spread = clamp01(diff_k  + cv_spr_b);

    core.setParams(p);

#if VM_CORE_HAS_INPUTS
    core.VM_CORE_PROCESS_NAME(in[0], in[1], out[0], out[1], (int)n);
#else
    core.VM_CORE_PROCESS_NAME(out[0], out[1], (int)n);
#endif

    // ENV follower (same shape as your Rack template)
    float peak = 0.f;
    for(size_t i = 0; i < n; ++i)
        peak = std::max(peak, std::max(std::fabs(out[0][i]), std::fabs(out[1][i])));
    float amp = peak / 5.f;
    const float atk = 1.f - std::exp(-1.f / (0.008f * kSr));
    const float rel = 1.f - std::exp(-1.f / (0.160f * kSr));
    float acoef = (amp > pepperEnv) ? atk : rel;
    pepperEnv += acoef * (amp - pepperEnv);
    float env01 = clamp01(pepperEnv);

    panelLed.Write(env01 > 0.05f);
#if USE_PWM_CVOUT && VM_HAVE_PWMOUT
    cvLed.SetDuty(env01);
#endif
}

static void InitAdc()
{
    adc_cfg[0].InitSingle(hwpins::ADC_SIZE);
    adc_cfg[1].InitSingle(hwpins::ADC_TONE);
    adc_cfg[2].InitSingle(hwpins::ADC_FEEDB);
    adc_cfg[3].InitSingle(hwpins::ADC_DIFF);
    adc_cfg[4].InitSingle(hwpins::ADC_CV_TIMBRE); // CV1
    adc_cfg[5].InitSingle(hwpins::ADC_CV_PITCH);  // CV2 (V/Oct)
    adc_cfg[6].InitSingle(hwpins::ADC_CV_SPREAD); // CV3
    adc_cfg[7].InitSingle(hwpins::ADC_CV_MORPH);  // CV4
    hw.adc.Init(adc_cfg, kAdcCount);
    hw.adc.Start();
}

static void InitIo()
{
    panelLed.Init(hwpins::PANEL_LED, GPIO::Mode::OUTPUT, GPIO::Pull::NOPULL);

#if USE_PWM_CVOUT && VM_HAVE_PWMOUT
    Pin cvpin =
    #ifdef HWPINS_D_CVLED
        hw.GetPin(HWPINS_D_CVLED);
    #else
        hwpins::CVLED_PIN_DEFAULT;
    #endif

    PwmOut::Config c;
    c.pin  = cvpin;
    c.freq = 48000;   // high PWM to minimize ripple (assumes RC on board)
    c.duty = 0.0f;
    cvLed.Init(c);
    cvLed.Set(0.0f);
#endif
}

int main(void)
{
    hw.Configure();
    hw.Init();

    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(kBlock);

    InitAdc();
    InitIo();

    // Board-level audio start (older libDaisy API)
    hw.StartAudio(AudioCb);

#if HW_TEST
    for(;;){ panelLed.Toggle(); System::Delay(150); }
#else
    for(;;){ System::Delay(1000); }
#endif
}
