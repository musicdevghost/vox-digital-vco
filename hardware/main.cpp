// =====================================================
// vox-digital-vco — Daisy Seed minimal hardware entry
// 48 kHz stereo, FZ/DN enabled, heap-free audio path
// =====================================================

#include "daisy_seed.h"
#include "daisysp.h"
#include <math.h> // tanhf

// Avoid 'using namespace daisy;' to prevent 'seed' ambiguity.
// Bring in only what we need:
using daisy::AudioHandle;
using daisy::DaisySeed;

static DaisySeed seed_hw;  // renamed from 'seed' to avoid conflict with namespace daisy::seed

#ifndef VM_USE_DSP_PATH
#define VM_USE_DSP_PATH 1  // 0 = dry passthrough, 1 = DSP demo
#endif

#ifdef VM_USE_SHARED_CORE
  #include "dsp/SelectedCore.hpp"
  static vm::SelectedCore core;
  struct VMParams { float pitch, timbre, morph, spread; };
  static VMParams g_params { 0.0f, 0.5f, 0.2f, 0.5f };
#endif

// Denormal protection: set FZ (bit24) + DN (bit25)
static inline void EnableFlushToZeroDefaultNaN()
{
#if defined(__ARM_FP)
    FPU->FPDSCR |= (1u << 24); // FZ
    FPU->FPDSCR |= (1u << 25); // DN
    uint32_t fpscr;
    asm volatile("vmrs %0, fpscr" : "=r"(fpscr));
    fpscr |= (1u << 24);
    fpscr |= (1u << 25);
    asm volatile("vmsr fpscr, %0" : : "r"(fpscr));
#endif
}

static inline float SoftShape(float x, float amt)
{
    const float sat = tanhf(x * (1.0f + 9.0f * amt));
    return (1.0f - amt) * x + amt * sat;
}

// Optional: wire ADCs later to map knobs to params:
//  AdcChannelConfig adc_cfg[4]; ... seed_hw.adc.Init(adc_cfg, 4); seed_hw.adc.Start();
static inline void ReadKnobs()
{
#ifdef VM_USE_SHARED_CORE
    // g_params.pitch  = ...
    // g_params.timbre = ...
    // g_params.morph  = ...
    // g_params.spread = ...
#endif
}

static void AudioCb(AudioHandle::InputBuffer  in,
                    AudioHandle::OutputBuffer out,
                    size_t                    frames)
{
    ReadKnobs();

#if defined(VM_USE_SHARED_CORE)
    core.setParams({g_params.pitch, g_params.timbre, g_params.morph, g_params.spread});
    core.processBlock(in[0], in[1], out[0], out[1], (int)frames);

#elif VM_USE_DSP_PATH
    const float makeup = 0.6f;
    const float morph  = 0.3f;
    const float spread = 0.5f;
    for(size_t i = 0; i < frames; ++i)
    {
        float l = in[0][i];
        float r = in[1][i];

        float ml = morph + 0.25f * (spread - 0.5f);
        float mr = morph - 0.25f * (spread - 0.5f);
        if(ml < 0.f) ml = 0.f;
        if(ml > 1.f) ml = 1.f;
        if(mr < 0.f) mr = 0.f;
        if(mr > 1.f) mr = 1.f;

        out[0][i] = SoftShape(l, ml) * makeup;
        out[1][i] = SoftShape(r, mr) * makeup;
    }
#else
    const float makeup = 0.98f;
    for(size_t i = 0; i < frames; ++i)
    {
        out[0][i] = in[0][i] * makeup;
        out[1][i] = in[1][i] * makeup;
    }
#endif
}

int main(void)
{
    EnableFlushToZeroDefaultNaN();

    seed_hw.Configure();
    seed_hw.Init();

    seed_hw.SetAudioBlockSize(48); // 1 ms @ 48 kHz
    const float sr = seed_hw.AudioSampleRate();
    (void)sr;

#ifdef VM_USE_SHARED_CORE
    core.init(sr);
    core.reset();
#endif

    seed_hw.StartAudio(AudioCb);
    for(;;) { }
}
