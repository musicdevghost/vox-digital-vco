#include "daisy_seed.h"
#include <math.h>
#include "SelectedCore.hpp" // lives in ../src/dsp (paths added in Makefile)

// ---- Build-time macros (overridable from Makefile) ---------------------------
#ifndef VM_USE_SHARED_CORE
#define VM_USE_SHARED_CORE 1
#endif

#ifndef VM_CORE_TYPE
#define VM_CORE_TYPE vm::SelectedCore
#endif

#ifndef VM_CORE_PROCESS_NAME
#define VM_CORE_PROCESS_NAME processBlock
#endif

#ifndef VM_CORE_HAS_INPUTS
// 1 => expects (inL,inR,outL,outR,n), 0 => expects (outL,outR,n)
#define VM_CORE_HAS_INPUTS 1
#endif

#ifndef VM_SR
#define VM_SR 48000
#endif

#ifndef VM_BLOCKSIZE
#define VM_BLOCKSIZE 48
#endif

#ifndef VM_USE_DSP_PATH
#define VM_USE_DSP_PATH 1
#endif

#ifndef VM_USE_ADCS
#define VM_USE_ADCS 0
#endif

namespace ds = daisy;

// ------------------------- Parameters -----------------------------------------
struct Params {
    float pitch  = 0.0f;
    float timbre = 0.5f;
    float morph  = 0.25f;
    float spread = 0.0f;
} g_params;

// ------------------------- Soft saturation helper -----------------------------
static inline float softsat(float x, float drive)
{
    float g = 1.0f + drive * 0.6f;
    return tanhf(x * g);
}

// ------------------------- Daisy globals --------------------------------------
static ds::DaisySeed hw;

#if VM_USE_SHARED_CORE
static VM_CORE_TYPE core;
#endif

// ------------------------- Audio callback -------------------------------------
static void AudioCb(ds::AudioHandle::InputBuffer in,
                    ds::AudioHandle::OutputBuffer out,
                    size_t n)
{
#if VM_USE_DSP_PATH
  #if VM_USE_SHARED_CORE
    #if VM_CORE_HAS_INPUTS
      core.VM_CORE_PROCESS_NAME(in[0], in[1], out[0], out[1], n);
    #else
      core.VM_CORE_PROCESS_NAME(out[0], out[1], n);
    #endif
  #else
    for(size_t i = 0; i < n; ++i)
    {
        const float d = g_params.morph; // gentle "alive" shaping
        out[0][i] = softsat(in[0][i], d) * 0.8f;
        out[1][i] = softsat(in[1][i], d) * 0.8f;
    }
  #endif
#else
    for(size_t i = 0; i < n; ++i)
    {
        out[0][i] = in[0][i];
        out[1][i] = in[1][i];
    }
#endif
}

// ------------------------- Entry ----------------------------------------------
int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(ds::SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(VM_BLOCKSIZE);

#if VM_USE_SHARED_CORE
  #ifdef VM_CORE_HAS_INIT
   #if VM_CORE_HAS_INIT
    core.init(static_cast<float>(VM_SR));
   #endif
  #endif
#endif

    hw.StartAudio(AudioCb);
    while(1) { }
}
