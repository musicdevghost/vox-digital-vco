#include "daisy_seed.h"
#include "Hw.hpp"
#include <cmath>

#include "dsp/IDspCore.hpp"
#include "dsp/SelectedCore.hpp"
#include "dsp/shared/EnvFollower.hpp"

using namespace daisy;

static hw::Hw g_hw;

#ifndef VM_SR
#define VM_SR 48000
#endif

#ifndef VM_BLOCKSIZE
#define VM_BLOCKSIZE 48
#endif

#ifndef VM_CORE_HAS_INPUTS
#define VM_CORE_HAS_INPUTS 1
#endif

#ifndef HW_TEST
#define HW_TEST 0
#endif

// ===== Test mode (simple 440Hz, LED blink, DAC ramp) =====
#if HW_TEST
static float phs  = 0.f;
static float dacv = 0.f;
static int   blink = 0;

static void TestAudio(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    (void)in;
    const float sr  = float(VM_SR);
    const float inc = 440.f / sr;

    for (size_t i = 0; i < n; ++i)
    {
        phs += inc;
        if (phs >= 1.f) phs -= 1.f;
        const float s = 0.2f * sinf(2.f * 3.14159265f * phs);
        out[0][i] = s;
        out[1][i] = s;
    }

    // Blink ~2 Hz
    static int count = 0;
    if (++count > int(sr / n / 2)) { count = 0; blink ^= 1; }
    g_hw.seed()->SetLed(blink);

    // DAC ramp 0..1
    dacv += 0.0025f;
    if (dacv > 1.f) dacv = 0.f;
    g_hw.writeEnv(dacv);
}
#endif // HW_TEST

// ===== Production audio callback =====
namespace {
vm::SelectedCore core;        // alias set in dsp/SelectedCore.hpp
vm::EnvFollower  env;         // AR envelope follower
#if !VM_CORE_HAS_INPUTS
static float     zeroL[VM_BLOCKSIZE] = {0.f};
static float     zeroR[VM_BLOCKSIZE] = {0.f};
#endif
} // anon

static void AudioCb(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    // 1) Advance hardware scanning/mix controls
    g_hw.poll();

    // 2) Push params to core (using your IDspCore::Params)
    const auto c = g_hw.controls();
    vm::IDspCore::Params p;
    p.pitch       = c.pitch;
    p.timbre      = c.timbre;
    p.morph       = c.morph;
    p.spread      = c.spread;
    p.pitchVolts  = static_cast<double>(c.pitchVolts);
    p.macros[0]   = p.macros[1] = p.macros[2] = p.macros[3] = 0.0; // not used here
    p.fmCablePresent = false; // VoxVcoCore can infer if needed
    core.setParams(p);

    // 3) Prepare pointers (support cores without inputs)
#if VM_CORE_HAS_INPUTS
    const float* inL = in[0];
    const float* inR = in[1];
#else
    const float* inL = zeroL;
    const float* inR = zeroR;
#endif

    // 4) Process with your signature
    core.processBlock(inL, inR, out[0], out[1], static_cast<int>(n));

    // 5) Envelope from outputs (8 ms attack / 160 ms release)
    const float e = env.processBlockRmsStereo(out[0], out[1], n);
    g_hw.writeEnv(e);
}

int main(void)
{
    const float  sr = float(VM_SR);
    const size_t bs = size_t(VM_BLOCKSIZE);

    hw::Tunables t;
    t.smoothAlpha = 0.12f;
    t.envAtkMs    = 8.0f;
    t.envRelMs    = 160.0f;

    g_hw.init(sr, bs, t);

#if HW_TEST
    g_hw.startAudio(TestAudio);
#else
    core.init(double(sr));
    env.setupMs(sr, t.envAtkMs, t.envRelMs);
    g_hw.startAudio(AudioCb);
#endif

    while (1) { System::Delay(100); }
}
