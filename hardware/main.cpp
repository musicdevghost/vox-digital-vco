// VOX — Hardware main.cpp (Shared core, audio-rate FM/HSYNC/SSYNC, interleaved per-frame sampling)
#include "daisy_seed.h"
#include <cmath>
#include "../src/dsp/VoxCore.hpp"
#include "../src/hal/ControlMap.hpp"
#include "../src/shared/Config.hpp"

using namespace daisy;
using namespace daisy::seed;
using namespace vm::vox;
using namespace vm::vox::glue;

#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7
#define CH_PITCH   0
#define CH_MORPH   1
#define CH_SPREAD  2
#define CH_TIMBRE  3

#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D0
#define MUX2_SEL1     D1
#define MUX2_SEL2     D2
#define AT_CH_TIMBRE 0
#define AT_CH_PITCH  1
#define AT_CH_SPREAD 2
#define AT_CH_MORPH  3

static constexpr float HSYNC_ENV_ATTACK  = 0.01f;
static constexpr float HSYNC_ENV_RELEASE = 0.001f;
static constexpr float HSYNC_ENV_THRESH  = 0.02f;
static constexpr float FM_ENV_ATTACK     = 0.01f;
static constexpr float FM_ENV_RELEASE    = 0.001f;
static constexpr float FM_ENV_THRESH     = 0.02f;

static DaisySeed hw;
static AdcChannelConfig adc_cfg[7];

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    adc_cfg[0].InitSingle(A0);
    adc_cfg[1].InitSingle(A1);
    adc_cfg[2].InitSingle(A2);
    adc_cfg[3].InitSingle(A3);
    adc_cfg[4].InitSingle(A4);
    adc_cfg[5].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2);
    adc_cfg[6].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2);
    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();

    CoreParams params;
    params.sampleRate = 48000.0;
    VoxCore core;
    core.setup(params);
    State state;

    auto cb = [](AudioHandle::InterleavingInputBuffer in,
                 AudioHandle::InterleavingOutputBuffer out,
                 size_t size) {
        static constexpr int kBlock = 48;
        static float L[kBlock], R[kBlock];
        static float fm[kBlock], hsync[kBlock], ssync[kBlock];
        static int idx = kBlock;

        static float envL = 0.f, envR = 0.f;

        if(idx >= kBlock) {
            idx = 0;

            const float kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
            const float kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
            const float kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
            const float kTimbre = hw.adc.GetMuxFloat(5, CH_TIMBRE);

            const float at_timbre = hw.adc.GetMuxFloat(6, AT_CH_TIMBRE);
            const float at_pitch  = hw.adc.GetMuxFloat(6, AT_CH_PITCH);
            const float at_spread = hw.adc.GetMuxFloat(6, AT_CH_SPREAD);
            const float at_morph  = hw.adc.GetMuxFloat(6, AT_CH_MORPH);

            const float CV_Timbre = 1.f - hw.adc.GetFloat(0);
            const float CV_Pitch  = 1.f - hw.adc.GetFloat(1);
            const float CV_Spread = 1.f - hw.adc.GetFloat(2);
            const float CV_Morph  = 1.f - hw.adc.GetFloat(3);
            const float ss        = hw.adc.GetFloat(4);
            const float ssGate    = ss > 0.5f ? 1.f : 0.f;

            Controls c;
            c.pitchKnob01 = quantize12(apply_cv_att_hw(kPitch,  CV_Pitch,  at_pitch));
            c.morph01     = quantize12(apply_cv_att_hw(kMorph,  CV_Morph,  at_morph));
            c.timbre01    = quantize12(apply_cv_att_hw(kTimbre, CV_Timbre, at_timbre));
            c.spread01    = quantize12(apply_cv_att_hw(kSpread, CV_Spread, at_spread));

            // Fill audio-rate arrays from interleaved input per frame
            // size should be 2*kBlock when blocksize=48
            for (int i = 0; i < kBlock; ++i) {
                const int ii = i * 2;
                float lin = (ii + 0 < (int)size) ? in[ii + 0] : 0.f;
                float rin = (ii + 1 < (int)size) ? in[ii + 1] : 0.f;

                // Presence detection (per-sample)
                float aL = (fabsf(lin) > envL) ? HSYNC_ENV_ATTACK : HSYNC_ENV_RELEASE;
                envL = (1.f - aL) * envL + aL * fabsf(lin);
                float aR = (fabsf(rin) > envR) ? FM_ENV_ATTACK    : FM_ENV_RELEASE;
                envR = (1.f - aR) * envR + aR * fabsf(rin);

                hsync[i] = (envL > HSYNC_ENV_THRESH) ? lin : 0.f;
                fm[i]    = (envR > FM_ENV_THRESH)    ? rin : 0.f;
                ssync[i] = ssGate;
            }

            Mods m;
            m.hsync     = hsync;
            m.fm        = fm;
            m.ssync     = ssync;
            m.fmDepthHz = vm::vox::cfg::kDefaultFmDepthHz;

            static CoreParams p = {48000.0, 440.0, 5};
            static VoxCore core_local;
            static bool init = false;
            if(!init) { core_local.setup(p); init = true; }
            static State s;
            core_local.processBlock(p, c, m, s, L, R, kBlock);
        }

        for(size_t i = 0; i < size; i += 2) {
            out[i + 0] = L[idx];
            out[i + 1] = R[idx];
            idx++;
        }
    };

    hw.StartAudio(cb);
    while(1) { System::Delay(10); }
}
