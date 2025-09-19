// VOX — Hardware main.cpp (Shared core, audio-rate FM/HSYNC/SSYNC, interleaved fix)
#include "daisy_seed.h"
#include <cmath>
#include "../src/dsp/VoxCore.hpp"
#include "../src/hal/ControlMap.hpp"

using namespace daisy;
using namespace daisy::seed;
using namespace vm::vox;
using namespace vm::vox::glue;

// ---- MUX config (your mapping) ----
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

// ---- Presence detection thresholds (file-scope so lambda can see them) ----
static constexpr float HSYNC_ENV_ATTACK  = 0.01f;
static constexpr float HSYNC_ENV_RELEASE = 0.001f;
static constexpr float HSYNC_ENV_THRESH  = 0.02f;  // ~ -34 dBFS

static constexpr float FM_ENV_ATTACK     = 0.01f;
static constexpr float FM_ENV_RELEASE    = 0.001f;
static constexpr float FM_ENV_THRESH     = 0.02f;  // ~ -34 dBFS

static DaisySeed hw;
static AdcChannelConfig adc_cfg[7];

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ADC: direct CVs A0..A3 (invert), A4 soft sync gate, MUX for pots/attenuverters
    adc_cfg[0].InitSingle(A0); // TIMBRE (invert)
    adc_cfg[1].InitSingle(A1); // PITCH  (invert)
    adc_cfg[2].InitSingle(A2); // SPREAD (invert)
    adc_cfg[3].InitSingle(A3); // MORPH  (invert)
    adc_cfg[4].InitSingle(A4); // SSYNC gate (used as >0.5 => on)
    adc_cfg[5].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2); // pots
    adc_cfg[6].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2); // attenuverters
    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();

    // Shared core setup
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

        // Static env followers for presence detection
        static float envL = 0.f;
        static float envR = 0.f;

        if(idx >= kBlock) {
            idx = 0;

            // ---- Read controls once per tick ----
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

            // ---- Audio-rate signals (read first interleaved frame) ----
            // Interleaved input: in[0]=L0, in[1]=R0, in[2]=L1, ...
            float lin0 = 0.f, rin0 = 0.f;
            if(in) {
                lin0 = in[0];
                rin0 = (size >= 2) ? in[1] : 0.f;
            }

            // Presence detection with AR envs
            const float aL = (fabsf(lin0) > envL) ? HSYNC_ENV_ATTACK : HSYNC_ENV_RELEASE;
            envL = (1.f - aL) * envL + aL * fabsf(lin0);
            const bool hsync_on = envL > HSYNC_ENV_THRESH;

            const float aR = (fabsf(rin0) > envR) ? FM_ENV_ATTACK : FM_ENV_RELEASE;
            envR = (1.f - aR) * envR + aR * fabsf(rin0);
            const bool fm_on = envR > FM_ENV_THRESH;

            for(int i = 0; i < kBlock; ++i) {
                hsync[i] = hsync_on ? lin0 : 0.f;
                fm[i]    = fm_on    ? rin0 : 0.f;
                ssync[i] = ssGate;
            }

            Mods m;
            m.hsync     = hsync_on ? hsync : nullptr;
            m.fm        = fm_on    ? fm    : nullptr;
            m.ssync     = ssync;
            m.fmDepthHz = 440.0; // tune as needed

            static CoreParams p = {48000.0, 440.0, 5};
            static VoxCore core_local;
            static bool init = false;
            if(!init) { core_local.setup(p); init = true; }
            static State s;
            core_local.processBlock(p, c, m, s, L, R, kBlock);
        }

        // ---- Output interleaved ----
        for(size_t i = 0; i < size; i += 2) {
            out[i + 0] = L[idx];
            out[i + 1] = R[idx];
            idx++;
        }
    };

    hw.StartAudio(cb);
    while(1) { System::Delay(10); }
}
