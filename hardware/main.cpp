// VOX — Hardware main.cpp (Phase B parity with Rack sim, using shared VoxCore)
#include "daisy_seed.h"
#include <cmath>
#include "../src/dsp/VoxCore.hpp"

using namespace daisy;
using namespace daisy::seed;
using namespace vm::vox;

// ---------- MUX1 (pots): COM=A5, selects D5/D6/D7 ----------
#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7
// Pots (fixed)
#define CH_PITCH   0
#define CH_MORPH   1
#define CH_SPREAD  2
#define CH_TIMBRE  3

// ---------- MUX2 (attenuverters): COM=A6, selects D0/D1/D2 ----------
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D0
#define MUX2_SEL1     D1
#define MUX2_SEL2     D2
#define AT_CH_TIMBRE 0
#define AT_CH_PITCH  1
#define AT_CH_SPREAD 2
#define AT_CH_MORPH  3

static DaisySeed hw;
static AdcChannelConfig adc_cfg[7];

static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; }
static inline float apply_cv_att(float knob01, float cv01, float at01) {
    const float av = uni_to_bi(at01);
    const float mod = av * (cv01 - 0.5f);
    return clamp01(knob01 + mod);
}
static inline float quantize12(float x01) {
    if(x01 < 0.f) x01 = 0.f;
    if(x01 > 1.f) x01 = 1.f;
    const int q = int(lroundf(x01 * 4095.f));
    return float(q) / 4095.f;
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ADC: direct CVs A0..A3 (invert), A4 (unused), muxes for pots and attenuverters
    adc_cfg[0].InitSingle(A0); // TIMBRE (invert)
    adc_cfg[1].InitSingle(A1); // PITCH  (invert)
    adc_cfg[2].InitSingle(A2); // SPREAD (invert)
    adc_cfg[3].InitSingle(A3); // MORPH  (invert)
    adc_cfg[4].InitSingle(A4); // SSYNC (unused in Phase B)
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
        static int idx = kBlock;

        if(idx >= kBlock) {
            idx = 0;
            // Pots
            const float kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
            const float kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
            const float kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
            const float kTimbre = hw.adc.GetMuxFloat(5, CH_TIMBRE);
            // Atten
            const float at_timbre = hw.adc.GetMuxFloat(6, AT_CH_TIMBRE);
            const float at_pitch  = hw.adc.GetMuxFloat(6, AT_CH_PITCH);
            const float at_spread = hw.adc.GetMuxFloat(6, AT_CH_SPREAD);
            const float at_morph  = hw.adc.GetMuxFloat(6, AT_CH_MORPH);
            // Direct CVs (invert)
            const float CV_Timbre = 1.f - hw.adc.GetFloat(0);
            const float CV_Pitch  = 1.f - hw.adc.GetFloat(1);
            const float CV_Spread = 1.f - hw.adc.GetFloat(2);
            const float CV_Morph  = 1.f - hw.adc.GetFloat(3);

            Controls c;
            c.pitchKnob01 = quantize12(apply_cv_att(kPitch,  CV_Pitch,  at_pitch));
            c.morph01     = quantize12(apply_cv_att(kMorph,  CV_Morph,  at_morph));
            c.timbre01    = quantize12(apply_cv_att(kTimbre, CV_Timbre, at_timbre));
            c.spread01    = quantize12(apply_cv_att(kSpread, CV_Spread, at_spread));

            Mods m;
            static CoreParams p = {48000.0, 440.0, 5};
            static VoxCore core_local;
            static bool init = false;
            if(!init) { core_local.setup(p); init = true; }
            static State s;
            core_local.processBlock(p, c, m, s, L, R, kBlock);
        }
        for(size_t i = 0; i < size; i += 2) {
            out[i+0] = L[idx];
            out[i+1] = R[idx];
            idx++;
        }
    };

    hw.StartAudio(cb);
    while(1) { System::Delay(10); }
}
