// Vox — stable mapping + per-sample swarm
// Spread = unison swarm (1..7 voices + detune); per-sample update (no block jumps)
// Pots/CVs/ATs exactly per mapping; SSYNC read but bypassed; A8=CV OUT, A7=LED.

#include "daisy_seed.h"
#include <cmath>

using namespace daisy;
using namespace daisy::seed;

DaisySeed hw;

// ---------- MUX1 (pots): COM=A5, selects D5/D6/D7 ----------
#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7

// Pots channel map (do not change)
#define CH_PITCH   0  // KNOB 0
#define CH_MORPH   1  // KNOB 1 (used here as volume)
#define CH_SPREAD  2  // KNOB 2 (swarm)
#define CH_TIMBRE  3  // KNOB 3 (wave morph)

// ---------- MUX2 (attenuverters): COM=A6, selects D1/D2/D3 ----------
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D1  // 4051 A (LSB)
#define MUX2_SEL1     D2  // 4051 B
#define MUX2_SEL2     D3  // 4051 C

// Attenuverter channels (do not change)
#define AT_CH_TIMBRE  0   // timbre attenuverter
#define AT_CH_PITCH   1   // pitch attenuverter
#define AT_CH_SPREAD  2   // spread attenuverter
#define AT_CH_MORPH   3   // morph attenuverter

// ---------- CV inputs (direct ADCs) (do not change) ----------
enum CvAdcIndex : int { CV_TIMBRE = 0, CV_VOCT = 1, CV_SPREAD = 2, CV_MORPH = 3, CV_SSYNC = 4 };
// A0..A3 are inverting; SSYNC (A4) is raw (active-low), read but not used to gate.
static constexpr float CV_POL_TIMBRE = -1.0f; // A0
static constexpr float CV_POL_VOCT   = -1.0f; // A1
static constexpr float CV_POL_SPREAD = -1.0f; // A2
static constexpr float CV_POL_MORPH  = -1.0f; // A3

// ---------- DAC modes ----------
static constexpr DacHandle::Channel AUX_DAC_CHANNEL = DacHandle::Channel::ONE; // A8 = CV OUT
static constexpr DacHandle::Channel LED_DAC_CHANNEL = DacHandle::Channel::TWO; // A7 = LED

// ADC and DAC
AdcChannelConfig adc_cfg[7];
static DacHandle dac;

// ---- Direct CVs (raw 0..1) ----
static float cv_timbre_raw = 0.f; // A0
static float cv_voct_raw   = 0.f; // A1
static float cv_spread_raw = 0.f; // A2
static float cv_morph_raw  = 0.f; // A3
static float cv_ssync_raw  = 0.f; // A4 (unused here)

// ---- Pots via MUX1 ----
static float kPitch  = 0.f;
static float kMorph  = 0.f; // volume
static float kSpread = 0.f; // swarm amount
static float kTimbre = 0.f; // wave morph

// ---- Attenuverters via MUX2 (0..1 → -1..+1) ----
static float at_timbre = 0.f; // idx 0
static float at_pitch  = 0.f; // idx 1
static float at_spread = 0.f; // idx 2
static float at_morph  = 0.f; // idx 3

// Baselines to remove DC from three CVs (pitch/timbre/morph)
static float cv_timbre_base = 0.5f;
static float cv_voct_base   = 0.5f;
static float cv_morph_base  = 0.5f;

// DSP state
static float lfo_phase = 0.f; // for AUX/CV OUT

// Unison phases (max 7 voices incl. center)
static constexpr int kMaxVoices = 7;
static float phases[kMaxVoices] = {0};

// ---------- helpers ----------
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float apply_uni(float cv01, float pol){ return pol >= 0.f ? cv01 : (1.f - cv01); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; } // 0..1 -> -1..+1
static inline void track_baseline(float in01, float &base, float alpha = 0.0005f){ base += alpha * (in01 - base); }

static inline float wave_sine(float ph)   { return sinf(2.f * M_PI * ph); }
static inline float wave_tri(float ph)    { return 1.f - 4.f * fabsf(ph - 0.5f); }
static inline float wave_saw(float ph)    { return 2.f * ph - 1.f; }
static inline float wave_square(float ph) { return ph < 0.5f ? 1.f : -1.f; }
static inline float wave_morph(float ph, float tone01)
{
    tone01 = clamp01(tone01);
    float idx = tone01 * 3.f;
    int   seg = (int)idx;           // 0,1,2
    float t   = idx - float(seg);   // 0..1
    if (seg == 0) return lerp(wave_sine(ph),   wave_tri(ph),  t);
    if (seg == 1) return lerp(wave_tri(ph),    wave_saw(ph),  t);
    return            lerp(wave_saw(ph),       wave_square(ph), t);
}

static void AudioCb(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    (void)in;
    const float sr = hw.AudioSampleRate();

    // ---- Read/derive smoothed control values (0..1) ----
    const float basePitch   = kPitch;
    const float baseVol     = 0.05f + 0.95f * kMorph;
    const float baseSpreadK = kSpread;
    const float baseTimbre  = kTimbre;

    const float cvTimbre = apply_uni(cv_timbre_raw, CV_POL_TIMBRE);
    const float cvVoct   = apply_uni(cv_voct_raw,   CV_POL_VOCT);
    const float cvSpread = apply_uni(cv_spread_raw, CV_POL_SPREAD);
    const float cvMorph  = apply_uni(cv_morph_raw,  CV_POL_MORPH);
    (void)cv_ssync_raw; // read but unused here

    // update baselines
    track_baseline(cvTimbre, cv_timbre_base);
    track_baseline(cvVoct,   cv_voct_base);
    track_baseline(cvMorph,  cv_morph_base);

    const float avTimbre = uni_to_bi(at_timbre);
    const float avPitch  = uni_to_bi(at_pitch);
    const float avSpread = uni_to_bi(at_spread);
    const float avMorph  = uni_to_bi(at_morph);

    // Pitch & params
    const float pitch01 = clamp01(basePitch + avPitch * (cvVoct - cv_voct_base));
    const float baseHz  = 50.f + pitch01 * 1950.f;

    const float vol     = clamp01(baseVol    + avMorph  * (cvMorph  - cv_morph_base));
    const float timbre  = clamp01(baseTimbre + avTimbre * (cvTimbre - cv_timbre_base));

    // Spread (swarm amount) uses straight attenuverter (classic DC control)
    const float spread  = clamp01(baseSpreadK + avSpread * (cvSpread - 0.5f));

    // Map Spread → voice count and detune
    const int   voices       = 1 + (int)floorf(spread * 6.0f + 1e-6f); // 1..7
    const float max_cents    = 30.0f;
    const float detune_cents = spread * max_cents;

    // Precompute per-voice detune factors
    float detune_factor[kMaxVoices];
    for(int v = 0; v < voices; ++v)
    {
        float rel = (voices == 1) ? 0.f : (-1.f + 2.f * (float)v / (float)(voices - 1));
        float shaped = copysignf(powf(fabsf(rel), 0.75f), rel); // pull inner voices closer
        float cents  = shaped * detune_cents;
        detune_factor[v] = powf(2.f, cents / 1200.f);
    }

    // Per-sample synthesis (fixes the “jumps”)
    for(size_t i = 0; i < n; ++i)
    {
        float mix = 0.f;

        for(int v = 0; v < voices; ++v)
        {
            float inc = (baseHz * detune_factor[v]) / sr;
            phases[v] += inc;
            if(phases[v] >= 1.f) phases[v] -= 1.f;

            mix += wave_morph(phases[v], timbre);
        }

        // average to keep level stable across voice counts
        if(voices > 0) mix *= (1.0f / (float)voices);

        float y = vol * mix;

        out[0][i] = y;
        out[1][i] = y;

        // LED on A7: rectified level
        float led = fabsf(y);
        if(led > 1.f) led = 1.f;
        dac.WriteValue(LED_DAC_CHANNEL, (uint16_t)(led * 4095.f));

        // A8: slow triangle 0..1
        const float lfo_hz = 0.25f;
        lfo_phase += lfo_hz / sr;
        if(lfo_phase >= 1.f) lfo_phase -= 1.f;
        float lfo = (lfo_phase < 0.5f) ? (lfo_phase * 2.f) : (2.f - lfo_phase * 2.f);
        dac.WriteValue(AUX_DAC_CHANNEL, (uint16_t)(lfo * 4095.f));
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ADC setup — do not change mapping
    adc_cfg[CV_TIMBRE].InitSingle(A0);
    adc_cfg[CV_VOCT  ].InitSingle(A1);
    adc_cfg[CV_SPREAD].InitSingle(A2);
    adc_cfg[CV_MORPH ].InitSingle(A3);
    adc_cfg[CV_SSYNC ].InitSingle(A4);

    // MUX1 pots
    adc_cfg[5].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2);
    // MUX2 attenuverters
    adc_cfg[6].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2);

    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();

    // DAC setup A8(CH1)=CV OUT, A7(CH2)=LED
    DacHandle::Config dcfg;
    dcfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dcfg.buff_state = DacHandle::BufferState::ENABLED;
    dcfg.mode       = DacHandle::Mode::POLLING;
    dcfg.chn        = DacHandle::Channel::BOTH;
    dac.Init(dcfg);

    hw.StartAudio(AudioCb);

    // Control polling
    while (1)
    {
        // Direct CVs (raw 0..1)
        cv_timbre_raw = hw.adc.GetFloat(CV_TIMBRE);
        cv_voct_raw   = hw.adc.GetFloat(CV_VOCT);
        cv_spread_raw = hw.adc.GetFloat(CV_SPREAD);
        cv_morph_raw  = hw.adc.GetFloat(CV_MORPH);
        cv_ssync_raw  = hw.adc.GetFloat(CV_SSYNC);

        // MUX1 (pots) on index 5
        kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
        kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
        kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
        kTimbre = hw.adc.GetMuxFloat(5, CH_TIMBRE);

        // MUX2 (attenuverters) on index 6
        at_timbre = hw.adc.GetMuxFloat(6, AT_CH_TIMBRE);
        at_pitch  = hw.adc.GetMuxFloat(6, AT_CH_PITCH);
        at_spread = hw.adc.GetMuxFloat(6, AT_CH_SPREAD);
        at_morph  = hw.adc.GetMuxFloat(6, AT_CH_MORPH);

        System::Delay(1);
    }
}
