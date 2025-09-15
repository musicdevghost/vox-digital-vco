// Pots unchanged; add MUX2 attenuverters that scale/invert the 4 CVs.
// Gate is bypassed so you always hear sound.

#include "daisy_seed.h"
#include <cmath>

using namespace daisy;
using namespace daisy::seed;

// ---------- MUX1 (pots): COM=A5, selects D5/D6/D7 ----------
#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7

// Channel map you confirmed for pots:
// ch0 = Pitch, ch1 = Morph (volume), ch2 = Spread (fold), ch3 = Tone (wave morph)
#define CH_PITCH   0
#define CH_MORPH   1
#define CH_SPREAD  2
#define CH_TONE    3

// ---------- MUX2 (attenuverters): COM=A6, selects D1/D2/D3 ----------
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D1  // 4051 A (LSB)
#define MUX2_SEL1     D2  // 4051 B
#define MUX2_SEL2     D3  // 4051 C

// Attenuverter channel guesses (change if your board maps differently)
#define AT_CH_FILTER  0   // FILTER_AT  → will scale CV for Wave Morph
#define AT_CH_SIZE    1   // SIZE_AT    → will scale CV for Pitch
#define AT_CH_FEEDB   2   // FEEDB_AT   → will scale CV for Volume
#define AT_CH_DIFF    3   // DIFF_AT    → will scale CV for Fold

// CV polarity (your CV1..CV4 stages are inverting per your schematic)
static constexpr float CV_POL_PITCH = -1.0f; // A0
static constexpr float CV_POL_VOL   = -1.0f; // A1
static constexpr float CV_POL_FOLD  = -1.0f; // A2
static constexpr float CV_POL_MORPH = -1.0f; // A3

// Pitch CV depth in the 0..1 domain (1.0 → full ±0.5 around the knob)
static constexpr float CV_PITCH_RANGE = 1.0f;

DaisySeed hw;
// 5 direct CVs + MUX1 + MUX2
AdcChannelConfig adc_cfg[7];

// ---- Direct CVs (A0..A4) ----
static float cv1_pitch = 0.f;  // A0
static float cv2_vol   = 0.f;  // A1
static float cv3_fold  = 0.f;  // A2
static float cv4_morph = 0.f;  // A3
static float cv5_gate  = 0.f;  // A4 (ignored/bypassed)

// ---- Pots via MUX1 ----
static float kPitch  = 0.f;
static float kMorph  = 0.f; // used as volume
static float kSpread = 0.f; // fold
static float kTone   = 0.f; // wave morph

// ---- Attenuverters via MUX2 (0..1 → -1..+1) ----
static float at_filter = 0.f;
static float at_size   = 0.f;
static float at_feedb  = 0.f;
static float at_diff   = 0.f;

// DSP state
static float phase = 0.f;

// Helpers
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float apply_uni(float cv01, float pol){ return pol >= 0.f ? cv01 : (1.f - cv01); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; } // 0..1 -> -1..+1

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
static inline float softFold(float x, float amt)
{
    const float drive = 1.f + 9.f * clamp01(amt);
    return tanhf(drive * x);
}

static void AudioCb(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    (void)in;
    const float sr = hw.AudioSampleRate();

    // ---- Baseline from pots (unchanged) ----
    const float basePitch = kPitch;                             // 0..1
    const float baseVol   = 0.05f + 0.95f * kMorph;             // 0..1
    const float baseFold  = kSpread;                            // 0..1
    const float baseMorph = kTone;                              // 0..1

    // ---- CVs with polarity correction ----
    const float cvPitchCorr = apply_uni(cv1_pitch, CV_POL_PITCH); // 0..1
    const float cvVolCorr   = apply_uni(cv2_vol,   CV_POL_VOL);
    const float cvFoldCorr  = apply_uni(cv3_fold,  CV_POL_FOLD);
    const float cvMorphCorr = apply_uni(cv4_morph, CV_POL_MORPH);

    // ---- Attenuverters (bipolar –1..+1) ----
    const float avPitch  = uni_to_bi(at_size);    // SIZE_AT controls pitch CV
    const float avVol    = uni_to_bi(at_feedb);   // FEEDB_AT controls volume CV
    const float avFold   = uni_to_bi(at_diff);    // DIFF_AT controls fold CV
    const float avMorph  = uni_to_bi(at_filter);  // FILTER_AT controls morph CV

    // ---- Apply CVs with attenuverters ----
    // Pitch: knob + (±)CV depth (continuous, no quantization)
    const float pitch01 = clamp01(basePitch + CV_PITCH_RANGE * avPitch * (cvPitchCorr - 0.5f));
    const float freq    = 50.f + pitch01 * 1950.f;

    // Volume: add CV around the knob (keep 0..1)
    const float vol     = clamp01(baseVol + avVol  * (cvVolCorr   - 0.5f));

    // Fold / Morph: add CV around the knob (keep 0..1)
    const float foldAmt = clamp01(baseFold  + avFold  * (cvFoldCorr  - 0.5f));
    const float morph   = clamp01(baseMorph + avMorph * (cvMorphCorr - 0.5f));

    const float inc = freq / sr;

    for (size_t i = 0; i < n; ++i)
    {
        phase += inc;
        if (phase >= 1.f) phase -= 1.f;

        float x = wave_morph(phase, morph);
        x = softFold(x, foldAmt);

        const float y = vol * x;
        out[0][i] = y;
        out[1][i] = y;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ----- ADC setup -----
    // Direct CVs
    adc_cfg[0].InitSingle(A0); // CV1: pitch (inverting front-end)
    adc_cfg[1].InitSingle(A1); // CV2: volume (inverting)
    adc_cfg[2].InitSingle(A2); // CV3: fold (inverting)
    adc_cfg[3].InitSingle(A3); // CV4: morph (inverting)
    adc_cfg[4].InitSingle(A4); // CV5: gate (ignored here)

    // MUX1 pots
    adc_cfg[5].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2);
    // MUX2 attenuverters
    adc_cfg[6].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2);

    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();

    hw.StartAudio(AudioCb);

    // Poll all CVs, pots, and attenuverters
    while (1)
    {
        // Direct CVs
        cv1_pitch = hw.adc.GetFloat(0);
        cv2_vol   = hw.adc.GetFloat(1);
        cv3_fold  = hw.adc.GetFloat(2);
        cv4_morph = hw.adc.GetFloat(3);
        cv5_gate  = hw.adc.GetFloat(4); // not used in this test

        // MUX1 (pots) on index 5
        kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
        kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
        kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
        kTone   = hw.adc.GetMuxFloat(5, CH_TONE);

        // MUX2 (attenuverters) on index 6
        at_filter = hw.adc.GetMuxFloat(6, AT_CH_FILTER);
        at_size   = hw.adc.GetMuxFloat(6, AT_CH_SIZE);
        at_feedb  = hw.adc.GetMuxFloat(6, AT_CH_FEEDB);
        at_diff   = hw.adc.GetMuxFloat(6, AT_CH_DIFF);

        System::Delay(1);
    }
}
