// Pots unchanged; MUX2 attenuverters scale the 4 CVs.
// SSYNC is read but bypassed so you always hear sound.
// Spread CV path is now a straightforward attenuverter (no baseline HP).

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

// Pots channel map (confirmed):
#define CH_PITCH   0  // KNOB 0
#define CH_MORPH   1  // KNOB 1 (used as volume here)
#define CH_SPREAD  2  // KNOB 2 (fold)
#define CH_TIMBRE  3  // KNOB 3 (wave morph)

// ---------- MUX2 (attenuverters): COM=A6, selects D1/D2/D3 ----------
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D1  // 4051 A (LSB)
#define MUX2_SEL1     D2  // 4051 B
#define MUX2_SEL2     D3  // 4051 C

// Attenuverter channels (AT_ACTIVE_INDEX mapping)
#define AT_CH_TIMBRE  0   // Timbre attenuverter
#define AT_CH_PITCH   1   // Pitch attenuverter
#define AT_CH_SPREAD  2   // Spread attenuverter
#define AT_CH_MORPH   3   // Morph attenuverter

// ---------- CV inputs (direct ADCs) per mapping ----------
enum CvAdcIndex : int { CV_TIMBRE = 0, CV_VOCT = 1, CV_SPREAD = 2, CV_MORPH = 3, CV_SSYNC = 4 };
// A0..A3 are inverting; SSYNC (A4) is raw (active-low), read but not used to gate.
static constexpr float CV_POL_TIMBRE = -1.0f; // A0
static constexpr float CV_POL_VOCT   = -1.0f; // A1
static constexpr float CV_POL_SPREAD = -1.0f; // A2
static constexpr float CV_POL_MORPH  = -1.0f; // A3

// ---------- DAC modes ----------
static constexpr DacHandle::Channel AUX_DAC_CHANNEL = DacHandle::Channel::ONE; // A8 = CV OUT
static constexpr DacHandle::Channel LED_DAC_CHANNEL = DacHandle::Channel::TWO; // A7 = LED

// 5 direct CVs + MUX1 + MUX2
AdcChannelConfig adc_cfg[7];
static DacHandle dac;

// ---- Direct CVs (A0..A4) raw 0..1 ----
static float cv_timbre_raw = 0.f; // A0 (inverting front-end)
static float cv_voct_raw   = 0.f; // A1 (inverting)
static float cv_spread_raw = 0.f; // A2 (inverting)
static float cv_morph_raw  = 0.f; // A3 (inverting)
static float cv_ssync_raw  = 0.f; // A4 RAW (active-low), not used to gate

// ---- Pots via MUX1 ----
static float kPitch  = 0.f;
static float kMorph  = 0.f; // used as volume
static float kSpread = 0.f; // fold
static float kTimbre = 0.f; // wave morph

// ---- Attenuverters via MUX2 (0..1 → -1..+1) ----
static float at_timbre = 0.f; // AT idx 0
static float at_pitch  = 0.f; // AT idx 1
static float at_spread = 0.f; // AT idx 2
static float at_morph  = 0.f; // AT idx 3

// Per-CV baselines (used for Timbre/Morph/Pitch only)
static float cv_timbre_base = 0.5f;
static float cv_voct_base   = 0.5f;
static float cv_morph_base  = 0.5f;

// DSP state
static float phase = 0.f;
static float lfo_phase = 0.f; // for AUX/CV OUT bring-up

// Helpers
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float apply_uni(float cv01, float pol){ return pol >= 0.f ? cv01 : (1.f - cv01); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; } // 0..1 -> -1..+1

// Small slew for baseline tracking (leaky average)
static inline void track_baseline(float in01, float &base, float alpha = 0.0005f)
{
    base += alpha * (in01 - base);
}

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
    const float basePitch  = kPitch;                     // 0..1
    const float baseVol    = 0.05f + 0.95f * kMorph;     // 0..1  (Morph knob used as volume)
    const float baseSpread = kSpread;                    // 0..1 (fold parameter)
    const float baseTimbre = kTimbre;                    // 0..1

    // ---- CVs (polarity-corrected to 0..1) ----
    const float cvTimbre = apply_uni(cv_timbre_raw, CV_POL_TIMBRE);
    const float cvVoct   = apply_uni(cv_voct_raw,   CV_POL_VOCT);
    const float cvSpread = apply_uni(cv_spread_raw, CV_POL_SPREAD);
    const float cvMorph  = apply_uni(cv_morph_raw,  CV_POL_MORPH);
    const float ssync    = cv_ssync_raw; // raw, active-low (unused here)
    (void)ssync;

    // ---- Update per-CV baselines for the three that use deviation ----
    track_baseline(cvTimbre, cv_timbre_base);
    track_baseline(cvVoct,   cv_voct_base);
    track_baseline(cvMorph,  cv_morph_base);

    // ---- Attenuverters (0..1 → −1..+1) ----
    const float avTimbre = uni_to_bi(at_timbre);  // AT idx 0 → Timbre
    const float avPitch  = uni_to_bi(at_pitch);   // AT idx 1 → Pitch
    const float avSpread = uni_to_bi(at_spread);  // AT idx 2 → Spread
    const float avMorph  = uni_to_bi(at_morph);   // AT idx 3 → Morph (volume here)

    // ---- Apply CVs ----
    // Pitch: knob + avPitch * (cvVoct - baseline)  (no DC bias from AT)
    const float pitch01 = clamp01(basePitch + avPitch * (cvVoct - cv_voct_base));
    const float freq    = 50.f + pitch01 * 1950.f;

    // Volume: avMorph * (cvMorph - baseline)
    const float vol     = clamp01(baseVol    + avMorph  * (cvMorph - cv_morph_base));

    // Timbre (wave morph): avTimbre * (cvTimbre - baseline)
    const float timbre  = clamp01(baseTimbre + avTimbre * (cvTimbre - cv_timbre_base));

    // Spread (fold): **straight attenuverter** around knob (no baseline HP)
    // This makes the Spread CV behave like a standard attenuverted DC control.
    const float spread  = clamp01(baseSpread + avSpread * (cvSpread - 0.5f));

    const float inc = freq / sr;

    for (size_t i = 0; i < n; ++i)
    {
        phase += inc;
        if (phase >= 1.f) phase -= 1.f;

        float x = wave_morph(phase, timbre);
        x = softFold(x, spread);

        const float y = vol * x;

        // Audio out
        out[0][i] = y;
        out[1][i] = y;

        // LED on A7 (Channel TWO): rectified level
        float led = fabsf(y);
        if(led > 1.f) led = 1.f;
        dac.WriteValue(LED_DAC_CHANNEL, (uint16_t)(led * 4095.f));

        // AUX/CV OUT on A8 (Channel ONE): slow triangle 0..1
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

    // ----- ADC setup -----
    // Direct CVs (A0..A4)
    adc_cfg[CV_TIMBRE].InitSingle(A0); // CV_IN 0: Timbre (inverting)
    adc_cfg[CV_VOCT  ].InitSingle(A1); // CV_IN 1: V/Oct  (inverting)
    adc_cfg[CV_SPREAD].InitSingle(A2); // CV_IN 2: Spread (inverting)
    adc_cfg[CV_MORPH ].InitSingle(A3); // CV_IN 3: Morph  (inverting)
    adc_cfg[CV_SSYNC ].InitSingle(A4); // CV_IN 4: SSYNC  (RAW, active-low; ignored here)

    // MUX1 pots
    adc_cfg[5].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2);
    // MUX2 attenuverters
    adc_cfg[6].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2);

    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();

    // ----- DAC setup -----
    DacHandle::Config dcfg;
    dcfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dcfg.buff_state = DacHandle::BufferState::ENABLED;
    dcfg.mode       = DacHandle::Mode::POLLING;
    dcfg.chn        = DacHandle::Channel::BOTH; // A8 (CH1) + A7 (CH2)
    dac.Init(dcfg);

    hw.StartAudio(AudioCb);

    // Poll all CVs, pots, and attenuverters
    while (1)
    {
        // Direct CVs (raw 0..1)
        cv_timbre_raw = hw.adc.GetFloat(CV_TIMBRE);
        cv_voct_raw   = hw.adc.GetFloat(CV_VOCT);
        cv_spread_raw = hw.adc.GetFloat(CV_SPREAD);
        cv_morph_raw  = hw.adc.GetFloat(CV_MORPH);
        cv_ssync_raw  = hw.adc.GetFloat(CV_SSYNC); // RAW, not used here

        // MUX1 (pots) on index 5
        kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
        kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
        kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
        kTimbre = hw.adc.GetMuxFloat(5, CH_TIMBRE);

        // MUX2 (attenuverters) on index 6
        at_timbre = hw.adc.GetMuxFloat(6, AT_CH_TIMBRE); // idx 0
        at_pitch  = hw.adc.GetMuxFloat(6, AT_CH_PITCH);  // idx 1
        at_spread = hw.adc.GetMuxFloat(6, AT_CH_SPREAD); // idx 2
        at_morph  = hw.adc.GetMuxFloat(6, AT_CH_MORPH);  // idx 3

        System::Delay(1);
    }
}
