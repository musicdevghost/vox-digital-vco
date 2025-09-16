// Daisy Seed — Full module wiring test (robust version)
// Stable when HSYNC/FM jacks are empty; sync/FM enable only above tiny input level.
// - Knobs (MUX1) + CVs (A0..A3) scaled by attenuverters (MUX2)
// - SSYNC on A4 (active-low, hysteresis) soft-syncs
// - L-IN (HSYNC) hard-syncs on rising zero-crossing *only if present*
// - R-IN (FM) linear FM *only if present*
// - AUX/ENV (DAC CH1) slow sine LFO; LED (DAC CH2) pulses once per cycle

#include "daisy_seed.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace daisy;
using namespace daisy::seed;

// ===== Feature toggles (quick on/off if you need to isolate) =====
static constexpr bool ENABLE_SSYNC = true;
static constexpr bool ENABLE_HSYNC = true;
static constexpr bool ENABLE_FM    = true;

// ---------- MUX1 (pots): COM=A5, selects D5/D6/D7 ----------
#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7
// Pots (fixed)
#define CH_PITCH   0   // Pitch (frequency)
#define CH_MORPH   1   // square↔sine morph
#define CH_SPREAD  2   // amplitude
#define CH_TIMBRE  3   // PWM duty

// ---------- MUX2 (attenuverters): COM=A6, selects D0/D1/D2 ----------
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D0  // 4051 A (LSB)
#define MUX2_SEL1     D1  // 4051 B
#define MUX2_SEL2     D2  // 4051 C
// Attenuverter channels (fixed)
#define AT_CH_TIMBRE 0   // → PWM duty
#define AT_CH_PITCH  1   // → Pitch
#define AT_CH_SPREAD 2   // → Amplitude
#define AT_CH_MORPH  3   // → Morph

// ---------- CV inputs (A0..A3 inverted in hardware; A4 = SSYNC active-low) ----------
static constexpr float THRESH_LOW  = 0.30f; // SSYNC active below this
static constexpr float THRESH_HIGH = 0.70f; // SSYNC inactive above this

// ---------- DAC outputs ----------
static constexpr DacHandle::Channel AUX_DAC_CHANNEL = DacHandle::Channel::ONE; // AUX/ENV
static constexpr DacHandle::Channel LED_DAC_CHANNEL = DacHandle::Channel::TWO; // LED
static constexpr float   LFO_FREQ_HZ          = 0.20f;
static constexpr float   LFO_CENTER           = 0.50f;
static constexpr float   LFO_AMPLITUDE        = 0.49f;
static constexpr uint32_t LED_PULSE_MS        = 6;
static constexpr float   LED_HIGH             = 0.98f;
static constexpr float   LED_LOW              = 0.0f;

// ---------- FM / pitch scaling ----------
static constexpr float FM_DEPTH_HZ   = 440.0f; // conservative depth for cleaner test
static constexpr float PITCH_MIN_HZ  = 30.0f;
static constexpr float PITCH_MAX_HZ  = 2030.0f;

// ---------- HSYNC/FM presence detection ----------
static constexpr float IN_ENV_ATTACK = 0.01f; // env smoothing (per-sample)
static constexpr float IN_ENV_RELEASE= 0.001f;
static constexpr float HSYNC_ENV_THRESH = 0.02f; // enable HSYNC above ~ -34 dBFS
static constexpr float FM_ENV_THRESH    = 0.02f; // enable FM above ~ -34 dBFS

DaisySeed hw;
DacHandle dac;

// ADC: 5 direct (A0..A4) + MUX1 + MUX2
AdcChannelConfig adc_cfg[7];

// Direct CVs
static float cv_timbre_raw = 0.f, cv_pitch_raw = 0.f, cv_spread_raw = 0.f, cv_morph_raw = 0.f, cv_ssync_raw = 1.f;
static float CV_Timbre = 0.f, CV_Pitch = 0.f, CV_Spread = 0.f, CV_Morph = 0.f;
static bool  ssync_active = false;

// Pots
static float kPitch = 0.f, kMorph = 0.f, kSpread = 0.f, kTimbre = 0.f;

// Attenuverters
static float at_timbre = 0.f, at_pitch = 0.f, at_spread = 0.f, at_morph = 0.f;

// DSP state
static float phase = 0.f;
static volatile bool  led_kick = false;
static uint32_t       led_until_ms = 0;

// Audio input envs for presence detect
static float env_lin = 0.f, env_rin = 0.f;

// Helpers
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; }
static inline uint16_t f01_to_dac12(float x){ if(x<0) x=0; if(x>1) x=1; return (uint16_t)(x*4095.f); }
static inline float apply_cv_att(float knob01, float cv01, float at01)
{
    const float av = uni_to_bi(at01);         // -1..+1
    const float mod = av * (cv01 - 0.5f);     // bipolar around knob
    return clamp01(knob01 + mod);
}
static inline float env_step(float env, float xabs)
{
    const float a = (xabs > env) ? IN_ENV_ATTACK : IN_ENV_RELEASE;
    return (1.f - a) * env + a * xabs;
}

static void AudioCb(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    const float sr = hw.AudioSampleRate();

    // Combine knobs + CVs (attenuvertered)
    const float pitch01 = apply_cv_att(kPitch,  CV_Pitch,  at_pitch);
    const float morph01 = apply_cv_att(kMorph,  CV_Morph,  at_morph);
    const float amp01   = apply_cv_att(kSpread, CV_Spread, at_spread);
    const float pwmDuty = 0.05f + 0.90f * apply_cv_att(kTimbre, CV_Timbre, at_timbre);

    float freq = PITCH_MIN_HZ + pitch01 * (PITCH_MAX_HZ - PITCH_MIN_HZ);

    static float prev_lin = 0.f;

    for(size_t i = 0; i < n; ++i)
    {
        const float lin = in[0][i]; // L-IN (HSYNC)
        const float rin = in[1][i]; // R-IN (FM)

        // Update envelopes for presence detection
        env_lin = env_step(env_lin, fabsf(lin));
        env_rin = env_step(env_rin, fabsf(rin));
        const bool hsync_present = ENABLE_HSYNC && (env_lin > HSYNC_ENV_THRESH);
        const bool fm_present    = ENABLE_FM    && (env_rin > FM_ENV_THRESH);

        // Hard sync: rising zero-crossing only when a real signal is present
        if(hsync_present && prev_lin <= 0.f && lin > 0.f)
            phase = 0.f;
        prev_lin = lin;

        // Soft sync (A4) if enabled and active
        if(ENABLE_SSYNC && ssync_active)
            phase = 0.f;

        // Linear FM only when present
        float inc = (freq + (fm_present ? FM_DEPTH_HZ * rin : 0.f)) / sr;
        if(inc < 0.f) inc = 0.f;

        float new_phase = phase + inc;
        if(new_phase >= 1.f)
        {
            new_phase -= 1.f;
            led_kick = true;
        }
        phase = new_phase;

        // Osc: square with PWM, morph to sine
        const float square = (phase < pwmDuty) ? 1.f : -1.f;
        const float sine   = sinf(2.f * (float)M_PI * phase);
        const float sig    = lerp(square, sine, morph01);

        const float y = amp01 * sig;
        out[0][i] = y;
        out[1][i] = y;
    }
}

int main(void)
{
    // Core/AIO
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ADC setup
    adc_cfg[0].InitSingle(A0); // TIMBRE (invert in hw)
    adc_cfg[1].InitSingle(A1); // PITCH  (invert)
    adc_cfg[2].InitSingle(A2); // SPREAD (invert)
    adc_cfg[3].InitSingle(A3); // MORPH  (invert)
    adc_cfg[4].InitSingle(A4); // SSYNC (active-low)
    adc_cfg[5].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2); // pots
    adc_cfg[6].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2); // attenuverters
    hw.adc.Init(adc_cfg, 7);
    hw.adc.Start();

    // DAC both channels
    DacHandle::Config dcfg;
    dcfg.mode     = DacHandle::Mode::POLLING;
    dcfg.bitdepth = DacHandle::BitDepth::BITS_12;
    dcfg.chn      = DacHandle::Channel::BOTH;
    dac.Init(dcfg);

    // Start audio
    hw.StartAudio(AudioCb);

    // AUX LFO + SSYNC read + LED pulse scheduling
    float lfo_phase = 0.f;
    uint32_t last_ms = System::GetNow();

    while(1)
    {
        // ---- Read CVs (invert A0..A3) ----
        cv_timbre_raw = hw.adc.GetFloat(0);
        cv_pitch_raw  = hw.adc.GetFloat(1);
        cv_spread_raw = hw.adc.GetFloat(2);
        cv_morph_raw  = hw.adc.GetFloat(3);
        cv_ssync_raw  = hw.adc.GetFloat(4);

        CV_Timbre = 1.f - cv_timbre_raw;
        CV_Pitch  = 1.f - cv_pitch_raw;
        CV_Spread = 1.f - cv_spread_raw;
        CV_Morph  = 1.f - cv_morph_raw;

        // SSYNC (active-low) with hysteresis
        if(ENABLE_SSYNC)
        {
            if(!ssync_active && cv_ssync_raw < THRESH_LOW)  ssync_active = true;
            if( ssync_active && cv_ssync_raw > THRESH_HIGH) ssync_active = false;
        }
        else
        {
            ssync_active = false;
        }

        // ---- Pots (MUX1) ----
        kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
        kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
        kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
        kTimbre = hw.adc.GetMuxFloat(5, CH_TIMBRE);

        // ---- Attenuverters (MUX2) ----
        at_timbre = hw.adc.GetMuxFloat(6, AT_CH_TIMBRE);
        at_pitch  = hw.adc.GetMuxFloat(6, AT_CH_PITCH);
        at_spread = hw.adc.GetMuxFloat(6, AT_CH_SPREAD);
        at_morph  = hw.adc.GetMuxFloat(6, AT_CH_MORPH);

        // ---- LED pulse (DAC CH2) ----
        const uint32_t now = System::GetNow();
        if(led_kick)
        {
            led_kick = false;
            led_until_ms = now + LED_PULSE_MS;
        }
        dac.WriteValue(LED_DAC_CHANNEL, f01_to_dac12((now < led_until_ms) ? LED_HIGH : LED_LOW));

        // ---- AUX slow sine LFO (DAC CH1) ----
        const float dt = (now - last_ms) / 1000.0f;
        last_ms = now;
        lfo_phase += LFO_FREQ_HZ * dt;
        if(lfo_phase >= 1.f) lfo_phase -= 1.f;
        const float lfo = LFO_CENTER + LFO_AMPLITUDE * sinf(2.f * (float)M_PI * lfo_phase);
        dac.WriteValue(AUX_DAC_CHANNEL, f01_to_dac12(lfo));

        System::Delay(1);
    }
}
