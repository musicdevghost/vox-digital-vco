// Vox — Buchla-ish fold with reduced range
// • Fold range tightened: subtle at low Timbre, capped max (≈2 stages, gentler drive/sat/enh)
// • All else unchanged (mappings, Spread/unison, AGC, LED=AUX env follower, PWM polyBLEP, saw multiply)

#include "daisy_seed.h"
#include <cmath>

using namespace daisy;
using namespace daisy::seed;

DaisySeed hw;

// ======================= Tunables =========================
static constexpr float kOutputLimit   = 0.40f; // master output cap (unchanged)
static constexpr float kSpreadTauSec  = 0.030f; // Spread smoothing (~30 ms)

// Envelope follower feel (LED & AUX)
static constexpr float kEnvAtkMs      = 6.0f;
static constexpr float kEnvRelMs      = 140.0f;
static constexpr float kEnvGain       = 1.6f;
static constexpr float kEnvGamma      = 0.65f;

// LED behavior
static constexpr bool  kLedInvert     = true;   // invert if LED is current-sink

// ===== Fold range scaler (NEW): lower → gentler overall max =====
static constexpr float kFoldRange     = 0.60f;  // 0..1. Try 0.50–0.70 to taste.

// ======================= AGC Tunables =====================
static constexpr float kAgcMin        = 0.5f;
static constexpr float kAgcMax        = 2.0f;
static constexpr float kAgcTargetRms  = 0.35f;
static constexpr float kAgcSlew       = 0.0025f;

// ======================= Pins / Mapping ===================
#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7

#define CH_PITCH   0  // KNOB 0
#define CH_MORPH   1  // KNOB 1  -> shape morph
#define CH_SPREAD  2  // KNOB 2  -> swarm
#define CH_TIMBRE  3  // KNOB 3  -> per-shape modulation

#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D1
#define MUX2_SEL1     D2
#define MUX2_SEL2     D3

#define AT_CH_TIMBRE  0   // affects Timbre modulation CV
#define AT_CH_PITCH   1   // affects V/Oct CV
#define AT_CH_SPREAD  2   // affects Spread CV
#define AT_CH_MORPH   3   // affects Morph CV

enum CvAdcIndex : int { CV_TIMBRE = 0, CV_VOCT = 1, CV_SPREAD = 2, CV_MORPH = 3, CV_SSYNC = 4 };
// A0..A3 are inverting; SSYNC (A4) is raw (active-low).
static constexpr float CV_POL_TIMBRE = -1.0f;
static constexpr float CV_POL_VOCT   = -1.0f;
static constexpr float CV_POL_SPREAD = -1.0f;
static constexpr float CV_POL_MORPH  = -1.0f;

static constexpr DacHandle::Channel AUX_DAC_CHANNEL = DacHandle::Channel::ONE; // A8 = Aux/Env out
static constexpr DacHandle::Channel LED_DAC_CHANNEL = DacHandle::Channel::TWO; // A7 = LED

// ======================= IO State =========================
AdcChannelConfig adc_cfg[7];
static DacHandle dac;

// Direct CVs (raw 0..1)
static float cv_timbre_raw = 0.f; // A0
static float cv_voct_raw   = 0.f; // A1
static float cv_spread_raw = 0.f; // A2
static float cv_morph_raw  = 0.f; // A3
static float cv_ssync_raw  = 0.f; // A4 (unused for now)

// Pots via MUX1
static float kPitch  = 0.f; // pitch
static float kMorph  = 0.f; // shape morph  (0..1)
static float kSpread = 0.f; // swarm amount (0..1)
static float kTimbre = 0.f; // modulation   (0..1)

// Attenuverters via MUX2 (0..1 → -1..+1)
static float at_timbre = 0.f; // idx 0
static float at_pitch  = 0.f; // idx 1
static float at_spread = 0.f; // idx 2
static float at_morph  = 0.f; // idx 3

// Baselines for CVs that use deviation
static float cv_timbre_base = 0.5f;
static float cv_voct_base   = 0.5f;
static float cv_morph_base  = 0.5f;

// DSP state
static constexpr int kMaxVoices = 7;
static float phases[kMaxVoices] = {0};

// ---- Envelope followers ----
static float env2 = 0.f;   // post-gain power follower (LED & AUX)
static float env_atk2 = 0;
static float env_rel2 = 0;

// Pre-gain RMS (AGC)
static float pre_env2 = 0.f;
static float pre_atk2 = 0;
static float pre_rel2 = 0;
static float agc_gain = 1.0f;

// Spread smoothing
static float spread_smooth = 0.f;
static float spread_alpha  = 0.f;

// ======================= Helpers ==========================
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float apply_uni(float cv01, float pol){ return pol >= 0.f ? cv01 : (1.f - cv01); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; }
static inline void  track_baseline(float in01, float &base, float alpha = 0.0005f){ base += alpha * (in01 - base); }

// Basic waves
static inline float wave_sine(float ph)   { return sinf(2.f * M_PI * ph); }
static inline float wave_tri(float ph)    { return 1.f - 4.f * fabsf(ph - 0.5f); }

// ---- polyBLEP for band-limited transitions ----
static inline float poly_blep(float t, float dt)
{
    if(t < dt)
    {
        t /= dt;
        return t + t - t * t - 1.0f;
    }
    else if(t > 1.0f - dt)
    {
        t = (t - 1.0f) / dt;
        return t * t + t + t + 1.0f;
    }
    return 0.0f;
}

// Band-limited PWM square (duty 0..1, 5..95% internally), returns [-1,1]
static inline float wave_square_pwm_blep(float ph, float duty, float dt)
{
    duty = clamp01(duty);
    if(duty < 0.05f) duty = 0.05f;
    if(duty > 0.95f) duty = 0.95f;

    float y = (ph < duty) ? 1.0f : -1.0f;

    // upward step at ph = 0.0
    y -= poly_blep(ph, dt);
    // downward step at ph = duty
    float tt = ph - duty;
    if(tt < 0.f) tt += 1.f;
    y += poly_blep(tt, dt);

    return y;
}

// ===== Wavefolder: Buchla-ish multi-stage triangle wrap + soft saturation =====
// Range reduced & tapered for finer control.
//   • amt' = pow( clamp(amt * kFoldRange), 1.25 )   → gentle near 0, capped max
//   • drive  : 1 .. 9
//   • stages : 1 .. 2
//   • bias   : up to ~0.10 (gentle asymmetry)
//   • sat    : subtle, 0.9 .. 1.1
//   • enh_mix: up to ~0.12 (digital spice)
static inline float tri_wrap_core(float x)
{
    // Map any x to a centered triangle in [-1,1]
    float u     = x * 0.5f + 0.5f;           // -1..1 -> 0..1
    float m     = u - floorf(u);             // fract
    float tri01 = 1.f - fabsf(m * 2.f - 1.f);
    return tri01 * 2.f - 1.f;                // back to -1..1
}

static inline float buchlaish_fold(float x, float amt)
{
    // Taper & cap the effective amount
    float a = clamp01(amt * kFoldRange);
    a = powf(a, 1.25f);                      // finer low-end control

    float drive  = 1.0f + 8.0f * a;          // 1..9
    int   stages = 1 + (int)floorf(a * 1.9f); // 1..2
    float bias   = 0.10f * a;                // gentle asymmetry
    float sat    = 0.9f  + 0.20f * a;        // mild tanh strength

    float y = x;
    for(int s = 0; s < stages; ++s)
    {
        float b = ((s & 1) ? -1.f : 1.f) * bias; // alternate the bias per stage
        y = tri_wrap_core((y + b) * drive);
        y = tanhf(y * sat);
    }

    // Subtle digital enhancement (reduced range)
    float enh_mix = 0.12f * a;               // up to 12%
    float enh     = sinf((2.0f + 4.0f * a) * 0.5f * M_PI * y);
    return lerp(y, 0.7f * y + 0.3f * enh, enh_mix);
}

// Saw phase-multiply (sync-like) keeping base period
static inline float wave_saw_multiply(float ph, float mult_amt)
{
    float mul = 1.f + 4.f * clamp01(mult_amt); // 1..5
    float ph2 = ph * mul - floorf(ph * mul);
    return 2.f * ph2 - 1.f;
}

// Morph engine with per-shape modulation (fold uses buchlaish_fold)
static inline float wave_morph_with_timbre(float ph, float morph01, float timbre01, float dt)
{
    morph01  = clamp01(morph01);
    timbre01 = clamp01(timbre01);

    float s_sin = buchlaish_fold(wave_sine(ph), timbre01);
    float s_tri = buchlaish_fold(wave_tri(ph),  timbre01);
    float s_saw = wave_saw_multiply(ph,   timbre01);           // multiply
    float duty  = 0.5f + 0.45f * (timbre01 - 0.5f) * 2.f;      // ~5..95%
    float s_sqr = wave_square_pwm_blep(ph, duty, dt);          // PWM (polyBLEP)

    float idx = morph01 * 3.f;
    int   seg = (int)idx;                 // 0,1,2
    float t   = idx - float(seg);         // 0..1

    if(seg == 0) return lerp(s_sin, s_tri, t);
    if(seg == 1) return lerp(s_tri, s_saw, t);
    /*seg==2*/   return lerp(s_saw, s_sqr, t);
}

static void AudioCb(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    (void)in;
    const float sr = hw.AudioSampleRate();

    // ---- Read/derive control values ----
    const float basePitch   = kPitch;                    // 0..1
    const float baseMorph   = kMorph;                    // 0..1 shape crossfade
    const float baseSpreadK = kSpread;                   // 0..1 swarm
    const float baseTimbre  = kTimbre;                   // 0..1 per-shape modulation

    const float cvTimbre = apply_uni(cv_timbre_raw, CV_POL_TIMBRE);
    const float cvVoct   = apply_uni(cv_voct_raw,   CV_POL_VOCT);
    const float cvSpread = apply_uni(cv_spread_raw, CV_POL_SPREAD);
    const float cvMorph  = apply_uni(cv_morph_raw,  CV_POL_MORPH);

    // update baselines for deviation-based CVs
    track_baseline(cvTimbre, cv_timbre_base);
    track_baseline(cvVoct,   cv_voct_base);
    track_baseline(cvMorph,  cv_morph_base);

    const float avTimbre = uni_to_bi(at_timbre);
    const float avPitch  = uni_to_bi(at_pitch);
    const float avSpread = uni_to_bi(at_spread);
    const float avMorph  = uni_to_bi(at_morph);

    // Pitch
    const float pitch01 = clamp01(basePitch + avPitch * (cvVoct - cv_voct_base));
    const float baseHz  = 50.f + pitch01 * 1950.f;

    // Morph & Timbre
    const float morph   = clamp01(baseMorph + avMorph  * (cvMorph - cv_morph_base));
    const float timbre  = clamp01(baseTimbre + avTimbre * (cvTimbre - cv_timbre_base));

    // Spread (swarm amount) classic attenuverter + smoothing
    const float spread_target = clamp01(baseSpreadK + avSpread * (cvSpread - 0.5f));
    spread_smooth += spread_alpha * (spread_target - spread_smooth);
    const float spread = spread_smooth;

    // Map Spread → fractional voices and detune
    const float voices_f     = 1.0f + spread * 6.0f;  // 1..7 continuous
    const int   v_int        = (int)voices_f;         // floor
    const float v_frac       = voices_f - (float)v_int; // 0..1 fade for the next voice
    const int   voices_used  = v_int + (v_int < kMaxVoices ? 1 : 0); // include the fading-in one
    const float max_cents    = 30.0f;
    const float detune_cents = spread * max_cents;

    // Precompute detune factors and per-voice weights
    float detune_factor[kMaxVoices];
    float vweight[kMaxVoices];
    float wsum = 0.0f;

    for(int v = 0; v < voices_used; ++v)
    {
        float rel = (voices_used == 1) ? 0.f : (-1.f + 2.f * (float)v / (float)(voices_used - 1));
        float shaped = copysignf(powf(fabsf(rel), 0.75f), rel); // pull inner voices closer
        float cents  = shaped * detune_cents;
        detune_factor[v] = powf(2.f, cents / 1200.f);

        float w = (v < v_int) ? 1.0f : v_frac;
        vweight[v] = w;
        wsum += w;
    }
    if(wsum <= 0.f) wsum = 1.f; // safety

    // Per-sample synthesis + AGC + followers
    for(size_t i = 0; i < n; ++i)
    {
        // --- Osc block (pre-gain) ---
        float mix = 0.f;
        const float inc_base = baseHz / sr;

        for(int v = 0; v < voices_used; ++v)
        {
            float inc = inc_base * detune_factor[v];
            phases[v] += inc;
            if(phases[v] >= 1.f) phases[v] -= 1.f;

            float sig = wave_morph_with_timbre(phases[v], morph, timbre, inc);
            mix += vweight[v] * sig;
        }
        mix *= (1.0f / wsum);

        // --- PRE-gain RMS follower for AGC ---
        float m2 = mix * mix;
        float pre_coef = (m2 > pre_env2) ? pre_atk2 : pre_rel2;
        pre_env2 += pre_coef * (m2 - pre_env2);
        float pre_rms = sqrtf(pre_env2) + 1e-6f;

        // AGC desired gain
        float desired_gain = kAgcTargetRms / pre_rms;
        if(desired_gain < kAgcMin) desired_gain = kAgcMin;
        if(desired_gain > kAgcMax) desired_gain = kAgcMax;
        agc_gain += kAgcSlew * (desired_gain - agc_gain);

        // Apply base cap and AGC
        float y = kOutputLimit * agc_gain * mix;
        if(y > 1.f)  y = 1.f;
        if(y < -1.f) y = -1.f;

        out[0][i] = y;
        out[1][i] = y;

        // --- POST-gain RMS follower → LED/AUX ---
        float y2   = y * y;
        float coef = (y2 > env2) ? env_atk2 : env_rel2;
        env2 += coef * (y2 - env2);

        float env_lin   = sqrtf(env2);
        float spread_boost = 0.9f + 0.2f * spread; // 0.9..1.1
        float env_shaped  = powf(clamp01(env_lin * kEnvGain * spread_boost), kEnvGamma);
        if(env_shaped > 1.f) env_shaped = 1.f;

        // AUX/ENV (A8): 0..4095 → ~0..5V analog
        dac.WriteValue(AUX_DAC_CHANNEL, (uint16_t)(env_shaped * 4095.f));

        // LED (A7): same envelope (optionally inverted)
        uint16_t led_dac = (uint16_t)(env_shaped * 4095.f);
        if(kLedInvert) led_dac = 4095 - led_dac;
        dac.WriteValue(LED_DAC_CHANNEL, led_dac);
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ----- ADC setup (mapping unchanged) -----
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

    // ----- DAC setup -----
    DacHandle::Config dcfg;
    dcfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dcfg.buff_state = DacHandle::BufferState::ENABLED;
    dcfg.mode       = DacHandle::Mode::POLLING;
    dcfg.chn        = DacHandle::Channel::BOTH; // A8 (CH1) + A7 (CH2)
    dac.Init(dcfg);

    const float sr = hw.AudioSampleRate();

    // POST-gain follower (LED/AUX)
    env_atk2 = 1.0f / ((kEnvAtkMs / 1000.f) * sr);
    env_rel2 = 1.0f / ((kEnvRelMs / 1000.f) * sr);

    // PRE-gain follower (AGC)
    pre_atk2 = 1.0f / (0.040f * sr); // ~40 ms
    pre_rel2 = 1.0f / (0.400f * sr); // ~400 ms

    // Spread smoothing coefficient
    spread_alpha = 1.0f / (kSpreadTauSec * sr);

    hw.StartAudio(AudioCb);

    // Control polling
    while (1)
    {
        // Direct CVs (raw 0..1)
        cv_timbre_raw = hw.adc.GetFloat(CV_TIMBRE);
        cv_voct_raw   = hw.adc.GetFloat(CV_VOCT);
        cv_spread_raw = hw.adc.GetFloat(CV_SPREAD);
        cv_morph_raw  = hw.adc.GetFloat(CV_MORPH);
        cv_ssync_raw  = hw.adc.GetFloat(CV_SSYNC); // unused for now

        // MUX1 (pots)
        kPitch  = hw.adc.GetMuxFloat(5, CH_PITCH);
        kMorph  = hw.adc.GetMuxFloat(5, CH_MORPH);
        kSpread = hw.adc.GetMuxFloat(5, CH_SPREAD);
        kTimbre = hw.adc.GetMuxFloat(5, CH_TIMBRE);

        // MUX2 (attenuverters)
        at_timbre = hw.adc.GetMuxFloat(6, AT_CH_TIMBRE);
        at_pitch  = hw.adc.GetMuxFloat(6, AT_CH_PITCH);
        at_spread = hw.adc.GetMuxFloat(6, AT_CH_SPREAD);
        at_morph  = hw.adc.GetMuxFloat(6, AT_CH_MORPH);

        System::Delay(1);
    }
}
