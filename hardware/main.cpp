// Vox — SSYNC as true hard sync (GPIO D18, active-low, per-sample)
// Keeps: CV stretch (±5V), extended Morph, fold/additive, Spread smoothing/limits,
// AGC, LED/AUX follower, heavy-guard. Only SSYNC path changed to a GPIO.
// If your GATE_IN_1 is on D19 instead, change SYNC_PIN below to D19.

#include "daisy_seed.h"
#include <cmath>

using namespace daisy;
using namespace daisy::seed;

DaisySeed hw;

// ======================= Tunables =========================
static constexpr float kOutputLimit      = 0.40f; // master output cap
static constexpr float kSpreadTauSec     = 0.030f; // Spread smoothing (~30 ms)
static constexpr float kSpreadCurveExp   = 1.60f;  // >1 = gentler near max

// Envelope follower feel (LED & AUX)
static constexpr float kEnvAtkMs         = 6.0f;
static constexpr float kEnvRelMs         = 140.0f;
static constexpr float kEnvGain          = 1.6f;
static constexpr float kEnvGamma         = 0.65f;

// LED behavior (inverted if LED is current-sink)
static constexpr bool  kLedInvert        = true;

// ======================= AGC Tunables =====================
static constexpr float kAgcMin           = 0.5f;
static constexpr float kAgcMax           = 2.0f;
static constexpr float kAgcTargetRms     = 0.35f;
static constexpr float kAgcSlew          = 0.0025f;

// ===== Your config =====
static constexpr float kFoldRange        = 0.60f;  // max fold intensity
static constexpr int   kAddMaxHarm       = 8;      // additive absolute cap
static constexpr int   kMaxVoices        = 5;      // hard cap voices
static constexpr float kDetuneMaxCents   = 30.0f;  // max spread detune

// ===== Heavy-guard thresholds (worst-case protection) =====
static constexpr float kHG_SpreadTh      = 0.85f;
static constexpr float kHG_MorphTh       = 0.80f;
static constexpr float kHG_TimbreTh      = 0.60f;

// ===== CV/AT stretch (compensate 4051/ADC rails) =====
static constexpr float kCvLo             = 0.03f;  // direct CVs ~0.03..0.97
static constexpr float kCvHi             = 0.97f;
static constexpr float kMuxLo            = 0.04f;  // attenuverters ~0.04..0.96
static constexpr float kMuxHi            = 0.96f;

// ======================= Pins / Mapping ===================
// Pots (MUX1)
#define MUX1_COM_PIN  A5
#define MUX1_SEL0     D5
#define MUX1_SEL1     D6
#define MUX1_SEL2     D7

#define CH_PITCH   0  // KNOB 0
#define CH_MORPH   1  // KNOB 1  -> extended morph
#define CH_SPREAD  2  // KNOB 2  -> swarm
#define CH_TIMBRE  3  // KNOB 3  -> fold depth / harmonic amount

// Attenuverters (MUX2)
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D1
#define MUX2_SEL1     D2
#define MUX2_SEL2     D3

#define AT_CH_TIMBRE  0
#define AT_CH_PITCH   1
#define AT_CH_SPREAD  2
#define AT_CH_MORPH   3

// Direct CVs (A0..A3 only; SSYNC now uses GPIO)
enum CvAdcIndex : int { CV_TIMBRE = 0, CV_VOCT = 1, CV_SPREAD = 2, CV_MORPH = 3 };

// A0..A3 are inverting
static constexpr float CV_POL_TIMBRE = -1.0f;
static constexpr float CV_POL_VOCT   = -1.0f;
static constexpr float CV_POL_SPREAD = -1.0f;
static constexpr float CV_POL_MORPH  = -1.0f;

// DACs
static constexpr DacHandle::Channel AUX_DAC_CHANNEL = DacHandle::Channel::ONE; // A8 = Aux/Env out
static constexpr DacHandle::Channel LED_DAC_CHANNEL = DacHandle::Channel::TWO; // A7 = LED

// SSYNC (HARD SYNC) — GPIO settings
// Your schematic shows GATE_IN_1 on a pin labeled "18/ADC3". Use D18 here.
// If it's actually on the next pad, set to D19.
static constexpr Pin  SYNC_PIN        = D19;
static constexpr bool SYNC_ACTIVE_LOW = true;

// ======================= IO State =========================
AdcChannelConfig adc_cfg[6]; // 4 direct + 2 mux
static DacHandle dac;
static GPIO      sync_in;    // SSYNC digital input (note: GPIO, not Gpio)

// Direct CVs (raw 0..1)
static float cv_timbre_raw = 0.f; // A0
static float cv_voct_raw   = 0.f; // A1
static float cv_spread_raw = 0.f; // A2
static float cv_morph_raw  = 0.f; // A3

// Pots via MUX1
static float kPitch  = 0.f;
static float kMorph  = 0.f;
static float kSpread = 0.f;
static float kTimbre = 0.f;

// Attenuverters via MUX2 (0..1 → -1..+1)
static float at_timbre = 0.f; // idx 0
static float at_pitch  = 0.f; // idx 1
static float at_spread = 0.f; // idx 2
static float at_morph  = 0.f; // idx 3

// Baselines
static float cv_timbre_base = 0.5f;
static float cv_voct_base   = 0.5f;
static float cv_morph_base  = 0.5f;

// DSP state
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

// SSYNC edge tracking
static bool sync_prev = true; // idle (high) for active-low

// ======================= Helpers ==========================
static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float apply_uni(float cv01, float pol){ return (pol >= 0.f) ? cv01 : (1.f - cv01); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; }
static inline void  track_baseline(float in01, float &base, float alpha = 0.0005f){ base += alpha * (in01 - base); }
static inline float smoothstep01(float t){ t = clamp01(t); return t * t * (3.f - 2.f * t); }
static inline float stretch01(float x, float lo, float hi){ return clamp01((x - lo) / (hi - lo)); }

// Basic waves
static inline float wave_sine(float ph)   { return sinf(2.f * M_PI * ph); }
static inline float wave_tri(float ph)    { return 1.f - 4.f * fabsf(ph - 0.5f); }

// polyBLEP helpers
static inline float poly_blep(float t, float dt)
{
    if(t < dt)           { t /= dt; return t + t - t * t - 1.0f; }
    else if(t > 1.f-dt)  { t = (t - 1.f) / dt; return t * t + t + t + 1.0f; }
    return 0.0f;
}
static inline float wave_square_pwm_blep(float ph, float duty, float dt)
{
    duty = clamp01(duty);
    if(duty < 0.05f) duty = 0.05f;
    if(duty > 0.95f) duty = 0.95f;
    float y = (ph < duty) ? 1.0f : -1.0f;
    y -= poly_blep(ph, dt);
    float tt = ph - duty; if(tt < 0.f) tt += 1.f;
    y += poly_blep(tt, dt);
    return y;
}

// Buchla-ish folder
static inline float tri_wrap_core(float x)
{
    float u     = x * 0.5f + 0.5f;
    float m     = u - floorf(u);
    float tri01 = 1.f - fabsf(m * 2.f - 1.f);
    return tri01 * 2.f - 1.f;
}
static inline float buchlaish_fold(float x, float amt)
{
    float a = clamp01(amt * kFoldRange);
    a = powf(a, 1.25f);
    float drive  = 1.0f + 8.0f * a;
    int   stages = 1 + (int)floorf(a * 1.9f);
    float bias   = 0.10f * a;
    float sat    = 0.9f  + 0.20f * a;
    float y = x;
    for(int s = 0; s < stages; ++s)
    {
        float b = ((s & 1) ? -1.f : 1.f) * bias;
        y = tri_wrap_core((y + b) * drive);
        y = tanhf(y * sat);
    }
    float enh_mix = 0.12f * a;
    float enh     = sinf((2.0f + 4.0f * a) * 0.5f * M_PI * y);
    return lerp(y, 0.7f * y + 0.3f * enh, enh_mix);
}

// Saw multiply
static inline float wave_saw_multiply(float ph, float mult_amt)
{
    float mul = 1.f + 4.f * clamp01(mult_amt); // 1..5
    float ph2 = ph * mul - floorf(ph * mul);
    return 2.f * ph2 - 1.f;
}

// Additive (budgeted)
static inline float sine_additive_budgeted(float ph, float amt, float dt, int kmax)
{
    amt = clamp01(amt);
    if(amt <= 1e-4f || kmax < 1) return 0.f;
    int nyq = (int)fminf((0.49f / fmaxf(dt, 1e-6f)), (float)kmax);
    if(nyq < 1) return 0.f;
    float p = 1.1f - 0.5f * amt;
    float odd_bias = 0.6f + 0.4f * amt;
    float acc = 0.f, norm = 0.f;
    for(int k = 2; k <= nyq + 1; ++k)
    {
        float w = powf((float)k, -p);
        float ob = (k % 2) ? odd_bias : (1.0f - 0.5f * amt);
        w *= (0.7f + 0.3f * ob);
        norm += w;
        float pk = ph * (float)k; pk -= floorf(pk);
        acc += w * sinf(2.f * M_PI * pk);
    }
    if(norm > 1e-6f) acc /= norm;
    float mix = 0.25f + 0.75f * amt;
    return mix * acc;
}

// Morph path (6 nodes)
static inline float wave_node(int node, float ph, float timbre, float dt, int add_budget)
{
    switch(node)
    {
        case 0: return buchlaish_fold(wave_sine(ph), timbre);
        case 1: return buchlaish_fold(wave_tri(ph),  timbre);
        case 2: return wave_saw_multiply(ph, timbre);
        case 3: { float duty = 0.5f + 0.45f * (timbre - 0.5f) * 2.f; return wave_square_pwm_blep(ph, duty, dt); }
        case 4: return buchlaish_fold(wave_tri(ph),  0.6f * timbre);
        default:
        case 5: { float base = wave_sine(ph); float add = sine_additive_budgeted(ph, timbre, dt, add_budget);
                  return tanhf(0.95f * (base + add)); }
    }
}
static inline float wave_morph_extended(float ph, float morph01, float timbre01, float dt, int add_budget)
{
    morph01  = clamp01(morph01);
    timbre01 = clamp01(timbre01);
    float idx = morph01 * 5.f;
    int   seg = (int)idx; if(seg > 4) seg = 4;
    float t   = idx - (float)seg;
    float ts  = smoothstep01(t);
    float a = wave_node(seg,     ph, timbre01, dt, add_budget);
    float b = wave_node(seg + 1, ph, timbre01, dt, add_budget);
    return lerp(a, b, ts);
}

static void AudioCb(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t n)
{
    (void)in;
    const float sr = hw.AudioSampleRate();

    // ---- Control values ----
    const float basePitch   = kPitch;
    const float baseMorph   = kMorph;
    const float baseSpreadK = kSpread;
    const float baseTimbre  = kTimbre;

    // CVs → polarity-corrected then stretched
    float cvTimbre = stretch01(apply_uni(cv_timbre_raw, CV_POL_TIMBRE), kCvLo, kCvHi);
    float cvVoct   = stretch01(apply_uni(cv_voct_raw,   CV_POL_VOCT),   kCvLo, kCvHi);
    float cvSpread = stretch01(apply_uni(cv_spread_raw, CV_POL_SPREAD), kCvLo, kCvHi);
    float cvMorph  = stretch01(apply_uni(cv_morph_raw,  CV_POL_MORPH),  kCvLo, kCvHi);

    // baselines (mainly helpful for pitch drift)
    track_baseline(cvTimbre, cv_timbre_base);
    track_baseline(cvVoct,   cv_voct_base);
    track_baseline(cvMorph,  cv_morph_base);

    // attenuverters → bipolar
    float avTimbre = uni_to_bi(stretch01(at_timbre, kMuxLo, kMuxHi));
    float avPitch  = uni_to_bi(stretch01(at_pitch,  kMuxLo, kMuxHi));
    float avSpread = uni_to_bi(stretch01(at_spread, kMuxLo, kMuxHi));
    float avMorph  = uni_to_bi(stretch01(at_morph,  kMuxLo, kMuxHi));

    // pitch
    const float pitch01 = clamp01(basePitch + avPitch * (cvVoct - cv_voct_base));
    const float baseHz  = 50.f + pitch01 * 1950.f;

    // bipolar deltas for full throw
    const float dMorph   = (cvMorph  - 0.5f) * 2.0f;
    const float dTimbre  = (cvTimbre - 0.5f) * 2.0f;
    const float dSpread  = (cvSpread - 0.5f) * 2.0f;

    const float morph    = clamp01(baseMorph  + avMorph  * dMorph);
    const float timbre   = clamp01(baseTimbre + avTimbre * dTimbre);

    // Spread with smoothing
    const float spread_target = clamp01(baseSpreadK + avSpread * dSpread);
    spread_smooth += spread_alpha * (spread_target - spread_smooth);
    const float spread = spread_smooth;

    // Heavy-guard
    const bool heavy_guard = (spread > kHG_SpreadTh) && (morph > kHG_MorphTh) && (timbre > kHG_TimbreTh);

    // Voices
    const float voices_f     = 1.0f + powf(spread, kSpreadCurveExp) * (float)(kMaxVoices - 1);
    const int   v_int        = (int)voices_f;
    const float v_frac       = voices_f - (float)v_int;
    const int   voices_used  = v_int + ((v_int < kMaxVoices && !heavy_guard) ? 1 : 0);
    const float detune_cents = spread * kDetuneMaxCents;

    // Detune/weights
    float detune_factor[kMaxVoices];
    float vweight[kMaxVoices];
    float wsum = 0.f;
    for(int v = 0; v < voices_used; ++v)
    {
        float rel = (voices_used == 1) ? 0.f : (-1.f + 2.f * (float)v / (float)(voices_used - 1));
        float shaped = copysignf(powf(fabsf(rel), 0.75f), rel);
        float cents  = shaped * detune_cents;
        detune_factor[v] = powf(2.f, cents / 1200.f);
        float w = (v < v_int) ? 1.0f : v_frac;
        vweight[v] = w; wsum += w;
    }
    if(wsum <= 0.f) wsum = 1.f;

    // Additive budget
    int add_budget = 2 + (int)floorf((1.0f - clamp01(spread)) * (float)(kAddMaxHarm - 2));
    if(add_budget < 1) add_budget = 1;
    if(heavy_guard) add_budget = 1;

    // ===== Per-sample synthesis =====
    for(size_t i = 0; i < n; ++i)
    {
        // --- HARD SYNC (GPIO, active-low) ---
        bool pin_high = sync_in.Read();                        // true = logic HIGH at pin
        bool sync_now = SYNC_ACTIVE_LOW ? !pin_high : pin_high;
        bool edge     = SYNC_ACTIVE_LOW ? (sync_prev && !sync_now)
                                        : (!sync_prev && sync_now);
        sync_prev = sync_now;
        if(edge)
        {
            // Reset every oscillator phase right at this sample
            for(int v = 0; v < kMaxVoices; ++v) phases[v] = 0.0f;
        }

        // --- Osc block (pre-gain) ---
        float mix = 0.f;
        const float inc_base = baseHz / sr;
        for(int v = 0; v < voices_used; ++v)
        {
            float inc = inc_base * detune_factor[v];
            phases[v] += inc;
            if(phases[v] >= 1.f) phases[v] -= 1.f;
            float sig = wave_morph_extended(phases[v], morph, timbre, inc, add_budget);
            mix += vweight[v] * sig;
        }
        mix *= (1.0f / wsum);

        // --- PRE-gain RMS follower for AGC ---
        float m2 = mix * mix;
        float pre_coef = (m2 > pre_env2) ? pre_atk2 : pre_rel2;
        pre_env2 += pre_coef * (m2 - pre_env2);
        float pre_rms = sqrtf(pre_env2) + 1e-6f;

        float desired_gain = kAgcTargetRms / pre_rms;
        if(desired_gain < kAgcMin) desired_gain = kAgcMin;
        if(desired_gain > kAgcMax) desired_gain = kAgcMax;
        agc_gain += kAgcSlew * (desired_gain - agc_gain);

        float y = kOutputLimit * agc_gain * mix;
        if(y > 1.f)  y = 1.f;
        if(y < -1.f) y = -1.f;

        out[0][i] = y;
        out[1][i] = y;

        // --- POST-gain follower (for LED/AUX) ---
        float y2   = y * y;
        float coef = (y2 > env2) ? env_atk2 : env_rel2;
        env2 += coef * (y2 - env2);
    }

    // ---- One DAC write per block (LED/AUX env) ----
    float env_lin   = sqrtf(env2);
    float spread_boost = 0.9f + 0.2f * spread; // 0.9..1.1
    float env_shaped  = powf(clamp01(env_lin * kEnvGain * spread_boost), kEnvGamma);
    if(env_shaped > 1.f) env_shaped = 1.f;

    uint16_t aux_val = (uint16_t)(env_shaped * 4095.f);
    uint16_t led_val = kLedInvert ? (uint16_t)(4095 - aux_val) : aux_val;

    dac.WriteValue(AUX_DAC_CHANNEL, aux_val);
    dac.WriteValue(LED_DAC_CHANNEL, led_val);
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    hw.SetAudioBlockSize(48);

    // ----- ADC setup (4 direct + 2 mux) -----
    adc_cfg[CV_TIMBRE].InitSingle(A0);
    adc_cfg[CV_VOCT  ].InitSingle(A1);
    adc_cfg[CV_SPREAD].InitSingle(A2);
    adc_cfg[CV_MORPH ].InitSingle(A3);
    adc_cfg[4].InitMux(MUX1_COM_PIN, 8, MUX1_SEL0, MUX1_SEL1, MUX1_SEL2); // pots
    adc_cfg[5].InitMux(MUX2_COM_PIN, 8, MUX2_SEL0, MUX2_SEL1, MUX2_SEL2); // attenuverters

    hw.adc.Init(adc_cfg, 6);
    hw.adc.Start();

    // ----- DAC setup -----
    DacHandle::Config dcfg;
    dcfg.bitdepth   = DacHandle::BitDepth::BITS_12;
    dcfg.buff_state = DacHandle::BufferState::ENABLED;
    dcfg.mode       = DacHandle::Mode::POLLING;
    dcfg.chn        = DacHandle::Channel::BOTH; // A8 (CH1) + A7 (CH2)
    dac.Init(dcfg);

    // ----- SSYNC GPIO (active-low) -----
    sync_in.Init(SYNC_PIN, GPIO::Mode::INPUT, GPIO::Pull::NOPULL);
    sync_prev = true; // idle high for active-low

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

        // MUX1 (pots)
        kPitch  = hw.adc.GetMuxFloat(4, CH_PITCH);
        kMorph  = hw.adc.GetMuxFloat(4, CH_MORPH);
        kSpread = hw.adc.GetMuxFloat(4, CH_SPREAD);
        kTimbre = hw.adc.GetMuxFloat(4, CH_TIMBRE);

        // MUX2 (attenuverters)
        at_timbre = hw.adc.GetMuxFloat(5, AT_CH_TIMBRE);
        at_pitch  = hw.adc.GetMuxFloat(5, AT_CH_PITCH);
        at_spread = hw.adc.GetMuxFloat(5, AT_CH_SPREAD);
        at_morph  = hw.adc.GetMuxFloat(5, AT_CH_MORPH);

        System::Delay(1);
    }
}
