// Vox — stable mapping + per-sample swarm
// LED (A7) and AUX/ENV (A8) are both audio envelope followers (RMS w/ attack/release + gamma).
// • A7 (DAC CH2) = non-inverted write (bright = louder)
// • A8 (DAC CH1) = non-inverted 0..4095
// Everything else unchanged (controls, swarm, mappings). SSYNC is read but unused for now.

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

// Pots (do not change)
#define CH_PITCH   0
#define CH_MORPH   1  // used as volume
#define CH_SPREAD  2  // swarm amount
#define CH_TIMBRE  3  // wave morph

// ---------- MUX2 (attenuverters): COM=A6, selects D1/D2/D3 ----------
#define MUX2_COM_PIN  A6
#define MUX2_SEL0     D1
#define MUX2_SEL1     D2
#define MUX2_SEL2     D3

// AT channels (do not change)
#define AT_CH_TIMBRE  0
#define AT_CH_PITCH   1
#define AT_CH_SPREAD  2
#define AT_CH_MORPH   3

// ---------- CV inputs (direct ADCs) (do not change) ----------
enum CvAdcIndex : int { CV_TIMBRE = 0, CV_VOCT = 1, CV_SPREAD = 2, CV_MORPH = 3, CV_SSYNC = 4 };
// A0..A3 are inverting; SSYNC (A4) is raw (active-low).
static constexpr float CV_POL_TIMBRE = -1.0f;
static constexpr float CV_POL_VOCT   = -1.0f;
static constexpr float CV_POL_SPREAD = -1.0f;
static constexpr float CV_POL_MORPH  = -1.0f;

// ---------- DAC modes ----------
static constexpr DacHandle::Channel AUX_DAC_CHANNEL = DacHandle::Channel::ONE; // A8 = Aux/Env out
static constexpr DacHandle::Channel LED_DAC_CHANNEL = DacHandle::Channel::TWO; // A7 = LED

// ADC and DAC
AdcChannelConfig adc_cfg[7];
static DacHandle dac;

// ---- Direct CVs (raw 0..1) ----
static float cv_timbre_raw = 0.f; // A0
static float cv_voct_raw   = 0.f; // A1
static float cv_spread_raw = 0.f; // A2
static float cv_morph_raw  = 0.f; // A3
static float cv_ssync_raw  = 0.f; // A4 (active-low, unused here)

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

// Baselines for three CVs
static float cv_timbre_base = 0.5f;
static float cv_voct_base   = 0.5f;
static float cv_morph_base  = 0.5f;

// DSP state
static constexpr int kMaxVoices = 7;
static float phases[kMaxVoices] = {0};

// ---- Audio envelope follower shared by LED (A7) and AUX (A8) ----
// We smooth y^2 with attack/release, then sqrt and apply a perceptual gamma.
static float env2 = 0.f;   // smoothed power
static float env_atk2 = 0; // set in main()
static float env_rel2 = 0;

static inline float clamp01(float x){ return x < 0.f ? 0.f : (x > 1.f ? 1.f : x); }
static inline float lerp(float a, float b, float t){ return a + t * (b - a); }
static inline float apply_uni(float cv01, float pol){ return pol >= 0.f ? cv01 : (1.f - cv01); }
static inline float uni_to_bi(float u){ return (u * 2.f) - 1.f; }
static inline void  track_baseline(float in01, float &base, float alpha = 0.0005f){ base += alpha * (in01 - base); }

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

    // ---- Read/derive control values ----
    const float basePitch   = kPitch;
    const float baseVol     = 0.05f + 0.95f * kMorph;
    const float baseSpreadK = kSpread;
    const float baseTimbre  = kTimbre;

    const float cvTimbre = apply_uni(cv_timbre_raw, CV_POL_TIMBRE);
    const float cvVoct   = apply_uni(cv_voct_raw,   CV_POL_VOCT);
    const float cvSpread = apply_uni(cv_spread_raw, CV_POL_SPREAD);
    const float cvMorph  = apply_uni(cv_morph_raw,  CV_POL_MORPH);

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

    // Spread (swarm amount) classic attenuverter
    const float spread  = clamp01(baseSpreadK + avSpread * (cvSpread - 0.5f));

    // Map Spread → voice count and detune
    const int   voices       = 1 + (int)floorf(spread * 6.0f + 1e-6f); // 1..7
    const float max_cents    = 30.0f;
    const float detune_cents = spread * max_cents;

    float detune_factor[kMaxVoices];
    for(int v = 0; v < voices; ++v)
    {
        float rel = (voices == 1) ? 0.f : (-1.f + 2.f * (float)v / (float)(voices - 1));
        float shaped = copysignf(powf(fabsf(rel), 0.75f), rel); // pull inner voices closer
        float cents  = shaped * detune_cents;
        detune_factor[v] = powf(2.f, cents / 1200.f);
    }

    // Per-sample synthesis + shared envelope follower
    for(size_t i = 0; i < n; ++i)
    {
        // --- AUDIO ---
        float mix = 0.f;
        for(int v = 0; v < voices; ++v)
        {
            float inc = (baseHz * detune_factor[v]) / sr;
            phases[v] += inc;
            if(phases[v] >= 1.f) phases[v] -= 1.f;

            mix += wave_morph(phases[v], timbre);
        }
        if(voices > 0) mix *= (1.0f / (float)voices);

        float y = vol * mix;

        out[0][i] = y;
        out[1][i] = y;

        // --- AUDIO ENVELOPE FOLLOWER (shared for LED and AUX) ---
        float y2   = y * y;
        float coef = (y2 > env2) ? env_atk2 : env_rel2;
        env2 += coef * (y2 - env2);
        float env_lin = sqrtf(env2);                   // 0..1 after RMS
        float env_out = powf(clamp01(env_lin), 0.6f);  // perceptual

        // A7 LED (now non-inverted so brighter = louder)
        dac.WriteValue(LED_DAC_CHANNEL, (uint16_t)(env_out * 4095.f));
        // A8 AUX/ENV (non-inverted)
        dac.WriteValue(AUX_DAC_CHANNEL, (uint16_t)(env_out * 4095.f));
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

    // Envelope follower time constants (RMS power domain):
    // attack ≈ 10 ms, release ≈ 120 ms; tweak to taste.
    const float sr = hw.AudioSampleRate();
    env_atk2 = 1.0f / (0.010f * sr);
    env_rel2 = 1.0f / (0.120f * sr);

    hw.StartAudio(AudioCb);

    // Control polling (unchanged)
    while (1)
    {
        // Direct CVs (raw 0..1)
        cv_timbre_raw = hw.adc.GetFloat(CV_TIMBRE);
        cv_voct_raw   = hw.adc.GetFloat(CV_VOCT);
        cv_spread_raw = hw.adc.GetFloat(CV_SPREAD);
        cv_morph_raw  = hw.adc.GetFloat(CV_MORPH);
        cv_ssync_raw  = hw.adc.GetFloat(CV_SSYNC); // read for future sync use

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
