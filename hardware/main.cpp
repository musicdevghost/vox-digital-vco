#include "daisy_seed.h"
#include "daisysp.h"
#include <cmath>
#include <cstring>

// Shared DSP core
#include "../src/dsp/VoxAudioCore.hpp"

using namespace daisy;
using namespace daisysp;

DaisySeed hw;
VoxAudioCore core;

// Simple fixed params for the template (adjust or wire to pots)
static VoxAudioCore::Params params = {0.5f, 1.0f, 0.5f, 0.0f};

void AudioCallback(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size) {
    // Pass block directly to core (non-interleaved channel arrays)
    core.setParams(params);
    core.processBlock(in[0], in[1], out[0], out[1], size);
}

int main(void) {
    hw.Configure();
    hw.Init();

    // 48kHz, stereo
    hw.SetAudioBlockSize(48); // ~1ms @ 48k
    float sr = hw.AudioSampleRate();

    core.init(sr);
    core.reset();

    hw.StartAudio(AudioCallback);
    for(;;) { System::Delay(100); }
}
