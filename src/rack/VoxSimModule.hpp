#pragma once
#include <rack.hpp>
#include "../dsp/VoxCore.hpp"
#include "../hal/ControlMap.hpp"
#include "../framework/TemplateIO.hpp"

struct VoxSimModule : rack::engine::Module {
    VoxSimModule();
    void process(const ProcessArgs& args) override;

private:
    static constexpr int kBlock = 48;

    // Shared DSP bits
    vm::vox::CoreParams coreParams_;
    vm::vox::VoxCore    core_;
    vm::vox::State      state_;

    // Controls snapshot used for a whole block (fixes zipper/boundary)
    vm::vox::Controls   c_block_;

    // Staging buffers (audio-rate mods + rendered audio)
    float outL_[kBlock];
    float outR_[kBlock];
    float fm_[kBlock];
    float hsync_[kBlock];
    float ssync_[kBlock];
    int   frameIndex_ = kBlock;
};
