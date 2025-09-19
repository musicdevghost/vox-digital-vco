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
    vm::vox::CoreParams coreParams_;
    vm::vox::VoxCore core_;
    vm::vox::State state_;

    // Staging
    float outL_[kBlock];
    float outR_[kBlock];
    float fm_[kBlock];
    float hsync_[kBlock];
    float ssync_[kBlock];
    int   frameIndex_ = kBlock;
};
