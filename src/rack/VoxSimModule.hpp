#pragma once
#include <rack.hpp>
#include "../dsp/VoxCore.hpp"
#include "../hw/sim_hal.hpp"
#include "../framework/TemplateIO.hpp"

struct VoxSimModule : rack::engine::Module {
    VoxSimModule();
    void process(const ProcessArgs& args) override;

private:
    static constexpr int kBlock = 48;
    vm::vox::CoreParams coreParams_;
    vm::vox::VoxCore core_;
    vm::vox::SimHal hal_;
    vm::vox::State state_;

    float outL_[kBlock];
    float outR_[kBlock];
    int   frameIndex_ = kBlock;
};
