#pragma once
#include <cstdint>

// Central place for constants shared by Rack and Daisy.
namespace vm { namespace vox { namespace cfg {

// Default linear FM depth in Hz used by both wrappers until mapped to a control.
constexpr double kDefaultFmDepthHz = 220.0;

}}} // namespace vm::vox::cfg
