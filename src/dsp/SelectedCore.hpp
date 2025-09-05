#pragma once
#include "dsp/VoxVcoCore.hpp"

namespace vm {

// Hardware build selector: alias the desired core type here.
// Your hardware/main.cpp can include this header and stay untouched.
using SelectedCore = VoxVcoCore;

} // namespace vm
