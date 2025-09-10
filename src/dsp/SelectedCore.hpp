#pragma once
// Flexible core selector so hardware/main.cpp never changes.
//
// Three ways to pick your core:
//
// 1) Keep your current default (works with your existing code):
//      #include "dsp/VoxVcoCore.hpp"
//      using vm::SelectedCore = vm::VoxVcoCore;
//
// 2) Use the template core in dsp/cores/:
//      add DEFS_EXTRA="-DVM_USE_VOXTEMPLATE=1" at build time
//
// 3) Fully override via macros (no source edit), e.g.:
//      DEFS_EXTRA='-DVM_SELECTED_CORE_HEADER="\"dsp/cores/MyNewCore.hpp\"" -DVM_SELECTED_CORE_TYPE=vm::MyNewCore'
//
// Notes:
// - VM_SELECTED_CORE_HEADER must expand to a quoted header path, e.g. "\"dsp/cores/MyNewCore.hpp\"".
// - VM_SELECTED_CORE_TYPE must be the fully-qualified C++ type name.

#include "IDspCore.hpp"

// --- Option 3: full macro override (header + type) ---
#ifdef VM_SELECTED_CORE_HEADER
  #include VM_SELECTED_CORE_HEADER
  #ifndef VM_SELECTED_CORE_TYPE
    #error "VM_SELECTED_CORE_TYPE must be defined when using VM_SELECTED_CORE_HEADER"
  #endif
  namespace vm { using SelectedCore = VM_SELECTED_CORE_TYPE; }

// --- Option 2: opt into the template core in dsp/cores/ ---
#elif defined(VM_USE_VOXTEMPLATE)
  #include "dsp/cores/VoxTemplateCore.hpp"
  namespace vm { using SelectedCore = VoxTemplateCore; }

// --- Option 1: default = your current VoxVcoCore in dsp/ ---
#else
  #include "dsp/VoxVcoCore.hpp"
  namespace vm { using SelectedCore = VoxVcoCore; }
#endif
