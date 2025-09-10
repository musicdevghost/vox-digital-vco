#pragma once

// Heap usage is discouraged in audio paths.
// This header is intentionally empty at compile time.
// Enforcement is handled by your Makefile `verify` rule.
//
// If you want link-time enforcement, uncomment these to hard-error on any heap use.
/*
void* operator new  (unsigned long)              = delete;
void* operator new[](unsigned long)              = delete;
void  operator delete  (void*) noexcept          = delete;
void  operator delete[](void*) noexcept          = delete;
*/
