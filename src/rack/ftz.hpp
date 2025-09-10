#pragma once

// Enable Flush-To-Zero and Denormals-Are-Zero to avoid denormal slowdowns on x86.
// No-ops on platforms without SSE2.
#if defined(__SSE2__)
  #include <xmmintrin.h>
namespace vm { namespace rackutil {
inline void enableFTZ_DAZ()
{
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);
}
}} // namespace vm::rackutil
#else
namespace vm { namespace rackutil {
inline void enableFTZ_DAZ() {}
}} // namespace vm::rackutil
#endif
