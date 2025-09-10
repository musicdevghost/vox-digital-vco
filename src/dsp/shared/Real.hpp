// a tiny shared header, e.g. src/dsp/shared/Real.hpp
#pragma once
#if defined(TARGET_DAISY)
using real_t = float;
#else
using real_t = double;
#endif
