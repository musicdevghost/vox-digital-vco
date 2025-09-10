#pragma once
#include <cmath>
#include <cstddef>
#include <algorithm>

namespace vm {

// Attack/Release envelope follower with handy RMS helpers.
// All functions are inline; no allocations.
struct EnvFollower
{
    float y    = 0.f;  // current envelope
    float aAtk = 0.1f; // attack coefficient per-sample
    float aRel = 0.01f; // release coefficient per-sample

    // Configure directly in seconds.
    inline void setup(float sr, float atk_s, float rel_s)
    {
        const float atk = std::max(1e-6f, atk_s);
        const float rel = std::max(1e-6f, rel_s);
        aAtk = 1.f - std::exp(-1.f / (atk * sr));
        aRel = 1.f - std::exp(-1.f / (rel * sr));
    }

    // Convenience in milliseconds.
    inline void setupMs(float sr, float atk_ms, float rel_ms)
    {
        setup(sr, atk_ms * 1e-3f, rel_ms * 1e-3f);
    }

    // One-sample update (absolute detector).
    inline float process(float x)
    {
        x = std::fabs(x);
        const float a = (x >= y) ? aAtk : aRel;
        y += a * (x - y);
        return y;
    }

    // Compute RMS over a mono block, then AR-follow the RMS.
    inline float processBlockRms(const float* x, size_t n)
    {
        double acc = 0.0;
        for (size_t i = 0; i < n; ++i)
            acc += double(x[i]) * double(x[i]);
        const float rms = std::sqrt(float(acc / double(std::max<size_t>(1, n))));
        return process(rms);
    }

    // Compute RMS over a stereo block, then AR-follow the RMS.
    inline float processBlockRmsStereo(const float* l, const float* r, size_t n)
    {
        double acc = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            const double L = l[i];
            const double R = r[i];
            acc += 0.5 * (L * L + R * R);
        }
        const float rms = std::sqrt(float(acc / double(std::max<size_t>(1, n))));
        return process(rms);
    }
};

} // namespace vm
