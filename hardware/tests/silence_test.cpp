// Minimal offline test: ensure no NaN/Inf/subnormal results for a short buffer.

#include <cmath>
#include <cfloat>
#include <cstdio>
#include <random>

static inline float SoftShape(float x, float amt)
{
    const float sat = std::tanh(x * (1.0f + 9.0f * amt));
    return (1.0f - amt) * x + amt * sat;
}

static inline bool is_subnormal(float x)
{
    return std::fpclassify(x) == FP_SUBNORMAL;
}

int main()
{
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> dist(-0.5f, 0.5f);

    int n = 48000; // ~1s
    int bad_nan = 0, bad_inf = 0, bad_sub = 0;

    for(int i = 0; i < n; ++i)
    {
        float x = dist(rng);
        float y = SoftShape(x, 0.3f);
        if(std::isnan(y)) bad_nan++;
        if(std::isinf(y)) bad_inf++;
        if(is_subnormal(y)) bad_sub++;
    }

    if(bad_nan || bad_inf || bad_sub)
    {
        std::printf("FAIL: NaN=%d Inf=%d Subnormals=%d\n", bad_nan, bad_inf, bad_sub);
        return 1;
    }

    std::puts("OK: no NaN/Inf/Subnormals detected");
    return 0;
}
