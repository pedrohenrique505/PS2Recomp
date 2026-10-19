#pragma once
#include <algorithm>
#include <cstdint>
#if defined(__SSE4_1__) && defined(__FMA__)
#include <immintrin.h>
#endif

namespace GSInternal
{
    inline uint32_t bilinearColorScalar(uint32_t c00, uint32_t c10, uint32_t c01,
                                        uint32_t c11, float fx, float fy)
    {
        uint32_t color = 0;
        for (uint32_t shift = 0; shift < 32u; shift += 8u)
        {
            const float a = (c00 >> shift) & 255u;
            const float b = (c10 >> shift) & 255u;
            const float c = (c01 >> shift) & 255u;
            const float d = (c11 >> shift) & 255u;
            const float top = a + (b - a) * fx;
            const float bottom = c + (d - c) * fx;
            const int channel = static_cast<int>(static_cast<double>(top + (bottom - top) * fy) + 0.5);
            color |= static_cast<uint32_t>(std::clamp(channel, 0, 255)) << shift;
        }
        return color;
    }

    inline uint32_t bilinearColor(uint32_t c00, uint32_t c10, uint32_t c01,
                                  uint32_t c11, float fx, float fy)
    {
#if defined(__SSE4_1__) && defined(__FMA__)
        const auto unpack = [](uint32_t color) {
            return _mm_cvtepi32_ps(_mm_cvtepu8_epi32(_mm_cvtsi32_si128(static_cast<int>(color))));
        };
        const __m128 a = unpack(c00), b = unpack(c10), c = unpack(c01), d = unpack(c11);
        const __m128 x = _mm_set1_ps(fx), y = _mm_set1_ps(fy);
        const __m128 top = _mm_fmadd_ps(_mm_sub_ps(b, a), x, a);
        const __m128 bottom = _mm_fmadd_ps(_mm_sub_ps(d, c), x, c);
        const __m128 value = _mm_fmadd_ps(_mm_sub_ps(bottom, top), y, top);
        // O +0.5 original é em double. Fazê-lo em float altera casos perto
        // de meio inteiro, inclusive com o arredondamento VU para zero.
        const __m128d half = _mm_set1_pd(0.5);
        const __m128i low = _mm_cvttpd_epi32(_mm_add_pd(_mm_cvtps_pd(value), half));
        const __m128i high = _mm_cvttpd_epi32(_mm_add_pd(_mm_cvtps_pd(_mm_movehl_ps(value, value)), half));
        __m128i channels = _mm_unpacklo_epi64(low, high);
        channels = _mm_min_epi32(_mm_max_epi32(channels, _mm_setzero_si128()), _mm_set1_epi32(255));
        const __m128i words = _mm_packus_epi32(channels, _mm_setzero_si128());
        return static_cast<uint32_t>(_mm_cvtsi128_si32(_mm_packus_epi16(words, _mm_setzero_si128())));
#else
        return bilinearColorScalar(c00, c10, c01, c11, fx, fy);
#endif
    }
}
