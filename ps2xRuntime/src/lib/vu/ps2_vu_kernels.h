#ifndef PS2_VU_KERNELS_H
#define PS2_VU_KERNELS_H

// Kernels de op: a semântica de cada operação aritmética VU do modelo de referência como função
// pura. Entram operandos normalizados (normalizeVuOperand), salvo onde a op lê os bits crus;
// saem valores por pista, flags por pista (Z=1, S=2, U=4, O=8) e o sticky dos produtos, ou Q com
// I/D e P. Nenhum kernel lê estado do interpretador nem ciclo: filas de flags e latências ficam
// com quem chama. O interpretador e os blocos AOT chamam estes kernels; os blocos estáticos vão
// chamá-los nas ops que não especializarem, e eles são o oráculo dos testes diferenciais. Este
// cabeçalho e ps2_vu1_detail.h só dependem da biblioteca padrão, para o código gerado incluí-los
// sem ps2_runtime.h. Os valores dependem do arredondamento do VU (FE_TOWARDZERO), que
// VU1Interpreter::run instala.

#include "ps2_vu1_detail.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <type_traits>

#if defined(__GNUC__) || defined(__clang__)
#define PS2_VU_KERNEL_INLINE inline __attribute__((always_inline))
#else
#define PS2_VU_KERNEL_INLINE inline
#endif

namespace VUKernels
{
    // Valor fora da faixa normal vira zero com sinal (U) ou ±FLT_MAX (O).
    PS2_VU_KERNEL_INLINE float normalizeResult(float value, uint8_t &laneFlags)
    {
        uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        const uint32_t sign = bits & 0x80000000u;
        const uint32_t magnitude = bits & 0x7FFFFFFFu;
        const uint32_t exponent = (bits >> 23) & 0xFFu;

        laneFlags = sign != 0u ? 0x2u : 0u;
        if (magnitude == 0u)
        {
            laneFlags |= 0x1u;
        }
        else if (exponent == 0u)
        {
            laneFlags |= 0x5u;
            bits = sign;
        }
        else if (exponent == 0xFFu)
        {
            laneFlags |= 0x8u;
            bits = sign | 0x7F7FFFFFu;
        }

        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    PS2_VU_KERNEL_INLINE float normalizeResult(float value)
    {
        uint8_t ignoredFlags = 0u;
        return normalizeResult(value, ignoredFlags);
    }

    // ------------------------------------------------------------------------
    // FMAC
    // ------------------------------------------------------------------------

    // OPMULA e OPMSUB são o produto cruzado: quem chama permuta as pistas (x←y·z, y←z·x,
    // z←x·y; w fica w·w, que só alimenta o sticky dos produtos) e o kernel zera w.
    enum class FmacOp : uint8_t
    {
        Add,
        Sub,
        Mul,
        Madd,
        Msub,
        Opmula,
        Opmsub
    };

    struct FmacLanes
    {
        float value[4];
        uint8_t flags[4];
        uint32_t productSticky;
    };

    // left * right + addend com um só arredondamento. Com operandos float o produto é exato em
    // double e long double, então fundir não muda o valor; o x87 não tem FMA e std::fma em
    // long double viraria uma chamada a fmal.
    template <typename Real>
    PS2_VU_KERNEL_INLINE Real multiplyAdd(Real left, Real right, Real addend)
    {
        if constexpr (std::is_same_v<Real, long double>)
            return addend + left * right;
        else
            return std::fma(left, right, addend);
    }

    // A mesma fórmula em float (o valor), em double (classificação rápida: o produto de dois
    // floats é exato e a soma arredondada preserva sinal e zero) e em long double (referência).
    template <FmacOp Op, typename Real>
    PS2_VU_KERNEL_INLINE Real fmacLane(Real left, Real right, Real addend, uint32_t lane)
    {
        if constexpr (Op == FmacOp::Add)
            return left + right;
        else if constexpr (Op == FmacOp::Sub)
            return left - right;
        else if constexpr (Op == FmacOp::Mul)
            return left * right;
        else if constexpr (Op == FmacOp::Madd)
            return multiplyAdd(left, right, addend);
        else if constexpr (Op == FmacOp::Msub)
            return multiplyAdd(-left, right, addend);
        else if constexpr (Op == FmacOp::Opmula)
            return lane == 3u ? Real(0) : left * right;
        else
            return lane == 3u ? Real(0) : multiplyAdd(-left, right, addend);
    }

    template <FmacOp Op>
    constexpr bool hasProductSum = Op == FmacOp::Madd || Op == FmacOp::Msub || Op == FmacOp::Opmsub;

    // Mesmo resultado de normalizeExact a partir do valor calculado em double, ou false quando
    // ele está perto de FLT_MAX/FLT_MIN e só o long double de referência decide.
    PS2_VU_KERNEL_INLINE bool normalizeExactFast(float &value, double exactResult, uint8_t &flags)
    {
        const double magnitude = std::fabs(exactResult);
        if (magnitude >= 0x1p127 || (magnitude != 0.0 && magnitude < 0x1p-125))
            return false;
        const bool negative = std::signbit(exactResult);
        flags = negative ? 0x2u : 0u;
        if (magnitude == 0.0)
        {
            flags |= 0x1u;
            const uint32_t bits = negative ? 0x80000000u : 0u;
            std::memcpy(&value, &bits, sizeof(value));
        }
        return true;
    }

    // Classificação completa de um resultado exato em double (produtos de floats são exatos).
    PS2_VU_KERNEL_INLINE uint8_t exactFlags(double exactResult)
    {
        const double magnitude = std::fabs(exactResult);
        uint8_t flags = std::signbit(exactResult) ? 0x2u : 0u;
        if (magnitude == 0.0)
            flags |= 0x1u;
        else if (magnitude > static_cast<double>(std::numeric_limits<float>::max()))
            flags |= 0x8u;
        else if (magnitude < static_cast<double>(std::numeric_limits<float>::min()))
            flags |= 0x5u;
        return flags;
    }

    inline uint8_t normalizeExact(float &value, long double exactResult)
    {
        const bool negative = std::signbit(exactResult);
        const long double magnitude = std::fabs(exactResult);
        const long double maximum = static_cast<long double>(std::numeric_limits<float>::max());
        const long double minimum = static_cast<long double>(std::numeric_limits<float>::min());
        uint8_t flags = negative ? 0x2u : 0u;

        uint32_t bits = negative ? 0x80000000u : 0u;
        if (magnitude == 0.0L)
        {
            flags |= 0x1u;
            std::memcpy(&value, &bits, sizeof(value));
        }
        else if (magnitude > maximum)
        {
            flags |= 0x8u;
            bits |= 0x7F7FFFFFu;
            std::memcpy(&value, &bits, sizeof(value));
        }
        else if (magnitude < minimum)
        {
            flags |= 0x5u;
            std::memcpy(&value, &bits, sizeof(value));
        }

        return flags;
    }

    // left/right/addend por pista, já normalizados; right já traz o broadcast, Q ou I. O valor
    // vem da aritmética float; o resultado exato decide flags e o ajuste perto de FLT_MAX/FLT_MIN.
    // Pistas fora de dest ficam com flags zero. As ops produto-soma reportam Z/S/U/O da soma, e
    // as condições de cada produto (sem o sinal de MSUB) acumulam no sticky.
    template <FmacOp Op>
    PS2_VU_KERNEL_INLINE FmacLanes fmac(const float left[4], const float right[4],
                                        const float addend[4], uint8_t dest)
    {
        FmacLanes out{};
        for (uint32_t lane = 0; lane < 4u; ++lane)
            out.value[lane] = fmacLane<Op, float>(left[lane], right[lane], addend[lane], lane);
        for (uint32_t lane = 0; lane < 4u; ++lane)
        {
            if ((dest & (8u >> lane)) == 0u)
                continue;
            const double fast = fmacLane<Op, double>(left[lane], right[lane], addend[lane], lane);
            if (!normalizeExactFast(out.value[lane], fast, out.flags[lane]))
            {
                const long double exact = fmacLane<Op, long double>(left[lane], right[lane], addend[lane], lane);
                out.flags[lane] = normalizeExact(out.value[lane], exact);
            }
            if constexpr (hasProductSum<Op>)
                out.productSticky |= exactFlags(static_cast<double>(left[lane]) * static_cast<double>(right[lane])) & 0xFu;
        }
        return out;
    }

    inline FmacLanes fmac(FmacOp op, const float left[4], const float right[4],
                          const float addend[4], uint8_t dest)
    {
        switch (op)
        {
        case FmacOp::Add:
            return fmac<FmacOp::Add>(left, right, addend, dest);
        case FmacOp::Sub:
            return fmac<FmacOp::Sub>(left, right, addend, dest);
        case FmacOp::Mul:
            return fmac<FmacOp::Mul>(left, right, addend, dest);
        case FmacOp::Madd:
            return fmac<FmacOp::Madd>(left, right, addend, dest);
        case FmacOp::Msub:
            return fmac<FmacOp::Msub>(left, right, addend, dest);
        case FmacOp::Opmula:
            return fmac<FmacOp::Opmula>(left, right, addend, dest);
        case FmacOp::Opmsub:
            break;
        }
        return fmac<FmacOp::Opmsub>(left, right, addend, dest);
    }

    // ------------------------------------------------------------------------
    // Demais ops do upper (sem flags MAC/status)
    // ------------------------------------------------------------------------

    PS2_VU_KERNEL_INLINE float maximum(float left, float right) { return left > right ? left : right; }
    PS2_VU_KERNEL_INLINE float minimum(float left, float right) { return left < right ? left : right; }
    PS2_VU_KERNEL_INLINE float absolute(float value) { return std::fabs(value); }

    // ITOF0/4/12/15: bits crus da pista como inteiro com FractionBits bits de fração.
    template <uint32_t FractionBits>
    PS2_VU_KERNEL_INLINE float itof(int32_t value)
    {
        if constexpr (FractionBits == 0u)
            return static_cast<float>(value);
        else
            return static_cast<float>(value) / static_cast<float>(1u << FractionBits);
    }

    // FTOI0/4/12/15: satura em int32; o resultado são os bits da pista.
    PS2_VU_KERNEL_INLINE int32_t ftoi(float value, float scale)
    {
        const double scaled = static_cast<double>(value) * static_cast<double>(scale);
        if (scaled >= static_cast<double>(std::numeric_limits<int32_t>::max()))
            return std::numeric_limits<int32_t>::max();
        if (scaled <= static_cast<double>(std::numeric_limits<int32_t>::min()))
            return std::numeric_limits<int32_t>::min();
        return static_cast<int32_t>(scaled);
    }

    // CLIP: seis bits (+x, -x, +y, -y, +z, -z) de fs.xyz contra |ft.w|, comparando bits crus
    // como inteiros ordenados; um w com expoente zero vale o maior denormal.
    PS2_VU_KERNEL_INLINE uint32_t clip(const float fs[4], float ftW)
    {
        uint32_t wBits = 0u;
        std::memcpy(&wBits, &ftW, sizeof(wBits));
        const int32_t limit = (wBits & 0x7F800000u) != 0u ? static_cast<int32_t>(wBits & 0x7FFFFFFFu) : 0x007FFFFF;

        const auto exceedsClipPlane = [limit](float value, uint32_t signMask)
        {
            uint32_t bits = 0u;
            std::memcpy(&bits, &value, sizeof(bits));
            bits ^= signMask;
            int32_t orderedBits = 0;
            std::memcpy(&orderedBits, &bits, sizeof(orderedBits));
            return orderedBits > limit;
        };

        uint32_t flags = 0u;
        for (uint32_t lane = 0; lane < 3u; ++lane)
        {
            if (exceedsClipPlane(fs[lane], 0x00000000u))
                flags |= 1u << (2u * lane);
            if (exceedsClipPlane(fs[lane], 0x80000000u))
                flags |= 2u << (2u * lane);
        }
        return flags;
    }

    // ------------------------------------------------------------------------
    // FDIV: Q com os bits I (0x10) e D (0x20) do status
    // ------------------------------------------------------------------------

    struct QResult
    {
        float q;
        uint32_t statusDi;
    };

    inline QResult div(float numerator, float denominator)
    {
        QResult out{0.0f, 0u};
        if (denominator == 0.0f)
        {
            out.statusDi = numerator == 0.0f ? 0x10u : 0x20u;
            out.q = std::signbit(numerator) != std::signbit(denominator)
                        ? -std::numeric_limits<float>::max()
                        : std::numeric_limits<float>::max();
        }
        else
        {
            out.q = numerator / denominator;
        }
        out.q = normalizeResult(out.q);
        return out;
    }

    inline QResult sqrt(float radicand)
    {
        return {normalizeResult(std::sqrt(std::fabs(radicand))), radicand < 0.0f ? 0x10u : 0u};
    }

    inline QResult rsqrt(float numerator, float radicand)
    {
        const float denominator = std::sqrt(std::fabs(radicand));
        QResult out{0.0f, radicand < 0.0f ? 0x10u : 0u};
        if (denominator != 0.0f)
            out.q = numerator / denominator;
        else
        {
            out.statusDi = numerator == 0.0f ? 0x10u : 0x20u;
            out.q = std::signbit(numerator)
                        ? -std::numeric_limits<float>::max()
                        : std::numeric_limits<float>::max();
        }
        out.q = normalizeResult(out.q);
        return out;
    }

    // ------------------------------------------------------------------------
    // EFU: o valor de P, já normalizado
    // ------------------------------------------------------------------------

    // A ordem em que o GCC contraía x*x + y*y + z*z: só y*y é arredondado à parte.
    inline float sumOfSquares(float x, float y, float z)
    {
        return std::fma(z, z, std::fma(x, x, y * y));
    }

    inline float atanPolynomial(float value)
    {
        constexpr float coefficients[] = {
            0.999999344348907f,
            -0.333298563957214f,
            0.199465364217758f,
            -0.13085337519646f,
            0.096420042216778f,
            -0.055909886956215f,
            0.021861229091883f,
            -0.004054057877511f};
        constexpr float quarterPi = 0.785398185253143f;

        const float squared = value * value;
        float polynomial = coefficients[7];
        for (int index = 6; index >= 0; --index)
            polynomial = std::fma(squared, polynomial, coefficients[index]);
        return std::fma(value, polynomial, quarterPi);
    }

    inline float esadd(float x, float y, float z) { return normalizeResult(sumOfSquares(x, y, z)); }

    inline float ersadd(float x, float y, float z)
    {
        const float sum = sumOfSquares(x, y, z);
        return normalizeResult(sum != 0.0f ? 1.0f / sum : sum);
    }

    inline float eleng(float x, float y, float z) { return normalizeResult(std::sqrt(sumOfSquares(x, y, z))); }

    inline float erleng(float x, float y, float z)
    {
        const float length = std::sqrt(sumOfSquares(x, y, z));
        return normalizeResult(length != 0.0f ? 1.0f / length : length);
    }

    // EATANxy e EATANxz: arco tangente de other / x.
    inline float eatanRatio(float x, float other)
    {
        return normalizeResult(x != 0.0f ? atanPolynomial(other / x) : 0.0f);
    }

    inline float esum(const float lanes[4])
    {
        float sum = 0.0f;
        for (uint32_t lane = 0; lane < 4u; ++lane)
            sum += lanes[lane];
        return normalizeResult(sum);
    }

    inline float ersqrt(float value)
    {
        float result = value;
        if (result >= 0.0f)
        {
            result = std::sqrt(result);
            if (result != 0.0f)
                result = 1.0f / result;
        }
        return normalizeResult(result);
    }

    inline float esqrt(float value) { return normalizeResult(value >= 0.0f ? std::sqrt(value) : value); }

    inline float esin(float value)
    {
        constexpr float coefficients[] = {
            1.0f,
            -0.166666567325592f,
            0.008333025500178f,
            -0.000198074136279f,
            0.000002601886990f};

        const float squared = value * value;
        float polynomial = coefficients[4];
        for (int index = 3; index >= 0; --index)
            polynomial = std::fma(squared, polynomial, coefficients[index]);
        return normalizeResult(value * polynomial);
    }

    inline float ercpr(float value) { return normalizeResult(value != 0.0f ? 1.0f / value : value); }

    inline float eatan(float value) { return normalizeResult(atanPolynomial(value)); }

    inline float eexp(float value)
    {
        constexpr float coefficients[] = {
            0.249998688697815f,
            0.031257584691048f,
            0.002591371303424f,
            0.000171562001924f,
            0.000005430199963f,
            0.000000690600018f};

        float polynomial = coefficients[5];
        for (int index = 4; index >= 0; --index)
            polynomial = std::fma(value, polynomial, coefficients[index]);
        polynomial = std::fma(value, polynomial, 1.0f);
        polynomial *= polynomial;
        polynomial *= polynomial;
        return normalizeResult(polynomial != 0.0f ? 1.0f / polynomial : std::numeric_limits<float>::max());
    }

    // ------------------------------------------------------------------------
    // Registrador R (gerador pseudoaleatório): mantissa de 23 bits com expoente de 1.0
    // ------------------------------------------------------------------------

    PS2_VU_KERNEL_INLINE uint32_t rnext(uint32_t r)
    {
        const uint32_t x = (r >> 4) & 1u;
        const uint32_t y = (r >> 22) & 1u;
        return (((r << 1) ^ x ^ y) & 0x007FFFFFu) | 0x3F800000u;
    }

    PS2_VU_KERNEL_INLINE uint32_t rinit(uint32_t bits) { return 0x3F800000u | (bits & 0x007FFFFFu); }

    PS2_VU_KERNEL_INLINE uint32_t rxor(uint32_t r, uint32_t bits) { return 0x3F800000u | ((r ^ bits) & 0x007FFFFFu); }
}

#endif
