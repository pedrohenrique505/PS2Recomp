#ifndef PS2_VU_NATIVE_H
#define PS2_VU_NATIVE_H

#include "runtime/ps2_vu1.h"
#include "ps2_vu1_detail.h"
#include <algorithm>
#include <bit>
#include <cstring>
#include <cmath>
#include <limits>

// Interface interna dos blocos AOT. Código gerado continua fora do runtime versionado.
using VUNativeRunner = bool (*)(VU1Interpreter &, uint32_t, uint8_t *, uint32_t,
                               GS &, PS2Memory *, uint64_t);
struct VUNativeProgram
{
    const uint8_t *code;
    uint32_t codeSize;
    VU1Interpreter::Unit unit;
    VUNativeRunner run;
};

struct VUNativeAccess
{
    using Pair = VU1Interpreter::DecodedInstructionPair;
    using Usage = VU1Interpreter::InstructionUsage;
    using Pipeline = VU1Interpreter::Pipeline;
    using VfAccess = VU1Interpreter::VfAccess;
    enum class Result { Next, Ended, Stopped };

    struct InterpretedUpper
    {
        static void run(VU1Interpreter &vu, uint32_t instr) { vu.execUpper(instr); }
    };

    static Pair decode(VU1Interpreter &vu, const uint8_t *code, uint32_t pc)
    {
        return vu.decodeInstructionPair(code, pc);
    }

    template <uint32_t Instr> struct NativeUpper;

    static inline uint64_t currentCycle(const VU1Interpreter &vu) { return vu.m_cycle; }
    static constexpr uint32_t accForwardLatency() { return VU1Interpreter::kAccForwardLatency; }

    // Guardas de entrada: somente dependências ainda não produzidas pelo bloco.
    static inline bool vfReadyBy(const VU1Interpreter &vu, uint32_t reg, uint32_t lane, uint64_t cycle)
    { return vu.m_vfReady[reg][lane] <= cycle; }
    static inline bool viReadyBy(const VU1Interpreter &vu, uint32_t reg, uint64_t cycle)
    { return vu.m_viReady[reg] <= cycle; }
    static inline bool accReadyBy(const VU1Interpreter &vu, uint32_t lane, uint64_t cycle)
    { return vu.m_accReady[lane] <= cycle; }

#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    static inline uint64_t pairReady(const VU1Interpreter &vu, const Pair &decoded)
    {
        uint64_t ready = vu.m_cycle;
        const Usage *usages[2] = {
            &decoded.upperUsage,
            &decoded.lowerUsage};
        for (const Usage *usage : usages)
        {
            if (!usage)
                continue;
            for (uint32_t index = 0; index < usage->vfReadCount; ++index)
            {
                const VfAccess &access = usage->vfRead[index];
                for (uint32_t component = 0; component < 4u; ++component)
                {
                    if ((access.lanes & (1u << (3u - component))) != 0u)
                        ready = std::max(ready, vu.m_vfReady[access.reg][component]);
                }
            }
            // VI0 é constante; percorre somente os registradores realmente lidos.
            uint32_t reads = usage->viRead & ~1u;
            while (reads != 0u)
            {
                const uint32_t reg = std::countr_zero(reads);
                ready = std::max(ready, vu.m_viReady[reg]);
                reads &= reads - 1u;
            }
            for (uint32_t component = 0; component < 4u; ++component)
            {
                if ((usage->accRead & (1u << (3u - component))) != 0u)
                    ready = std::max(ready, vu.m_accReady[component]);
            }
        }

        if (decoded.lowerUsage.pipeline == VU1Interpreter::PipelineFdiv && vu.m_fdiv.valid)
            ready = std::max(ready, vu.m_fdiv.readyCycle);
        if (decoded.lowerUsage.pipeline == VU1Interpreter::PipelineEfu)
            ready = std::max(ready, vu.m_efuResourceReady);
        if (decoded.lowerUsage.waitQ && vu.m_fdiv.valid)
            ready = std::max(ready, vu.m_fdiv.readyCycle);
        if (decoded.lowerUsage.waitP)
        {
            for (const VU1Interpreter::ScalarPipelineEntry &entry : vu.m_efu)
                if (entry.valid)
                    ready = std::max(ready, entry.readyCycle);
        }
        if (decoded.lowerUsage.pipeline == VU1Interpreter::PipelineXgkick && vu.m_xgkick.active)
            ready = std::max(ready, vu.m_cycle + 1u);
        return ready;
    }

#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    static inline void markWrites(VU1Interpreter &vu, const Pair &decoded)
    {
        const VfAccess lowerWrite = decoded.lowerUsage.vfWrite;
        if (lowerWrite.reg != 0u &&
            decoded.suppressedLowerVf != lowerWrite.reg)
        {
            const uint32_t latency = decoded.lowerUsage.vfLatency != 0u
                                         ? decoded.lowerUsage.vfLatency
                                         : decoded.lowerUsage.latency;
            for (uint32_t component = 0; component < 4u; ++component)
            {
                if ((lowerWrite.lanes & (1u << (3u - component))) != 0u)
                    vu.m_vfReady[lowerWrite.reg][component] = vu.m_cycle + latency;
            }
        }

        const VfAccess upperWrite = decoded.upperUsage.vfWrite;
        if (upperWrite.reg != 0u)
        {
            const uint32_t latency = decoded.upperUsage.vfLatency != 0u
                                         ? decoded.upperUsage.vfLatency
                                         : decoded.upperUsage.latency;
            for (uint32_t component = 0; component < 4u; ++component)
            {
                if ((upperWrite.lanes & (1u << (3u - component))) != 0u)
                    vu.m_vfReady[upperWrite.reg][component] = vu.m_cycle + latency;
            }
        }

        uint32_t writes = decoded.lowerUsage.viWrite & ~1u;
        while (writes != 0u)
        {
            const uint32_t reg = std::countr_zero(writes);
            vu.m_viReady[reg] = vu.m_cycle + (decoded.lowerUsage.viLatency != 0u ? decoded.lowerUsage.viLatency : decoded.lowerUsage.latency);
            writes &= writes - 1u;
        }
        for (uint32_t component = 0; component < 4u; ++component)
        {
            if ((decoded.upperUsage.accWrite & (1u << (3u - component))) != 0u)
                vu.m_accReady[component] = vu.m_cycle + VU1Interpreter::kAccForwardLatency;
        }
    }

#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    static inline bool exactFmac(const VU1Interpreter &vu, uint32_t upper,
                                 uint32_t component, long double &result)
    {
        return exactFmacIn(vu, upper, component, result);
    }

    // A mesma fórmula em outra precisão. Em double o produto de dois floats é exato e a
    // soma arredondada preserva sinal e zero; normalizeExactFast usa isso para classificar.
    template <typename Real>
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    static inline bool exactFmacIn(const VU1Interpreter &vu, uint32_t upper,
                                   uint32_t component, Real &result)
    {
        const uint8_t op = static_cast<uint8_t>(upper & 0x3Fu);
        const uint8_t special = op >= 0x3Cu
                                    ? static_cast<uint8_t>((upper & 3u) | ((upper >> 4) & 0x7Cu))
                                    : 0xFFu;
        const uint8_t fs = FS(upper);
        const uint8_t ft = FT(upper);

        const auto operand = [](float value)
        {
            return static_cast<Real>(normalizeVuOperand(value));
        };
        const auto vs = [&](uint32_t lane)
        {
            return operand(vu.m_state.vf[fs][lane]);
        };
        const auto vt = [&](uint32_t lane)
        {
            return operand(vu.m_state.vf[ft][lane]);
        };
        const auto acc = [&](uint32_t lane)
        {
            return operand(vu.m_state.acc[lane]);
        };

        const Real q = operand(vu.m_state.q);
        const Real i = operand(vu.m_state.i);

        if (op < 0x3Cu)
        {
            if (op <= 0x03u)
                result = vs(component) + vt(op & 3u);
            else if (op <= 0x07u)
                result = vs(component) - vt(op & 3u);
            else if (op <= 0x0Bu)
                result = acc(component) + vs(component) * vt(op & 3u);
            else if (op <= 0x0Fu)
                result = acc(component) - vs(component) * vt(op & 3u);
            else if (op >= 0x18u && op <= 0x1Bu)
                result = vs(component) * vt(op & 3u);
            else
            {
                switch (op)
                {
                case 0x1Cu:
                    result = vs(component) * q;
                    break;
                case 0x1Eu:
                    result = vs(component) * i;
                    break;
                case 0x20u:
                    result = vs(component) + q;
                    break;
                case 0x21u:
                    result = acc(component) + vs(component) * q;
                    break;
                case 0x22u:
                    result = vs(component) + i;
                    break;
                case 0x23u:
                    result = acc(component) + vs(component) * i;
                    break;
                case 0x24u:
                    result = vs(component) - q;
                    break;
                case 0x25u:
                    result = acc(component) - vs(component) * q;
                    break;
                case 0x26u:
                    result = vs(component) - i;
                    break;
                case 0x27u:
                    result = acc(component) - vs(component) * i;
                    break;
                case 0x28u:
                    result = vs(component) + vt(component);
                    break;
                case 0x29u:
                    result = acc(component) + vs(component) * vt(component);
                    break;
                case 0x2Au:
                    result = vs(component) * vt(component);
                    break;
                case 0x2Cu:
                    result = vs(component) - vt(component);
                    break;
                case 0x2Du:
                    result = acc(component) - vs(component) * vt(component);
                    break;
                case 0x2Eu:
                {
                    static constexpr uint8_t left[4] = {1u, 2u, 0u, 3u};
                    static constexpr uint8_t right[4] = {2u, 0u, 1u, 3u};
                    result = component == 3u
                                 ? Real(0)
                                 : acc(component) - vs(left[component]) * vt(right[component]);
                    break;
                }
                default:
                    return false;
                }
            }
            return true;
        }

        if (special <= 0x03u)
            result = vs(component) + vt(special & 3u);
        else if (special <= 0x07u)
            result = vs(component) - vt(special & 3u);
        else if (special <= 0x0Bu)
            result = acc(component) + vs(component) * vt(special & 3u);
        else if (special <= 0x0Fu)
            result = acc(component) - vs(component) * vt(special & 3u);
        else if (special >= 0x18u && special <= 0x1Bu)
            result = vs(component) * vt(special & 3u);
        else
        {
            switch (special)
            {
            case 0x1Cu:
                result = vs(component) * q;
                break;
            case 0x1Eu:
                result = vs(component) * i;
                break;
            case 0x20u:
                result = vs(component) + q;
                break;
            case 0x21u:
                result = acc(component) + vs(component) * q;
                break;
            case 0x22u:
                result = vs(component) + i;
                break;
            case 0x23u:
                result = acc(component) + vs(component) * i;
                break;
            case 0x24u:
                result = vs(component) - q;
                break;
            case 0x25u:
                result = acc(component) - vs(component) * q;
                break;
            case 0x26u:
                result = vs(component) - i;
                break;
            case 0x27u:
                result = acc(component) - vs(component) * i;
                break;
            case 0x28u:
                result = vs(component) + vt(component);
                break;
            case 0x29u:
                result = acc(component) + vs(component) * vt(component);
                break;
            case 0x2Au:
                result = vs(component) * vt(component);
                break;
            case 0x2Cu:
                result = vs(component) - vt(component);
                break;
            case 0x2Du:
                result = acc(component) - vs(component) * vt(component);
                break;
            case 0x2Eu:
            {
                static constexpr uint8_t left[4] = {1u, 2u, 0u, 3u};
                static constexpr uint8_t right[4] = {2u, 0u, 1u, 3u};
                result = component == 3u
                             ? Real(0)
                             : vs(left[component]) * vt(right[component]);
                break;
            }
            default:
                return false;
            }
        }
        return true;
    }

#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    // Mesmo resultado de normalizeExact a partir do valor calculado em double, ou false quando
    // ele está perto de FLT_MAX/FLT_MIN e só o long double de referência decide.
    static inline bool normalizeExactFast(float &value, double exactResult, uint8_t &flags)
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
    static inline uint8_t exactFlags(double exactResult)
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

    static inline uint8_t normalizeExact(float &value, long double exactResult)
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

#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    static inline uint32_t productSticky(const VU1Interpreter &vu, uint32_t upper, uint8_t dest)
    {
        uint32_t extraSticky = 0u;
        const uint8_t op = static_cast<uint8_t>(upper & 0x3Fu);
        const uint8_t special = op >= 0x3Cu ? static_cast<uint8_t>((upper & 3u) | ((upper >> 4) & 0x7Cu)) : 0xFFu;
        const bool productSum =
            (op >= 0x08u && op <= 0x0Fu) ||
            op == 0x21u || op == 0x23u || op == 0x25u || op == 0x27u ||
            op == 0x29u || op == 0x2Du || op == 0x2Eu ||
            (special >= 0x08u && special <= 0x0Fu) ||
            special == 0x21u || special == 0x23u || special == 0x25u ||
            special == 0x27u || special == 0x29u || special == 0x2Du;
        if (!productSum)
            return 0u;

        const uint8_t fs = FS(upper);
        const uint8_t ft = FT(upper);
        for (uint32_t component = 0; component < 4u; ++component)
        {
            if ((dest & (1u << (3u - component))) == 0u)
                continue;
            static constexpr uint8_t crossLeft[4] = {1u, 2u, 0u, 3u};
            static constexpr uint8_t crossRight[4] = {2u, 0u, 1u, 3u};
            const uint8_t leftComponent = op == 0x2Eu ? crossLeft[component] : static_cast<uint8_t>(component);
            const float left = normalizeVuOperand(vu.m_state.vf[fs][leftComponent]);
            float right = 0.0f;
            if ((op >= 0x08u && op <= 0x0Fu) || (special >= 0x08u && special <= 0x0Fu))
            {
                right = normalizeVuOperand(vu.m_state.vf[ft][(op >= 0x08u && op <= 0x0Fu ? op : special) & 3u]);
            }
            else if (op == 0x21u || op == 0x25u || special == 0x21u || special == 0x25u)
            {
                right = normalizeVuOperand(vu.m_state.q);
            }
            else if (op == 0x23u || op == 0x27u || special == 0x23u || special == 0x27u)
            {
                right = normalizeVuOperand(vu.m_state.i);
            }
            else if (op == 0x2Eu)
            {
                right = normalizeVuOperand(vu.m_state.vf[ft][crossRight[component]]);
            }
            else
            {
                right = normalizeVuOperand(vu.m_state.vf[ft][component]);
            }

            // O produto de dois floats é exato em double.
            const uint8_t productFlags = exactFlags(static_cast<double>(left) * static_cast<double>(right));
            // Product-sum instructions report Z/S/U/O from the add/subtract result
            // as current flags, while every product condition accumulates into the
            // corresponding sticky flag.
            extraSticky |= productFlags & 0xFu;
        }
        return extraSticky;
    }

    template <typename Upper>
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#endif
    static inline Result issue(VU1Interpreter &vu, const Pair &decoded, uint32_t codeSize,
                        uint8_t *vuData, uint32_t dataSize, GS &gs,
                        PS2Memory *memory, uint64_t budgetEnd,
                        uint64_t scheduledReady = UINT64_MAX)
    {
        if (vu.m_cycle >= budgetEnd || vu.m_stopRequested || vu.m_state.pc + 8u > codeSize)
            return Result::Stopped;
        vu.commitReadyPipelines();
        bool programEnded = false;
        if (decoded.upperUsage.reserved || decoded.lowerUsage.reserved)
        {
            vu.reportReservedInstruction(decoded.upperUsage.reserved, decoded.upperUsage.reserved ? decoded.upper : decoded.lower);
            return Result::Stopped;
        }

        // O calendário AOT só vale após as guardas de entrada e até o próximo
        // desvio. As filas continuam materializadas em cada fronteira de ciclo.
        uint64_t readyCycle = scheduledReady != UINT64_MAX ? scheduledReady : pairReady(vu, decoded);
        while (readyCycle > vu.m_cycle)
        {
            if (readyCycle >= budgetEnd)
            {
                vu.advanceTo(budgetEnd);
                break;
            }
            vu.advanceTo(readyCycle);
            if (scheduledReady != UINT64_MAX)
                break;
            readyCycle = pairReady(vu, decoded);
        }
        if (vu.m_cycle >= budgetEnd)
            return Result::Stopped;

        uint8_t writtenVi = 0u;
        int32_t oldVi = 0;
        const uint32_t viWrites = decoded.lowerUsage.viWrite & ~1u;
        if (viWrites != 0u)
        {
            writtenVi = static_cast<uint8_t>(std::countr_zero(viWrites));
            oldVi = vu.m_state.vi[writtenVi];
        }

        const VfAccess upperWrite = decoded.upperUsage.vfWrite;
        const VfAccess lowerWrite = decoded.lowerUsage.vfWrite;
        const bool hasUpperWrite = upperWrite.reg != 0u;
        const bool hasLowerWrite = lowerWrite.reg != 0u && decoded.suppressedLowerVf != lowerWrite.reg;
        const bool hasDistinctLowerWrite = hasLowerWrite && (!hasUpperWrite || lowerWrite.reg != upperWrite.reg);
        float oldUpperVf[4]{};
        float oldLowerVf[4]{};
        float oldAcc[4]{};
        if (hasUpperWrite)
            std::memcpy(oldUpperVf, vu.m_state.vf[upperWrite.reg], sizeof(oldUpperVf));
        if (hasDistinctLowerWrite)
            std::memcpy(oldLowerVf, vu.m_state.vf[lowerWrite.reg], sizeof(oldLowerVf));
        if (decoded.upperUsage.accWrite != 0u)
            std::memcpy(oldAcc, vu.m_state.acc, sizeof(oldAcc));

        if (decoded.iBit)
        {
            Upper::run(vu, decoded.upper);
            float immediate = 0.0f;
            std::memcpy(&immediate, &decoded.lower, sizeof(immediate));
            vu.m_state.i = vu.normalizeOperand(immediate);
        }
        else if (decoded.upperVfShadowReg != 0u)
        {
            float oldVf[4]{};
            float upperVf[4]{};
            std::memcpy(oldVf,
                        vu.m_state.vf[decoded.upperVfShadowReg],
                        sizeof(oldVf));
            Upper::run(vu, decoded.upper);
            std::memcpy(upperVf,
                        vu.m_state.vf[decoded.upperVfShadowReg],
                        sizeof(upperVf));
            std::memcpy(vu.m_state.vf[decoded.upperVfShadowReg],
                        oldVf,
                        sizeof(oldVf));
            vu.execLower(decoded.lower, vuData, dataSize, gs, memory, decoded.upper);
            std::memcpy(vu.m_state.vf[decoded.upperVfShadowReg],
                        upperVf,
                        sizeof(upperVf));
        }
        else
        {
            Upper::run(vu, decoded.upper);
            vu.execLower(decoded.lower, vuData, dataSize, gs, memory, decoded.upper);
        }

        vu.m_viBranchBackupValid = false;

        // Escrita imediata (ver VU1Interpreter::ImmediateWrite), na mesma ordem em que a
        // referência enfileirava. A escrita do upper ocupa só as pistas dele no registrador.
        if (hasUpperWrite)
        {
            float *reg = vu.m_state.vf[upperWrite.reg];
            for (uint32_t c = 0; c < 4u; ++c)
                if ((upperWrite.lanes & (8u >> c)) == 0u)
                    reg[c] = oldUpperVf[c];
            const uint32_t latency = decoded.upperUsage.vfLatency != 0u ? decoded.upperUsage.vfLatency : decoded.upperUsage.latency;
            vu.logImmediateWrite(VU1Interpreter::ImmediateWrite::Kind::Vf, upperWrite.reg, upperWrite.lanes,
                                 oldUpperVf, reg, 0, 0, latency);
        }
        if (hasDistinctLowerWrite)
        {
            const uint32_t latency = decoded.lowerUsage.vfLatency != 0u ? decoded.lowerUsage.vfLatency : decoded.lowerUsage.latency;
            vu.logImmediateWrite(VU1Interpreter::ImmediateWrite::Kind::Vf, lowerWrite.reg, lowerWrite.lanes,
                                 oldLowerVf, vu.m_state.vf[lowerWrite.reg], 0, 0, latency);
        }
        if (decoded.upperUsage.accWrite != 0u)
            vu.logImmediateWrite(VU1Interpreter::ImmediateWrite::Kind::Acc, 0, decoded.upperUsage.accWrite,
                                 oldAcc, vu.m_state.acc, 0, 0, VU1Interpreter::kAccForwardLatency);
        if (writtenVi != 0u)
        {
            const int32_t newVi = static_cast<int16_t>(vu.m_state.vi[writtenVi]);
            vu.m_state.vi[writtenVi] = newVi;
            const uint32_t latency = decoded.lowerUsage.viLatency != 0u ? decoded.lowerUsage.viLatency : decoded.lowerUsage.latency;
            vu.logImmediateWrite(VU1Interpreter::ImmediateWrite::Kind::Vi, writtenVi, 0, nullptr, nullptr,
                                 oldVi, newVi, latency);
        }

        markWrites(vu, decoded);
        if (writtenVi != 0u && decoded.lowerUsage.delaysNextBranchRead)
            vu.recordViWriteForBranch(writtenVi, oldVi);

        vu.m_state.vf[0][0] = 0.0f;
        vu.m_state.vf[0][1] = 0.0f;
        vu.m_state.vf[0][2] = 0.0f;
        vu.m_state.vf[0][3] = 1.0f;
        vu.m_state.vi[0] = 0;

        uint32_t nextPc = vu.m_state.pc + 8u;
        if (nextPc >= codeSize)
            nextPc = 0u;
        vu.m_state.pc = nextPc;

        if (vu.m_state.branchPending)
        {
            if (vu.m_state.branchDelay == 0u)
            {
                vu.m_state.pc = vu.m_state.branchTarget & vu.microAddressMask();
                vu.m_state.branchPending = false;
            }
            else
            {
                --vu.m_state.branchDelay;
            }
        }

        const bool dHalt = decoded.dBit && vu.m_state.dBitEnabled;
        const bool tHalt = decoded.tBit && vu.m_state.tBitEnabled;
        const bool haltBit = dHalt || tHalt;
        const bool haltBranch = haltBit && decoded.lowerUsage.pipeline == VU1Interpreter::PipelineBranch;

        if (vu.m_state.haltAfterDelaySlot)
        {
            vu.m_state.stoppedByD = vu.m_pendingHaltD;
            vu.m_state.stoppedByT = vu.m_pendingHaltT;
            programEnded = true;
        }
        else if (vu.m_state.ebit)
            programEnded = true;
        else if (haltBit && !haltBranch)
        {
            vu.m_state.stoppedByD = dHalt;
            vu.m_state.stoppedByT = tHalt;
            programEnded = true;
        }
        else if (decoded.eBit)
            vu.m_state.ebit = true;
        else if (haltBranch)
        {
            vu.m_state.haltAfterDelaySlot = true;
            vu.m_pendingHaltD = dHalt;
            vu.m_pendingHaltT = tHalt;
        }

        // advanceOneCycle sem a chamada quando nada está pendente.
        ++vu.m_cycle;
        vu.m_state.cycles = vu.m_cycle;
        if (vu.m_cycle >= vu.m_nextReady)
            vu.commitReadyPipelines();
        if (vu.m_xgkick.active)
            vu.progressXgkick();
        return programEnded ? Result::Ended : Result::Next;
    }
};

// O compilador C++ resolve opcode, registradores e máscaras nos programas AOT.
// Operações ainda não especializadas usam a implementação de referência.
template <uint32_t Instr>
struct VUNativeAccess::NativeUpper
{
    static void run(VU1Interpreter &vu, uint32_t)
    {
        constexpr uint32_t op = Instr & 0x3fu;
        constexpr bool toAcc = op >= 0x3cu;
        constexpr uint32_t sel = toAcc ? ((Instr & 3u) | ((Instr >> 4) & 0x7cu)) : op;
        constexpr uint8_t dest = (Instr >> 21) & 15u;
        constexpr uint32_t fs = (Instr >> 11) & 31u;
        constexpr uint32_t ft = (Instr >> 16) & 31u;
        constexpr uint32_t fd = (Instr >> 6) & 31u;
        constexpr bool add = sel <= 3u || sel == 0x20u || sel == 0x22u || sel == 0x28u;
        constexpr bool sub = (sel >= 4u && sel <= 7u) || sel == 0x24u || sel == 0x26u || sel == 0x2cu;
        constexpr bool madd = (sel >= 8u && sel <= 0xbu) || sel == 0x21u || sel == 0x23u || sel == 0x29u;
        constexpr bool msub = (sel >= 0xcu && sel <= 0xfu) || sel == 0x25u || sel == 0x27u || sel == 0x2du;
        constexpr bool mul = (sel >= 0x18u && sel <= 0x1cu) || sel == 0x1eu || sel == 0x2au;
        constexpr bool max = !toAcc && ((sel >= 0x10u && sel <= 0x13u) || sel == 0x1du || sel == 0x2bu);
        constexpr bool min = !toAcc && ((sel >= 0x14u && sel <= 0x17u) || sel == 0x1fu || sel == 0x2fu);
        vu.m_currentUpperInstruction = Instr;
        if constexpr (toAcc && (sel == 0x2fu || sel == 0x30u))
            return;
        else if constexpr (add || sub || madd || msub || mul || max || min)
        {
            float result[4];
            for (uint32_t c = 0; c < 4u; ++c)
            {
                const float left = normalizeVuOperand(vu.m_state.vf[fs][c]);
                float right;
                if constexpr (sel <= 0x1bu)
                    right = normalizeVuOperand(vu.m_state.vf[ft][sel & 3u]);
                else if constexpr (sel == 0x1cu || sel == 0x20u || sel == 0x21u || sel == 0x24u || sel == 0x25u)
                    right = normalizeVuOperand(vu.m_state.q);
                else if constexpr (sel == 0x1du || sel == 0x1eu || sel == 0x1fu || sel == 0x22u || sel == 0x23u || sel == 0x26u || sel == 0x27u)
                    right = normalizeVuOperand(vu.m_state.i);
                else
                    right = normalizeVuOperand(vu.m_state.vf[ft][c]);
                if constexpr (add) result[c] = left + right;
                else if constexpr (sub) result[c] = left - right;
                else if constexpr (mul) result[c] = left * right;
                else if constexpr (madd) result[c] = normalizeVuOperand(vu.m_state.acc[c]) + left * right;
                else if constexpr (msub) result[c] = normalizeVuOperand(vu.m_state.acc[c]) - left * right;
                else if constexpr (max) result[c] = left > right ? left : right;
                else result[c] = left < right ? left : right;
            }
            if constexpr (!(max || min))
            {
                uint8_t flags[4]{};
                for (uint32_t component = 0; component < 4u; ++component)
                {
                    if ((dest & (1u << (3u - component))) == 0u)
                        continue;
                    double fast = 0.0;
                    long double exact = 0.0L;
                    if (VUNativeAccess::exactFmacIn(vu, Instr, component, fast))
                    {
                        if (!VUNativeAccess::normalizeExactFast(result[component], fast, flags[component]))
                        {
                            VUNativeAccess::exactFmac(vu, Instr, component, exact);
                            flags[component] = VUNativeAccess::normalizeExact(result[component], exact);
                        }
                    }
                    else
                    {
                        uint32_t laneFlags = 0u;
                        result[component] = vu.normalizeResult(result[component], laneFlags);
                        flags[component] = static_cast<uint8_t>(laneFlags);
                    }
                }
                vu.updateFmacFlags(flags, dest, VUNativeAccess::productSticky(vu, Instr, dest));
            }
            if constexpr (toAcc)
                vu.applyDestAcc(result, dest);
            else
                vu.applyDest(vu.m_state.vf[fd], result, dest);
        }
        else
            vu.execUpper(Instr);
    }
};

#endif
