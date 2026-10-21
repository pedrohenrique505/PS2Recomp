#ifndef PS2_VU_NATIVE_H
#define PS2_VU_NATIVE_H

#include "runtime/ps2_vu1.h"
#include "ps2_vu1_detail.h"
#include "ps2_vu_kernels.h"
#include <algorithm>
#include <bit>
#include <cstring>
#include <cmath>

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

// Id de uma imagem de código: índice no registro AOT, ou nenhum.
constexpr uint32_t kNoVuCodeImage = UINT32_MAX;

// Hash de 64 bits do conteúdo de uma imagem de código (tamanho múltiplo de 8). Só escolhe
// o candidato; a seleção confirma com comparação de bytes.
inline uint64_t vuCodeImageHash(const uint8_t *code, uint32_t size)
{
    uint64_t hash = 0x9e3779b97f4a7c15ull ^ size;
    for (uint32_t offset = 0; offset + 8u <= size; offset += 8u)
    {
        uint64_t word;
        std::memcpy(&word, code + offset, sizeof(word));
        hash = std::rotl(hash ^ (word * 0x87c37b91114253d5ull), 31) * 0x4cf5ad432745937full;
    }
    return hash ^ (hash >> 29);
}

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
            vu.m_state.i = normalizeVuOperand(immediate);
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

// O compilador C++ resolve opcode, registradores e máscaras nos programas AOT; os valores e
// flags vêm dos kernels de op. Operações ainda não especializadas usam execUpper.
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
        if constexpr (toAcc && (sel == 0x2fu || sel == 0x30u))
            return;
        else if constexpr (add || sub || madd || msub || mul || max || min)
        {
            float left[4];
            float right[4];
            float addend[4]{};
            for (uint32_t c = 0; c < 4u; ++c)
            {
                left[c] = normalizeVuOperand(vu.m_state.vf[fs][c]);
                if constexpr (sel <= 0x1bu)
                    right[c] = normalizeVuOperand(vu.m_state.vf[ft][sel & 3u]);
                else if constexpr (sel == 0x1cu || sel == 0x20u || sel == 0x21u || sel == 0x24u || sel == 0x25u)
                    right[c] = normalizeVuOperand(vu.m_state.q);
                else if constexpr (sel == 0x1du || sel == 0x1eu || sel == 0x1fu || sel == 0x22u || sel == 0x23u || sel == 0x26u || sel == 0x27u)
                    right[c] = normalizeVuOperand(vu.m_state.i);
                else
                    right[c] = normalizeVuOperand(vu.m_state.vf[ft][c]);
                if constexpr (madd || msub)
                    addend[c] = normalizeVuOperand(vu.m_state.acc[c]);
            }
            if constexpr (max || min)
            {
                float result[4];
                for (uint32_t c = 0; c < 4u; ++c)
                    result[c] = max ? VUKernels::maximum(left[c], right[c]) : VUKernels::minimum(left[c], right[c]);
                vu.applyDest(vu.m_state.vf[fd], result, dest);
            }
            else
            {
                using VUKernels::FmacOp;
                constexpr FmacOp kind = add ? FmacOp::Add : sub ? FmacOp::Sub : mul ? FmacOp::Mul
                                                          : madd ? FmacOp::Madd : FmacOp::Msub;
                const VUKernels::FmacLanes lanes = VUKernels::fmac<kind>(left, right, addend, dest);
                vu.updateFmacFlags(lanes.flags, dest, lanes.productSticky);
                if constexpr (toAcc)
                    vu.applyDestAcc(lanes.value, dest);
                else
                    vu.applyDest(vu.m_state.vf[fd], lanes.value, dest);
            }
        }
        else
            vu.execUpper(Instr);
    }
};

#endif
