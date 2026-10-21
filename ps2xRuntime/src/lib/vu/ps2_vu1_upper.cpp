#include "runtime/ps2_vu1.h"
#include "ps2_vu1_detail.h"
#include "ps2_vu_kernels.h"

#include <algorithm>
#include <cstring>

namespace
{
    using VUKernels::FmacOp;

    // Formas FMAC pelo seletor comum às escritas em VF (bits 5:0) e em ACC (grupo especial).
    bool fmacOpFor(uint8_t sel, bool toAcc, FmacOp &op)
    {
        if (sel <= 0x03u || sel == 0x20u || sel == 0x22u || sel == 0x28u)
            op = FmacOp::Add;
        else if ((sel >= 0x04u && sel <= 0x07u) || sel == 0x24u || sel == 0x26u || sel == 0x2Cu)
            op = FmacOp::Sub;
        else if ((sel >= 0x08u && sel <= 0x0Bu) || sel == 0x21u || sel == 0x23u || sel == 0x29u)
            op = FmacOp::Madd;
        else if ((sel >= 0x0Cu && sel <= 0x0Fu) || sel == 0x25u || sel == 0x27u || sel == 0x2Du)
            op = FmacOp::Msub;
        else if ((sel >= 0x18u && sel <= 0x1Cu) || sel == 0x1Eu || sel == 0x2Au)
            op = FmacOp::Mul;
        else if (sel == 0x2Eu)
            op = toAcc ? FmacOp::Opmula : FmacOp::Opmsub;
        else
            return false;
        return true;
    }
}

// ============================================================================
// Upper instructions (FMAC pipeline)
// ============================================================================
void VU1Interpreter::execUpper(uint32_t instr)
{
    const uint8_t dest = DEST(instr);
    const uint8_t ft = FT(instr);
    const uint8_t fs = FS(instr);
    const uint8_t fd = FD(instr);
    const uint8_t op = instr & 0x3F;

    // Upper special group (low op 0x3C..0x3F) writes ACC, or FT for ITOF/FTOI/ABS.
    // Like lower1 special, the real selector is not just bits 5:0.  Dobie decodes:
    //   op = (instr & 0x3) | ((instr >> 4) & 0x7C)
    const bool toAcc = op >= 0x3Cu;
    const uint8_t sel = toAcc ? static_cast<uint8_t>((instr & 3u) | ((instr >> 4) & 0x7Cu)) : op;

    // NOP não lê operandos; evita normalizar quatorze valores sem uso.
    if (toAcc && (sel == 0x2Fu || sel == 0x30u))
        return;
    if (!toAcc && op >= 0x30u)
    {
        reportReservedInstruction(true, instr);
        return;
    }

    float vs[4];
    float vt[4];
    float acc[4];
    for (uint32_t component = 0; component < 4u; ++component)
    {
        vs[component] = normalizeVuOperand(m_state.vf[fs][component]);
        vt[component] = normalizeVuOperand(m_state.vf[ft][component]);
        acc[component] = normalizeVuOperand(m_state.acc[component]);
    }

    // Operando direito por pista: broadcast de vt, Q, I ou vt inteiro.
    float right[4];
    if (sel <= 0x1Bu)
        std::fill(right, right + 4, vt[sel & 3u]);
    else if (sel == 0x1Cu || sel == 0x20u || sel == 0x21u || sel == 0x24u || sel == 0x25u)
        std::fill(right, right + 4, normalizeVuOperand(m_state.q));
    else if (sel == 0x1Du || sel == 0x1Eu || sel == 0x1Fu || (sel >= 0x22u && sel <= 0x27u))
        std::fill(right, right + 4, normalizeVuOperand(m_state.i));
    else
        std::copy(vt, vt + 4, right);

    FmacOp fmacOp;
    if (fmacOpFor(sel, toAcc, fmacOp))
    {
        VUKernels::FmacLanes lanes;
        if (sel == 0x2Eu)
        {
            const float crossLeft[4] = {vs[1], vs[2], vs[0], vs[3]};
            const float crossRight[4] = {vt[2], vt[0], vt[1], vt[3]};
            lanes = VUKernels::fmac(fmacOp, crossLeft, crossRight, acc, dest);
        }
        else
            lanes = VUKernels::fmac(fmacOp, vs, right, acc, dest);
        updateFmacFlags(lanes.flags, dest, lanes.productSticky);
        applyDest(toAcc ? m_state.acc : m_state.vf[fd], lanes.value, dest);
        return;
    }

    float result[4];
    if (!toAcc)
    {
        const bool max = (sel >= 0x10u && sel <= 0x13u) || sel == 0x1Du || sel == 0x2Bu;
        const bool mini = (sel >= 0x14u && sel <= 0x17u) || sel == 0x1Fu || sel == 0x2Fu;
        if (!max && !mini)
        {
            reportReservedInstruction(true, instr);
            return;
        }
        for (uint32_t c = 0; c < 4u; ++c)
            result[c] = max ? VUKernels::maximum(vs[c], right[c]) : VUKernels::minimum(vs[c], right[c]);
        applyDest(m_state.vf[fd], result, dest);
        return;
    }

    if (sel >= 0x10u && sel <= 0x13u) // ITOF0/4/12/15
    {
        for (uint32_t c = 0; c < 4u; ++c)
        {
            int32_t iv;
            std::memcpy(&iv, &m_state.vf[fs][c], 4);
            switch (sel)
            {
            case 0x10u: result[c] = VUKernels::itof<0>(iv); break;
            case 0x11u: result[c] = VUKernels::itof<4>(iv); break;
            case 0x12u: result[c] = VUKernels::itof<12>(iv); break;
            default: result[c] = VUKernels::itof<15>(iv); break;
            }
        }
        applyDest(m_state.vf[ft], result, dest);
        return;
    }
    if (sel >= 0x14u && sel <= 0x17u) // FTOI0/4/12/15
    {
        static constexpr float scales[4] = {1.0f, 16.0f, 4096.0f, 32768.0f};
        for (uint32_t c = 0; c < 4u; ++c)
        {
            const int32_t iv = VUKernels::ftoi(vs[c], scales[sel & 3u]);
            std::memcpy(&result[c], &iv, 4);
        }
        applyDest(m_state.vf[ft], result, dest);
        return;
    }
    if (sel == 0x1Du) // ABS
    {
        for (uint32_t c = 0; c < 4u; ++c)
            result[c] = VUKernels::absolute(vs[c]);
        applyDest(m_state.vf[ft], result, dest);
        return;
    }
    if (sel == 0x1Fu) // CLIP
    {
        queueClip(VUKernels::clip(m_state.vf[fs], m_state.vf[ft][3]));
        return;
    }
    reportReservedInstruction(true, instr);
}
