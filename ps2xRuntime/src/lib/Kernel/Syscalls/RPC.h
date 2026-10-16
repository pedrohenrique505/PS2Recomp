#pragma once

#include "ps2_syscalls.h"

namespace ps2_syscalls
{
    void SifStopModule(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifLoadModule(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifInitRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifBindRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifCallRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifRegisterRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifCheckStatRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifSetRpcQueue(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifRemoveRpcQueue(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void SifRemoveRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void sceSifCallRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void sceSifSendCmd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);
    void sceRpcGetPacket(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime);

    // Chamado pelo IOP quando um RPC devolvido como deferred termina (resultado já no buffer do EE):
    // libera o cliente, sinaliza o semáforo, acorda quem chamou em modo bloqueante e agenda a end function.
    void completeDeferredRpc(PS2Runtime *runtime, const ps2x::iop::RpcRequest &request);
}
