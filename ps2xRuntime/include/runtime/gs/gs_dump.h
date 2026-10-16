#pragma once
// PS2X_GSDUMP="vsync_inicial:frames:arquivo.gs": grava tudo que entra no GS no formato de GS dump do
// PCSX2 (state_version 9), reproduzível pelo tools/gsplay e pelo próprio PCSX2.
#include <cstdint>

struct GSRegisters;

namespace gsdump
{
    // Chamado no início de cada VBlank pelo EeScheduler: abre o dump no VSync pedido, grava os
    // registradores privilegiados e o marcador de VSync, e fecha depois do número de frames pedido.
    void onVSync(const GSRegisters &priv, uint64_t tick);
}
