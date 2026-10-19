#pragma once

#include <cstdint>

// PS2X_PROF_VSYNC=arquivo.csv: uma linha por VSync com o tempo real gasto entre dois VBlanks, separado em núcleo do
// VU1, rasterizador do GS, espera do EE pelo VBlank (polling de INTC_STAT) e o resto (código do EE, IOP, scheduler).
// Só acumula contadores; a escrita acontece uma vez por VSync, para não distorcer o que mede.
namespace frameprof
{
    bool enabled();
    // Tempo de uma execução do VU1, incluindo o GS chamado de dentro dela (gsNs), e os ciclos de VU1 executados.
    void addVu1(uint64_t ns, uint64_t gsNs, uint64_t cycles);
    // Leitura de INTC_STAT: a primeira com o bit 2 (VBlank) limpo marca o início da espera pelo VBlank.
    void onIntcStatRead(uint32_t value);
    void onVBlank(uint64_t tick, uint64_t eeCycle);
}
