#pragma once

#include <cstdlib>
#include <ctime>

// PS2X_FIXED_CLOCK=1: toda hora que o HLE entrega ao jogo (sceCdReadClock, datas do memory card e do fio) vira
// 2004-01-01 12:00:00, para que execuções com a mesma entrada saiam iguais bit a bit. A hora do host era a única
// diferença na EE RAM entre duas execuções no mesmo VSync. Devolve true se trocou tm.
inline bool applyFixedClock(std::tm &tm)
{
    static const bool fixed = std::getenv("PS2X_FIXED_CLOCK") != nullptr;
    if (!fixed)
        return false;
    tm = {};
    tm.tm_hour = 12;
    tm.tm_mday = 1;
    tm.tm_mon = 0;
    tm.tm_year = 104;
    return true;
}
