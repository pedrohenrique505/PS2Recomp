#pragma once
#include <cstdint>

constexpr uint32_t kThreadReturnSentinel = 0x1FFFFF00u;
constexpr uint32_t kCallReturnSentinel = 0x1FFFFF04u;
// Retorno de uma função de servidor RPC executada na thread dona do servidor (ver IopRpcBridge).
constexpr uint32_t kRpcReturnSentinel = 0x1FFFFF08u;
// Teto para uma função de servidor RPC terminar na sua thread (~0,25 s de IOP); esperas reais (CD) levam bem menos.
constexpr uint64_t kDeferredRpcTimeoutCycles = 36'864'000ull / 4ull;
constexpr uint64_t kIopClockHz = 36'864'000ull;
// NTSC field cadence (approximately 59.94 Hz).  VBlank imports are
// scheduler waits, not no-op timing hints: returning immediately lets
// high-priority IRX threads busy-loop and starve RPC server threads.
constexpr uint64_t kVblankPeriodCycles = (kIopClockHz * 1001ull + 30'000ull) / 60'000ull;
constexpr uint64_t kVblankEndPhaseCycles = kVblankPeriodCycles / 16ull;
constexpr uint32_t kDefaultSlice = 256u;
constexpr uint32_t kMaxCallInstructions = 2'000'000u;
constexpr uint32_t kModuleLoadBase = 0x00010000u;
constexpr uint32_t kStackGuardBytes = 64u;