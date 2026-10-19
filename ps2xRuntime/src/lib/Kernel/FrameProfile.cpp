#include "FrameProfile.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>

extern std::atomic<uint64_t> g_gsSubmitNs;

namespace frameprof
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        struct State
        {
            std::FILE *out = nullptr;
            Clock::time_point frameStart = Clock::now();
            Clock::time_point waitStart{};
            bool waiting = false;
            uint64_t vu1Ns = 0, vu1GsNs = 0, vu1Calls = 0, vu1Cycles = 0;
            uint64_t gsStart = 0, eeCycleStart = 0;

            State()
            {
                if (const char *path = std::getenv("PS2X_PROF_VSYNC"))
                {
                    out = std::fopen(path, "w");
                    if (out)
                        std::fprintf(out, "vsync,wall_us,vu1_core_us,gs_us,vblank_wait_us,other_us,vu1_calls,vu1_cycles,ee_cycles\n");
                    else
                        std::fprintf(stderr, "[prof-vsync] não consegui abrir %s\n", path);
                }
            }
        };

        State &state()
        {
            static State s;
            return s;
        }

        uint64_t nsBetween(Clock::time_point a, Clock::time_point b)
        {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count());
        }
    }

    bool enabled()
    {
        static const bool on = state().out != nullptr;
        return on;
    }

    void addVu1(uint64_t ns, uint64_t gsNs, uint64_t cycles)
    {
        State &s = state();
        s.vu1Ns += ns;
        s.vu1GsNs += gsNs;
        s.vu1Cycles += cycles;
        ++s.vu1Calls;
    }

    void onIntcStatRead(uint32_t value)
    {
        State &s = state();
        if ((value & 0x4u) == 0u && !s.waiting)
        {
            s.waiting = true;
            s.waitStart = Clock::now();
        }
    }

    void onVBlank(uint64_t tick, uint64_t eeCycle)
    {
        State &s = state();
        const Clock::time_point now = Clock::now();
        const uint64_t wall = nsBetween(s.frameStart, now);
        const uint64_t wait = s.waiting ? nsBetween(s.waitStart, now) : 0u;
        const uint64_t gsTotal = g_gsSubmitNs.load(std::memory_order_relaxed);
        const uint64_t gs = gsTotal - s.gsStart;
        const uint64_t vu1Core = s.vu1Ns > s.vu1GsNs ? s.vu1Ns - s.vu1GsNs : 0u;
        const uint64_t accounted = vu1Core + gs + wait;
        const uint64_t other = wall > accounted ? wall - accounted : 0u;
        std::fprintf(s.out, "%llu,%.1f,%.1f,%.1f,%.1f,%.1f,%llu,%llu,%llu\n",
                     static_cast<unsigned long long>(tick), wall / 1e3, vu1Core / 1e3, gs / 1e3, wait / 1e3,
                     other / 1e3, static_cast<unsigned long long>(s.vu1Calls),
                     static_cast<unsigned long long>(s.vu1Cycles),
                     static_cast<unsigned long long>(eeCycle - s.eeCycleStart));
        std::fflush(s.out);
        s.frameStart = now;
        s.waiting = false;
        s.vu1Ns = s.vu1GsNs = s.vu1Calls = s.vu1Cycles = 0;
        s.gsStart = gsTotal;
        s.eeCycleStart = eeCycle;
    }
}
