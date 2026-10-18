#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace GSMem
{
    class TexturePageCache
    {
    public:
        static constexpr uint32_t kPageSize = 8192u;

        void Invalidate() noexcept
        {
            m_pageBases.fill(UINT32_MAX);
            m_lastPage = 0;
            m_nextVictim = 0;
        }

        // byteAddress is the wrapped, swizzled VRAM address. The returned
        // pointer is valid only until the next miss or invalidation.
        const uint8_t* Resolve(const uint8_t* vram, uint32_t byteAddress) noexcept
        {
            const uint32_t pageBase = byteAddress & ~(kPageSize - 1u);
            if (m_pageBases[m_lastPage] != pageBase)
            {
                uint32_t slot = 0;
                while (slot < kPageCount && m_pageBases[slot] != pageBase)
                    ++slot;
                if (slot == kPageCount)
                {
                    slot = m_nextVictim;
                    m_nextVictim = (m_nextVictim + 1u) % kPageCount;
                    std::memcpy(m_bytes[slot].data(), vram + pageBase, kPageSize);
                    m_pageBases[slot] = pageBase;
                }
                m_lastPage = slot;
            }
            return m_bytes[m_lastPage].data() + (byteAddress & (kPageSize - 1u));
        }

    private:
        // Quatro páginas evitam recópias ao filtrar texels em fronteiras de página.
        static constexpr uint32_t kPageCount = 4;
        alignas(64) std::array<std::array<uint8_t, kPageSize>, kPageCount> m_bytes{};
        std::array<uint32_t, kPageCount> m_pageBases{UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX};
        uint32_t m_lastPage = 0;
        uint32_t m_nextVictim = 0;
    };
}
