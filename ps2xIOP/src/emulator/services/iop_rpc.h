#pragma once

#include "ps2x/iop/iop_types.h"

#include "../core/iop_cpu.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <unordered_map>
#include <unordered_set>

namespace ps2x::iop
{
    class IopHost;
}

namespace ps2x::iop::detail
{
    class IopKernel;
    struct IopThread;
    class IopMemory;

    class IopGuestExecutor
    {
    public:
        virtual ~IopGuestExecutor() = default;

        [[nodiscard]] virtual uint32_t executeGuestFunction(uint32_t address,
                                                            uint32_t a0,
                                                            uint32_t a1,
                                                            uint32_t a2,
                                                            uint32_t a3,
                                                            uint32_t gp) = 0;
        [[nodiscard]] virtual uint32_t executeGuestFunctionWithBudget(uint32_t address,
                                                                      uint32_t a0,
                                                                      uint32_t a1,
                                                                      uint32_t a2,
                                                                      uint32_t a3,
                                                                      uint32_t gp,
                                                                      uint32_t instructionBudget)
        {
            return executeGuestFunction(address, a0, a1, a2, a3, gp);
        }
    };

    class IopRpcBridge
    {
    public:
        IopRpcBridge(IopHost &host, IopMemory &memory, IopKernel &kernel) noexcept;

        void reset();
        [[nodiscard]] bool dispatchSifManImport(uint16_t ordinal, IopCpuState &cpu);
        [[nodiscard]] bool dispatchSifCmdImport(uint16_t ordinal, IopCpuState &cpu);
        [[nodiscard]] RpcResult handleRpc(const RpcRequest &request, IopGuestExecutor &executor);
        void onSifTransfer(const SifTransfer &transfer);
        void removeServersInRange(uint32_t base, uint32_t size);
        // A thread voltou de uma função de servidor (pc == kRpcReturnSentinel): copia o resultado para o
        // EE, devolve a thread ao RpcLoop e avisa o host. Retorna false se a thread não tinha chamada ativa.
        bool onThreadReturn(IopThread &thread);
        // Aborta chamadas que passaram de kDeferredRpcTimeoutCycles (servidor esperando hardware que o IOP
        // emulado não tem, ex. SIO2): devolve o buffer como está, como o caminho síncrono antigo fazia.
        void expireThreadCalls(uint64_t currentCycle);
        void setCurrentCycle(uint64_t cycle) noexcept { m_currentCycle = cycle; }

        [[nodiscard]] bool hasServer(uint32_t sid) const noexcept;
        [[nodiscard]] size_t serverCount() const noexcept { return m_servers.size(); }

    private:
        struct RpcServer
        {
            uint32_t sid = 0;
            uint32_t serverData = 0;
            uint32_t function = 0;
            uint32_t gp = 0;
            uint32_t buffer = 0;
            uint32_t callback = 0;
            uint32_t callbackBuffer = 0;
            uint32_t queue = 0;
            int thread = 0; // thread IOP que registrou o servidor (e roda o RpcLoop)
        };

        // Chamada em andamento na thread dona do servidor; saved é o estado da thread parada no RpcLoop.
        struct ThreadCall
        {
            RpcRequest request;
            uint32_t buffer = 0;
            IopCpuState saved;
            uint64_t startCycle = 0;
        };
        void finishThreadCall(IopThread &thread, ThreadCall call, uint32_t returnPointer);

        [[nodiscard]] bool startThreadCall(const RpcServer &server, const RpcRequest &request);
        void copyServerInput(const RpcServer &server, const RpcRequest &request);
        void copyServerResult(const RpcRequest &request, uint32_t returnPointer, uint32_t buffer);

        IopHost &m_host;
        IopMemory &m_memory;
        IopKernel &m_kernel;
        std::unordered_map<uint32_t, RpcServer> m_servers;
        std::unordered_set<int> m_threadsInRpcLoop;
        std::unordered_set<uint32_t> m_timedOutSids; // para logar uma vez por servidor
        uint64_t m_currentCycle = 0;
        std::unordered_map<int, ThreadCall> m_threadCalls;          // por thread
        std::unordered_map<int, std::deque<RpcRequest>> m_queuedCalls; // pedidos esperando a thread
        uint32_t m_nextDmaId = 1u;
        bool m_sifInitialized = false;
    };
}
