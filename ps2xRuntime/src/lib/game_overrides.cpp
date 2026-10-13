#include "game_overrides.h"
#include "ps2_runtime.h"
#include "ps2_runtime_calls.h"
#include "ps2_stubs.h"
#include "ps2_syscalls.h"
#include "ps2_log.h"
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <optional>
#include <vector>

namespace
{
    std::mutex &registryMutex()
    {
        static std::mutex mutex;
        return mutex;
    }

    std::vector<ps2_game_overrides::Descriptor> &descriptorRegistry()
    {
        static std::vector<ps2_game_overrides::Descriptor> registry;
        return registry;
    }

    bool equalsIgnoreCaseAscii(std::string_view lhs, std::string_view rhs)
    {
        if (lhs.size() != rhs.size())
        {
            return false;
        }

        for (size_t i = 0; i < lhs.size(); ++i)
        {
            const auto l = static_cast<unsigned char>(lhs[i]);
            const auto r = static_cast<unsigned char>(rhs[i]);
            if (std::tolower(l) != std::tolower(r))
            {
                return false;
            }
        }

        return true;
    }

    std::string basenameFromPath(const std::string &path)
    {
        std::error_code ec;
        const std::filesystem::path fsPath(path);
        const std::filesystem::path leaf = fsPath.filename();
        if (leaf.empty())
        {
            return path;
        }
        return leaf.string();
    }

    std::optional<PS2Runtime::RecompiledFunction> resolveHandlerByName(std::string_view handlerName)
    {
        const std::string_view resolvedSyscall = ps2_runtime_calls::resolveSyscallName(handlerName);
        if (!resolvedSyscall.empty())
        {
#define PS2_RESOLVE_SYSCALL(name)                   \
    if (resolvedSyscall == std::string_view{#name}) \
    {                                               \
        return &ps2_syscalls::name;                 \
    }
            PS2_SYSCALL_LIST(PS2_RESOLVE_SYSCALL)
#undef PS2_RESOLVE_SYSCALL
        }

        const std::string_view resolvedStub = ps2_runtime_calls::resolveStubName(handlerName);
        if (!resolvedStub.empty())
        {
#define PS2_RESOLVE_STUB(name)                   \
    if (resolvedStub == std::string_view{#name}) \
    {                                            \
        return &ps2_stubs::name;                 \
    }
            PS2_STUB_LIST(PS2_RESOLVE_STUB)
#undef PS2_RESOLVE_STUB
        }

        return std::nullopt;
    }
}
namespace ps2_game_overrides
{
    AutoRegister::AutoRegister(const Descriptor &descriptor)
    {
        registerDescriptor(descriptor);
    }

    void registerDescriptor(const Descriptor &descriptor)
    {
        if (!descriptor.apply)
        {
            std::cerr << "[game_overrides] ignoring descriptor with null apply callback." << std::endl;
            return;
        }

        std::lock_guard<std::mutex> lock(registryMutex());
        descriptorRegistry().push_back(descriptor);
    }

    bool bindAddressHandler(PS2Runtime &runtime, uint32_t address, std::string_view handlerName)
    {
        const auto resolved = resolveHandlerByName(handlerName);
        if (!resolved.has_value())
        {
            std::cerr << "[game_overrides] unresolved handler '" << handlerName
                      << "' for address 0x" << std::hex << address << std::dec << std::endl;
            return false;
        }

        return runtime.replaceFunction(address, resolved.value());
    }

    void applyMatching(PS2Runtime &runtime,
                       const std::string &elfPath,
                       uint32_t entry,
                       uint32_t fileCrc32,
                       bool fileCrcValid)
    {
        // PS2X_BINDS: text file, one "handler@0xADDR" per line ('#' starts a comment).
        // Rebinds without regenerating or rebuilding the recompiled game.
        // shortcut: applies to whichever ELF is loaded, no CRC check; fine while there is one game.
        if (const char *bindsPath = std::getenv("PS2X_BINDS"))
        {
            std::ifstream in(bindsPath);
            std::string line;
            while (std::getline(in, line))
            {
                line = line.substr(0, line.find('#'));
                line.erase(std::remove_if(line.begin(), line.end(), [](unsigned char c) { return std::isspace(c); }), line.end());
                const size_t at = line.rfind('@');
                if (at == std::string::npos)
                {
                    continue;
                }
                bindAddressHandler(runtime, static_cast<uint32_t>(std::strtoul(line.c_str() + at + 1, nullptr, 0)),
                                   std::string_view(line).substr(0, at));
            }
        }


        std::vector<Descriptor> descriptors;
        {
            std::lock_guard<std::mutex> lock(registryMutex());
            descriptors = descriptorRegistry();
        }

        if (descriptors.empty())
        {
            return;
        }

        const std::string elfName = basenameFromPath(elfPath);
        size_t appliedCount = 0;
        for (const Descriptor &descriptor : descriptors)
        {
            if (!descriptor.apply)
            {
                continue;
            }

            if (descriptor.elfName && descriptor.elfName[0] != '\0')
            {
                if (!equalsIgnoreCaseAscii(descriptor.elfName, elfName))
                {
                    continue;
                }
            }

            if (descriptor.entry != 0u && descriptor.entry != entry)
            {
                continue;
            }

            if (descriptor.crc32 != 0u)
            {
                if (!fileCrcValid || fileCrc32 != descriptor.crc32)
                {
                    continue;
                }
            }

            const char *name = (descriptor.name && descriptor.name[0] != '\0')
                                   ? descriptor.name
                                   : "unnamed";
            RUNTIME_LOG("[game_overrides] applying '" << name << "'");
            descriptor.apply(runtime);
            ++appliedCount;
        }

        if (appliedCount > 0)
        {
            RUNTIME_LOG("[game_overrides] applied " << appliedCount << " matching override(s).");
        }
    }
}
