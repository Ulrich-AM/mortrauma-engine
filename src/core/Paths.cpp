#include "mortrauma/core/Paths.hpp"

#include "mortrauma/core/Log.hpp"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h>
#endif

namespace mt {
namespace {
std::filesystem::path exeDirCached() {
#if defined(_WIN32)
    wchar_t buffer[MAX_PATH];
    const DWORD length = GetModuleFileNameW(nullptr, buffer, MAX_PATH);
    if (length == 0 || length == MAX_PATH) {
        return std::filesystem::current_path();
    }
    return std::filesystem::path(buffer).parent_path();
#else
    char buffer[4096];
    const ssize_t length = ::readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (length <= 0) {
        return std::filesystem::current_path();
    }
    buffer[length] = '\0';
    return std::filesystem::path(buffer).parent_path();
#endif
}
}  // namespace

std::filesystem::path executableDirectory() { return exeDirCached(); }

std::vector<std::filesystem::path> assetRoots() {
    std::vector<std::filesystem::path> roots;
    const auto exe = executableDirectory();
    roots.push_back(exe);
    roots.push_back(exe / "..");
    roots.push_back(exe / ".." / "..");
#if defined(MORTRAUMA_SOURCE_DIR)
    roots.push_back(std::filesystem::path(MORTRAUMA_SOURCE_DIR));
#endif
    return roots;
}

std::filesystem::path findAsset(const std::filesystem::path& relative) {
    for (const auto& root : assetRoots()) {
        const auto direct = root / relative;
        if (std::filesystem::exists(direct)) {
            return std::filesystem::weakly_canonical(direct);
        }
        const auto underAssets = root / "assets" / relative;
        if (std::filesystem::exists(underAssets)) {
            return std::filesystem::weakly_canonical(underAssets);
        }
    }
    Log::error("Asset not found: " + relative.generic_string());
    return {};
}

}  // namespace mt
