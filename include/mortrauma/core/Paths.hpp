#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace mt {

std::filesystem::path executableDirectory();

// Ordered search roots: directory of the executable first, then the source
// tree when the application is launched from a build directory. Callers must
// not depend on the process working directory.
std::vector<std::filesystem::path> assetRoots();

std::filesystem::path findAsset(const std::filesystem::path& relative);

}  // namespace mt
