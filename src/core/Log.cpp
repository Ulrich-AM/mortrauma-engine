#include "mortrauma/core/Log.hpp"

#include <iostream>

namespace mt {
namespace {
std::vector<std::string> gLines;
std::string gLastError;

void push(const char* level, const std::string& message, bool isError) {
    const std::string line = std::string(level) + message;
    gLines.push_back(line);
    if (gLines.size() > 400) {
        gLines.erase(gLines.begin(), gLines.begin() + 100);
    }
    if (isError) {
        gLastError = message;
        std::cerr << line << '\n';
    } else {
        std::cout << line << '\n';
    }
}
}  // namespace

void Log::info(const std::string& message) { push("[info] ", message, false); }
void Log::warn(const std::string& message) { push("[warn] ", message, false); }
void Log::error(const std::string& message) { push("[error] ", message, true); }
const std::string& Log::lastError() { return gLastError; }
const std::vector<std::string>& Log::lines() { return gLines; }
void Log::clear() {
    gLines.clear();
    gLastError.clear();
}

}  // namespace mt
