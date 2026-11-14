#pragma once

#include <string>
#include <vector>

namespace mt {

// Process-wide diagnostic log. The physics library reports failures here
// instead of failing silently. The UI reads the same buffer.
class Log {
public:
    static void info(const std::string& message);
    static void warn(const std::string& message);
    static void error(const std::string& message);

    static const std::string& lastError();
    static const std::vector<std::string>& lines();
    static void clear();
};

}  // namespace mt
