#pragma once

#include "mortrauma/assets/Model.hpp"

#include <string>

namespace mt {

class ModelLoader {
public:
    // Loads FBX or glTF. Vertices are baked into the dominant bone's local
    // frame in meters, Y-up. A failed import does not invent geometry.
    static ImportedModel load(const std::string& path);
};

// Standing capsule humanoid with the same bone names as humanoidguy.
// Used only when the supplied character cannot be read. The report is marked
// usedFallback so the failure stays visible.
ImportedModel makeProceduralHumanoid();

}  // namespace mt
