#pragma once

#include "mortrauma/poses/PoseLibrary.hpp"

#include <string>

namespace mt {

// Samples one Mixamo frame and retargets bone directions into Humanoidguy
// constraint frames. It does not copy Mixamo local rotations and it does not
// replace the character mesh.
PoseImportResult importMixamoPose(const PhysicsWorld& world, const Ragdoll& ragdoll, const std::string& path);

}  // namespace mt
