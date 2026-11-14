cmake_minimum_required(VERSION 3.24)
project(MORTRAUMA VERSION 0.3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>" CACHE STRING "" FORCE)
    add_compile_definitions(NOMINMAX WIN32_LEAN_AND_MEAN _CRT_SECURE_NO_WARNINGS)
endif()

# Forward slashes so the path is a valid C string on Windows.
string(REPLACE "\\" "/" MORTRAUMA_SOURCE_DIR_FWD "${CMAKE_SOURCE_DIR}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# --- pinned third-party dependencies -------------------------------------------------
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

set(USE_STATIC_MSVC_RUNTIME_LIBRARY ON CACHE BOOL "" FORCE)
set(INTERPROCEDURAL_OPTIMIZATION OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    JoltPhysics
    GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
    GIT_TAG v5.3.0
    GIT_SHALLOW TRUE
    SOURCE_SUBDIR Build)
FetchContent_MakeAvailable(JoltPhysics)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(glfw)

set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
set(JSON_Install OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(nlohmann_json)

set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(ASSIMP_NO_EXPORT ON CACHE BOOL "" FORCE)
set(ASSIMP_WARNINGS_AS_ERRORS OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_FBX_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_GLTF_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ZLIB ON CACHE BOOL "" FORCE)
FetchContent_Declare(
    assimp
    GIT_REPOSITORY https://github.com/assimp/assimp.git
    GIT_TAG v5.4.3
    GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(assimp)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.8-docking
    GIT_SHALLOW TRUE)
FetchContent_GetProperties(imgui)
if(NOT imgui_POPULATED)
    FetchContent_Populate(imgui)
endif()
if(NOT TARGET imgui)
    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
    target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
    target_link_libraries(imgui PUBLIC glfw)
    target_compile_definitions(imgui PUBLIC IMGUI_DISABLE_DEMO_WINDOWS)
endif()

# --- engine (no Dear ImGui, no OpenGL) ----------------------------------------------
add_library(mortrauma_engine STATIC
    src/core/Log.cpp
    src/core/Paths.cpp
    src/physics/PhysicsWorld.cpp
    src/assets/ModelLoader.cpp
    src/rig/HumanoidConfig.cpp
    src/rig/Ragdoll.cpp
    src/motors/MotorProfile.cpp
    src/motors/MotorController.cpp
    src/motors/PoseController.cpp
    src/poses/PoseLibrary.cpp
    src/poses/PoseImporter.cpp
    src/balance/BalanceConfig.cpp
    src/balance/SupportContacts.cpp
    src/balance/BalanceEstimator.cpp
    src/balance/StandingPose.cpp
    src/balance/BalanceController.cpp)
target_include_directories(mortrauma_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(mortrauma_engine PUBLIC Jolt nlohmann_json::nlohmann_json assimp)
target_compile_definitions(mortrauma_engine PUBLIC MORTRAUMA_SOURCE_DIR="${MORTRAUMA_SOURCE_DIR_FWD}")
if(MSVC)
    target_compile_options(mortrauma_engine PRIVATE /W4 /EHsc)
else()
    target_compile_options(mortrauma_engine PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# --- application ---------------------------------------------------------------------
find_package(OpenGL REQUIRED)

add_executable(MORTRAUMA
    src/app/main.cpp
    src/renderer/Renderer.cpp
    src/app/Application.cpp)
target_link_libraries(MORTRAUMA PRIVATE mortrauma_engine glfw imgui OpenGL::GL)
target_compile_definitions(MORTRAUMA PRIVATE MORTRAUMA_VERSION="0.3.0")
if(MSVC)
    target_compile_options(MORTRAUMA PRIVATE /W4 /EHsc)
    set_property(TARGET MORTRAUMA PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    target_compile_options(MORTRAUMA PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_link_libraries(MORTRAUMA PRIVATE dl pthread)
endif()

# --- tests ---------------------------------------------------------------------------
include(CTest)
if(BUILD_TESTING)
    add_executable(mortrauma_tests tests/test_ragdoll.cpp tests/test_motors.cpp tests/test_grab.cpp tests/test_balance.cpp)
    target_link_libraries(mortrauma_tests PRIVATE mortrauma_engine)
    if(MSVC)
        target_compile_options(mortrauma_tests PRIVATE /W4 /EHsc)
    else()
        target_compile_options(mortrauma_tests PRIVATE -Wall -Wextra -Wno-unused-parameter)
    endif()
    foreach(test_name RIG-001 RIG-002 RIG-003 RIG-004 RIG-005 RIG-006 PHY-001 PHY-002 PHY-003 PHY-004 PHY-005 PHY-006 LIM-001 MOT-001 MOT-002 MOT-003 MOT-004 MOT-005 MOT-006 MOT-007 MOT-008 MOT-009 MOT-010 MOT-011 MOT-012 GRAB-001 GRAB-002 GRAB-003 GRAB-004 GRAB-005 GRAB-006 GRAB-007 BAL-001 BAL-002 BAL-003 BAL-004 BAL-005 BAL-006 BAL-007 BAL-008 BAL-009 BAL-010 BAL-011 BAL-012 BAL-013 BAL-014)
        add_test(NAME ${test_name} COMMAND mortrauma_tests ${test_name})
        set_tests_properties(${test_name} PROPERTIES TIMEOUT 180)
    endforeach()
endif()
