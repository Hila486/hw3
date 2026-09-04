cmake_minimum_required(VERSION 3.15)

# Support configuring Simulator/ directly while still using vcpkg.
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(
        CMAKE_TOOLCHAIN_FILE
        "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING ""
    )
endif()

# Help standalone Simulator builds locate packages installed by vcpkg.
if(NOT DEFINED CMAKE_PREFIX_PATH)
    list(APPEND CMAKE_PREFIX_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/default/vcpkg_installed/x64-linux"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/default/vcpkg_installed"
    )
endif()

# Point standalone builds at the root vcpkg.json manifest.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg.json"
   AND NOT DEFINED VCPKG_MANIFEST_DIR)
    set(
        VCPKG_MANIFEST_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
        CACHE PATH ""
    )
endif()

project(Simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Simulator uses:
# mp-units -> strongly typed physical quantities.
# yaml-cpp -> YAML configuration parsing.
# Threads  -> std::thread worker pool used by the simulation engine.
find_package(mp-units CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(Threads REQUIRED)

# Import the shared common target when Simulator is built independently.
if(NOT TARGET common::common)
    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/../common
        common_build
    )
endif()

# Main simulator executable.
#
# Unlike Algorithm and MissionControl, this target is an executable,
# not a shared library. It is the process that loads the .so plugins.
add_executable(
    simulator_207610130_215664087

    # Program entry point and CLI parsing.
    src/main.cpp
    src/ArgumentParser.cpp

    # Dynamic plugin loading and registration.
    src/DlLoader.cpp
    src/Registrar.cpp

    # Results/reporting and top-level orchestration.
    src/ResultExporter.cpp
    src/SimulationEngine.cpp

    # Configuration and map handling.
    src/ConfigParser.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/NpyMapIO.cpp

    # Simulated drone hardware.
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
)

# Headers used by the simulator implementation.
target_include_directories(
    simulator_207610130_215664087
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

# Simulator dependencies:
#
# common::common  -> common interfaces and strong unit types
# yaml-cpp        -> YAML configuration parsing
# Threads::Threads-> pthreads backing for std::thread
# CMAKE_DL_LIBS   -> platform dynamic-loading support used by
#                    dlopen(), dlclose(), dlerror(), etc.
target_link_libraries(
    simulator_207610130_215664087
    PRIVATE
    common::common
    yaml-cpp
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

# Export symbols from the main executable on Unix.
# This helps dynamically loaded .so plugins resolve simulator-side
# symbols when necessary.
if(UNIX)
    target_link_options(
        simulator_207610130_215664087
        PRIVATE
        -rdynamic
    )
endif()

# Enable strict compiler warnings.
if(COMMAND drone_warnings)
    drone_warnings(simulator_207610130_215664087)
else()
    target_compile_options(
        simulator_207610130_215664087
        PRIVATE
        -Wall -Wextra -Werror -pedantic
    )
endif()


# ---------------------------------------------------------------------------
# Standalone map-comparison executable inherited from Assignment 2.
#
# This utility can load/compare maps without running the full simulator.
# ---------------------------------------------------------------------------
add_executable(
    maps_comparison
    src/maps_comparison_main.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/NpyMapIO.cpp
)

set_target_properties(
    maps_comparison
    PROPERTIES
    OUTPUT_NAME "maps_comparison"
)

target_include_directories(
    maps_comparison
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(
    maps_comparison
    PRIVATE
    common::common
    yaml-cpp
)

if(COMMAND drone_warnings)
    drone_warnings(maps_comparison)
else()
    target_compile_options(
        maps_comparison
        PRIVATE
        -Wall -Wextra -Werror -pedantic
    )
endif()