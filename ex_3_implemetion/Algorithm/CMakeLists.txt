cmake_minimum_required(VERSION 3.15)

# If this directory is configured independently instead of from the
# root project, use the vcpkg toolchain automatically when VCPKG_ROOT exists.
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(
        CMAKE_TOOLCHAIN_FILE
        "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
        CACHE STRING ""
    )
endif()

# Help CMake find packages already installed by the root vcpkg build.
# This mainly supports building Algorithm/ independently.
if(NOT DEFINED CMAKE_PREFIX_PATH)
    list(APPEND CMAKE_PREFIX_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/default/vcpkg_installed/x64-linux"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/default/vcpkg_installed"
    )
endif()

# Tell vcpkg where the project's vcpkg.json manifest lives when this
# subproject is configured by itself.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg.json"
   AND NOT DEFINED VCPKG_MANIFEST_DIR)
    set(
        VCPKG_MANIFEST_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
        CACHE PATH ""
    )
endif()

project(Algorithm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mp-units CONFIG REQUIRED)

# When Algorithm is built independently, common::common may not exist yet.
# In that case, import the shared common target manually.
if(NOT TARGET common::common)
    add_subdirectory(
        ${CMAKE_CURRENT_SOURCE_DIR}/../common
        common_build
    )
endif()

# Build our MappingAlgorithm implementation as a SHARED library.
# On Linux this produces a dynamically loadable .so plugin.
#
# SimulationEngine later loads this library using dlopen().
add_library(
    Algorithm_207610130_215664087
    SHARED
    src/MappingAlgorithmImpl_207610130_215664087.cpp
)

# Linux normally prefixes shared libraries with "lib".
# PREFIX "" removes that prefix so the result is:
#
# Algorithm_207610130_215664087.so
#
# rather than:
# libAlgorithm_207610130_215664087.so
set_target_properties(
    Algorithm_207610130_215664087
    PROPERTIES
    PREFIX ""
    OUTPUT_NAME "Algorithm_207610130_215664087"
)

# Header search paths used while compiling this Algorithm target.
target_include_directories(
    Algorithm_207610130_215664087
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

# Algorithm uses the common interfaces/types.
target_link_libraries(
    Algorithm_207610130_215664087
    PRIVATE
    common::common
)

# Use the root project's warning function when available.
# The fallback allows this directory to also build independently.
if(COMMAND drone_warnings)
    drone_warnings(Algorithm_207610130_215664087)
else()
    target_compile_options(
        Algorithm_207610130_215664087
        PRIVATE
        -Wall -Wextra -Werror -pedantic
    )
endif()