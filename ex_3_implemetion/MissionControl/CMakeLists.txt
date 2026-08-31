cmake_minimum_required(VERSION 3.15)

if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")
endif()

if(NOT DEFINED CMAKE_PREFIX_PATH)
    list(APPEND CMAKE_PREFIX_PATH
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/default/vcpkg_installed/x64-linux"
        "${CMAKE_CURRENT_SOURCE_DIR}/../build/default/vcpkg_installed"
    )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../vcpkg.json" AND NOT DEFINED VCPKG_MANIFEST_DIR)
    set(VCPKG_MANIFEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "")
endif()

project(MissionControl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mp-units CONFIG REQUIRED)

# If built independently, import common target
if(NOT TARGET common::common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common_build)
endif()

# Build Mission Control as a shared dynamic library (.so)
add_library(MissionControl_207610130_215664087 SHARED
    src/MissionControlImpl_207610130_215664087.cpp
    src/DroneControlImpl.cpp
    src/ScanResultToVoxels.cpp
)

set_target_properties(MissionControl_207610130_215664087 PROPERTIES
    PREFIX ""
    OUTPUT_NAME "MissionControl_207610130_215664087"
)

target_include_directories(MissionControl_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
)

target_link_libraries(MissionControl_207610130_215664087 PRIVATE
    common::common
)

if(COMMAND drone_warnings)
    drone_warnings(MissionControl_207610130_215664087)
else()
    target_compile_options(MissionControl_207610130_215664087 PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
