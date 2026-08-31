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

project(Simulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mp-units CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# If built independently, import common target
if(NOT TARGET common::common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common_build)
endif()

add_executable(simulator_207610130_215664087
    src/main.cpp
    src/ArgumentParser.cpp
    src/DlLoader.cpp
    src/Registrar.cpp
    src/ResultExporter.cpp
    src/SimulationEngine.cpp
    src/ConfigParser.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/NpyMapIO.cpp
)

target_include_directories(simulator_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(simulator_207610130_215664087 PRIVATE
    common::common
    yaml-cpp
    ${CMAKE_DL_LIBS}
)

if(UNIX)
    target_link_options(simulator_207610130_215664087 PRIVATE -rdynamic)
endif()

if(COMMAND drone_warnings)
    drone_warnings(simulator_207610130_215664087)
else()
    target_compile_options(simulator_207610130_215664087 PRIVATE -Wall -Wextra -Werror -pedantic)
endif()

# Standalone maps_comparison executable (inherited from HW2)
add_executable(maps_comparison
    src/maps_comparison_main.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/NpyMapIO.cpp
)

set_target_properties(maps_comparison PROPERTIES
    OUTPUT_NAME "maps_comparison"
)

target_include_directories(maps_comparison PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(maps_comparison PRIVATE
    common::common
    yaml-cpp
)

if(COMMAND drone_warnings)
    drone_warnings(maps_comparison)
else()
    target_compile_options(maps_comparison PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
