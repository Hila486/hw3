cmake_minimum_required(VERSION 3.15)
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
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/Map3DImpl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/MockGPS.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/MockMovement.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/MockLidar.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/DroneControlImpl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ScanResultToVoxels.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/ConfigParser.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/NpyMapIO.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/src/MapsComparison.cpp
)

target_include_directories(simulator_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../MissionControl/common_mission_control/include
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
