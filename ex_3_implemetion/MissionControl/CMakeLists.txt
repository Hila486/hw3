cmake_minimum_required(VERSION 3.15)
project(MissionControl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mp-units CONFIG REQUIRED)

if(NOT TARGET common::common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common_build)
endif()

# Build MissionControl as a shared dynamic library (.so)
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
