# MissionControl — builds as a SHARED library (.so) dynamically loaded by the Simulator.
# The .so name must be MissionControl_207610130_215664087.so (no "lib" prefix).

find_package(yaml-cpp CONFIG REQUIRED)

add_library(MissionControl_207610130_215664087 SHARED
    src/MissionControlImpl_207610130_215664087.cpp
    # UserCommon sources compiled directly into this library
    ${CMAKE_SOURCE_DIR}/UserCommon/src/Map3DImpl.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/MockGPS.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/MockMovement.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/MockLidar.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/DroneControlImpl.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/ScanResultToVoxels.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/ConfigParser.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/NpyMapIO.cpp
    ${CMAKE_SOURCE_DIR}/UserCommon/src/MapsComparison.cpp
)

# Remove the "lib" prefix so the output is MissionControl_207610130_215664087.so
set_target_properties(MissionControl_207610130_215664087 PROPERTIES PREFIX "")

target_include_directories(MissionControl_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
)

target_link_libraries(MissionControl_207610130_215664087 PRIVATE
    common::common
    yaml-cpp
)

drone_warnings(MissionControl_207610130_215664087)
