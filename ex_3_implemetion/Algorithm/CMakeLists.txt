# Algorithm — builds as a SHARED library (.so) dynamically loaded by the Simulator.
# The .so name must be Algorithm_207610130_215664087.so (no "lib" prefix).

find_package(yaml-cpp CONFIG REQUIRED)

add_library(Algorithm_207610130_215664087 SHARED
    src/MappingAlgorithmImpl_207610130_215664087.cpp
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

# Remove the "lib" prefix so the output is Algorithm_207610130_215664087.so
set_target_properties(Algorithm_207610130_215664087 PROPERTIES PREFIX "")

target_include_directories(Algorithm_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
    ${CMAKE_SOURCE_DIR}/MissionControl/common_mission_control/include
)

target_link_libraries(Algorithm_207610130_215664087 PRIVATE
    common::common
    yaml-cpp
)

drone_warnings(Algorithm_207610130_215664087)
