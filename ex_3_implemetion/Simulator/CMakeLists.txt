# Simulator — builds the main executable simulator_207610130_215664087.
# Must export registration symbols so dynamically loaded .so plugins can
# resolve MappingAlgorithmRegistration and MissionControlRegistration constructors.

find_package(yaml-cpp CONFIG REQUIRED)

add_executable(simulator_207610130_215664087
    src/main.cpp
    src/ArgumentParser.cpp
    src/DlLoader.cpp
    src/Registrar.cpp
    src/ResultExporter.cpp
    src/SimulationEngine.cpp
    # UserCommon sources compiled directly into the executable
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

target_include_directories(simulator_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
    ${CMAKE_SOURCE_DIR}/UserCommon/include
    ${CMAKE_SOURCE_DIR}/MissionControl/common_mission_control/include
)

target_link_libraries(simulator_207610130_215664087 PRIVATE
    common::common
    yaml-cpp
    ${CMAKE_DL_LIBS}
)

# Export all symbols so that dlopen'd .so plugins can resolve the registration
# constructors (MappingAlgorithmRegistration, MissionControlRegistration)
# that are defined in Registrar.cpp inside the executable.
if(UNIX)
    target_link_options(simulator_207610130_215664087 PRIVATE -rdynamic)
endif()

drone_warnings(simulator_207610130_215664087)
