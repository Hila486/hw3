cmake_minimum_required(VERSION 3.15)

# Main project definition.
# This project is written in C++.
project(assignment_3 LANGUAGES CXX)

# Compile all targets using C++20.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generates compile_commands.json.
# Useful for IDEs/tools such as clangd so they know how each file is compiled.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# External dependencies.
# mp-units -> strongly typed physical units.
# yaml-cpp -> reading YAML configuration files.
find_package(mp-units CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# Common warning configuration used by the project's targets.
# -Wall/-Wextra : enable many compiler warnings.
# -Werror       : treat warnings as errors.
# -pedantic     : enforce standard C++ more strictly.
function(drone_warnings target)
    target_compile_options(
        ${target}
        PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
    )
endfunction()

# Build the different parts of the project.
#
# common         -> shared interfaces/types
# Algorithm      -> MappingAlgorithm shared library (.so)
# MissionControl -> MissionControl shared library (.so)
# Simulator      -> main executable
add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)