cmake_minimum_required(VERSION 3.15)
project(Algorithm LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(mp-units CONFIG REQUIRED)

# If built independently, import common target
if(NOT TARGET common::common)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../common common_build)
endif()

# Build Mapping Algorithm as a shared dynamic library (.so)
add_library(Algorithm_207610130_215664087 SHARED
    src/MappingAlgorithmImpl_207610130_215664087.cpp
)

set_target_properties(Algorithm_207610130_215664087 PROPERTIES
    PREFIX ""
    OUTPUT_NAME "Algorithm_207610130_215664087"
)

target_include_directories(Algorithm_207610130_215664087 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(Algorithm_207610130_215664087 PRIVATE
    common::common
)

if(COMMAND drone_warnings)
    drone_warnings(Algorithm_207610130_215664087)
else()
    target_compile_options(Algorithm_207610130_215664087 PRIVATE -Wall -Wextra -Werror -pedantic)
endif()
