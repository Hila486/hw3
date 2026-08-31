# UserCommon — shared implementation library (header-only usage via INTERFACE target)
# No makefile required per assignment spec, but we need a CMake INTERFACE target
# so that Algorithm, MissionControl, and Simulator can include UserCommon headers
# and compile the UserCommon .cpp files into their own builds.

add_library(UserCommon INTERFACE)

target_include_directories(UserCommon INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(UserCommon INTERFACE
    common::common
    yaml-cpp
)
