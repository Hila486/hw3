# Header-only/common CMake target.
#
# INTERFACE means this target does not create a physical .so or .a library.
# Instead, it carries shared include paths and dependencies that other
# targets inherit when they link against common::common.
add_library(drone_common INTERFACE)

# Convenient namespaced alias used by the rest of the project.
add_library(common::common ALIAS drone_common)

# Any target that links common::common can include headers from common/include.
#
# BUILD_INTERFACE means this include path is intended while building
# the project from its source tree.
target_include_directories(
    drone_common
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Common interfaces/types use mp-units, so this dependency is propagated
# to targets that use common::common.
target_link_libraries(
    drone_common
    INTERFACE
    mp-units::mp-units
)