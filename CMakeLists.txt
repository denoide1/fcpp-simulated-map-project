# minimum required version of CMake
cmake_minimum_required(VERSION 3.9)
# avoid building internal tests of the FCPP library
option(FCPP_INTERNAL_TESTS "Build internal tests for FCPP." OFF)
# load the FCPP sub-module
add_subdirectory(./fcpp/src)
# CMake macro setting up the FCPP library
fcpp_setup()

# project description
project(
    fcpp-simulated-map-project
    VERSION 0.0.0
    DESCRIPTION "Quick project using the new component simulated map."
)

# target declaration
fcpp_target(run/test_sim_map.cpp ON)
