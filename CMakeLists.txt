# Top level project file focuses on determining what to do
# The project may be responsible for generating multiple targets
# so global compile flags should not be set at this level
project(top)

cmake_minimum_required (VERSION 3.0)

# At the top level, determine the targets to include and build
# and what installation actions to perform
# You can override these options in command line like this:
# cmake -DBUILD_TESTS=ON ..
OPTION(BUILD_APP "Build position engine" ON)

#Always build test application to be in sync with main sources
OPTION(BUILD_TESTS "Build test programs" ON)

#Set repository root
set(REPOSITORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Include shared cmake stuff
# set cmake flags for coverage and unittest
# add target coverage
include(${REPOSITORY_ROOT}/cmake/shared.cmake)

#Build all stuff
if(BUILD_APP)
   add_subdirectory(core)
endif()

#build all tests
if(BUILD_TESTS)
   enable_testing()
   add_subdirectory(test)
endif()