# Top level project file focuses on determining what to do
# The project may be responsible for generating multiple targets
# so global compile flags should not be set at this level
project(top)

cmake_minimum_required (VERSION 3.0)

# At the top level, determine the targets to incldue and build
# and what installation actions to perform
# You can override these options in command line like this:
# cmake -DBUILD_TESTS=ON ..
OPTION(BUILD_APP "Build position engine" ON)

#Alway build test aplication to be in sync with main sources
OPTION(BUILD_TESTS "Build test programs" ON)


#add global include folders
#include_directories(
#   ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
#)

#implementation of all stuff
if(BUILD_APP)
   add_subdirectory(utils)
   add_subdirectory(calibration)
endif()