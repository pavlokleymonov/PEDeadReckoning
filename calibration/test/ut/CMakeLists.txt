cmake_minimum_required(VERSION 3.0)

set(REPOSITORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../..)
set(GTEST_GMOCK_DIR ${REPOSITORY_ROOT}/test-utils/gtest_gmock_1.7.0)

# Include shared cmake stuff
# set cmake flags for coverage and unittest
# add target coverage
# add target coverage-html
include(${REPOSITORY_ROOT}/cmake/shared.cmake)

enable_testing()

include_directories(
   ${GTEST_GMOCK_DIR}
   ${CMAKE_CURRENT_SOURCE_DIR}
   ${REPOSITORY_ROOT}/clibration/include
)

#Test class PEState
add_executable(test_PEState
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
   ${REPOSITORY_ROOT}/calibration/source/PEState.cpp
   PEStateTest.cpp
)
#target_link_libraries(test_PEState pthread CommonAPI)
add_test(NAME test_PEState COMMAND test_PEState)
