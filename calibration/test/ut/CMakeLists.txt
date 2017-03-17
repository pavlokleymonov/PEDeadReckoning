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
   ${REPOSITORY_ROOT}/calibration/include
   ${REPOSITORY_ROOT}/utils/include
)

#Test class PE::calibration
add_executable(test_pe_calibration
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
   ${REPOSITORY_ROOT}/calibration/sources/PECalibration.cpp
   ${REPOSITORY_ROOT}/utils/sources/PETools.cpp
   PECalibrationTest.cpp
)

target_link_libraries(test_pe_calibration pthread )
add_test(NAME test_pe_calibration COMMAND test_pe_calibration)
