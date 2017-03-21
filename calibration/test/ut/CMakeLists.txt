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

#build gtest static library
add_library ( gtest STATIC 
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
)

#build petools static library
add_library ( petools STATIC 
   ${REPOSITORY_ROOT}/utils/sources/PETools.cpp
)

#Test class PE::calibration
add_executable(test_pe_calibration
   ${REPOSITORY_ROOT}/calibration/sources/PECalibration.cpp
   PECalibrationTest.cpp
)
target_link_libraries(test_pe_calibration petools gtest pthread )
add_test(NAME test_pe_calibration COMMAND test_pe_calibration)

#Test class PE::normalisation
add_executable(test_pe_normalisation
   ${REPOSITORY_ROOT}/calibration/sources/PENormalisation.cpp
   PENormalisationTest.cpp
)
target_link_libraries(test_pe_normalisation petools gtest pthread )
add_test(NAME test_pe_normalisation COMMAND test_pe_normalisation)

#Test class PE::position_filter
add_executable(test_pe_position_filter
   ${REPOSITORY_ROOT}/calibration/sources/PEPositionFilter.cpp
   PEPositionFilterTest.cpp
)
target_link_libraries(test_pe_position_filter petools gtest pthread )
add_test(NAME test_pe_position_filter COMMAND test_pe_position_filter)
