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
   ${CMAKE_CURRENT_SOURCE_DIR}/mock
   ${REPOSITORY_ROOT}/calibration/include
   ${REPOSITORY_ROOT}/common/include
)

#build gtest static library
add_library ( gtest STATIC 
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
)

#build pe_common static library
add_library ( pe_common STATIC 
   ${REPOSITORY_ROOT}/common/source/PESPosition.cpp
   ${REPOSITORY_ROOT}/common/source/PETools.cpp
)

#Test class PE::CCalibrationScale
add_executable(test_pe_calibration_scale
   ${REPOSITORY_ROOT}/calibration/source/PECCalibrationScale.cpp
   ${REPOSITORY_ROOT}/calibration/source/PECNormalisation.cpp
   PECCalibrationScaleTest.cpp
)

target_link_libraries(test_pe_calibration_scale pe_common gtest pthread )
add_test(NAME test_pe_calibration_scale COMMAND test_pe_calibration_scale)

#Test class PE::CCalibrationBase
add_executable(test_pe_calibration_base
   ${REPOSITORY_ROOT}/calibration/source/PECCalibrationBase.cpp
   ${REPOSITORY_ROOT}/calibration/source/PECNormalisation.cpp
   PECCalibrationBaseTest.cpp
)

target_link_libraries(test_pe_calibration_base pe_common gtest pthread )
add_test(NAME test_pe_calibration_base COMMAND test_pe_calibration_base)

#Test class PE::CNormalisation
add_executable(test_pe_normalisation
   ${REPOSITORY_ROOT}/calibration/source/PECNormalisation.cpp
   PECNormalisationTest.cpp
)
target_link_libraries(test_pe_normalisation pe_common gtest pthread )
add_test(NAME test_pe_normalisation COMMAND test_pe_normalisation)