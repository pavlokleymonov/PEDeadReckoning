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
   ${REPOSITORY_ROOT}/common/include
   ${REPOSITORY_ROOT}/fusion/include
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

#Test class PE::CFusionSensor
add_executable(test_pe_fusion_sensor
   ${REPOSITORY_ROOT}/fusion/source/PECFusionSensor.cpp
   PECFusionSensorTest.cpp
)

target_link_libraries(test_pe_fusion_sensor pe_common gtest pthread )
add_test(NAME test_pe_fusion_sensor COMMAND test_pe_fusion_sensor)