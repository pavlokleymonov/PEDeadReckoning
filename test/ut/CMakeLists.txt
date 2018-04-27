cmake_minimum_required(VERSION 3.0)

set(REPOSITORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(GTEST_GMOCK_DIR ${REPOSITORY_ROOT}/test-utils/gtest_gmock_1.7.0)

# Include shared cmake stuff
# set cmake flags for coverage and unittest
# add target coverage
# add target coverage-html
include(${REPOSITORY_ROOT}/cmake/shared.cmake)

enable_testing()

include_directories(
   ${GTEST_GMOCK_DIR}
   ${REPOSITORY_ROOT}/common/include
   ${REPOSITORY_ROOT}/fusion/include
   ${REPOSITORY_ROOT}/calibration/include
   ${CMAKE_CURRENT_SOURCE_DIR}/mock
)

#build gtest static library
add_library ( gtest STATIC 
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
)

add_library ( pe_common STATIC
   ${REPOSITORY_ROOT}/common/source/PESPosition.cpp
   ${REPOSITORY_ROOT}/common/source/PESBasicSensor.cpp
   ${REPOSITORY_ROOT}/common/source/PESPosition.cpp
   ${REPOSITORY_ROOT}/common/source/PETools.cpp
)

add_library ( pe_fusion STATIC
   ${REPOSITORY_ROOT}/fusion/source/PEFusionTools.cpp
   ${REPOSITORY_ROOT}/fusion/source/PECFusionSensor.cpp
)

add_library ( pe_calibration STATIC
   ${REPOSITORY_ROOT}/calibration/source/PECCalibration.cpp
   ${REPOSITORY_ROOT}/calibration/source/PECNormalisation.cpp
)

##########################
#Test struct PE::SPosition
add_executable(test_pe_sposition
   PESPositionTest.cpp
)
target_link_libraries(test_pe_sposition pe_common gtest pthread)
add_test(NAME test_pe_sposition COMMAND test_pe_sposition)

#############################
#Test struct PE::SBasicSensor
add_executable(test_pe_sbasic_sensor
   PESBasicSensorTest.cpp
)
target_link_libraries(test_pe_sbasic_sensor pe_common gtest pthread)
add_test(NAME test_pe_sbasic_sensor COMMAND test_pe_sbasic_sensor)

#####################
#Test tools PE::TOOLS
add_executable(test_pe_tools
   PEToolsTest.cpp
)
target_link_libraries(test_pe_tools pe_common gtest pthread)
add_test(NAME test_pe_tools COMMAND test_pe_tools)

#####################
#Test fusion tools PE::FUSION
add_executable(test_pe_fusion_tools
   PEFusionToolsTest.cpp
)
target_link_libraries(test_pe_fusion_tools pe_fusion pe_common gtest pthread)
add_test(NAME test_pe_fusion_tools COMMAND test_pe_fusion_tools)

#############################
#Test class PE::CFusionSensor
add_executable(test_pe_fusion_sensor
   PECFusionSensorTest.cpp
)
target_link_libraries(test_pe_fusion_sensor pe_fusion pe_common gtest pthread )
add_test(NAME test_pe_fusion_sensor COMMAND test_pe_fusion_sensor)

#################################
#Test class PE::CCalibration
add_executable(test_pe_calibration
   PECCalibrationTest.cpp
)
target_link_libraries(test_pe_calibration pe_common pe_calibration gtest pthread )
add_test(NAME test_pe_calibration COMMAND test_pe_calibration)

##############################
#Test class PE::CNormalisation
add_executable(test_pe_normalisation
   PECNormalisationTest.cpp
)
target_link_libraries(test_pe_normalisation pe_common pe_calibration gtest pthread )
add_test(NAME test_pe_normalisation COMMAND test_pe_normalisation)