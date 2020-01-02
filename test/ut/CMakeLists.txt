cmake_minimum_required(VERSION 3.0)

set(GTEST_GMOCK_DIR ${REPOSITORY_ROOT}/test-utils/gtest_gmock_1.7.0)

include_directories(
   ${GTEST_GMOCK_DIR}
   ${REPOSITORY_ROOT}/common/include
   ${REPOSITORY_ROOT}/fusion/include
   ${REPOSITORY_ROOT}/calibration/include
   ${REPOSITORY_ROOT}/normalisation/include
   ${REPOSITORY_ROOT}/core/include
   ${REPOSITORY_ROOT}/sensors/include
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
)

add_library ( pe_normalisation STATIC
   ${REPOSITORY_ROOT}/normalisation/source/PECNormalisation.cpp
)

add_library ( pe_sensors STATIC
   ${REPOSITORY_ROOT}/sensors/source/PESensorTools.cpp
   ${REPOSITORY_ROOT}/sensors/source/PECOdometer.cpp
)

##########################
#Test types of namespace PE::
add_executable(test_pe_types
   PETypesTest.cpp
)
target_link_libraries(test_pe_types gtest pthread)
add_test(NAME test_pe_types COMMAND test_pe_types)

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

##############################
#Test class PE::CNormalisation
add_executable(test_pe_normalisation
   PECNormalisationTest.cpp
)
target_link_libraries(test_pe_normalisation pe_common pe_normalisation gtest pthread )
add_test(NAME test_pe_normalisation COMMAND test_pe_normalisation)

#################################
#Test class PE::CCalibration
add_executable(test_pe_calibration
   PECCalibrationTest.cpp
)
target_link_libraries(test_pe_calibration pe_calibration gtest pthread )
add_test(NAME test_pe_calibration COMMAND test_pe_calibration)

#################################
#Test class PE::COdometer
#copying track file
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/testtracks/ODO_40ms_60sec_GNSS_100ms_32sec.txt ${CMAKE_CURRENT_BINARY_DIR} COPYONLY)
#create binary
add_executable(test_pe_odometer
   PECOdometerTest.cpp
)
target_link_libraries(test_pe_odometer pe_sensors pe_common pe_calibration pe_normalisation gtest pthread )
add_test(NAME test_pe_odometer COMMAND test_pe_odometer)

#################################
#Test class PE::Sensor
add_executable(test_pe_sensor
   PESensorToolsTest.cpp
)
target_link_libraries(test_pe_sensor pe_sensors pe_common gtest pthread )
add_test(NAME test_pe_sensor COMMAND test_pe_sensor)

#################################
#Test C library PECore
add_executable(test_pe_core_c_lib
   PECoreTest.cpp
)
target_link_libraries(test_pe_core_c_lib pe gtest pthread )
add_test(NAME test_pe_core_c_lib COMMAND test_pe_core_c_lib)
