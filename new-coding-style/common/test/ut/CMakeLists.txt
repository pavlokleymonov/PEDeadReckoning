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
)

#build gtest static library
add_library ( gtest STATIC 
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
)

#Test struct PE::SPosition
add_executable(test_pe_sposition
   ${REPOSITORY_ROOT}/common/sources/PESPosition.cpp
   PESPositionTest.cpp
)
target_link_libraries(test_pe_sposition gtest pthread)
add_test(NAME test_pe_sposition COMMAND test_pe_sposition)


#Test tools PE::TOOLS
add_executable(test_pe_tools
   ${REPOSITORY_ROOT}/common/sources/PESPosition.cpp
   ${REPOSITORY_ROOT}/common/sources/PETools.cpp
   PEToolsTest.cpp
)
target_link_libraries(test_pe_tools gtest pthread)
add_test(NAME test_pe_tools COMMAND test_pe_tools)