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
   ${REPOSITORY_ROOT}/utils/include
)

#Test tools PE::TOOLS
add_executable(test_pe_tools
   ${GTEST_GMOCK_DIR}/gmock-gtest-all.cc
   ${REPOSITORY_ROOT}/utils/sources/PETools.cpp
   PEToolsTest.cpp
)

target_link_libraries(test_pe_tools pthread)
add_test(NAME test_pe_tools COMMAND test_pe_tools)
