# TODO: What version of CMake will we be allowed to use? CMAKE_CXX_STANDARD was
#       added in 3.1? (CommonAPI currently allows only <= 3.0)
#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
#set(CMAKE_CXX_EXTENSIONS FALSE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Enable strict errors/warnings by compiler.
# TODO: option()(s?) and compiler checks etc... more warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Woverloaded-virtual")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wformat=2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-conversion-null") #Suppress warning caused by gtest
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wold-style-cast")

# Handle coverage for subcomponent
if (DEFINED ENABLE_COVERAGE)
    # Set compiler and linker flags for unit test / coverage builds

    # GCOV operates on object code.
    # To get coverage value representative for our own code / scope of control only,
    # disable exceptions, asserts, and enable basic optimization to get rid of 'implicit branches'.    
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 --coverage -fno-exceptions -DENABLE_TESTS=1 -DNDEBUG")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 --coverage -fno-exceptions -DENABLE_TESTS=1 -DNDEBUG")
    
    set(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} --coverage")
    find_program(GCOVR_PATH gcovr REQUIRED)
    find_program(CTEST_PATH ctest REQUIRED)

    if (NOT TARGET coverage)
        # XML/HTML coverage
        add_custom_target(coverage
                          COMMENT "Generating coverage report to ${CMAKE_CURRENT_BINARY_DIR}/coverage.xml"
                          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                          COMMAND ${CMAKE_MAKE_PROGRAM} all
                          COMMAND ${CTEST_PATH} --timeout 30 --output-on-failure
                          COMMAND ${GCOVR_PATH} -r ${CMAKE_CURRENT_SOURCE_DIR}/../.. -e .*test-utils.* -e .*Test.* -b --print-summary --xml --output=${CMAKE_CURRENT_BINARY_DIR}/coverage.xml
                          COMMAND ${GCOVR_PATH} -r ${CMAKE_CURRENT_SOURCE_DIR}/../.. -e .*test-utils.* -e .*Test.* -b --html --html-details --output=${CMAKE_CURRENT_BINARY_DIR}/coverage.html
                         )
    endif (NOT TARGET coverage)
endif (DEFINED ENABLE_COVERAGE)
