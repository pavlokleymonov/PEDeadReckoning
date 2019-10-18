cmake_minimum_required(VERSION 3.0)

project (position_engine)

#Set component version
set(PROJECT_MAJOR_VERSION 0)
set(PROJECT_MINOR_VERSION 0)
set(PROJECT_PATCH_LEVEL 1)
set(PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_LEVEL})

file(GLOB SRC
   #${REPOSITORY_ROOT}/common/source/PETools.cpp
   #${REPOSITORY_ROOT}/common/source/*.cpp
   ${REPOSITORY_ROOT}/core/source/PECore.cpp
   ${REPOSITORY_ROOT}/core/source/PECCore.cpp
   #${REPOSITORY_ROOT}/core/source/*.cpp
)


include_directories(
   ${REPOSITORY_ROOT}/core/include
)

# Building a static library with source
add_library ( pe STATIC
   ${SRC}
)
