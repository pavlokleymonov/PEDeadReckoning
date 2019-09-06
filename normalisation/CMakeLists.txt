cmake_minimum_required(VERSION 3.0)

project (normalisation)

#Set component version
set(PROJECT_MAJOR_VERSION 0)
set(PROJECT_MINOR_VERSION 0)
set(PROJECT_PATCH_LEVEL 1)
set(PROJECT_VERSION ${PROJECT_MAJOR_VERSION}.${PROJECT_MINOR_VERSION}.${PROJECT_PATCH_LEVEL})

# find the module headers
file(GLOB HDR include/*.h)

# find the module sources
file(GLOB SRC source/*.cpp)

# We're building a static library with source
add_library ( normalisation STATIC 
              ${SRC}
              ${HDR}
)

# prefer using the target_include_directories instead of include_directories as it allows per target assignment
target_include_directories( normalisation PUBLIC 
                            ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Assign target PUBLIC_HEADERs for ease of use by parent
set_target_properties ( normalisation PROPERTIES
    PUBLIC_HEADER "${HDR}"
)

