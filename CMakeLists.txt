cmake_minimum_required(VERSION 3.0)
project(Snow_Simulation)

find_package(OpenGL REQUIRED)

# Safety: do not build in source folder
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Please select another Build Directory!")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Include directories
include_directories(
    external/AntTweakBar-1.16/include/
    external/glfw-3.1.2/include/
    external/glm-0.9.7.1/
    external/glew-1.13.0/include/
    external/assimp-3.0.1270/include/
    external/bullet-2.81-rev2613/src/
    external/eigen-3.4.0/
    external/stb_image/
    .
)

# Libraries
set(ALL_LIBS
    OpenGL::GL
    glfw
    GLEW_1130
)

# Definitions
add_definitions(
    -DTW_STATIC
    -DTW_NO_LIB_PRAGMA
    -DTW_NO_DIRECT3D
    -DGLEW_STATIC
    -D_CRT_SECURE_NO_WARNINGS
    -DRENDER_DIR="${CMAKE_SOURCE_DIR}/render"
)

# External dependencies
add_subdirectory(external)

# Main executable
add_executable(MPM_solver
    src/main.cpp
    src/Particles.cpp
    src/Grid.cpp
    src/Particles.h
    src/Grid.h
    src/Shapes.h
    src/Parameters.h
    src/helpers.h
)

target_link_libraries(MPM_solver ${ALL_LIBS})