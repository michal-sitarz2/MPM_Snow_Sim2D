cmake_minimum_required(VERSION 3.0)
project(Snow_Simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)

# Safety: do not build in source folder
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Please select another Build Directory!")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# 2D or 3D simulation option
option(USE_3D "Build 3D simulation" OFF)
if(USE_3D)
    add_definitions(-DSIM_3D)
    add_definitions(-DSHADER_DIR="${CMAKE_SOURCE_DIR}/src/gpu/shaders/3d")
else()
    add_definitions(-DSHADER_DIR="${CMAKE_SOURCE_DIR}/src/gpu/shaders/2d")
endif()

add_definitions(
    -DTW_STATIC
    -DTW_NO_LIB_PRAGMA
    -DTW_NO_DIRECT3D
    -DGLEW_STATIC
    -D_CRT_SECURE_NO_WARNINGS
    -DRENDER_DIR="${CMAKE_SOURCE_DIR}/outputs"
)

add_subdirectory(external)

set(MPM_SOURCES
    src/main.cpp
    src/gpu/GPUSolver.cpp
)

if(NOT USE_3D)
    list(APPEND MPM_SOURCES
        src/cpu/Grid.cpp
        src/cpu/Particles.cpp
    )
endif()

add_executable(MPM ${MPM_SOURCES})

target_include_directories(MPM PRIVATE
    src/shared/
    src/cpu/
    src/gpu/
    external/eigen-3.4.0/
    external/glm-0.9.7.1/
    external/glfw-3.1.2/include/
    external/glew-1.13.0/include/
    external/AntTweakBar-1.16/include/
    external/stb_image/
    .
)

target_link_libraries(MPM
    OpenGL::GL
    glfw
    GLEW_1130
)