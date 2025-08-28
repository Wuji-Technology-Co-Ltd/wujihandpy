cmake_minimum_required(VERSION 3.15)

project(wujihandpy)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Set C standard to C11
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Disable GNU extensions
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_EXTENSIONS OFF)

# Set default build type to Release
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Add compiler options based on compiler
if(MSVC)
    add_compile_options(/W4 /Zc:preprocessor)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    # GCC/Clang
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Get project sources
file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.c
)

include_directories(${PROJECT_SOURCE_DIR}/src)

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core ${PROJECT_SOURCE})

target_link_libraries(_core PRIVATE pybind11::headers)
target_link_libraries(_core PRIVATE wujihandcpp)

# The install directory is the output (wheel) directory
install(TARGETS _core DESTINATION wujihandpy)
