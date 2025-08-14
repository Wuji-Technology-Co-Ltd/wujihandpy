cmake_minimum_required(VERSION 3.15)

project(wujihand_test)
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

# Set default build type to Release With Debug Info
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
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

# Set libusb binaries directory (if provided)
set(LIBUSB_BINARY_PATH "" CACHE STRING "Path to libusb binaries directory")
if (NOT LIBUSB_BINARY_PATH STREQUAL "")
    message(STATUS "Using libusb binary from: ${LIBUSB_BINARY_PATH}")
    link_directories(${LIBUSB_BINARY_PATH})
endif()

# Set libusb header files directory (if provided)
if (UNIX)
    set(LIBUSB_HEADER_PATH "/usr/include/libusb-1.0" CACHE STRING "Path to libusb header files directory")
else()
    set(LIBUSB_HEADER_PATH "" CACHE STRING "Path to libusb header files directory")
endif()
if (NOT LIBUSB_HEADER_PATH STREQUAL "")
    message(STATUS "Using libusb header from: ${LIBUSB_HEADER_PATH}")
    include_directories(SYSTEM ${LIBUSB_HEADER_PATH})
endif()

# Get project sources
file (GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.c
)

add_executable (
  ${PROJECT_NAME}
  ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE libusb-1.0)
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE usb-1.0)
endif()