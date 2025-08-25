cmake_minimum_required(VERSION 3.15)

project(wujihandcpp LANGUAGES C CXX)

# Get the latest Git tag
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE GIT_TAG_RESULT
)

# Extract version number if tag format is valid
if(GIT_TAG_RESULT EQUAL 0 AND GIT_TAG MATCHES "^v([0-9]+\\.[0-9]+\\.[0-9]+)$")
    string(REGEX REPLACE "^v" "" PROJECT_VERSION ${GIT_TAG})
    message(STATUS "Using Git tag version: ${PROJECT_VERSION}")
else()
    set(PROJECT_VERSION 0.0.0)
    message(STATUS "Unable to extract git tag, using version: ${PROJECT_VERSION}")
endif()

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add compiler options based on compiler
if(MSVC)
    add_compile_options(/W4 /Zc:preprocessor)
    add_compile_definitions(NOMINMAX _CRT_SECURE_NO_WARNINGS)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    # GCC/Clang
    add_compile_options(-Wall -Wextra -Wpedantic -fvisibility=hidden)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set libusb binaries directory (if provided)
set(LIBUSB_BINARY_PATH "" CACHE STRING "Path to libusb binaries directory")
if(NOT LIBUSB_BINARY_PATH STREQUAL "")
    message(STATUS "Using libusb binary from: ${LIBUSB_BINARY_PATH}")
    link_directories(${LIBUSB_BINARY_PATH})
endif()

# Set libusb header files directory (if provided)
if(UNIX)
    set(LIBUSB_HEADER_PATH "/usr/include/libusb-1.0" CACHE STRING "Path to libusb header files directory")
else()
    set(LIBUSB_HEADER_PATH "" CACHE STRING "Path to libusb header files directory")
endif()
if(NOT LIBUSB_HEADER_PATH STREQUAL "")
    message(STATUS "Using libusb header from: ${LIBUSB_HEADER_PATH}")
    include_directories(SYSTEM ${LIBUSB_HEADER_PATH})
endif()

# Get project sources
file(GLOB_RECURSE PROJECT_SOURCE CONFIGURE_DEPENDS
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/*.c
)

add_library(
    ${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE}
)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE libusb-1.0)
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE usb-1.0)
endif()

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(UNIX AND NOT APPLE)
    set(CPACK_GENERATOR "DEB;RPM")

    set(ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
    if(ARCHITECTURE STREQUAL "x86_64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
        set(CPACK_RPM_PACKAGE_ARCHITECTURE "x86_64")
    elseif(ARCHITECTURE STREQUAL "aarch64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
        set(CPACK_RPM_PACKAGE_ARCHITECTURE "aarch64")
    else()
        message(FATAL_ERROR "Unsupported architecture: ${ARCHITECTURE}.")
    endif()

    set(CPACK_DEBIAN_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_DEBIAN_PACKAGE_VERSION "${PROJECT_VERSION}")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Wuji Technology <support@pan-motor.com>")
    set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "A Lightweight C++ SDK for Wujihand")
    set(CPACK_DEBIAN_PACKAGE_HOMEPAGE "https://github.com/Wuji-Technology-Co-Ltd/wujihandcpp")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libusb-1.0-0-dev (>= 2:1.0.23)")
    set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
    set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")

    set(CPACK_RPM_PACKAGE_NAME "${PROJECT_NAME}")
    set(CPACK_RPM_PACKAGE_VERSION "${PROJECT_VERSION}")
    set(CPACK_RPM_PACKAGE_VENDOR "Wuji Technology <support@pan-motor.com>")
    set(CPACK_RPM_PACKAGE_DESCRIPTION "A Lightweight C++ SDK for Wujihand")
    set(CPACK_RPM_PACKAGE_URL "https://github.com/Wuji-Technology-Co-Ltd/wujihandcpp")
    set(CPACK_RPM_PACKAGE_RELEASE "1")
    set(CPACK_RPM_PACKAGE_LICENSE "MIT")
    set(CPACK_RPM_PACKAGE_REQUIRES "libusbx-devel >= 1.0.23")
    set(CPACK_RPM_PACKAGE_GROUP "Development/Libraries")

    set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PROJECT_VERSION}-${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")

    include(CPack)
endif()
