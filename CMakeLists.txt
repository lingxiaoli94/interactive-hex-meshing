cmake_minimum_required(VERSION 3.18)

project(PolycubeHexMesher CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(HDF5_USE_STATIC_LIBRARIES OFF) # need to be off on Windows

# Allow custom CMakeLists.txt
include(${PROJECT_SOURCE_DIR}/CMakeCustomLists.txt OPTIONAL)

# Torch
# For some reason, on MSVC this cannot be put in a subdirectory.
# If the default torch is located instead of the one downloaded from torch website, you may need to use
# list(APPEND CMAKE_PREFIX_PATH) "...") to force find_package to locate the correct directory.
find_package(Torch REQUIRED)

# Vulkan
find_package(Vulkan REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected; default to release.")
    set(CMAKE_BUILD_TYPE "Release")
endif ()


if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(cxx_warning_flags "-Wall" "-ferror-limit=1")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(cxx_warning_flags "-Wall")
elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options (/permissive-) # silence errors by enabling conformance mode
    set(cxx_warning_flags "/W0")
endif()

add_subdirectory(external)

add_subdirectory(vkoo)

add_subdirectory(geomlib)

add_subdirectory(hex)
