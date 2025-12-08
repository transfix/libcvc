cmake_minimum_required(VERSION 3.15...3.28)

# Check for CUDA early to add it to project languages
option(CVC_ENABLE_CUDA "Enable CUDA support for GPU acceleration" OFF)

if(CVC_ENABLE_CUDA)
  # Require CMake 3.17+ for native CUDA support
  cmake_minimum_required(VERSION 3.17...3.28)
endif()

# Define project with languages
if(CVC_ENABLE_CUDA)
  project(trans-cvc
    VERSION 2.0.0
    DESCRIPTION "Computational Visualization Center library from VolumeRover package"
    LANGUAGES C CXX CUDA
  )
else()
  project(trans-cvc
    VERSION 2.0.0
    DESCRIPTION "Computational Visualization Center library from VolumeRover package"
    LANGUAGES C CXX
  )
endif()

# Require C++14 as a minimum, but allow newer standards
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Use modern CMake policies
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW) # Use <Package>_ROOT variables
endif()
if(POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW) # MSVC runtime library flags
endif()

# Only build what's being actively developed
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# CMake Modules
list(PREPEND CMAKE_MODULE_PATH
  "${CMAKE_SOURCE_DIR}/CMake"
)

# Use modern output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# The version number (now using project VERSION)
set(transcvc_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(transcvc_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(transcvc_VERSION ${PROJECT_VERSION})
message(STATUS "trans-cvc version: ${transcvc_VERSION}")

# Configure CUDA if enabled
if(CVC_ENABLE_CUDA)
  # Set CUDA standard to match C++ standard
  if(NOT DEFINED CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD ${CMAKE_CXX_STANDARD})
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  endif()
  
  # Enable CUDA separable compilation for device code
  set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
  
  # Set CUDA architectures (adjust as needed for your target GPUs)
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    # Default to common architectures: Maxwell, Pascal, Volta, Turing, Ampere
    set(CMAKE_CUDA_ARCHITECTURES "50;60;70;75;80")
  endif()
  
  message(STATUS "CUDA support enabled")
  message(STATUS "CUDA compiler: ${CMAKE_CUDA_COMPILER}")
  message(STATUS "CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")
  message(STATUS "CUDA standard: ${CMAKE_CUDA_STANDARD}")
else()
  message(STATUS "CUDA support disabled")
endif()

# Build options
option(DISABLE_CGAL "Disable CGAL even if available on the build system" OFF)
option(USING_STANDARD_INSTALL_LOCATION "Use standard installation paths instead of trans-cvc subdirectory" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CVC_BUILD_TESTS "Build unit tests with Google Test" ON)

# Include macros for setting up library dependencies
include(SetupFFTW)
include(SetupBoost)
include(SetupGSL)

if(NOT DISABLE_CGAL)
  include(SetupCGAL)
endif()

if(CVC_ENABLE_CUDA)
  include(SetupCUDA)
endif()

# Platform-specific settings
if(APPLE)
  # Possibly prevent crashing on OSX (historical fix from 05/11/2012)
  add_compile_definitions(_GLIBCXX_FULLY_DYNAMIC_STRING)
endif()

# Installation prefix handling
if(USING_STANDARD_INSTALL_LOCATION)
  set(ADDED_PREFIX ".")
else()
  set(ADDED_PREFIX "trans-cvc")
endif()

# Include GNUInstallDirs for standard installation paths
include(GNUInstallDirs)

# Enable testing with Google Test
if(CVC_BUILD_TESTS)
  enable_testing()
  
  # Fetch Google Test
  include(FetchContent)
  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
  )
  
  # For Windows: Prevent overriding the parent project's compiler/linker settings
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  
  # Make Google Test available
  FetchContent_MakeAvailable(googletest)
  
  message(STATUS "Google Test enabled - tests will be built")
endif()

add_subdirectory(src)
