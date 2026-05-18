cmake_minimum_required(VERSION 3.15...3.28)

# Check for CUDA availability early to add it to project languages
# Default ON - will auto-disable if CUDA not found
option(CVC_ENABLE_CUDA "Enable CUDA support for GPU acceleration" ON)

# Auto-detect CUDA availability before defining project
if(CVC_ENABLE_CUDA)
  # Require CMake 3.17+ for native CUDA support
  cmake_minimum_required(VERSION 3.17...3.28)
endif()

# First declare project with C and CXX to initialize CMake properly
project(libcvc
  VERSION 3.2.0
  DESCRIPTION "Computational Visualization Center library from VolumeRover package"
  LANGUAGES C CXX
)

# Now check if CUDA is actually available and enable it if found
if(CVC_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
  else()
    message(STATUS "CUDA compiler not found - disabling CUDA support")
    set(CVC_ENABLE_CUDA OFF)
  endif()
endif()

# Default to C++20 so we can use std::format (and other C++20 features)
# in place of deprecated APIs like sprintf. Callers may still override
# via -DCMAKE_CXX_STANDARD=23. C++20 is supported by GCC 13+, Clang 17+,
# and MSVC 19.29+, all of which the project's CI already exercises.
if(NOT DEFINED CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# MSVC-specific: enable M_PI, silence deprecated POSIX / CRT names, and
# force-include a tiny header that undoes UCRT's `snprintf -> _snprintf`
# macro remap (see inc/cvc/msvc_compat.h for rationale).
if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
    _CRT_SECURE_NO_WARNINGS
    _CRT_NONSTDC_NO_WARNINGS
    _CRT_NONSTDC_NO_DEPRECATE
    # Silence Boost.Asio/WinAPI "Please define _WIN32_WINNT" notice.
    # 0x0601 = Windows 7, matching Boost's own auto-assumed default.
    _WIN32_WINNT=0x0601
  )
  # Force-include the compatibility shim in every translation unit so that
  # third-party headers (Boost.Core, Boost.Asio) see `std::snprintf` rather
  # than the macro-mangled `std::_snprintf`.
  #
  # Use string(APPEND CMAKE_CXX_FLAGS ...) rather than add_compile_options()
  # because the VS generator silently drops /FI"path-with-spaces" entries
  # supplied via add_compile_options, but respects them via the global
  # flags variable. Keep quoting so paths with spaces work.
  set(_CVC_MSVC_FI_HDR "${PROJECT_SOURCE_DIR}/inc/cvc/msvc_compat.h")
  string(APPEND CMAKE_CXX_FLAGS " /FI\"${_CVC_MSVC_FI_HDR}\"")
endif()

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
# Use PROJECT_SOURCE_DIR so libcvc's vendored CMake modules are found first
# even when libcvc is consumed as a subproject (FetchContent/add_subdirectory).
list(PREPEND CMAKE_MODULE_PATH
  "${PROJECT_SOURCE_DIR}/CMake"
)

# Use modern output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# The version number (now using project VERSION)
set(libcvc_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
set(libcvc_VERSION_MINOR ${PROJECT_VERSION_MINOR})
set(libcvc_VERSION ${PROJECT_VERSION})
message(STATUS "libcvc version: ${libcvc_VERSION}")

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
option(USING_STANDARD_INSTALL_LOCATION "Use standard installation paths instead of libcvc subdirectory" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CVC_BUILD_TESTS "Build unit tests with Google Test" ON)
option(CVC_ENABLE_COVERAGE "Enable code coverage analysis (requires GCC or Clang)" OFF)
option(CVC_ENABLE_OPENMP "Enable OpenMP for parallel CPU processing" ON)

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

# Setup OpenMP if enabled
if(CVC_ENABLE_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP support enabled")
    message(STATUS "OpenMP version: ${OpenMP_CXX_VERSION}")
    message(STATUS "OpenMP flags: ${OpenMP_CXX_FLAGS}")
  else()
    message(STATUS "OpenMP not found - disabling OpenMP support")
    set(CVC_ENABLE_OPENMP OFF)
  endif()
else()
  message(STATUS "OpenMP support disabled")
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
  set(ADDED_PREFIX "libcvc")
endif()

# Include GNUInstallDirs for standard installation paths
include(GNUInstallDirs)

# Code coverage support
if(CVC_ENABLE_COVERAGE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(STATUS "Code coverage enabled")
    
    # Add coverage flags to all targets
    set(COVERAGE_COMPILE_FLAGS "--coverage -fprofile-arcs -ftest-coverage")
    set(COVERAGE_LINK_FLAGS "--coverage")
    
    # Apply to C and C++ compilation
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COVERAGE_COMPILE_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COVERAGE_COMPILE_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${COVERAGE_LINK_FLAGS}")
    
    # Find required coverage tools
    find_program(GCOV_PATH gcov)
    find_program(LCOV_PATH lcov)
    find_program(GENHTML_PATH genhtml)
    
    if(LCOV_PATH AND GENHTML_PATH)
      message(STATUS "Found lcov: ${LCOV_PATH}")
      message(STATUS "Found genhtml: ${GENHTML_PATH}")
      
      # Add custom target for generating coverage report
      add_custom_target(coverage
        COMMAND ${CMAKE_COMMAND} -E echo "Clearing previous coverage data..."
        COMMAND ${LCOV_PATH} --directory . --zerocounters
        
        COMMAND ${CMAKE_COMMAND} -E echo "Running tests..."
        COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
        
        COMMAND ${CMAKE_COMMAND} -E echo "Capturing coverage data..."
        COMMAND ${LCOV_PATH} --directory . --capture --output-file coverage.info --ignore-errors mismatch
        
        COMMAND ${CMAKE_COMMAND} -E echo "Filtering coverage data..."
        COMMAND ${LCOV_PATH} --remove coverage.info 
          '/usr/*' 
          '*/test/*' 
          '*/tests/*'
          '*/_deps/*'
          '*/googletest/*'
          --output-file coverage_filtered.info
        
        COMMAND ${CMAKE_COMMAND} -E echo "Generating HTML report..."
        COMMAND ${GENHTML_PATH} coverage_filtered.info 
          --output-directory coverage_html
          --title "libcvc Code Coverage"
          --legend
          --demangle-cpp
          --ignore-errors source
        
        COMMAND ${CMAKE_COMMAND} -E echo ""
        COMMAND ${CMAKE_COMMAND} -E echo "Coverage report generated in: ${CMAKE_BINARY_DIR}/coverage_html/index.html"
        
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating code coverage report"
      )
      
      # Add target to just view the report
      add_custom_target(coverage-view
        COMMAND ${CMAKE_COMMAND} -E echo "Opening coverage report..."
        COMMAND xdg-open coverage_html/index.html || open coverage_html/index.html || start coverage_html/index.html
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Opening code coverage report in browser"
      )
      
    else()
      message(WARNING "lcov or genhtml not found. Install with: sudo apt-get install lcov")
      message(WARNING "Coverage target will not be available")
    endif()
  else()
    message(WARNING "Code coverage only supported with GCC or Clang compilers")
  endif()
endif()

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

# ************* Install documentation & resources *************
install(FILES README.md USAGE.md LICENSE
  DESTINATION ${CMAKE_INSTALL_DOCDIR}
  COMPONENT libcvc)

# Application icons (used by both libcvc and volrover3 components on
# Linux for desktop integration; on macOS/Windows the icon is embedded
# into the app bundle / EXE rather than installed standalone).
if(NOT WIN32 AND NOT APPLE)
  install(FILES share/icons/volrover_logo.png
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/icons/hicolor/256x256/apps
    RENAME volrover_logo.png
    COMPONENT volrover3
    OPTIONAL)
endif()

# ************* CPack configuration *************
include(GNUInstallDirs)

# Ship Microsoft's MSVC CRT redist AND the LLVM OpenMP runtime
# (libomp140.x86_64.dll) alongside our binaries. Without
# CMAKE_INSTALL_OPENMP_LIBRARIES the OpenMP DLL is not bundled and
# volrover3.exe / cvc.dll fail to launch on a vanilla Windows machine
# with "libomp140.x86_64.dll was not found".
set(CMAKE_INSTALL_OPENMP_LIBRARIES TRUE)
# InstallRequiredSystemLibraries installs into bin/ by default, which
# is what we want next to volrover3.exe.
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT volrover3)
include(InstallRequiredSystemLibraries)

# Compute architecture tag for archive file names.
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "(arm|aarch)64|ARM64")
    set(_CVC_ARCH "arm64")
  else()
    set(_CVC_ARCH "x86_64")
  endif()
else()
  set(_CVC_ARCH "x86")
endif()

string(TOLOWER "${CMAKE_SYSTEM_NAME}" _CVC_OS)
if(_CVC_OS STREQUAL "darwin")
  set(_CVC_OS "macos")
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" _CVC_BUILD_LC)
if(NOT _CVC_BUILD_LC)
  set(_CVC_BUILD_LC "release")
endif()

# Per-platform generator selection happens at cpack time via -G; the
# CMakeLists declares all viable generators so the user can pick.
if(WIN32)
  set(CPACK_GENERATOR "NSIS;ZIP")
elseif(APPLE)
  set(CPACK_GENERATOR "DragNDrop;ZIP")
else()
  set(CPACK_GENERATOR "TGZ;DEB")
endif()

set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_VENDOR "Computational Visualization Center")
set(CPACK_PACKAGE_CONTACT "transfix@ices.utexas.edu")
set(CPACK_PACKAGE_HOMEPAGE_URL "https://github.com/transfix/libcvc")
set(CPACK_PACKAGE_VERSION_MAJOR "${PROJECT_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${PROJECT_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${PROJECT_VERSION_PATCH}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "libcvc ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "libcvc-${PROJECT_VERSION}")
set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${PROJECT_SOURCE_DIR}/README.md")
set(CPACK_STRIP_FILES ON)

# Default file-name stem (callers / CI may override CPACK_PACKAGE_FILE_NAME
# to inject a commit SHA for non-tag CI artifacts).
set(CPACK_PACKAGE_FILE_NAME
  "${PROJECT_NAME}-${PROJECT_VERSION}-${_CVC_OS}-${_CVC_ARCH}-${_CVC_BUILD_LC}")

# Component-based packaging: separate libcvc and volrover3 artifacts
set(CPACK_COMPONENTS_ALL libcvc volrover3)
set(CPACK_COMPONENT_LIBCVC_DISPLAY_NAME "libcvc Library")
set(CPACK_COMPONENT_LIBCVC_DESCRIPTION "CVC library, headers, and CMake config files")
set(CPACK_COMPONENT_LIBCVC_REQUIRED ON)
set(CPACK_COMPONENT_VOLROVER3_DISPLAY_NAME "VolumeRover3 Application")
set(CPACK_COMPONENT_VOLROVER3_DESCRIPTION "VolumeRover3 desktop visualization application")
set(CPACK_COMPONENT_VOLROVER3_DEPENDS libcvc)

# Archive component packaging (TGZ, ZIP) — put each component in its own
# archive so libcvc-*.tar.gz and volrover3-*.tar.gz are produced separately
# even when both are built in the same install tree.
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

# DEB component packaging (Linux): produce libcvc and volrover3 .deb files.
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Joe Rivera")
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_LIBCVC_PACKAGE_NAME "libcvc")
set(CPACK_DEBIAN_LIBCVC_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_VOLROVER3_PACKAGE_NAME "volrover3")
set(CPACK_DEBIAN_VOLROVER3_PACKAGE_SECTION "graphics")
set(CPACK_DEBIAN_VOLROVER3_PACKAGE_DEPENDS "libcvc")

# NSIS settings (Windows volrover3 installer)
# Provide a desktop / start-menu entry, branded icons, and modify-PATH.
set(CPACK_NSIS_DISPLAY_NAME "VolumeRover3 ${PROJECT_VERSION}")
set(CPACK_NSIS_PACKAGE_NAME "VolumeRover3")
set(CPACK_NSIS_HELP_LINK "https://github.com/transfix/libcvc")
set(CPACK_NSIS_URL_INFO_ABOUT "https://github.com/transfix/libcvc")
set(CPACK_NSIS_CONTACT "transfix@ices.utexas.edu")
set(CPACK_NSIS_MODIFY_PATH ON)
set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
set(CPACK_NSIS_MUI_FINISHPAGE_RUN "volrover3.exe")
# Branded icons. The .ico file ships in share/icons/.
if(EXISTS "${PROJECT_SOURCE_DIR}/share/icons/volrover_logo.ico")
  set(CPACK_NSIS_MUI_ICON
    "${PROJECT_SOURCE_DIR}/share/icons/volrover_logo.ico")
  set(CPACK_NSIS_MUI_UNIICON
    "${PROJECT_SOURCE_DIR}/share/icons/volrover_logo.ico")
  set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\volrover3.exe")
endif()
# CPACK_PACKAGE_EXECUTABLES creates Start-Menu entries automatically:
# pairs of (binary-name-without-ext, display-name). Setting it here means
# the NSIS installer offers a Start-Menu shortcut to volrover3.
set(CPACK_PACKAGE_EXECUTABLES
  "volrover3" "VolumeRover3"
)
# CPACK_CREATE_DESKTOP_LINKS adds a Desktop shortcut for the same exe.
set(CPACK_CREATE_DESKTOP_LINKS volrover3)

include(CPack)

# Per-component CPack groupings. Configured AFTER include(CPack) so the
# cpack_add_component / cpack_add_component_group commands are defined.
cpack_add_component(libcvc
  DISPLAY_NAME "libcvc Library"
  DESCRIPTION  "CVC library, headers, and CMake config files"
  REQUIRED)
cpack_add_component(volrover3
  DISPLAY_NAME "VolumeRover3 Application"
  DESCRIPTION  "VolumeRover3 desktop visualization application"
  DEPENDS libcvc)
