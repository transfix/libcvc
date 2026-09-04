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
  VERSION 3.3.0
  DESCRIPTION "Computational Visualization Center library from VolumeRover package"
  LANGUAGES C CXX
)

# Now check if CUDA is actually available and enable it if found
if(CVC_ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  
  if(CMAKE_CUDA_COMPILER)
    # The architecture list MUST be set before enable_language(CUDA).
    #
    # Under CMP0104 — NEW for any CMake >= 3.18, which this file's
    # `cmake_minimum_required(VERSION 3.15...3.28)` range selects —
    # enable_language(CUDA) DEFINES CMAKE_CUDA_ARCHITECTURES itself, from
    # nvcc's own default of sm_52. The "if(NOT DEFINED ...)" block further
    # down then saw it as already-defined and never fired, so the intended
    # multi-arch list was dead code and every CUDA build silently produced a
    # single-arch sm_52 fatbin.
    #
    # That is not a cosmetic loss. cubins are only forward-compatible within
    # one major compute capability, and nothing here emits PTX to JIT from, so
    # an sm_52-only libcvc-cuda fails to launch on any sm_6x+ GPU with
    # "no kernel image is available for execution on the device" — including
    # the fleet's own sm_86 cards. Verified by unpacking the built bundle:
    # `cuobjdump -lelf` listed exactly one target, sm_52.
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
      # Maxwell, Pascal, Volta, Turing, Ampere — accepted by every CUDA
      # toolkit we support (50/60/70 warn as deprecated on 12.9).
      set(_cvc_cuda_arch 50 60 70 75 80)

      # Newer generations are added only when THIS nvcc knows them, so a source
      # build on an older toolkit still configures instead of dying with
      # "Unsupported gpu architecture 'compute_100'". The version has to be
      # probed directly: enable_language(CUDA) has not run yet, so
      # CMAKE_CUDA_COMPILER_VERSION does not exist at this point.
      #
      # These are REAL targets, not virtual ones, and that is deliberate: the
      # shipped bundles contain no PTX to JIT from, because SetupCUDA.cmake
      # sets CUDA_RESOLVE_DEVICE_SYMBOLS and the device-link step emits SASS
      # only (measured — adding an `-virtual` entry changes nothing). So an
      # architecture that is not compiled in simply cannot run, and this list
      # is the package's entire compatibility surface.
      execute_process(COMMAND "${CMAKE_CUDA_COMPILER}" --version
                      OUTPUT_VARIABLE _cvc_nvcc_version ERROR_QUIET)
      if(_cvc_nvcc_version MATCHES "release ([0-9]+)\\.([0-9]+)")
        set(_cvc_cuda_ver "${CMAKE_MATCH_1}.${CMAKE_MATCH_2}")
        if(_cvc_cuda_ver VERSION_GREATER_EQUAL 11.8)
          list(APPEND _cvc_cuda_arch 90)        # Hopper
        endif()
        if(_cvc_cuda_ver VERSION_GREATER_EQUAL 12.8)
          list(APPEND _cvc_cuda_arch 100 120)   # Blackwell (datacenter, consumer)
        endif()
      endif()

      # CACHE, not a plain variable: a plain `set` here leaves CMakeCache.txt
      # with no CMAKE_CUDA_ARCHITECTURES entry at all, which silently defeats
      # anything inspecting the cache to confirm what was built (the publish
      # workflow's multi-arch guard does exactly that).
      set(CMAKE_CUDA_ARCHITECTURES "${_cvc_cuda_arch}" CACHE STRING
          "CUDA architectures libcvc compiles device code for")
    endif()
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

# Emscripten disables C++ exception catching by default; cvc throws and catches
# as part of its normal API surface, so JS-based EH must stay on everywhere.
if(EMSCRIPTEN)
  add_compile_options(-fexceptions)
  add_link_options(-fexceptions)
  # Boost's config only detects threading under -pthread, but a single-threaded
  # wasm build must not use -pthread (SharedArrayBuffer + COOP/COEP). Emscripten
  # ships the pthread API as single-threaded stubs, so force Boost onto it —
  # the same define the wasm Boost bundle itself was compiled with
  # (libcvc-deps recipes/boost/build-wasm.sh).
  add_compile_definitions(BOOST_HAS_PTHREADS)
  # Threaded variant: real std::thread/SharedArrayBuffer. Every static lib in
  # the link must also be -pthread (a deps prefix built with CVC_WASM_THREADS=1)
  # and the hosting page needs COOP/COEP headers (examples/wasm/serve.py).
  option(CVC_WASM_PTHREADS "Emscripten: build with -pthread (needs -pthread deps + COOP/COEP hosting)" OFF)
  if(CVC_WASM_PTHREADS)
    add_compile_options(-pthread)
    add_link_options(-pthread)
  endif()
endif()

# MSVC-specific: enable M_PI, silence deprecated POSIX / CRT names, and
# force-include a tiny header that undoes UCRT's `snprintf -> _snprintf`
# macro remap (see inc/cvc/utility/msvc_compat.h for rationale).
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
  set(_CVC_MSVC_FI_HDR "${PROJECT_SOURCE_DIR}/inc/cvc/utility/msvc_compat.h")
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
  
  # NOTE: CMAKE_CUDA_ARCHITECTURES is deliberately NOT set here — by this point
  # enable_language(CUDA) has already defined it, so a default applied here can
  # never take effect. It is set just before enable_language() above; see the
  # comment there. Overriding on the command line still works from either spot.

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
# The `cvc` CLI is packaged SEPARATELY (the cvc-cli / cvc-cli-cuda cvcpkg
# recipes) so it lands deliberately in a user's PATH rather than riding along
# with the libcvc SDK bundle. The libcvc recipe builds with CVC_BUILD_CLI=OFF;
# a plain in-tree build still builds it by default.
option(CVC_BUILD_CLI "Build the cvc command-line executable" ON)
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
        # Mirror ci.yml's "Generate coverage report" filter exactly, so
        # local numbers are comparable to the CI coverage gate: an
        # allowlist of repo sources (a blocklist still counts dep-prefix
        # Boost/CGAL headers inlined into our TUs), minus test code and
        # the vendored XmlRpc++ / contour-spectrum trees
        # (docs/roadmap/COVERAGE_IMPROVEMENT_PLAN.md §4, Option A).
        COMMAND ${LCOV_PATH} --extract coverage.info
          '${CMAKE_SOURCE_DIR}/src/*'
          '${CMAKE_SOURCE_DIR}/inc/*'
          --output-file coverage_filtered.info
          --ignore-errors unused
        COMMAND ${LCOV_PATH} --remove coverage_filtered.info
          '*/test/*'
          '*/tests/*'
          '*_test.cpp'
          '${CMAKE_SOURCE_DIR}/src/xmlrpc/*'
          '*/geometry/cvc-mesher/contour/*'
          '*/cvc/image/third_party/*'
          --output-file coverage_filtered.info
          --ignore-errors unused

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

# ************* cvcGL scene-graph library (opt-in, standalone) *************
# cvcGL is normally pulled in by the pycvc bindings (add_subdirectory below), but
# a pure-C++ consumer — an example, a demo, an app — should not have to build SWIG
# and Python just to link the scene graph. This builds cvcGL (and, with
# CVC_BUILD_EXAMPLES, its examples) in-tree against the just-built cvc core, no
# pycvc required. Guarded on the target so it composes with the pycvc path, which
# reuses this target when it is already defined.
option(CVC_BUILD_CVCGL "Build the cvcGL scene-graph library in-tree (no pycvc needed)" OFF)
if(CVC_BUILD_CVCGL AND NOT TARGET cvcGL)
  add_subdirectory(src/cvcGL)
endif()

# ************* pycvc Python bindings (opt-in) *************
# Off by default so a plain libcvc build needs neither SWIG nor Python.
# Enabled by the cvcpkg recipe so the published bundle carries `import pycvc`.
option(CVC_BUILD_PYCVC "Build and install the pycvc Python bindings" OFF)
if(CVC_BUILD_PYCVC)
  add_subdirectory(bindings/pycvc)
endif()

# ************* Install documentation & resources *************
install(FILES README.md USAGE.md LICENSE
  DESTINATION ${CMAKE_INSTALL_DOCDIR}
  COMPONENT libcvc)

# ************* CPack configuration *************
include(GNUInstallDirs)

# Ship Microsoft's MSVC CRT redist AND the LLVM OpenMP runtime
# (libomp140.x86_64.dll) alongside our binaries. Without
# CMAKE_INSTALL_OPENMP_LIBRARIES the OpenMP DLL is not bundled and
# cvc.dll / cvc.exe fail to launch on a vanilla Windows machine
# with "libomp140.x86_64.dll was not found".
set(CMAKE_INSTALL_OPENMP_LIBRARIES TRUE)
# InstallRequiredSystemLibraries installs into bin/ by default, which
# is what we want next to cvc.exe / cvc.dll.
set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT libcvc)
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
  set(CPACK_GENERATOR "ZIP")
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

# Component-based packaging: the libcvc SDK is the only component.
# (The volrover3 end-user application moved to the volrover repository:
# https://github.com/transfix/volrover)
set(CPACK_COMPONENTS_ALL libcvc)
set(CPACK_COMPONENT_LIBCVC_DISPLAY_NAME "libcvc Library")
set(CPACK_COMPONENT_LIBCVC_DESCRIPTION "CVC library, headers, and CMake config files")
set(CPACK_COMPONENT_LIBCVC_REQUIRED ON)

# Archive component packaging (TGZ, ZIP) — keep component-style archive
# naming so libcvc-*.tar.gz continues to be produced the same way CI
# expects (cpack -D CPACK_COMPONENTS_ALL=libcvc).
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)

# DEB component packaging (Linux): produce the libcvc .deb.
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Joe Rivera")
set(CPACK_DEBIAN_PACKAGE_SECTION "libs")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_LIBCVC_PACKAGE_NAME "libcvc")
set(CPACK_DEBIAN_LIBCVC_PACKAGE_SECTION "libs")

include(CPack)

# Per-component CPack groupings. Configured AFTER include(CPack) so the
# cpack_add_component / cpack_add_component_group commands are defined.
cpack_add_component(libcvc
  DISPLAY_NAME "libcvc Library"
  DESCRIPTION  "CVC library, headers, and CMake config files"
  REQUIRED)
