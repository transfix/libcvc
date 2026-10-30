cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# Dual-mode build.
#
#   In-tree  (add_subdirectory(cvc-cli) from the libcvc build): the cvc target
#            already exists, so we just link cvc::cvc.
#   Standalone (the cvc-cli / cvc-cli-cuda cvcpkg recipes build ONLY this dir):
#            no libcvc tree is present, so we project() and find the installed
#            libcvc SDK via find_package(cvc CONFIG). The recipe points
#            CMAKE_PREFIX_PATH at the deps prefix that holds libcvc's SDK.
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(cvc-cli LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(cvc CONFIG REQUIRED)
else()
  # In-tree: re-import the Boost targets cvc links PUBLIC so its transitive link
  # evaluates in this directory scope (src/arisrv/CMakeLists.txt explains why).
  if(NOT TARGET Boost::filesystem)
    find_package(Boost REQUIRED COMPONENTS thread date_time regex filesystem
                 OPTIONAL_COMPONENTS system chrono)
  endif()
endif()

# Unified CVC CLI executable
add_executable(cvc-cli main.cpp)

# Link against the libcvc library (brings in Boost::filesystem, etc.)
target_link_libraries(cvc-cli PRIVATE cvc::cvc)

# Boost::program_options, which cvc::cvc links too (its hosts' command line); named here because
# main.cpp uses it directly.
find_package(Boost REQUIRED COMPONENTS program_options)
target_link_libraries(cvc-cli PRIVATE Boost::program_options)

# Install the executable as "cvc". COMPONENT lets the libcvc bundle exclude the
# CLI (it is CVC_BUILD_CLI=OFF there); the cvc-cli recipe installs this
# component into a user's PATH deliberately.
set_target_properties(cvc-cli PROPERTIES OUTPUT_NAME cvc)
# MSVC writes an import library for an executable that exports symbols, named
# after OUTPUT_NAME and placed in CMAKE_ARCHIVE_OUTPUT_DIRECTORY. Linked against a
# STATIC libcvc the executable does export some, so it tried to write
# build/lib/<config>/cvc.lib, which is the static library it was linking:
# "LNK1149: output filename matches input filename" (release.yml's Windows
# -static lane). Give the executable's import library its own name.
set_target_properties(cvc-cli PROPERTIES ARCHIVE_OUTPUT_NAME cvc-cli)
# Installed bin/cvc finds libcvc and its dependencies in ../lib (CMake/CvcInstallRpath.cmake).
include("${CMAKE_CURRENT_LIST_DIR}/../../CMake/CvcInstallRpath.cmake")
cvc_set_install_rpath(cvc-cli bin)
install(TARGETS cvc-cli RUNTIME DESTINATION bin COMPONENT cvc-cli)

# ---------------------------------------------------------------------------
# Integration tests (exercise the CLI as a subprocess). In-tree only: they link
# the raw cvc target and need GTest from the libcvc superbuild.
# ---------------------------------------------------------------------------
if(CVC_BUILD_TESTS AND NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  add_executable(cvc_cli_test cvc_cli_test.cpp)
  target_link_libraries(cvc_cli_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(cvc_cli_test PRIVATE cxx_std_17)

  # Make sure cvc-cli is built before the test runs
  add_dependencies(cvc_cli_test cvc-cli)

  include(GoogleTest)
  # macOS: defer discovery to test time (PRE_TEST) with a generous timeout so the
  # debug build's slow cold dyld-load of libcvc's dylib closure can't blow the
  # 5 s POST_BUILD DISCOVERY_TIMEOUT (empty --gtest_list_tests → ParseTestList
  # aborts the build). Windows keeps POST_BUILD with the same timeout: the 5 s
  # default failed cvc_cli_test's discovery under the parallel CUDA build on
  # package-windows (TEST_DISCOVERY_TIMEOUT=5 in the failing step). See the full
  # note in src/cvc/tests/CMakeLists.txt, whose test targets already do both.
  # _gtest_discover_tests is the shadowed original module command.
  if(APPLE)
    macro(gtest_discover_tests)
      _gtest_discover_tests(${ARGV} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 120)
    endmacro()
  elseif(WIN32)
    macro(gtest_discover_tests)
      _gtest_discover_tests(${ARGV} DISCOVERY_TIMEOUT 120)
    endmacro()
  endif()
  gtest_discover_tests(cvc_cli_test
    PROPERTIES ENVIRONMENT "CVC_CLI_BINARY=$<TARGET_FILE:cvc-cli>"
  )
endif()
