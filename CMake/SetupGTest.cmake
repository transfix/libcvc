#
# GoogleTest for libcvc's test suites.
#
# cvc_setup_gtest() provides GTest::gtest and GTest::gtest_main (plus
# GTest::gmock and GTest::gmock_main) to the calling directory and everything
# below it, from the first of:
#
#   1. An installed GoogleTest >= CVC_GTEST_VERSION, via find_package(GTest
#      CONFIG) -- e.g. from a cvcpkg prefix on CMAKE_PREFIX_PATH. The catalog's
#      `googletest` bundle is a build dependency of cvcpkg/recipes/libcvc, so
#          cvcpkg install-deps cvcpkg/recipes/libcvc --prefix <p> ...
#          cmake -B build -DCMAKE_PREFIX_PATH=<p> ...
#      finds it there and the configure needs no network.
#   2. Otherwise the same GoogleTest release, downloaded with FetchContent and
#      built in-tree. Until this module existed every test configure did this.
#
# CVC_GTEST_VERSION tracks the catalog's googletest recipe (cy-pca/cvcpkg,
# recipes/googletest), and step 2 fetches the tarball that recipe builds (same
# URL, same SHA-256), so both paths test against the same GoogleTest.
#
# -DCMAKE_DISABLE_FIND_PACKAGE_GTest=ON skips step 1 and always builds from
# source.

include_guard(GLOBAL)

set(CVC_GTEST_VERSION 1.17.0)
set(_CVC_GTEST_URL
  "https://github.com/google/googletest/releases/download/v${CVC_GTEST_VERSION}/googletest-${CVC_GTEST_VERSION}.tar.gz")
set(_CVC_GTEST_SHA256 65fab701d9829d38cb77c14acdc431d2108bfdbf8979e40eb8ae567edf10b27c)
set(_CVC_GTEST_PROBE_DIR "${CMAKE_CURRENT_LIST_DIR}/gtest_probe")

# MSVC only: may the installed GoogleTest be linked into these tests?
#
# The question is the C runtime. GoogleTest passes std::string and friends
# across the gtest DLL boundary, so a Debug (/MDd) test against a Release (/MD)
# gtest.dll -- or the reverse, or a static-CRT (/MT) test against any gtest DLL
# -- compiles and links fine and then corrupts memory at run time. The cvcpkg
# catalog ships one CRT per googletest bundle (`--config release` is /MD,
# `--config debug` is /MDd), so a deps prefix holds exactly one of them, and a
# Debug build against a release prefix -- or a Visual Studio build whose
# configurations span both -- must not use it.
#
# The package's IMPORTED_CONFIGURATIONS stand in for its CRT: every
# configuration being built has to resolve to one with the same debug-ness.
# Reading them needs the package loaded, but find_package() creates GTest::* as
# IMPORTED targets that can never be removed, and a rejected package's targets
# would then shadow the GTest::* aliases FetchContent's googletest defines:
# CMake (3.22 at least) does not flag the clash, the imported targets win, and
# the tests would link the rejected package anyway. So the package is loaded
# first in a throwaway child directory (gtest_probe/), where its targets stay,
# and only what it imports comes back.
#
# Sets <out_use> to TRUE when step 1 may go ahead. Otherwise <out_why> says why
# not, or is left alone when there was no package to reject.
function(_cvc_gtest_msvc_check out_use out_why)
  set(${out_use} FALSE PARENT_SCOPE)
  unset(_cvc_gtest_probe)
  add_subdirectory("${_CVC_GTEST_PROBE_DIR}"
    "${CMAKE_BINARY_DIR}/CMakeFiles/cvc_gtest_probe" EXCLUDE_FROM_ALL)
  if(NOT DEFINED _cvc_gtest_probe)
    return()
  endif()
  list(POP_FRONT _cvc_gtest_probe _dir _type)
  set(_pkg_cfgs ${_cvc_gtest_probe})
  list(JOIN _pkg_cfgs ", " _pkg_cfgs_text)

  if(DEFINED CMAKE_MSVC_RUNTIME_LIBRARY
     AND NOT CMAKE_MSVC_RUNTIME_LIBRARY MATCHES "DLL"
     AND _type STREQUAL "SHARED_LIBRARY")
    string(CONCAT _why "the tests use the static CRT (CMAKE_MSVC_RUNTIME_LIBRARY="
      "${CMAKE_MSVC_RUNTIME_LIBRARY}) but the GoogleTest in ${_dir} is a DLL")
    set(${out_why} "${_why}" PARENT_SCOPE)
    return()
  endif()

  get_property(_multi GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
  if(_multi)
    set(_cfgs ${CMAKE_CONFIGURATION_TYPES})
  elseif(CMAKE_BUILD_TYPE)
    set(_cfgs ${CMAKE_BUILD_TYPE})
  else()
    set(_cfgs NoConfig)
  endif()
  foreach(_cfg IN LISTS _cfgs)
    string(TOUPPER "${_cfg}" _CFG)
    # Debug is the configuration CMake builds against the debug CRT, so it
    # needs the package's DEBUG build. Any other configuration gets the
    # package's same-named build if there is one and otherwise the first one the
    # package lists -- which can be DEBUG whenever the package has a DEBUG build.
    if(NOT _CFG IN_LIST _pkg_cfgs
       AND (_CFG STREQUAL "DEBUG" OR "DEBUG" IN_LIST _pkg_cfgs))
      string(CONCAT _why "the GoogleTest in ${_dir} (configurations: "
        "${_pkg_cfgs_text}) would mix C runtimes with this ${_cfg} build")
      set(${out_why} "${_why}" PARENT_SCOPE)
      return()
    endif()
  endforeach()

  set(${out_use} TRUE PARENT_SCOPE)
endfunction()

function(cvc_setup_gtest)
  # libcvc consumed as a subproject: the parent already has GoogleTest.
  if(TARGET GTest::gtest AND TARGET GTest::gtest_main)
    message(STATUS "GoogleTest: using the GTest:: targets the parent project defines")
    return()
  endif()

  set(_use TRUE)
  set(_why "no GTest >= ${CVC_GTEST_VERSION} CMake package found on CMAKE_PREFIX_PATH")
  if(CMAKE_DISABLE_FIND_PACKAGE_GTest)
    set(_use FALSE)
    set(_why "CMAKE_DISABLE_FIND_PACKAGE_GTest is set")
  elseif(MSVC)
    _cvc_gtest_msvc_check(_use _why)
  endif()

  if(_use)
    find_package(GTest ${CVC_GTEST_VERSION} CONFIG QUIET)
    if(GTest_FOUND)
      message(STATUS "GoogleTest ${GTest_VERSION}: installed package (${GTest_DIR})")
      return()
    endif()
  endif()

  message(STATUS "GoogleTest ${CVC_GTEST_VERSION}: building from source with "
                 "FetchContent -- ${_why}")
  include(FetchContent)
  FetchContent_Declare(googletest
    URL "${_CVC_GTEST_URL}"
    URL_HASH SHA256=${_CVC_GTEST_SHA256}
  )
  # MSVC: build GoogleTest against the dynamic CRT the rest of the build uses,
  # not its own static-CRT default, or the tests cannot link against it.
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  # Test-only: keep GoogleTest out of `cmake --install` and the packages.
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
endfunction()
