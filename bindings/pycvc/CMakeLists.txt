# pycvc — SWIG Python bindings for libcvc (Phase-13 modernization deliverable).
#
# Dual-mode:
#   * In-tree  — add_subdirectory(bindings/pycvc) from libcvc with
#                -DCVC_BUILD_PYCVC=ON; links the in-tree `cvc` target and
#                installs the module into the bundle's site-packages, so a
#                published/activatable libcvc prefix carries `import pycvc`.
#   * Standalone — `cmake -S bindings/pycvc -B build
#                   -DCMAKE_PREFIX_PATH="<cvcpkg-prefix>;<installed-libcvc-sdk>"`
#                consumes libcvc via find_package(cvc CONFIG), like volrover3.
cmake_minimum_required(VERSION 3.16)

# Standalone iff this directory is the top of the CMake build.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(pycvc LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  find_package(cvc CONFIG REQUIRED)
  set(_PYCVC_CVC_TARGET cvc::cvc)
else()
  # Part of the libcvc build: the `cvc` library target already exists.
  set(_PYCVC_CVC_TARGET cvc)
endif()

find_package(SWIG 4.0 REQUIRED COMPONENTS python)
include(UseSWIG)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

set_property(SOURCE pycvc.i PROPERTY CPLUSPLUS ON)
# Re-run SWIG when a %include'd facade header changes (ninja otherwise keeps a
# stale wrapper after edits to pycvc_*.h).
set_property(SOURCE pycvc.i PROPERTY DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_geometry.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_volume.h)

swig_add_library(pycvc
  LANGUAGE python
  SOURCES pycvc.i pycvc_geometry.cpp pycvc_volume.cpp
)
target_include_directories(pycvc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pycvc PRIVATE ${_PYCVC_CVC_TARGET} Python3::Module)

set_target_properties(pycvc PROPERTIES
  SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

# Install into the prefix's site-packages so `import pycvc` works from an
# activated libcvc prefix (alongside where python_sdist recipes install).
set(_PYCVC_SITEDIR
  "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
install(TARGETS pycvc LIBRARY DESTINATION "${_PYCVC_SITEDIR}" COMPONENT libcvc)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pycvc.py"
  DESTINATION "${_PYCVC_SITEDIR}" COMPONENT libcvc)

# ctest: import the module and exercise the facades end-to-end. Needs the
# built libcvc on the loader path when run in-tree.
enable_testing()
add_test(
  NAME pycvc_smoke
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pycvc.py
)
set(_PYCVC_TEST_LIBPATH "$<TARGET_FILE_DIR:${_PYCVC_CVC_TARGET}>")
set_tests_properties(pycvc_smoke PROPERTIES
  ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=${_PYCVC_TEST_LIBPATH}:$ENV{LD_LIBRARY_PATH}"
)
