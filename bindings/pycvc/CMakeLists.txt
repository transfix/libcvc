# pycvc — SWIG Python bindings for libcvc (Phase-13 modernization deliverable).
#
# Builds the `pycvc` Python extension from the SWIG-safe facade + interface.
# Consumes libcvc as an external SDK via find_package(cvc CONFIG), exactly
# like volrover3, so it builds against any installed/cvcpkg libcvc.
#
# Standalone configure:
#   cmake -S bindings/pycvc -B build \
#     -DCMAKE_PREFIX_PATH="<cvcpkg-prefix>;<installed-libcvc-sdk>"
cmake_minimum_required(VERSION 3.16)
project(pycvc LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(cvc CONFIG REQUIRED)
find_package(SWIG 4.0 REQUIRED COMPONENTS python)
include(UseSWIG)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)

set_property(SOURCE pycvc.i PROPERTY CPLUSPLUS ON)

swig_add_library(pycvc
  LANGUAGE python
  SOURCES pycvc.i pycvc_geometry.cpp pycvc_volume.cpp
)
target_include_directories(pycvc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pycvc PRIVATE cvc::cvc Python3::Module)

# Place the importable module (_pycvc.so + pycvc.py) together for tests.
set_target_properties(pycvc PROPERTIES
  SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
)

# ctest: import the module and exercise the geometry facade end-to-end.
enable_testing()
add_test(
  NAME pycvc_smoke
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pycvc.py
)
set_tests_properties(pycvc_smoke PROPERTIES
  ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}"
)
