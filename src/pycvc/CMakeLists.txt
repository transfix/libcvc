cmake_minimum_required(VERSION 3.15)

# ---------------------------------------------------------------------------
# pycvc — SWIG Python bindings for libcvc (skeleton).
# Built only when CVC_BUILD_PYTHON=ON (see top-level CMakeLists.txt).
#
# Produces ${CMAKE_BINARY_DIR}/python/{pycvc.py,_pycvc*.so}; point PYTHONPATH
# at that directory (or install) and `import pycvc`.
# ---------------------------------------------------------------------------

if(POLICY CMP0078)
  cmake_policy(SET CMP0078 NEW) # standard target names for swig_add_library
endif()
if(POLICY CMP0086)
  cmake_policy(SET CMP0086 NEW) # pass -module to SWIG
endif()

find_package(SWIG 4.0 REQUIRED)
include(${SWIG_USE_FILE})

# Interpreter (for site-packages layout / running tests), Development.Module
# (Python.h without requiring the embeddable full Development component) and
# NumPy headers for the array conversion helpers.
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

message(STATUS "pycvc: SWIG ${SWIG_VERSION} (${SWIG_EXECUTABLE})")
message(STATUS "pycvc: Python ${Python3_VERSION} (${Python3_EXECUTABLE})")
message(STATUS "pycvc: NumPy ${Python3_NumPy_VERSION} (${Python3_NumPy_INCLUDE_DIRS})")

set(PYCVC_OUTPUT_DIR ${CMAKE_BINARY_DIR}/python)

set_property(SOURCE pycvc.i PROPERTY CPLUSPLUS ON)
set_property(SOURCE pycvc.i PROPERTY USE_SWIG_DEPENDENCIES TRUE)
set_property(SOURCE pycvc.i PROPERTY INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR})

# Keep SWIG's preprocessor in sync with the cvc library feature gates so the
# gated fragments (sdf.i) only wrap symbols that actually exist.  The C++
# compile of the generated wrapper picks the same definitions up from the
# cvc target's PUBLIC compile definitions.
if(CVC_ENABLE_SDF)
  set_property(SOURCE pycvc.i APPEND PROPERTY COMPILE_DEFINITIONS CVC_ENABLE_SDF)
endif()
if(CVC_ENABLE_MESHER)
  set_property(SOURCE pycvc.i APPEND PROPERTY COMPILE_DEFINITIONS CVC_ENABLE_MESHER)
endif()

swig_add_library(pycvc
  TYPE MODULE
  LANGUAGE python
  OUTPUT_DIR ${PYCVC_OUTPUT_DIR}
  OUTFILE_DIR ${CMAKE_CURRENT_BINARY_DIR}
  SOURCES pycvc.i
)

# Place the extension next to the generated pycvc.py proxy so the build tree
# is directly importable.
set_target_properties(pycvc PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${PYCVC_OUTPUT_DIR}
)

target_link_libraries(pycvc PRIVATE cvc::cvc Python3::Module Python3::NumPy)

# Skeleton install: stage the module into <prefix>/lib/pythonX.Y/site-packages.
# The cvcpkg python_sdist recipe (Modernization Phase 7 / recipes/pycvc) will
# supersede this for hermetic-prefix packaging.
set(PYCVC_INSTALL_DIR
  ${CMAKE_INSTALL_LIBDIR}/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages)
install(TARGETS pycvc LIBRARY DESTINATION ${PYCVC_INSTALL_DIR} COMPONENT libcvc)
install(FILES ${PYCVC_OUTPUT_DIR}/pycvc.py DESTINATION ${PYCVC_INSTALL_DIR} COMPONENT libcvc)
