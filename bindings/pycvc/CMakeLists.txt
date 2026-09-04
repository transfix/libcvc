# pycvc — SWIG Python bindings for libcvc (Phase-13 modernization deliverable).
#
# Two independently-selectable modules that ship as SEPARATE, side-by-side
# cvcpkg packages (installable together):
#   * pycvc     (CORE=ON,  GL=OFF) — the numpy bindings, no VTK.
#   * pycvc-gl  (CORE=OFF, GL=ON)  — only the scene module; depends on the
#                                    `pycvc` package for the core at runtime.
# In-tree / dev builds default both ON, so one build produces both modules.
#
# Dual-mode:
#   * In-tree  — add_subdirectory(bindings/pycvc) from libcvc with
#                -DCVC_BUILD_PYCVC=ON; links the in-tree `cvc` target.
#   * Standalone — `cmake -S bindings/pycvc -B build
#                   -DCMAKE_PREFIX_PATH="<cvcpkg-prefix>;<installed-libcvc-sdk>"`
#                consumes libcvc via find_package(cvc CONFIG), like volrover3.
#                The bindings pick up CUDA automatically when the resolved
#                libcvc is the CUDA variant (cvc::cvc carries CVC_USING_CUDA in
#                its interface compile defs), so pycvc/pycvc-gl work against
#                both libcvc and libcvc-cuda.
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
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)

# ── SWIG regeneration dependencies (derived, never hand-listed) ──────────────
# swig parses every file a .i pulls in with %include / %import, so all of them
# have to be dependencies of the GENERATE step. A missing one is invisible: the
# stale <module>_wrap.cxx still compiles and links, and the only symptom is a
# Python API that silently reflects the PREVIOUS version of the header. Being
# an ordinary compile dependency of the generated wrapper is NOT enough — that
# recompiles the stale wrapper, it does not regenerate it.
#
# So scan the set out of the interface files rather than hand-listing it: the
# hand-written list drifted, and every directly-wrapped libcvc/cvcGL header
# (geometry.h, volume.h, cvc/gl/*.h, ...) was missing from it.

# cvc_swig_interface_deps(<out-var> <entry .i> [SEARCH_DIRS <dir>...])
#   Absolute paths of the project files <entry .i> pulls in via %include /
#   %import (and quoted #include, for the headers pasted into the wrapper's
#   %{ %} block), following nested .i files. Every scanned file also becomes a
#   CMAKE_CONFIGURE_DEPENDS, so adding a %include re-runs the scan by itself.
function(cvc_swig_interface_deps outvar entry)
  cmake_parse_arguments(PARSE_ARGV 2 _arg "" "" "SEARCH_DIRS")
  set(_pending "${entry}")
  set(_scanned "")
  set(_deps "")
  while(_pending)
    list(POP_FRONT _pending _file)
    if("${_file}" IN_LIST _scanned)
      continue()
    endif()
    list(APPEND _scanned "${_file}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${_file}")
    # %include "x.h" / %import(module="m") "x.i" / #include "x.h"
    file(STRINGS "${_file}" _refs REGEX "^[ \t]*[%#](include|import)[^\"]*\"[^\"]+\"")
    foreach(_ref IN LISTS _refs)
      # the LAST quoted token on the line is the file (%import may carry
      # quoted options ahead of it).
      string(REGEX MATCH "\"([^\"]+)\"[^\"]*$" _match "${_ref}")
      set(_rel "${CMAKE_MATCH_1}")
      set(_resolved "")
      foreach(_dir IN LISTS _arg_SEARCH_DIRS)
        if(EXISTS "${_dir}/${_rel}")
          get_filename_component(_resolved "${_dir}/${_rel}" ABSOLUTE)
          break()
        endif()
      endforeach()
      if(NOT _resolved)
        # Not one of ours (VTK, Python, ...) — the compiler tracks those for
        # the wrapper's own compilation. A %include'd one is a real gap though,
        # since swig reads it to build the wrapper.
        if(_ref MATCHES "^[ \t]*%")
          message(AUTHOR_WARNING
            "pycvc: '${_rel}' (pulled in by ${_file}) is outside the SWIG search "
            "dirs, so editing it will not regenerate the wrapper.")
        endif()
        continue()
      endif()
      if(NOT "${_resolved}" IN_LIST _deps)
        list(APPEND _deps "${_resolved}")
        if(_resolved MATCHES "\\.i$")
          list(APPEND _pending "${_resolved}")
        endif()
      endif()
    endforeach()
  endwhile()
  list(REMOVE_ITEM _deps "${entry}")  # already swig's MAIN_DEPENDENCY
  set(${outvar} "${_deps}" PARENT_SCOPE)
endfunction()

# Header roots of the resolved libcvc/cvcGL targets, so the scan finds the
# wrapped headers in a standalone build (installed SDK) too. In-tree targets
# carry $<BUILD_INTERFACE:...> genexes; unwrap those and drop anything else
# that is still a genex.
function(cvc_target_header_dirs outvar)
  set(_dirs "")
  foreach(_tgt IN LISTS ARGN)
    if(NOT TARGET ${_tgt})
      continue()
    endif()
    get_target_property(_inc ${_tgt} INTERFACE_INCLUDE_DIRECTORIES)
    foreach(_dir IN LISTS _inc)
      string(REGEX REPLACE "^\\$<BUILD_INTERFACE:(.+)>$" "\\1" _dir "${_dir}")
      if(NOT _dir MATCHES "\\$<" AND IS_DIRECTORY "${_dir}")
        list(APPEND _dirs "${_dir}")
      endif()
    endforeach()
  endforeach()
  set(${outvar} "${_dirs}" PARENT_SCOPE)
endfunction()

# pycvc_*.h/.i sit next to this file; the wrapped libcvc headers come from the
# source tree in-tree and from the installed SDK standalone.
cvc_target_header_dirs(_PYCVC_CVC_HEADER_DIRS ${_PYCVC_CVC_TARGET})
set(_PYCVC_SWIG_SEARCH_DIRS "${CMAKE_CURRENT_SOURCE_DIR}" ${_PYCVC_CVC_HEADER_DIRS})
if(NOT CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  list(INSERT _PYCVC_SWIG_SEARCH_DIRS 1 "${CMAKE_CURRENT_SOURCE_DIR}/../../inc")
endif()

set_property(SOURCE pycvc.i PROPERTY CPLUSPLUS ON)
cvc_swig_interface_deps(_PYCVC_SWIG_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/pycvc.i"
  SEARCH_DIRS ${_PYCVC_SWIG_SEARCH_DIRS})
set_property(SOURCE pycvc.i PROPERTY DEPENDS ${_PYCVC_SWIG_DEPS})

# Shared by both modules — defined before either option block so the scene
# module can install/test even when the core module isn't built in this run.
enable_testing()
# site-packages of the prefix interpreter (where python_sdist recipes install).
#
# ASK the interpreter; do not assume a layout. The POSIX scheme is
# lib/pythonX.Y/site-packages, but Windows CPython uses Lib/site-packages and
# searches ONLY that. Hardcoding the POSIX form meant a Windows install landed
# in a directory sys.path never contains: `import pycvc` failed even though the
# files were sitting in the prefix, which reads as a missing package rather
# than a misfiled one.
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
    "import os,sys,sysconfig;print(os.path.relpath(sysconfig.get_paths()['platlib'],sys.prefix).replace(os.sep,'/'))"
  OUTPUT_VARIABLE _PYCVC_SITEDIR
  OUTPUT_STRIP_TRAILING_WHITESPACE
  RESULT_VARIABLE _pycvc_sitedir_rc)
if(NOT _pycvc_sitedir_rc EQUAL 0 OR _PYCVC_SITEDIR STREQUAL "" OR _PYCVC_SITEDIR MATCHES "^\\.\\.")
  if(WIN32)
    set(_PYCVC_SITEDIR "Lib/site-packages")
  else()
    set(_PYCVC_SITEDIR
      "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
  endif()
endif()
message(STATUS "pycvc: installing python modules to \${CMAKE_INSTALL_PREFIX}/${_PYCVC_SITEDIR}")
# libcvc.so must be on the loader path for the tests; the dual-mode cvc target's
# output dir works both in-tree (built target) and standalone (imported target).
set(_PYCVC_TEST_LIBPATH "$<TARGET_FILE_DIR:${_PYCVC_CVC_TARGET}>")

# ── Core pycvc — cvc::geometry/volume facades + zero-copy numpy views ──
option(CVC_BUILD_PYCVC_CORE "Build the core pycvc numpy bindings module" ON)
if(CVC_BUILD_PYCVC_CORE)
  swig_add_library(pycvc
    LANGUAGE python
    SOURCES pycvc.i pycvc_context.cpp pycvc_algorithm.cpp pycvc_state.cpp pycvc_exec.cpp
  )
  target_include_directories(pycvc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(pycvc PRIVATE ${_PYCVC_CVC_TARGET} Python3::Module Python3::NumPy)
  set_target_properties(pycvc PROPERTIES
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
  install(TARGETS pycvc LIBRARY DESTINATION "${_PYCVC_SITEDIR}" COMPONENT pycvc)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pycvc.py"
    DESTINATION "${_PYCVC_SITEDIR}" COMPONENT pycvc)
  # ctest: facades + zero-copy numpy views end-to-end.
  foreach(_t pycvc_smoke:test_pycvc.py pycvc_numpy:test_pycvc_numpy.py
             pycvc_memory:test_pycvc_memory.py pycvc_threading:test_pycvc_threading.py
             pycvc_app:test_pycvc_app.py pycvc_filters:test_pycvc_filters.py
             pycvc_algorithm:test_pycvc_algorithm.py
             pycvc_image:test_pycvc_image.py
             pycvc_model:test_pycvc_model.py
             pycvc_state:test_pycvc_state.py
             pycvc_async:test_pycvc_async.py
             pycvc_exec:test_pycvc_exec.py
             pycvc_world_clock:test_pycvc_world_clock.py
             pycvc_integration:test_pycvc_integration.py)
    string(REPLACE ":" ";" _pair "${_t}")
    list(GET _pair 0 _name)
    list(GET _pair 1 _script)
    add_test(NAME ${_name}
             COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${_script})
    set_tests_properties(${_name} PROPERTIES ENVIRONMENT
      "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=${_PYCVC_TEST_LIBPATH}:$ENV{LD_LIBRARY_PATH}")
  endforeach()
endif() # CVC_BUILD_PYCVC_CORE

# ── pycvc_gl — direct wrap of the cvcGL scene graph (scriptable; needs VTK) ──
# pycvc_gl `%import`s pycvc.i for cross-module types and `import pycvc` at
# runtime, so the pycvc-gl package depends on the pycvc package. It wraps the
# REAL cvcGL classes (SceneGraph + the node hierarchy) directly — no facade — so
# it does not need the core module built here.
option(CVC_BUILD_PYCVC_GL "Build the pycvc.gl scene bindings (needs cvcGL + VTK)" ON)
if(CVC_BUILD_PYCVC_GL)
  # cvcGL (the sibling scene-graph library) is dual-mode like cvc above:
  #   * Standalone — consume the installed cvcGL via find_package(cvcGL CONFIG)
  #                  and link the imported cvc::cvcGL (the `cvcgl` cvcpkg pkg).
  #   * In-tree    — build it here via add_subdirectory and link the `cvcGL`
  #                  target (which also aliases cvc::cvcGL).
  if(TARGET cvcGL)
    # Already built in-tree (e.g. top-level CVC_BUILD_CVCGL) — reuse that target.
    set(_PYCVC_CVCGL_TARGET cvcGL)
    set(_PYCVC_GL_TEST_LIBPATH "$<TARGET_FILE_DIR:cvcGL>:")
  elseif(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    find_package(cvcGL CONFIG REQUIRED)
    set(_PYCVC_CVCGL_TARGET cvc::cvcGL)
    # Imported cvcGL is a static archive — no runtime dir to add to the loader
    # path, so the scene test relies only on the cvc/VTK dirs.
    set(_PYCVC_GL_TEST_LIBPATH "")
  else()
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../src/cvcGL
                     ${CMAKE_CURRENT_BINARY_DIR}/cvcGL)
    set(_PYCVC_CVCGL_TARGET cvcGL)
    set(_PYCVC_GL_TEST_LIBPATH "$<TARGET_FILE_DIR:cvcGL>:")
  endif()
  set_property(SOURCE pycvc_gl.i PROPERTY CPLUSPLUS ON)
  # Same derived dependency scan as the core module (see above). Recursing
  # through the %import'd pycvc.i means the cvcGL headers wrapped here AND the
  # core headers whose types cross the module boundary all retrigger swig.
  cvc_target_header_dirs(_PYCVC_CVCGL_HEADER_DIRS ${_PYCVC_CVCGL_TARGET})
  cvc_swig_interface_deps(_PYCVC_GL_SWIG_DEPS "${CMAKE_CURRENT_SOURCE_DIR}/pycvc_gl.i"
    SEARCH_DIRS ${_PYCVC_SWIG_SEARCH_DIRS} ${_PYCVC_CVCGL_HEADER_DIRS})
  set_property(SOURCE pycvc_gl.i PROPERTY DEPENDS ${_PYCVC_GL_SWIG_DEPS})
  # pycvc_scene.cpp holds only the standalone VTK render helpers + the Python
  # vtkProp bridge (free functions); the scene model itself is the directly
  # %include'd cvcGL headers, which cross cvc::geometry/cvc::volume in via %import.
  swig_add_library(pycvc_gl
    LANGUAGE python
    SOURCES pycvc_gl.i pycvc_scene.cpp
  )
  target_include_directories(pycvc_gl PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(pycvc_gl PRIVATE ${_PYCVC_CVCGL_TARGET} ${_PYCVC_CVC_TARGET} Python3::Module Python3::NumPy)
  find_package(VTK REQUIRED COMPONENTS RenderingCore RenderingOpenGL2 IOImage InteractionStyle)
  target_link_libraries(pycvc_gl PRIVATE ${VTK_LIBRARIES})
  vtk_module_autoinit(TARGETS pycvc_gl MODULES ${VTK_LIBRARIES})
  # VTK Python bridge (F3): link the wrapping-core lib that defines
  # vtkPythonUtil::* (from the vtk-python cvcpkg package — installs side-by-side
  # with vtk in the same prefix). vtkPythonUtil.h is already on the include path
  # via VTK::CommonCore. VTK's monolithic CMake config doesn't enumerate the
  # Python modules, so find the plain shared lib directly rather than as a
  # find_package COMPONENT.
  # The wrapping-core lib carries the Python-ABI suffix, e.g.
  # libvtkWrappingPythonCore3.12-9.5.so — construct that exact name (plus
  # unsuffixed fallbacks for other VTK packagings).
  find_library(VTK_WRAPPING_PYTHONCORE
    NAMES vtkWrappingPythonCore${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}-9.5
          vtkWrappingPythonCore-9.5 vtkWrappingPythonCore
    HINTS ${CMAKE_PREFIX_PATH} PATH_SUFFIXES lib)
  if(NOT VTK_WRAPPING_PYTHONCORE)
    message(FATAL_ERROR
      "libvtkWrappingPythonCore not found — install the `vtk-python` cvcpkg "
      "package (VTK built with VTK_WRAP_PYTHON=ON) into the prefix.")
  endif()
  target_link_libraries(pycvc_gl PRIVATE ${VTK_WRAPPING_PYTHONCORE} Python3::Module)
  set_target_properties(pycvc_gl PROPERTIES
    SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  )
  # pycvc_gl installs as a PACKAGE: the SWIG proxy becomes the package
  # __init__.py, the extension module lives inside it, and the pure-Python scene
  # helpers (pymod_gl/{camera,vehicle,lab,scenes}.py) ship as submodules — so a
  # consumer gets BOTH the bindings (pycvc_gl.SceneGraph) and the reusable scene
  # utilities (pycvc_gl.camera.ChaseCamera, pycvc_gl.vehicle.VehiclePose,
  # pycvc_gl.lab.Lab, pycvc_gl.scenes.*) from one import, with no external package.
  install(TARGETS pycvc_gl LIBRARY DESTINATION "${_PYCVC_SITEDIR}/pycvc_gl" COMPONENT pycvc_gl)
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/pycvc_gl.py"
    DESTINATION "${_PYCVC_SITEDIR}/pycvc_gl" RENAME __init__.py COMPONENT pycvc_gl)
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pymod_gl/"
    DESTINATION "${_PYCVC_SITEDIR}/pycvc_gl" COMPONENT pycvc_gl
    FILES_MATCHING PATTERN "*.py")
  add_test(NAME pycvc_gl_smoke
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pycvc_gl.py)
  set_tests_properties(pycvc_gl_smoke PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=${_PYCVC_GL_TEST_LIBPATH}${_PYCVC_TEST_LIBPATH}:$ENV{LD_LIBRARY_PATH}")
  # VTK Python bridge test — skips gracefully unless the vtk-python wrappers
  # (vtkmodules) are importable; runs the vtkProp<->vtkActor round trip when they are.
  add_test(NAME pycvc_vtk
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pycvc_vtk.py)
  set_tests_properties(pycvc_vtk PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=${_PYCVC_GL_TEST_LIBPATH}${_PYCVC_TEST_LIBPATH}:$ENV{LD_LIBRARY_PATH}")
  # Phase-5 zero-copy texture: node.set_texture + image.numpy() live edit +
  # texture_modified (the strong VTK-inspection check self-skips without vtkmodules).
  add_test(NAME pycvc_texture
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pycvc_texture.py)
  set_tests_properties(pycvc_texture PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=${_PYCVC_GL_TEST_LIBPATH}${_PYCVC_TEST_LIBPATH}:$ENV{LD_LIBRARY_PATH}")
  # CameraController: navigation + full cvc::state config (both directions),
  # construct from an injected app OR a SceneRenderer, canonical viewer path.
  add_test(NAME pycvc_gl_camera
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_pycvc_gl_camera.py)
  set_tests_properties(pycvc_gl_camera PROPERTIES ENVIRONMENT
    "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};LD_LIBRARY_PATH=${_PYCVC_GL_TEST_LIBPATH}${_PYCVC_TEST_LIBPATH}:$ENV{LD_LIBRARY_PATH}")
endif() # CVC_BUILD_PYCVC_GL
