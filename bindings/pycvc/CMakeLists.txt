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

set_property(SOURCE pycvc.i PROPERTY CPLUSPLUS ON)
# Re-run SWIG when a helper header the interface pulls in changes (ninja
# otherwise keeps a stale wrapper). The directly-wrapped libcvc headers are
# tracked as ordinary compile dependencies of the generated wrapper.
set_property(SOURCE pycvc.i PROPERTY DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_buffer.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_context.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_algorithm.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_state.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_exec.h
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_image.i
  ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_model.i)

# Shared by both modules — defined before either option block so the scene
# module can install/test even when the core module isn't built in this run.
enable_testing()
# site-packages of the prefix interpreter (where python_sdist recipes install).
set(_PYCVC_SITEDIR
  "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
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
  if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
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
  # Re-run SWIG when the render-helper header (or the %import'd pycvc.i) changes.
  set_property(SOURCE pycvc_gl.i PROPERTY DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/pycvc_scene.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pycvc.i)
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
endif() # CVC_BUILD_PYCVC_GL
