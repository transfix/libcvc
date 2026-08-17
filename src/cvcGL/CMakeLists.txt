# cvcGL — shared graphics library (Phase-11 modernization).
#
# The Qt-free VTK scene graph extracted from volrover3: a reactive
# SceneGraph of GraphicsNode/GeometryNode/VolumeNode/... over cvc::state,
# rendering cvc::geometry / cvc::volume via VTK. No Qt — so it is
# SWIG-wrappable (pycvc.gl) and drivable from a non-Qt loop
# (SceneGraph::processEvents() is the re-pumpable pump).
#
# Standalone build (consumes the cvc SDK like volrover3):
#   cmake -S src/cvcGL -B build \
#     -DCMAKE_PREFIX_PATH="<cvcpkg-prefix>;<installed-libcvc-sdk>"
cmake_minimum_required(VERSION 3.16)
project(cvcGL VERSION 3.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT TARGET cvc::cvc)
  find_package(cvc CONFIG REQUIRED)
endif()

# The VTK modules cvcGL links. Kept in one variable so the exact same component
# list is used for find_package here and re-found by find_dependency(VTK) in the
# generated cvcGLConfig.cmake (consumers get the same VTK closure).
set(CVCGL_VTK_COMPONENTS
  CommonCore CommonDataModel CommonExecutionModel CommonTransforms
  FiltersSources FiltersGeneral
  RenderingCore RenderingVolume RenderingVolumeOpenGL2 RenderingOpenGL2
  RenderingAnnotation RenderingFreeType
  InteractionStyle
)
find_package(VTK REQUIRED COMPONENTS ${CVCGL_VTK_COMPONENTS})

file(GLOB CVCGL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
# Header root is <repo>/inc  (headers under inc/cvc/gl/).
get_filename_component(_CVCGL_INC "${CMAKE_CURRENT_SOURCE_DIR}/../../inc" ABSOLUTE)

# Library type follows BUILD_SHARED_LIBS: STATIC by default (a copy embedded in
# each consumer), or a single shared libcvcGL.so with -DBUILD_SHARED_LIBS=ON
# (one in-process copy shared by volrover3 + pycvc_gl — avoids the two-copies
# hazard). Both builds are supported.
add_library(cvcGL ${CVCGL_SOURCES})
# Namespaced alias so in-tree consumers (e.g. pycvc-gl via add_subdirectory) can
# link cvc::cvcGL — the same name find_package(cvcGL CONFIG) exports.
add_library(cvc::cvcGL ALIAS cvcGL)
target_include_directories(cvcGL PUBLIC
  $<BUILD_INTERFACE:${_CVCGL_INC}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cvcGL PUBLIC cvc::cvc ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS cvcGL MODULES ${VTK_LIBRARIES})

set_target_properties(cvcGL PROPERTIES POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION} SOVERSION 3)

# Same reason as src/cvc/CMakeLists.txt:795-801, which cvcGL was missing: the
# sources carry no __declspec(dllexport), so a SHARED Windows build emits a
# .dll with an empty export table and NO import .lib. Every consumer then dies
# with "LNK1181: cannot open input file 'cvcGL.lib'" — including this
# directory's own test executables, which is how it was found: cvcpkg builds
# windows/release/SHARED, and the test targets restored by #182 plus the new
# SceneRenderer tests from #184 were the first things to link cvcGL in that
# configuration. The static default had hidden it.
if(WIN32)
  set_target_properties(cvcGL PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# ── Install + export a package config so consumers can find_package(cvcGL) ──
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS cvcGL EXPORT cvcGLTargets
  ARCHIVE DESTINATION lib   # static libcvcGL.a
  LIBRARY DESTINATION lib   # shared libcvcGL.so
  RUNTIME DESTINATION bin)  # cvcGL.dll (Windows)
install(DIRECTORY "${_CVCGL_INC}/cvc/gl" DESTINATION include/cvc
        FILES_MATCHING PATTERN "*.h")

install(EXPORT cvcGLTargets
  NAMESPACE cvc::
  FILE cvcGLTargets.cmake
  DESTINATION lib/cmake/cvcGL
)

# Substitutes @CVCGL_VTK_COMPONENTS@ (the list cvcGL links) into the config so
# find_dependency(VTK COMPONENTS ...) re-finds the same modules for consumers.
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cvcGLConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cvcGLConfig.cmake"
  INSTALL_DESTINATION lib/cmake/cvcGL
)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cvcGLConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/cvcGLConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/cvcGLConfigVersion.cmake"
  DESTINATION lib/cmake/cvcGL
)

# ── functional smoke test ──
enable_testing()
add_executable(cvcgl_smoke test/cvcgl_smoke.cpp)
target_link_libraries(cvcgl_smoke PRIVATE cvcGL)
add_test(NAME cvcgl_smoke COMMAND cvcgl_smoke)

# Guards the "solid opaque block" volume bug: a stale cached min/max after an
# in-place voxel edit, and a degenerate range collapsing the transfer function.
add_executable(cvcgl_volume_range test/cvcgl_volume_range.cpp)
target_link_libraries(cvcgl_volume_range PRIVATE cvcGL)
add_test(NAME cvcgl_volume_range COMMAND cvcgl_volume_range)

# The explicit traversal: state stack, ORDER-DEPENDENT property semantics,
# per-view divergence and teardown. Runs with a null renderer, so it exercises
# the whole walk without a GL context.
add_executable(cvcgl_traversal test/cvcgl_traversal.cpp)
target_link_libraries(cvcgl_traversal PRIVATE cvcGL)
add_test(NAME cvcgl_traversal COMMAND cvcgl_traversal)

# Headless add + remove path (no VTK renderer). Guards SceneGraph::removeGraphics
# against the no-renderer teardown crash the Python lab would otherwise hit.
add_executable(cvcgl_remove test/cvcgl_remove.cpp)
target_link_libraries(cvcgl_remove PRIVATE cvcGL)
add_test(NAME cvcgl_remove COMMAND cvcgl_remove)

# Teardown-safety regression guard: stresses add/remove/destroy churn and asserts
# no state-handler threads are spawned (the invariant that makes teardown clean).
add_executable(cvcgl_teardown test/cvcgl_teardown.cpp)
target_link_libraries(cvcgl_teardown PRIVATE cvcGL)
add_test(NAME cvcgl_teardown COMMAND cvcgl_teardown)

# Phase-4 texture support: geometry UVs -> SetTCoords + setTexture(cvc::image).
add_executable(cvcgl_texture test/cvcgl_texture.cpp)
target_link_libraries(cvcgl_texture PRIVATE cvcGL)
add_test(NAME cvcgl_texture COMMAND cvcgl_texture)

# Phase-5 zero-copy texture: setTexture aliases the cvc::image buffer (no copy),
# in-place pixel edits are live, texture_modified() bumps the MTime.
add_executable(cvcgl_texture_zerocopy test/cvcgl_texture_zerocopy.cpp)
target_link_libraries(cvcgl_texture_zerocopy PRIVATE cvcGL)
add_test(NAME cvcgl_texture_zerocopy COMMAND cvcgl_texture_zerocopy)

# SceneRenderer: a render target held open across frames (one GL context for a
# whole sequence, a camera that survives the next render, live scene updates).
add_executable(cvcgl_renderer test/cvcgl_renderer.cpp)
target_link_libraries(cvcgl_renderer PRIVATE cvcGL)
add_test(NAME cvcgl_renderer COMMAND cvcgl_renderer)

# Not a test: prints the per-frame cost of a held-open renderer against the
# one-shot-per-frame shape it replaces. Built so the number can be re-measured
# on any machine rather than quoted from a commit message.
add_executable(cvcgl_renderer_bench test/cvcgl_renderer_bench.cpp)
target_link_libraries(cvcgl_renderer_bench PRIVATE cvcGL)
