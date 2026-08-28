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
  CommonSystem # vtkTimerLog (ImGuiOverlay frame timing) — MSVC link needs it explicit
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

# ---------------------------------------------------------------------------
# Dear ImGui overlay (optional): real menus/panels/widgets drawn inside the VTK
# render, native AND WebAssembly. The cvcpkg imgui bundle ships no CMake config,
# so locate it by header/lib like the other config-less deps, and compile the
# stock imgui_impl_opengl3 backend (shipped as SOURCE in the bundle's
# include/imgui/backends) straight into cvcGL — there is no ImGui platform
# backend: ImGuiOverlay feeds ImGuiIO from the VTK interactor itself, which is
# what lets the same code path work in the browser (VTK owns the canvas).
option(CVC_ENABLE_IMGUI "Build the cvcGL Dear ImGui overlay (ImGuiOverlay)" ON)
if(CVC_ENABLE_IMGUI)
  find_path(CVC_IMGUI_INCLUDE_DIR imgui.h PATH_SUFFIXES imgui)
  find_library(CVC_IMGUI_LIBRARY NAMES imgui)
  if(CVC_IMGUI_INCLUDE_DIR AND CVC_IMGUI_LIBRARY
     AND EXISTS "${CVC_IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp")
    target_sources(cvcGL PRIVATE "${CVC_IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp")
    # PUBLIC: demos/consumers write ImGui::* in their draw callbacks.
    target_include_directories(cvcGL PUBLIC "$<BUILD_INTERFACE:${CVC_IMGUI_INCLUDE_DIR}>")
    target_link_libraries(cvcGL PUBLIC "${CVC_IMGUI_LIBRARY}")
    target_compile_definitions(cvcGL PUBLIC CVC_ENABLE_IMGUI)
    if(EMSCRIPTEN) # WebGL2 loader for the browser build
      set_source_files_properties("${CVC_IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp"
                                  PROPERTIES COMPILE_DEFINITIONS IMGUI_IMPL_OPENGL_ES3)
    endif()
    message(STATUS "cvcGL: Dear ImGui overlay ENABLED (${CVC_IMGUI_LIBRARY})")
  else()
    set(CVC_ENABLE_IMGUI OFF)
    message(STATUS "cvcGL: imgui not found -- ImGuiOverlay compiles to no-ops")
  endif()
endif()

vtk_module_autoinit(TARGETS cvcGL MODULES ${VTK_LIBRARIES})

# X11 is used by CameraController for Quake-style pointer capture (warp the cursor
# to centre each frame so mouse-look is continuous). Optional: without it, captured
# look degrades to cursor-delta. Linux/X11 only.
#
# cvcGL is a STATIC archive, so its PRIVATE link deps propagate to consumers through
# the exported INTERFACE_LINK_LIBRARIES. find_package(X11)'s ${X11_LIBRARIES} are
# ABSOLUTE builder paths (e.g. .../py/lib/libX11.so) — baking those into
# cvcGLTargets.cmake breaks any downstream consumer (volrover3) on another machine
# with "missing and no known rule to make it". cvcGL only calls Xlib (XWarpPointer/
# XFlush), so link libX11 BY NAME: the export then carries a relocatable "-lX11" the
# consumer's own linker resolves, not a path from this builder.
if(UNIX AND NOT APPLE)
  find_package(X11)
  if(X11_FOUND)
    target_link_libraries(cvcGL PRIVATE X11)
    target_include_directories(cvcGL PRIVATE ${X11_INCLUDE_DIR})
  endif()
endif()

# macOS clipboard backend. Named EXPLICITLY because the file(GLOB "*.cpp") above
# cannot see Objective-C++ — which is how Clipboard.cpp's __APPLE__ branch ended
# up calling two extern "C" symbols that nothing defined, leaving libcvcGL
# unlinkable on macOS until cvcGL was first built in CI. Link AppKit here rather
# than relying on the -framework Cocoa that VTK's RenderingUI happens to drag
# in, so this does not silently break if that transitive dependency changes.
if(APPLE)
  target_sources(cvcGL PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/Clipboard_mac.mm")
  target_link_libraries(cvcGL PRIVATE "-framework AppKit" "-framework Foundation")
endif()

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

# Scene-owned lighting: lights must survive setRenderer, so a second renderer
# (offscreen capture, second window) lights the scene identically.
add_executable(cvcgl_lighting test/cvcgl_lighting.cpp)
target_link_libraries(cvcgl_lighting PRIVATE cvcGL)
add_test(NAME cvcgl_lighting COMMAND cvcgl_lighting)

# Shadows must not hide volumes: the shadow pass draws only the opaque layer, so
# the sequence has to carry a volumetric pass or every VolumeNode vanishes the
# moment shadows switch on. Structural check (no GPU) + offscreen render.
add_executable(cvcgl_shadow_volume test/cvcgl_shadow_volume.cpp)
target_link_libraries(cvcgl_shadow_volume PRIVATE cvcGL)
add_test(NAME cvcgl_shadow_volume COMMAND cvcgl_shadow_volume)

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

# The publisher keeps the state tree correct while keeping writes off the render
# path: the value arrives (after a flush), the last write wins, and an external
# write still drives the node.
add_executable(cvcgl_state_publisher test/cvcgl_state_publisher.cpp)
target_link_libraries(cvcgl_state_publisher PRIVATE cvcGL)
add_test(NAME cvcgl_state_publisher COMMAND cvcgl_state_publisher)

add_executable(cvcgl_state_probe test/cvcgl_state_probe.cpp)
target_link_libraries(cvcgl_state_probe PRIVATE cvcGL)

# Not a test: prints what posing a deep hierarchy costs. The shape is the
# lsystem_forest example's (70 trees of ~21 nested modules), which is what
# exposed the O(nodes * depth) world-transform cascade.
add_executable(cvcgl_transform_bench test/cvcgl_transform_bench.cpp)
target_link_libraries(cvcgl_transform_bench PRIVATE cvcGL)

# Not a test: prints the per-frame cost of a held-open renderer against the
# one-shot-per-frame shape it replaces. Built so the number can be re-measured
# on any machine rather than quoted from a commit message.
add_executable(cvcgl_renderer_bench test/cvcgl_renderer_bench.cpp)
target_link_libraries(cvcgl_renderer_bench PRIVATE cvcGL)

# CameraController: orbit + Quake-fly navigation math (Z-up). Drives a bare
# vtkCamera and asserts framing, the seamless orbit<->fly toggle, WASD/strafe/
# vertical motion, mouse-look, and orbit-drag — no window needed.
add_executable(cvcgl_camera test/cvcgl_camera.cpp)
target_link_libraries(cvcgl_camera PRIVATE cvcGL)
add_test(NAME cvcgl_camera COMMAND cvcgl_camera)

# FpsHud: the on-screen FPS readout's cvc::state surface (enabled/toggle key/
# update rate, two-way binding, key-sym normalization) — no window needed.
add_executable(cvcgl_fps_hud test/cvcgl_fps_hud.cpp)
target_link_libraries(cvcgl_fps_hud PRIVATE cvcGL)
add_test(NAME cvcgl_fps_hud COMMAND cvcgl_fps_hud)

# LightNode is a light that IS a scene node. The claim worth guarding is
# PARENTING: a light hung under another node inherits that node's transform, so
# a lamp on a vehicle travels with it. Also pins the two rules that are easy to
# regress — a hidden light contributes no vtkLight, and lights that exist but are
# hidden must NOT be papered over with the fallback headlight.
add_executable(cvcgl_light_node test/cvcgl_light_node.cpp)
target_link_libraries(cvcgl_light_node PRIVATE cvcGL)
add_test(NAME cvcgl_light_node COMMAND cvcgl_light_node)

# Every cvcGL setting must be IN the state tree and bound BOTH ways. This caught
# cvc::state::SEPARATOR initialising to "" outside libcvc, which had been
# silently flattening every state_object key in cvcGL — so it stays a gate.
add_executable(cvcgl_state_binding test/cvcgl_state_binding.cpp)
target_link_libraries(cvcgl_state_binding PRIVATE cvcGL)
add_test(NAME cvcgl_state_binding COMMAND cvcgl_state_binding)

# Adding a shadow-casting light between frames used to index past the end of the
# baker's cached shadow-map vector and dereference garbage — which is why raising
# the wash-light count died while lowering it was fine. Renders for real, so it
# needs a working GL driver (CI supplies Mesa llvmpipe).
add_executable(cvcgl_shadow_caster_growth test/cvcgl_shadow_caster_growth.cpp)
target_link_libraries(cvcgl_shadow_caster_growth PRIVATE cvcGL)
add_test(NAME cvcgl_shadow_caster_growth COMMAND cvcgl_shadow_caster_growth)

# The rig used to advertise "2 shadow casters" while VTK baked seven: fill and
# wash were Spots with cones under 90, which LightCreatesShadow accepts. Pins the
# advertised count to the real one, and pins which roles may cast.
add_executable(cvcgl_stage_caster_truth test/cvcgl_stage_caster_truth.cpp)
target_link_libraries(cvcgl_stage_caster_truth PRIVATE cvcGL)
add_test(NAME cvcgl_stage_caster_truth COMMAND cvcgl_stage_caster_truth)

# ── examples (opt-in, OFF by default) ──
# Standalone C++ programs that drive cvcGL directly — SceneGraph + SceneRenderer
# (the onscreen "basic window") + CameraController (orbit / Quake-fly / cinematic
# track). What cvcGL-as-a-demo-toolkit is for: quick self-contained scenes that
# scale toward a real-time sim/game loop. Off by default; -DCVC_BUILD_EXAMPLES=ON.
option(CVC_BUILD_EXAMPLES "Build cvcGL example programs" OFF)
if(CVC_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
