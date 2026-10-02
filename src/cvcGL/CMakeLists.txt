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
# Ariadne — the cvcGL UI & scene DSL (cvc::gl::ariadne). Sources live in a
# subdirectory, so the non-recursive top-level glob above does not pick them up.
file(GLOB CVCGL_ARIADNE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ariadne/*.cpp")
list(APPEND CVCGL_SOURCES ${CVCGL_ARIADNE_SOURCES})
# Phase-3 peripheral capture (SDL3 camera / audio-device sources). SDL-gated internally; with no SDL
# the sources compile to graceful no-ops (open_camera returns an empty result).
file(GLOB CVCGL_CAPTURE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/capture/*.cpp")
list(APPEND CVCGL_SOURCES ${CVCGL_CAPTURE_SOURCES})
# Phase-3 peripheral playback (SDL3 audio-output sink, cvc::gl::playback). Lives in a subdirectory,
# so the non-recursive top-level glob does not pick it up. SDL-gated internally; with no SDL the
# sink compiles to a graceful no-op (open_audio_output returns an empty result).
file(GLOB CVCGL_PLAYBACK_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/playback/*.cpp")
list(APPEND CVCGL_SOURCES ${CVCGL_PLAYBACK_SOURCES})
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
set(CVC_IMGUI_VIA_CONFIG OFF)  # set when linked via the imgui::imgui cmake target
set(CVC_IMGUI_VIA_FIND OFF)    # set when located by header/lib (config-less imgui bundle)
if(CVC_ENABLE_IMGUI)
  # Prefer the imgui cmake package (imgui::imgui imported target — relocatable,
  # carries its own include + lib). A consumer of the installed cvcGL then
  # re-finds it via find_dependency(imgui) in cvcGLConfig.cmake, so nothing has
  # to hand-wire -limgui or an -L path. Falls back to header/lib discovery for
  # an older imgui bundle that ships no cmake config.
  find_package(imgui CONFIG QUIET)
  set(_cvc_imgui_inc "")
  if(TARGET imgui::imgui)
    get_target_property(_cvc_imgui_inc imgui::imgui INTERFACE_INCLUDE_DIRECTORIES)
  endif()
  if(TARGET imgui::imgui AND _cvc_imgui_inc
     AND EXISTS "${_cvc_imgui_inc}/backends/imgui_impl_opengl3.cpp")
    target_sources(cvcGL PRIVATE "${_cvc_imgui_inc}/backends/imgui_impl_opengl3.cpp")
    target_link_libraries(cvcGL PUBLIC imgui::imgui)  # PUBLIC: consumers write ImGui::*
    target_compile_definitions(cvcGL PUBLIC CVC_ENABLE_IMGUI)
    if(EMSCRIPTEN) # WebGL2 loader for the browser build
      set_source_files_properties("${_cvc_imgui_inc}/backends/imgui_impl_opengl3.cpp"
                                  PROPERTIES COMPILE_DEFINITIONS IMGUI_IMPL_OPENGL_ES3)
    endif()
    set(CVC_IMGUI_VIA_CONFIG ON)
    message(STATUS "cvcGL: Dear ImGui overlay ENABLED (imgui::imgui target)")
  else()
    # Fallback: a config-less imgui bundle (the cvcpkg windows + wasm builds ship
    # no imguiConfig.cmake) — locate by header/lib. find_path/find_library return
    # absolute BUILDER paths, so they stay $<BUILD_INTERFACE>-only; the export
    # names the imgui::imgui target instead, which cvcGLConfig.cmake re-creates
    # from the CONSUMER's own prefix (see cvcGLConfig.cmake.in). It used to export
    # $<INSTALL_INTERFACE:include/imgui> + a bare imgui: that assumed imgui was
    # co-installed in cvcGL's prefix (a split prefix failed configure with
    # "non-existent path .../include/imgui"), and a bare imgui has no -L/LIBPATH.
    find_path(CVC_IMGUI_INCLUDE_DIR imgui.h PATH_SUFFIXES imgui)
    find_library(CVC_IMGUI_LIBRARY NAMES imgui)
    if(CVC_IMGUI_INCLUDE_DIR AND CVC_IMGUI_LIBRARY
       AND EXISTS "${CVC_IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp")
      target_sources(cvcGL PRIVATE "${CVC_IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp")
      target_include_directories(cvcGL PUBLIC
        "$<BUILD_INTERFACE:${CVC_IMGUI_INCLUDE_DIR}>")
      target_link_libraries(cvcGL PUBLIC
        "$<BUILD_INTERFACE:${CVC_IMGUI_LIBRARY}>"
        "$<INSTALL_INTERFACE:imgui::imgui>")
      set(CVC_IMGUI_VIA_FIND ON)
      target_compile_definitions(cvcGL PUBLIC CVC_ENABLE_IMGUI)
      if(EMSCRIPTEN) # WebGL2 loader for the browser build
        set_source_files_properties("${CVC_IMGUI_INCLUDE_DIR}/backends/imgui_impl_opengl3.cpp"
                                    PROPERTIES COMPILE_DEFINITIONS IMGUI_IMPL_OPENGL_ES3)
      endif()
      message(STATUS "cvcGL: Dear ImGui overlay ENABLED (${CVC_IMGUI_LIBRARY}, no cmake config)")
    else()
      set(CVC_ENABLE_IMGUI OFF)
      message(STATUS "cvcGL: imgui not found -- ImGuiOverlay compiles to no-ops")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# SDL3 (optional): the input + peripheral seam — keyboard/mouse/gamepad and later
# audio/microphone/camera — native AND WebAssembly (wasm/wasm-mt, verified with VTK).
# Found via its own CMake config (SDL3::SDL3 imported target, relocatable — a consumer
# of the installed cvcGL re-finds it via find_dependency(SDL3) in cvcGLConfig.cmake, so
# nothing hand-wires -lSDL3 or an -L path). When present, CVC_ENABLE_SDL is defined so
# cvc::gl code (the §4.6 input backend / on_key etc.) can use SDL. GRACEFULLY ABSENT: with
# no SDL3 in the prefix, CVC_ENABLE_SDL is off and the SDL-backed paths compile to no-ops
# (input falls back to the VTK interactor, as today). The sdl3 cvcpkg dependency (declared
# in the cvcgl recipe) is what puts SDL3 in the build + consumer prefix.
option(CVC_ENABLE_SDL "Build the cvcGL SDL3 input/peripheral seam (keyboard/mouse/gamepad/audio)" ON)
set(CVC_SDL_VIA_CONFIG OFF)  # set when linked via the SDL3::SDL3 cmake target
if(CVC_ENABLE_SDL)
  find_package(SDL3 CONFIG QUIET)
  if(TARGET SDL3::SDL3)
    target_link_libraries(cvcGL PUBLIC SDL3::SDL3)
    target_compile_definitions(cvcGL PUBLIC CVC_ENABLE_SDL)
    set(CVC_SDL_VIA_CONFIG ON)
    message(STATUS "cvcGL: SDL3 input/peripheral seam ENABLED (SDL3::SDL3)")
  else()
    set(CVC_ENABLE_SDL OFF)
    message(STATUS "cvcGL: SDL3 not found -- input seam falls back to the VTK interactor")
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
# XFlush), so link the X11::X11 imported TARGET (libX11 alone): the export carries the
# relocatable name $<LINK_ONLY:X11::X11>, and cvcGLConfig.cmake's find_dependency(X11)
# re-resolves it to the CONSUMER's own libX11 (the same FindX11 pass VTK's config
# already runs). A bare "-lX11" (the previous fix) only resolved where libX11.so sat
# on the default linker path — e.g. the GitHub images' system libx11-dev — and failed
# in a clean container ("cannot find -lX11") even with the hermetic libX11 present.
set(CVCGL_X11_DEP "")
if(UNIX AND NOT APPLE)
  find_package(X11)
  if(X11_FOUND)
    target_link_libraries(cvcGL PRIVATE X11::X11)
    target_include_directories(cvcGL PRIVATE ${X11_INCLUDE_DIR})
    get_target_property(_cvcgl_type cvcGL TYPE)
    if(_cvcgl_type STREQUAL "STATIC_LIBRARY") # a shared cvcGL resolves libX11 itself
      set(CVCGL_X11_DEP "find_dependency(X11)")
    endif()
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

# @CVCGL_IMGUI_DEP@ — a find_dependency(imgui) line iff cvcGL linked the
# imgui::imgui target this build (so consumers re-find it); empty otherwise.
if(CVC_IMGUI_VIA_CONFIG)
  set(CVCGL_IMGUI_DEP "find_dependency(imgui CONFIG)")
else()
  set(CVCGL_IMGUI_DEP "")
endif()

# @CVCGL_SDL_DEP@ — a find_dependency(SDL3) line iff cvcGL linked SDL3::SDL3 this build
# (so a consumer re-finds it); empty otherwise.
if(CVC_SDL_VIA_CONFIG)
  set(CVCGL_SDL_DEP "find_dependency(SDL3 CONFIG)")
else()
  set(CVCGL_SDL_DEP "")
endif()

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

# LodGraphicsNode + SceneGraph::selectLOD (the cvc::lod -> scene-graph seam):
# distance selection, user visibility vs the drawn rung (a hidden parent stays
# hidden; re-showing it shows ONE rung), rungs kept registered across switches,
# setPyramid/appendRungs re-population and the rung style, make_view_params from
# a real vtkCamera (vertical ViewAngle, ParallelScale), setLODEnabled + lod_stats
# and a feature-on synthetic tiled city. All headless (bare vtkRenderer) except a
# final offscreen section -- rung colours, a pixel-identical zero-error switch and
# the shadow-bake hook -- which skips itself when the build cannot rasterise.
add_executable(cvcgl_lod_node test/cvcgl_lod_node.cpp)
target_link_libraries(cvcgl_lod_node PRIVATE cvcGL)
add_test(NAME cvcgl_lod_node COMMAND cvcgl_lod_node)

# GraphicsNode::setMetadata's read-only state mirror: it follows the node (it
# used to keep the first value forever, each later write throwing a swallowed
# read_only_error), stays locked to other writers, costs nothing for an
# unchanged value, mirrors Python ints, and asks for no redraw. Headless.
add_executable(cvcgl_metadata_mirror test/cvcgl_metadata_mirror.cpp)
target_link_libraries(cvcgl_metadata_mirror PRIVATE cvcGL)
add_test(NAME cvcgl_metadata_mirror COMMAND cvcgl_metadata_mirror)

# Spectral GPU FFT ocean (the ABYSSAL port used by lsystem_coast). Checks the
# invariants a broken butterfly IFFT / bit-reversal / conjugate-symmetry would
# violate: zero-mean height, unit-mean Jacobian, determinism, time evolution and
# live wind-knob response. Compiles OceanFFT.cpp (it lives in examples/, has no
# cvc dependency) and runs against an offscreen GL context. SKIPs (rc 0) if the
# context has no float render targets.
add_executable(cvcgl_ocean_fft test/cvcgl_ocean_fft.cpp examples/OceanFFT.cpp)
target_include_directories(cvcgl_ocean_fft PRIVATE examples)
target_link_libraries(cvcgl_ocean_fft PRIVATE cvcGL)
add_test(NAME cvcgl_ocean_fft COMMAND cvcgl_ocean_fft)

# GeometryNode::updateNormals — the in-place normal fast path the FFT ocean uses
# to shade its CPU-deformed surface. Renders a lit quad re-normal'd toward vs away
# from the light and checks the brightness responds, plus the no-op-on-mismatch guard.
add_executable(cvcgl_update_normals test/cvcgl_update_normals.cpp)
target_link_libraries(cvcgl_update_normals PRIVATE cvcGL)
add_test(NAME cvcgl_update_normals COMMAND cvcgl_update_normals)

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

# The world grid must enclose the geometry actually in the scene. It was sized
# only from GraphicsNode::transformChanged, so it tracked geometry that MOVED
# and ignored geometry that was merely ADDED — every static scene drew a
# degenerate grid at the origin while its bbox enclosed the real extent.
add_executable(cvcgl_grid_bounds test/cvcgl_grid_bounds.cpp)
target_link_libraries(cvcgl_grid_bounds PRIVATE cvcGL)
add_test(NAME cvcgl_grid_bounds COMMAND cvcgl_grid_bounds)

# The world-bounds walk (computeGraphicsBounds) through cached world matrices and
# plain flags instead of a matrix copy and two dynamic_casts per child: the same
# bounds bit for bit over random hierarchies (against the old walk, replicated),
# no grow-only walk while the grid and axis are hidden, and a catch-up when one
# is shown. Headless.
add_executable(cvcgl_bounds_walk test/cvcgl_bounds_walk.cpp)
target_link_libraries(cvcgl_bounds_walk PRIVATE cvcGL)
add_test(NAME cvcgl_bounds_walk COMMAND cvcgl_bounds_walk)

# Scene-wide diagnostic chrome: the setters reach every node (not just the root),
# and the getters are derived from the nodes so a checkbox can show real state.
add_executable(cvcgl_scene_chrome test/cvcgl_scene_chrome.cpp)
target_link_libraries(cvcgl_scene_chrome PRIVATE cvcGL)
add_test(NAME cvcgl_scene_chrome COMMAND cvcgl_scene_chrome)

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

# A LightNode must apply state only on its scene's owner thread -- the handler is
# reachable from the publisher's flush thread, where it used to race applyLights().
add_executable(cvcgl_light_state_thread test/cvcgl_light_state_thread.cpp)
target_link_libraries(cvcgl_light_state_thread PRIVATE cvcGL)
add_test(NAME cvcgl_light_state_thread COMMAND cvcgl_light_state_thread)

# A node's own pose, published and flushed back from off the owner thread, must
# not rewind it. Guards the echo check in GraphicsNode::handleStateChanged.
add_executable(cvcgl_pose_echo test/cvcgl_pose_echo.cpp)
target_link_libraries(cvcgl_pose_echo PRIVATE cvcGL)
add_test(NAME cvcgl_pose_echo COMMAND cvcgl_pose_echo)

# GraphicsNode::setPoseMatrix, the per-frame pose path for many moving nodes:
# exact actor matrix, 100 poses between two pumps = ONE scene-bounds walk, an
# off-thread call marshalled to the owner thread (latest-wins), the pose reaching
# state without a second cascade, and pixel parity with setTransform (the render
# part SKIPs when the build cannot rasterise).
add_executable(cvcgl_pose_matrix test/cvcgl_pose_matrix.cpp)
target_link_libraries(cvcgl_pose_matrix PRIVATE cvcGL)
add_test(NAME cvcgl_pose_matrix COMMAND cvcgl_pose_matrix)

# A node can outlive the scene that made it (removeGraphics hands it back; a
# Python proxy or a viewer keeps holding it). Guards the raw back-pointers that
# used to dangle in that window -- SceneGraph* (a pose write locked the freed
# publisher's mutex) and the parent GraphicsNode*.
add_executable(cvcgl_node_outlives_scene test/cvcgl_node_outlives_scene.cpp)
target_link_libraries(cvcgl_node_outlives_scene PRIVATE cvcGL)
add_test(NAME cvcgl_node_outlives_scene COMMAND cvcgl_node_outlives_scene)

# Phase-4 texture support: geometry UVs -> SetTCoords + setTexture(cvc::image).
add_executable(cvcgl_texture test/cvcgl_texture.cpp)
target_link_libraries(cvcgl_texture PRIVATE cvcGL)
add_test(NAME cvcgl_texture COMMAND cvcgl_texture)

# Phase-5 zero-copy texture: setTexture aliases the cvc::image buffer (no copy),
# in-place pixel edits are live, texture_modified() bumps the MTime.
add_executable(cvcgl_texture_zerocopy test/cvcgl_texture_zerocopy.cpp)
target_link_libraries(cvcgl_texture_zerocopy PRIVATE cvcGL)
add_test(NAME cvcgl_texture_zerocopy COMMAND cvcgl_texture_zerocopy)

# Phase-2 stream sink: StreamTextureBinding aliases a cvc::ariadne::stream video frame into
# a GeometryNode's texture zero-copy on each tick (end-to-end, headless).
add_executable(cvcgl_stream_texture test/cvcgl_stream_texture.cpp)
target_link_libraries(cvcgl_stream_texture PRIVATE cvcGL)
add_test(NAME cvcgl_stream_texture COMMAND cvcgl_stream_texture)

# Phase-2 PR7: the "stream" scene-node realizer (register_scene_node_type) builds a UV'd
# quad, resolves+subscribes the named stream, and wires a per-frame tick into the scene.
add_executable(cvcgl_stream_node test/cvcgl_stream_node.cpp)
target_link_libraries(cvcgl_stream_node PRIVATE cvcGL)
add_test(NAME cvcgl_stream_node COMMAND cvcgl_stream_node)

# Phase-2 PR8: the imperative (gl-bind-stream NODE-ID TOKEN) verb seam — registration, the
# per-document ictx.document routing, arg parsing and error paths (via a mock StreamBindingSink,
# no GL). The live GL binding is covered by cvcgl_stream_texture / cvcgl_stream_node.
add_executable(cvcgl_stream_bind test/cvcgl_stream_bind.cpp)
target_link_libraries(cvcgl_stream_bind PRIVATE cvcGL)
add_test(NAME cvcgl_stream_bind COMMAND cvcgl_stream_bind)

# Phase-3 video capture: cvc::gl::capture::camera_source (SDL3 camera -> stream frame_source).
# DEVICE-GATED — skips (passes) with no camera; the transport side is covered hardware-free by
# stream_test's StreamProducer.* (synthetic_source).
add_executable(cvcgl_camera_source test/cvcgl_camera_source.cpp)
target_link_libraries(cvcgl_camera_source PRIVATE cvcGL)
add_test(NAME cvcgl_camera_source COMMAND cvcgl_camera_source)

# Phase-3 audio capture: cvc::gl::capture::audio_source (SDL3 recording device -> stream
# frame_source). DEVICE-GATED — skips (passes) with no microphone / without CVC_TEST_AUDIO; the
# transport side and the pure audio sources (tone/gain/mix) are covered hardware-free by stream_test.
add_executable(cvcgl_audio_source test/cvcgl_audio_source.cpp)
target_link_libraries(cvcgl_audio_source PRIVATE cvcGL)
add_test(NAME cvcgl_audio_source COMMAND cvcgl_audio_source)

# Phase-3 audio playback: cvc::gl::playback::audio_output (f32 PCM -> SDL3 playback device).
# DEVICE-GATED — enumeration always; live playback behind CVC_TEST_AUDIO, skips (passes) with no
# output device / no SDL.
add_executable(cvcgl_audio_output test/cvcgl_audio_output.cpp)
target_link_libraries(cvcgl_audio_output PRIVATE cvcGL)
add_test(NAME cvcgl_audio_output COMMAND cvcgl_audio_output)

# Phase-3 Ariadne stream WIDGET: cvc::gl::ariadne stream_view — its render-thread stream->image
# pull step (next_stream_frame_image), exercised hardware-free with synthetic_source (no GL/ImGui).
add_executable(cvcgl_stream_view test/cvcgl_stream_view.cpp)
target_link_libraries(cvcgl_stream_view PRIVATE cvcGL)
add_test(NAME cvcgl_stream_view COMMAND cvcgl_stream_view)

# texture_modified_rows / _rect: a zero-copy texture patched on the GPU with
# glTexSubImage2D and nothing marked modified -- no whole-texture re-upload, no
# shadow re-bake. Reads the texture back (glGetTexImage), so desktop GL only;
# skips where nothing rasterises unless CVC_REQUIRE_RENDER=1.
if(NOT EMSCRIPTEN)
  add_executable(cvcgl_texture_rows test/cvcgl_texture_rows.cpp)
  target_link_libraries(cvcgl_texture_rows PRIVATE cvcGL)
  add_test(NAME cvcgl_texture_rows COMMAND cvcgl_texture_rows)
endif()

# SceneRenderer: a render target held open across frames (one GL context for a
# whole sequence, a camera that survives the next render, live scene updates).
add_executable(cvcgl_renderer test/cvcgl_renderer.cpp)
target_link_libraries(cvcgl_renderer PRIVATE cvcGL)
add_test(NAME cvcgl_renderer COMMAND cvcgl_renderer)

# SceneRenderer is now a facade over ViewportManager: pin its state-footprint
# parity (no viewer camera/layout state) and the HostStyle host-input contract.
add_executable(cvcgl_scene_renderer_facade test/cvcgl_scene_renderer_facade.cpp)
target_link_libraries(cvcgl_scene_renderer_facade PRIVATE cvcGL)
add_test(NAME cvcgl_scene_renderer_facade COMMAND cvcgl_scene_renderer_facade)

add_executable(cvcgl_volren_node test/cvcgl_volren_node.cpp)
target_link_libraries(cvcgl_volren_node PRIVATE cvcGL)
add_test(NAME cvcgl_volren_node COMMAND cvcgl_volren_node)

add_executable(cvcgl_volslice_node test/cvcgl_volslice_node.cpp)
target_link_libraries(cvcgl_volslice_node PRIVATE cvcGL)
add_test(NAME cvcgl_volslice_node COMMAND cvcgl_volslice_node)

# The publisher keeps the state tree correct while keeping writes off the render
# path: the value arrives (after a flush), the last write wins, and an external
# write still drives the node.
add_executable(cvcgl_state_publisher test/cvcgl_state_publisher.cpp)
target_link_libraries(cvcgl_state_publisher PRIVATE cvcGL)
add_test(NAME cvcgl_state_publisher COMMAND cvcgl_state_publisher)

add_executable(cvcgl_nav_stats_publish test/cvcgl_nav_stats_publish.cpp)
target_link_libraries(cvcgl_nav_stats_publish PRIVATE cvcGL)
add_test(NAME cvcgl_nav_stats_publish COMMAND cvcgl_nav_stats_publish)

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

# CameraController::update() on a still camera over a still scene: no clipping-
# range reset and no pose mirror, yet the range is re-fitted the moment the scene
# could have grown (node added/moved/re-meshed, prop shown) or the camera moved.
# Headless, plus a rendered section (shadows on) that skips without GL.
add_executable(cvcgl_camera_idle test/cvcgl_camera_idle.cpp)
target_link_libraries(cvcgl_camera_idle PRIVATE cvcGL)
add_test(NAME cvcgl_camera_idle COMMAND cvcgl_camera_idle)

# Numeric parity: the push-fed Track controller reproduces pymod_gl/camera.py's
# ChaseCamera (the two-stage EMA follow) to floating point, so the C++ and Python
# chase cameras genuinely "work the same". Headless (getPose reads trackEye/Focal
# directly); no renderer needed.
add_executable(cvcgl_track_parity test/cvcgl_track_parity.cpp)
target_link_libraries(cvcgl_track_parity PRIVATE cvcGL)
add_test(NAME cvcgl_track_parity COMMAND cvcgl_track_parity)

# World-units integration: GraphicsNode::localToWorld/worldToLocal (lazily
# cached inverse) and localPointToReal mapping a graphic's own-frame point to a
# real-world km/mile coordinate via cvc::world_units, plus SceneRenderer::
# pickWorld's miss contract (SKIPs without a GL context). The matrix math runs
# headless; no renderer needed.
add_executable(cvcgl_world_units test/cvcgl_world_units.cpp)
target_link_libraries(cvcgl_world_units PRIVATE cvcGL)
add_test(NAME cvcgl_world_units COMMAND cvcgl_world_units)

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

# Ariadne §9 realize_scene: lights (batch position bake + kind routing), a rig, and
# nested-node LOCAL transforms — the realize invariants the core loader gtests can't
# reach (no VTK). Inspects the realized vtkLights + node world transforms offscreen.
add_executable(cvcgl_ariadne_realize test/cvcgl_ariadne_realize.cpp)
target_link_libraries(cvcgl_ariadne_realize PRIVATE cvcGL)
add_test(NAME cvcgl_ariadne_realize COMMAND cvcgl_ariadne_realize)

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

# GraphicsNode::setCastsShadow: a non-caster is left out of the shadow maps AND
# out of the re-bake decision -- moving or restyling one never bakes, while a
# caster, a light, or a node joining/leaving the casters still does. Checks the
# picture too (a non-caster throws no shadow). Renders; skips without GL.
add_executable(cvcgl_shadow_casters test/cvcgl_shadow_casters.cpp)
target_link_libraries(cvcgl_shadow_casters PRIVATE cvcGL)
add_test(NAME cvcgl_shadow_casters COMMAND cvcgl_shadow_casters)

# The rig used to advertise "2 shadow casters" while VTK baked seven: fill and
# wash were Spots with cones under 90, which LightCreatesShadow accepts. Pins the
# advertised count to the real one, and pins which roles may cast.
add_executable(cvcgl_stage_caster_truth test/cvcgl_stage_caster_truth.cpp)
target_link_libraries(cvcgl_stage_caster_truth PRIVATE cvcGL)
add_test(NAME cvcgl_stage_caster_truth COMMAND cvcgl_stage_caster_truth)

# Multi-field StageLighting setters (setStage/setKey/setWash/applyPreset) must keep
# EVERY field: the rig's own change callback re-reads all fields from state, so
# writing them one at a time with it live kept only the first. Headless.
add_executable(cvcgl_stage_setters test/cvcgl_stage_setters.cpp)
target_link_libraries(cvcgl_stage_setters PRIVATE cvcGL)
add_test(NAME cvcgl_stage_setters COMMAND cvcgl_stage_setters)

# ViewportManager composites N Viewports (each its own vtkRenderer + camera +
# scene) as SetViewport/SetLayer regions in ONE vtkRenderWindow (one GL context —
# mandatory under WASM). Pins the picture-in-picture path the SceneRenderer test
# documents as unsolvable there: a primary full-screen view + an inset over a
# SECOND scene both composite in one window (both rects draw non-empty), and a
# second viewport over an already-attached scene is a LOUD error, not a silently
# blanked first view. Renders for real, so it needs a working GL driver (CI
# supplies Mesa llvmpipe).
add_executable(cvcgl_viewport test/cvcgl_viewport.cpp)
target_link_libraries(cvcgl_viewport PRIVATE cvcGL)
add_test(NAME cvcgl_viewport COMMAND cvcgl_viewport)

# ViewportManager input router: routes one window's interactor events to the
# right viewport's CameraController (viewport-under-cursor for mouse/wheel, latch
# a drag to its origin viewport, keyboard to the active viewport). The routing
# DECISION and DISPATCH are public interactor-free methods, so this drives them
# with synthetic display coordinates offscreen — no live interactor — and reads
# each viewport's vtkCamera to confirm the effect. Renders for real, so it needs
# a working GL driver (CI supplies Mesa llvmpipe).
add_executable(cvcgl_viewport_input test/cvcgl_viewport_input.cpp)
target_link_libraries(cvcgl_viewport_input PRIVATE cvcGL)
add_test(NAME cvcgl_viewport_input COMMAND cvcgl_viewport_input)

# Streaming geometry (StreamingGeometryNode / RibbonNode / DrapedLinkNode over the
# two streaming VTK mappers). Uploads are counted twice: by the node's own
# streamStats() and at the driver, by test/gl_upload_counter.cpp, which swaps
# VTK's glad function pointers for counting wrappers. Both mapper families run
# natively (the low-memory one forced), and the GLES texture-emulation upload is
# checked against a real 2-D texture. Renders for real; skips where nothing
# rasterises unless CVC_REQUIRE_RENDER=1.
#   cvcgl_streaming_geometry: coalesced events, mapper choice, the GLES row-span
#     upload, height-field sampling, partial uploads / draw ranges / uniforms,
#     capacity overflow, off-thread writes, ribbon growth.
#   cvcgl_streaming_overlays: track append, spine replan + visible range, draped
#     link, pixel parity against the equivalent wholesale GeometryNode, and all of
#     it under vtkShadowMapPass (one upload per frame, not per pass).
#   cvcgl_streaming_races: interleavings forced at chosen lock acquisitions (an
#     interposed pthread_mutex_lock, so Linux only): a producer posting while the
#     owner destroys the scene, and the draped link's bounds derive/commit races.
# Desktop GL only: gl_upload_counter.cpp swaps glad function pointers and the
# texture read-back uses glGetTexImage, neither of which exists on GLES/WebGL2
# (the wasm VTK includes <GLES3/gl3.h>, not glad).
if(NOT EMSCRIPTEN)
  add_executable(cvcgl_streaming_geometry test/cvcgl_streaming_geometry.cpp
                                          test/gl_upload_counter.cpp)
  target_link_libraries(cvcgl_streaming_geometry PRIVATE cvcGL)
  add_test(NAME cvcgl_streaming_geometry COMMAND cvcgl_streaming_geometry)
  add_executable(cvcgl_streaming_overlays test/cvcgl_streaming_overlays.cpp
                                          test/gl_upload_counter.cpp)
  target_link_libraries(cvcgl_streaming_overlays PRIVATE cvcGL)
  add_test(NAME cvcgl_streaming_overlays COMMAND cvcgl_streaming_overlays)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    add_executable(cvcgl_streaming_races test/cvcgl_streaming_races.cpp)
    target_link_libraries(cvcgl_streaming_races PRIVATE cvcGL ${CMAKE_DL_LIBS})
    add_test(NAME cvcgl_streaming_races COMMAND cvcgl_streaming_races)
  endif()
endif()

# SdlInput: SDL3 event -> normalized cvc::gl::InputEvent translation (the §4.6 input seam's SDL
# source). Pushes synthetic SDL key/mouse events onto the queue and verifies poll() returns them.
# Headless (the SDL EVENTS subsystem needs no window). SKIPs (rc 0) in a build without SDL.
add_executable(cvcgl_sdl_input test/cvcgl_sdl_input.cpp)
target_link_libraries(cvcgl_sdl_input PRIVATE cvcGL)
add_test(NAME cvcgl_sdl_input COMMAND cvcgl_sdl_input)

# ── GL tests: one at a time ──
# Every cvcGL test that renders through a GL context takes the CTest resource
# lock cvcgl_gl_context, so ctest never runs two of them at once; the headless
# tests listed below keep running in parallel. Why: on the macOS runners every
# failed attempt of cvcgl_shadow_casters (4 of 12 seen) ran alongside
# cvcgl_shadow_caster_growth and other GL tests under ctest --parallel, no
# passing attempt overlapped growth, and each retry passed. On every platform,
# on purpose: it costs only wall time (the GL tests run end to end instead of
# overlapped) and takes GL contention out of every run. A test added above is
# locked unless it is listed here as headless.
get_property(_cvcgl_gl_tests DIRECTORY PROPERTY TESTS)
list(REMOVE_ITEM _cvcgl_gl_tests
  cvcgl_smoke cvcgl_metadata_mirror cvcgl_volume_range cvcgl_traversal
  cvcgl_grid_bounds cvcgl_bounds_walk cvcgl_scene_chrome cvcgl_remove
  cvcgl_teardown cvcgl_light_state_thread cvcgl_pose_echo
  cvcgl_node_outlives_scene cvcgl_texture cvcgl_texture_zerocopy
  cvcgl_stream_texture cvcgl_state_publisher cvcgl_nav_stats_publish
  cvcgl_camera cvcgl_track_parity cvcgl_fps_hud cvcgl_stage_setters
  cvcgl_streaming_races cvcgl_sdl_input)
set_tests_properties(${_cvcgl_gl_tests} PROPERTIES RESOURCE_LOCK cvcgl_gl_context)

# ── examples (opt-in, OFF by default) ──
# Standalone C++ programs that drive cvcGL directly — SceneGraph + SceneRenderer
# (the onscreen "basic window") + CameraController (orbit / Quake-fly / cinematic
# track). What cvcGL-as-a-demo-toolkit is for: quick self-contained scenes that
# scale toward a real-time sim/game loop. Off by default; -DCVC_BUILD_EXAMPLES=ON.
option(CVC_BUILD_EXAMPLES "Build cvcGL example programs" OFF)
if(CVC_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()
