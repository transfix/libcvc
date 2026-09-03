# cvcGL examples — standalone C++ programs that drive the scene graph directly.
#
# Each links the in-tree cvcGL target plus the VTK modules cvcGL does NOT already
# pull for a consumer: RenderingUI (the onscreen interactor factory) and IOImage
# (SceneRenderer::writePNG's vtkPNGWriter). InteractionStyle/RenderingOpenGL2 come
# transitively from cvcGL.

find_package(VTK REQUIRED COMPONENTS RenderingUI IOImage)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(lsystem_forest lsystem_forest.cpp)
target_link_libraries(lsystem_forest PRIVATE cvcGL ${VTK_LIBRARIES} Boost::program_options)
vtk_module_autoinit(TARGETS lsystem_forest MODULES ${VTK_LIBRARIES})

# bunny_shadow — a Stanford-bunny shader/shadow-mapping test bench (one mesh, one
# ground plane, a StageLighting rig). Loads the bunny via cvc::read_geometry, so
# it needs only cvcGL (no nav helpers).
add_executable(bunny_shadow bunny_shadow.cpp)
target_link_libraries(bunny_shadow PRIVATE cvcGL ${VTK_LIBRARIES} Boost::program_options)
vtk_module_autoinit(TARGETS bunny_shadow MODULES ${VTK_LIBRARIES})

add_executable(volren_bunny volren_bunny.cpp)
target_link_libraries(volren_bunny PRIVATE cvcGL ${VTK_LIBRARIES} Boost::program_options)
vtk_module_autoinit(TARGETS volren_bunny MODULES ${VTK_LIBRARIES})

# lsystem_coast — island + GPU FFT-ocean trial. Hosts OceanFFT (the ABYSSAL port
# target, docs/roadmap/OCEAN-AND-VOLUMETRIC-TERRAIN-NOTES.md). A SEPARATE demo
# from lsystem_forest by design: the forest stays the untouched perf control.
# OceanFFT depends only on VTK's OpenGL2 module (transitive via cvcGL), so no
# extra find_package. Enrolled in _wasm_demos below (the default GPU no-readback
# path is WebGL2-viable; the CPU --cpu path is native-only).
add_executable(lsystem_coast lsystem_coast.cpp OceanFFT.cpp)
target_link_libraries(lsystem_coast PRIVATE cvcGL ${VTK_LIBRARIES} Boost::program_options)
vtk_module_autoinit(TARGETS lsystem_coast MODULES ${VTK_LIBRARIES})

# cvc::nav demos — the reactive swarm rendered through cvcGL. nav symbols come from
# cvc::cvc, which cvcGL PUBLIC-links. Shared helpers (occupancy walls, merged agent
# glyph mesh, capture camera) live in nav_common.
add_library(nav_demo_common STATIC nav_common.cpp)
target_link_libraries(nav_demo_common PUBLIC cvcGL)

foreach(_demo nav_city_swarm nav_fog_ghost nav_finale)
  add_executable(${_demo} ${_demo}.cpp)
  target_link_libraries(${_demo} PRIVATE nav_demo_common cvcGL ${VTK_LIBRARIES}
                        Boost::program_options)
  vtk_module_autoinit(TARGETS ${_demo} MODULES ${VTK_LIBRARIES})
endforeach()

# Headless ABI/consistency guard (no GL). The nav demos compile against the
# in-tree cvc/nav headers but link the SHARED libcvc from the deps prefix; if that
# libcvc is older than the headers, sim_world's by-value return from
# from_occupancy() corrupts the caller (sim_world embeds config+material_config by
# value) and the demos segfault at startup. nav_abi_smoke exercises exactly that
# path and asserts a sane read-back; the cvcgl-examples recipe runs it after the
# build so a stale libcvc fails loudly here instead of shipping a broken demo. It
# links only cvc::cvc (no cvcGL/VTK/GL), is native-only (it must run), and is not
# installed — it is a build-time check.
if(NOT EMSCRIPTEN)
  add_executable(nav_abi_smoke nav_abi_smoke.cpp)
  target_link_libraries(nav_abi_smoke PRIVATE cvc::cvc)
endif()

# ---- WebAssembly demo builds -------------------------------------------------
# The browser-capable examples. nav_finale is native-only (it needs assimp +
# imagemagick + a runtime bundle the wasm toolchain lacks). Add a new browser demo
# to _wasm_demos and it builds via the `wasm-demos` target and is auto-discovered
# into the gallery by wasm/build-pages.py — no host page to hand-write.
if(EMSCRIPTEN)
  set(_wasm_demos lsystem_forest lsystem_coast bunny_shadow volren_bunny nav_city_swarm nav_fog_ghost)
  # Optional runtime asset bundle for nav_city_swarm (Austin: terrain.json +
  # buildings.glb + satellite.png). When CVC_WASM_BUNDLE is set to a scene dir,
  # its contents are preloaded to /bundle and, if the sibling shared/Humvee.glb
  # exists (installed by the vehicle-humvee cvcpkg bundle), that dir is preloaded
  # to /shared so the demo's <bundle>/../../shared/Humvee.glb autopath resolves.
  # If CVC_WASM_BUNDLE is empty (the default), the demo falls back to the
  # synthetic city.
  set(_nav_swarm_bundle_flags "")
  if(CVC_WASM_BUNDLE AND EXISTS "${CVC_WASM_BUNDLE}")
    message(STATUS "cvcGL nav_city_swarm wasm: preloading bundle from ${CVC_WASM_BUNDLE}")
    list(APPEND _nav_swarm_bundle_flags "--preload-file=${CVC_WASM_BUNDLE}@/bundle")
    get_filename_component(_parent "${CVC_WASM_BUNDLE}" DIRECTORY)
    if(_parent AND EXISTS "${_parent}/shared/Humvee.glb")
      message(STATUS "cvcGL nav_city_swarm wasm: preloading shared vehicle from ${_parent}/shared")
      list(APPEND _nav_swarm_bundle_flags "--preload-file=${_parent}/shared@/shared")
    endif()
  endif()
  foreach(_wt ${_wasm_demos})
    # Asyncify: each demo keeps its own while-loop and yields via emscripten_sleep.
    # Growable heap: the forest/sea/sky volumes and city scenes exceed the 16 MB default.
    # VTK's patched FindOpenGL injects the WebGL2/GLES3 flags; RenderingUI its js-library.
    target_link_options(${_wt} PRIVATE
      -sASYNCIFY=1
      -sASYNCIFY_STACK_SIZE=131072
      -sALLOW_MEMORY_GROWTH=1
      # Austin's buildings mesh produces a ~140 MB vtkFloatArray for
      # colors/normals; the default 2 GB ceiling reachable via growth
      # sometimes lands short of it once VTK's other buffers pile on.
      # Bump the hard cap so wasm32 can grow into the full 4 GB address
      # space rather than throwing "Unable to allocate ..." mid-scene.
      -sMAXIMUM_MEMORY=4GB
      -sSTACK_SIZE=2097152
      -sEXIT_RUNTIME=0
    )
    if(CVC_WASM_PTHREADS)
      # Pre-spawn workers so std::thread creation doesn't stall on the busy main thread.
      target_link_options(${_wt} PRIVATE -sPTHREAD_POOL_SIZE=4)
    endif()
    if(_nav_swarm_bundle_flags AND "${_wt}" STREQUAL "nav_city_swarm")
      target_link_options(${_wt} PRIVATE ${_nav_swarm_bundle_flags})
    endif()
  endforeach()
  # One target builds them all; wasm/build-wasm-demo.sh drives it, then build-pages.py
  # turns the built bin/ into the servable gallery (per-demo page + index of cards).
  add_custom_target(wasm-demos DEPENDS ${_wasm_demos})
endif()

# Install rules so the cvcgl-examples recipe packages the example binaries with no
# hand-staging. $ORIGIN/../lib lets them find libcvcGL / libcvc / VTK / Boost in the
# same cvcpkg prefix with NO LD_LIBRARY_PATH; INSTALL_RPATH_USE_LINK_PATH adds the
# deps-prefix link dirs too, which cvcpkg's rewrite-install-paths relocates to
# $ORIGIN-relative. cvcgl-examples is its own install COMPONENT so the recipe can
# `cmake --install --component cvcgl-examples` and get only the bins (not the cvcGL SDK).
set(_cvcgl_example_bins lsystem_forest lsystem_coast bunny_shadow volren_bunny nav_city_swarm nav_fog_ghost nav_finale)
set_target_properties(${_cvcgl_example_bins} PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
  INSTALL_RPATH_USE_LINK_PATH ON)
install(TARGETS ${_cvcgl_example_bins}
        RUNTIME DESTINATION bin COMPONENT cvcgl-examples)

# Unit tests for nav_common's bounds-safe compositing (the minimap-overflow crash
# guard) + the occupancy rasterizer. Only when GoogleTest is available (CVC_BUILD_TESTS
# pulled it in) — the test does no rendering, so it needs no GL context to run.
if(TARGET GTest::gtest)
  add_executable(nav_common_test nav_common_test.cpp)
  target_link_libraries(nav_common_test PRIVATE nav_demo_common GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(nav_common_test)
endif()
