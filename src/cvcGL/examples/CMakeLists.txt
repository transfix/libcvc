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

# ---- WebAssembly demo builds -------------------------------------------------
# The browser-capable examples. nav_finale is native-only (it needs assimp +
# imagemagick + a runtime bundle the wasm toolchain lacks). Add a new browser demo
# to _wasm_demos and it builds via the `wasm-demos` target and is auto-discovered
# into the gallery by wasm/build-pages.py — no host page to hand-write.
if(EMSCRIPTEN)
  set(_wasm_demos lsystem_forest nav_city_swarm nav_fog_ghost)
  foreach(_wt ${_wasm_demos})
    # Asyncify: each demo keeps its own while-loop and yields via emscripten_sleep.
    # Growable heap: the forest/sea/sky volumes and city scenes exceed the 16 MB default.
    # VTK's patched FindOpenGL injects the WebGL2/GLES3 flags; RenderingUI its js-library.
    target_link_options(${_wt} PRIVATE
      -sASYNCIFY=1
      -sASYNCIFY_STACK_SIZE=131072
      -sALLOW_MEMORY_GROWTH=1
      -sSTACK_SIZE=2097152
      -sEXIT_RUNTIME=0
    )
    if(CVC_WASM_PTHREADS)
      # Pre-spawn workers so std::thread creation doesn't stall on the busy main thread.
      target_link_options(${_wt} PRIVATE -sPTHREAD_POOL_SIZE=4)
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
set(_cvcgl_example_bins lsystem_forest nav_city_swarm nav_fog_ghost nav_finale)
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
