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

if(EMSCRIPTEN)
  # Browser build: the demo keeps its own while-loop and yields to the event
  # loop via emscripten_sleep, which needs Asyncify. VTK's patched FindOpenGL
  # already injects the WebGL2/GLES3 link flags; RenderingUI injects its
  # --js-library glue. Memory must be growable — the forest + sea/sky volumes
  # comfortably exceed the 16 MB default heap.
  target_link_options(lsystem_forest PRIVATE
    -sASYNCIFY=1
    -sASYNCIFY_STACK_SIZE=131072
    -sALLOW_MEMORY_GROWTH=1
    -sSTACK_SIZE=2097152
    -sEXIT_RUNTIME=0
  )
  if(CVC_WASM_PTHREADS)
    # Pre-spawn workers so std::thread creation (the scene's state_publisher)
    # doesn't stall waiting for a worker to boot on the busy main thread.
    target_link_options(lsystem_forest PRIVATE -sPTHREAD_POOL_SIZE=4)
  endif()
  # Keep the output dir directly servable: python3 -m http.server -d <here>.
  add_custom_command(TARGET lsystem_forest POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${CMAKE_CURRENT_SOURCE_DIR}/wasm/index.html
      $<TARGET_FILE_DIR:lsystem_forest>/index.html)
endif()

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

if(EMSCRIPTEN)
  # The procedural nav demos are wasm-capable too (nav_finale is native-only — it
  # needs assimp + imagemagick + a runtime bundle, none of which the wasm toolchain
  # has). Same Asyncify + growable-heap link options as lsystem_forest; each keeps
  # its own while-loop and yields via emscripten_sleep.
  foreach(_wt nav_city_swarm nav_fog_ghost)
    target_link_options(${_wt} PRIVATE
      -sASYNCIFY=1
      -sASYNCIFY_STACK_SIZE=131072
      -sALLOW_MEMORY_GROWTH=1
      -sSTACK_SIZE=2097152
      -sEXIT_RUNTIME=0
    )
    if(CVC_WASM_PTHREADS)
      target_link_options(${_wt} PRIVATE -sPTHREAD_POOL_SIZE=4)
    endif()
    add_custom_command(TARGET ${_wt} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/wasm/${_wt}.html
        $<TARGET_FILE_DIR:${_wt}>/${_wt}.html)
  endforeach()
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
