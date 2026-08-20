# cvcGL examples — standalone C++ programs that drive the scene graph directly.
#
# Dual-mode, exactly like src/cvcGL and src/cvc-cli:
#   In-tree    (add_subdirectory(examples) when -DCVC_BUILD_EXAMPLES=ON): the
#              cvcGL target already exists, so we just link cvc::cvcGL.
#   Standalone (this dir configured on its own — an examples-only build): no libcvc
#              tree is present, so we project() and find the installed cvcGL package
#              via find_package(cvcGL CONFIG) — which pulls cvc::cvc and the VTK
#              closure through find_dependency. Point CMAKE_PREFIX_PATH at the deps
#              prefix that holds the cvcGL SDK.
#
# Each links cvc::cvcGL plus the VTK modules cvcGL does NOT already pull for a
# consumer: RenderingUI (the onscreen interactor factory) and IOImage
# (SceneRenderer::writePNG's vtkPNGWriter). InteractionStyle/RenderingOpenGL2
# come transitively from cvcGL.
cmake_minimum_required(VERSION 3.16)

if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  project(cvcgl_examples LANGUAGES CXX)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  # cvc::cvcGL (+ cvc::cvc + the VTK closure, via find_dependency in cvcGLConfig).
  find_package(cvcGL CONFIG REQUIRED)
endif()

find_package(VTK REQUIRED COMPONENTS RenderingUI IOImage)
find_package(Boost REQUIRED COMPONENTS program_options)

add_executable(lsystem_forest lsystem_forest.cpp)
target_link_libraries(lsystem_forest PRIVATE cvc::cvcGL ${VTK_LIBRARIES} Boost::program_options)
vtk_module_autoinit(TARGETS lsystem_forest MODULES ${VTK_LIBRARIES})

# cvc::nav demos — the reactive swarm rendered through cvcGL. nav symbols come from
# cvc::cvc, which cvcGL PUBLIC-links. Shared helpers (occupancy walls, merged agent
# glyph mesh, capture camera) live in nav_common.
add_library(nav_demo_common STATIC nav_common.cpp)
target_link_libraries(nav_demo_common PUBLIC cvc::cvcGL)

foreach(_demo nav_city_swarm nav_fog_ghost nav_finale)
  add_executable(${_demo} ${_demo}.cpp)
  target_link_libraries(${_demo} PRIVATE nav_demo_common cvc::cvcGL ${VTK_LIBRARIES}
                        Boost::program_options)
  vtk_module_autoinit(TARGETS ${_demo} MODULES ${VTK_LIBRARIES})
endforeach()

# Install rules so a cvcpkg recipe can package the example binaries with no
# hand-staging (the single cvcgl-examples recipe lives on the wasm PR #204 and picks
# these up when both land). $ORIGIN/../lib lets the binaries find libcvcGL / libcvc /
# VTK / Boost in the same cvcpkg prefix with NO LD_LIBRARY_PATH;
# INSTALL_RPATH_USE_LINK_PATH adds the deps-prefix link dirs too, which cvcpkg's
# rewrite-install-paths then relocates to $ORIGIN-relative.
set(_cvcgl_example_bins lsystem_forest nav_city_swarm nav_fog_ghost nav_finale)
set_target_properties(${_cvcgl_example_bins} PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
  INSTALL_RPATH_USE_LINK_PATH ON)
install(TARGETS ${_cvcgl_example_bins}
        RUNTIME DESTINATION bin COMPONENT cvcgl-examples)

# Unit tests for nav_common's bounds-safe compositing (the minimap-overflow crash
# guard) + the occupancy rasterizer. Only when GoogleTest is available (CVC_BUILD_TESTS
# pulled it in) — the test does no rendering, so it needs no GL context to run. The
# standalone recipe build has no GTest, so this is skipped there.
if(TARGET GTest::gtest)
  add_executable(nav_common_test nav_common_test.cpp)
  target_link_libraries(nav_common_test PRIVATE nav_demo_common GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(nav_common_test)
endif()
