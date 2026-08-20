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

# Unit tests for nav_common's bounds-safe compositing (the minimap-overflow crash
# guard) + the occupancy rasterizer. Only when GoogleTest is available (CVC_BUILD_TESTS
# pulled it in) — the test does no rendering, so it needs no GL context to run.
if(TARGET GTest::gtest)
  add_executable(nav_common_test nav_common_test.cpp)
  target_link_libraries(nav_common_test PRIVATE nav_demo_common GTest::gtest GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(nav_common_test)
endif()
