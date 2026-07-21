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
project(cvcGL LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT TARGET cvc::cvc)
  find_package(cvc CONFIG REQUIRED)
endif()

find_package(VTK REQUIRED COMPONENTS
  CommonCore CommonDataModel CommonExecutionModel CommonTransforms
  FiltersSources FiltersGeneral
  RenderingCore RenderingVolume RenderingVolumeOpenGL2 RenderingOpenGL2
  RenderingAnnotation RenderingFreeType
  InteractionStyle
)

file(GLOB CVCGL_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
# Header root is <repo>/inc  (headers under inc/cvc/gl/).
get_filename_component(_CVCGL_INC "${CMAKE_CURRENT_SOURCE_DIR}/../../inc" ABSOLUTE)

add_library(cvcGL STATIC ${CVCGL_SOURCES})
target_include_directories(cvcGL PUBLIC
  $<BUILD_INTERFACE:${_CVCGL_INC}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cvcGL PUBLIC cvc::cvc ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS cvcGL MODULES ${VTK_LIBRARIES})

set_target_properties(cvcGL PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS cvcGL EXPORT cvcGLTargets ARCHIVE DESTINATION lib)
install(DIRECTORY "${_CVCGL_INC}/cvc/gl" DESTINATION include/cvc
        FILES_MATCHING PATTERN "*.h")

# ── functional smoke test ──
enable_testing()
add_executable(cvcgl_smoke test/cvcgl_smoke.cpp)
target_link_libraries(cvcgl_smoke PRIVATE cvcGL)
add_test(NAME cvcgl_smoke COMMAND cvcgl_smoke)

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
