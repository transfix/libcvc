# The VolumeRover ports' tests (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.7, §1.8): googletest
# binaries that run natively, with the rest of the suite, and as wasm-mt binaries under node in
# ci.yml's `ports (wasm-mt, node)` lane. Included from tests/CMakeLists.txt (CVC_BUILD_TESTS) and,
# for the wasm lane, from ../CMakeLists.txt with CVC_BUILD_PORT_TESTS and the test tree off, so a
# wasm build compiles only these.
#
# A port adds its test here: its name to CVC_PORT_TESTS (the source is <name>.cpp beside this file),
# and any extra libraries with target_link_libraries after the include. Each test must pass under
# node: no files outside its own data, no threads beyond the app's pool, nothing the wasm-mt
# libcvc leaves out.

set(CVC_PORT_TESTS
  param_table_test
  result_job_test
  job_intrinsics_test
  tet_complex_test
  skeleton_test
  skeleton_intrinsics_test
  contour_stack_test
  point_classes_test
)

# One port test: cvc and googletest, and under Emscripten what node needs to run it (as
# physics_replay_test: the host's file system, main on a pthread so the app's workers can start,
# and the process exiting with main's status). -sFETCH: a test that reaches Ariadne links cvc::net's
# fetch backend (src/cvc/CMakeLists.txt: an executable adds it, with -pthread here).
function(cvc_setup_port_test target)
  target_link_libraries(${target} PRIVATE cvc GTest::gtest GTest::gtest_main)
  target_compile_features(${target} PRIVATE cxx_std_20)
  if(EMSCRIPTEN)
    target_link_options(${target} PRIVATE -sNODERAWFS=1 -sEXIT_RUNTIME=1 -sALLOW_MEMORY_GROWTH=1
      -sPROXY_TO_PTHREAD=1 -sSTACK_SIZE=4mb -sFETCH=1)
  endif()
endfunction()

foreach(_cvc_port_test IN LISTS CVC_PORT_TESTS)
  add_executable(${_cvc_port_test} ${CMAKE_CURRENT_LIST_DIR}/${_cvc_port_test}.cpp)
  cvc_setup_port_test(${_cvc_port_test})
endforeach()

# tet_complex_test compares the incremental build with Qhull's batch build, and checks exact
# Delaunay with Shewchuk's predicates: both come with cvc_permissive_geometry (the test skips
# itself without CVC_WITH_SHEWCHUK).
target_link_libraries(tet_complex_test PRIVATE cvc_permissive_geometry)

# skeleton_test compares the port with the legacy library's goldens (tools/volrover-goldens): the
# committed inputs and goldens are read from the source tree.
target_compile_definitions(skeleton_test PRIVATE
  CVC_REPO_DIR="${CMAKE_CURRENT_LIST_DIR}/../../..")
