# Unit tests for libcvc library
cmake_minimum_required(VERSION 3.15)

# Test executables
add_executable(app_test app_test.cpp)
add_executable(state_test state_test.cpp)
add_executable(voxels_test voxels_test.cpp)
add_executable(volume_test volume_test.cpp)
add_executable(geometry_test geometry_test.cpp)
add_executable(procedural_geometry_test procedural_geometry_test.cpp)

# Track all test targets so we can conditionally add HDF5 tests
set(TEST_TARGETS
  app_test
  state_test
  voxels_test
  volume_test
  geometry_test
  procedural_geometry_test
)

# Link against the CVC library and Google Test
target_link_libraries(app_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(voxels_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(volume_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(geometry_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(procedural_geometry_test
  PRIVATE
    cvc
)

# Set C++ standard to match the library
target_compile_features(app_test PRIVATE cxx_std_14)
target_compile_features(state_test PRIVATE cxx_std_14)
target_compile_features(voxels_test PRIVATE cxx_std_14)
target_compile_features(volume_test PRIVATE cxx_std_14)
target_compile_features(geometry_test PRIVATE cxx_std_14)
target_compile_features(procedural_geometry_test PRIVATE cxx_std_14)

# Only build HDF5-dependent tests when HDF5 support is enabled
if(CVC_USING_HDF5 AND NOT CVC_HDF5_DISABLED)
  find_package(HDF5 REQUIRED COMPONENTS C CXX)
  add_executable(hdf5_test hdf5_test.cpp)
  target_link_libraries(hdf5_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
      ${HDF5_LIBRARIES}
  )
  target_include_directories(hdf5_test PRIVATE ${HDF5_INCLUDE_DIRS})
  target_compile_features(hdf5_test PRIVATE cxx_std_14)
  list(APPEND TEST_TARGETS hdf5_test)
else()
  message(STATUS "HDF5 disabled; skipping hdf5_test")
endif()

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(app_test)
gtest_discover_tests(state_test)
gtest_discover_tests(voxels_test)
gtest_discover_tests(volume_test)
gtest_discover_tests(geometry_test)
# Note: procedural_geometry_test uses plain main(), not gtest
if(TARGET hdf5_test)
  gtest_discover_tests(hdf5_test)
endif()

# Optional: Add a custom target to run all tests
add_custom_target(check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS ${TEST_TARGETS}
  COMMENT "Running all unit tests"
)
