# Unit tests for trans-cvc library
cmake_minimum_required(VERSION 3.15)

# Test executables
add_executable(app_test app_test.cpp)
add_executable(state_test state_test.cpp)
add_executable(voxels_test voxels_test.cpp)
add_executable(volume_test volume_test.cpp)
add_executable(geometry_test geometry_test.cpp)

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

# Set C++ standard to match the library
target_compile_features(app_test PRIVATE cxx_std_14)
target_compile_features(state_test PRIVATE cxx_std_14)
target_compile_features(voxels_test PRIVATE cxx_std_14)
target_compile_features(volume_test PRIVATE cxx_std_14)
target_compile_features(geometry_test PRIVATE cxx_std_14)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(app_test)
gtest_discover_tests(state_test)
gtest_discover_tests(voxels_test)
gtest_discover_tests(volume_test)
gtest_discover_tests(geometry_test)

# Optional: Add a custom target to run all tests
add_custom_target(check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS app_test state_test voxels_test volume_test
  COMMENT "Running all unit tests"
)
