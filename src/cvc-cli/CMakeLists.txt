cmake_minimum_required(VERSION 3.15)

# Unified CVC CLI executable
add_executable(cvc-cli main.cpp)

# Link against the libcvc library (brings in Boost::filesystem, etc.)
target_link_libraries(cvc-cli PRIVATE cvc::cvc)

# We also need Boost::program_options which is not part of the library
find_package(Boost REQUIRED COMPONENTS program_options)
target_link_libraries(cvc-cli PRIVATE Boost::program_options)

# Install the executable as "cvc"
set_target_properties(cvc-cli PROPERTIES OUTPUT_NAME cvc)
install(TARGETS cvc-cli RUNTIME DESTINATION bin)

# ---------------------------------------------------------------------------
# Integration tests (exercise the CLI as a subprocess)
# ---------------------------------------------------------------------------
if(CVC_BUILD_TESTS)
  add_executable(cvc_cli_test cvc_cli_test.cpp)
  target_link_libraries(cvc_cli_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(cvc_cli_test PRIVATE cxx_std_17)

  # Make sure cvc-cli is built before the test runs
  add_dependencies(cvc_cli_test cvc-cli)

  include(GoogleTest)
  gtest_discover_tests(cvc_cli_test
    PROPERTIES ENVIRONMENT "CVC_CLI_BINARY=$<TARGET_FILE:cvc-cli>"
  )
endif()
