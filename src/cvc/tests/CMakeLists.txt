# Unit tests for libcvc library
cmake_minimum_required(VERSION 3.15)

# Test executables
add_executable(app_test app_test.cpp)
add_executable(state_test state_test.cpp)
add_executable(state_change_journal_test state_change_journal_test.cpp)
add_executable(state_subscription_router_test state_subscription_router_test.cpp)
add_executable(state_sync_adapter_test state_sync_adapter_test.cpp)
add_executable(state_blob_store_test state_blob_store_test.cpp)
add_executable(state_chunked_blob_test state_chunked_blob_test.cpp)
add_executable(state_compression_registry_test state_compression_registry_test.cpp)
add_executable(state_codec_registry_test state_codec_registry_test.cpp)
add_executable(state_replica_test state_replica_test.cpp)
add_executable(state_authority_map_test state_authority_map_test.cpp)
add_executable(state_cluster_shard_test state_cluster_shard_test.cpp)
add_executable(state_message_bus_test state_message_bus_test.cpp)
add_executable(state_peer_registry_test state_peer_registry_test.cpp)
add_executable(state_write_policy_test state_write_policy_test.cpp)
add_executable(state_distributed_metrics_test state_distributed_metrics_test.cpp)
add_executable(state_delegation_manager_test state_delegation_manager_test.cpp)
add_executable(state_distributed_admin_test state_distributed_admin_test.cpp)
add_executable(state_distributed_bench_test state_distributed_bench_test.cpp)
add_executable(state_message_test state_message_test.cpp)
add_executable(state_bounded_queue_test state_bounded_queue_test.cpp)
add_executable(state_transport_inproc_test state_transport_inproc_test.cpp)
add_executable(state_transport_ipc_test state_transport_ipc_test.cpp)
add_executable(state_blob_transport_integration_test state_blob_transport_integration_test.cpp)
add_executable(state_distributed_delegation_integration_test state_distributed_delegation_integration_test.cpp)
add_executable(state_link_test state_link_test.cpp)
add_executable(state_send_message_test state_send_message_test.cpp)
add_executable(state_link_cycles_test state_link_cycles_test.cpp)
if(CVC_ENABLE_GRPC)
  add_executable(state_transport_grpc_test state_transport_grpc_test.cpp)
endif()
add_executable(voxels_test voxels_test.cpp)
add_executable(volume_test volume_test.cpp)
add_executable(procedural_geometry_test procedural_geometry_test.cpp)
add_executable(volume_ops_test volume_ops_test.cpp)
add_executable(volume_io_test volume_io_test.cpp)
add_executable(utility_test utility_test.cpp)

# Track all test targets so we can conditionally add HDF5 tests
set(TEST_TARGETS
  app_test
  state_test
  state_change_journal_test
  state_subscription_router_test
  state_sync_adapter_test
  state_blob_store_test
  state_chunked_blob_test
  state_compression_registry_test
  state_codec_registry_test
  state_replica_test
  state_authority_map_test
  state_cluster_shard_test
  state_message_bus_test
  state_transport_inproc_test
  state_transport_ipc_test
  state_blob_transport_integration_test
  state_distributed_delegation_integration_test
  state_link_test
  state_send_message_test
  state_link_cycles_test
  voxels_test
  volume_test
  procedural_geometry_test
  volume_ops_test
  volume_io_test
  utility_test
)

# geometry_test exercises sdf(), iso(), tetrahedralize(), and
# geometry::quality_improve(), which are only available when SDF and
# Mesher are enabled. Skip it otherwise so core CI stays green.
if(CVC_ENABLE_SDF AND CVC_ENABLE_MESHER)
  add_executable(geometry_test geometry_test.cpp)
  list(APPEND TEST_TARGETS geometry_test)
else()
  message(STATUS "SDF/Mesher disabled; skipping geometry_test")
endif()

# algorithm_test drives cvc::iso/tetrahedralize/hexahedralize which require
# the Mesher.
if(CVC_ENABLE_MESHER)
  add_executable(algorithm_test algorithm_test.cpp)
  list(APPEND TEST_TARGETS algorithm_test)
else()
  message(STATUS "Mesher disabled; skipping algorithm_test")
endif()

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

if(TARGET geometry_test)
  target_link_libraries(geometry_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
endif()

if(TARGET algorithm_test)
  target_link_libraries(algorithm_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(algorithm_test PRIVATE cxx_std_17)
endif()

target_link_libraries(procedural_geometry_test
  PRIVATE
    cvc
)

target_link_libraries(volume_ops_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(volume_io_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(utility_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_change_journal_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_subscription_router_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_sync_adapter_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_blob_store_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_chunked_blob_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_compression_registry_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_codec_registry_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_replica_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_authority_map_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_cluster_shard_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_message_bus_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_peer_registry_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_write_policy_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_distributed_metrics_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_delegation_manager_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_distributed_admin_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_distributed_bench_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_message_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_bounded_queue_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_transport_inproc_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_transport_ipc_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_blob_transport_integration_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_distributed_delegation_integration_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_link_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_send_message_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_link_cycles_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

if(TARGET state_transport_grpc_test)
  target_link_libraries(state_transport_grpc_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(state_transport_grpc_test PRIVATE cxx_std_17)
  list(APPEND TEST_TARGETS state_transport_grpc_test)
endif()

# Set C++ standard to match the library
target_compile_features(app_test PRIVATE cxx_std_14)
target_compile_features(state_test PRIVATE cxx_std_14)
target_compile_features(voxels_test PRIVATE cxx_std_14)
target_compile_features(volume_test PRIVATE cxx_std_14)
if(TARGET geometry_test)
  target_compile_features(geometry_test PRIVATE cxx_std_14)
endif()
target_compile_features(procedural_geometry_test PRIVATE cxx_std_14)
target_compile_features(volume_ops_test PRIVATE cxx_std_14)
target_compile_features(volume_io_test PRIVATE cxx_std_17)
target_compile_features(utility_test PRIVATE cxx_std_17)
target_compile_features(state_change_journal_test PRIVATE cxx_std_17)
target_compile_features(state_subscription_router_test PRIVATE cxx_std_17)
target_compile_features(state_sync_adapter_test PRIVATE cxx_std_17)
target_compile_features(state_blob_store_test PRIVATE cxx_std_17)
target_compile_features(state_chunked_blob_test PRIVATE cxx_std_17)
target_compile_features(state_compression_registry_test PRIVATE cxx_std_17)
target_compile_features(state_codec_registry_test PRIVATE cxx_std_17)
target_compile_features(state_replica_test PRIVATE cxx_std_17)
target_compile_features(state_authority_map_test PRIVATE cxx_std_17)
target_compile_features(state_cluster_shard_test PRIVATE cxx_std_17)
target_compile_features(state_message_bus_test PRIVATE cxx_std_17)
target_compile_features(state_peer_registry_test PRIVATE cxx_std_17)
target_compile_features(state_write_policy_test PRIVATE cxx_std_17)
target_compile_features(state_distributed_metrics_test PRIVATE cxx_std_17)
target_compile_features(state_delegation_manager_test PRIVATE cxx_std_17)
target_compile_features(state_distributed_admin_test PRIVATE cxx_std_17)
target_compile_features(state_distributed_bench_test PRIVATE cxx_std_17)
target_compile_features(state_message_test PRIVATE cxx_std_17)
target_compile_features(state_bounded_queue_test PRIVATE cxx_std_17)
target_compile_features(state_transport_inproc_test PRIVATE cxx_std_17)
target_compile_features(state_transport_ipc_test PRIVATE cxx_std_17)
target_compile_features(state_blob_transport_integration_test PRIVATE cxx_std_17)
target_compile_features(state_distributed_delegation_integration_test PRIVATE cxx_std_17)
target_compile_features(state_link_test PRIVATE cxx_std_17)
target_compile_features(state_send_message_test PRIVATE cxx_std_17)
target_compile_features(state_link_cycles_test PRIVATE cxx_std_17)

# Only build HDF5-dependent tests when HDF5 support is enabled
if(CVC_USING_HDF5 AND NOT CVC_HDF5_DISABLED)
  # Mirror src/cvc/CMakeLists.txt: prefer CONFIG (vcpkg), fall back to
  # MODULE (apt/brew). The cvc target already links against HDF5, so
  # this lookup is just to populate HDF5_LIBRARIES / HDF5_INCLUDE_DIRS
  # for hdf5_test.cpp.
  set(_hdf5_config_targets "")
  find_package(hdf5 CONFIG QUIET)
  if(hdf5_FOUND)
    foreach(_cand
        hdf5::hdf5-shared hdf5::hdf5-static hdf5::hdf5
        hdf5::hdf5_cpp-shared hdf5::hdf5_cpp-static hdf5::hdf5_cpp)
      if(TARGET ${_cand})
        list(APPEND _hdf5_config_targets ${_cand})
      endif()
    endforeach()
  endif()
  if(_hdf5_config_targets)
    set(HDF5_LIBRARIES ${_hdf5_config_targets})
    set(HDF5_INCLUDE_DIRS "")
  else()
    find_package(HDF5 REQUIRED COMPONENTS C CXX)
  endif()
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
if(TARGET geometry_test)
  gtest_discover_tests(geometry_test)
endif()
if(TARGET algorithm_test)
  gtest_discover_tests(algorithm_test)
endif()
gtest_discover_tests(volume_ops_test)
gtest_discover_tests(volume_io_test)
gtest_discover_tests(utility_test)
gtest_discover_tests(state_change_journal_test)
gtest_discover_tests(state_subscription_router_test)
gtest_discover_tests(state_sync_adapter_test)
gtest_discover_tests(state_blob_store_test)
gtest_discover_tests(state_chunked_blob_test)
gtest_discover_tests(state_compression_registry_test)
gtest_discover_tests(state_codec_registry_test)
gtest_discover_tests(state_replica_test)
gtest_discover_tests(state_authority_map_test)
gtest_discover_tests(state_cluster_shard_test)
gtest_discover_tests(state_message_bus_test)
gtest_discover_tests(state_peer_registry_test)
gtest_discover_tests(state_write_policy_test)
gtest_discover_tests(state_distributed_metrics_test)
gtest_discover_tests(state_delegation_manager_test)
gtest_discover_tests(state_distributed_admin_test)
gtest_discover_tests(state_distributed_bench_test)
gtest_discover_tests(state_message_test)
gtest_discover_tests(state_bounded_queue_test)
gtest_discover_tests(state_transport_inproc_test)
gtest_discover_tests(state_transport_ipc_test)
gtest_discover_tests(state_blob_transport_integration_test)
gtest_discover_tests(state_distributed_delegation_integration_test)
gtest_discover_tests(state_link_test)
gtest_discover_tests(state_send_message_test)
gtest_discover_tests(state_link_cycles_test)
if(TARGET state_transport_grpc_test)
  gtest_discover_tests(state_transport_grpc_test)
endif()
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
