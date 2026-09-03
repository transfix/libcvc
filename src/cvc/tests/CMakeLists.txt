# Unit tests for libcvc library
cmake_minimum_required(VERSION 3.15)

# Test executables
add_executable(app_test app_test.cpp)
add_executable(thread_pool_test thread_pool_test.cpp)
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
add_executable(state_cluster_membership_test state_cluster_membership_test.cpp)
add_executable(state_distributed_bench_test state_distributed_bench_test.cpp)
add_executable(state_message_test state_message_test.cpp)
add_executable(state_bounded_queue_test state_bounded_queue_test.cpp)
add_executable(state_transport_inproc_test state_transport_inproc_test.cpp)
# state_transport_ipc is POSIX-only (UDS + poll); skip on Windows.
if(NOT WIN32)
  add_executable(state_transport_ipc_test state_transport_ipc_test.cpp)
  add_executable(state_reconnect_resilience_test state_reconnect_resilience_test.cpp)
endif()
add_executable(state_blob_transport_integration_test state_blob_transport_integration_test.cpp)
add_executable(state_distributed_delegation_integration_test state_distributed_delegation_integration_test.cpp)
add_executable(state_link_test state_link_test.cpp)
add_executable(state_send_message_test state_send_message_test.cpp)
add_executable(state_link_cycles_test state_link_cycles_test.cpp)
add_executable(state_expiry_test state_expiry_test.cpp)
add_executable(state_interest_filter_test state_interest_filter_test.cpp)
add_executable(state_link_mode_test state_link_mode_test.cpp)
add_executable(state_writable_link_test state_writable_link_test.cpp)
add_executable(state_transparent_link_index_test state_transparent_link_index_test.cpp)
add_executable(state_sync_adapter_link_forwarding_test state_sync_adapter_link_forwarding_test.cpp)
add_executable(state_cross_cluster_link_test state_cross_cluster_link_test.cpp)
add_executable(state_link_bench_test state_link_bench_test.cpp)
add_executable(state_list_test state_list_test.cpp)
if(CVC_STATE_MEMORY_MANAGER)
  add_executable(state_memory_manager_test state_memory_manager_test.cpp)
endif()
if(CVC_STATE_EXEC)
  add_executable(state_exec_types_test state_exec_types_test.cpp)
  add_executable(state_exec_parser_test state_exec_parser_test.cpp)
  add_executable(state_exec_builtins_test state_exec_builtins_test.cpp)
  add_executable(state_exec_evaluator_test state_exec_evaluator_test.cpp)
  add_executable(state_exec_stackless_evaluator_test state_exec_stackless_evaluator_test.cpp)
  add_executable(state_exec_codec_test state_exec_codec_test.cpp)
  add_executable(state_exec_async_test state_exec_async_test.cpp)
  add_executable(state_exec_scheduler_test state_exec_scheduler_test.cpp)
  add_executable(state_exec_intrinsics_test state_exec_intrinsics_test.cpp)
  add_executable(state_exec_coordinator_test state_exec_coordinator_test.cpp)
  add_executable(state_exec_bench_test state_exec_bench_test.cpp)
  add_executable(state_exec_integration_test state_exec_integration_test.cpp)
  add_executable(state_exec_advanced_integration_test state_exec_advanced_integration_test.cpp)
  add_executable(state_exec_gaps_test state_exec_gaps_test.cpp)
  if(NOT WIN32)
    add_executable(state_exec_multiprocess_test state_exec_multiprocess_test.cpp)
  endif()
endif()
add_executable(state_delta_codec_test state_delta_codec_test.cpp)
add_executable(state_volume_codec_test state_volume_codec_test.cpp)
add_executable(state_brick_manifest_test state_brick_manifest_test.cpp)
add_executable(state_data_hydrator_test state_data_hydrator_test.cpp)
add_executable(state_node_telemetry_test state_node_telemetry_test.cpp)
add_executable(state_telemetry_aggregator_test state_telemetry_aggregator_test.cpp)
add_executable(file_state_blob_store_test file_state_blob_store_test.cpp)
add_executable(distributed_state_session_test distributed_state_session_test.cpp)
add_executable(state_large_tree_bench_test state_large_tree_bench_test.cpp)
add_executable(state_hash_partition_test state_hash_partition_test.cpp)
add_executable(state_hybrid_time_test state_hybrid_time_test.cpp)
add_executable(world_clock_test world_clock_test.cpp)
add_executable(state_brick_frustum_test state_brick_frustum_test.cpp)
add_executable(state_conflict_ring_test state_conflict_ring_test.cpp)
add_executable(state_zstd_round_trip_test state_zstd_round_trip_test.cpp)
add_executable(state_session_status_test state_session_status_test.cpp)
if(NOT WIN32)
  add_executable(state_multiprocess_ipc_test state_multiprocess_ipc_test.cpp)
  add_executable(state_snapshot_protocol_test state_snapshot_protocol_test.cpp)
endif()
if(CVC_ENABLE_GRPC)
  add_executable(state_transport_grpc_test state_transport_grpc_test.cpp)
endif()
add_executable(voxels_test voxels_test.cpp)
add_executable(volume_test volume_test.cpp)
add_executable(volren_math_test volren_math_test.cpp)
add_executable(volren_camera_test volren_camera_test.cpp)
add_executable(volren_tf_test volren_tf_test.cpp)
add_executable(volren_kernels_test volren_kernels_test.cpp)
add_executable(volren_render_test volren_render_test.cpp)
add_executable(volren_state_test volren_state_test.cpp)
add_executable(volren_cuda_test volren_cuda_test.cpp)
add_executable(procedural_geometry_test procedural_geometry_test.cpp)
add_executable(volume_ops_test volume_ops_test.cpp)
add_executable(volume_io_test volume_io_test.cpp)
add_executable(volume_io_extra_test volume_io_extra_test.cpp)
add_executable(image_test image_test.cpp)
add_executable(model_test model_test.cpp)
add_executable(geometry_attributes_test geometry_attributes_test.cpp)
add_executable(utility_test utility_test.cpp)
add_executable(nav_test nav_test.cpp)
add_executable(nav_coef_train_test nav_coef_train_test.cpp)
add_executable(nav_drive_refinements_test nav_drive_refinements_test.cpp)
add_executable(nav_clearance_cost_test nav_clearance_cost_test.cpp)
add_executable(nav_material_test nav_material_test.cpp)
add_executable(nav_coef_energy_test nav_coef_energy_test.cpp)
add_executable(nav_material_rollout_grad_test nav_material_rollout_grad_test.cpp)
add_executable(nav_coef_energy_grad_test nav_coef_energy_grad_test.cpp)
add_executable(nav_material_rollout_cuda_test nav_material_rollout_cuda_test.cpp)
add_executable(nav_coef_energy_cuda_test nav_coef_energy_cuda_test.cpp)
add_executable(nav_material_adam_cuda_test nav_material_adam_cuda_test.cpp)
add_executable(nav_material_train_cuda_test nav_material_train_cuda_test.cpp)
add_executable(nav_material_train_test nav_material_train_test.cpp)
add_executable(nav_material_optim_test nav_material_optim_test.cpp)
add_executable(nav_geom_rollout_grad_test nav_geom_rollout_grad_test.cpp)
add_executable(lod_select_test lod_select_test.cpp)
add_executable(lod_select_bench_test lod_select_bench_test.cpp)
add_executable(vis_frustum_test vis_frustum_test.cpp)
add_executable(vis_oracle_test vis_oracle_test.cpp)
add_executable(vis_grid_test vis_grid_test.cpp)
add_executable(vis_tiler_test vis_tiler_test.cpp)

# Every test target `check` should build. Keep this in the same order as the
# add_executable() calls above so the two blocks can be diffed by eye; the
# guard at the bottom of this file turns any omission into a configure error.
set(TEST_TARGETS
  app_test
  thread_pool_test
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
  state_peer_registry_test
  state_write_policy_test
  state_distributed_metrics_test
  state_delegation_manager_test
  state_distributed_admin_test
  state_cluster_membership_test
  state_distributed_bench_test
  state_message_test
  state_bounded_queue_test
  state_transport_inproc_test
  state_blob_transport_integration_test
  state_distributed_delegation_integration_test
  state_link_test
  state_send_message_test
  state_link_cycles_test
  state_expiry_test
  state_interest_filter_test
  state_link_mode_test
  state_writable_link_test
  state_transparent_link_index_test
  state_sync_adapter_link_forwarding_test
  state_cross_cluster_link_test
  state_link_bench_test
  state_list_test
  state_delta_codec_test
  state_volume_codec_test
  state_brick_manifest_test
  world_clock_test
  state_data_hydrator_test
  state_node_telemetry_test
  state_telemetry_aggregator_test
  file_state_blob_store_test
  distributed_state_session_test
  state_large_tree_bench_test
  state_hash_partition_test
  state_hybrid_time_test
  state_brick_frustum_test
  state_conflict_ring_test
  state_zstd_round_trip_test
  state_session_status_test
  voxels_test
  volume_test
  procedural_geometry_test
  volume_ops_test
  volume_io_test
  volume_io_extra_test
  image_test
  model_test
  geometry_attributes_test
  utility_test
  nav_test
  nav_coef_train_test
  nav_drive_refinements_test
  nav_clearance_cost_test
  nav_material_test
  nav_coef_energy_test
  nav_material_rollout_grad_test
  nav_coef_energy_grad_test
  nav_material_train_test
  nav_material_optim_test
  nav_geom_rollout_grad_test
  nav_material_rollout_cuda_test
  nav_coef_energy_cuda_test
  nav_material_adam_cuda_test
  nav_material_train_cuda_test
  lod_select_test
  lod_select_bench_test
  vis_frustum_test
  vis_oracle_test
  vis_grid_test
  vis_tiler_test
  volren_math_test
  volren_camera_test
  volren_tf_test
  volren_kernels_test
  volren_render_test
  volren_state_test
  volren_cuda_test
)

# POSIX-only targets, declared under the matching if(NOT WIN32) guards above.
if(NOT WIN32)
  list(APPEND TEST_TARGETS
    state_transport_ipc_test
    state_reconnect_resilience_test
    state_multiprocess_ipc_test
    state_snapshot_protocol_test
  )
endif()

if(CVC_STATE_EXEC)
  list(APPEND TEST_TARGETS
    state_exec_types_test
    state_exec_parser_test
    state_exec_builtins_test
    state_exec_evaluator_test
    state_exec_stackless_evaluator_test
    state_exec_codec_test
    state_exec_async_test
    state_exec_scheduler_test
    state_exec_intrinsics_test
    state_exec_coordinator_test
    state_exec_bench_test
    state_exec_integration_test
    state_exec_advanced_integration_test
    state_exec_gaps_test
  )
  if(NOT WIN32)
    list(APPEND TEST_TARGETS state_exec_multiprocess_test)
  endif()
endif()

if(CVC_STATE_MEMORY_MANAGER)
  list(APPEND TEST_TARGETS state_memory_manager_test)
endif()

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

  # Deep-drive tests for the LBIE octree mesher and the FastContouring
  # module. These include mesher-internal headers (LBIE/, Mesher/,
  # FastContouring/), which are PRIVATE include dirs of the cvc target,
  # so the test targets add them explicitly.
  add_executable(lbie_mesher_test lbie_mesher_test.cpp)
  target_include_directories(lbie_mesher_test PRIVATE
    ${PROJECT_SOURCE_DIR}/src/cvc/geometry/cvc-mesher
    ${PROJECT_SOURCE_DIR}/src/cvc/geometry/cvc-mesher/LBIE
    ${PROJECT_SOURCE_DIR}/src/cvc/geometry/cvc-mesher/Mesher
    ${PROJECT_SOURCE_DIR}/src/cvc/geometry/cvc-mesher/FastContouring)
  list(APPEND TEST_TARGETS lbie_mesher_test)

  add_executable(fastcontouring_math_test fastcontouring_math_test.cpp)
  target_include_directories(fastcontouring_math_test PRIVATE
    ${PROJECT_SOURCE_DIR}/src/cvc/geometry/cvc-mesher/FastContouring)
  list(APPEND TEST_TARGETS fastcontouring_math_test)
else()
  message(STATUS "Mesher disabled; skipping algorithm_test")
endif()

# mtxlib (SDF V2's self-contained vector/matrix library) and the other
# SignDistanceFunction_v2 internals are compiled whenever SDF is enabled;
# their headers are PRIVATE to the cvc target, so add the dir explicitly.
if(CVC_ENABLE_SDF)
  add_executable(mtxlib_test mtxlib_test.cpp)
  target_include_directories(mtxlib_test PRIVATE
    ${PROJECT_SOURCE_DIR}/src/cvc/geometry/SDF/SignDistanceFunction_v2)
  list(APPEND TEST_TARGETS mtxlib_test)
else()
  message(STATUS "SDF disabled; skipping mtxlib_test")
endif()

# Link against the CVC library and Google Test
target_link_libraries(app_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(thread_pool_test
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

target_link_libraries(volren_math_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_math_test PRIVATE cxx_std_17)

target_link_libraries(volren_camera_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_camera_test PRIVATE cxx_std_17)

target_link_libraries(volren_tf_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_tf_test PRIVATE cxx_std_17)

target_link_libraries(volren_kernels_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_kernels_test PRIVATE cxx_std_17)

target_link_libraries(volren_render_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_render_test PRIVATE cxx_std_17)

target_link_libraries(volren_state_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_state_test PRIVATE cxx_std_17)

target_link_libraries(volren_cuda_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)
target_compile_features(volren_cuda_test PRIVATE cxx_std_17)

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

if(TARGET lbie_mesher_test)
  target_link_libraries(lbie_mesher_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(lbie_mesher_test PRIVATE cxx_std_17)
endif()

if(TARGET fastcontouring_math_test)
  target_link_libraries(fastcontouring_math_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(fastcontouring_math_test PRIVATE cxx_std_17)
endif()

if(TARGET mtxlib_test)
  target_link_libraries(mtxlib_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(mtxlib_test PRIVATE cxx_std_17)
endif()

target_link_libraries(volume_io_extra_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

if(TARGET state_exec_gaps_test)
  target_link_libraries(state_exec_gaps_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
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

target_link_libraries(image_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(model_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(geometry_attributes_test
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

target_link_libraries(nav_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_coef_train_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_drive_refinements_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_clearance_cost_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_coef_energy_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_rollout_grad_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_coef_energy_grad_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_rollout_cuda_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_coef_energy_cuda_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_adam_cuda_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_train_cuda_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_train_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_material_optim_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(nav_geom_rollout_grad_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(lod_select_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(lod_select_bench_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(vis_frustum_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(vis_oracle_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(vis_grid_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(vis_tiler_test
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

target_link_libraries(state_cluster_membership_test
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

if(NOT WIN32)
  target_link_libraries(state_transport_ipc_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_link_libraries(state_reconnect_resilience_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
endif()

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

target_link_libraries(state_expiry_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_interest_filter_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_link_mode_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_writable_link_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_transparent_link_index_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_sync_adapter_link_forwarding_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_cross_cluster_link_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_link_bench_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_volume_codec_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_node_telemetry_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_brick_manifest_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_data_hydrator_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_telemetry_aggregator_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

if(CVC_STATE_EXEC)
  target_link_libraries(state_exec_parser_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_types_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_builtins_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_evaluator_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_stackless_evaluator_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_codec_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_async_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_scheduler_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_intrinsics_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_coordinator_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_bench_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_integration_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )

  target_link_libraries(state_exec_advanced_integration_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  if(TARGET state_exec_multiprocess_test)
    target_link_libraries(state_exec_multiprocess_test
      PRIVATE
        cvc
        GTest::gtest
        GTest::gtest_main
    )
  endif()
endif()

if(CVC_STATE_MEMORY_MANAGER)
  target_link_libraries(state_memory_manager_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
endif()

target_link_libraries(state_list_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_delta_codec_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(file_state_blob_store_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(distributed_state_session_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_large_tree_bench_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_hash_partition_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_hybrid_time_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(world_clock_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_brick_frustum_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_conflict_ring_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_zstd_round_trip_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(state_session_status_test
  PRIVATE
    cvc
    GTest::gtest
    GTest::gtest_main
)

if(NOT WIN32)
  target_link_libraries(state_multiprocess_ipc_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
  target_link_libraries(state_snapshot_protocol_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
  )
endif()

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
target_compile_features(image_test PRIVATE cxx_std_17)
target_compile_features(model_test PRIVATE cxx_std_17)
target_compile_features(geometry_attributes_test PRIVATE cxx_std_17)
target_compile_features(utility_test PRIVATE cxx_std_17)
target_compile_features(nav_test PRIVATE cxx_std_17)
target_compile_features(nav_coef_train_test PRIVATE cxx_std_17)
target_compile_features(nav_material_test PRIVATE cxx_std_17)
target_compile_features(nav_coef_energy_test PRIVATE cxx_std_17)
target_compile_features(nav_material_rollout_grad_test PRIVATE cxx_std_17)
target_compile_features(nav_coef_energy_grad_test PRIVATE cxx_std_17)
target_compile_features(nav_material_rollout_cuda_test PRIVATE cxx_std_17)
target_compile_features(nav_coef_energy_cuda_test PRIVATE cxx_std_17)
target_compile_features(nav_material_adam_cuda_test PRIVATE cxx_std_17)
target_compile_features(nav_material_train_cuda_test PRIVATE cxx_std_17)
target_compile_features(nav_material_train_test PRIVATE cxx_std_17)
target_compile_features(nav_material_optim_test PRIVATE cxx_std_17)
target_compile_features(nav_geom_rollout_grad_test PRIVATE cxx_std_17)
# Let the sim_world gtest load the shipped reference policy from the source tree.
if(EXISTS "${PROJECT_SOURCE_DIR}/share/cvc/nav/coef_mlp.cvcnav")
  target_compile_definitions(nav_test PRIVATE
    CVC_NAV_SHIPPED_WEIGHTS="${PROJECT_SOURCE_DIR}/share/cvc/nav/coef_mlp.cvcnav")
endif()
# Enable the CUDA parity tests (device-resident sim_world + CUDA trainer) when built.
if(CVC_ENABLE_CUDA)
  target_compile_definitions(nav_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(nav_coef_train_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(nav_drive_refinements_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(nav_material_rollout_cuda_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(nav_coef_energy_cuda_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(nav_material_adam_cuda_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(nav_material_train_cuda_test PRIVATE CVC_ENABLE_CUDA)
  target_compile_definitions(volren_cuda_test PRIVATE CVC_ENABLE_CUDA)
endif()
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
target_compile_features(state_cluster_membership_test PRIVATE cxx_std_17)
target_compile_features(state_distributed_bench_test PRIVATE cxx_std_17)
target_compile_features(state_message_test PRIVATE cxx_std_17)
target_compile_features(state_bounded_queue_test PRIVATE cxx_std_17)
target_compile_features(state_transport_inproc_test PRIVATE cxx_std_17)
if(NOT WIN32)
  target_compile_features(state_transport_ipc_test PRIVATE cxx_std_17)
  target_compile_features(state_reconnect_resilience_test PRIVATE cxx_std_17)
endif()
target_compile_features(state_blob_transport_integration_test PRIVATE cxx_std_17)
target_compile_features(state_distributed_delegation_integration_test PRIVATE cxx_std_17)
target_compile_features(state_link_test PRIVATE cxx_std_17)
target_compile_features(state_send_message_test PRIVATE cxx_std_17)
target_compile_features(state_link_cycles_test PRIVATE cxx_std_17)
target_compile_features(state_expiry_test PRIVATE cxx_std_17)
target_compile_features(state_interest_filter_test PRIVATE cxx_std_17)
target_compile_features(state_link_mode_test PRIVATE cxx_std_17)
target_compile_features(state_writable_link_test PRIVATE cxx_std_17)
target_compile_features(state_transparent_link_index_test PRIVATE cxx_std_17)
target_compile_features(state_sync_adapter_link_forwarding_test PRIVATE cxx_std_17)
target_compile_features(state_cross_cluster_link_test PRIVATE cxx_std_17)
target_compile_features(state_link_bench_test PRIVATE cxx_std_17)
if(CVC_STATE_EXEC)
  target_compile_features(state_exec_parser_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_types_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_builtins_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_evaluator_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_stackless_evaluator_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_codec_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_async_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_scheduler_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_intrinsics_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_coordinator_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_bench_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_integration_test PRIVATE cxx_std_20)
  target_compile_features(state_exec_advanced_integration_test PRIVATE cxx_std_20)
  if(TARGET state_exec_multiprocess_test)
    target_compile_features(state_exec_multiprocess_test PRIVATE cxx_std_20)
  endif()
endif()
if(CVC_STATE_MEMORY_MANAGER)
  target_compile_features(state_memory_manager_test PRIVATE cxx_std_17)
endif()
target_compile_features(state_list_test PRIVATE cxx_std_17)
target_compile_features(state_delta_codec_test PRIVATE cxx_std_17)
target_compile_features(state_volume_codec_test PRIVATE cxx_std_17)
target_compile_features(state_brick_manifest_test PRIVATE cxx_std_17)
target_compile_features(state_data_hydrator_test PRIVATE cxx_std_17)
target_compile_features(state_node_telemetry_test PRIVATE cxx_std_17)
target_compile_features(state_telemetry_aggregator_test PRIVATE cxx_std_17)
target_compile_features(file_state_blob_store_test PRIVATE cxx_std_17)
target_compile_features(distributed_state_session_test PRIVATE cxx_std_17)
target_compile_features(state_large_tree_bench_test PRIVATE cxx_std_17)
target_compile_features(state_hash_partition_test PRIVATE cxx_std_17)
target_compile_features(state_hybrid_time_test PRIVATE cxx_std_17)
target_compile_features(world_clock_test PRIVATE cxx_std_17)
target_compile_features(state_brick_frustum_test PRIVATE cxx_std_17)
target_compile_features(state_conflict_ring_test PRIVATE cxx_std_17)
target_compile_features(state_zstd_round_trip_test PRIVATE cxx_std_17)
target_compile_features(state_session_status_test PRIVATE cxx_std_17)
if(NOT WIN32)
  target_compile_features(state_multiprocess_ipc_test PRIVATE cxx_std_17)
  target_compile_features(state_snapshot_protocol_test PRIVATE cxx_std_17)
endif()

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

  # Volume-level HDF5 backend tests (hdf5_io.cpp via volume::read/write,
  # volume_file_info, |object addressing) — separate target so hdf5_test
  # stays the low-level hdf5_utils suite.
  add_executable(hdf5_volume_test hdf5_volume_test.cpp)
  target_link_libraries(hdf5_volume_test
    PRIVATE
      cvc
      GTest::gtest
      GTest::gtest_main
      ${HDF5_LIBRARIES}
  )
  target_include_directories(hdf5_volume_test PRIVATE ${HDF5_INCLUDE_DIRS})
  target_compile_features(hdf5_volume_test PRIVATE cxx_std_14)
  list(APPEND TEST_TARGETS hdf5_volume_test)
else()
  message(STATUS "HDF5 disabled; skipping hdf5_test")
endif()

# Register tests with CTest
include(GoogleTest)

# ── macOS test-discovery robustness ───────────────────────────────────────
# gtest_discover_tests defaults to POST_BUILD mode: it runs each freshly-built
# test executable at BUILD time to enumerate its cases. On the macOS *debug* CI
# that is fragile — every launch dyld-loads the full libcvc-deps dylib closure
# (~30 libs: boost, Magick++, hdf5, tiff, gmp/mpfr, …) and, under parallel-build
# CPU contention, intermittently exceeds the 5 s default DISCOVERY_TIMEOUT. The
# exe is killed before it prints its --gtest_list_tests JSON, so CMake's
# ParseTestList aborts the *build* with "Syntax error: value, object or array
# expected". The offending test varied run to run — the signature of a timeout,
# not a real per-test crash. Release binaries load fast enough to stay under
# 5 s, which is why only macos-debug failed.
#
# Fix (macOS only, so Linux/Windows POST_BUILD behavior — including Windows'
# build-time DLL-PATH setup for discovery — is left untouched): run discovery at
# TEST time (PRE_TEST), moving it off the build critical path, and give cold
# dyld loads a generous timeout. macOS binaries resolve their deps via embedded
# @rpath, so test-time discovery needs no extra environment. _gtest_discover_tests
# is the original module command, shadowed here by a same-named macro so every
# bare gtest_discover_tests(<target>) call below inherits these options.
if(APPLE)
  macro(gtest_discover_tests)
    _gtest_discover_tests(${ARGV} DISCOVERY_MODE PRE_TEST DISCOVERY_TIMEOUT 120)
  endmacro()
elseif(WIN32)
  # Same 5 s-timeout failure mode as macOS, but here under nvcc CPU contention
  # from the CUDA nav translation units (drive.cu / coef_train.cu): POST_BUILD
  # discovery runs each freshly-built test exe at build time to list its cases,
  # and on a saturated runner the exe finishes printing its --gtest_list_tests
  # output but is killed before it *exits* within the 5 s default (observed on
  # state_write_policy_test / state_brick_manifest_test — a timeout, not a real
  # per-test hang; those exes exit promptly on an unloaded runner). Keep
  # POST_BUILD (Windows relies on its build-time DLL-PATH setup for discovery)
  # and just give it the same generous timeout.
  macro(gtest_discover_tests)
    _gtest_discover_tests(${ARGV} DISCOVERY_TIMEOUT 120)
  endmacro()
endif()

# ── Discovery drift guard, registration side ──────────────────────────────
# nav_coef_train_test sat in TEST_TARGETS, compiled green, and never ran:
# nothing called gtest_discover_tests() on it, and the TEST_TARGETS guard at
# the bottom of this file only proves a target *builds* under `check`, not
# that CTest knows about it. Route every discovery through this wrapper so
# the guard can also prove each TEST_TARGETS entry is registered with CTest.
# Plain-main() targets registered via add_test() append themselves to
# CVC_REGISTERED_TESTS directly.
function(cvc_discover_tests target)
  gtest_discover_tests(${target} ${ARGN})
  set_property(DIRECTORY APPEND PROPERTY CVC_REGISTERED_TESTS ${target})
endfunction()

cvc_discover_tests(app_test)
cvc_discover_tests(thread_pool_test)
cvc_discover_tests(state_test)
cvc_discover_tests(voxels_test)
cvc_discover_tests(volume_test)
cvc_discover_tests(volren_math_test)
cvc_discover_tests(volren_camera_test)
cvc_discover_tests(volren_tf_test)
cvc_discover_tests(volren_kernels_test)
cvc_discover_tests(volren_render_test)
cvc_discover_tests(volren_state_test)
cvc_discover_tests(volren_cuda_test)
if(TARGET geometry_test)
  cvc_discover_tests(geometry_test)
endif()
if(TARGET algorithm_test)
  cvc_discover_tests(algorithm_test)
endif()
cvc_discover_tests(volume_ops_test)
cvc_discover_tests(volume_io_test)
cvc_discover_tests(image_test)
cvc_discover_tests(model_test)
cvc_discover_tests(geometry_attributes_test)
cvc_discover_tests(utility_test)
cvc_discover_tests(nav_test)
cvc_discover_tests(nav_coef_train_test)
cvc_discover_tests(nav_drive_refinements_test)
cvc_discover_tests(nav_clearance_cost_test)
cvc_discover_tests(nav_material_test)
cvc_discover_tests(nav_coef_energy_test)
cvc_discover_tests(nav_material_rollout_grad_test)
cvc_discover_tests(nav_coef_energy_grad_test)
cvc_discover_tests(nav_material_rollout_cuda_test)
cvc_discover_tests(nav_coef_energy_cuda_test)
cvc_discover_tests(nav_material_adam_cuda_test)
cvc_discover_tests(nav_material_train_cuda_test)
cvc_discover_tests(nav_material_train_test)
cvc_discover_tests(nav_material_optim_test)
cvc_discover_tests(nav_geom_rollout_grad_test)
cvc_discover_tests(lod_select_test)
cvc_discover_tests(lod_select_bench_test)
cvc_discover_tests(vis_frustum_test)
cvc_discover_tests(vis_oracle_test)
cvc_discover_tests(vis_grid_test)
cvc_discover_tests(vis_tiler_test)
cvc_discover_tests(volume_io_extra_test)
if(TARGET lbie_mesher_test)
  cvc_discover_tests(lbie_mesher_test)
endif()
if(TARGET fastcontouring_math_test)
  cvc_discover_tests(fastcontouring_math_test)
endif()
if(TARGET mtxlib_test)
  cvc_discover_tests(mtxlib_test)
endif()
if(TARGET hdf5_volume_test)
  cvc_discover_tests(hdf5_volume_test)
endif()
if(TARGET state_exec_gaps_test)
  cvc_discover_tests(state_exec_gaps_test)
endif()
cvc_discover_tests(state_change_journal_test)
cvc_discover_tests(state_subscription_router_test)
cvc_discover_tests(state_sync_adapter_test)
cvc_discover_tests(state_blob_store_test)
cvc_discover_tests(state_chunked_blob_test)
cvc_discover_tests(state_compression_registry_test)
cvc_discover_tests(state_codec_registry_test)
cvc_discover_tests(state_replica_test)
cvc_discover_tests(state_authority_map_test)
cvc_discover_tests(state_cluster_shard_test)
cvc_discover_tests(state_message_bus_test)
cvc_discover_tests(state_peer_registry_test)
cvc_discover_tests(state_write_policy_test)
cvc_discover_tests(state_distributed_metrics_test)
cvc_discover_tests(state_delegation_manager_test)
cvc_discover_tests(state_distributed_admin_test)
cvc_discover_tests(state_cluster_membership_test)
cvc_discover_tests(state_distributed_bench_test)
cvc_discover_tests(state_message_test)
cvc_discover_tests(state_bounded_queue_test)
cvc_discover_tests(state_transport_inproc_test)
if(NOT WIN32)
  cvc_discover_tests(state_transport_ipc_test)
  cvc_discover_tests(state_reconnect_resilience_test)
endif()
cvc_discover_tests(state_blob_transport_integration_test)
cvc_discover_tests(state_distributed_delegation_integration_test)
cvc_discover_tests(state_link_test)
cvc_discover_tests(state_send_message_test)
cvc_discover_tests(state_link_cycles_test)
cvc_discover_tests(state_expiry_test)
cvc_discover_tests(state_interest_filter_test)
cvc_discover_tests(state_link_mode_test)
cvc_discover_tests(state_writable_link_test)
cvc_discover_tests(state_transparent_link_index_test)
cvc_discover_tests(state_sync_adapter_link_forwarding_test)
cvc_discover_tests(state_cross_cluster_link_test)
cvc_discover_tests(state_link_bench_test)
if(CVC_STATE_EXEC)
  cvc_discover_tests(state_exec_parser_test)
  cvc_discover_tests(state_exec_types_test)
  cvc_discover_tests(state_exec_builtins_test)
  cvc_discover_tests(state_exec_evaluator_test)
  cvc_discover_tests(state_exec_stackless_evaluator_test)
  cvc_discover_tests(state_exec_codec_test)
  cvc_discover_tests(state_exec_async_test)
  cvc_discover_tests(state_exec_scheduler_test)
  cvc_discover_tests(state_exec_intrinsics_test)
  cvc_discover_tests(state_exec_coordinator_test)
  cvc_discover_tests(state_exec_bench_test)
  cvc_discover_tests(state_exec_integration_test)
  cvc_discover_tests(state_exec_advanced_integration_test)
  if(TARGET state_exec_multiprocess_test)
    # Forks real OS processes and replicates state over Unix-domain-socket IPC
    # with internal pump deadlines. Under `ctest --parallel` on a loaded CI
    # runner the forked children get starved and miss those windows, so
    # StateTreeReplication intermittently fails all `until-pass` retries.
    # RUN_SERIAL keeps this suite off the shared parallel scheduler.
    cvc_discover_tests(state_exec_multiprocess_test PROPERTIES RUN_SERIAL TRUE)
  endif()
endif()
if(CVC_STATE_MEMORY_MANAGER)
  cvc_discover_tests(state_memory_manager_test)
endif()
cvc_discover_tests(state_list_test)
cvc_discover_tests(state_delta_codec_test)
cvc_discover_tests(state_volume_codec_test)
cvc_discover_tests(state_brick_manifest_test)
cvc_discover_tests(state_data_hydrator_test)
cvc_discover_tests(state_node_telemetry_test)
cvc_discover_tests(state_telemetry_aggregator_test)
cvc_discover_tests(file_state_blob_store_test)
cvc_discover_tests(distributed_state_session_test)
cvc_discover_tests(state_large_tree_bench_test)
cvc_discover_tests(state_hash_partition_test)
cvc_discover_tests(state_hybrid_time_test)
cvc_discover_tests(world_clock_test)
cvc_discover_tests(state_brick_frustum_test)
cvc_discover_tests(state_conflict_ring_test)
cvc_discover_tests(state_zstd_round_trip_test)
cvc_discover_tests(state_session_status_test)
if(NOT WIN32)
  # Same fork()+IPC timing profile as state_exec_multiprocess_test — serialize
  # so the parallel scheduler can't starve its socket/replication windows.
  cvc_discover_tests(state_multiprocess_ipc_test PROPERTIES RUN_SERIAL TRUE)
  cvc_discover_tests(state_snapshot_protocol_test)
endif()
if(TARGET state_transport_grpc_test)
  cvc_discover_tests(state_transport_grpc_test)
endif()
# procedural_geometry_test uses plain main(), not gtest, so register it with
# a direct add_test() and record it for the registration guard below.
add_test(NAME procedural_geometry_test COMMAND procedural_geometry_test)
set_property(DIRECTORY APPEND PROPERTY CVC_REGISTERED_TESTS procedural_geometry_test)
if(TARGET hdf5_test)
  cvc_discover_tests(hdf5_test)
endif()

# ── TEST_TARGETS drift guard ──────────────────────────────────────────────
# `check` builds exactly what TEST_TARGETS names, so a test declared with
# add_executable() but never added to the list quietly stops being built by
# `make check` / `ninja check`. It keeps *passing* under a bare `ctest` for as
# long as a stale binary survives in the build tree, which is what let 20
# targets drift off the list unnoticed.
#
# Fail at configure time instead. Every executable defined in this directory
# must be reachable from TEST_TARGETS; list a target in
# TEST_TARGETS_INTENTIONALLY_EXCLUDED (with a reason) to opt out.
set(TEST_TARGETS_INTENTIONALLY_EXCLUDED
  # (none today — add entries here with a comment explaining why)
)

get_property(_cvc_dir_targets DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
set(_cvc_untracked_tests "")
foreach(_cvc_target IN LISTS _cvc_dir_targets)
  get_target_property(_cvc_target_type ${_cvc_target} TYPE)
  if(_cvc_target_type STREQUAL "EXECUTABLE"
     AND NOT _cvc_target IN_LIST TEST_TARGETS
     AND NOT _cvc_target IN_LIST TEST_TARGETS_INTENTIONALLY_EXCLUDED)
    list(APPEND _cvc_untracked_tests ${_cvc_target})
  endif()
endforeach()

if(_cvc_untracked_tests)
  string(REPLACE ";" "\n  - " _cvc_untracked_pretty "${_cvc_untracked_tests}")
  message(FATAL_ERROR
    "These test executables are not in TEST_TARGETS, so `check` would not "
    "build them:\n  - ${_cvc_untracked_pretty}\n"
    "Add them to TEST_TARGETS in ${CMAKE_CURRENT_LIST_FILE} (under the same "
    "if() guard as their add_executable), or to "
    "TEST_TARGETS_INTENTIONALLY_EXCLUDED with a reason.")
endif()

# The converse: a name in TEST_TARGETS that no longer exists as a target makes
# `check` fail late with an opaque dependency error.
foreach(_cvc_target IN LISTS TEST_TARGETS)
  if(NOT TARGET ${_cvc_target})
    message(FATAL_ERROR
      "TEST_TARGETS names '${_cvc_target}', but no such target is defined in "
      "${CMAKE_CURRENT_LIST_FILE}. Remove it, or move it under the if() guard "
      "that declares it.")
  endif()
endforeach()

# Registration side: building under `check` is not enough — a target that no
# one passes to cvc_discover_tests()/add_test() never runs under ctest, and
# nothing else notices (nav_coef_train_test compiled green for weeks while
# ctest ran none of its cases). Every TEST_TARGETS entry must be registered.
get_property(_cvc_registered DIRECTORY PROPERTY CVC_REGISTERED_TESTS)
foreach(_cvc_target IN LISTS TEST_TARGETS)
  if(NOT _cvc_target IN_LIST _cvc_registered)
    message(FATAL_ERROR
      "Test target '${_cvc_target}' is never registered with CTest. Call "
      "cvc_discover_tests(${_cvc_target}) (gtest binaries), or add_test() + "
      "append to the CVC_REGISTERED_TESTS directory property (plain main()), "
      "in ${CMAKE_CURRENT_LIST_FILE}.")
  endif()
endforeach()

# Optional: Add a custom target to run all tests
add_custom_target(check
  COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
  DEPENDS ${TEST_TARGETS}
  COMMENT "Running all unit tests"
)
