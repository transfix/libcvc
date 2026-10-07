// The deprecated <cvc/core/state*.h> / <cvc/core/state_exec/*.h> forwarding headers must keep
// resolving to the real <cvc/state/...> declarations for as long as they ship (one release
// cycle, docs/roadmap/STATE_DIR_LAYOUT_REFACTOR.md). They live in compat/inc, which is on THIS
// target's include path only -- not libcvc's -- so in-tree code cannot drift back to the old
// paths, and this is the one TU that still spells them. Every shim is included below; a shim
// pointing at a header that no longer exists fails the build here, not in a consumer's.
#include <cvc/core/distributed_state_session.h>
#include <cvc/core/state.h>
#include <cvc/core/state_authority_map.h>
#include <cvc/core/state_blob_store.h>
#include <cvc/core/state_bounded_queue.h>
#include <cvc/core/state_brick_manifest.h>
#include <cvc/core/state_change_journal.h>
#include <cvc/core/state_chunked_blob.h>
#include <cvc/core/state_cluster_membership.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_codec_registry.h>
#include <cvc/core/state_compression_registry.h>
#include <cvc/core/state_data_hydrator.h>
#include <cvc/core/state_delegation_manager.h>
#include <cvc/core/state_delta_codec.h>
#include <cvc/core/state_distributed_admin.h>
#include <cvc/core/state_distributed_metrics.h>
#include <cvc/core/state_eviction_store.h>
#include <cvc/core/state_exec/async_evaluator.h>
#include <cvc/core/state_exec/async_scheduler.h>
#include <cvc/core/state_exec/async_stackless_evaluator.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/evaluator.h>
#include <cvc/core/state_exec/exec_coordinator.h>
#include <cvc/core/state_exec/generator.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/resource_policy.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/scheduler_base.h>
#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/state_value_codec.h>
#include <cvc/core/state_exec/stdlib.h>
#include <cvc/core/state_exec/task.h>
#include <cvc/core/state_exec/types.h>
#include <cvc/core/state_exec/utf8.h>
#include <cvc/core/state_hash_partition.h>
#include <cvc/core/state_hybrid_time.h>
#include <cvc/core/state_list.h>
#include <cvc/core/state_memory_manager.h>
#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_node_telemetry.h>
#include <cvc/core/state_object.h>
#include <cvc/core/state_peer_registry.h>
#include <cvc/core/state_replica.h>
#include <cvc/core/state_subscription_router.h>
#include <cvc/core/state_sync_adapter.h>
#include <cvc/core/state_telemetry_aggregator.h>
#include <cvc/core/state_transport.h>
#include <cvc/core/state_transport_grpc.h>
#include <cvc/core/state_transport_inproc.h>
#include <cvc/core/state_transport_ipc.h>
#include <cvc/core/state_volume_codec.h>
#include <cvc/core/state_write_policy.h>
#include <gtest/gtest.h>
#include <string_view>

TEST(StateCompatHeaders, OldPathsReachTheMovedDeclarations) {
  // The include list above is the check: a shim whose target is missing does not compile.
  // These are smoke checks that the moved types are still complete under cvc:: and
  // cvc::state_exec:: (sizeof rejects a merely forward-declared type). They cannot pin a
  // declaration to one particular shim -- the shims' targets include each other.
  static_assert(sizeof(cvc::state) > 0);
  static_assert(sizeof(cvc::state_list) > 0);
  static_assert(sizeof(cvc::state_replica) > 0);
  static_assert(sizeof(cvc::distributed_state_session) > 0);
  static_assert(sizeof(cvc::state_exec::evaluator) > 0);
  static_assert(sizeof(cvc::state_exec::exec_coordinator) > 0);
  EXPECT_EQ(cvc::state_exec::utf8::count(std::string_view("abc")), 3u);
}
