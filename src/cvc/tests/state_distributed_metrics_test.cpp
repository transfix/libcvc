/*
  Copyright 2026 The University of Texas at Austin
  Phase 5 — state_distributed_metrics tests.
*/

#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_distributed_metrics.h>
#include <cvc/state_transport_inproc.h>
#include <gtest/gtest.h>

using cvc::app;
using cvc::state;
using cvc::state_cluster_shard;
using cvc::state_distributed_metrics;
using cvc::state_mutation;
using cvc::state_transport_inproc;

TEST(StateDistributedMetrics, WriteU64Persists) {
  app ctx;
  state_distributed_metrics::write_u64(ctx, "cluster_x", "alpha", 42);
  std::string v = state::instance(ctx)("__system.distributed.cluster_x.alpha").value();
  EXPECT_EQ("42", v);
}

TEST(StateDistributedMetrics, ClusterIsolation) {
  app ctx;
  state_distributed_metrics::write_u64(ctx, "c1", "k", 1);
  state_distributed_metrics::write_u64(ctx, "c2", "k", 2);
  EXPECT_EQ("1", state::instance(ctx)("__system.distributed.c1.k").value());
  EXPECT_EQ("2", state::instance(ctx)("__system.distributed.c2.k").value());
}

TEST(StateDistributedMetrics, PublishShardWritesAllKeys) {
  app sender, receiver;
  auto src = std::make_unique<state_cluster_shard>(sender, "metrics_cluster", "src_node", "data");
  auto dst = std::make_unique<state_cluster_shard>(receiver, "metrics_cluster", "dst_node", "data");
  src->attach();
  dst->attach();
  dst->set_enforce_authority(false);

  // Force a couple of remote ingests so counters are non-zero.
  state_mutation m1;
  m1.cluster_id = "metrics_cluster";
  m1.origin_node_id = "src_node";
  m1.path = "data.x";
  m1.sequence = 1;
  m1.string_value = "hello";
  dst->ingest_remote(m1);

  // Same mutation again -> duplicate counter.
  dst->ingest_remote(m1);

  app metrics_ctx;
  std::size_t n = state_distributed_metrics::publish_shard(metrics_ctx, *dst);
  EXPECT_EQ(5u, n);

  EXPECT_EQ(
      "1",
      state::instance(metrics_ctx)("__system.distributed.metrics_cluster.remote.applied").value());
  EXPECT_EQ("1",
            state::instance(metrics_ctx)("__system.distributed.metrics_cluster.remote.duplicates")
                .value());

  src->detach();
  dst->detach();
}
