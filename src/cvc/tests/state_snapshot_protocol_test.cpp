/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_transport_ipc.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <thread>

// ---------------------------------------------------------------
// Snapshot protocol tests over IPC transport.
// ---------------------------------------------------------------

namespace {

std::string make_socket_path(const std::string &label) {
  return (std::filesystem::temp_directory_path() / ("cvc_snap_test_" + label + ".sock")).string();
}

} // namespace

class StateSnapshotProtocolTest : public ::testing::Test {
protected:
  void TearDown() override {
    // Clean up socket files.
    for (const auto &p : sockets)
      std::filesystem::remove(p);
  }
  std::vector<std::string> sockets;
};

TEST_F(StateSnapshotProtocolTest, SnapshotFromLocalShard) {
  cvc::app ctx;
  cvc::state_cluster_shard shard(ctx, "clust", "node_A");
  shard.attach();

  // Write some state.
  cvc::state::instance(ctx)("a.b").value(std::string("hello"));
  cvc::state::instance(ctx)("a.c").value(std::string("world"));

  auto snap = shard.snapshot();
  EXPECT_GE(snap.size(), 2u);

  bool found_b = false, found_c = false;
  for (const auto &e : snap) {
    if (e.path == "a.b" && e.string_value == "hello")
      found_b = true;
    if (e.path == "a.c" && e.string_value == "world")
      found_c = true;
  }
  EXPECT_TRUE(found_b) << "a.b not in snapshot";
  EXPECT_TRUE(found_c) << "a.c not in snapshot";

  shard.detach();
}

TEST_F(StateSnapshotProtocolTest, SnapshotWithPrefixFilter) {
  cvc::app ctx;
  cvc::state_cluster_shard shard(ctx, "clust", "node_A");
  shard.attach();

  cvc::state::instance(ctx)("foo.x").value(std::string("1"));
  cvc::state::instance(ctx)("bar.y").value(std::string("2"));

  auto snap = shard.snapshot("foo");
  bool found_foo = false, found_bar = false;
  for (const auto &e : snap) {
    if (e.path.find("foo") == 0)
      found_foo = true;
    if (e.path.find("bar") == 0)
      found_bar = true;
  }
  EXPECT_TRUE(found_foo);
  EXPECT_FALSE(found_bar);

  shard.detach();
}

TEST_F(StateSnapshotProtocolTest, IpcSnapshotRequestResponse) {
  std::string sock = make_socket_path("snap_ipc");
  sockets.push_back(sock);

  cvc::app ctx_a, ctx_b;

  // Server: node_A with data.
  cvc::state_transport_ipc server;
  cvc::state_cluster_shard shard_a(ctx_a, "clust", "node_A");
  shard_a.attach();
  cvc::state::instance(ctx_a)("data.x").value(std::string("42"));
  cvc::state::instance(ctx_a)("data.y").value(std::string("99"));
  server.register_shard(&shard_a);
  server.start(sock, "node_A", "clust");

  // Client: node_B, empty. Must call start() so _running is set.
  std::string sock_b = make_socket_path("snap_ipc_b");
  sockets.push_back(sock_b);
  cvc::state_transport_ipc client;
  cvc::state_cluster_shard shard_b(ctx_b, "clust", "node_B");
  shard_b.attach();
  client.register_shard(&shard_b);
  client.start(sock_b, "node_B", "clust");
  ASSERT_TRUE(client.connect_to_peer(sock));

  // Wait for handshake + reader thread to be established.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Verify connections are up.
  ASSERT_GE(client.connection_count(), 1u) << "client has no connections";
  ASSERT_GE(server.connection_count(), 1u) << "server has no connections";

  // Request snapshot.
  std::vector<cvc::state_transport::snapshot_entry> entries;
  bool got_snapshot = client.request_snapshot(
      "clust", "", [&](const std::vector<cvc::state_transport::snapshot_entry> &e, bool /*final*/) {
        entries = e;
      });

  EXPECT_TRUE(got_snapshot);
  EXPECT_GE(entries.size(), 2u);

  bool found_x = false, found_y = false;
  for (const auto &e : entries) {
    if (e.path == "data.x" && e.string_value == "42")
      found_x = true;
    if (e.path == "data.y" && e.string_value == "99")
      found_y = true;
  }
  EXPECT_TRUE(found_x);
  EXPECT_TRUE(found_y);

  client.stop();
  server.stop();
  shard_a.detach();
  shard_b.detach();
}
