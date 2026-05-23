/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <chrono>
#include <cstdlib>
#include <cvc/app.h>
#include <cvc/state.h>
#include <cvc/state_cluster_shard.h>
#include <cvc/state_transport_ipc.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <thread>

// ---------------------------------------------------------------
// Reconnect / resilience tests for distributed-state transports.
// Uses the IPC (Unix domain socket) transport since it is the
// simplest multi-process-capable transport and is POSIX-only.
// ---------------------------------------------------------------

namespace {

std::string make_socket_path(const std::string &label) {
  auto pid = static_cast<long long>(::getpid());
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  auto dir = std::filesystem::temp_directory_path();
  return (dir /
          ("cvc_res_" + std::to_string(pid) + "_" + std::to_string(now) + "_" + label + ".sock"))
      .string();
}

bool wait_connected(cvc::state_transport_ipc &a, cvc::state_transport_ipc &b,
                    std::chrono::milliseconds to) {
  auto deadline = std::chrono::steady_clock::now() + to;
  while (std::chrono::steady_clock::now() < deadline) {
    if (a.connection_count() >= 1 && b.connection_count() >= 1)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

} // namespace

// ---------------------------------------------------------------
// Test: data replicates, one side stops, restarts, reconnects,
//       and subsequent writes still replicate.
// ---------------------------------------------------------------
TEST(StateReconnectResilienceTest, StopRestartReconnect) {
  cvc::app aA, aB;

  auto pathA = make_socket_path("A_recon");
  auto pathB = make_socket_path("B_recon");

  // -- Phase 1: establish and replicate.
  auto tA = std::make_unique<cvc::state_transport_ipc>();
  cvc::state_transport_ipc tB;
  tA->start(pathA, "A", "C");
  tB.start(pathB, "B", "C");
  ASSERT_TRUE(tA->connect_to_peer(pathB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(*tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA->register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("k").value(std::string("seed"));
  cvc::state::instance(aA)("k").value(std::string("v1"));
  tA->pump_all();
  tA->flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v1");

  // -- Phase 2: stop A's transport (simulate crash / restart).
  tA->unregister_shard(&sA);
  tA->stop();
  tA.reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // -- Phase 3: restart A with a new transport, reconnect to B.
  tA = std::make_unique<cvc::state_transport_ipc>();
  tA->start(pathA, "A", "C");
  ASSERT_TRUE(tA->connect_to_peer(pathB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(*tA, tB, std::chrono::milliseconds(2000)));
  tA->register_shard(&sA);

  // -- Phase 4: replicate after reconnect.
  cvc::state::instance(aA)("k").value(std::string("v2_after_reconnect"));
  tA->pump_all();
  tA->flush();
  tB.wait_for_received(2, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("k").value(), "v2_after_reconnect");

  tA->stop();
  tB.stop();
}

// ---------------------------------------------------------------
// Test: peer goes away, local side survives (no crash/hang).
// ---------------------------------------------------------------
TEST(StateReconnectResilienceTest, PeerDisconnectNoHang) {
  cvc::app aA, aB;

  auto pathA = make_socket_path("A_disc");
  auto pathB = make_socket_path("B_disc");

  cvc::state_transport_ipc tA, tB;
  tA.start(pathA, "A", "C");
  tB.start(pathB, "B", "C");
  ASSERT_TRUE(tA.connect_to_peer(pathB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));

  cvc::state_cluster_shard sA(aA, "C", "A");
  cvc::state_cluster_shard sB(aB, "C", "B");
  sA.attach();
  sB.attach();
  tA.register_shard(&sA);
  tB.register_shard(&sB);

  cvc::state::instance(aA)("x").value(std::string("seed"));
  cvc::state::instance(aA)("x").value(std::string("v1"));
  tA.pump_all();
  tA.flush();
  tB.wait_for_received(1, std::chrono::milliseconds(2000));
  EXPECT_EQ(cvc::state::instance(aB)("x").value(), "v1");

  // B goes away.
  tB.stop();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // A can still pump without crashing.
  cvc::state::instance(aA)("x").value(std::string("v2"));
  EXPECT_NO_THROW(tA.pump_all());
  // Flush may fail silently since B is gone — just must not hang.
  EXPECT_NO_THROW(tA.flush());

  tA.stop();
}

// ---------------------------------------------------------------
// Test: session-level status() reflects peer count correctly.
// ---------------------------------------------------------------
TEST(StateReconnectResilienceTest, ConnectionCountTracking) {
  auto pathA = make_socket_path("A_cnt");
  auto pathB = make_socket_path("B_cnt");

  cvc::state_transport_ipc tA, tB;
  EXPECT_EQ(tA.connection_count(), 0u);

  tA.start(pathA, "A", "C");
  tB.start(pathB, "B", "C");
  EXPECT_EQ(tA.connection_count(), 0u);

  ASSERT_TRUE(tA.connect_to_peer(pathB, std::chrono::milliseconds(2000)));
  ASSERT_TRUE(wait_connected(tA, tB, std::chrono::milliseconds(2000)));
  EXPECT_GE(tA.connection_count(), 1u);
  EXPECT_GE(tB.connection_count(), 1u);

  tB.stop();
  // Give reader threads time to notice the disconnect.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  // A's connection_count may drop; at minimum we don't crash.
  EXPECT_NO_THROW(tA.connection_count());

  tA.stop();
}
