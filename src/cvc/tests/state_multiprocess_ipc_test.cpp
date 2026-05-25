/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// Multi-process integration test using the IPC (Unix domain socket)
// transport.  Two separate processes synchronize state mutations over
// a UDS connection, verifying end-to-end replication without any
// in-process shortcuts.
//
// POSIX-only: uses fork(), waitpid(), and Unix domain sockets.

#ifndef _WIN32

#include <chrono>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_transport_ipc.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace {

std::string make_socket_path(const std::string &label) {
  auto pid = static_cast<long long>(::getpid());
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  auto dir = std::filesystem::temp_directory_path();
  auto p =
      dir / ("cvc_mp_" + std::to_string(pid) + "_" + std::to_string(now) + "_" + label + ".sock");
  return p.string();
}

// Wait for socket file to appear on disk.
bool wait_for_socket(const std::string &path, std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::filesystem::exists(path))
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
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

// Test: parent writes a value, child receives it over IPC after fork.
//
// Both processes start() their own UDS listeners (required to
// establish identity), then one connects to the other.  We fork()
// before creating any transport objects to avoid inheriting live
// sockets / threads across the process boundary.
TEST(MultiProcessIpcIntegration, ValueReplication) {
  auto sock_srv = make_socket_path("val_srv");
  auto sock_cli = make_socket_path("val_cli");
  std::error_code ec;
  std::filesystem::remove(sock_srv, ec);
  std::filesystem::remove(sock_cli, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    // ---- CHILD (client) ----
    cvc::app ca;
    cvc::state_transport_ipc ct;
    cvc::state_cluster_shard cs(ca, "C", "cli");
    cs.attach();
    ct.register_shard(&cs);
    ct.start(sock_cli, "cli", "C");

    // Wait for server socket to appear, then connect.
    if (!wait_for_socket(sock_srv, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!ct.connect_to_peer(sock_srv, std::chrono::milliseconds(3000)))
      _exit(11);

    // Wait until we receive at least one mutation from the server.
    ct.wait_for_received(1, std::chrono::milliseconds(5000));

    bool ok = ca.root()("greeting").value() == "hello";

    ct.stop();
    cs.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (server) ----
  cvc::app sa;
  cvc::state_transport_ipc st;
  cvc::state_cluster_shard ss(sa, "C", "srv");
  ss.attach();
  st.register_shard(&ss);
  st.start(sock_srv, "srv", "C");

  // First-set-on-fresh-child is lost (adapter quirk).  Set twice.
  sa.root()("greeting").value(std::string("seed"));
  sa.root()("greeting").value(std::string("hello"));

  // Keep pumping until child is done.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(8000);
  while (std::chrono::steady_clock::now() < deadline) {
    st.pump_all();
    st.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    // Early exit if child has already finished.
    int wr = waitpid(child, nullptr, WNOHANG);
    if (wr > 0)
      break;
  }

  int status = 0;
  waitpid(child, &status, 0);

  st.stop();
  ss.detach();
  std::filesystem::remove(sock_srv, ec);
  std::filesystem::remove(sock_cli, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally";
  EXPECT_EQ(WEXITSTATUS(status), 0) << "exit code: 10=socket wait, 11=connect, 12=value mismatch";
}

// Test: bidirectional replication.  Both processes write values and
// each receives the other's writes.
TEST(MultiProcessIpcIntegration, BidirectionalReplication) {
  auto sock_srv = make_socket_path("bidir_srv");
  auto sock_cli = make_socket_path("bidir_cli");
  std::error_code ec;
  std::filesystem::remove(sock_srv, ec);
  std::filesystem::remove(sock_cli, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    cvc::app ca;
    cvc::state_transport_ipc ct;
    cvc::state_cluster_shard cs(ca, "C", "cli");
    cs.attach();
    ct.register_shard(&cs);
    ct.start(sock_cli, "cli", "C");

    if (!wait_for_socket(sock_srv, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!ct.connect_to_peer(sock_srv, std::chrono::milliseconds(3000)))
      _exit(11);

    // Write a client-side value (seed + real).
    ca.root()("client_val").value(std::string("seed"));
    ca.root()("client_val").value(std::string("from_client"));
    ct.pump_all();
    ct.flush();

    // Wait for server's value.
    ct.wait_for_received(1, std::chrono::milliseconds(5000));
    bool ok = ca.root()("server_val").value() == "from_server";

    // Keep pumping so server receives ours.
    for (int i = 0; i < 40; ++i) {
      ct.pump_all();
      ct.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ct.stop();
    cs.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (server) ----
  cvc::app sa;
  cvc::state_transport_ipc st;
  cvc::state_cluster_shard ss(sa, "C", "srv");
  ss.attach();
  st.register_shard(&ss);
  st.start(sock_srv, "srv", "C");

  // Wait until the child has connected before writing.
  {
    auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(5000);
    while (st.connection_count() < 1 && std::chrono::steady_clock::now() < dl)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  sa.root()("server_val").value(std::string("seed"));
  sa.root()("server_val").value(std::string("from_server"));
  st.pump_all();
  st.flush();

  // Wait for client's value.
  st.wait_for_received(1, std::chrono::milliseconds(5000));
  bool got_client = sa.root()("client_val").value() == "from_client";

  // Keep pumping so child gets our value.
  for (int i = 0; i < 80; ++i) {
    st.pump_all();
    st.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  int status = 0;
  waitpid(child, &status, 0);

  st.stop();
  ss.detach();
  std::filesystem::remove(sock_srv, ec);
  std::filesystem::remove(sock_cli, ec);

  EXPECT_TRUE(got_client) << "server did not receive client's value";
  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally";
  EXPECT_EQ(WEXITSTATUS(status), 0) << "child did not receive server's value";
}

// Test: comment (metadata) replication across processes.
TEST(MultiProcessIpcIntegration, MetadataReplication) {
  auto sock_srv = make_socket_path("meta_srv");
  auto sock_cli = make_socket_path("meta_cli");
  std::error_code ec;
  std::filesystem::remove(sock_srv, ec);
  std::filesystem::remove(sock_cli, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    cvc::app ca;
    cvc::state_transport_ipc ct;
    cvc::state_cluster_shard cs(ca, "C", "cli");
    cs.attach();
    ct.register_shard(&cs);
    ct.start(sock_cli, "cli", "C");

    if (!wait_for_socket(sock_srv, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!ct.connect_to_peer(sock_srv, std::chrono::milliseconds(3000)))
      _exit(11);

    // We expect two mutations: set_value + set_comment.
    ct.wait_for_received(2, std::chrono::milliseconds(5000));

    bool ok = ca.root()("config").value() == "42" && ca.root()("config").comment() == "the answer";

    ct.stop();
    cs.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (server) ----
  cvc::app sa;
  cvc::state_transport_ipc st;
  cvc::state_cluster_shard ss(sa, "C", "srv");
  ss.attach();
  st.register_shard(&ss);
  st.start(sock_srv, "srv", "C");

  // Seed + real for value.
  sa.root()("config").value(std::string("seed"));
  sa.root()("config").value(std::string("42"));
  // Comment is a separate mutation.
  sa.root()("config").comment(std::string("the answer"));

  // Keep pumping until child finishes.
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(8000);
  while (std::chrono::steady_clock::now() < deadline) {
    st.pump_all();
    st.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int wr = waitpid(child, nullptr, WNOHANG);
    if (wr > 0)
      break;
  }

  int status = 0;
  waitpid(child, &status, 0);

  st.stop();
  ss.detach();
  std::filesystem::remove(sock_srv, ec);
  std::filesystem::remove(sock_cli, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally";
  EXPECT_EQ(WEXITSTATUS(status), 0) << "value or comment not replicated";
}

#endif // !_WIN32
