/// @file state_exec_multiprocess_test.cpp
/// @brief Cross-process integration tests for state_exec.
///
/// Spawns real OS processes via fork() and verifies that state_exec
/// programs can communicate across the host boundary using both
/// IPC (Unix domain socket) and gRPC transports.
///
/// POSIX-only: uses fork(), waitpid(), Unix domain sockets.

#ifndef _WIN32

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/intrinsics.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_exec/types.h>
#include <cvc/core/state_message.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_transport_ipc.h>
#include <filesystem>
#include <gtest/gtest.h>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#ifdef CVC_ENABLE_GRPC
#include <cvc/core/state_transport_grpc.h>
#endif

using namespace cvc::state_exec;

// ===========================================================================
// Helpers
// ===========================================================================

namespace {

std::string make_socket_path(const std::string &label) {
  auto pid = static_cast<long long>(::getpid());
  auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  auto dir = std::filesystem::temp_directory_path();
  auto p = dir / ("cvc_exec_mp_" + std::to_string(pid) + "_" + std::to_string(now) + "_" + label +
                  ".sock");
  return p.string();
}

bool wait_for_socket(const std::string &path, std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (std::filesystem::exists(path))
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

/// Set up an intrinsics_context and register intrinsics into the environment.
/// The scheduler must already have execute() capability. Returns the env
/// that should be passed to scheduler.execute() via execute_options.env.
struct exec_env {
  intrinsics_context ictx;
  process_ptr host_proc;
  environment_ptr env;
};

exec_env make_exec_env(scheduler &sched, cvc::state &root) {
  exec_env e;
  e.host_proc = make_process();
  e.host_proc->pid = 0;
  e.host_proc->status = process_status::ready;

  e.ictx.sched = &sched;
  e.ictx.root = &root;
  e.ictx.proc = e.host_proc;
  e.ictx.pid = 0;
  e.ictx.uid = "test-user";
  e.ictx.cluster_id = "test-cluster";
  e.ictx.node_id = "test-node";

  e.env = builtins::make_default_environment();
  register_intrinsics(e.env, &e.ictx);
  return e;
}

/// Pump transport and step scheduler in an interleaved loop until
/// either the scheduler has no runnable processes or timeout expires.
template <typename Transport>
void pump_and_run(Transport &transport, scheduler &sched, std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    transport.pump_all();
    transport.flush();
    if (sched.has_runnable())
      sched.step();
    else
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

/// Pump transport and step scheduler until all processes terminate
/// or timeout. Specifically designed for processes that may be
/// waiting on state values from a remote peer — keeps pumping even
/// when no process is runnable if some processes are still waiting.
template <typename Transport>
void pump_until_done(Transport &transport, scheduler &sched, std::chrono::milliseconds timeout) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    transport.pump_all();
    transport.flush();
    if (sched.has_runnable() && sched.step() > 0) {
      continue; // Made progress, skip sleep.
    }
    // All done? (total_processes == terminated + killed means no live processes)
    auto stats = sched.get_stats();
    if (stats.total_processes == stats.terminated + stats.killed)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

} // namespace

// ===========================================================================
// IPC: State tree replication between state_exec programs
// ===========================================================================

// Process A (parent) runs a DSL program that writes values to the
// state tree. Process B (child) runs a DSL program that polls for
// the replicated values and reads them.
TEST(StateExecMultiprocessIpc, StateTreeReplication) {
  auto sock_a = make_socket_path("exec_st_a");
  auto sock_b = make_socket_path("exec_st_b");
  std::error_code ec;
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    // ---- CHILD (Process B): consumer ----
    cvc::app app_b;
    cvc::state_transport_ipc tr_b;
    cvc::state_cluster_shard shard_b(app_b, "C", "B");
    shard_b.attach();
    tr_b.register_shard(&shard_b);
    tr_b.start(sock_b, "B", "C");

    if (!wait_for_socket(sock_a, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!tr_b.connect_to_peer(sock_a, std::chrono::milliseconds(3000)))
      _exit(11);

    // Wait for replicated values from Process A.
    tr_b.wait_for_received(3, std::chrono::milliseconds(8000));

    // Set up scheduler and run a DSL program that reads the replicated values.
    scheduler sched_b;
    auto &root_b = cvc::state::instance(app_b);
    sched_b.set_watch_root(&root_b);
    auto e = make_exec_env(sched_b, root_b);

    execute_options opts;
    opts.name = "reader";
    opts.env = e.env;
    int pid = sched_b.execute(std::string(R"(
      (begin
        (set v1 (state-get "shared.x"))
        (set v2 (state-get "shared.y"))
        (set v3 (state-get "shared.z"))
        (str-concat v1 ":" v2 ":" v3))
    )"),
                              opts);

    sched_b.run();
    auto result = sched_b.get_result(pid);

    bool ok = result.has_value() && std::holds_alternative<std::string>(result->v) &&
              std::get<std::string>(result->v) == "10:20:30";

    tr_b.stop();
    shard_b.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (Process A): producer ----
  cvc::app app_a;
  cvc::state_transport_ipc tr_a;
  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  shard_a.attach();
  tr_a.register_shard(&shard_a);
  tr_a.start(sock_a, "A", "C");

  // Set up scheduler and run a DSL program that writes values.
  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto e = make_exec_env(sched_a, root_a);

  execute_options opts;
  opts.name = "writer";
  opts.env = e.env;

  // Seed + real writes (adapter quirk: first write on fresh shard may be lost).
  root_a("shared.x").value(std::string("seed"));
  root_a("shared.y").value(std::string("seed"));
  root_a("shared.z").value(std::string("seed"));

  int pid = sched_a.execute(std::string(R"(
    (begin
      (state-set "shared.x" "10")
      (state-set "shared.y" "20")
      (state-set "shared.z" "30")
      "written")
  )"),
                            opts);

  sched_a.run();
  auto result = sched_a.get_result(pid);
  ASSERT_TRUE(result.has_value());

  // Pump until child is done.
  int status = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(10000);
  while (std::chrono::steady_clock::now() < deadline) {
    tr_a.pump_all();
    tr_a.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int wr = waitpid(child, &status, WNOHANG);
    if (wr > 0)
      break;
  }

  if (!WIFEXITED(status)) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
  }

  tr_a.stop();
  shard_a.detach();
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally (killed after timeout)";
  EXPECT_EQ(WEXITSTATUS(status), 0) << "exit codes: 10=socket, 11=connect, 12=value mismatch";
}

// ===========================================================================
// IPC: OOB message delivery between state_exec programs
// ===========================================================================

// Process A runs a DSL program that sends an OOB message via msg-send.
// Process B runs a DSL program that receives it via msg-recv. A message
// bus subscriber on Process B bridges the OOB message to the scheduler.
TEST(StateExecMultiprocessIpc, OobMessageDelivery) {
  auto sock_a = make_socket_path("exec_oob_a");
  auto sock_b = make_socket_path("exec_oob_b");
  std::error_code ec;
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    // ---- CHILD (Process B): receiver ----
    cvc::app app_b;
    cvc::state_transport_ipc tr_b;
    cvc::state_cluster_shard shard_b(app_b, "C", "B");
    shard_b.attach();
    tr_b.register_shard(&shard_b);
    tr_b.start(sock_b, "B", "C");

    if (!wait_for_socket(sock_a, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!tr_b.connect_to_peer(sock_a, std::chrono::milliseconds(3000)))
      _exit(11);

    scheduler sched_b;
    auto &root_b = cvc::state::instance(app_b);
    sched_b.set_watch_root(&root_b);
    auto e = make_exec_env(sched_b, root_b);

    // Thread-safe bridge: subscriber queues from reader thread,
    // main thread drains and delivers.
    std::mutex msg_mu;
    std::vector<std::pair<std::string, value_t>> msg_queue;

    shard_b.message_bus().subscribe("oob.chan", [&](const cvc::state_message &m) {
      std::vector<std::pair<std::string, value_t>> entries;
      entries.emplace_back("status", value_t(std::string("delivered")));
      entries.emplace_back("path", value_t(m.path));
      entries.emplace_back("payload", value_t(m.string_value));
      auto dict = make_dict(std::move(entries));
      std::lock_guard<std::mutex> lk(msg_mu);
      msg_queue.emplace_back(m.path, dict);
    });

    // Start the receiver DSL program.
    execute_options opts;
    opts.name = "receiver";
    opts.env = e.env;
    int pid = sched_b.execute(std::string(R"(
      (begin
        (set m (msg-recv "oob.chan"))
        (get-attr m "payload"))
    )"),
                              opts);

    // Step the scheduler once so the process reaches msg-recv and suspends.
    for (int i = 0; i < 200 && sched_b.has_runnable(); ++i) {
      if (sched_b.step() == 0)
        break;
    }

    // Pump, deliver queued messages from main thread, and step.
    {
      auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(25000);
      while (std::chrono::steady_clock::now() < dl) {
        tr_b.pump_all();
        tr_b.flush();
        {
          std::lock_guard<std::mutex> lk(msg_mu);
          for (auto &[path, dict] : msg_queue)
            sched_b.deliver_to_receivers(path, dict);
          msg_queue.clear();
        }
        while (sched_b.has_runnable()) {
          if (sched_b.step() == 0)
            break;
        }
        auto stats = sched_b.get_stats();
        if (stats.total_processes == stats.terminated + stats.killed)
          break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }

    auto result = sched_b.get_result(pid);
    bool ok = result.has_value() && std::holds_alternative<std::string>(result->v) &&
              std::get<std::string>(result->v) == "hello-from-A";

    tr_b.stop();
    shard_b.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (Process A): sender ----
  cvc::app app_a;
  cvc::state_transport_ipc tr_a;
  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  shard_a.attach();
  shard_a.set_transport(&tr_a);
  tr_a.register_shard(&shard_a);
  tr_a.start(sock_a, "A", "C");

  // Wait for child's socket.
  ASSERT_TRUE(wait_for_socket(sock_b, std::chrono::milliseconds(5000)));
  ASSERT_TRUE(tr_a.connect_to_peer(sock_b, std::chrono::milliseconds(3000)));

  // Small delay so the child has time to set up its receiver.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto e = make_exec_env(sched_a, root_a);

  execute_options opts;
  opts.name = "sender";
  opts.env = e.env;
  int pid = sched_a.execute(std::string(R"(
    (begin
      (msg-send "oob.chan" "hello-from-A")
      "sent")
  )"),
                            opts);

  // Run sender to completion.
  sched_a.run();
  auto result = sched_a.get_result(pid);
  ASSERT_TRUE(result.has_value());

  // Pump to deliver the OOB message to the child.
  int status = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(30000);
  while (std::chrono::steady_clock::now() < deadline) {
    tr_a.pump_all();
    tr_a.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int wr = waitpid(child, &status, WNOHANG);
    if (wr > 0)
      break;
  }

  if (!WIFEXITED(status)) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
  }

  tr_a.stop();
  shard_a.detach();
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally (killed after timeout)";
  EXPECT_EQ(WEXITSTATUS(status), 0)
      << "exit codes: 10=socket, 11=connect, 12=msg-recv payload mismatch";
}

// ===========================================================================
// IPC: Bidirectional producer/consumer with generators
// ===========================================================================

// Process A runs a DSL producer that writes numbered items via state-set.
// Process B runs a DSL consumer with a generator that reads and transforms
// them. Both sides replicate their results back via the state tree.
TEST(StateExecMultiprocessIpc, BidirectionalProducerConsumerGenerators) {
  auto sock_a = make_socket_path("exec_gen_a");
  auto sock_b = make_socket_path("exec_gen_b");
  std::error_code ec;
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    // ---- CHILD (Process B): consumer with generator ----
    cvc::app app_b;
    cvc::state_transport_ipc tr_b;
    cvc::state_cluster_shard shard_b(app_b, "C", "B");
    shard_b.attach();
    tr_b.register_shard(&shard_b);
    tr_b.start(sock_b, "B", "C");

    if (!wait_for_socket(sock_a, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!tr_b.connect_to_peer(sock_a, std::chrono::milliseconds(3000)))
      _exit(11);

    // Wait for the producer's "ready" signal.
    tr_b.wait_for_received(7, std::chrono::milliseconds(8000));

    scheduler sched_b;
    auto &root_b = cvc::state::instance(app_b);
    sched_b.set_watch_root(&root_b);
    auto e = make_exec_env(sched_b, root_b);

    // Seed result path so shard can replicate it.
    root_b("result.consumer").value(std::string("seed"));

    execute_options opts;
    opts.name = "consumer-gen";
    opts.env = e.env;
    // Consumer uses a generator to iterate over replicated items,
    // counts them and concatenates the string values.
    int pid = sched_b.execute(std::string(R"(
      (begin
        (set reader (generator (lambda ()
          (for i (range 5)
            (yield (state-get (str-concat "items.v" (str i))))))))
        (set count 0)
        (set concat "")
        (for val reader
          (begin
            (set count (+ count 1))
            (set concat (str-concat concat val ":"))))
        (state-set "result.consumer" (str-concat (str count) "|" concat))
        count)
    )"),
                              opts);

    sched_b.run();
    auto result = sched_b.get_result(pid);

    // 5 items read → count = 5
    bool ok = result.has_value() && std::holds_alternative<int64_t>(result->v) &&
              std::get<int64_t>(result->v) == 5;

    // Pump our result back to Process A.
    for (int i = 0; i < 40; ++i) {
      tr_b.pump_all();
      tr_b.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    tr_b.stop();
    shard_b.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (Process A): producer ----
  cvc::app app_a;
  cvc::state_transport_ipc tr_a;
  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  shard_a.attach();
  tr_a.register_shard(&shard_a);
  tr_a.start(sock_a, "A", "C");

  // Connect to child so we can receive B→A state replication.
  ASSERT_TRUE(wait_for_socket(sock_b, std::chrono::milliseconds(5000)));
  ASSERT_TRUE(tr_a.connect_to_peer(sock_b, std::chrono::milliseconds(3000)));

  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto e = make_exec_env(sched_a, root_a);

  // Seed the state tree paths (including the result path for B→A replication).
  for (int i = 0; i < 5; ++i)
    root_a("items.v" + std::to_string(i)).value(std::string("seed"));
  root_a("result.consumer").value(std::string("seed"));

  execute_options opts;
  opts.name = "producer-gen";
  opts.env = e.env;
  // Producer uses a generator that yields string values "v0".."v4".
  int pid = sched_a.execute(std::string(R"(
    (begin
      (set val-gen (generator (lambda ()
        (for i (range 5)
          (yield (str-concat "v" (str i)))))))
      (for val val-gen
        (begin
          (set idx (str-concat "items." val))
          (state-set idx val)))
      (state-set "items.ready" "true")
      "produced")
  )"),
                            opts);

  sched_a.run();
  auto result = sched_a.get_result(pid);
  ASSERT_TRUE(result.has_value());

  // Pump until child finishes, then check the consumer's replicated result.
  int status = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(12000);
  while (std::chrono::steady_clock::now() < deadline) {
    tr_a.pump_all();
    tr_a.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int wr = waitpid(child, &status, WNOHANG);
    if (wr > 0)
      break;
  }

  if (!WIFEXITED(status)) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
  }

  // Check that the consumer's result was replicated back.
  tr_a.wait_for_received(1, std::chrono::milliseconds(3000));
  std::string consumer_result = root_a("result.consumer").value();

  tr_a.stop();
  shard_a.detach();
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally (killed after timeout)";
  EXPECT_EQ(WEXITSTATUS(status), 0)
      << "exit codes: 10=socket, 11=connect, 12=generator count mismatch";
  // Consumer should have read 5 items
  EXPECT_TRUE(consumer_result.find("5|") == 0) << "consumer result: " << consumer_result;
}

// ===========================================================================
// IPC: Multiple producers, single consumer across processes
// ===========================================================================

// Process A spawns 3 DSL producer processes that each write items
// to the state tree. Process B reads all items and aggregates.
TEST(StateExecMultiprocessIpc, MultiProducerSingleConsumer) {
  auto sock_a = make_socket_path("exec_mp_a");
  auto sock_b = make_socket_path("exec_mp_b");
  std::error_code ec;
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    // ---- CHILD (Process B): consumer aggregator ----
    cvc::app app_b;
    cvc::state_transport_ipc tr_b;
    cvc::state_cluster_shard shard_b(app_b, "C", "B");
    shard_b.attach();
    tr_b.register_shard(&shard_b);
    tr_b.start(sock_b, "B", "C");

    if (!wait_for_socket(sock_a, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!tr_b.connect_to_peer(sock_a, std::chrono::milliseconds(3000)))
      _exit(11);

    // Wait for all producer data + ready signal.
    tr_b.wait_for_received(10, std::chrono::milliseconds(10000));

    scheduler sched_b;
    auto &root_b = cvc::state::instance(app_b);
    sched_b.set_watch_root(&root_b);
    auto e = make_exec_env(sched_b, root_b);

    execute_options opts;
    opts.name = "aggregator";
    opts.env = e.env;
    // Count all 9 items (3 producers × 3 items each).
    int pid = sched_b.execute(std::string(R"(
      (begin
        (set count 0)
        (for p (range 3)
          (for i (range 3)
            (begin
              (set key (str-concat "producer." (str p) ".item." (str i)))
              (if (state-exists key)
                (set count (+ count 1))
                0))))
        count)
    )"),
                              opts);

    sched_b.run();
    auto result = sched_b.get_result(pid);
    // 9 items total (3 producers × 3 items)
    bool ok = result.has_value() && std::holds_alternative<int64_t>(result->v) &&
              std::get<int64_t>(result->v) == 9;

    tr_b.stop();
    shard_b.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (Process A): 3 producer processes ----
  cvc::app app_a;
  cvc::state_transport_ipc tr_a;
  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  shard_a.attach();
  tr_a.register_shard(&shard_a);
  tr_a.start(sock_a, "A", "C");

  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto e = make_exec_env(sched_a, root_a);

  // Seed all paths.
  for (int p = 0; p < 3; ++p)
    for (int i = 0; i < 3; ++i)
      root_a("producer." + std::to_string(p) + ".item." + std::to_string(i))
          .value(std::string("seed"));

  // Spawn 3 producer DSL processes. Each writes items with a tag.
  for (int p = 0; p < 3; ++p) {
    execute_options opts;
    opts.name = "producer-" + std::to_string(p);
    opts.env = e.env;
    std::string ps = std::to_string(p);
    std::string script = "(begin"
                         "  (for i (range 3)"
                         "    (state-set (str-concat \"producer." +
                         ps +
                         ".item.\" (str i))"
                         "              (str-concat \"" +
                         ps +
                         "-\" (str i))))"
                         "  \"done\")";
    sched_a.execute(script, opts);
  }

  sched_a.run();

  // Signal that all producers are done.
  root_a("producers.ready").value(std::string("true"));

  // Pump until child finishes.
  int status = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(12000);
  while (std::chrono::steady_clock::now() < deadline) {
    tr_a.pump_all();
    tr_a.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int wr = waitpid(child, &status, WNOHANG);
    if (wr > 0)
      break;
  }

  if (!WIFEXITED(status)) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
  }

  tr_a.stop();
  shard_a.detach();
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally (killed after timeout)";
  EXPECT_EQ(WEXITSTATUS(status), 0)
      << "exit codes: 10=socket, 11=connect, 12=aggregation mismatch (expected 9)";
}

// ===========================================================================
// IPC: OOB message pipeline — producer/consumer with generators
// ===========================================================================

// Process A sends multiple OOB messages in a loop.
// Process B uses msg-recv in a generator to receive and transform them.
TEST(StateExecMultiprocessIpc, OobMessagePipeline) {
  auto sock_a = make_socket_path("exec_pipe_a");
  auto sock_b = make_socket_path("exec_pipe_b");
  std::error_code ec;
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  pid_t child = fork();
  ASSERT_NE(child, -1) << "fork() failed";

  if (child == 0) {
    // ---- CHILD (Process B): receiver pipeline ----
    cvc::app app_b;
    cvc::state_transport_ipc tr_b;
    cvc::state_cluster_shard shard_b(app_b, "C", "B");
    shard_b.attach();
    tr_b.register_shard(&shard_b);
    tr_b.start(sock_b, "B", "C");

    if (!wait_for_socket(sock_a, std::chrono::milliseconds(5000)))
      _exit(10);
    if (!tr_b.connect_to_peer(sock_a, std::chrono::milliseconds(3000)))
      _exit(11);

    scheduler sched_b;
    auto &root_b = cvc::state::instance(app_b);
    sched_b.set_watch_root(&root_b);
    auto e = make_exec_env(sched_b, root_b);

    // Thread-safe queue: subscriber pushes from reader thread,
    // main thread drains and delivers.
    std::mutex msg_mu;
    std::vector<std::pair<std::string, value_t>> msg_queue;

    shard_b.message_bus().subscribe("pipe.data", [&](const cvc::state_message &m) {
      std::vector<std::pair<std::string, value_t>> entries;
      entries.emplace_back("status", value_t(std::string("delivered")));
      entries.emplace_back("path", value_t(m.path));
      entries.emplace_back("payload", value_t(m.string_value));
      auto dict = make_dict(std::move(entries));
      std::lock_guard<std::mutex> lk(msg_mu);
      msg_queue.emplace_back(m.path, dict);
    });

    execute_options opts;
    opts.name = "pipeline-consumer";
    opts.env = e.env;
    // Receive 4 messages, concatenate payloads.
    int pid = sched_b.execute(std::string(R"(
      (begin
        (set result "")
        (set i 0)
        (while (< i 4)
          (begin
            (set m (msg-recv "pipe.data"))
            (set result (str-concat result (get-attr m "payload") ","))
            (set i (+ i 1))))
        (state-set "pipeline.result" result)
        result)
    )"),
                              opts);

    // Step to get the process started and waiting on msg-recv.
    for (int i = 0; i < 200 && sched_b.has_runnable(); ++i) {
      if (sched_b.step() == 0)
        break;
    }

    // Pump, deliver queued messages from main thread, and step.
    auto dl = std::chrono::steady_clock::now() + std::chrono::milliseconds(25000);
    while (std::chrono::steady_clock::now() < dl) {
      tr_b.pump_all();
      tr_b.flush();
      // Drain queued messages on the main thread.
      {
        std::lock_guard<std::mutex> lk(msg_mu);
        for (auto &[path, dict] : msg_queue)
          sched_b.deliver_to_receivers(path, dict);
        msg_queue.clear();
      }
      // Step repeatedly to consume delivered messages and reach
      // the next msg-recv suspend point (or completion).
      while (sched_b.has_runnable()) {
        if (sched_b.step() == 0)
          break;
      }
      auto stats = sched_b.get_stats();
      if (stats.total_processes == stats.terminated + stats.killed)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    auto result = sched_b.get_result(pid);
    // Payloads: "aa", "bb", "cc", "dd" → concatenated "aa,bb,cc,dd,"
    bool ok = result.has_value() && std::holds_alternative<std::string>(result->v) &&
              std::get<std::string>(result->v) == "aa,bb,cc,dd,";

    // Pump result back.
    for (int i = 0; i < 40; ++i) {
      tr_b.pump_all();
      tr_b.flush();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    tr_b.stop();
    shard_b.detach();
    _exit(ok ? 0 : 12);
  }

  // ---- PARENT (Process A): sender pipeline ----
  cvc::app app_a;
  cvc::state_transport_ipc tr_a;
  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  shard_a.attach();
  shard_a.set_transport(&tr_a);
  tr_a.register_shard(&shard_a);
  tr_a.start(sock_a, "A", "C");

  ASSERT_TRUE(wait_for_socket(sock_b, std::chrono::milliseconds(5000)));
  ASSERT_TRUE(tr_a.connect_to_peer(sock_b, std::chrono::milliseconds(3000)));

  // Wait for child's receiver to be ready.
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto e = make_exec_env(sched_a, root_a);

  // Send 4 messages with string payloads using a generator.
  execute_options opts;
  opts.name = "pipeline-sender";
  opts.env = e.env;
  int pid = sched_a.execute(std::string(R"(
    (begin
      (set payload-gen (generator (lambda ()
        (for tag (list "aa" "bb" "cc" "dd")
          (yield tag)))))
      (for payload payload-gen
        (msg-send "pipe.data" payload))
      "all-sent")
  )"),
                            opts);

  // Run sender synchronously (matches the working OobMessageDelivery pattern).
  sched_a.run();
  auto result = sched_a.get_result(pid);
  ASSERT_TRUE(result.has_value());

  // Keep pumping until child is done.
  int status = 0;
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(30000);
  while (std::chrono::steady_clock::now() < deadline) {
    tr_a.pump_all();
    tr_a.flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    int wr = waitpid(child, &status, WNOHANG);
    if (wr > 0)
      break;
  }

  if (!WIFEXITED(status)) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
  }

  tr_a.stop();
  shard_a.detach();
  std::filesystem::remove(sock_a, ec);
  std::filesystem::remove(sock_b, ec);

  ASSERT_TRUE(WIFEXITED(status)) << "child did not exit normally (killed after timeout)";
  EXPECT_EQ(WEXITSTATUS(status), 0)
      << "exit codes: 10=socket, 11=connect, 12=pipeline concat mismatch";
}

// ===========================================================================
// gRPC: State tree replication between state_exec programs
// ===========================================================================

#ifdef CVC_ENABLE_GRPC

namespace {

bool wait_grpc_connected(cvc::state_transport_grpc &a, cvc::state_transport_grpc &b,
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

// Same as the IPC counterpart but over gRPC (networked transport).
TEST(StateExecMultiprocessGrpc, StateTreeReplication) {
  // gRPC uses TCP so we don't need socket files or fork.
  // We use two in-process endpoints with ephemeral ports.
  cvc::app app_a, app_b;
  cvc::state_transport_grpc tr_a, tr_b;

  tr_a.start("127.0.0.1:0", "A", "C");
  tr_b.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tr_a.connect_to_peer(tr_b.listen_address(), std::chrono::milliseconds(3000)));
  ASSERT_TRUE(wait_grpc_connected(tr_a, tr_b, std::chrono::milliseconds(3000)));

  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  cvc::state_cluster_shard shard_b(app_b, "C", "B");
  shard_a.attach();
  shard_b.attach();
  tr_a.register_shard(&shard_a);
  tr_b.register_shard(&shard_b);

  // Process A: producer scheduler.
  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto ea = make_exec_env(sched_a, root_a);

  // Seed paths.
  root_a("grpc.x").value(std::string("seed"));
  root_a("grpc.y").value(std::string("seed"));

  execute_options opts_a;
  opts_a.name = "grpc-writer";
  opts_a.env = ea.env;
  int pid_a = sched_a.execute(std::string(R"(
    (begin
      (state-set "grpc.x" "alpha")
      (state-set "grpc.y" "beta")
      "written")
  )"),
                              opts_a);
  sched_a.run();
  ASSERT_TRUE(sched_a.get_result(pid_a).has_value());

  // Pump values to B.
  tr_a.pump_all();
  tr_a.flush();
  tr_b.wait_for_received(2, std::chrono::milliseconds(5000));

  // Process B: consumer scheduler.
  scheduler sched_b;
  auto &root_b = cvc::state::instance(app_b);
  sched_b.set_watch_root(&root_b);
  auto eb = make_exec_env(sched_b, root_b);

  execute_options opts_b;
  opts_b.name = "grpc-reader";
  opts_b.env = eb.env;
  int pid_b = sched_b.execute(std::string(R"(
    (begin
      (str-concat (state-get "grpc.x") ":" (state-get "grpc.y")))
  )"),
                              opts_b);
  sched_b.run();

  auto result = sched_b.get_result(pid_b);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "alpha:beta");

  tr_a.stop();
  tr_b.stop();
}

// gRPC: OOB message delivery between schedulers in separate endpoints.
TEST(StateExecMultiprocessGrpc, OobMessageDelivery) {
  cvc::app app_a, app_b;
  cvc::state_transport_grpc tr_a, tr_b;

  tr_a.start("127.0.0.1:0", "A", "C");
  tr_b.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tr_a.connect_to_peer(tr_b.listen_address(), std::chrono::milliseconds(3000)));
  ASSERT_TRUE(wait_grpc_connected(tr_a, tr_b, std::chrono::milliseconds(3000)));

  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  cvc::state_cluster_shard shard_b(app_b, "C", "B");
  shard_a.attach();
  shard_b.attach();
  tr_a.register_shard(&shard_a);
  tr_b.register_shard(&shard_b);

  // Process B: receiver.
  scheduler sched_b;
  auto &root_b = cvc::state::instance(app_b);
  sched_b.set_watch_root(&root_b);
  auto eb = make_exec_env(sched_b, root_b);

  // Bridge OOB messages to scheduler B.
  shard_b.message_bus().subscribe("grpc.chan", [&](const cvc::state_message &m) {
    std::vector<std::pair<std::string, value_t>> entries;
    entries.emplace_back("status", value_t(std::string("delivered")));
    entries.emplace_back("path", value_t(m.path));
    entries.emplace_back("payload", value_t(m.string_value));
    auto dict = make_dict(std::move(entries));
    sched_b.deliver_to_receivers(m.path, dict);
  });

  execute_options opts_b;
  opts_b.name = "grpc-receiver";
  opts_b.env = eb.env;
  int pid_b = sched_b.execute(std::string(R"(
    (begin
      (set m (msg-recv "grpc.chan"))
      (get-attr m "payload"))
  )"),
                              opts_b);

  // Step until the receiver is waiting on msg-recv.
  for (int i = 0; i < 200 && sched_b.has_runnable(); ++i) {
    if (sched_b.step() == 0)
      break;
  }

  // Process A: sender.
  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto ea = make_exec_env(sched_a, root_a);

  execute_options opts_a;
  opts_a.name = "grpc-sender";
  opts_a.env = ea.env;
  int pid_a = sched_a.execute(std::string(R"(
    (begin
      (msg-send "grpc.chan" "hello-grpc")
      "sent")
  )"),
                              opts_a);

  // Run sender, pump OOB to B.
  pump_and_run(tr_a, sched_a, std::chrono::milliseconds(5000));
  ASSERT_TRUE(sched_a.get_result(pid_a).has_value());

  tr_a.pump_all();
  tr_a.flush();

  // Wait for OOB delivery on B's side.
  tr_b.wait_for_received_messages(1, std::chrono::milliseconds(5000));

  // The subscriber fires synchronously inside wait_for_received_messages,
  // which calls deliver_to_receivers, which wakes the process.
  // Now step the receiver to completion.
  pump_until_done(tr_b, sched_b, std::chrono::milliseconds(5000));

  auto result = sched_b.get_result(pid_b);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<std::string>(result->v), "hello-grpc");

  tr_a.stop();
  tr_b.stop();
}

// gRPC: Bidirectional producer/consumer with generators.
TEST(StateExecMultiprocessGrpc, BidirectionalProducerConsumerGenerators) {
  cvc::app app_a, app_b;
  cvc::state_transport_grpc tr_a, tr_b;

  tr_a.start("127.0.0.1:0", "A", "C");
  tr_b.start("127.0.0.1:0", "B", "C");
  ASSERT_TRUE(tr_a.connect_to_peer(tr_b.listen_address(), std::chrono::milliseconds(3000)));
  ASSERT_TRUE(wait_grpc_connected(tr_a, tr_b, std::chrono::milliseconds(3000)));

  cvc::state_cluster_shard shard_a(app_a, "C", "A");
  cvc::state_cluster_shard shard_b(app_b, "C", "B");
  shard_a.attach();
  shard_b.attach();
  tr_a.register_shard(&shard_a);
  tr_b.register_shard(&shard_b);

  // Process A: producer with generator.
  scheduler sched_a;
  auto &root_a = cvc::state::instance(app_a);
  sched_a.set_watch_root(&root_a);
  auto ea = make_exec_env(sched_a, root_a);

  // Seed paths.
  for (int i = 0; i < 5; ++i)
    root_a("data.v" + std::to_string(i)).value(std::string("seed"));

  execute_options opts_a;
  opts_a.name = "grpc-producer-gen";
  opts_a.env = ea.env;
  int pid_a = sched_a.execute(std::string(R"(
    (begin
      (set squares (generator (lambda ()
        (for i (range 5)
          (yield (* i i))))))
      (let ((i 0))
        (for sq squares
          (begin
            (state-set (str-concat "data.v" (str i)) (str sq))
            (set i (+ i 1)))))
      (state-set "data.ready" "true")
      "produced")
  )"),
                              opts_a);

  sched_a.run();
  ASSERT_TRUE(sched_a.get_result(pid_a).has_value());

  // Pump to B.
  tr_a.pump_all();
  tr_a.flush();
  tr_b.wait_for_received(6, std::chrono::milliseconds(5000));

  // Process B: consumer with generator.
  scheduler sched_b;
  auto &root_b = cvc::state::instance(app_b);
  sched_b.set_watch_root(&root_b);
  auto eb = make_exec_env(sched_b, root_b);

  execute_options opts_b;
  opts_b.name = "grpc-consumer-gen";
  opts_b.env = eb.env;
  int pid_b = sched_b.execute(std::string(R"(
    (begin
      (set reader (generator (lambda ()
        (for i (range 5)
          (yield (int (state-get (str-concat "data.v" (str i)))))))))
      (set sum 0)
      (for val reader
        (set sum (+ sum val)))
      sum)
  )"),
                              opts_b);

  sched_b.run();
  auto result = sched_b.get_result(pid_b);
  ASSERT_TRUE(result.has_value());
  // 0^2 + 1^2 + 2^2 + 3^2 + 4^2 = 0+1+4+9+16 = 30
  EXPECT_EQ(std::get<int64_t>(result->v), 30);

  tr_a.stop();
  tr_b.stop();
}

#endif // CVC_ENABLE_GRPC

#endif // !_WIN32
