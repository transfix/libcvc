/// @file state_exec_scheduler_test.cpp
/// @brief Tests for Phase 4: Process, MemoryTracker, Scheduler, AsyncScheduler.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cvc/state/state_exec/async_evaluator.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/async_stackless_evaluator.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <gtest/gtest.h>
#include <random>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <vector>

using namespace cvc::state_exec;

// ===========================================================================
// ProcessStatusTest
// ===========================================================================

class ProcessStatusTest : public ::testing::Test {};

TEST_F(ProcessStatusTest, ToStringValues) {
  EXPECT_STREQ(to_string(process_status::ready), "ready");
  EXPECT_STREQ(to_string(process_status::running), "running");
  EXPECT_STREQ(to_string(process_status::paused), "paused");
  EXPECT_STREQ(to_string(process_status::waiting), "waiting");
  EXPECT_STREQ(to_string(process_status::terminated), "terminated");
  EXPECT_STREQ(to_string(process_status::killed), "killed");
}

TEST_F(ProcessStatusTest, ParseStatusValues) {
  EXPECT_EQ(parse_process_status("ready"), process_status::ready);
  EXPECT_EQ(parse_process_status("running"), process_status::running);
  EXPECT_EQ(parse_process_status("paused"), process_status::paused);
  EXPECT_EQ(parse_process_status("waiting"), process_status::waiting);
  EXPECT_EQ(parse_process_status("terminated"), process_status::terminated);
  EXPECT_EQ(parse_process_status("killed"), process_status::killed);
}

TEST_F(ProcessStatusTest, ParseUnknownDefaultsToReady) {
  EXPECT_EQ(parse_process_status("bogus"), process_status::ready);
  EXPECT_EQ(parse_process_status(""), process_status::ready);
}

TEST_F(ProcessStatusTest, ProcessElapsedTime) {
  process p;
  p.status = process_status::paused;
  p.accumulated_time = 2.5;
  EXPECT_DOUBLE_EQ(p.elapsed_time(), 2.5);
}

TEST_F(ProcessStatusTest, ProcessStepCount) {
  process p;
  p.state.stats.start();
  p.state.stats.increment_step();
  p.state.stats.increment_step();
  EXPECT_EQ(p.step_count(), 2u);
}

// ===========================================================================
// MemoryTrackerTest
// ===========================================================================

class MemoryTrackerTest : public ::testing::Test {
protected:
  memory_tracker tracker;
};

TEST_F(MemoryTrackerTest, EmptyTrackerReturnsZero) {
  EXPECT_EQ(tracker.current_bytes(1), 0u);
  EXPECT_EQ(tracker.peak_bytes(1), 0u);
}

TEST_F(MemoryTrackerTest, SingleWrite) {
  tracker.record_write(1, 100, 256);
  EXPECT_EQ(tracker.current_bytes(1), 256u);
  EXPECT_EQ(tracker.peak_bytes(1), 256u);
}

TEST_F(MemoryTrackerTest, MultipleWrites) {
  tracker.record_write(1, 100, 256);
  tracker.record_write(1, 200, 512);
  EXPECT_EQ(tracker.current_bytes(1), 768u);
}

TEST_F(MemoryTrackerTest, OverwriteSameOwner) {
  tracker.record_write(1, 100, 256);
  tracker.record_write(1, 100, 128); // Overwrite with smaller
  EXPECT_EQ(tracker.current_bytes(1), 128u);
  EXPECT_EQ(tracker.peak_bytes(1), 256u); // Peak stays at 256
}

TEST_F(MemoryTrackerTest, OverwriteDifferentOwner) {
  tracker.record_write(1, 100, 256);
  tracker.record_write(2, 100, 512); // Different owner takes over
  EXPECT_EQ(tracker.current_bytes(1), 0u);
  EXPECT_EQ(tracker.current_bytes(2), 512u);
}

TEST_F(MemoryTrackerTest, Delete) {
  tracker.record_write(1, 100, 256);
  tracker.record_write(1, 200, 512);
  tracker.record_delete(100);
  EXPECT_EQ(tracker.current_bytes(1), 512u);
}

TEST_F(MemoryTrackerTest, DeleteNonexistent) {
  tracker.record_delete(999); // Should not crash
  EXPECT_EQ(tracker.current_bytes(1), 0u);
}

TEST_F(MemoryTrackerTest, ReleaseOwnership) {
  tracker.record_write(1, 100, 256);
  tracker.record_write(1, 200, 512);
  tracker.release_ownership(1);
  EXPECT_EQ(tracker.current_bytes(1), 0u);
  // Peak should still be recorded
  EXPECT_EQ(tracker.peak_bytes(1), 768u);
}

TEST_F(MemoryTrackerTest, ForkOwnership) {
  tracker.record_write(1, 100, 256);
  tracker.record_write(1, 200, 512);
  tracker.fork_ownership(1, 2, {100, 200});
  EXPECT_EQ(tracker.current_bytes(1), 768u); // Parent unchanged
  EXPECT_EQ(tracker.current_bytes(2), 768u); // Child gets same
}

TEST_F(MemoryTrackerTest, Clear) {
  tracker.record_write(1, 100, 256);
  tracker.clear();
  EXPECT_EQ(tracker.current_bytes(1), 0u);
  EXPECT_EQ(tracker.peak_bytes(1), 0u);
}

// ===========================================================================
// SchedulerTest — Basic execution
// ===========================================================================

class SchedulerTest : public ::testing::Test {
protected:
  scheduler sched{scheduling_policy::round_robin};
};

TEST_F(SchedulerTest, SingleProcessSimple) {
  int pid = sched.execute(std::string("42"));
  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  auto &val = results[pid];
  ASSERT_TRUE(std::holds_alternative<int64_t>(val.v));
  EXPECT_EQ(std::get<int64_t>(val.v), 42);
}

TEST_F(SchedulerTest, TimeBudgetStopsALoopingInitMethod) {
  // A defclass constructor's init is a closure run as frames of the calling process, so a
  // looping body is the process's own steps: the scheduler's max_time check kills it, as any
  // loop, instead of the body spinning inside one step on an evaluator of its own.
  execute_options opts;
  opts.max_time = 0.05; // 50 ms process budget
  const int pid =
      sched.execute(std::string("(begin (defclass B (init (self) (while 1 1))) (B))"), opts);
  sched.run(); // bounded (~50 ms), not a hang
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(SchedulerTest, SingleProcessArithmetic) {
  int pid = sched.execute(std::string("(+ 10 20 30)"));
  auto results = sched.run();
  ASSERT_TRUE(results.count(pid));
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 60);
}

TEST_F(SchedulerTest, MultipleProcesses) {
  int p1 = sched.execute(std::string("(+ 1 2)"));
  int p2 = sched.execute(std::string("(* 3 4)"));
  int p3 = sched.execute(std::string("(- 100 50)"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[p1].v), 3);
  EXPECT_EQ(std::get<int64_t>(results[p2].v), 12);
  EXPECT_EQ(std::get<int64_t>(results[p3].v), 50);
}

TEST_F(SchedulerTest, ProcessWithName) {
  execute_options opts;
  opts.name = "my-process";
  int pid = sched.execute(std::string("42"), opts);
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->name, "my-process");
}

TEST_F(SchedulerTest, DefaultProcessName) {
  int pid = sched.execute(std::string("42"));
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->name, "proc-" + std::to_string(pid));
}

TEST_F(SchedulerTest, ProcessWithUidGid) {
  execute_options opts;
  opts.uid = "user1";
  opts.gid = "group1";
  int pid = sched.execute(std::string("42"), opts);
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->uid, "user1");
  EXPECT_EQ(info->gid, "group1");
}

TEST_F(SchedulerTest, FibonacciProcess) {
  int pid = sched.execute(std::string(R"(
        (begin
          (defun fib (n)
            (if (<= n 1) n
              (+ (fib (- n 1)) (fib (- n 2)))))
          (fib 10))
    )"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 55);
}

TEST_F(SchedulerTest, StepByStep) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  int stepped = 0;
  while (sched.has_runnable()) {
    sched.step();
    ++stepped;
  }
  EXPECT_GT(stepped, 0);
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<int64_t>(result->v), 3);
}

TEST_F(SchedulerTest, ProcessStatus) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::ready);

  sched.run();
  info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::terminated);
}

TEST_F(SchedulerTest, OnCompleteCallback) {
  value_t captured;
  execute_options opts;
  opts.on_complete = [&captured](value_t val) { captured = val; };
  sched.execute(std::string("99"), opts);
  sched.run();
  ASSERT_TRUE(std::holds_alternative<int64_t>(captured.v));
  EXPECT_EQ(std::get<int64_t>(captured.v), 99);
}

TEST_F(SchedulerTest, GetResultBeforeCompletion) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  auto result = sched.get_result(pid);
  EXPECT_FALSE(result.has_value()); // Not done yet
}

TEST_F(SchedulerTest, GetResultNonexistentPid) {
  auto result = sched.get_result(999);
  EXPECT_FALSE(result.has_value());
}

TEST_F(SchedulerTest, GetProcessInfoNonexistent) {
  auto info = sched.get_process_info(999);
  EXPECT_FALSE(info.has_value());
}

// ===========================================================================
// SchedulerTest — Scheduling policies
// ===========================================================================

TEST_F(SchedulerTest, RoundRobinInterleaving) {
  // Two processes should be interleaved
  int p1 = sched.execute(std::string("(begin 1 2 3)"));
  int p2 = sched.execute(std::string("(begin 4 5 6)"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[p1].v), 3);
  EXPECT_EQ(std::get<int64_t>(results[p2].v), 6);
}

TEST(SchedulerPolicyTest, PrioritySelectsHighest) {
  scheduler sched(scheduling_policy::priority);
  execute_options low_opts;
  low_opts.priority = 10;
  low_opts.name = "low";
  execute_options high_opts;
  high_opts.priority = -10;
  high_opts.name = "high";
  int p_low = sched.execute(std::string("(begin 1 2 3)"), low_opts);
  int p_high = sched.execute(std::string("(begin 4 5 6)"), high_opts);
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[p_low].v), 3);
  EXPECT_EQ(std::get<int64_t>(results[p_high].v), 6);
}

TEST(SchedulerPolicyTest, PriorityRRPolicy) {
  scheduler sched(scheduling_policy::priority_rr);
  execute_options opts;
  opts.priority = 0;
  int p1 = sched.execute(std::string("(+ 1 2)"), opts);
  int p2 = sched.execute(std::string("(+ 3 4)"), opts);
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[p1].v), 3);
  EXPECT_EQ(std::get<int64_t>(results[p2].v), 7);
}

// ===========================================================================
// SchedulerTest — Process control
// ===========================================================================

TEST_F(SchedulerTest, PauseAndResume) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));

  // Step a bit
  sched.step();
  sched.step();

  // Pause
  EXPECT_TRUE(sched.pause(pid));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::paused);

  // Step should do nothing while paused
  EXPECT_EQ(sched.step(), 0);

  // Resume
  EXPECT_TRUE(sched.resume(pid));
  info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::ready);

  // Complete
  auto results = sched.run();
  EXPECT_TRUE(results.count(pid));
}

TEST_F(SchedulerTest, PauseNonexistent) { EXPECT_FALSE(sched.pause(999)); }

TEST_F(SchedulerTest, ResumeNonPaused) {
  int pid = sched.execute(std::string("42"));
  EXPECT_FALSE(sched.resume(pid));
}

TEST_F(SchedulerTest, SleepAndWake) {
  // Sleep for a very short duration so the test completes quickly.
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));

  // Step once to get the process running.
  sched.step();

  // Put the process to sleep for 10 ms.
  EXPECT_TRUE(sched.sleep(pid, 0.01));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::waiting);

  // Stepping should not run the sleeping process.
  EXPECT_EQ(sched.step(), 0);

  // But has_runnable should still return true (sleeping counts).
  EXPECT_TRUE(sched.has_runnable());

  // Wait for the sleep to expire, then it should wake up.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  auto results = sched.run();
  EXPECT_TRUE(results.count(pid));
}

TEST_F(SchedulerTest, SleepNonexistent) { EXPECT_FALSE(sched.sleep(999, 1.0)); }

TEST_F(SchedulerTest, ReceiveMessageSuspendsProcess) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  EXPECT_TRUE(sched.receive_message(pid, "test.path"));
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting);
}

TEST_F(SchedulerTest, ReceiveMessageNonexistent) {
  EXPECT_FALSE(sched.receive_message(999, "test.path"));
}

TEST_F(SchedulerTest, DeliverToReceiversWakesProcess) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  sched.receive_message(pid, "test.inbox");
  auto msg = make_dict({{"payload", value_t(std::string("hello"))}});
  int woken = sched.deliver_to_receivers("test.inbox", msg);
  EXPECT_EQ(woken, 1);
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::ready);
}

TEST_F(SchedulerTest, DeliverToReceiversNoMatch) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  sched.receive_message(pid, "test.inbox");
  auto msg = make_dict({{"payload", value_t(std::string("hello"))}});
  int woken = sched.deliver_to_receivers("other.path", msg);
  EXPECT_EQ(woken, 0);
  // Process should still be waiting
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::waiting);
}

TEST_F(SchedulerTest, HasRunnableCountsMessageWaiting) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  sched.receive_message(pid, "test.inbox");
  // Only waiting process, but has recv_path → still considered runnable
  EXPECT_TRUE(sched.has_runnable());
}

// --- Message queue tests ---

TEST_F(SchedulerTest, DeliverQueuesWhenNoReceiver) {
  // No processes waiting → message goes to pending queue
  auto msg = make_dict({{"payload", value_t(std::string("queued"))}});
  int woken = sched.deliver_to_receivers("q.path", msg);
  EXPECT_EQ(woken, 0);
  EXPECT_EQ(sched.pending_message_count("q.path"), 1u);
  EXPECT_EQ(sched.total_pending_messages(), 1u);
}

TEST_F(SchedulerTest, PopPendingMessageDrainsQueue) {
  auto msg1 = make_dict({{"n", value_t(int64_t(1))}});
  auto msg2 = make_dict({{"n", value_t(int64_t(2))}});
  sched.deliver_to_receivers("q", msg1);
  sched.deliver_to_receivers("q", msg2);
  EXPECT_EQ(sched.pending_message_count("q"), 2u);

  auto m1 = sched.pop_pending_message("q");
  ASSERT_TRUE(m1.has_value());
  EXPECT_EQ(sched.pending_message_count("q"), 1u);

  auto m2 = sched.pop_pending_message("q");
  ASSERT_TRUE(m2.has_value());
  EXPECT_EQ(sched.pending_message_count("q"), 0u);

  auto m3 = sched.pop_pending_message("q");
  EXPECT_FALSE(m3.has_value());
}

TEST_F(SchedulerTest, PopPendingMessageFIFOOrder) {
  for (int i = 0; i < 5; ++i)
    sched.deliver_to_receivers("fifo", make_dict({{"i", value_t(int64_t(i))}}));
  EXPECT_EQ(sched.pending_message_count("fifo"), 5u);

  for (int i = 0; i < 5; ++i) {
    auto m = sched.pop_pending_message("fifo");
    ASSERT_TRUE(m.has_value());
    auto *dp = std::get_if<dict_ptr>(&m->v);
    ASSERT_NE(dp, nullptr);
    // dict is vector<pair<string,value_t>>; find "i" key
    int64_t val = -1;
    for (auto &[k, v] : **dp) {
      if (k == "i") {
        val = std::get<int64_t>(v.v);
        break;
      }
    }
    EXPECT_EQ(val, i);
  }
}

TEST_F(SchedulerTest, DeliverPrefersLiveReceiverOverQueue) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  sched.receive_message(pid, "live");
  auto msg = make_dict({{"x", value_t(int64_t(42))}});
  int woken = sched.deliver_to_receivers("live", msg);
  EXPECT_EQ(woken, 1);
  // Message went to receiver, not to queue
  EXPECT_EQ(sched.pending_message_count("live"), 0u);
}

TEST_F(SchedulerTest, PendingQueueBounded) {
  sched.max_pending_messages = 3;
  for (int i = 0; i < 10; ++i)
    sched.deliver_to_receivers("bounded", make_dict({{"i", value_t(int64_t(i))}}));
  EXPECT_EQ(sched.pending_message_count("bounded"), 3u);
  // First 3 messages are preserved (FIFO)
  for (int i = 0; i < 3; ++i) {
    auto m = sched.pop_pending_message("bounded");
    ASSERT_TRUE(m.has_value());
    auto *dp = std::get_if<dict_ptr>(&m->v);
    ASSERT_NE(dp, nullptr);
    int64_t val = -1;
    for (auto &[k, v] : **dp) {
      if (k == "i") {
        val = std::get<int64_t>(v.v);
        break;
      }
    }
    EXPECT_EQ(val, i);
  }
}

TEST_F(SchedulerTest, PendingQueueMultiplePaths) {
  sched.deliver_to_receivers("a", make_dict({{"path", value_t(std::string("a"))}}));
  sched.deliver_to_receivers("b", make_dict({{"path", value_t(std::string("b"))}}));
  sched.deliver_to_receivers("a", make_dict({{"path", value_t(std::string("a2"))}}));
  EXPECT_EQ(sched.pending_message_count("a"), 2u);
  EXPECT_EQ(sched.pending_message_count("b"), 1u);
  EXPECT_EQ(sched.total_pending_messages(), 3u);
}

TEST_F(SchedulerTest, PopPendingEmptyPath) {
  auto m = sched.pop_pending_message("nonexistent");
  EXPECT_FALSE(m.has_value());
  EXPECT_EQ(sched.pending_message_count("nonexistent"), 0u);
}

TEST_F(SchedulerTest, KillProcess) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  EXPECT_TRUE(sched.kill(pid));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
  ASSERT_TRUE(info.has_value());
}

TEST_F(SchedulerTest, KillAlreadyTerminated) {
  int pid = sched.execute(std::string("42"));
  sched.run();
  EXPECT_FALSE(sched.kill(pid)); // Already terminated
}

TEST_F(SchedulerTest, KillNonexistent) { EXPECT_FALSE(sched.kill(999)); }

// ===========================================================================
// SchedulerTest — Resource limits
// ===========================================================================

TEST_F(SchedulerTest, MaxStepsLimit) {
  execute_options opts;
  opts.max_steps = 5;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"), opts);
  sched.run();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(SchedulerTest, MaxStepsLimitError) {
  execute_options opts;
  opts.max_steps = 3;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"), opts);
  sched.run();
  auto it = sched.list_processes();
  auto proc_it =
      std::find_if(it.begin(), it.end(), [pid](const process_info &pi) { return pi.pid == pid; });
  ASSERT_NE(proc_it, it.end());
  EXPECT_EQ(proc_it->status, process_status::killed);
}

TEST_F(SchedulerTest, MaxStepsZeroIsUnlimited) {
  execute_options opts;
  opts.max_steps = 0; // Unlimited
  int pid = sched.execute(std::string("(+ 1 2)"), opts);
  auto results = sched.run();
  EXPECT_TRUE(results.count(pid));
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 3);
}

TEST_F(SchedulerTest, SetMaxStepsRuntime) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"));
  sched.set_max_steps(pid, 3);
  sched.run();
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(SchedulerTest, SetMaxTimeRuntime) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  EXPECT_TRUE(sched.set_max_time(pid, 10.0));
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
}

TEST_F(SchedulerTest, SetMaxMemoryRuntime) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  EXPECT_TRUE(sched.set_max_memory(pid, 1024));
}

TEST_F(SchedulerTest, SetMaxMessagesRuntime) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  EXPECT_TRUE(sched.set_max_messages(pid, 100));
}

TEST_F(SchedulerTest, SetPriorityRuntime) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  EXPECT_TRUE(sched.set_priority(pid, -5));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->priority, -5);
}

TEST_F(SchedulerTest, SetLimitsNonexistentPid) {
  EXPECT_FALSE(sched.set_max_steps(999, 10));
  EXPECT_FALSE(sched.set_max_time(999, 10.0));
  EXPECT_FALSE(sched.set_max_memory(999, 1024));
  EXPECT_FALSE(sched.set_max_messages(999, 100));
  EXPECT_FALSE(sched.set_priority(999, 0));
}

// ===========================================================================
// SchedulerTest — Global run limits
// ===========================================================================

TEST_F(SchedulerTest, RunWithGlobalMaxSteps) {
  sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"));
  auto results = sched.run(3); // Max 3 global steps
  // Process should still be running (not all steps used)
  EXPECT_TRUE(sched.has_runnable());
}

TEST_F(SchedulerTest, StopRunLoop) {
  sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"));
  // We can't really test stop() in single-threaded context easily,
  // but we can verify the flag behavior
  sched.stop();
  auto results = sched.run();
  // Should return quickly because stop was requested
}

// ===========================================================================
// SchedulerTest — Signal handling
// ===========================================================================

TEST_F(SchedulerTest, SignalDelivery) {
  // Register a signal handler that sets a variable
  execute_options opts;
  opts.signal_handlers["SIGUSR1"] = make_list({value_t{symbol{"begin"}}, value_t{int64_t(42)}});
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"), opts);

  // Step a couple times to get it going
  sched.step();

  // Send signal
  EXPECT_TRUE(sched.send_signal(pid, "SIGUSR1"));

  // Run to completion
  sched.run();
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::terminated);
}

TEST_F(SchedulerTest, SigkillImmediateTermination) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"));
  sched.step();
  EXPECT_TRUE(sched.send_signal(pid, "SIGKILL"));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(SchedulerTest, SignalToNonexistentProcess) {
  EXPECT_FALSE(sched.send_signal(999, "SIGUSR1"));
}

TEST_F(SchedulerTest, SignalToTerminatedProcess) {
  int pid = sched.execute(std::string("42"));
  sched.run();
  EXPECT_FALSE(sched.send_signal(pid, "SIGUSR1"));
}

TEST_F(SchedulerTest, UnhandledSignalIsIgnored) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.step();
  // Send signal with no registered handler
  EXPECT_TRUE(sched.send_signal(pid, "SIGUSR1"));
  auto results = sched.run();
  // Process should still complete normally
  EXPECT_TRUE(results.count(pid));
}

// ===========================================================================
// SchedulerTest — Forking
// ===========================================================================

TEST_F(SchedulerTest, ForkCreatesChild) {
  int parent_pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.step();
  int child_pid = sched.fork(parent_pid);
  EXPECT_GT(child_pid, 0);
  EXPECT_NE(child_pid, parent_pid);

  auto child_info = sched.get_process_info(child_pid);
  ASSERT_TRUE(child_info.has_value());
  EXPECT_EQ(child_info->parent_pid, parent_pid);
  EXPECT_EQ(child_info->status, process_status::ready);
}

TEST_F(SchedulerTest, ForkInheritsProperties) {
  execute_options opts;
  opts.priority = -5;
  opts.uid = "user1";
  opts.gid = "group1";
  opts.max_steps = 100;
  opts.max_time = 60.0;
  int parent = sched.execute(std::string("(begin 1 2 3)"), opts);
  sched.step();
  int child = sched.fork(parent);

  auto info = sched.get_process_info(child);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->priority, -5);
  EXPECT_EQ(info->uid, "user1");
  EXPECT_EQ(info->gid, "group1");
}

TEST_F(SchedulerTest, ForkNonexistentFails) { EXPECT_EQ(sched.fork(999), -1); }

TEST_F(SchedulerTest, ForkTerminatedFails) {
  int pid = sched.execute(std::string("42"));
  sched.run();
  EXPECT_EQ(sched.fork(pid), -1);
}

TEST_F(SchedulerTest, ForkBothComplete) {
  int parent = sched.execute(std::string("(+ 1 2)"));
  sched.step();
  int child = sched.fork(parent);
  EXPECT_GT(child, 0);

  // Both should eventually complete
  sched.run();
  // At least one should be terminated
  auto parent_info = sched.get_process_info(parent);
  auto child_info = sched.get_process_info(child);
  EXPECT_TRUE(parent_info->status == process_status::terminated ||
              parent_info->status == process_status::killed);
  EXPECT_TRUE(child_info->status == process_status::terminated ||
              child_info->status == process_status::killed);
}

// ===========================================================================
// SchedulerTest — Stats and listing
// ===========================================================================

TEST_F(SchedulerTest, ListProcesses) {
  sched.execute(std::string("1"));
  sched.execute(std::string("2"));
  sched.execute(std::string("3"));
  auto procs = sched.list_processes();
  EXPECT_EQ(procs.size(), 3u);
  // Should be sorted by PID
  EXPECT_LT(procs[0].pid, procs[1].pid);
  EXPECT_LT(procs[1].pid, procs[2].pid);
}

TEST_F(SchedulerTest, GetStats) {
  sched.execute(std::string("1"));
  sched.execute(std::string("2"));
  auto stats = sched.get_stats();
  EXPECT_EQ(stats.total_processes, 2);
  EXPECT_EQ(stats.ready, 2);
  EXPECT_EQ(stats.terminated, 0);

  sched.run();
  stats = sched.get_stats();
  EXPECT_EQ(stats.terminated, 2);
  EXPECT_GT(stats.total_steps, 0u);
}

TEST_F(SchedulerTest, GetResults) {
  int p1 = sched.execute(std::string("10"));
  int p2 = sched.execute(std::string("20"));
  sched.run();
  auto results = sched.get_results();
  EXPECT_EQ(results.size(), 2u);
  EXPECT_EQ(std::get<int64_t>(results[p1].v), 10);
  EXPECT_EQ(std::get<int64_t>(results[p2].v), 20);
}

TEST_F(SchedulerTest, HasRunnable) {
  EXPECT_FALSE(sched.has_runnable());
  sched.execute(std::string("42"));
  EXPECT_TRUE(sched.has_runnable());
  sched.run();
  EXPECT_FALSE(sched.has_runnable());
}

TEST_F(SchedulerTest, ProcessCount) {
  EXPECT_EQ(sched.process_count(), 0);
  sched.execute(std::string("1"));
  sched.execute(std::string("2"));
  EXPECT_EQ(sched.process_count(), 2);
}

TEST_F(SchedulerTest, PolicyAccessor) {
  EXPECT_EQ(sched.policy(), scheduling_policy::round_robin);
  scheduler sched2(scheduling_policy::priority);
  EXPECT_EQ(sched2.policy(), scheduling_policy::priority);
}

// ===========================================================================
// SchedulerTest — Complex programs
// ===========================================================================

TEST_F(SchedulerTest, TwoProcessesFibonacci) {
  int p1 = sched.execute(std::string(R"(
        (begin
          (defun fib (n)
            (if (<= n 1) n
              (+ (fib (- n 1)) (fib (- n 2)))))
          (fib 8))
    )"));
  int p2 = sched.execute(std::string(R"(
        (begin
          (defun fact (n)
            (if (<= n 1) 1
              (* n (fact (- n 1)))))
          (fact 6))
    )"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[p1].v), 21);
  EXPECT_EQ(std::get<int64_t>(results[p2].v), 720);
}

TEST_F(SchedulerTest, LoopProcess) {
  int pid = sched.execute(std::string(R"(
        (begin
          (let ((sum 0))
            (for i (list 1 2 3 4 5)
              (set sum (+ sum i)))
            sum))
    )"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 15);
}

TEST_F(SchedulerTest, WhileLoopProcess) {
  int pid = sched.execute(std::string(R"(
        (begin
          (let ((i 0) (sum 0))
            (while (< i 10)
              (begin
                (set sum (+ sum i))
                (set i (+ i 1))))
            sum))
    )"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 45);
}

TEST_F(SchedulerTest, LambdaProcess) {
  int pid = sched.execute(std::string(R"(
        (begin
          (let ((double (lambda (x) (* x 2))))
            (double 21)))
    )"));
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 42);
}

TEST_F(SchedulerTest, StringOpsProcess) {
  int pid = sched.execute(std::string(R"(
        (str-concat "hello" " " "world")
    )"));
  auto results = sched.run();
  EXPECT_EQ(std::get<std::string>(results[pid].v), "hello world");
}

// ===========================================================================
// SchedulerTest — Multiple concurrent processes with varying complexity
// ===========================================================================

TEST_F(SchedulerTest, ManyProcessesConcurrent) {
  std::vector<int> pids;
  for (int i = 0; i < 10; ++i) {
    pids.push_back(sched.execute("(+ " + std::to_string(i) + " 1)"));
  }
  auto results = sched.run();
  EXPECT_EQ(results.size(), 10u);
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(std::get<int64_t>(results[pids[i]].v), i + 1);
  }
}

TEST_F(SchedulerTest, ExecuteExpression) {
  auto expr = make_list({value_t{symbol{"+"}}, value_t{int64_t(10)}, value_t{int64_t(20)}});
  int pid = sched.execute(expr);
  auto results = sched.run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 30);
}

// ===========================================================================
// SchedulerTest — Elapsed time
// ===========================================================================

TEST_F(SchedulerTest, ProcessInfoHasElapsedTime) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.run();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_GE(info->elapsed_time, 0.0);
}

TEST_F(SchedulerTest, ProcessInfoStepCount) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  sched.run();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_GT(info->step_count, 0u);
}

// ===========================================================================
// AsyncSchedulerTest
// ===========================================================================

class AsyncSchedulerTest : public ::testing::Test {
protected:
  async_scheduler sched{scheduling_policy::round_robin};
};

TEST_F(AsyncSchedulerTest, SingleProcessSimple) {
  int pid = sched.execute(std::string("42"));
  auto results = sched.sync_run();
  ASSERT_TRUE(results.count(pid));
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 42);
}

TEST_F(AsyncSchedulerTest, MultipleProcesses) {
  int p1 = sched.execute(std::string("(+ 1 2)"));
  int p2 = sched.execute(std::string("(* 3 4)"));
  auto results = sched.sync_run();
  EXPECT_EQ(std::get<int64_t>(results[p1].v), 3);
  EXPECT_EQ(std::get<int64_t>(results[p2].v), 12);
}

TEST_F(AsyncSchedulerTest, StepByStep) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  while (sched.has_runnable()) {
    sched.sync_step();
  }
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(std::get<int64_t>(result->v), 3);
}

TEST_F(AsyncSchedulerTest, PauseResume) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step();
  EXPECT_TRUE(sched.pause(pid));
  EXPECT_EQ(sched.sync_step(), 0);
  EXPECT_TRUE(sched.resume(pid));
  sched.sync_run();
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::terminated);
}

TEST_F(AsyncSchedulerTest, KillProcess) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step();
  EXPECT_TRUE(sched.kill(pid));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(AsyncSchedulerTest, MaxStepsLimit) {
  execute_options opts;
  opts.max_steps = 3;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"), opts);
  sched.sync_run();
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(AsyncSchedulerTest, FibonacciProcess) {
  int pid = sched.execute(std::string(R"(
        (begin
          (defun fib (n)
            (if (<= n 1) n
              (+ (fib (- n 1)) (fib (- n 2)))))
          (fib 8))
    )"));
  auto results = sched.sync_run();
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 21);
}

TEST_F(AsyncSchedulerTest, SignalDelivery) {
  execute_options opts;
  opts.signal_handlers["SIGUSR1"] = make_list({value_t{symbol{"begin"}}, value_t{int64_t(42)}});
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"), opts);
  sched.sync_step();
  EXPECT_TRUE(sched.send_signal(pid, "SIGUSR1"));
  sched.sync_run();
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::terminated);
}

TEST_F(AsyncSchedulerTest, SigkillImmediate) {
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step();
  EXPECT_TRUE(sched.send_signal(pid, "SIGKILL"));
  auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(AsyncSchedulerTest, ForkProcess) {
  int parent = sched.execute(std::string("(begin 1 2 3)"));
  sched.sync_step();
  int child = sched.fork(parent);
  EXPECT_GT(child, 0);

  auto child_info = sched.get_process_info(child);
  ASSERT_TRUE(child_info.has_value());
  EXPECT_EQ(child_info->parent_pid, parent);
}

TEST_F(AsyncSchedulerTest, GetStats) {
  sched.execute(std::string("1"));
  sched.execute(std::string("2"));
  auto stats = sched.get_stats();
  EXPECT_EQ(stats.total_processes, 2);
  sched.sync_run();
  stats = sched.get_stats();
  EXPECT_EQ(stats.terminated, 2);
}

TEST_F(AsyncSchedulerTest, ListProcesses) {
  sched.execute(std::string("1"));
  sched.execute(std::string("2"));
  auto procs = sched.list_processes();
  EXPECT_EQ(procs.size(), 2u);
}

TEST_F(AsyncSchedulerTest, RunWithMaxSteps) {
  sched.execute(std::string("(begin 1 2 3 4 5 6 7 8 9 10)"));
  sched.sync_run(3); // Max 3 global steps
  EXPECT_TRUE(sched.has_runnable());
}

TEST_F(AsyncSchedulerTest, CoroutineStep) {
  int pid = sched.execute(std::string("(+ 1 2)"));
  // Use coroutine step directly
  auto t = sched.step();
  int stepped = t.sync_wait();
  EXPECT_EQ(stepped, 1);
}

TEST_F(AsyncSchedulerTest, CoroutineRun) {
  int pid = sched.execute(std::string("42"));
  auto t = sched.run();
  auto results = t.sync_wait();
  ASSERT_TRUE(results.count(pid));
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 42);
}

TEST_F(AsyncSchedulerTest, PriorityPolicy) {
  async_scheduler sched2(scheduling_policy::priority);
  execute_options low_opts;
  low_opts.priority = 10;
  execute_options high_opts;
  high_opts.priority = -10;
  int p_low = sched2.execute(std::string("(+ 1 2)"), low_opts);
  int p_high = sched2.execute(std::string("(+ 3 4)"), high_opts);
  auto results = sched2.sync_run();
  EXPECT_EQ(std::get<int64_t>(results[p_low].v), 3);
  EXPECT_EQ(std::get<int64_t>(results[p_high].v), 7);
}

TEST_F(AsyncSchedulerTest, PolicyAccessor) {
  EXPECT_EQ(sched.policy(), scheduling_policy::round_robin);
}

TEST_F(AsyncSchedulerTest, TenProcessesConcurrent) {
  std::vector<int> pids;
  for (int i = 0; i < 10; ++i) {
    pids.push_back(sched.execute("(+ " + std::to_string(i) + " 100)"));
  }
  auto results = sched.sync_run();
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(std::get<int64_t>(results[pids[i]].v), i + 100);
  }
}

// ===========================================================================
// Cross-scheduler consistency
// ===========================================================================

class CrossSchedulerTest : public ::testing::Test {};

TEST_F(CrossSchedulerTest, SyncVsAsyncSameResults) {
  const std::string script = R"(
        (begin
          (defun fib (n)
            (if (<= n 1) n
              (+ (fib (- n 1)) (fib (- n 2)))))
          (fib 8))
    )";

  scheduler sync_sched;
  int sync_pid = sync_sched.execute(std::string(script));
  auto sync_results = sync_sched.run();

  async_scheduler async_sched;
  int async_pid = async_sched.execute(std::string(script));
  auto async_results = async_sched.sync_run();

  EXPECT_EQ(std::get<int64_t>(sync_results[sync_pid].v),
            std::get<int64_t>(async_results[async_pid].v));
  EXPECT_EQ(std::get<int64_t>(sync_results[sync_pid].v), 21);
}

TEST_F(CrossSchedulerTest, MultipleProgramsSameResults) {
  std::vector<std::string> scripts = {"(+ 1 2 3)", "(* 4 5)", "(if #t 42 99)",
                                      R"((let ((x 10)) (* x x)))", "(length \"hello\")"};

  scheduler sync_sched;
  async_scheduler async_sched;

  std::vector<int> sync_pids, async_pids;
  for (const auto &script : scripts) {
    sync_pids.push_back(sync_sched.execute(std::string(script)));
    async_pids.push_back(async_sched.execute(std::string(script)));
  }

  auto sync_results = sync_sched.run();
  auto async_results = async_sched.sync_run();

  for (size_t i = 0; i < scripts.size(); ++i) {
    auto &sv = sync_results[sync_pids[i]];
    auto &av = async_results[async_pids[i]];
    EXPECT_TRUE(values_equal(sv, av)) << "Mismatch on script: " << scripts[i];
  }
}

// ===========================================================================
// A time budget cut inside one step: an unsliced job, a generator body or a stdlib call polls
// the step's deadline and throws evaluation_timeout, reported as the process's time limit.
// ===========================================================================

// A generator whose body never yields: (next g) loops inside one step until a deadline cuts it.
constexpr const char *kNeverYields = "(next (generator (lambda () (while t 1))))";

TEST(StepDeadlineTest, ACheckThrowsEvaluationTimeoutNamingTheCall) {
  EXPECT_NO_THROW(eval_deadline_check("collections.range")); // no deadline armed
  eval_deadline_guard spent(0.0);
  EXPECT_TRUE(spent.expired());
  try {
    eval_deadline_check("collections.range");
    FAIL() << "no evaluation_timeout";
  } catch (const evaluation_timeout &e) {
    EXPECT_STREQ(e.what(), "collections.range: evaluation exceeded timeout");
  }
}

TEST(StepDeadlineTest, EveryEvaluatorCutsALoopInsideAStepAsATimeout) {
  // Each evaluator's timeout arms the thread's deadline, so the generator's own poll cuts the
  // loop where the evaluator's between-steps check cannot.
  const auto cut = [](auto &&evaluate) -> std::string {
    try {
      evaluate();
    } catch (const evaluation_timeout &e) {
      return e.what();
    }
    return "no evaluation_timeout";
  };
  auto env = builtins::make_default_environment();
  stackless_evaluator stackless(env);
  evaluator tree(env);
  async_evaluator async_tree(env);
  async_stackless_evaluator async_stackless(env);
  for (const std::string &what :
       {cut([&] { stackless.evaluate_script(kNeverYields, nullptr, 0.05); }),
        cut([&] { tree.evaluate_script(kNeverYields, nullptr, 0.05); }),
        cut([&] { async_tree.sync_evaluate_script(kNeverYields, nullptr, 0.05); }),
        cut([&] { async_stackless.sync_evaluate_script(kNeverYields, nullptr, 0.05); })})
    EXPECT_NE(what.find("evaluation exceeded timeout"), std::string::npos) << what;
}

TEST(StepDeadlineTest, TheSyncSchedulerKillsAProcessCutInsideAStep) {
  scheduler sched;
  execute_options opts;
  opts.max_time = 0.05;
  const int pid = sched.execute(std::string(kNeverYields), opts);
  EXPECT_NO_THROW(sched.run()); // was a runtime_error out of run()
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
}

TEST(StepDeadlineTest, TheSyncSchedulerLeavesADeadlineArmedOutsideItToItsCaller) {
  scheduler sched;
  const int pid = sched.execute(std::string(kNeverYields)); // no max_time of its own
  eval_deadline_guard outer(0.05);
  EXPECT_THROW(sched.run(), evaluation_timeout);
  EXPECT_NE(sched.get_process_info(pid)->status, process_status::killed);
}

TEST(StepDeadlineTest, TheAsyncSchedulerReportsATimeLimitWhereverItRanOut) {
  async_scheduler sched;
  execute_options in_step;
  in_step.max_time = 0.05;
  const int cut = sched.execute(std::string(kNeverYields), in_step);
  execute_options between_steps;
  between_steps.max_time = 0.05;
  const int looped = sched.execute(std::string("(while t 1)"), between_steps);
  execute_options per_step; // a resident: no total budget, a per-step cap
  per_step.max_step_time = 0.05;
  const int resident = sched.execute(std::string(kNeverYields), per_step);
  sched.sync_run();
  for (const int pid : {cut, looped, resident}) {
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed) << pid;
    EXPECT_EQ(sched.get_exit_error(pid).value_or(""), "time_limit_exceeded") << pid;
  }
  // Any other error is still reported by its message.
  const int failed = sched.execute(std::string("(/ 1 0)"));
  sched.sync_run();
  EXPECT_NE(sched.get_exit_error(failed).value_or(""), "time_limit_exceeded");
}

// ===========================================================================
// The run queue and the indexes (process_table): the pick order is the one the schedulers always
// had -- the runnable processes in pid order, indexed by rr_index (round_robin), the lowest pid
// of the best priority (priority), rr_index among the best priority's (priority_rr) -- and every
// index agrees with the processes' fields after every change.
// ===========================================================================

namespace {

// The pick the schedulers made before the run queue: scan, sort by pid, index.
struct reference_pick {
  scheduling_policy policy;
  int rr = 0;

  // `procs`: (pid, status, priority). Returns the pid picked, or -1.
  int pick(const std::vector<std::tuple<int, process_status, int>> &procs) {
    std::vector<std::pair<int, int>> runnable; // (pid, priority), pid order
    for (const auto &[pid, st, prio] : procs)
      if (st == process_status::ready || st == process_status::running)
        runnable.emplace_back(pid, prio);
    std::sort(runnable.begin(), runnable.end());
    if (runnable.empty())
      return -1;
    if (policy == scheduling_policy::round_robin) {
      rr = rr % static_cast<int>(runnable.size());
      const int pid = runnable[rr].first;
      rr = (rr + 1) % static_cast<int>(runnable.size());
      return pid;
    }
    int best = runnable[0].second;
    for (const auto &r : runnable)
      best = std::min(best, r.second);
    std::vector<int> same;
    for (const auto &r : runnable)
      if (r.second == best)
        same.push_back(r.first);
    if (policy == scheduling_policy::priority)
      return same.front();
    rr = rr % static_cast<int>(same.size());
    const int pid = same[rr];
    rr = (rr + 1) % static_cast<int>(same.size());
    return pid;
  }
};

const process_status kStatuses[] = {process_status::ready,  process_status::running,
                                    process_status::paused, process_status::waiting,
                                    process_status::killed, process_status::terminated,
                                    process_status::failed};

} // namespace

class ProcessTablePolicyTest : public ::testing::TestWithParam<scheduling_policy> {};

TEST_P(ProcessTablePolicyTest, PicksAsTheSortedScanDidAndKeepsItsIndexes) {
  const scheduling_policy policy = GetParam();
  detail::process_table table(policy != scheduling_policy::round_robin);
  reference_pick ref{policy};
  int rr = 0;
  std::mt19937 rng(1234);
  int next_pid = 1;
  const auto now = std::chrono::steady_clock::now();
  for (int op = 0; op < 20000; ++op) {
    const int kind = static_cast<int>(rng() % 100);
    std::vector<process *> live;
    for (const auto &[pid, pp] : table.all())
      live.push_back(pp.get());
    std::sort(live.begin(), live.end(),
              [](const process *a, const process *b) { return a->pid < b->pid; });
    if (kind < 15 || live.empty()) {
      auto p = make_process();
      p->pid = next_pid++;
      p->priority = static_cast<int>(rng() % 5) - 2;
      p->status = rng() % 4 == 0 ? process_status::paused : process_status::ready;
      table.insert(std::move(p));
    } else if (kind < 60) {
      process &p = *live[rng() % live.size()];
      if (is_finished(p.status)) {
        if (rng() % 2 == 0)
          ASSERT_TRUE(table.erase(p.pid));
        continue;
      }
      const process_status st = kStatuses[rng() % std::size(kStatuses)];
      if (p.status == process_status::waiting && !is_finished(st)) {
        p.wake_time.reset();
        p.recv_path.reset();
        p.recv_deadline.reset();
        p.awaiting_frame = false;
      }
      if (st == process_status::waiting) {
        // A sleep, a wait for a message (timed or not), or a frame wait.
        switch (rng() % 4) {
        case 0:
          p.wake_time = now + std::chrono::milliseconds(rng() % 1000);
          break;
        case 1:
          p.recv_path = "chan." + std::to_string(rng() % 4);
          break;
        case 2:
          p.recv_path = "chan." + std::to_string(rng() % 4);
          p.recv_deadline = now + std::chrono::milliseconds(rng() % 1000);
          break;
        default:
          p.awaiting_frame = true;
        }
      }
      if (rng() % 3 == 0) {
        p.watch_handlers[1] = {};
        table.update(p);
      }
      table.set_status(p, st);
    } else if (kind < 70) {
      process &p = *live[rng() % live.size()];
      p.priority = static_cast<int>(rng() % 5) - 2;
      table.update(p);
    } else if (kind < 75) {
      // Wake what is due by a time among the deadlines, or take a channel's waiters.
      if (rng() % 2 == 0) {
        table.wake_due(now + std::chrono::milliseconds(rng() % 1000));
      } else {
        std::vector<process *> out;
        table.take_receivers("chan." + std::to_string(rng() % 4), out);
        for (process *p : out) {
          p->recv_path.reset();
          p->recv_deadline.reset();
          table.set_status(*p, process_status::ready);
        }
      }
    } else {
      std::vector<std::tuple<int, process_status, int>> snap;
      for (const process *p : live)
        snap.emplace_back(p->pid, p->status, p->priority);
      const int want = ref.pick(snap);
      const process_ptr *got =
          policy == scheduling_policy::priority ? table.pick_first() : table.pick_round_robin(rr);
      ASSERT_EQ(got ? (*got)->pid : -1, want) << "op " << op;
      if (policy != scheduling_policy::priority)
        ASSERT_EQ(rr, ref.rr) << "op " << op;
    }
    ASSERT_TRUE(table.check()) << "op " << op;
  }
}

INSTANTIATE_TEST_SUITE_P(Policies, ProcessTablePolicyTest,
                         ::testing::Values(scheduling_policy::round_robin,
                                           scheduling_policy::priority,
                                           scheduling_policy::priority_rr));

TEST(ProcessTableTest, TimersWakeInDeadlineOrderAndStaleEntriesAreDropped) {
  detail::process_table table;
  const auto t0 = std::chrono::steady_clock::now();
  for (int i = 0; i < 6; ++i) {
    auto p = make_process();
    p->pid = i + 1;
    p->status = process_status::waiting;
    p->wake_time = t0 + std::chrono::milliseconds(10 * (6 - i)); // the last pid wakes first
    table.insert(std::move(p));
  }
  EXPECT_EQ(table.sleeping_count(), 6u);
  EXPECT_EQ(table.timed_count(), 6u);
  // Re-sleep pid 6 later: its first entry goes stale.
  process &six = *table.find(6);
  six.wake_time = t0 + std::chrono::milliseconds(100);
  table.update(six);
  EXPECT_EQ(table.next_timer(), t0 + std::chrono::milliseconds(20)); // pid 5
  EXPECT_EQ(table.wake_due(t0 + std::chrono::milliseconds(35)), 2u); // pids 5 and 4
  EXPECT_EQ(table.find(5)->status, process_status::ready);
  EXPECT_EQ(table.find(4)->status, process_status::ready);
  EXPECT_EQ(table.find(6)->status, process_status::waiting); // its stale 10 ms entry dropped
  EXPECT_EQ(table.sleeping_count(), 4u);
  EXPECT_EQ(table.wake_due(t0 + std::chrono::milliseconds(100)), 4u);
  EXPECT_FALSE(table.next_timer().has_value());
  EXPECT_EQ(table.runnable_count(), 6u);
  EXPECT_TRUE(table.check());
}

TEST(ProcessTableTest, ATimedWaitThatRunsOutBecomesReady) {
  detail::process_table table;
  const auto t0 = std::chrono::steady_clock::now();
  auto p = make_process();
  p->pid = 1;
  p->status = process_status::waiting;
  p->recv_path = "c";
  p->recv_deadline = t0 + std::chrono::milliseconds(5);
  table.insert(std::move(p));
  EXPECT_EQ(table.receiving_count(), 1u);
  EXPECT_EQ(table.wake_due(t0), 0u);
  EXPECT_EQ(table.wake_due(t0 + std::chrono::milliseconds(5)), 1u);
  const process &q = *table.find(1);
  EXPECT_EQ(q.status, process_status::ready);
  EXPECT_FALSE(q.recv_path.has_value());
  EXPECT_EQ(table.receiving_count(), 0u);
  EXPECT_TRUE(table.check());
}

TEST(ProcessTableTest, EndedProcessesLeaveEveryIndexButStayReadable) {
  detail::process_table table;
  for (int pid = 1; pid <= 1000; ++pid) {
    auto p = make_process();
    p->pid = pid;
    table.insert(std::move(p));
  }
  for (int pid = 1; pid <= 1000; ++pid)
    if (pid % 10 != 0)
      table.set_status(*table.find(pid), process_status::terminated);
  EXPECT_EQ(table.runnable_count(), 100u);
  EXPECT_EQ(table.size(), 1000u); // still there for get_result / get_process_info
  // New processes after many ended ones: the slots are compacted, the order kept.
  for (int pid = 1001; pid <= 1100; ++pid) {
    auto p = make_process();
    p->pid = pid;
    table.insert(std::move(p));
  }
  EXPECT_TRUE(table.check());
  int rr = 0;
  std::vector<int> order;
  for (int i = 0; i < 200; ++i)
    order.push_back((*table.pick_round_robin(rr))->pid);
  for (int i = 0; i < 100; ++i)
    EXPECT_EQ(order[i], 10 * (i + 1));
  for (int i = 100; i < 200; ++i)
    EXPECT_EQ(order[i], 901 + i);
  EXPECT_FALSE(table.erase(1001)); // live
  EXPECT_TRUE(table.erase(1));
  EXPECT_EQ(table.find(1), nullptr);
}

// The schedulers end to end: the pick under every policy, with processes pausing, resuming,
// waiting for messages, getting them, changing priority, ending, being killed, forked and reaped
// between steps, is the sorted scan's. The process stepped is the one whose step count moved.
template <class Sched> class SchedulerPickOrderTest : public ::testing::Test {};
using PickOrderSchedulers = ::testing::Types<scheduler, async_scheduler>;
TYPED_TEST_SUITE(SchedulerPickOrderTest, PickOrderSchedulers);

TYPED_TEST(SchedulerPickOrderTest, MatchesTheSortedScanUnderEveryPolicy) {
  for (const scheduling_policy policy :
       {scheduling_policy::round_robin, scheduling_policy::priority,
        scheduling_policy::priority_rr}) {
    TypeParam sched(policy);
    reference_pick ref{policy};
    std::mt19937 rng(static_cast<unsigned>(policy) * 7919u + 17u);
    auto step_once = [&sched]() {
      if constexpr (std::is_same_v<TypeParam, scheduler>)
        return sched.step();
      else
        return sched.sync_step();
    };
    for (int op = 0; op < 3000; ++op) {
      const std::vector<process_info> before = sched.list_processes();
      std::vector<int> live;
      for (const process_info &pi : before)
        if (!is_finished(pi.status))
          live.push_back(pi.pid);
      const int kind = static_cast<int>(rng() % 100);
      if (kind < 8 || live.empty()) {
        execute_options opts;
        opts.priority = static_cast<int>(rng() % 3) - 1;
        // Some end on their own (a few steps), the rest loop.
        const std::string src = rng() % 3 == 0 ? "(begin 1 2 3 4 5 6 7 8)" : "(while t 1)";
        sched.execute(src, opts);
      } else if (kind < 20) {
        const int pid = live[rng() % live.size()];
        switch (rng() % 7) {
        case 0:
          sched.pause(pid);
          break;
        case 1:
          sched.resume(pid);
          break;
        case 2:
          sched.receive_message(pid, "chan." + std::to_string(rng() % 3));
          break;
        case 3:
          sched.deliver_to_receivers("chan." + std::to_string(rng() % 3), value_t(int64_t(1)));
          break;
        case 4:
          sched.set_priority(pid, static_cast<int>(rng() % 3) - 1);
          break;
        case 5:
          sched.kill(pid);
          break;
        default:
          sched.fork(pid);
        }
      } else if (kind < 23) {
        for (const process_info &pi : before)
          if (is_finished(pi.status))
            sched.reap(pi.pid);
      } else {
        std::vector<std::tuple<int, process_status, int>> snap;
        for (const process_info &pi : before)
          snap.emplace_back(pi.pid, pi.status, pi.priority);
        const int want = ref.pick(snap);
        const int stepped = step_once();
        int got = -1;
        for (const process_info &pi : before) {
          const auto after = sched.get_process_info(pi.pid);
          if (after && after->step_count != pi.step_count) {
            ASSERT_EQ(got, -1) << "two processes stepped";
            got = pi.pid;
          }
        }
        ASSERT_EQ(stepped, want < 0 ? 0 : 1) << "op " << op;
        ASSERT_EQ(got, want) << "policy " << static_cast<int>(policy) << " op " << op;
      }
    }
  }
}

// ===========================================================================
// The sync run() while only sleepers are left: it waits instead of spinning
// ===========================================================================

TEST(SchedulerIdleTest, RunSleepsUntilTheEarliestWakeInsteadOfSpinning) {
  scheduler sched;
  const int a = sched.execute(std::string("(begin 1 2 3)"));
  const int b = sched.execute(std::string("(begin 4 5 6)"));
  sched.step();
  sched.step();
  ASSERT_TRUE(sched.sleep(a, 0.15));
  ASSERT_TRUE(sched.sleep(b, 0.3));
  const double cpu0 = thread_cpu_seconds();
  const auto t0 = std::chrono::steady_clock::now();
  auto results = sched.run();
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  const double cpu = thread_cpu_seconds() - cpu0;
  EXPECT_EQ(std::get<int64_t>(results.at(a).v), 3);
  EXPECT_EQ(std::get<int64_t>(results.at(b).v), 6);
  EXPECT_GE(wall, 0.29);
  EXPECT_LT(cpu, 0.5 * wall) << "run() spun while the processes slept";
}

TEST(SchedulerIdleTest, RunReturnsAtItsMaxTimeWhileAProcessSleepsPastIt) {
  scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.step();
  ASSERT_TRUE(sched.sleep(pid, 30.0));
  const auto t0 = std::chrono::steady_clock::now();
  sched.run(std::nullopt, 0.1);
  const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GE(wall, 0.09);
  EXPECT_LT(wall, 5.0);
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  EXPECT_TRUE(sched.has_runnable()); // the sleeper still counts
}

TEST(SchedulerIdleTest, RunWithAStepBoundReturnsWhenOnlyMessageWaitersAreLeft) {
  scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.step();
  ASSERT_TRUE(sched.receive_message(pid, "nobody.sends"));
  const auto t0 = std::chrono::steady_clock::now();
  sched.run(1000);
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 5.0);
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  // Delivered, it runs to its end.
  EXPECT_EQ(sched.deliver_to_receivers("nobody.sends", value_t(int64_t(7))), 1);
  const auto results = sched.run();
  EXPECT_TRUE(results.count(pid));
}

TEST(SchedulerIdleTest, ATimedWaitExpiresDuringTheIdleWait) {
  scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.step();
  ASSERT_TRUE(sched.receive_message_until(pid, "nobody.sends", 0.05));
  const auto results = sched.run(); // waits for the deadline, then the process goes on
  EXPECT_TRUE(results.count(pid));
}

TEST(SchedulerReapTest, ReapDropsOnlyFinishedProcesses) {
  scheduler sched;
  const int done = sched.execute(std::string("(+ 1 2)"));
  const int live = sched.execute(std::string("(while t 1)"));
  sched.run(100);
  ASSERT_EQ(std::get<int64_t>(sched.get_result(done)->v), 3);
  EXPECT_FALSE(sched.reap(live));
  EXPECT_FALSE(sched.reap(999));
  EXPECT_TRUE(sched.reap(done));
  EXPECT_FALSE(sched.get_result(done).has_value());
  EXPECT_FALSE(sched.get_process_info(done).has_value());
  EXPECT_EQ(sched.process_count(), 1);
  EXPECT_TRUE(sched.kill(live));
  EXPECT_TRUE(sched.reap(live));
  EXPECT_FALSE(sched.has_runnable());
}

// ===========================================================================
// async_scheduler::wait_for_work: an idle driver blocks until work arrives
// ===========================================================================

TEST(AsyncWaitForWorkTest, ReturnsAtOnceWhenAProcessIsReady) {
  async_scheduler sched;
  sched.execute(std::string("(+ 1 2)"));
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(sched.wait_for_work(5.0));
  EXPECT_LT(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 1.0);
}

TEST(AsyncWaitForWorkTest, TimesOutWithNothingToDo) {
  async_scheduler sched;
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_FALSE(sched.wait_for_work(0.05));
  EXPECT_GE(std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), 0.045);
}

TEST(AsyncWaitForWorkTest, WakesOnAPostFromAnotherThread) {
  async_scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.sync_step();
  ASSERT_TRUE(sched.receive_message(pid, "in.box"));
  std::thread poster([&sched] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sched.post_message("in.box", value_t(int64_t(9)));
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(sched.wait_for_work(30.0));
  const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  poster.join();
  EXPECT_LT(waited, 10.0); // woken by the post, not the 30 s deadline
  sched.drain_ingress();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::ready);
  sched.sync_run();
  EXPECT_TRUE(sched.get_result(pid).has_value());
}

TEST(AsyncWaitForWorkTest, WakesAtASleepersDeadline) {
  async_scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.sync_step();
  ASSERT_TRUE(sched.sleep(pid, 0.05));
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(sched.wait_for_work(30.0));
  const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_GE(waited, 0.045);
  EXPECT_LT(waited, 10.0);
  sched.drain_ingress();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::ready);
}

TEST(AsyncWaitForWorkTest, FrameWaitersAreTheDriversToWake) {
  async_scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.sync_step();
  ASSERT_TRUE(sched.yield_frame(pid));
  EXPECT_EQ(sched.awaiting_frame_count(), 1u);
  EXPECT_FALSE(sched.wait_for_work(0.02));
  EXPECT_EQ(sched.wake_awaiting(), 1);
  EXPECT_EQ(sched.awaiting_frame_count(), 0u);
  EXPECT_TRUE(sched.wait_for_work(0.0));
}

// A host may reap a process from its on_complete, while its last step is still on the stack:
// it is erased once that step is over.
TYPED_TEST(SchedulerPickOrderTest, AProcessReapedFromItsOnCompleteIsErasedAfterItsStep) {
  TypeParam sched;
  int pid = -1;
  bool reaped = false;
  execute_options opts;
  opts.on_complete = [&](value_t) { reaped = sched.reap(pid); };
  pid = sched.execute(std::string("(+ 1 2)"), opts);
  const int other = sched.execute(std::string("(begin 1 2 3)"));
  if constexpr (std::is_same_v<TypeParam, scheduler>)
    sched.run();
  else
    sched.sync_run();
  EXPECT_TRUE(reaped);
  EXPECT_FALSE(sched.get_process_info(pid).has_value());
  EXPECT_TRUE(sched.get_result(other).has_value());
  EXPECT_EQ(sched.process_count(), 1);
}
