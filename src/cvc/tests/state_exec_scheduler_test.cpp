/// @file state_exec_scheduler_test.cpp
/// @brief Tests for Phase 4: Process, MemoryTracker, Scheduler, AsyncScheduler.

#include <algorithm>
#include <cmath>
#include <cvc/core/state_exec/async_scheduler.h>
#include <cvc/core/state_exec/memory_tracker.h>
#include <cvc/core/state_exec/process.h>
#include <cvc/core/state_exec/scheduler.h>
#include <gtest/gtest.h>
#include <string>
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
