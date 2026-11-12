#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_call.h> // AsyncPostControl
#include <cvc/state/state_exec/async_evaluator.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/async_stackless_evaluator.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>  // data_step_scope: a native's charge
#include <cvc/state/state_exec/evaluator.h> // step_kind
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h> // SyncPostControl
#include <cvc/state/state_exec/task.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <gtest/gtest.h>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace cvc::state_exec;

// A program's closure bound in the scope it captured (a defun, a lambda set at top level) and that
// scope hold each other, and only the cycle collector frees them
// (docs/STATE_EXEC_DEVELOPER_GUIDE.md, "Reference cycles"). The evaluator tests run with no
// scheduler and the scheduler tests are too short for one to be due, so nothing collects; once
// every test is done, one collection leaves LeakSanitizer only real leaks to report.
class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

// ===========================================================================
// task<T> unit tests
// ===========================================================================

static task<int> basic_value_coro() { co_return 42; }

TEST(TaskTest, BasicValue) {
  auto t = basic_value_coro();
  EXPECT_EQ(t.sync_wait(), 42);
}

static task<void> void_coro(bool *called) {
  *called = true;
  co_return;
}

TEST(TaskTest, VoidTask) {
  bool called = false;
  auto t = void_coro(&called);
  t.sync_wait();
  EXPECT_TRUE(called);
}

static task<int> inner_coro() { co_return 10; }
static task<int> chained_coro() {
  int a = co_await inner_coro();
  int b = co_await inner_coro();
  co_return a + b;
}

TEST(TaskTest, ChainedAwait) {
  auto t = chained_coro();
  EXPECT_EQ(t.sync_wait(), 20);
}

static task<int> throwing_coro() {
  throw std::runtime_error("boom");
  co_return 0;
}

TEST(TaskTest, ExceptionPropagation) {
  auto t = throwing_coro();
  EXPECT_THROW(t.sync_wait(), std::runtime_error);
}

// Helper: a coroutine that increments a counter, suspends, increments again
static task<int> suspend_point_coro(int *counter) {
  (*counter)++;
  co_await suspend_point{};
  (*counter)++;
  co_return *counter;
}

TEST(TaskTest, SuspendPoint) {
  int counter = 0;
  auto t = suspend_point_coro(&counter);
  // Task is lazy — nothing has run yet
  EXPECT_EQ(counter, 0);
  // First resume: runs until suspend_point, then returns to caller
  t.handle().resume();
  EXPECT_EQ(counter, 1);
  // Second resume: continues past suspend_point, runs to co_return,
  // then final_suspend transfers to noop_coroutine
  t.handle().resume();
  EXPECT_EQ(counter, 2);
  EXPECT_TRUE(t.done());
}

static task<int> move_only_coro() { co_return 99; }

TEST(TaskTest, MoveOnly) {
  auto t1 = move_only_coro();
  auto t2 = std::move(t1);
  EXPECT_EQ(t2.sync_wait(), 99);
}

// ===========================================================================
// async_evaluator tests
// ===========================================================================

class AsyncEvaluatorTest : public ::testing::Test {
protected:
  std::unique_ptr<async_evaluator> ev;
  void SetUp() override {
    ev = std::make_unique<async_evaluator>(builtins::make_default_environment());
  }

  value_t eval(const std::string &script) { return ev->sync_evaluate_script(script); }

  int64_t eval_int(const std::string &s) { return std::get<int64_t>(eval(s).v); }

  double eval_double(const std::string &s) { return std::get<double>(eval(s).v); }

  bool eval_bool(const std::string &s) { return std::get<bool>(eval(s).v); }

  std::string eval_string(const std::string &s) { return std::get<std::string>(eval(s).v); }
};

// --- Atoms ---

TEST_F(AsyncEvaluatorTest, IntegerLiteral) { EXPECT_EQ(eval_int("42"), 42); }

TEST_F(AsyncEvaluatorTest, DoubleLiteral) { EXPECT_DOUBLE_EQ(eval_double("3.14"), 3.14); }

TEST_F(AsyncEvaluatorTest, StringLiteral) { EXPECT_EQ(eval_string("\"hello\""), "hello"); }

TEST_F(AsyncEvaluatorTest, NilLiteral) {
  auto result = eval("nil");
  EXPECT_TRUE(std::holds_alternative<std::monostate>(result.v));
}

TEST_F(AsyncEvaluatorTest, TrueLiteral) { EXPECT_TRUE(eval_bool("t")); }

// --- Arithmetic ---

TEST_F(AsyncEvaluatorTest, Addition) { EXPECT_EQ(eval_int("(+ 1 2 3)"), 6); }

TEST_F(AsyncEvaluatorTest, Subtraction) { EXPECT_EQ(eval_int("(- 10 3)"), 7); }

TEST_F(AsyncEvaluatorTest, Multiplication) { EXPECT_EQ(eval_int("(* 4 5)"), 20); }

TEST_F(AsyncEvaluatorTest, Division) {
  // Division always returns double (like Python3)
  EXPECT_DOUBLE_EQ(eval_double("(/ 10 4)"), 2.5);
}

TEST_F(AsyncEvaluatorTest, NestedArithmetic) { EXPECT_EQ(eval_int("(+ (* 2 3) (- 10 4))"), 12); }

// --- Comparison ---

TEST_F(AsyncEvaluatorTest, LessThan) {
  EXPECT_TRUE(eval_bool("(< 1 2)"));
  EXPECT_FALSE(eval_bool("(< 2 1)"));
}

TEST_F(AsyncEvaluatorTest, Equality) {
  EXPECT_TRUE(eval_bool("(= 5 5)"));
  EXPECT_FALSE(eval_bool("(= 5 6)"));
}

// --- Control flow ---

TEST_F(AsyncEvaluatorTest, IfTrue) { EXPECT_EQ(eval_int("(if t 1 2)"), 1); }

TEST_F(AsyncEvaluatorTest, IfFalse) { EXPECT_EQ(eval_int("(if nil 1 2)"), 2); }

TEST_F(AsyncEvaluatorTest, Begin) { EXPECT_EQ(eval_int("(begin 1 2 3)"), 3); }

TEST_F(AsyncEvaluatorTest, SetAndRead) { EXPECT_EQ(eval_int("(begin (set x 42) x)"), 42); }

TEST_F(AsyncEvaluatorTest, While) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                     "  sum)"),
            10);
}

// A loop body of several forms runs them all, in order (they used to be dropped after the first).
TEST_F(AsyncEvaluatorTest, LoopBodiesRunEveryForm) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (set sum (+ sum i)) (set i (+ i 1))) "
                     "  sum)"),
            10);
  EXPECT_EQ(eval_int("(begin (set sum 0) (set n 0) "
                     "  (for x (list 1 2 3 4) (set sum (+ sum x)) (set n (+ n 1))) "
                     "  (+ (* 100 n) sum))"),
            410);
}

TEST_F(AsyncEvaluatorTest, IfAndReturnRejectExtraArguments) {
  EXPECT_THROW(eval("(if t 1 2 3)"), std::runtime_error);
  EXPECT_THROW(eval("(begin (defun f () (return 1 2)) (f))"), std::runtime_error);
}

TEST_F(AsyncEvaluatorTest, For) {
  EXPECT_EQ(eval_int("(begin (set sum 0) "
                     "  (for x (list 1 2 3 4 5) (set sum (+ sum x))) "
                     "  sum)"),
            15);
}

// --- Lambda / closures ---

TEST_F(AsyncEvaluatorTest, LambdaBasic) {
  EXPECT_EQ(eval_int("(begin (set sq (lambda (x) (* x x))) (sq 5))"), 25);
}

TEST_F(AsyncEvaluatorTest, LambdaClosure) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set make-adder (lambda (n) (lambda (x) (+ x n)))) "
                     "  (set add5 (make-adder 5)) "
                     "  (add5 10))"),
            15);
}

TEST_F(AsyncEvaluatorTest, DefunRecursive) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fact (n) "
                     "    (if (< n 2) 1 (* n (fact (- n 1))))) "
                     "  (fact 10))"),
            3628800);
}

// --- Let ---

TEST_F(AsyncEvaluatorTest, LetBasic) { EXPECT_EQ(eval_int("(let ((x 10) (y 20)) (+ x y))"), 30); }

// --- Return ---

TEST_F(AsyncEvaluatorTest, Return) {
  EXPECT_EQ(eval_int("(begin (set f (lambda () (begin (return 42) 99))) (f))"), 42);
}

// --- Quote ---

TEST_F(AsyncEvaluatorTest, Quote) {
  auto result = eval("(quote (1 2 3))");
  ASSERT_TRUE(std::holds_alternative<list_ptr>(result.v));
  auto &lst = *std::get<list_ptr>(result.v);
  EXPECT_EQ(lst.size(), 3u);
}

// --- Macros ---

TEST_F(AsyncEvaluatorTest, Defmacro) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defmacro double-it (x) (* x 2)) "
                     "  (double-it 21))"),
            42);
}

// --- Eval ---

TEST_F(AsyncEvaluatorTest, Eval) { EXPECT_EQ(eval_int("(eval (quote (+ 1 2)))"), 3); }

// --- String ops ---

TEST_F(AsyncEvaluatorTest, StrConcat) {
  EXPECT_EQ(eval_string("(str-concat \"hello\" \" \" \"world\")"), "hello world");
}

// --- List ops ---

TEST_F(AsyncEvaluatorTest, CarCdrCons) {
  EXPECT_EQ(eval_int("(car (list 1 2 3))"), 1);
  EXPECT_EQ(eval_int("(car (cdr (list 1 2 3)))"), 2);
}

TEST_F(AsyncEvaluatorTest, ListLength) { EXPECT_EQ(eval_int("(length (list 1 2 3 4 5))"), 5); }

// --- Fibonacci ---

TEST_F(AsyncEvaluatorTest, Fibonacci) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fib (n) "
                     "    (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
                     "  (fib 15))"),
            610);
}

// --- Await special form ---

TEST_F(AsyncEvaluatorTest, AwaitEvaluatesAndSuspends) {
  // (await expr) should evaluate expr, suspend, then return the value
  EXPECT_EQ(eval_int("(await (+ 1 2))"), 3);
}

// --- Coroutine API ---

TEST_F(AsyncEvaluatorTest, CoroutineAPIDirect) {
  // Use the coroutine API directly
  auto t = ev->evaluate_script("(+ 10 20)");
  auto result = t.sync_wait();
  EXPECT_EQ(std::get<int64_t>(result.v), 30);
}

// ===========================================================================
// async_stackless_evaluator tests
// ===========================================================================

class AsyncStacklessTest : public ::testing::Test {
protected:
  std::unique_ptr<async_stackless_evaluator> ev;
  void SetUp() override {
    ev = std::make_unique<async_stackless_evaluator>(builtins::make_default_environment());
  }

  value_t eval(const std::string &script) { return ev->sync_evaluate_script(script); }

  int64_t eval_int(const std::string &s) { return std::get<int64_t>(eval(s).v); }

  double eval_double(const std::string &s) { return std::get<double>(eval(s).v); }

  bool eval_bool(const std::string &s) { return std::get<bool>(eval(s).v); }

  std::string eval_string(const std::string &s) { return std::get<std::string>(eval(s).v); }
};

// --- Atoms ---

TEST_F(AsyncStacklessTest, IntegerLiteral) { EXPECT_EQ(eval_int("42"), 42); }

TEST_F(AsyncStacklessTest, DoubleLiteral) { EXPECT_DOUBLE_EQ(eval_double("3.14"), 3.14); }

TEST_F(AsyncStacklessTest, StringLiteral) { EXPECT_EQ(eval_string("\"hello\""), "hello"); }

TEST_F(AsyncStacklessTest, NilLiteral) {
  EXPECT_TRUE(std::holds_alternative<std::monostate>(eval("nil").v));
}

TEST_F(AsyncStacklessTest, TrueLiteral) { EXPECT_TRUE(eval_bool("t")); }

// --- Arithmetic ---

TEST_F(AsyncStacklessTest, Addition) { EXPECT_EQ(eval_int("(+ 1 2 3)"), 6); }

TEST_F(AsyncStacklessTest, NestedArithmetic) { EXPECT_EQ(eval_int("(+ (* 2 3) (- 10 4))"), 12); }

// --- Control flow ---

TEST_F(AsyncStacklessTest, IfTrue) { EXPECT_EQ(eval_int("(if t 1 2)"), 1); }

TEST_F(AsyncStacklessTest, Begin) { EXPECT_EQ(eval_int("(begin 1 2 3)"), 3); }

TEST_F(AsyncStacklessTest, SetAndRead) { EXPECT_EQ(eval_int("(begin (set x 42) x)"), 42); }

TEST_F(AsyncStacklessTest, While) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                     "  sum)"),
            10);
}

// A loop body of several forms runs them all, in order (they used to be dropped after the first).
TEST_F(AsyncStacklessTest, LoopBodiesRunEveryForm) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (set sum (+ sum i)) (set i (+ i 1))) "
                     "  sum)"),
            10);
  EXPECT_EQ(eval_int("(begin (set sum 0) (set n 0) "
                     "  (for x (list 1 2 3 4) (set sum (+ sum x)) (set n (+ n 1))) "
                     "  (+ (* 100 n) sum))"),
            410);
}

TEST_F(AsyncStacklessTest, IfAndReturnRejectExtraArguments) {
  EXPECT_THROW(eval("(if t 1 2 3)"), std::runtime_error);
  EXPECT_THROW(eval("(begin (defun f () (return 1 2)) (f))"), std::runtime_error);
}

TEST_F(AsyncStacklessTest, For) {
  EXPECT_EQ(eval_int("(begin (set sum 0) "
                     "  (for x (list 1 2 3 4 5) (set sum (+ sum x))) "
                     "  sum)"),
            15);
}

// --- Lambda / closures ---

TEST_F(AsyncStacklessTest, LambdaBasic) {
  EXPECT_EQ(eval_int("(begin (set sq (lambda (x) (* x x))) (sq 5))"), 25);
}

TEST_F(AsyncStacklessTest, DefunRecursive) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fact (n) "
                     "    (if (< n 2) 1 (* n (fact (- n 1))))) "
                     "  (fact 10))"),
            3628800);
}

// --- Fibonacci ---

TEST_F(AsyncStacklessTest, Fibonacci) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fib (n) "
                     "    (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
                     "  (fib 15))"),
            610);
}

// --- Coroutine step API ---

TEST_F(AsyncStacklessTest, StepByStep) {
  auto state = ev->create_state(std::string("(+ 1 2 3)"));
  int steps = 0;
  while (!state.done) {
    auto t = ev->step(state);
    t.sync_wait();
    steps++;
  }
  EXPECT_TRUE(state.done);
  EXPECT_EQ(std::get<int64_t>(state.result.v), 6);
  EXPECT_GT(steps, 0);
}

TEST_F(AsyncStacklessTest, RunWithMaxSteps) {
  auto state = ev->create_state(std::string("(begin "
                                            "  (defun fib (n) "
                                            "    (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
                                            "  (fib 15))"));
  auto t = ev->run(state, 10); // Only 10 steps
  t.sync_wait();
  // Should NOT be done yet — fib(15) takes many more steps
  EXPECT_FALSE(state.done);
}

TEST_F(AsyncStacklessTest, CoroutineRunToCompletion) {
  auto state = ev->create_state(std::string("(+ 10 20 30)"));
  auto t = ev->run(state);
  auto result = t.sync_wait();
  EXPECT_TRUE(state.done);
  EXPECT_EQ(std::get<int64_t>(result.v), 60);
}

// --- Cross-evaluator consistency ---

TEST(AsyncCrossEvaluatorTest, SyncVsAsyncRecursive) {
  auto sync_ev = std::make_unique<evaluator>(builtins::make_default_environment());
  auto async_ev = std::make_unique<async_evaluator>(builtins::make_default_environment());

  auto programs = {
      "42",
      "3.14",
      "\"hello\"",
      "nil",
      "t",
      "(+ 1 2 3)",
      "(- 10 3)",
      "(* 4 5)",
      "(if t 1 2)",
      "(if nil 1 2)",
      "(begin 1 2 3)",
      "(begin (set x 42) x)",
      "(quote (1 2 3))",
  };

  for (auto &prog : programs) {
    auto sync_result = sync_ev->evaluate_script(prog);
    auto async_result = async_ev->sync_evaluate_script(prog);
    EXPECT_TRUE(values_equal(sync_result, async_result))
        << "Mismatch for: " << prog << "\n  sync:  " << to_string(sync_result)
        << "\n  async: " << to_string(async_result);
  }
}

TEST(AsyncCrossEvaluatorTest, StacklessVsAsyncStackless) {
  auto sync_ev = std::make_unique<stackless_evaluator>(builtins::make_default_environment());
  auto async_ev = std::make_unique<async_stackless_evaluator>(builtins::make_default_environment());

  auto programs = {
      "42",
      "3.14",
      "\"hello\"",
      "nil",
      "t",
      "(+ 1 2 3)",
      "(- 10 3)",
      "(* 4 5)",
      "(if t 1 2)",
      "(begin 1 2 3)",
      "(begin (set x 42) x)",
      "(begin (set i 0) (set s 0) (while (< i 5) (begin (set s (+ s i)) (set i (+ i 1)))) s)",
  };

  for (auto &prog : programs) {
    auto sync_result = sync_ev->evaluate_script(prog);
    auto async_result = async_ev->sync_evaluate_script(prog);
    EXPECT_TRUE(values_equal(sync_result, async_result))
        << "Mismatch for: " << prog << "\n  sync:  " << to_string(sync_result)
        << "\n  async: " << to_string(async_result);
  }
}

TEST(AsyncCrossEvaluatorTest, AllFourEvaluatorsConsistent) {
  auto ev1 = std::make_unique<evaluator>(builtins::make_default_environment());
  auto ev2 = std::make_unique<stackless_evaluator>(builtins::make_default_environment());
  auto ev3 = std::make_unique<async_evaluator>(builtins::make_default_environment());
  auto ev4 = std::make_unique<async_stackless_evaluator>(builtins::make_default_environment());

  auto programs = {
      "(+ 1 2 3)",
      "(begin (set x 10) (set y 20) (+ x y))",
      "(begin (defun fact (n) (if (< n 2) 1 (* n (fact (- n 1))))) (fact 10))",
      "(begin (set sum 0) (for x (list 1 2 3 4 5) (set sum (+ sum x))) sum)",
      "(let ((a 5) (b 10)) (* a b))",
  };

  for (auto &prog : programs) {
    auto r1 = ev1->evaluate_script(prog);
    auto r2 = ev2->evaluate_script(prog);
    auto r3 = ev3->sync_evaluate_script(prog);
    auto r4 = ev4->sync_evaluate_script(prog);

    EXPECT_TRUE(values_equal(r1, r2)) << "ev1 vs ev2 mismatch for: " << prog;
    EXPECT_TRUE(values_equal(r1, r3)) << "ev1 vs ev3 mismatch for: " << prog;
    EXPECT_TRUE(values_equal(r1, r4)) << "ev1 vs ev4 mismatch for: " << prog;
  }
}

// ===========================================================================
// async_scheduler now implements scheduler_base, so the DSL intrinsics
// (spawn/sleep/msg-send/msg-recv/self) drive it identically to the sync
// scheduler.  These exercise the sleep + messaging + current-pid paths that
// were ported onto async_scheduler — at the scheduler API AND the intrinsic
// layer (ictx.sched = &async_scheduler, an upcast to scheduler_base*).
// ===========================================================================

TEST(AsyncSchedulerTest, SpawnRunsToCompletion) {
  async_scheduler sched;
  EXPECT_EQ(sched.current_pid(), -1); // idle
  int pid = sched.execute(std::string("(+ 40 2)"));
  EXPECT_GT(pid, 0);
  auto results = sched.sync_run();
  auto it = results.find(pid);
  ASSERT_NE(it, results.end());
  EXPECT_EQ(std::get<int64_t>(it->second.v), 42);
  EXPECT_EQ(sched.current_pid(), -1); // reset after stepping (guard)
}

TEST(AsyncSchedulerTest, ReceiveMessageSuspendsAndDeliverWakes) {
  async_scheduler sched;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step(); // start the process
  // No receiver waiting: receive_message parks the process in `waiting`.
  ASSERT_TRUE(sched.receive_message(pid, "chan.a"));
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting);
  // Delivery wakes exactly the one waiter and readies it.
  int woken = sched.deliver_to_receivers("chan.a", value_t(std::string("hi")));
  EXPECT_EQ(woken, 1);
  info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::ready);
}

TEST(AsyncSchedulerTest, DeliverWithNoReceiverQueuesAndPops) {
  async_scheduler sched;
  value_t msg{std::string("queued")};
  EXPECT_EQ(sched.deliver_to_receivers("chan.q", msg), 0); // nobody waiting
  EXPECT_EQ(sched.pending_message_count("chan.q"), 1u);
  auto popped = sched.pop_pending_message("chan.q");
  ASSERT_TRUE(popped.has_value());
  EXPECT_EQ(std::get<std::string>(popped->v), "queued");
  EXPECT_EQ(sched.pending_message_count("chan.q"), 0u);
}

TEST(AsyncSchedulerTest, SleepSetsWaitingThenWakesAfterDeadline) {
  async_scheduler sched;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5 6 7 8)"));
  sched.sync_step();                    // start
  ASSERT_TRUE(sched.sleep(pid, 0.005)); // 5ms
  ASSERT_TRUE(sched.get_process_info(pid).has_value());
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  EXPECT_FALSE(sched.has_runnable()); // asleep -> not runnable
  // After the deadline, step() wakes it (wake_sleeping_processes) and it resumes.
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  for (int i = 0; i < 64 && sched.get_process_info(pid) &&
                  sched.get_process_info(pid)->status != process_status::terminated;
       ++i)
    sched.sync_step();
  ASSERT_TRUE(sched.get_process_info(pid).has_value());
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::terminated);
}

// A process's max_time and a run's max_time are time_budgets: time the thread does not run --
// descheduled on a loaded machine, as a native step that sleeps stands in for here -- cannot
// spend their second halves. The Ariadne action lane caps both at 0.1 s and relies on that to
// finish a quick action within its drain: a process whose thread waited 150 ms is neither
// killed nor left for a later run. ((nap) is called in the program's last step, so nothing
// runs after the wait. A program that runs past its limit is still killed:
// AsyncSchedGapsTest.TimeLimitKillsProcess.)
TEST(AsyncSchedulerTest, TimeLimitsAreNotSpentWhileTheThreadDoesNotRun) {
  async_scheduler sched;
  auto env = builtins::make_default_environment();
  builtins::register_fn(env, "nap", [](std::span<const value_t>) {
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    return value_t(int64_t{7});
  });
  execute_options opts;
  opts.env = env;
  opts.max_time = 0.1;
  const int pid = sched.execute(std::string("(nap)"), opts);
  auto results = sched.sync_run(200000, 0.1);
  ASSERT_TRUE(results.count(pid)) << "killed: " << sched.get_exit_error(pid).value_or("no");
  EXPECT_EQ(std::get<int64_t>(results[pid].v), 7);
}

// A run's max_steps counts what a pick charged its process, not the pick (run()'s note): a native
// that charges 1000 steps on top of its own, as a script engine's slice does, spends a 5000-step
// run in about five calls, where plain steps would take five thousand picks.
TEST(AsyncSchedulerTest, ARunsStepBudgetCountsWhatAStepCharged) {
  async_scheduler sched;
  auto env = builtins::make_default_environment();
  int calls = 0;
  builtins::register_fn(env, "heavy", [&calls](std::span<const value_t>) {
    ++calls;
    if (detail::data_step_scope *data = detail::data_step_scope::current())
      data->stats().add_steps(step_kind::data, 1000);
    return value_t();
  });
  execute_options opts;
  opts.env = env;
  const int pid = sched.execute(std::string("(while #t (heavy))"), opts);
  const uint64_t taken = sched.sync_run_steps(5000);
  EXPECT_GE(calls, 4);
  EXPECT_LE(calls, 5);
  EXPECT_GE(taken, 5000u);
  EXPECT_LT(taken, 6100u);
  EXPECT_TRUE(sched.kill(pid));
}

class AsyncSchedulerIntrinsicsTest : public ::testing::Test {
protected:
  cvc::app app_ctx;
  async_scheduler sched;
  memory_tracker tracker;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env;

  void SetUp() override {
    proc->pid = 1;
    proc->status = process_status::running;
    ictx.sched = &sched; // async_scheduler* -> scheduler_base* (the whole point)
    ictx.root = &cvc::state::instance(app_ctx);
    ictx.tracker = &tracker;
    ictx.proc = proc;
    ictx.pid = 1;
    env = builtins::make_default_environment();
    register_intrinsics(env, &ictx);
  }

  value_t call(const std::string &name, std::vector<value_t> args) {
    auto *fn_val = env->lookup(name);
    EXPECT_NE(fn_val, nullptr) << "Function not found: " << name;
    auto *fn = std::get_if<native_fn>(&fn_val->v);
    EXPECT_NE(fn, nullptr) << "Not a function: " << name;
    return (*fn)(std::span<const value_t>(args.data(), args.size()));
  }
};

TEST_F(AsyncSchedulerIntrinsicsTest, SpawnViaIntrinsicCreatesProcess) {
  auto pid_val = call("spawn", {std::string("(+ 1 2)")});
  auto pid = std::get<int64_t>(pid_val.v);
  EXPECT_GT(pid, 0);
  EXPECT_GE(sched.process_count(), 1);
}

TEST_F(AsyncSchedulerIntrinsicsTest, SleepViaIntrinsicSetsWaiting) {
  auto pid_val = call("spawn", {std::string("(begin 1 2 3 4 5)")});
  auto pid = std::get<int64_t>(pid_val.v);
  sched.sync_step();
  ictx.pid = static_cast<int>(pid);
  auto result = call("sleep", {0.01});
  EXPECT_TRUE(std::get<bool>(result.v));
  auto info = sched.get_process_info(static_cast<int>(pid));
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting);
}

TEST_F(AsyncSchedulerIntrinsicsTest, MsgRecvViaIntrinsicSuspendsViaScheduler) {
  auto pid_val = call("spawn", {std::string("(begin 1 2 3 4 5)")});
  auto pid = std::get<int64_t>(pid_val.v);
  sched.sync_step();
  ictx.pid = static_cast<int>(pid);
  auto result = call("msg-recv", {std::string("test.path")});
  EXPECT_TRUE(result.is_nil()); // nil placeholder before delivery
  auto info = sched.get_process_info(static_cast<int>(pid));
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting);
}

TEST_F(AsyncSchedulerIntrinsicsTest, AwaitIntrinsicParksThenResumesWithValue) {
  // A process that awaits mid-program parks at the await (awaiting a frame), then — after
  // wake_awaiting re-readies it — resumes and terminates with the awaited value. There must be
  // work AFTER the await (here `r`), else the program would finish in the await's own step
  // (done=true) and terminate before the parked status matters.
  execute_options opts;
  opts.env = env; // reuse the fixture's intrinsics env (has `await`); await targets the RUNNING
                  // process via sched.current_pid(), not ictx.pid.
  int pid = sched.execute(std::string("(begin (set r (await 42)) r)"), opts);
  for (int i = 0; i < 16 && sched.get_process_info(pid) &&
                  sched.get_process_info(pid)->status != process_status::waiting;
       ++i)
    sched.sync_step();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::waiting); // parked by (await …)
  EXPECT_GE(sched.wake_awaiting(), 1);              // frame boundary re-readies it
  auto results = sched.sync_run();
  auto it = results.find(pid);
  ASSERT_NE(it, results.end());
  EXPECT_EQ(std::get<int64_t>(it->second.v), 42); // resumed with the awaited value
}

// §13.8 per-STEP cap: max_step_time arms a fresh per-step deadline (fresh each step, so it never
// accumulates and cannot kill a long-lived resident) — the SAME eval_deadline_guard mechanism the
// action time-limit uses, just applied to residents so an in-step loop can't hang the drain. This
// checks the wiring + that a NORMAL body under the cap completes untouched (no false-kill); the
// abort-on-overrun behaviour is covered by the existing action time-limit tests (same guard).
TEST_F(AsyncSchedulerIntrinsicsTest, PerStepCapLeavesHealthyWorkUntouched) {
  execute_options opts;
  opts.env = env;
  opts.max_step_time = 0.05; // a generous per-step cap; every normal step is far under it
  int pid =
      sched.execute(std::string("(begin (set s 0) (set i 0) "
                                "(while (< i 200) (begin (set s (+ s i)) (set i (+ i 1)))) s)"),
                    opts);
  auto results = sched.sync_run(2000000, 1.0);
  auto it = results.find(pid);
  ASSERT_NE(it, results.end());                      // completed, not killed
  EXPECT_EQ(std::get<int64_t>(it->second.v), 19900); // sum 0..199
  auto info = sched.get_process_info(pid);
  EXPECT_TRUE(!info.has_value() || info->status != process_status::killed);
}

TEST(AsyncSchedulerTest, KillOwnerReapsProcessGroup) {
  async_scheduler sched;
  execute_options a;
  a.owner = "docA";
  execute_options b;
  b.owner = "docB";
  int a1 = sched.execute(std::string("(begin 1 2 3 4 5)"), a);
  int a2 = sched.execute(std::string("(begin 1 2 3 4 5)"), a);
  int b1 = sched.execute(std::string("(begin 1 2 3 4 5)"), b);
  sched.sync_step(); // start stepping
  // Reap docA's whole group (both processes), leaving docB untouched.
  EXPECT_EQ(sched.kill_owner("docA"), 2);
  EXPECT_EQ(sched.get_process_info(a1)->status, process_status::killed);
  EXPECT_EQ(sched.get_process_info(a2)->status, process_status::killed);
  ASSERT_TRUE(sched.get_process_info(b1).has_value());
  EXPECT_NE(sched.get_process_info(b1)->status, process_status::killed);
  EXPECT_EQ(sched.kill_owner("docA"), 0); // already reaped
}

// S1.4: kill_owner tries every kill. Each of the owner's processes has a call whose cancel hook
// throws, so each kill throws (and still kills, kill_process): every process is killed, the first
// throw is the one rethrown, and the other owner's process is untouched. It used to stop at the
// first throw, leaving the rest running.
TEST(AsyncSchedulerTest, KillOwnerKillsTheRestAfterAThrowingKill) {
  std::vector<int> hooked; // the pids whose cancel hook ran, in order (outlives the hooks)
  async_scheduler sched;
  execute_options a;
  a.owner = "docA";
  std::vector<int> pids;
  for (int i = 0; i < 4; ++i) {
    const int pid = sched.execute(std::string("(begin (set i 0) (while #t (set i (+ i 1))))"), a);
    auto call = std::make_shared<async_call>("probe", pid);
    call->on_cancel([&hooked, pid] {
      hooked.push_back(pid);
      throw std::runtime_error("cancel " + std::to_string(pid));
    });
    sched.track_async_call(call);
    pids.push_back(pid);
  }
  execute_options b;
  b.owner = "docB";
  const int other = sched.execute(std::string("(begin 1 2 3 4 5)"), b);
  sched.sync_step();
  std::string thrown;
  try {
    sched.kill_owner("docA");
  } catch (const std::runtime_error &e) {
    thrown = e.what();
  }
  ASSERT_EQ(hooked.size(), pids.size());
  EXPECT_EQ(thrown, "cancel " + std::to_string(hooked.front()));
  for (const int pid : pids) {
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed) << pid;
    EXPECT_EQ(sched.get_exit_error(pid), "owner_reaped") << pid;
  }
  EXPECT_FALSE(is_finished(sched.get_process_info(other)->status));
  EXPECT_EQ(sched.kill_owner("docA"), 0); // all killed: nothing left to kill, nothing thrown
}

// The pool kills a failed group's processes with its own reason (S1.6): kill(pid, reason) is
// kill(pid) with that exit error. kill(pid) keeps killed_by_user.
TEST(AsyncSchedulerTest, KillWithAReasonSetsTheExitError) {
  async_scheduler sched;
  const int failed = sched.execute(std::string("(begin 1 2 3 4 5)"));
  const int plain = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step();
  EXPECT_TRUE(sched.kill(failed, "internal error"));
  EXPECT_TRUE(sched.kill(plain));
  EXPECT_EQ(sched.get_process_info(failed)->status, process_status::killed);
  EXPECT_EQ(sched.get_exit_error(failed), "internal error");
  EXPECT_EQ(sched.get_exit_error(plain), "killed_by_user");
  EXPECT_FALSE(sched.kill(failed, "internal error")); // finished
  EXPECT_FALSE(sched.kill(999, "internal error"));    // unknown
}

TEST(AsyncSchedulerTest, PostMessageFromAnotherThreadWakesReceiver) {
  async_scheduler sched;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step();                                 // start the process
  ASSERT_TRUE(sched.receive_message(pid, "chan.x")); // park it on the channel
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  // Post from a WORKER thread via the thread-safe ingress (join so the check is deterministic).
  std::thread worker([&] { sched.post_message("chan.x", value_t(std::string("done"))); });
  worker.join();
  // The delivery is applied on the scheduler thread at the next step (drain_ingress), waking it.
  sched.sync_step();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_NE(info->status, process_status::waiting); // woken
}

TEST(AsyncSchedulerTest, YieldFrameParksUntilWakeAwaiting) {
  async_scheduler sched;
  int pid = sched.execute(std::string("(begin 1 2 3 4 5)"));
  sched.sync_step();                   // start it
  ASSERT_TRUE(sched.yield_frame(pid)); // (await …) parks it awaiting a frame
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  EXPECT_FALSE(sched.has_runnable()); // parked -> not runnable this frame
  // A plain step does NOT wake it — only wake_awaiting (called once per frame by the pump) does,
  // so an await crosses exactly one frame rather than resuming within the same pump.
  sched.sync_step();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting);
  EXPECT_EQ(sched.wake_awaiting(), 1); // frame boundary
  EXPECT_NE(sched.get_process_info(pid)->status, process_status::waiting);
}

// The app-wide scheduler service: cvc::app::exec_scheduler() is one lazily-built
// per-app async_scheduler (the single cooperative timeline all Ariadne docs share).
TEST(AppExecScheduler, IsAppWideSingletonAndUsable) {
  cvc::app app_ctx;
  auto &s1 = app_ctx.exec_scheduler();
  auto &s2 = app_ctx.exec_scheduler();
  EXPECT_EQ(&s1, &s2); // same instance across calls — one app-wide scheduler
  int pid = s1.execute(std::string("(+ 2 3)"));
  auto results = s1.sync_run();
  auto it = results.find(pid);
  ASSERT_NE(it, results.end());
  EXPECT_EQ(std::get<int64_t>(it->second.v), 5);
}

// post_control (docs/STATE_EXEC_LOCKS.md 7.4; locks plan L2.6): a command posted from another
// thread is applied by the scheduler's next drain, between steps, in the order posted, and wakes a
// driver blocked in wait_for_work(). set_priority goes through the host path (unchecked; an
// unknown pid is ignored) and a query runs on the scheduler's thread.
TEST(AsyncPostControl, AppliedByTheNextDrainFromAnyThread) {
  async_scheduler sched(scheduling_policy::priority_rr);
  const int pid = sched.execute(std::string("(+ 1 2)"));
  std::thread::id query_thread;
  int seen = -100;
  std::thread poster([&] {
    control_cmd set;
    set.what = control_cmd::kind::set_priority;
    set.value = 4;
    EXPECT_TRUE(sched.post_control(pid, std::move(set)));
    control_cmd unknown;
    unknown.what = control_cmd::kind::set_priority;
    unknown.value = 1;
    EXPECT_TRUE(sched.post_control(9999, std::move(unknown))); // queued: the pid is looked up later
    control_cmd query;
    query.what = control_cmd::kind::query;
    query.query = [&](scheduler_base &s) {
      query_thread = std::this_thread::get_id();
      seen = s.get_priority(pid)->base;
    };
    EXPECT_TRUE(sched.post_control(pid, std::move(query)));
  });
  poster.join();
  EXPECT_EQ(sched.get_priority(pid)->base, 0); // not applied on the posting thread
  sched.drain_ingress();
  EXPECT_EQ(sched.get_priority(pid)->base, 4);
  EXPECT_EQ(seen, 4); // after the set_priority posted before it
  EXPECT_EQ(query_thread, std::this_thread::get_id());

  // A step drains too.
  control_cmd again;
  again.what = control_cmd::kind::set_priority;
  again.value = 2;
  sched.post_control(pid, std::move(again));
  EXPECT_EQ(sched.sync_step(), 1);
  EXPECT_EQ(sched.get_priority(pid)->base, 2);

  // A driver blocked in wait_for_work() with nothing to do wakes for a post.
  async_scheduler idle;
  bool ran = false;
  std::thread later([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    control_cmd query;
    query.what = control_cmd::kind::query;
    query.query = [&](scheduler_base &) { ran = true; };
    idle.post_control(-1, std::move(query));
  });
  const auto t0 = std::chrono::steady_clock::now();
  EXPECT_TRUE(idle.wait_for_work(10.0));
  const double waited =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  later.join();
  EXPECT_LT(waited, 10.0) << "wait_for_work woke after " << waited << " s";
  EXPECT_FALSE(ran);
  idle.drain_ingress();
  EXPECT_TRUE(ran);
}

// The synchronous scheduler's post_control: queued under its own mutex and applied at the top of
// its next step, on the stepping thread.
TEST(SyncPostControl, AppliedAtTheTopOfTheNextStep) {
  scheduler sched(scheduling_policy::priority_rr);
  const int pid = sched.execute(std::string("(begin (set i 0) (while (< i 100) (set i (+ i 1))))"));
  std::thread::id query_thread;
  std::thread poster([&] {
    control_cmd set;
    set.what = control_cmd::kind::set_priority;
    set.value = -3;
    EXPECT_TRUE(sched.post_control(pid, std::move(set)));
    control_cmd query;
    query.what = control_cmd::kind::query;
    query.query = [&](scheduler_base &) { query_thread = std::this_thread::get_id(); };
    EXPECT_TRUE(sched.post_control(pid, std::move(query)));
  });
  poster.join();
  EXPECT_EQ(sched.get_priority(pid)->base, 0);
  EXPECT_EQ(sched.step(), 1);
  EXPECT_EQ(sched.get_priority(pid)->base, -3);
  EXPECT_EQ(query_thread, std::this_thread::get_id());
}

// --- post_control's Stage 0 kinds (td 4.5, S0.7) ---

namespace {

// The done callbacks of posted commands: what each one got, in the order they ran, and where.
struct done_log {
  std::vector<std::pair<std::string, bool>> calls;
  std::vector<std::thread::id> threads;
  std::function<void(bool)> done(std::string what) {
    return [this, what = std::move(what)](bool applied) {
      calls.emplace_back(what, applied);
      threads.push_back(std::this_thread::get_id());
    };
  }
};

control_cmd control(control_cmd::kind what, std::function<void(bool)> done = nullptr,
                    std::string signal = {}) {
  control_cmd cmd;
  cmd.what = what;
  cmd.done = std::move(done);
  cmd.signal = std::move(signal);
  return cmd;
}

control_cmd throwing_query() {
  control_cmd cmd;
  cmd.what = control_cmd::kind::query;
  cmd.query = [](scheduler_base &) { throw std::runtime_error("the query threw"); };
  return cmd;
}

using done_calls = std::vector<std::pair<std::string, bool>>;

// Kill, pause, resume and signal posted from another thread: nothing changes until `apply` (a
// drain or a step) runs on this thread; then each lands, in the order posted, and its done gets
// what the direct call returns: true, or false for an unknown pid and for pausing a waiting
// process.
template <class S>
void kill_pause_resume_signal(S &sched, done_log &log, const std::function<void()> &apply) {
  const std::string loop = "(begin (set i 0) (while (< i 1000000) (set i (+ i 1))))";
  const int a = sched.execute(loop); // paused, resumed, paused again
  const int b = sched.execute(loop); // paused already; resumed
  const int c = sched.execute(loop); // killed
  const int d = sched.execute(loop); // signalled SIGKILL
  const int e = sched.execute(loop); // signalled SIGUSR1, which it has no handler for
  const int w = sched.execute(loop); // waiting for a message: a pause is refused
  ASSERT_TRUE(sched.pause(b));
  ASSERT_TRUE(sched.receive_message(w, "never"));
  using k = control_cmd::kind;
  std::thread poster([&] {
    EXPECT_TRUE(sched.post_control(a, control(k::pause, log.done("pause a"))));
    EXPECT_TRUE(sched.post_control(b, control(k::resume, log.done("resume b"))));
    EXPECT_TRUE(sched.post_control(c, control(k::kill, log.done("kill c"))));
    EXPECT_TRUE(sched.post_control(d, control(k::signal, log.done("SIGKILL d"), "SIGKILL")));
    EXPECT_TRUE(sched.post_control(e, control(k::signal, log.done("SIGUSR1 e"), "SIGUSR1")));
    EXPECT_TRUE(sched.post_control(w, control(k::pause, log.done("pause w"))));
    EXPECT_TRUE(sched.post_control(9999, control(k::kill, log.done("kill 9999"))));
    // First in, first out: applied in the other order, the pause would find a paused process.
    EXPECT_TRUE(sched.post_control(a, control(k::resume, log.done("resume a"))));
    EXPECT_TRUE(sched.post_control(a, control(k::pause, log.done("pause a again"))));
  });
  poster.join();
  // Not applied on the posting thread.
  EXPECT_TRUE(sched.has_pending_control());
  EXPECT_EQ(sched.get_process_info(a)->status, process_status::ready);
  EXPECT_EQ(sched.get_process_info(b)->status, process_status::paused);
  EXPECT_EQ(sched.get_process_info(c)->status, process_status::ready);
  EXPECT_EQ(sched.get_process_info(d)->status, process_status::ready);
  EXPECT_TRUE(log.calls.empty());

  apply();
  EXPECT_FALSE(sched.has_pending_control());
  EXPECT_EQ(sched.get_process_info(a)->status, process_status::paused);
  EXPECT_EQ(sched.get_process_info(b)->status, process_status::ready);
  EXPECT_EQ(sched.get_process_info(c)->status, process_status::killed);
  EXPECT_EQ(sched.get_exit_error(c), "killed_by_user");
  EXPECT_EQ(sched.get_process_info(d)->status, process_status::killed);
  EXPECT_EQ(sched.get_exit_error(d), "SIGKILL");
  EXPECT_FALSE(is_finished(sched.get_process_info(e)->status));
  EXPECT_EQ(sched.get_process_info(w)->status, process_status::waiting);
  EXPECT_EQ(log.calls, (done_calls{{"pause a", true},
                                   {"resume b", true},
                                   {"kill c", true},
                                   {"SIGKILL d", true},
                                   {"SIGUSR1 e", true},
                                   {"pause w", false},
                                   {"kill 9999", false},
                                   {"resume a", true},
                                   {"pause a again", true}}));
  for (const std::thread::id &t : log.threads)
    EXPECT_EQ(t, std::this_thread::get_id()); // the done callbacks ran on the scheduler's thread
  EXPECT_TRUE(sched.check_tables());
}

// A command that throws (a query; a kill can, through its locks' release or a tree write's slots,
// and still kills) stops the batch, and the commands after it go back to the front of the queue:
// the next drain or step applies them, in order, ahead of what was posted since.
template <class S>
void a_throwing_command_requeues_the_rest(S &sched, done_log &log, int thrower_pid,
                                          control_cmd thrower, const done_calls &first,
                                          const std::function<void()> &apply) {
  const std::string loop = "(begin (set i 0) (while (< i 1000000) (set i (+ i 1))))";
  const int a = sched.execute(loop);
  const int b = sched.execute(loop);
  using k = control_cmd::kind;
  sched.post_control(a, control(k::pause, log.done("pause a")));
  sched.post_control(thrower_pid, std::move(thrower));
  sched.post_control(b, control(k::kill, log.done("kill b")));
  sched.post_control(a, control(k::resume, log.done("resume a")));
  EXPECT_THROW(apply(), std::runtime_error);
  EXPECT_EQ(log.calls, first);
  EXPECT_EQ(sched.get_process_info(a)->status, process_status::paused);
  EXPECT_EQ(sched.get_process_info(b)->status, process_status::ready) << "applied past the throw";
  EXPECT_TRUE(sched.has_pending_control()) << "the commands after the throw were lost";
  sched.post_control(a, control(k::pause, log.done("pause a, posted after the throw")));
  apply();
  done_calls all = first;
  all.insert(all.end(),
             {{"kill b", true}, {"resume a", true}, {"pause a, posted after the throw", true}});
  EXPECT_EQ(log.calls, all);
  EXPECT_EQ(sched.get_process_info(b)->status, process_status::killed);
  EXPECT_EQ(sched.get_process_info(a)->status, process_status::paused);
  EXPECT_FALSE(sched.has_pending_control());
  EXPECT_TRUE(sched.check_tables());
}

// A scheduler destroyed with commands still queued gives each one's done false, once; a query
// left in the queue does not run, and has no done.
template <class S> void the_destructor_refuses_what_was_never_applied() {
  done_log log;
  bool queried = false;
  {
    S sched;
    const int pid = sched.execute(std::string("(+ 1 2)"));
    using k = control_cmd::kind;
    sched.post_control(pid, control(k::kill, log.done("kill")));
    sched.post_control(9999, control(k::pause, log.done("pause 9999")));
    control_cmd query = control(k::query, log.done("query"));
    query.query = [&queried](scheduler_base &) { queried = true; };
    sched.post_control(pid, std::move(query));
    sched.post_control(pid, control(k::signal, log.done("signal"), "SIGUSR1"));
    EXPECT_TRUE(log.calls.empty());
  }
  EXPECT_EQ(log.calls, (done_calls{{"kill", false}, {"pause 9999", false}, {"signal", false}}));
  EXPECT_FALSE(queried);
}

// R0-final's R0a-mem-6: a done the destructor calls (with false) may post another command; that
// one is refused too, once, rather than destroyed with its done never called.
template <class S> void a_post_from_a_refused_done_is_refused_too() {
  done_log log;
  {
    S sched;
    S *const raw = &sched;
    sched.post_control(1, control(control_cmd::kind::kill, [raw, &log](bool applied) {
                         log.calls.emplace_back("kill", applied);
                         raw->post_control(2, control(control_cmd::kind::pause, log.done("pause")));
                       }));
  }
  EXPECT_EQ(log.calls, (done_calls{{"kill", false}, {"pause", false}}));
}

} // namespace

TEST(AsyncPostControl, KillPauseResumeSignalFromAnyThread) {
  done_log first, log; // before the scheduler: its destructor may call what was never applied
  async_scheduler sched;
  kill_pause_resume_signal(sched, first, [&] { sched.drain_ingress(); });
  // A step drains too.
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.post_control(pid, control(control_cmd::kind::kill, log.done("kill")));
  sched.sync_step();
  EXPECT_EQ(sched.get_exit_error(pid), "killed_by_user");
  EXPECT_EQ(log.calls, (done_calls{{"kill", true}}));
}

// has_pending_control() is about commands only: a posted message is not one. A drain applies its
// messages and completions before its commands, whatever order they were posted in (S0.7).
TEST(AsyncPostControl, HasPendingControlCountsCommandsOnly) {
  async_scheduler sched;
  const int pid = sched.execute(std::string("(+ 1 2)"));
  ASSERT_TRUE(sched.receive_message(pid, "chan"));
  process_status seen = process_status::paused;
  control_cmd query;
  query.what = control_cmd::kind::query;
  query.query = [&](scheduler_base &s) { seen = s.get_process_info(pid)->status; };
  sched.post_message("other", value_t(int64_t(1)));
  EXPECT_FALSE(sched.has_pending_control()) << "a message counted as a command";
  sched.post_control(pid, std::move(query)); // posted before the message it will see delivered
  sched.post_message("chan", value_t(int64_t(2)));
  EXPECT_TRUE(sched.has_pending_control());
  sched.drain_ingress();
  EXPECT_FALSE(sched.has_pending_control());
  EXPECT_EQ(seen, process_status::ready); // the message was delivered first
}

TEST(AsyncPostControl, AThrowingCommandRequeuesTheRest) {
  done_log log, rest; // before the scheduler: its destructor may call what was never applied
  async_scheduler sched;
  // A kill whose async call's cancel hook throws: the kill throws, and the commands after it are
  // requeued, but the process is killed all the same (a kill always kills, R0-final), and the
  // kill's done hears so: true.
  const int c = sched.execute(std::string("(begin (set i 0) (while #t (set i (+ i 1))))"));
  auto call = std::make_shared<async_call>("probe", c);
  call->on_cancel([] { throw std::runtime_error("the cancel hook threw"); });
  sched.track_async_call(call);
  a_throwing_command_requeues_the_rest(sched, rest, c,
                                       control(control_cmd::kind::kill, log.done("kill c")),
                                       {{"pause a", true}}, [&] { sched.drain_ingress(); });
  EXPECT_EQ(log.calls, (done_calls{{"kill c", true}}));
  EXPECT_EQ(sched.get_process_info(c)->status, process_status::killed);
  EXPECT_EQ(sched.get_exit_error(c), "killed_by_user");
  EXPECT_FALSE(sched.kill(c)); // killed already
}

TEST(AsyncPostControl, TheDestructorRefusesWhatWasNeverApplied) {
  the_destructor_refuses_what_was_never_applied<async_scheduler>();
}

TEST(SyncPostControl, KillPauseResumeSignalFromAnyThread) {
  {
    done_log log; // before the scheduler: its destructor may call what was never applied
    scheduler sched;
    kill_pause_resume_signal(sched, log, [&] { EXPECT_EQ(sched.step(), 1); });
  }
  // run() applies what was posted before it tests has_runnable(): a resume of the only process,
  // which is paused, runs it.
  {
    scheduler sched;
    const int pid = sched.execute(std::string("(+ 1 2)"));
    ASSERT_TRUE(sched.pause(pid));
    ASSERT_FALSE(sched.has_runnable());
    sched.post_control(pid, control(control_cmd::kind::resume));
    sched.run();
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::terminated);
  }
  // A kill posted at 50 ms ends a run() whose only process sleeps for 5 s. The run's time budget
  // (4 s) ends first otherwise, with the kill still queued: the kill is applied only if the post
  // ended the idle wait.
  {
    done_log log; // before the scheduler: its destructor may call what was never applied
    scheduler sched;
    const int pid = sched.execute(std::string("(begin 1 2 3)"));
    sched.step();
    ASSERT_TRUE(sched.sleep(pid, 5.0));
    std::thread poster([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      sched.post_control(pid, control(control_cmd::kind::kill, log.done("kill")));
    });
    const auto t0 = std::chrono::steady_clock::now();
    sched.run(std::nullopt, 4.0);
    const double took =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    poster.join();
    std::printf("run() beside a 5 s sleeper, a kill posted after 50 ms, returned after %.3f s\n",
                took);
    EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
    EXPECT_EQ(sched.get_exit_error(pid), "killed_by_user");
    EXPECT_EQ(log.calls, (done_calls{{"kill", true}}));
    EXPECT_LT(took, 4.0);
    EXPECT_TRUE(sched.check_tables());
  }
}

// A command posted while run() waits idle (beside a 5 s sleeper, with no budget) ends the wait,
// and run() applies it: here a query that stops the run. The sleeper is still asleep when run()
// returns, so the post, not the sleeper's deadline, ended the wait.
TEST(SyncPostControl, APostedControlEndsTheIdleWait) {
  scheduler sched;
  const int pid = sched.execute(std::string("(begin 1 2 3)"));
  sched.step();
  ASSERT_TRUE(sched.sleep(pid, 5.0));
  std::thread::id query_thread;
  std::thread poster([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    control_cmd stop;
    stop.what = control_cmd::kind::query;
    stop.query = [&](scheduler_base &) {
      query_thread = std::this_thread::get_id();
      sched.stop();
    };
    sched.post_control(pid, std::move(stop));
  });
  const auto t0 = std::chrono::steady_clock::now();
  sched.run();
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  poster.join();
  std::printf("run() beside a 5 s sleeper, a stop posted after 50 ms, returned after %.3f s\n",
              took);
  EXPECT_EQ(query_thread, std::this_thread::get_id());
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::waiting) << "the sleeper woke";
  EXPECT_FALSE(sched.is_running());
  EXPECT_TRUE(sched.check_tables());
}

TEST(SyncPostControl, AThrowingCommandRequeuesTheRest) {
  done_log log; // before the scheduler: its destructor may call what was never applied
  scheduler sched;
  a_throwing_command_requeues_the_rest(sched, log, -1, throwing_query(), {{"pause a", true}},
                                       [&] { sched.step(); });
}

TEST(SyncPostControl, TheDestructorRefusesWhatWasNeverApplied) {
  the_destructor_refuses_what_was_never_applied<scheduler>();
}

TEST(AsyncPostControl, APostFromARefusedDoneIsRefusedToo) {
  a_post_from_a_refused_done_is_refused_too<async_scheduler>();
}

TEST(SyncPostControl, APostFromARefusedDoneIsRefusedToo) {
  a_post_from_a_refused_done_is_refused_too<scheduler>();
}

namespace {

// R0-final's R0a-conc-4: has_pending_control() from another thread while the scheduler's thread
// applies a batch: a query that holds the batch, then a pause still to come. The pause waits to be
// applied, so the flag reads true until the batch is done (it read false once the batch was taken).
template <class S>
void pending_while_a_batch_applies(S &sched, const std::function<void()> &apply) {
  const int pid =
      sched.execute(std::string("(begin (set i 0) (while (< i 1000000) (set i (+ i 1))))"));
  std::atomic<bool> in_query{false}, gate{false}, paused{false};
  control_cmd hold;
  hold.what = control_cmd::kind::query;
  hold.query = [&](scheduler_base &) {
    in_query = true;
    while (!gate.load())
      std::this_thread::yield();
  };
  sched.post_control(-1, std::move(hold));
  sched.post_control(pid, control(control_cmd::kind::pause, [&](bool ok) { paused = ok; }));
  std::thread pump(apply);
  while (!in_query.load())
    std::this_thread::yield();
  const bool pending = sched.has_pending_control(); // the pause is still to be applied
  gate = true;
  pump.join();
  EXPECT_TRUE(pending) << "has_pending_control() read false with a pause waiting in the batch";
  EXPECT_TRUE(paused.load());
  EXPECT_FALSE(sched.has_pending_control());
}

// R0-final's R0a-conc-5: a done that throws (against its contract) is called once: the refusal of
// the command that threw found its done still set and called it again, with false.
template <class S> int done_calls_when_a_done_throws(S &sched, const std::function<void()> &apply) {
  const int pid =
      sched.execute(std::string("(begin (set i 0) (while (< i 1000000) (set i (+ i 1))))"));
  int calls = 0;
  sched.post_control(pid, control(control_cmd::kind::pause, [&calls](bool) {
                       if (++calls == 1)
                         throw std::runtime_error("the done threw");
                     }));
  EXPECT_THROW(apply(), std::runtime_error);
  return calls;
}

} // namespace

TEST(AsyncPostControl, HasPendingControlWhileABatchApplies) {
  async_scheduler sched;
  pending_while_a_batch_applies(sched, [&] { sched.drain_ingress(); });
}

TEST(SyncPostControl, HasPendingControlWhileABatchApplies) {
  scheduler sched;
  pending_while_a_batch_applies(sched, [&] { sched.step(); });
}

TEST(AsyncPostControl, ADoneThatThrowsIsCalledOnce) {
  async_scheduler sched;
  EXPECT_EQ(done_calls_when_a_done_throws(sched, [&] { sched.drain_ingress(); }), 1);
}

TEST(SyncPostControl, ADoneThatThrowsIsCalledOnce) {
  scheduler sched;
  EXPECT_EQ(done_calls_when_a_done_throws(sched, [&] { sched.step(); }), 1);
}

namespace {

// The skeptic passes' SK-4 and R2-2: once the destructor has taken the queue, a post (here from
// the done it refuses, which posts again each time, 100000 times, as a done that retries until
// applied would) returns false, is never applied, and is refused, with done(false), by the
// destructor's next round: the refusals iterate. When the refusal was made inside post_control,
// they nested one level a repost, and ran out of stack (about 1800 on a 1 MiB thread, Release).
// On a thread of its own, whose stack is the default 1 MiB on Windows.
template <class S> void the_destructor_refuses_every_repost() {
  constexpr long kReposts = 100000;
  long refused = 0, accepted = 0;
  std::thread([&] {
    std::function<void(bool)> again;
    S *target = nullptr;
    again = [&](bool applied) {
      EXPECT_FALSE(applied);
      if (++refused < kReposts)
        accepted += target->post_control(1, control(control_cmd::kind::kill, again)) ? 1 : 0;
    };
    S sched;
    target = &sched;
    EXPECT_TRUE(sched.post_control(1, control(control_cmd::kind::kill, again)));
  }).join();
  EXPECT_EQ(refused, kReposts);
  EXPECT_EQ(accepted, 0) << "a post made while the destructor ran was taken as queued to apply";
}

// has_pending_control() inside the destructor's refusals: each command is counted out as it is
// refused, so the first refusal's done sees the second still waiting, and the last sees none
// (the skeptic's round 2: the destructor never lowered the count).
template <class S> void the_destructor_counts_out_what_it_refuses() {
  std::vector<bool> seen;
  {
    S sched;
    S *const raw = &sched;
    const auto look = [&seen, raw](bool) { seen.push_back(raw->has_pending_control()); };
    sched.post_control(1, control(control_cmd::kind::kill, look));
    sched.post_control(2, control(control_cmd::kind::kill, look));
  }
  EXPECT_EQ(seen, (std::vector<bool>{true, false}));
}

// has_pending_control() counts the commands posted and not yet applied or refused (the skeptic
// pass's note on its flag): a query in a batch sees the command after it waiting; a command that
// throws last in its batch is refused, and nothing waits; one that throws before another leaves
// that one waiting, back in the queue, until the next drain or step applies it.
template <class S> void has_pending_control_counts_what_waits(void (*apply)(S &)) {
  done_log log; // before the scheduler: its destructor may call what was never applied
  S sched;
  const int pid =
      sched.execute(std::string("(begin (set i 0) (while (< i 1000000) (set i (+ i 1))))"));
  bool seen = false;
  control_cmd look;
  look.what = control_cmd::kind::query;
  look.query = [&](scheduler_base &) { seen = sched.has_pending_control(); };
  sched.post_control(-1, std::move(look));
  sched.post_control(pid, control(control_cmd::kind::pause, log.done("pause")));
  apply(sched);
  EXPECT_TRUE(seen) << "the pause after the query didn't count";
  EXPECT_FALSE(sched.has_pending_control());

  sched.post_control(-1, throwing_query());
  EXPECT_THROW(apply(sched), std::runtime_error);
  EXPECT_FALSE(sched.has_pending_control()) << "the refused command still counted";

  sched.post_control(-1, throwing_query());
  sched.post_control(pid, control(control_cmd::kind::resume, log.done("resume")));
  EXPECT_THROW(apply(sched), std::runtime_error);
  EXPECT_TRUE(sched.has_pending_control()) << "the requeued resume didn't count";
  apply(sched);
  EXPECT_FALSE(sched.has_pending_control());
  EXPECT_EQ(log.calls, (done_calls{{"pause", true}, {"resume", true}}));
}

} // namespace

TEST(AsyncPostControl, TheDestructorRefusesEveryRepost) {
  the_destructor_refuses_every_repost<async_scheduler>();
}

TEST(SyncPostControl, TheDestructorRefusesEveryRepost) {
  the_destructor_refuses_every_repost<scheduler>();
}

TEST(AsyncPostControl, TheDestructorCountsOutWhatItRefuses) {
  the_destructor_counts_out_what_it_refuses<async_scheduler>();
}

TEST(SyncPostControl, TheDestructorCountsOutWhatItRefuses) {
  the_destructor_counts_out_what_it_refuses<scheduler>();
}

TEST(AsyncPostControl, HasPendingControlCountsWhatWaits) {
  has_pending_control_counts_what_waits<async_scheduler>(
      [](async_scheduler &s) { s.drain_ingress(); });
}

TEST(SyncPostControl, HasPendingControlCountsWhatWaits) {
  has_pending_control_counts_what_waits<scheduler>([](scheduler &s) { s.step(); });
}

// --- an async call's outcome (docs/STATE_EXEC_EXCEPTIONS.md 3) ---

// A kernel that throws a dsl_exception holding a built-in class dict itself is raised as it is:
// the class is read-only (td 5 item 1), so outcome() adds no origin or function to it, as before
// S0.1, rather than raising a TypeError from the read-only class.
TEST(AsyncCallOutcome, AKernelRaisingABuiltinClassDictRaisesItUnchanged) {
  const dict_ptr cls = exception_class("ValueError");
  const std::size_t size = cls->size();
  auto call = std::make_shared<async_call>("probe", 1);
  ASSERT_TRUE(call->settle({}, std::make_exception_ptr(dsl_exception(value_t(cls)))));
  try {
    call->outcome();
    ADD_FAILURE() << "the call's outcome returned";
  } catch (const dsl_exception &e) {
    const dict_ptr *raised = std::get_if<dict_ptr>(&e.value().v);
    ASSERT_NE(raised, nullptr);
    EXPECT_EQ(raised->get(), cls.get());
  } catch (const std::exception &e) {
    ADD_FAILURE() << "raised " << e.what() << " instead of the class";
  }
  EXPECT_EQ(cls->size(), size);
  EXPECT_EQ(field(value_t(cls), "origin"), nullptr);
  EXPECT_EQ(field(value_t(cls), "function"), nullptr);
}

// --- a future is finished once (td 4.9, S0.9) ---

// A call cancelled before it settled (the process that made it was killed or reaped) raises
// RuntimeError to whoever reads its outcome since: a fork child awaiting the same future, whom
// the cancel wakes. It used to read nil. Once settled, the outcome is the settlement.
TEST(AsyncCallOutcome, ACallCancelledBeforeItSettledRaisesRuntimeError) {
  auto call = std::make_shared<async_call>("probe", 1);
  call->cancel();
  try {
    call->outcome();
    ADD_FAILURE() << "a cancelled call's outcome returned";
  } catch (const dsl_error &e) {
    EXPECT_EQ(e.class_name(), "RuntimeError");
    EXPECT_NE(std::string(e.what()).find("probe: cancelled"), std::string::npos) << e.what();
  }
  ASSERT_TRUE(call->settle(value_t(int64_t(7)), nullptr));
  EXPECT_EQ(std::get<int64_t>(call->outcome().v), 7);
}

// --- Stage 0 hazards (td 8.1) ---

// HZ-14 (td 8.1, S0.6; TSan), the async scheduler: stop() from another thread ends a
// sync_run_steps() stepping a busy loop. The process is left as it was and runs on in the next
// run, and the tables hold.
TEST(AsyncSchedulerStop, HZ14_StopFromAnotherThreadEndsSyncRun) {
  async_scheduler sched;
  execute_options opts;
  opts.max_time = 60.0; // a stop that is lost ends the run here instead, and fails the case
  const int pid = sched.execute(std::string("(while #t nil)"), opts);
  bool saw_running = false;
  std::chrono::steady_clock::time_point stopped_at{};
  std::thread stopper([&] {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!sched.is_running() && std::chrono::steady_clock::now() < deadline)
      std::this_thread::yield();
    saw_running = sched.is_running();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    stopped_at = std::chrono::steady_clock::now();
    sched.stop();
  });
  const auto t0 = std::chrono::steady_clock::now();
  const uint64_t taken = sched.sync_run_steps();
  const auto t1 = std::chrono::steady_clock::now();
  stopper.join();
  std::printf(
      "HZ-14 async busy loop: %llu steps; run returned %.4f s after stop(), %.3f s in all\n",
      static_cast<unsigned long long>(taken),
      std::chrono::duration<double>(t1 - stopped_at).count(),
      std::chrono::duration<double>(t1 - t0).count());
  EXPECT_TRUE(saw_running);
  EXPECT_FALSE(sched.is_running());
  EXPECT_GT(taken, 0u);
  const auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_EQ(info->status, process_status::ready);
  EXPECT_TRUE(sched.check_tables());
  EXPECT_LT(std::chrono::duration<double>(t1 - t0).count(), 30.0);
  EXPECT_EQ(sched.sync_run_steps(100), 100u); // it runs on
  EXPECT_TRUE(sched.kill(pid));
}

// --- Stage 1 groups (td 8.2) ---

// The includes the cases below add to the file's own.
#include <atomic>
#include <cvc/state/state_exec/async_host.h>
#include <memory>

namespace {

// kill_owner's world (S1.4): an async scheduler with an id, so the step rate a kill reads to
// charge a cancelled call is published to the tree under state_exec.schedulers (a write whose
// slots run then), and (hold), an async function on the app's background pool that runs until its
// call is cancelled. Processes are submitted with the owner "doc".
struct kill_owner_world {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  async_scheduler sched;
  std::unique_ptr<async_host> host;
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();

  explicit kill_owner_world(const std::string &id) {
    sched.set_watch_root(&tree);
    sched.set_id(id);
    host = std::make_unique<async_host>(sched, &app);
    ictx.sched = &sched;
    ictx.root = &tree;
    ictx.proc = make_process();
    register_intrinsics(env, &ictx);
    register_async_fn(env, *host, "hold", [](std::span<const value_t>) -> async_kernel {
      return [](async_call &call) -> value_t {
        while (!call.cancelled())
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        throw std::runtime_error("hold: cancelled");
      };
    });
  }
  ~kill_owner_world() { host->shutdown(); } // before the scheduler it posts to

  int submit(const std::string &script) {
    execute_options opts;
    opts.env = env;
    opts.owner = "doc";
    opts.max_time = 60.0;
    return sched.execute(script, opts);
  }
  // `pid`'s status and exit error, for a failure's message.
  std::string describe(int pid) const {
    const auto i = sched.get_process_info(pid);
    return "pid " + std::to_string(pid) + ": " + (i ? to_string(i->status) : "gone") + " " +
           sched.get_exit_error(pid).value_or("");
  }
  std::optional<process_status> status(int pid) const {
    const auto i = sched.get_process_info(pid);
    return i ? std::optional<process_status>(i->status) : std::nullopt;
  }
  // Pump until `done()`, for at most 10 s; whether it held.
  template <class Done> bool pump_until(Done done) {
    const auto t0 = std::chrono::steady_clock::now();
    while (!done()) {
      if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(10))
        return false;
      sched.drain_ingress();
      sched.sync_run_steps(100000, 0.02);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
  }
  // A new rate in the tree, so the next read of it publishes it.
  void change_rate() {
    tree("state_exec.defaults.limits.async_steps_per_cpu_ms").value(std::string("2000"));
  }
};

} // namespace

// KO-1 (S1.4): kill_owner over a snapshot. The owner has a finished process, two parked on
// msg-recv and one parked on a (hold) call. Killing that one cancels its call and reads the rate,
// whose publication runs a tree slot that reaps the finished process and submits another of the
// owner's. Every process of the owner alive at the call is killed, the count is the three it
// killed, and the process submitted during it is not among them. Before, the loop walked the live
// process table that the slot erased from and inserted into.
TEST(AsyncScheduler, KO01_KillOwnerSurvivesSlotsThatReapAndSubmit) {
  kill_owner_world w("ko01");
  const int done = w.submit("1");
  ASSERT_TRUE(w.pump_until([&] { return w.status(done) == process_status::terminated; }));
  const int a = w.submit(R"((begin (msg-recv "ko01.a") 1))");
  const int b = w.submit(R"((begin (msg-recv "ko01.b") 1))");
  const int holder = w.submit("(hold)");
  ASSERT_TRUE(w.pump_until([&] {
    return w.status(a) == process_status::waiting && w.status(b) == process_status::waiting &&
           w.status(holder) == process_status::waiting;
  })) << w.describe(a)
      << "; " << w.describe(b) << "; " << w.describe(holder);
  w.change_rate();
  std::atomic<bool> armed{true};
  bool reaped = false;
  int submitted = -1;
  boost::signals2::scoped_connection slot =
      w.tree("state_exec.schedulers").childChanged.connect([&](const std::string &) {
        if (!armed.exchange(false))
          return;
        reaped = w.sched.reap(done);
        submitted = w.submit(R"((begin (msg-recv "ko01.c") 1))");
      });
  const int killed = w.sched.kill_owner("doc");
  slot.disconnect();
  EXPECT_FALSE(armed.load()) << "the rate's publication ran no slot";
  EXPECT_TRUE(reaped);
  EXPECT_EQ(killed, 3);
  for (const int pid : {a, b, holder}) {
    EXPECT_EQ(w.status(pid), process_status::killed) << "pid " << pid;
    EXPECT_EQ(w.sched.get_exit_error(pid), "owner_reaped") << "pid " << pid;
  }
  EXPECT_FALSE(w.status(done).has_value()) << "the slot's reap";
  ASSERT_GE(submitted, 0);
  ASSERT_TRUE(w.status(submitted).has_value());
  EXPECT_FALSE(is_finished(*w.status(submitted))) << "a process submitted during kill_owner";
  w.sched.kill(submitted);
}

// KO-2 (S1.4): a kill that throws does not stop kill_owner. Four of the owner's processes are
// parked on (stuck n) calls, work that never settles and whose cancel hook throws, naming n: every
// kill throws. Every process is still killed, and the first throw (that of the first hook to run)
// propagates. Before, the first throw ended the loop, leaving the processes after it alive.
TEST(AsyncScheduler, KO02_AThrowingKillDoesNotStopTheRest) {
  std::vector<std::string> thrown; // the hooks that threw, in order; outlives the world
  kill_owner_world w("ko02");
  register_async_fn(w.env, *w.host, "stuck", [&thrown](std::span<const value_t> a) -> async_work {
    const std::string tag = to_string(a[0]);
    async_work work;
    work.start = [&thrown, tag](const std::shared_ptr<async_call> &call) {
      call->on_cancel([&thrown, tag] {
        thrown.push_back(tag);
        throw std::runtime_error("ko02: the kill of " + tag + " threw");
      });
    };
    return work;
  });
  std::vector<int> pids;
  for (int k = 0; k < 4; ++k)
    pids.push_back(w.submit("(stuck " + std::to_string(k) + ")"));
  ASSERT_TRUE(w.pump_until([&] {
    for (const int pid : pids)
      if (w.status(pid) != process_status::waiting)
        return false;
    return true;
  }));
  std::string propagated;
  try {
    w.sched.kill_owner("doc");
    ADD_FAILURE() << "kill_owner did not throw";
  } catch (const std::runtime_error &e) {
    propagated = e.what();
  }
  std::printf("KO-2: %zu kills threw\n", thrown.size());
  ASSERT_FALSE(thrown.empty());
  EXPECT_EQ(thrown.size(), pids.size()) << "kill_owner stopped at a throw";
  EXPECT_EQ(propagated, "ko02: the kill of " + thrown.front() + " threw");
  for (const int pid : pids) {
    EXPECT_EQ(w.status(pid), process_status::killed) << w.describe(pid);
    EXPECT_EQ(w.sched.get_exit_error(pid), "owner_reaped") << w.describe(pid);
  }
}

// --- Stage S: slices (td 4.4, 8.10; S2-Q12; S2 section 4's SL-1..SL-9) ---

// A run is up to slice_steps() consecutive steps of one process, on one step's guards, clock
// reading and accounting. Every case sets its slice with set_slice(), so CVC_EXEC_SLICE_STEPS (the
// whole suite at 256: state_exec_async_test_slice256) changes nothing here but SL-1's default. No
// case times anything: each counts steps, or ticks, the iterations of a loop that calls (tick).

// The includes the cases below add to the file's own.
#include <algorithm>
#include <cstdlib>
#include <cvc/state/state_exec/exec_group.h>
#include <limits>
#include <optional>

namespace {

using steady_clock = std::chrono::steady_clock;

// set_slice()'s seconds for a run that ends on steps alone.
constexpr double kStepsAlone = std::numeric_limits<double>::infinity();

// A process that never ends on its own.
constexpr const char *kEndless = "(begin (set i 0) (while #t (set i (+ i 1))))";

// CVC_EXEC_SLICE_STEPS as a scheduler made now reads it: nullopt when unset or empty, else the
// whole number it holds, or 0 when it holds none (the cases then assert nothing about it).
std::optional<uint64_t> env_slice_steps() {
  const char *v = std::getenv("CVC_EXEC_SLICE_STEPS");
  if (!v || !*v)
    return std::nullopt;
  char *end = nullptr;
  const unsigned long long n = std::strtoull(v, &end, 10);
  return (end && *end == '\0' && n >= 1) ? static_cast<uint64_t>(n) : 0;
}

// Set an environment variable for a case, and restore what it was (state_exec_groups_test.cpp's).
class scoped_env_var {
public:
  scoped_env_var(const char *name, const char *value) : name_(name) {
    if (const char *old = std::getenv(name))
      old_ = old;
    set(value);
  }
  ~scoped_env_var() { set(old_ ? old_->c_str() : nullptr); }
  scoped_env_var(const scoped_env_var &) = delete;
  scoped_env_var &operator=(const scoped_env_var &) = delete;

private:
  void set(const char *value) {
#ifdef _WIN32
    _putenv_s(name_.c_str(), value ? value : "");
#else
    if (value)
      setenv(name_.c_str(), value, 1);
    else
      unsetenv(name_.c_str());
#endif
  }
  std::string name_;
  std::optional<std::string> old_;
};

// A slices case's world: an async scheduler (of `policy`) watching an app's tree, one intrinsics
// context on both, and two natives a program reports through: (tick) adds one to `ticks`, which
// another thread may read, then calls on_tick; (seen) calls on_seen. Both run inside the step of
// the process calling them, on the scheduler's thread. Each program gets its own scope over `env`.
// The counter and the hooks come first, so they outlive the scheduler.
struct slice_world {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  std::atomic<uint64_t> ticks{0};
  std::function<void()> on_tick;
  std::function<void()> on_seen;
  async_scheduler sched;
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();

  explicit slice_world(scheduling_policy policy = scheduling_policy::round_robin) : sched(policy) {
    sched.set_watch_root(&tree);
    ictx.sched = &sched;
    ictx.root = &tree;
    ictx.proc = make_process();
    register_intrinsics(env, &ictx);
    builtins::register_fn(env, "tick", [this](std::span<const value_t>) -> value_t {
      ticks.fetch_add(1);
      if (on_tick)
        on_tick();
      return value_t();
    });
    builtins::register_fn(env, "seen", [this](std::span<const value_t>) -> value_t {
      if (on_seen)
        on_seen();
      return value_t();
    });
  }
  slice_world(const slice_world &) = delete;
  slice_world &operator=(const slice_world &) = delete;

  int submit(const std::string &script, execute_options opts = {}) {
    opts.env = environment::extend(env);
    return sched.execute(script, opts);
  }
  std::optional<process_status> status(int pid) const {
    const auto i = sched.get_process_info(pid);
    return i ? std::optional<process_status>(i->status) : std::nullopt;
  }
  bool finished(int pid) const {
    const std::optional<process_status> s = status(pid);
    return s && is_finished(*s);
  }
  uint64_t steps(int pid) const {
    const auto i = sched.get_process_info(pid);
    return i ? i->step_count : 0;
  }
  std::string why(int pid) const { return sched.get_exit_error(pid).value_or(""); }
  // `pid`'s status, exit error and steps, for a failure's message.
  std::string describe(int pid) const {
    const std::optional<process_status> s = status(pid);
    return "pid " + std::to_string(pid) + ": " + (s ? to_string(*s) : "gone") + " " + why(pid) +
           ", " + std::to_string(steps(pid)) + " steps";
  }
  // Pump until `done()`; false when `give_up` seconds pass first (a guard against a hang, which
  // the case then reports: no case asserts how long anything took).
  template <class Done> bool pump_until(Done done, double give_up = 30.0) {
    const auto t0 = steady_clock::now();
    while (!done()) {
      if (std::chrono::duration<double>(steady_clock::now() - t0).count() > give_up)
        return false;
      sched.sync_run_steps(1000000, 0.05);
    }
    return true;
  }
};

// --- SL-1 ---

// One run of SL-1's program: three processes of different shapes, in the order submitted, a loop
// that sends the receiver a message every 60 iterations and "end" after them, a recursion, and
// the receiver. Pumped 97 steps at a time until all three end.
struct sl01_outcome {
  uint64_t slice = 0;           // slice_steps() for the run
  std::vector<value_t> results; // each process's, in submission order
  std::vector<uint64_t> steps;  // each process's step count
  std::vector<int> completed;   // the processes, in the order their on_complete ran
  std::vector<uint64_t> trace;  // every process's step count after each pump
};

sl01_outcome sl01_run(std::optional<uint64_t> slice) {
  const std::vector<std::string> programs = {
      "(begin (set i 0) (set s 0)"
      " (while (< i 300) (set s (+ s i))"
      "  (if (= (% i 60) 0) (msg-send \"sl01.ch\" (str i)) nil) (set i (+ i 1)))"
      " (msg-send \"sl01.ch\" \"end\") s)",
      "(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) (fib 12))",
      "(begin (set n 0) (set got 0) (set m (msg-recv \"sl01.ch\"))"
      " (while (!= (get-attr m \"payload\") \"end\")"
      "  (set n (+ n 1)) (set got (+ got (int (get-attr m \"payload\"))))"
      "  (set m (msg-recv \"sl01.ch\")))"
      " (list n got))",
  };
  sl01_outcome out; // before the world: an on_complete writes it
  slice_world w;
  if (slice)
    w.sched.set_slice(*slice);
  out.slice = w.sched.slice_steps();
  std::vector<int> pids;
  for (std::size_t k = 0; k < programs.size(); ++k) {
    execute_options opts;
    opts.on_complete = [&out, k](value_t) { out.completed.push_back(static_cast<int>(k)); };
    pids.push_back(w.submit(programs[k], opts));
  }
  const auto all_done = [&] {
    for (const int pid : pids)
      if (!w.finished(pid))
        return false;
    return true;
  };
  for (int pump = 0; pump < 100000 && !all_done(); ++pump) {
    w.sched.sync_run_steps(97);
    for (const int pid : pids)
      out.trace.push_back(w.steps(pid));
  }
  for (const int pid : pids) {
    EXPECT_EQ(w.status(pid), process_status::terminated) << w.describe(pid);
    out.results.push_back(w.sched.get_result(pid).value_or(value_t()));
    out.steps.push_back(w.steps(pid));
  }
  return out;
}

void expect_same_results(const sl01_outcome &a, const sl01_outcome &b, const char *what) {
  ASSERT_EQ(a.results.size(), b.results.size()) << what;
  for (std::size_t k = 0; k < a.results.size(); ++k)
    EXPECT_TRUE(values_equal(a.results[k], b.results[k]))
        << what << ", process " << k << ": " << to_string(a.results[k]) << " against "
        << to_string(b.results[k]);
}

} // namespace

// SL-1: slice_steps 1, the default (CVC_EXEC_SLICE_STEPS unset), is today's loop. SL-1's program
// at the default and at set_slice(1) gives each process the same result and step count, ends them
// in the same order, and has the same step counts after every pump (a trace at pump grain: no
// per-step trace hook exists, S2 section 0). At 256 only the interleaving may change: the results,
// and the step counts of the two processes whose work doesn't depend on it, are the same.
TEST(Slices, SL01_OneStepRunsAreTodaysLoop) {
  const std::optional<uint64_t> env = env_slice_steps();
  {
    async_scheduler made;
    if (!env)
      EXPECT_EQ(made.slice_steps(), 1u) << "the default, with CVC_EXEC_SLICE_STEPS unset";
    else if (*env > 0)
      EXPECT_EQ(made.slice_steps(), *env) << "CVC_EXEC_SLICE_STEPS, read as the scheduler is made";
    made.set_slice(1);
    EXPECT_EQ(made.slice_steps(), 1u);
    EXPECT_DOUBLE_EQ(made.slice_seconds(), 1e-3); // set_slice's default
  }

  const sl01_outcome by_default = sl01_run(std::nullopt);
  const sl01_outcome at_one = sl01_run(1);
  ASSERT_EQ(at_one.slice, 1u);
  ASSERT_EQ(at_one.results.size(), 3u);
  EXPECT_TRUE(values_equal(at_one.results[0], value_t(int64_t{44850}))) // 0 + ... + 299
      << to_string(at_one.results[0]);
  EXPECT_TRUE(values_equal(at_one.results[1], value_t(int64_t{144}))) // (fib 12)
      << to_string(at_one.results[1]);
  EXPECT_TRUE(values_equal(at_one.results[2], // 5 messages: 0, 60, 120, 180, 240
                           make_list({value_t(int64_t{5}), value_t(int64_t{600})})))
      << to_string(at_one.results[2]);
  EXPECT_EQ(at_one.completed.size(), 3u);

  expect_same_results(by_default, at_one, "the default against set_slice(1)");
  if (by_default.slice == 1) {
    EXPECT_EQ(by_default.steps, at_one.steps);
    EXPECT_EQ(by_default.completed, at_one.completed);
    EXPECT_EQ(by_default.trace, at_one.trace);
  } else {
    // CVC_EXEC_SLICE_STEPS made longer runs the default.
    EXPECT_EQ(by_default.steps[0], at_one.steps[0]);
    EXPECT_EQ(by_default.steps[1], at_one.steps[1]);
  }

  const sl01_outcome at_256 = sl01_run(256);
  ASSERT_EQ(at_256.slice, 256u);
  expect_same_results(at_256, at_one, "set_slice(256) against set_slice(1)");
  EXPECT_EQ(at_256.steps[0], at_one.steps[0]);
  EXPECT_EQ(at_256.steps[1], at_one.steps[1]);
  EXPECT_EQ(at_256.completed.size(), 3u);
}

// SL-2: max_steps is exact at any slice. An endless loop with max_steps 1000 is killed for it at
// exactly 1000 steps, the 1000th falling 232 steps into its fourth run of 256 (or a run's 1 ms, or
// on steps alone). Two such loops take turns, and each is killed at exactly 1000 too.
TEST(Slices, SL02_MaxStepsIsExact) {
  struct slice_cfg {
    uint64_t steps;
    double seconds;
  };
  for (const slice_cfg s :
       {slice_cfg{1, 1e-3}, slice_cfg{256, 1e-3}, slice_cfg{256, kStepsAlone}}) {
    SCOPED_TRACE("set_slice(" + std::to_string(s.steps) + ", " + std::to_string(s.seconds) + ")");
    slice_world w;
    w.sched.set_slice(s.steps, s.seconds);
    execute_options opts;
    opts.max_steps = 1000;
    const int alone = w.submit(kEndless, opts);
    w.sched.sync_run(10000000);
    EXPECT_EQ(w.status(alone), process_status::killed) << w.describe(alone);
    EXPECT_EQ(w.why(alone), "max_steps_exceeded");
    EXPECT_EQ(w.steps(alone), 1000u);

    const int a = w.submit(kEndless, opts);
    const int b = w.submit(kEndless, opts);
    w.sched.sync_run(10000000);
    for (const int pid : {a, b}) {
      EXPECT_EQ(w.status(pid), process_status::killed) << w.describe(pid);
      EXPECT_EQ(w.why(pid), "max_steps_exceeded") << w.describe(pid);
      EXPECT_EQ(w.steps(pid), 1000u) << w.describe(pid);
    }
  }
}

// SL-3: max_time at 256. An endless loop with max_time 0.05 s is killed for its time, alone and
// beside a process with no limit whose runs take turns with its own; that one runs on. The time is
// read at the 16-step clock checks, so a kill may come up to 16 steps late (S2 section 8): that
// bound is a matter of when the budget ran out, which a step count can't show without a clock, so
// only the kill is asserted (no wall-clock assertions).
TEST(Slices, SL03_MaxTimeOvershoot) {
  slice_world w;
  w.sched.set_slice(256);
  execute_options opts;
  opts.max_time = 0.05;
  const int alone = w.submit(kEndless, opts);
  w.sched.sync_run(std::nullopt, 30.0); // a kill that never comes ends the run here instead
  EXPECT_EQ(w.status(alone), process_status::killed) << w.describe(alone);
  EXPECT_EQ(w.why(alone), "time_limit_exceeded");
  EXPECT_GT(w.steps(alone), 0u);

  const int limited = w.submit(kEndless, opts);
  const int unlimited = w.submit(kEndless);
  ASSERT_TRUE(w.pump_until([&] { return w.finished(limited); })) << w.describe(limited);
  EXPECT_EQ(w.status(limited), process_status::killed) << w.describe(limited);
  EXPECT_EQ(w.why(limited), "time_limit_exceeded");
  EXPECT_FALSE(w.finished(unlimited)) << w.describe(unlimited);
  EXPECT_GT(w.steps(unlimited), 0u);
  EXPECT_TRUE(w.sched.kill(unlimited));
}

// SL-4: a process with max_step_time keeps one step a pick, so its per-step deadline is armed for
// each step, at 256 as at 1. A resident whose step loops in a native, (spin-step), which spins
// until a deadline cuts it (eval_deadline_check, as a long native loop does), is killed by its
// per-step deadline after its first 1000 iterations. And beside a process with no limit, on
// steps alone, it takes one step a pick while the other takes 256: its step count stays a small
// fraction of the other's, where at one step a pick for both they would be equal.
TEST(Slices, SL04_MaxStepTimeStaysPerStep) {
  {
    slice_world w;
    w.sched.set_slice(256);
    bool uncut = false; // the spin reached its own end: no deadline cut it
    builtins::register_fn(w.env, "spin-step", [&uncut](std::span<const value_t>) -> value_t {
      const auto until = steady_clock::now() + std::chrono::seconds(10);
      while (steady_clock::now() < until)
        eval_deadline_check("spin-step");
      uncut = true;
      return value_t();
    });
    execute_options opts;
    opts.max_step_time = 0.05;
    const int pid =
        w.submit("(begin (set i 0) (while (< i 1000) (set i (+ i 1))) (spin-step) 1)", opts);
    w.sched.sync_run();
    EXPECT_EQ(w.status(pid), process_status::killed) << w.describe(pid);
    EXPECT_EQ(w.why(pid), "time_limit_exceeded");
    EXPECT_FALSE(uncut);
    EXPECT_GT(w.steps(pid), 1000u); // the loop ran first
  }
  {
    slice_world w;
    w.sched.set_slice(256, kStepsAlone);
    execute_options capped;
    capped.max_step_time = 0.05;
    const int resident = w.submit(kEndless, capped);
    const int other = w.submit(kEndless);
    w.sched.sync_run_steps(257 * 40);
    const uint64_t resident_steps = w.steps(resident);
    const uint64_t other_steps = w.steps(other);
    EXPECT_GT(resident_steps, 0u);
    EXPECT_LT(resident_steps * 16, other_steps)
        << "the resident took " << resident_steps << " steps beside the other's " << other_steps;
    EXPECT_FALSE(w.finished(resident)) << w.describe(resident);
    EXPECT_TRUE(w.sched.kill(resident));
    EXPECT_TRUE(w.sched.kill(other));
  }
}

namespace {

// SL-5's self and slot kills: what became of the process, and the ticks it made after its kill.
struct kill_outcome {
  std::optional<process_status> status;
  std::string why;
  uint64_t steps = 0;
  uint64_t ticks = 0;
};

// A process that loops 100 times, then is killed in the middle of a run, by (kill (self-pid)) or
// by a tree slot its write runs (`by_slot`), then would tick forever.
kill_outcome sl05_run(uint64_t slice, bool by_slot) {
  slice_world w;
  w.sched.set_slice(slice);
  int pid = -1;
  boost::signals2::scoped_connection slot = w.tree("sl05.trip").valueChanged.connect([&w, &pid] {
    if (pid >= 0)
      w.sched.kill(pid);
  });
  pid = w.submit(std::string("(begin (set i 0) (while (< i 100) (set i (+ i 1))) ") +
                 (by_slot ? "(state-set \"sl05.trip\" \"go\")" : "(kill (self-pid))") +
                 " (while #t (tick)))");
  w.sched.sync_run_steps(100000);
  kill_outcome out{w.status(pid), w.why(pid), w.steps(pid), w.ticks.load()};
  slot.disconnect();
  if (!w.finished(pid))
    w.sched.kill(pid);
  return out;
}

} // namespace

// SL-5: a kill lands within a step at 256. A self kill, and a kill from a tree slot that the
// process's own write runs, end the run at once: no tick after it, and the same step count as at
// 1. A kill posted from another thread while the process runs an endless loop lands within a few
// steps, five times over: a run ends at the first step that finds the ingress set (two relaxed
// loads a step, S2-Q12), not at its end. The poster reads the ticks as soon as post_control
// returned, and the kill's done reads them as it is applied, on the scheduler's thread; nothing is
// timed.
TEST(Slices, SL05_KillsLandWithinAStep) {
  for (const bool by_slot : {false, true}) {
    SCOPED_TRACE(by_slot ? "a slot's kill" : "a self kill");
    const kill_outcome at_one = sl05_run(1, by_slot);
    const kill_outcome at_256 = sl05_run(256, by_slot);
    for (const kill_outcome *o : {&at_one, &at_256}) {
      EXPECT_EQ(o->status, process_status::killed) << o->why;
      EXPECT_EQ(o->why, "killed_by_user");
      EXPECT_EQ(o->ticks, 0u) << "the process stepped on after its kill";
    }
    EXPECT_EQ(at_256.steps, at_one.steps);
  }

  for (int round = 0; round < 5; ++round) {
    SCOPED_TRACE("a posted kill, round " + std::to_string(round));
    // Before the world: the scheduler's destructor may refuse what was never applied.
    done_log log;
    std::atomic<uint64_t> at_post{0};
    uint64_t at_apply = 0, steps_at_apply = 0;
    slice_world w;
    w.sched.set_slice(256);
    execute_options opts;
    opts.max_time = 60.0; // a post that is lost ends the run here instead, and fails the case
    const int pid = w.submit("(while #t (tick))", opts);
    std::thread poster([&] {
      const auto give_up = steady_clock::now() + std::chrono::seconds(10);
      while (w.ticks.load() < 1000 && steady_clock::now() < give_up)
        std::this_thread::yield();
      std::function<void(bool)> done = log.done("kill");
      w.sched.post_control(pid, control(control_cmd::kind::kill, [&, done](bool applied) {
                             if (applied) { // a refusal comes from the destructor: leave w alone
                               at_apply = w.ticks.load();
                               steps_at_apply = w.steps(pid);
                             }
                             done(applied);
                           }));
      at_post.store(w.ticks.load());
    });
    w.sched.sync_run_steps();
    poster.join();
    EXPECT_EQ(log.calls, (done_calls{{"kill", true}}));
    EXPECT_EQ(w.status(pid), process_status::killed) << w.describe(pid);
    EXPECT_EQ(w.why(pid), "killed_by_user");
    EXPECT_GE(at_post.load(), 1000u) << "posted before the loop got going";
    const uint64_t posted = at_post.load();
    const uint64_t after = at_apply >= posted ? at_apply - posted : 0;
    EXPECT_LE(after, 16u) << "ticks between the post and the kill (each at least a step)";
    EXPECT_EQ(w.ticks.load(), at_apply) << "a tick after the kill was applied";
    EXPECT_EQ(w.steps(pid), steps_at_apply) << "a step after the kill was applied";
  }
}

// SL-6: at 256 a watched node changed during a run fires the watcher's handler at the watcher's
// next run (watch events coalesce over a run, S2 section 8). Another process changes the node in
// the middle of one of its runs, while the watcher loops: the handler fires, and the watcher sees
// what it wrote. And a watcher's own write in the middle of its run: its handler fires before the
// watcher has looped a run's worth after the write (256 steps, each iteration at least one).
TEST(Slices, SL06_AWatchFiresAtTheNextRun) {
  {
    slice_world w;
    w.sched.set_slice(256);
    const int watcher = w.submit(R"((begin
        (state-watch "sl06.v" (lambda (path val) (state-set "sl06.seen" "yes")))
        (set n 0)
        (while (and (not (state-exists "sl06.seen")) (< n 1000000)) (set n (+ n 1)))
        (state-get "sl06.seen")))");
    const int writer = w.submit(R"((begin
        (set i 0) (while (< i 1000) (set i (+ i 1)))
        (state-set "sl06.v" "changed")
        (set j 0) (while (< j 1000) (set j (+ j 1)))
        "wrote"))");
    auto results = w.sched.sync_run(50000000);
    ASSERT_TRUE(results.count(watcher)) << w.describe(watcher);
    ASSERT_TRUE(results.count(writer)) << w.describe(writer);
    EXPECT_TRUE(values_equal(results[watcher], value_t(std::string("yes"))))
        << "the handler didn't fire: " << to_string(results[watcher]);
    EXPECT_TRUE(values_equal(results[writer], value_t(std::string("wrote"))));
  }
  {
    slice_world w;
    w.sched.set_slice(256);
    bool fired = false;
    uint64_t ticks_at_fire = 0;
    w.on_seen = [&] {
      if (!fired)
        ticks_at_fire = w.ticks.load();
      fired = true;
    };
    builtins::register_fn(w.env, "fired",
                          [&fired](std::span<const value_t>) -> value_t { return value_t(fired); });
    const int pid = w.submit(R"((begin
        (state-watch "sl06.own" (lambda (path val) (seen)))
        (set i 0) (while (< i 100) (set i (+ i 1)))
        (state-set "sl06.own" "x")
        (while (not (fired)) (tick))
        "done"))");
    w.sched.sync_run_steps(1000000);
    EXPECT_TRUE(fired) << w.describe(pid);
    EXPECT_EQ(w.status(pid), process_status::terminated) << w.describe(pid);
    EXPECT_LE(ticks_at_fire, 256u) << "iterations after the write before the handler fired";
  }
}

namespace {

// SL-7's message case: under the priority policy, a better process (-5) parked on msg-recv, and a
// worse one (5) that loops 100 times, sends it the message, then ticks forever. What the better
// one's (seen) read: the worse one's step count (its run over) and ticks.
struct sl07_outcome {
  bool seen = false;
  uint64_t low_steps = 0;
  uint64_t low_ticks = 0;
};

sl07_outcome sl07_message_run(uint64_t slice) {
  sl07_outcome out;
  slice_world w(scheduling_policy::priority);
  w.sched.set_slice(slice);
  int low = -1;
  w.on_seen = [&] {
    out.seen = true;
    out.low_ticks = w.ticks.load();
    out.low_steps = w.steps(low);
  };
  execute_options better;
  better.priority = -5;
  const int high = w.submit(R"((begin (msg-recv "sl07.go") (seen) 1))", better);
  execute_options worse;
  worse.priority = 5;
  low = w.submit(R"((begin (set i 0) (while (< i 100) (set i (+ i 1)))
                      (msg-send "sl07.go" "go")
                      (while #t (tick))))",
                 worse);
  w.sched.sync_run_steps(100000);
  EXPECT_EQ(w.status(high), process_status::terminated) << w.describe(high);
  EXPECT_TRUE(w.sched.kill(low));
  return out;
}

} // namespace

// SL-7: under the priority policy a process made ready at a better level ends the runner's run
// (the flag raised in the run queue's enqueue, S2-Q12). The worse process's send readies the
// better one, which runs within one step of it: the worse one has made no tick, and has taken the
// steps it takes at 1, give or take one. And a better sleeper falls due while the worse process
// runs on alone: it runs within 16 steps of its due time (the 16-step clock checks), counted in
// the worse process's ticks after the due time, with room for the clock reads.
TEST(Slices, SL07_ABetterPriorityPreempts) {
  const sl07_outcome at_one = sl07_message_run(1);
  const sl07_outcome at_256 = sl07_message_run(256);
  ASSERT_TRUE(at_one.seen);
  ASSERT_TRUE(at_256.seen);
  EXPECT_LE(at_256.low_ticks, 1u);
  EXPECT_GE(at_256.low_steps, at_one.low_steps);
  EXPECT_LE(at_256.low_steps, at_one.low_steps + 1);

  slice_world w(scheduling_policy::priority);
  w.sched.set_slice(256);
  bool armed = false, seen = false;
  steady_clock::time_point due{};
  uint64_t after_due = 0, after_due_at_seen = 0;
  // (arm) is called the step before (sleep 0.02): the scheduler's due time is no earlier.
  builtins::register_fn(w.env, "arm", [&](std::span<const value_t>) -> value_t {
    due = steady_clock::now() + std::chrono::milliseconds(20);
    armed = true;
    return value_t();
  });
  w.on_tick = [&] {
    if (armed && steady_clock::now() >= due)
      ++after_due;
  };
  w.on_seen = [&] {
    seen = true;
    after_due_at_seen = after_due;
    w.sched.stop();
  };
  execute_options better;
  better.priority = -5;
  const int sleeper = w.submit("(begin (arm) (sleep 0.02) (seen) 1)", better);
  execute_options worse;
  worse.priority = 5;
  const int runner = w.submit("(while #t (tick))", worse);
  w.sched.sync_run_steps(std::nullopt, 10.0); // (seen) stops it; a sleeper never woken, the budget
  ASSERT_TRUE(seen) << w.describe(sleeper);
  EXPECT_GT(w.ticks.load(), 0u);
  EXPECT_LE(after_due_at_seen, 32u) << "the runner's ticks after the sleeper's due time";
  EXPECT_TRUE(w.sched.kill(runner));
  w.sched.kill(sleeper); // stopped before its last step, or ended
}

// SL-8: a run counts charged steps (S2-Q12), as a pump's budget does (#1117): a script engine's
// slice is work inside one step, charged through add_steps, which (heavy) stands in for here: it
// charges 1000 steps on top of its own. At 256, each of its picks is one call, since the call's
// charge is past the run's 256: a 5000-step pump takes four or five calls, as at 1
// (AsyncSchedulerTest.ARunsStepBudgetCountsWhatAStepCharged). And beside a plain loop, on steps
// alone, the two take turns: a call, then 256 plain steps, so the calls match the plain process's
// runs, where a run counting picks would make 256 calls a pick and leave the plain one none.
TEST(Slices, SL08_ScriptEngineRunsCountChargedSteps) {
  const auto make_env = [](int &calls) {
    environment_ptr env = builtins::make_default_environment();
    builtins::register_fn(env, "heavy", [counter = &calls](std::span<const value_t>) {
      ++*counter;
      if (detail::data_step_scope *data = detail::data_step_scope::current())
        data->stats().add_steps(step_kind::data, 1000);
      return value_t();
    });
    return env;
  };
  {
    async_scheduler sched;
    sched.set_slice(256);
    int calls = 0;
    execute_options opts;
    opts.env = make_env(calls);
    const int pid = sched.execute(std::string("(while #t (heavy))"), opts);
    const uint64_t taken = sched.sync_run_steps(5000);
    EXPECT_GE(calls, 4);
    EXPECT_LE(calls, 5);
    EXPECT_GE(taken, 5000u);
    EXPECT_LT(taken, 6100u);
    EXPECT_TRUE(sched.kill(pid));
  }
  {
    async_scheduler sched;
    sched.set_slice(256, kStepsAlone);
    int calls = 0;
    execute_options opts;
    opts.env = make_env(calls);
    const int heavy = sched.execute(std::string("(while #t (heavy))"), opts);
    const int plain = sched.execute(std::string(kEndless));
    sched.sync_run_steps(100000);
    const uint64_t plain_runs = sched.get_process_info(plain)->step_count / 256;
    EXPECT_GE(plain_runs, 10u);
    EXPECT_LE(static_cast<uint64_t>(calls), plain_runs + 2)
        << calls << " calls beside " << plain_runs << " runs of the plain loop";
    EXPECT_GE(static_cast<uint64_t>(calls) + 1, plain_runs)
        << calls << " calls beside " << plain_runs << " runs of the plain loop";
    EXPECT_TRUE(sched.kill(heavy));
    EXPECT_TRUE(sched.kill(plain));
  }
}

// SL-9: a run ends at slice_seconds, not only at slice_steps. At set_slice(1000000, 1e-3), three
// processes each build two 400-deep lists, then compare them forever: deep-data work, 16 nodes a
// data step (data.step_nodes), the steps whose cost made slice_seconds' 1 ms default (S2-Q12). In
// each of three pumps with a time budget, every process makes progress, data steps included: the
// pump's time is shared out a run of about 1 ms at a time, where runs of 1000000 steps would give
// it all to one process. Step counts only.
TEST(Slices, SL09_SliceSecondsEndsADataRun) {
  slice_world w;
  data_options data;
  data.step_nodes = 16;
  w.sched.set_data_defaults(data);
  w.sched.set_slice(1000000, 1e-3);
  ASSERT_EQ(w.sched.slice_steps(), 1000000u);
  ASSERT_DOUBLE_EQ(w.sched.slice_seconds(), 1e-3);
  const std::string compare_forever =
      "(begin (set a 1) (set b 1) (set i 0)"
      " (while (< i 400) (set a (list a)) (set b (list b)) (set i (+ i 1)))"
      " (while #t (= a b)))";
  std::vector<int> pids;
  for (int k = 0; k < 3; ++k)
    pids.push_back(w.submit(compare_forever));
  const auto data_steps = [&](int pid) {
    const auto i = w.sched.get_process_info(pid);
    return i ? i->data_steps : 0;
  };
  // Until each is comparing (one pump each would do, even with runs of 1000000 steps).
  ASSERT_TRUE(w.pump_until([&] {
    for (const int pid : pids)
      if (data_steps(pid) == 0)
        return false;
    return true;
  }));
  for (int pump = 0; pump < 3; ++pump) {
    SCOPED_TRACE("pump " + std::to_string(pump));
    std::vector<uint64_t> steps_before, data_before;
    for (const int pid : pids) {
      steps_before.push_back(w.steps(pid));
      data_before.push_back(data_steps(pid));
    }
    w.sched.sync_run_steps(100000000, 0.03);
    for (std::size_t k = 0; k < pids.size(); ++k) {
      EXPECT_GT(w.steps(pids[k]), steps_before[k]) << w.describe(pids[k]);
      EXPECT_GT(data_steps(pids[k]), data_before[k]) << w.describe(pids[k]);
    }
  }
  for (const int pid : pids) {
    EXPECT_FALSE(w.finished(pid)) << w.describe(pid);
    EXPECT_TRUE(w.sched.kill(pid));
  }
}

// The settings (SS.1): CVC_EXEC_SLICE_STEPS, read as a scheduler is made; under the tree's
// state_exec.slice_steps and state_exec.slice_seconds, read at a run's start from the watch root
// when no set_slice() was called; under set_slice(), whichever came first. An exec_group's
// options::slice_steps and slice_seconds are set_slice() when given; unset, the group's scheduler
// takes the tree's at its pump.
TEST(Slices, SL10_SliceSettingsPrecedence) {
  {
    scoped_env_var env("CVC_EXEC_SLICE_STEPS", "32");
    async_scheduler made;
    EXPECT_EQ(made.slice_steps(), 32u);
  }
  {
    scoped_env_var env("CVC_EXEC_SLICE_STEPS", "32");
    cvc::app app;
    cvc::state &tree = cvc::state::instance(app);
    tree("state_exec.slice_steps").value(std::string("64"));
    tree("state_exec.slice_seconds").value(std::string("0.004"));
    async_scheduler sched;
    sched.set_watch_root(&tree);
    sched.execute(std::string("(+ 1 2)"));
    sched.sync_run();
    EXPECT_EQ(sched.slice_steps(), 64u) << "the tree's, over CVC_EXEC_SLICE_STEPS";
    EXPECT_DOUBLE_EQ(sched.slice_seconds(), 0.004);
    // A change applies at the next run.
    tree("state_exec.slice_steps").value(std::string("48"));
    sched.execute(std::string("(+ 1 2)"));
    sched.sync_run_steps(100);
    EXPECT_EQ(sched.slice_steps(), 48u);
    // set_slice() wins over the tree, at once and at every run after.
    sched.set_slice(8, 0.002);
    EXPECT_EQ(sched.slice_steps(), 8u);
    EXPECT_DOUBLE_EQ(sched.slice_seconds(), 0.002);
    tree("state_exec.slice_steps").value(std::string("128"));
    tree("state_exec.slice_seconds").value(std::string("0.006"));
    sched.execute(std::string("(+ 1 2)"));
    sched.sync_run();
    EXPECT_EQ(sched.slice_steps(), 8u);
    EXPECT_DOUBLE_EQ(sched.slice_seconds(), 0.002);
  }
  {
    // set_slice() before the scheduler's first run, and before it has a watch root.
    cvc::app app;
    cvc::state &tree = cvc::state::instance(app);
    tree("state_exec.slice_steps").value(std::string("64"));
    async_scheduler sched;
    sched.set_slice(8);
    sched.set_watch_root(&tree);
    sched.execute(std::string("(+ 1 2)"));
    sched.sync_run();
    EXPECT_EQ(sched.slice_steps(), 8u);
  }
  {
    cvc::app app;
    cvc::state &tree = cvc::state::instance(app);
    tree("state_exec.slice_steps").value(std::string("64"));
    const pump_budget budget{1000, kStepsAlone};
    exec_group::options given;
    given.name = "sl10";
    given.app = &app;
    given.slice_steps = 16;
    given.slice_seconds = 0.003;
    exec_group group(std::move(given));
    EXPECT_EQ(group.sched().slice_steps(), 16u);
    EXPECT_DOUBLE_EQ(group.sched().slice_seconds(), 0.003);
    group.execute(std::string("(+ 1 2)"));
    group.pump(budget);
    EXPECT_EQ(group.sched().slice_steps(), 16u) << "the group's set_slice, over the tree";
    EXPECT_DOUBLE_EQ(group.sched().slice_seconds(), 0.003);

    exec_group::options unset;
    unset.name = "sl10-unset";
    unset.app = &app;
    exec_group plain(std::move(unset));
    plain.execute(std::string("(+ 1 2)"));
    plain.pump(budget);
    EXPECT_EQ(plain.sched().slice_steps(), 64u) << "unset: the tree's, read at the pump";
  }
}

// SL-11: a run's steps reach the 64-step cycle check (heap.h). An endless loop leaks a reference
// cycle each iteration, a let scope and the closure bound in it, and runs alone at 256 on steps
// alone, so each pick is a run of exactly 256 steps: the check is made once a pick, and a run of
// 256 must still find a collection due when its steps cross a multiple of 64. Over 40 pumps (about
// a million steps) what the scheduler's registry tracks stays bounded, as at 1, where without the
// check it would pass tens of thousands. Nothing else collects: the idle collection needs nothing
// runnable.
TEST(Slices, SL11_RunsReachTheCycleCheck) {
  for (const uint64_t slice : {uint64_t{1}, uint64_t{256}}) {
    SCOPED_TRACE("set_slice(" + std::to_string(slice) + ")");
    cycle_registry_ptr reg = make_cycle_registry();
    async_scheduler sched;
    sched.set_cycle_registry(reg.get());
    sched.set_slice(slice, kStepsAlone);
    const int pid =
        sched.execute(std::string("(begin (set i 0) (while #t (let ((h nil)) (set h (lambda () h)))"
                                  " (set i (+ i 1))))"));
    std::size_t most = 0;
    for (int pump = 0; pump < 40; ++pump) {
      sched.sync_run_steps(256 * 100);
      most = std::max(most, tracked_objects(*reg));
    }
    const auto info = sched.get_process_info(pid);
    ASSERT_TRUE(info);
    EXPECT_FALSE(is_finished(info->status));
    EXPECT_GE(info->step_count, 40u * 256 * 100);
    EXPECT_LT(most, 4000u) << "objects the registry tracked at most, after a pump";
    EXPECT_TRUE(sched.kill(pid));
    sched.sync_run();
  }
}

// SL-12: a run's first step can preempt it too. SL-7's message case with the send as the first
// step of a pick: at 1, the step of the worse process that readies the better one is found (the
// better runs right after it); at 256 the worse one is stepped one step a pick (sync_run_steps(1))
// up to the step before it, then pumped. Its run's first step is the send, and the better process
// runs right after it: the worse one has taken exactly the send's steps and no tick, where a flag
// cleared after that step would leave it the rest of its run.
TEST(Slices, SL12_ARunsFirstStepPreempts) {
  const auto programs = [](slice_world &w, int &high, int &low) {
    execute_options better;
    better.priority = -5;
    high = w.submit(R"((begin (msg-recv "sl12.go") (seen) 1))", better);
    execute_options worse;
    worse.priority = 5;
    low = w.submit(R"((begin (set i 0) (while (< i 100) (set i (+ i 1)))
                        (msg-send "sl12.go" "go")
                        (while #t (tick))))",
                   worse);
  };
  uint64_t send_step = 0;
  {
    slice_world w(scheduling_policy::priority);
    w.sched.set_slice(1);
    int high = -1, low = -1;
    uint64_t ticks_at_seen = 1;
    w.on_seen = [&] {
      send_step = w.steps(low);
      ticks_at_seen = w.ticks.load();
    };
    programs(w, high, low);
    w.sched.sync_run_steps(100000);
    EXPECT_EQ(w.status(high), process_status::terminated) << w.describe(high);
    EXPECT_EQ(ticks_at_seen, 0u);
    EXPECT_TRUE(w.sched.kill(low));
  }
  ASSERT_GT(send_step, 100u);

  slice_world w(scheduling_policy::priority);
  w.sched.set_slice(256, kStepsAlone);
  int high = -1, low = -1;
  bool seen = false;
  uint64_t low_steps = 0, low_ticks = 0;
  w.on_seen = [&] {
    seen = true;
    low_steps = w.steps(low);
    low_ticks = w.ticks.load();
  };
  programs(w, high, low);
  for (int k = 0; k < 100000 && w.steps(low) + 1 < send_step; ++k)
    w.sched.sync_run_steps(1);
  ASSERT_EQ(w.steps(low) + 1, send_step);
  ASSERT_FALSE(seen);
  w.sched.sync_run_steps(100000);
  ASSERT_TRUE(seen) << w.describe(high);
  EXPECT_EQ(low_steps, send_step) << "the worse process's steps when the better one ran";
  EXPECT_EQ(low_ticks, 0u);
  EXPECT_EQ(w.status(high), process_status::terminated) << w.describe(high);
  EXPECT_TRUE(w.sched.kill(low));
}

namespace {

// SL-13's run: under the priority policy, `runner_script` runs and makes its own level worse while
// `other_script` is ready, or readied, at a level now better than the runner's; the other's (seen)
// reads the runner's ticks, made in the runner's endless loop after that.
struct sl13_outcome {
  bool seen = false;
  uint64_t runner_ticks = 0;
};

sl13_outcome sl13_run(uint64_t slice, const std::string &runner_script, int runner_priority,
                      const std::string &other_script, int other_priority) {
  sl13_outcome out;
  slice_world w(scheduling_policy::priority);
  w.sched.set_slice(slice, kStepsAlone);
  w.on_seen = [&] {
    out.seen = true;
    out.runner_ticks = w.ticks.load();
  };
  execute_options other_opts;
  other_opts.priority = other_priority;
  const int other = w.submit(other_script, other_opts);
  execute_options runner_opts;
  runner_opts.priority = runner_priority;
  const int runner = w.submit(runner_script, runner_opts);
  w.sched.sync_run_steps(100000);
  EXPECT_EQ(w.status(other), process_status::terminated) << w.describe(other);
  EXPECT_TRUE(w.sched.kill(runner)) << w.describe(runner);
  return out;
}

} // namespace

// SL-13: a runner whose own level gets worse ends its run, as one made ready at a better level
// does (process_table remembers the runner, and raises the flag when update() re-levels it to a
// worse key). Two ways, at 256 on steps alone, each against 1:
// - (set-priority 10) on itself, at 0, while another process is ready at 5: the other runs right
//   after it, before the runner's first tick.
// - a lock release that drops the boost it inherited: a holder at 10 that a waiter at 0 boosts
//   releases, and the waiter is woken at its own level, 0, the runner's level when its run began,
//   which is not better. The waiter runs right after the release, before the holder's first tick.
TEST(Slices, SL13_TheRunnersOwnLevelGettingWorsePreempts) {
  {
    SCOPED_TRACE("set-priority on itself");
    const std::string runner =
        "(begin (set i 0) (while (< i 100) (set i (+ i 1))) (set-priority 10) (while #t (tick)))";
    const std::string other = "(begin (seen) 1)";
    for (const uint64_t slice : {uint64_t{1}, uint64_t{256}}) {
      SCOPED_TRACE("set_slice(" + std::to_string(slice) + ")");
      const sl13_outcome o = sl13_run(slice, runner, 0, other, 5);
      ASSERT_TRUE(o.seen);
      EXPECT_EQ(o.runner_ticks, 0u) << "the runner's ticks before the better process ran";
    }
  }
  {
    SCOPED_TRACE("a release that drops an inherited boost");
    // The holder takes the lock, then wakes the waiter, which asks for it, parks and boosts the
    // holder to 0; the holder loops 50 times more inside, releases, then ticks forever.
    const std::string holder = R"((begin
        (with-lock "sl13.k" (begin (msg-send "sl13.w" "go")
                                   (set j 0) (while (< j 50) (set j (+ j 1)))))
        (while #t (tick))))";
    const std::string waiter = R"((begin (msg-recv "sl13.w") (with-lock "sl13.k" (seen)) 1))";
    for (const uint64_t slice : {uint64_t{1}, uint64_t{256}}) {
      SCOPED_TRACE("set_slice(" + std::to_string(slice) + ")");
      const sl13_outcome o = sl13_run(slice, holder, 10, waiter, 0);
      ASSERT_TRUE(o.seen);
      EXPECT_EQ(o.runner_ticks, 0u) << "the holder's ticks before the waiter ran";
    }
  }
}

// SL-14: max_time's CPU half ends a run in its middle. At set_slice(1000000) on steps alone, in a
// pump with no time budget (sync_run()), an endless loop with max_time 0.005 s is killed for its
// time well before a run of 1000000 steps ends: at the 16-step reads, a process whose first half
// ran out since its run began ends the run (its CPU starts being charged at the next pick), and one
// charging CPU ends it once what the run would charge reaches the second half. Nothing in the run
// checks a deadline between plain eval steps (the deadline guard is a native's), so it used to run
// two whole runs, 2000000 steps. Step counts only: a step would have to take 5 ns or less for
// 0.005 s to hold 1000000 of them (a CPU clock on a 15.6 ms tick allows about 18 ms: 18 ns).
TEST(Slices, SL14_MaxTimesCpuHalfEndsARun) {
  slice_world w;
  w.sched.set_slice(1000000, kStepsAlone);
  execute_options opts;
  opts.max_time = 0.005;
  const int pid = w.submit(kEndless, opts);
  w.sched.sync_run();
  EXPECT_EQ(w.status(pid), process_status::killed) << w.describe(pid);
  EXPECT_EQ(w.why(pid), "time_limit_exceeded");
  EXPECT_LT(w.steps(pid), 1000000u) << w.describe(pid);
}

// SL-15: costly steps neither overrun slice_seconds nor stretch the pump's budget. Data steps
// (SL-9's deep compares, 16 nodes a step) cost 0.05-0.4 ms each at the default step_nodes, so a
// run's first clock reading comes at its step 2 (then 4, 8, 16, and every 16th), and each reading
// charges the pump's time budget as the steps since the last one (time_budget::spent(now, checks)),
// as one step a pick checks it per step. Counted, not timed, with values that make the clock's
// grain irrelevant:
// - set_slice(1000000, 1e-9) in pumps with a time budget: any two data steps take longer than
//   that, so each run ends at its first reading. In pumps of 8 steps, both of two processes advance
//   in every pump, where a first reading at step 16 gave the whole pump to one.
// - set_slice(1000000) on steps alone, in pumps of 2e-9 s: the budget's first half is spent at
//   once and its second needs 16 checks' worth of capped gaps, which a reading worth its steps
//   gives within 16 to 64 steps. Charged as one check a reading, a pump took 256 steps or more.
TEST(Slices, SL15_CostlyStepsEndRunsOnTime) {
  const std::string compare_forever =
      "(begin (set a 1) (set b 1) (set i 0)"
      " (while (< i 400) (set a (list a)) (set b (list b)) (set i (+ i 1)))"
      " (while #t (= a b)))";
  const auto comparing = [](slice_world &w, const std::vector<int> &pids) {
    return w.pump_until([&] {
      for (const int pid : pids) {
        const auto i = w.sched.get_process_info(pid);
        if (!i || i->data_steps == 0)
          return false;
      }
      return true;
    });
  };
  {
    SCOPED_TRACE("slice_seconds");
    slice_world w;
    data_options data;
    data.step_nodes = 16;
    w.sched.set_data_defaults(data);
    w.sched.set_slice(1000000, 1e-9);
    const std::vector<int> pids = {w.submit(compare_forever), w.submit(compare_forever)};
    ASSERT_TRUE(comparing(w, pids));
    for (int pump = 0; pump < 10; ++pump) {
      SCOPED_TRACE("pump " + std::to_string(pump));
      const uint64_t a = w.steps(pids[0]), b = w.steps(pids[1]);
      w.sched.sync_run_steps(8, 10.0);
      EXPECT_GT(w.steps(pids[0]), a) << w.describe(pids[0]);
      EXPECT_GT(w.steps(pids[1]), b) << w.describe(pids[1]);
    }
    for (const int pid : pids)
      EXPECT_TRUE(w.sched.kill(pid));
  }
  {
    SCOPED_TRACE("the pump's time budget");
    slice_world w;
    data_options data;
    data.step_nodes = 16;
    w.sched.set_data_defaults(data);
    w.sched.set_slice(1000000, kStepsAlone);
    const int pid = w.submit(compare_forever);
    ASSERT_TRUE(comparing(w, {pid}));
    for (int pump = 0; pump < 10; ++pump) {
      SCOPED_TRACE("pump " + std::to_string(pump));
      const uint64_t taken = w.sched.sync_run_steps(1000000, 2e-9);
      EXPECT_GT(taken, 0u);
      EXPECT_LT(taken, 128u) << "steps one pump of 2e-9 s took";
    }
    EXPECT_TRUE(w.sched.kill(pid));
  }
}

// SL-16: a run counts the step that throws. A loop of 100 iterations, then a call of an unbound
// name, which throws in the middle of a run at 256: the process fails, and the scheduler's step
// total (get_stats) is the one at 1, the throwing step included.
TEST(Slices, SL16_AThrowingStepIsCounted) {
  const auto total = [](uint64_t slice) {
    async_scheduler sched;
    sched.set_slice(slice, kStepsAlone);
    const int pid = sched.execute(
        std::string("(begin (set i 0) (while (< i 100) (set i (+ i 1))) (sl16-nope 1))"));
    sched.sync_run();
    const auto info = sched.get_process_info(pid);
    EXPECT_TRUE(info && is_finished(info->status) && info->status != process_status::terminated)
        << (info ? to_string(info->status) : std::string("gone"));
    return sched.get_stats().total_steps;
  };
  const uint64_t at_one = total(1);
  EXPECT_GT(at_one, 100u);
  EXPECT_EQ(total(256), at_one);
}
