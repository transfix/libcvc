#include <chrono>
#include <cstdio>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_call.h> // AsyncPostControl
#include <cvc/state/state_exec/async_evaluator.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/async_stackless_evaluator.h>
#include <cvc/state/state_exec/builtins.h>
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

// A command that throws (here a query; a kill can, through its locks' release or a tree write's
// slots) is refused, and the commands after it go back to the front of the queue: the next drain
// or step applies them, in order, ahead of what was posted since.
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
  // A kill whose async call's cancel hook throws: the process is not killed, and the kill's done
  // gets false.
  const int c = sched.execute(std::string("(begin (set i 0) (while #t (set i (+ i 1))))"));
  auto call = std::make_shared<async_call>("probe", c);
  call->on_cancel([] { throw std::runtime_error("the cancel hook threw"); });
  sched.track_async_call(call);
  a_throwing_command_requeues_the_rest(sched, rest, c,
                                       control(control_cmd::kind::kill, log.done("kill c")),
                                       {{"pause a", true}}, [&] { sched.drain_ingress(); });
  EXPECT_EQ(log.calls, (done_calls{{"kill c", false}}));
  EXPECT_FALSE(is_finished(sched.get_process_info(c)->status));
  EXPECT_TRUE(sched.kill(c)); // its call is gone: the kill goes through
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
