#include <cvc/core/state_exec/async_evaluator.h>
#include <cvc/core/state_exec/async_stackless_evaluator.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/task.h>
#include <cvc/core/state_exec/types.h>
#include <gtest/gtest.h>

using namespace cvc::state_exec;

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
