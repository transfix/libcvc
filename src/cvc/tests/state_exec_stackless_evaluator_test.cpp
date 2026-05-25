#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/parser.h>
#include <cvc/core/state_exec/stackless_evaluator.h>
#include <cvc/core/state_exec/types.h>
#include <gtest/gtest.h>

using namespace cvc::state_exec;

class StacklessEvaluatorTest : public ::testing::Test {
protected:
  std::unique_ptr<stackless_evaluator> ev;
  void SetUp() override {
    ev = std::make_unique<stackless_evaluator>(builtins::make_default_environment());
  }

  value_t eval(const std::string &script) { return ev->evaluate_script(script); }

  int64_t eval_int(const std::string &s) { return std::get<int64_t>(eval(s).v); }

  double eval_double(const std::string &s) { return std::get<double>(eval(s).v); }

  bool eval_bool(const std::string &s) { return std::get<bool>(eval(s).v); }

  std::string eval_string(const std::string &s) { return std::get<std::string>(eval(s).v); }
};

// ─── Atoms ─────────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, IntegerLiteral) { EXPECT_EQ(eval_int("42"), 42); }

TEST_F(StacklessEvaluatorTest, DoubleLiteral) { EXPECT_DOUBLE_EQ(eval_double("3.14"), 3.14); }

TEST_F(StacklessEvaluatorTest, StringLiteral) { EXPECT_EQ(eval_string("\"hello\""), "hello"); }

TEST_F(StacklessEvaluatorTest, NilLiteral) { EXPECT_TRUE(eval("nil").is_nil()); }

TEST_F(StacklessEvaluatorTest, TrueLiteral) { EXPECT_TRUE(eval_bool("t")); }

// ─── Arithmetic ────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, Addition) { EXPECT_EQ(eval_int("(+ 1 2 3)"), 6); }

TEST_F(StacklessEvaluatorTest, Subtraction) { EXPECT_EQ(eval_int("(- 10 3 2)"), 5); }

TEST_F(StacklessEvaluatorTest, NestedArithmetic) {
  EXPECT_EQ(eval_int("(+ (* 3 4) (- 10 5))"), 17);
}

// ─── Special forms ─────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, IfTrue) { EXPECT_EQ(eval_int("(if t 1 2)"), 1); }

TEST_F(StacklessEvaluatorTest, IfFalse) { EXPECT_EQ(eval_int("(if nil 1 2)"), 2); }

TEST_F(StacklessEvaluatorTest, Begin) { EXPECT_EQ(eval_int("(begin 1 2 3)"), 3); }

TEST_F(StacklessEvaluatorTest, Set) { EXPECT_EQ(eval_int("(begin (set x 42) x)"), 42); }

TEST_F(StacklessEvaluatorTest, Quote) {
  auto r = eval("(quote (+ 1 2))");
  EXPECT_TRUE(std::holds_alternative<list_ptr>(r.v));
}

TEST_F(StacklessEvaluatorTest, While) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                     "  sum)"),
            10);
}

TEST_F(StacklessEvaluatorTest, For) {
  EXPECT_EQ(eval_int("(begin (set sum 0) "
                     "  (for x (list 1 2 3 4) (set sum (+ sum x))) "
                     "  sum)"),
            10);
}

// ─── Lambda / Closures ────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, LambdaBasic) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set add1 (lambda (x) (+ x 1))) "
                     "  (add1 10))"),
            11);
}

TEST_F(StacklessEvaluatorTest, LambdaClosure) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set make-adder (lambda (n) (lambda (x) (+ x n)))) "
                     "  (set add5 (make-adder 5)) "
                     "  (add5 10))"),
            15);
}

// ─── Defun ─────────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, Defun) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun double (x) (* x 2)) "
                     "  (double 21))"),
            42);
}

TEST_F(StacklessEvaluatorTest, DefunRecursive) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fact (n) (if (<= n 1) 1 (* n (fact (- n 1))))) "
                     "  (fact 5))"),
            120);
}

// ─── Let ───────────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, LetBasic) {
  EXPECT_EQ(eval_int("(let ((a 10) (b 20)) (+ a b))"), 30);
}

TEST_F(StacklessEvaluatorTest, LetNested) {
  EXPECT_EQ(eval_int("(let ((x 10)) "
                     "  (let ((y 20)) (+ x y)))"),
            30);
}

// ─── String / List ─────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, StrConcat) {
  EXPECT_EQ(eval_string("(str-concat \"hello\" \" \" \"world\")"), "hello world");
}

TEST_F(StacklessEvaluatorTest, CarCdrCons) {
  EXPECT_EQ(eval_int("(car (list 10 20 30))"), 10);
  EXPECT_EQ(eval_int("(car (cdr (list 10 20 30)))"), 20);
  EXPECT_EQ(eval_int("(car (cons 5 (list 10 20)))"), 5);
}

// ─── Step-by-step ──────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, StepByStep) {
  auto state = ev->create_state(std::string("(+ 1 2)"));
  EXPECT_FALSE(state.done);
  int steps = 0;
  while (!ev->step(state)) {
    ++steps;
    ASSERT_LT(steps, 100) << "too many steps";
  }
  EXPECT_TRUE(state.done);
  EXPECT_EQ(std::get<int64_t>(state.result.v), 3);
}

TEST_F(StacklessEvaluatorTest, MaxSteps) {
  // A while loop that runs many iterations
  auto state =
      ev->create_state(std::string("(begin (set i 0) (while (< i 1000) (set i (+ i 1))) i)"));
  // Run with limited steps - should not complete
  auto r = ev->run(state, 10);
  EXPECT_FALSE(state.done);
  // Now run to completion
  r = ev->run(state);
  EXPECT_TRUE(state.done);
  EXPECT_EQ(std::get<int64_t>(state.result.v), 1000);
}

// ─── Eval special form ────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, Eval) { EXPECT_EQ(eval_int("(eval (quote (+ 1 2)))"), 3); }

// ─── Root ──────────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, Root) {
  auto expr = parse("(root)");
  auto r = ev->evaluate(expr);
  EXPECT_TRUE(std::holds_alternative<list_ptr>(r.v));
}

// ─── Fibonacci ─────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, Fibonacci) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fib (n) "
                     "    (if (<= n 1) n "
                     "      (+ (fib (- n 1)) (fib (- n 2))))) "
                     "  (fib 10))"),
            55);
}

// ─── Cross-evaluator consistency ───────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, DictGetSetAttr) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set obj (dict \"x\" 10 \"y\" 20)) "
                     "  (set-attr obj \"x\" 99) "
                     "  (get-attr obj \"x\"))"),
            99);
}

TEST_F(StacklessEvaluatorTest, Macros) {
  // defmacro expands and evaluates the expansion
  EXPECT_EQ(eval_int("(begin "
                     "  (defmacro double-it (x) (* x 2)) "
                     "  (double-it 21))"),
            42);
}

// ─── Stats ─────────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, StatsTracking) {
  auto state = ev->create_state(std::string("(+ 1 2)"));
  ev->run(state);
  EXPECT_GT(state.stats.get_step_count(), 0u);
  EXPECT_TRUE(state.stats.is_complete());
}
