#include <atomic>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <thread>

using namespace cvc::state_exec;

class EvaluatorTest : public ::testing::Test {
protected:
  std::unique_ptr<evaluator> ev;
  void SetUp() override { ev = std::make_unique<evaluator>(builtins::make_default_environment()); }

  value_t eval(const std::string &script) { return ev->evaluate_script(script); }

  int64_t eval_int(const std::string &s) { return std::get<int64_t>(eval(s).v); }

  double eval_double(const std::string &s) { return std::get<double>(eval(s).v); }

  bool eval_bool(const std::string &s) { return std::get<bool>(eval(s).v); }

  std::string eval_string(const std::string &s) { return std::get<std::string>(eval(s).v); }
};

// ─── Atoms ─────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, IntegerLiteral) { EXPECT_EQ(eval_int("42"), 42); }

TEST_F(EvaluatorTest, DoubleLiteral) { EXPECT_DOUBLE_EQ(eval_double("3.14"), 3.14); }

TEST_F(EvaluatorTest, StringLiteral) { EXPECT_EQ(eval_string("\"hello\""), "hello"); }

TEST_F(EvaluatorTest, NilLiteral) { EXPECT_TRUE(eval("nil").is_nil()); }

TEST_F(EvaluatorTest, TrueLiteral) { EXPECT_TRUE(eval_bool("t")); }

// ─── Arithmetic ────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, Addition) { EXPECT_EQ(eval_int("(+ 1 2 3)"), 6); }

TEST_F(EvaluatorTest, Subtraction) { EXPECT_EQ(eval_int("(- 10 3 2)"), 5); }

TEST_F(EvaluatorTest, Multiplication) { EXPECT_EQ(eval_int("(* 3 4)"), 12); }

TEST_F(EvaluatorTest, Division) { EXPECT_DOUBLE_EQ(eval_double("(/ 10 4)"), 2.5); }

TEST_F(EvaluatorTest, Modulo) { EXPECT_EQ(eval_int("(% 10 3)"), 1); }

TEST_F(EvaluatorTest, NestedArithmetic) { EXPECT_EQ(eval_int("(+ (* 3 4) (- 10 5))"), 17); }

// ─── Comparison ────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, LessThan) {
  EXPECT_TRUE(eval_bool("(< 1 2)"));
  EXPECT_FALSE(eval_bool("(< 2 1)"));
}

TEST_F(EvaluatorTest, Equality) {
  EXPECT_TRUE(eval_bool("(= 42 42)"));
  EXPECT_FALSE(eval_bool("(= 1 2)"));
}

// ─── Special forms ─────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, IfTrue) { EXPECT_EQ(eval_int("(if t 1 2)"), 1); }

TEST_F(EvaluatorTest, IfFalse) { EXPECT_EQ(eval_int("(if nil 1 2)"), 2); }

TEST_F(EvaluatorTest, IfNoElse) { EXPECT_TRUE(eval("(if nil 1)").is_nil()); }

TEST_F(EvaluatorTest, Begin) { EXPECT_EQ(eval_int("(begin 1 2 3)"), 3); }

TEST_F(EvaluatorTest, BeginEmpty) { EXPECT_TRUE(eval("(begin)").is_nil()); }

TEST_F(EvaluatorTest, Set) { EXPECT_EQ(eval_int("(begin (set x 42) x)"), 42); }

TEST_F(EvaluatorTest, Quote) {
  auto r = eval("(quote (+ 1 2))");
  EXPECT_TRUE(std::holds_alternative<list_ptr>(r.v));
}

TEST_F(EvaluatorTest, While) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                     "  sum)"),
            10); // 0+1+2+3+4
}

TEST_F(EvaluatorTest, For) {
  EXPECT_EQ(eval_int("(begin (set sum 0) "
                     "  (for x (list 1 2 3 4) (set sum (+ sum x))) "
                     "  sum)"),
            10);
}

// A loop body of several forms runs them all, in order (they used to be dropped after the first).
TEST_F(EvaluatorTest, LoopBodiesRunEveryForm) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (set sum (+ sum i)) (set i (+ i 1))) "
                     "  sum)"),
            10);
  EXPECT_EQ(eval_int("(begin (set sum 0) (set n 0) "
                     "  (for x (list 1 2 3 4) (set sum (+ sum x)) (set n (+ n 1))) "
                     "  (+ (* 100 n) sum))"),
            410);
}

TEST_F(EvaluatorTest, IfAndReturnRejectExtraArguments) {
  EXPECT_THROW(eval("(if t 1 2 3)"), std::runtime_error);
  EXPECT_THROW(eval("(begin (defun f () (return 1 2)) (f))"), std::runtime_error);
}

// ─── Lambda / Closures ────────────────────────────────────────────────────

TEST_F(EvaluatorTest, LambdaBasic) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set add1 (lambda (x) (+ x 1))) "
                     "  (add1 10))"),
            11);
}

TEST_F(EvaluatorTest, LambdaClosure) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set make-adder (lambda (n) (lambda (x) (+ x n)))) "
                     "  (set add5 (make-adder 5)) "
                     "  (add5 10))"),
            15);
}

TEST_F(EvaluatorTest, LambdaVariadic) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set sum-all (lambda (&rest args) (apply + args))) "
                     "  (sum-all 1 2 3))"),
            6);
}

// ─── Defun ─────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, Defun) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun double (x) (* x 2)) "
                     "  (double 21))"),
            42);
}

TEST_F(EvaluatorTest, DefunRecursive) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fact (n) (if (<= n 1) 1 (* n (fact (- n 1))))) "
                     "  (fact 5))"),
            120);
}

// ─── Let ───────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, LetBasic) { EXPECT_EQ(eval_int("(let ((a 10) (b 20)) (+ a b))"), 30); }

TEST_F(EvaluatorTest, LetNested) {
  EXPECT_EQ(eval_int("(let ((x 10)) "
                     "  (let ((y 20)) (+ x y)))"),
            30);
}

// ─── Return ────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, Return) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun early (x) (if (< x 0) (return -1) x)) "
                     "  (early -5))"),
            -1);
}

// ─── Macros ────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, Defmacro) {
  // defmacro does syntactic substitution then evaluates the expanded form.
  EXPECT_EQ(eval_int("(begin "
                     "  (defmacro double-it (x) (* x 2)) "
                     "  (double-it 21))"),
            42);
}

// ─── Eval ──────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, Eval) { EXPECT_EQ(eval_int("(eval (quote (+ 1 2)))"), 3); }

// ─── Root ──────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, Root) {
  auto expr = parse("(root)");
  auto r = ev->evaluate(expr);
  // root should return the expression itself
  EXPECT_TRUE(std::holds_alternative<list_ptr>(r.v));
}

// ─── String builtins ───────────────────────────────────────────────────────

TEST_F(EvaluatorTest, StrConcat) {
  EXPECT_EQ(eval_string("(str-concat \"hello\" \" \" \"world\")"), "hello world");
}

// ─── List builtins ─────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, CarCdrCons) {
  EXPECT_EQ(eval_int("(car (list 10 20 30))"), 10);
  EXPECT_EQ(eval_int("(car (cdr (list 10 20 30)))"), 20);
  EXPECT_EQ(eval_int("(car (cons 5 (list 10 20)))"), 5);
}

TEST_F(EvaluatorTest, ListLength) { EXPECT_EQ(eval_int("(length (list 1 2 3))"), 3); }

// ─── Dict builtins ─────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, DictGetSetAttr) {
  EXPECT_EQ(eval_int("(begin "
                     "  (set obj (dict \"x\" 10 \"y\" 20)) "
                     "  (set-attr obj \"x\" 99) "
                     "  (get-attr obj \"x\"))"),
            99);
}

// ─── Stats ─────────────────────────────────────────────────────────────────

TEST_F(EvaluatorTest, StatsTracking) {
  eval("(+ 1 2)");
  EXPECT_GT(ev->stats().get_step_count(), 0u);
  EXPECT_TRUE(ev->stats().is_complete());
}

// ─── Fibonacci integration ─────────────────────────────────────────────────

TEST_F(EvaluatorTest, Fibonacci) {
  EXPECT_EQ(eval_int("(begin "
                     "  (defun fib (n) "
                     "    (if (<= n 1) n "
                     "      (+ (fib (- n 1)) (fib (- n 2))))) "
                     "  (fib 10))"),
            55);
}

// ─── Special-form dispatch ─────────────────────────────────────────────────

TEST_F(EvaluatorTest, SpecialFormsComeBeforeMacrosAndBindings) {
  EXPECT_EQ(eval_int("(begin (defmacro if (a b c) c) (if t 1 2))"), 1);
  EXPECT_EQ(eval_int("(begin (set while 5) (set i 0) (while (< i 3) (set i (+ i 1))) i)"), 3);
  EXPECT_EQ(eval_int("(begin (defun bxgin (x) (* x 2)) (bxgin 21))"), 42);
  EXPECT_EQ(eval_int("(begin (defmacro sett (v) (set v 7)) (sett y) y)"), 7);
}

TEST_F(EvaluatorTest, YieldAndBreakAreOrdinaryNamesHere) {
  // The recursive evaluator has no generators or loop breaks: these are functions like any.
  EXPECT_EQ(eval_int("(begin (defun yield (x) (* x 2)) (yield 21))"), 42);
  EXPECT_EQ(eval_int("(begin (defun break (x) (+ x 1)) (break 41))"), 42);
}

// ─── Step counting ─────────────────────────────────────────────────────────

TEST(EvaluationStats, CountsEvalStepsAndChargedStepsExactlyWhileReadConcurrently) {
  // The stepping thread counts its eval steps without a locked add; steps charged by add_steps()
  // from another thread meanwhile, and readers, see exact totals.
  evaluation_stats stats;
  stats.start();
  constexpr uint64_t kSteps = 200000, kCharged = 50000;
  std::atomic<bool> go{false};
  std::thread charger([&] {
    while (!go.load())
      std::this_thread::yield();
    for (uint64_t i = 0; i < kCharged; ++i)
      stats.add_steps(step_kind::async, 1);
  });
  std::thread reader([&] {
    uint64_t last = 0;
    while (!go.load())
      std::this_thread::yield();
    for (int i = 0; i < 100000; ++i) {
      const uint64_t now = stats.get_step_count();
      EXPECT_GE(now, last);
      last = now;
    }
  });
  go.store(true);
  for (uint64_t i = 0; i < kSteps; ++i)
    stats.increment_step(i % 10 == 0 ? step_kind::data : step_kind::eval);
  charger.join();
  reader.join();
  EXPECT_EQ(stats.get_step_count(), kSteps + kCharged);
  EXPECT_EQ(stats.steps(step_kind::data), kSteps / 10);
  EXPECT_EQ(stats.steps(step_kind::async), kCharged);
  EXPECT_EQ(stats.steps(step_kind::eval), kSteps - kSteps / 10);
  evaluation_stats moved(std::move(stats));
  EXPECT_EQ(moved.get_step_count(), kSteps + kCharged);
  EXPECT_EQ(moved.steps(step_kind::eval), kSteps - kSteps / 10);
  moved.start();
  EXPECT_EQ(moved.get_step_count(), 0u);
}

TEST(EvaluatorSpareVectors, KeepNoValues) {
  // A form's argument vectors are kept for the next forms, empty: what they held is dropped when
  // the form is done, as before.
  auto env = builtins::make_default_environment();
  std::weak_ptr<std::vector<value_t>> made;
  env->set("mk", value_t{native_fn{[&](std::span<const value_t>) -> value_t {
             value_t l = make_list({value_t(int64_t(1)), value_t(int64_t(2))});
             made = std::get<list_ptr>(l.v);
             return l;
           }}});
  env->set("alive", value_t{native_fn{[&](std::span<const value_t>) -> value_t {
             return value_t(!made.expired());
           }}});
  evaluator ev(env);
  EXPECT_FALSE(
      ev.evaluate_script("(begin (length (mk)) (if t (length (mk))) (alive))").is_truthy());
  EXPECT_TRUE(ev.evaluate_script("(begin (set keep (mk)) (alive))").is_truthy());
}
