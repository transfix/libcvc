#include <atomic>
#include <chrono>
#include <cvc/state/state_exec/async_evaluator.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <thread>
#include <vector>

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

// ─── Generators ────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, YieldInGenerator) {
  auto result = eval(R"(
    (let ((g (generator (lambda ()
                (yield 10)
                (yield 20)
                (yield 30)))))
      (list (next g) (next g) (next g)))
  )");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 3u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 10);
  EXPECT_EQ(std::get<int64_t>((**lst)[1].v), 20);
  EXPECT_EQ(std::get<int64_t>((**lst)[2].v), 30);
}

TEST_F(StacklessEvaluatorTest, GeneratorExhaustion) {
  auto result = eval(R"(
    (let ((g (generator (lambda () (yield 1)))))
      (list (next g) (next g)))
  )");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 2u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 1);
  EXPECT_TRUE((**lst)[1].is_nil());
}

TEST_F(StacklessEvaluatorTest, YieldWithComputation) {
  auto result = eval(R"(
    (let ((g (generator (lambda ()
                (yield (* 2 3))
                (yield (+ 10 5))))))
      (+ (next g) (next g)))
  )");
  EXPECT_EQ(std::get<int64_t>(result.v), 21); // 6 + 15
}

TEST_F(StacklessEvaluatorTest, RangeLazy) {
  // range should produce values lazily
  auto result = eval(R"(
    (let ((r (range 0 5)))
      (+ (next r) (next r) (next r)))
  )");
  EXPECT_EQ(std::get<int64_t>(result.v), 3); // 0+1+2
}

TEST_F(StacklessEvaluatorTest, CollectRange) {
  auto result = eval("(collect (range 5))");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 5u);
}

TEST_F(StacklessEvaluatorTest, ForWithGenerator) {
  auto result = eval(R"(
    (let ((total 0))
      (for x (range 1 6)
        (set total (+ total x)))
      total)
  )");
  EXPECT_EQ(std::get<int64_t>(result.v), 15); // 1+2+3+4+5
}

TEST_F(StacklessEvaluatorTest, ForWithClosureGenerator) {
  auto result = eval(R"(
    (let ((accum (list)))
      (for x (generator (lambda ()
               (yield "a")
               (yield "b")
               (yield "c")))
        (append accum x))
      (length accum))
  )");
  EXPECT_EQ(std::get<int64_t>(result.v), 3);
}

TEST_F(StacklessEvaluatorTest, GeneratorWithWhileLoop) {
  // Generator uses internal while loop to produce values
  auto result = eval(R"(
    (collect (generator (lambda ()
      (let ((i 0))
        (while (< i 4)
          (begin (yield i)
                 (set i (+ i 1))))))))
  )");
  auto *lst = std::get_if<list_ptr>(&result.v);
  ASSERT_NE(lst, nullptr);
  ASSERT_EQ((*lst)->size(), 4u);
  EXPECT_EQ(std::get<int64_t>((**lst)[0].v), 0);
  EXPECT_EQ(std::get<int64_t>((**lst)[3].v), 3);
}

// ─── Break ─────────────────────────────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, BreakWhileNoValue) {
  // (break) exits the while loop, loop returns nil
  auto r = eval("(while t (break))");
  EXPECT_TRUE(std::holds_alternative<std::monostate>(r.v));
}

TEST_F(StacklessEvaluatorTest, BreakWhileWithValue) {
  // (break 42) exits the while loop with value 42
  EXPECT_EQ(eval_int("(while t (break 42))"), 42);
}

TEST_F(StacklessEvaluatorTest, BreakWhileAfterIterations) {
  EXPECT_EQ(eval_int("(begin (set i 0) "
                     "  (while t "
                     "    (if (= i 5) (break i) "
                     "      (set i (+ i 1)))))"),
            5);
}

TEST_F(StacklessEvaluatorTest, BreakForList) {
  // Break out of for loop when element matches
  EXPECT_EQ(eval_int("(for x (list 1 2 3 4 5) "
                     "  (if (= x 3) (break x) nil))"),
            3);
}

TEST_F(StacklessEvaluatorTest, BreakForGenerator) {
  // Break out of for-over-generator early
  EXPECT_EQ(eval_int("(for x (range 1000) "
                     "  (if (= x 5) (break x) nil))"),
            5);
}

TEST_F(StacklessEvaluatorTest, BreakNestedLoops) {
  // Break only exits innermost loop
  EXPECT_EQ(eval_int("(begin (set sum 0) "
                     "  (for i (list 1 2 3) "
                     "    (for j (list 10 20 30) "
                     "      (if (= j 20) (break nil) "
                     "        (set sum (+ sum j))))) "
                     "  sum)"),
            30); // 10 + 10 + 10 (inner breaks at 20 each time)
}

TEST_F(StacklessEvaluatorTest, BreakInsideBegin) {
  // Break inside begin should still unwind to the loop
  EXPECT_EQ(eval_int("(begin (set i 0) "
                     "  (while t "
                     "    (begin "
                     "      (set i (+ i 1)) "
                     "      (if (= i 3) (break i) nil))))"),
            3);
}

TEST_F(StacklessEvaluatorTest, BreakInsideLet) {
  // Break inside let should unwind through let to the loop
  EXPECT_EQ(eval_int("(for x (list 1 2 3 4 5) "
                     "  (let ((doubled (* x 2))) "
                     "    (if (> doubled 6) (break doubled) nil)))"),
            8); // x=4, doubled=8, break returns 8
}

// -- loop bodies of several forms, and extra arguments -------------------------
//
// A while or for body of several forms runs them all, in order, as a begin. Before, only the first
// ran and the rest were dropped without a word: a counter bumped in the second form never moved.

TEST_F(StacklessEvaluatorTest, WhileRunsEveryBodyForm) {
  EXPECT_EQ(eval_int("(begin (set i 0) (set sum 0) "
                     "  (while (< i 5) (set sum (+ sum i)) (set i (+ i 1))) "
                     "  sum)"),
            10);
}

TEST_F(StacklessEvaluatorTest, ForRunsEveryBodyForm) {
  EXPECT_EQ(eval_int("(begin (set sum 0) (set n 0) "
                     "  (for x (list 1 2 3 4) (set sum (+ sum x)) (set n (+ n 1))) "
                     "  (+ (* 100 n) sum))"),
            410);
}

TEST_F(StacklessEvaluatorTest, ForOverAGeneratorRunsEveryBodyForm) {
  EXPECT_EQ(eval_int("(begin (set sum 0) (set n 0) "
                     "  (for x (range 4) (set sum (+ sum x)) (set n (+ n 1))) "
                     "  (+ (* 100 n) sum))"),
            406);
}

TEST_F(StacklessEvaluatorTest, BreakInAMultiFormBodyEndsTheLoop) {
  // The forms after the break don't run on the iteration that breaks.
  EXPECT_EQ(eval_int("(begin (set i 0) (set after 0) "
                     "  (set r (while t (set i (+ i 1)) (if (= i 3) (break (* 10 i)) nil) "
                     "    (set after (+ after 1)))) "
                     "  (+ r after))"),
            32); // r = 30; `after` counted iterations 1 and 2
}

TEST_F(StacklessEvaluatorTest, ALoopIsStillNil) {
  EXPECT_TRUE(eval("(begin (set i 0) (while (< i 2) (set i (+ i 1)) i))").is_nil());
  EXPECT_TRUE(eval("(for x (list 1 2) x x)").is_nil());
}

TEST_F(StacklessEvaluatorTest, SpecialFormsRejectExtraArguments) {
  // These used to drop the extra arguments silently.
  const std::pair<const char *, const char *> cases[] = {
      {"(if t 1 2 3)", "if:"},
      {"(begin (defun f () (return 1 2)) (f))", "return:"},
      {"(collect (generator (lambda () (yield 1 2))))", "yield:"},
      {"(while t (break 1 2))", "break:"},
  };
  for (const auto &[script, form] : cases) {
    try {
      eval(script);
      ADD_FAILURE() << script << " should throw";
    } catch (const std::runtime_error &e) {
      EXPECT_NE(std::string(e.what()).find(form), std::string::npos) << script << ": " << e.what();
    }
  }
}

TEST_F(StacklessEvaluatorTest, MultiFormLoopUnderRestrictedForms) {
  // The Ariadne predicate lane allows while, set and begin; a multi-form body works there.
  ev->restrict_special_forms(std::make_shared<const std::set<std::string>>(
      std::set<std::string>{"begin", "while", "set", "if"}));
  EXPECT_EQ(eval_int("(begin (set i 0) (set j 0) (while (< i 3) (set i (+ i 1)) (set j (+ j 2))) "
                     "  j)"),
            6);
}

// -- special-form gating -----------------------------------------------------

TEST_F(StacklessEvaluatorTest, RestrictSpecialFormsDeniesUnlistedForms) {
  auto allowed =
      std::make_shared<const std::set<std::string>>(std::set<std::string>{"if", "begin"});
  ev->restrict_special_forms(allowed);
  EXPECT_NO_THROW(ev->evaluate_script("(if #t 1 2)"));                               // allowed
  EXPECT_NO_THROW(ev->evaluate_script("(begin 1 2)"));                               // allowed
  EXPECT_THROW(ev->evaluate_script("(quote 1)"), std::exception);                    // denied
  EXPECT_THROW(ev->evaluate_script("(defclass C (init (self) 1))"), std::exception); // denied
  // A non-special-form head is unaffected by the gate (the environment governs it).
  EXPECT_NO_THROW(ev->evaluate_script("(+ 1 2)"));
}

TEST_F(StacklessEvaluatorTest, RestrictNullptrLeavesAllFormsAllowed) {
  ev->restrict_special_forms(nullptr); // the default: no restriction
  EXPECT_NO_THROW(ev->evaluate_script("(quote 1)"));
}

// -- cooperative deadline propagates into nested evaluators ------------------

TEST_F(StacklessEvaluatorTest, DeadlineAbortsUncappedNestedMethodEval) {
  // A defclass method body runs on a FRESH nested evaluator with no caps of its own; the
  // per-thread deadline armed by the outer run() must propagate so a looping method aborts
  // at the time budget instead of hanging inside a single outer step.
  auto st = ev->create_state(parse("(begin (defclass B (init (self) (while 1 1))) (B))"));
  EXPECT_THROW(ev->run(st, std::nullopt, /*timeout_sec=*/0.05), evaluation_timeout);
}

TEST_F(StacklessEvaluatorTest, DeadlineIsNotSpentWhileTheThreadDoesNotRun) {
  // The deadline is a time_budget: time the thread does not run -- descheduled on a loaded
  // machine, as a native step that sleeps stands in for here -- cannot spend its second half,
  // so an evaluation that does next to no work completes however long it waits. (One that
  // runs past the budget still aborts: DeadlineAbortsUncappedNestedMethodEval.)
  auto env = builtins::make_default_environment();
  builtins::register_fn(env, "nap", [](std::span<const value_t>) {
    std::this_thread::sleep_for(std::chrono::milliseconds(60));
    return value_t(int64_t{7});
  });
  stackless_evaluator napper(env);
  auto st = napper.create_state(parse("(nap)"));
  EXPECT_EQ(std::get<int64_t>(napper.run(st, std::nullopt, /*timeout_sec=*/0.02).v), 7);
}

// ─── Reading inside the steps (parse_options::step_tokens) ─────────────────

TEST_F(StacklessEvaluatorTest, SlicedReadingEvaluatesLikeParsingUpFront) {
  const std::string script = "(defun sq (x) (* x x)) (set a (sq 7)) (+ a 1)";
  auto eager = ev->create_state(script);
  ASSERT_FALSE(eager.reading);
  const value_t expected = ev->run(eager);

  parse_options popts;
  popts.step_tokens = 3;
  auto sliced = ev->create_state(script, nullptr, popts);
  ASSERT_TRUE(sliced.reading); // nothing parsed yet
  EXPECT_TRUE(sliced.stack.empty());
  const value_t got = ev->run(sliced);
  EXPECT_TRUE(sliced.done);
  EXPECT_FALSE(sliced.reading);
  EXPECT_EQ(to_string(got), to_string(expected));
  EXPECT_EQ(std::get<int64_t>(got.v), 50);
  // The reading steps are steps: 25 tokens at 3 per step take 9 (the ninth reads the last token,
  // finds the end of the script and sets up evaluation).
  EXPECT_EQ(sliced.stats.get_step_count(), eager.stats.get_step_count() + 9);
}

TEST_F(StacklessEvaluatorTest, SlicedReadingThrowsSyntaxErrorsFromStep) {
  parse_options popts;
  popts.step_tokens = 2;
  evaluator_state st;
  ASSERT_NO_THROW(st = ev->create_state(std::string("(+ 1 2) (+ 3"), nullptr, popts));
  EXPECT_THROW(ev->run(st), parse_error);
}

TEST_F(StacklessEvaluatorTest, SlicedReadingOfAnEmptyScriptFinishesWithNil) {
  parse_options popts;
  popts.step_tokens = 4;
  auto st = ev->create_state(std::string("  ; nothing here\n"), nullptr, popts);
  EXPECT_TRUE(ev->step(st));
  EXPECT_TRUE(st.done);
  EXPECT_TRUE(st.result.is_nil());
}

// ─── Special-form dispatch (special_forms.h) ──────────────────────────────

#include "../state/state_exec/special_forms.h"

TEST(SpecialFormOf, NamesEachSpecialFormAndNothingElse) {
  using detail::special_form;
  std::set<special_form> seen;
  for (const std::string &name : stackless_evaluator::all_special_forms()) {
    const special_form f = detail::special_form_of(name);
    EXPECT_NE(f, special_form::none) << name;
    EXPECT_TRUE(seen.insert(f).second) << name << " shares its form with another name";
    // Every one-letter change, insertion and deletion is not that form (it may be another).
    for (std::size_t i = 0; i <= name.size(); ++i) {
      for (char c : std::string("abcdefghijklmnopqrstuvwxyz-!?.")) {
        std::vector<std::string> near;
        if (i < name.size()) {
          std::string changed = name;
          changed[i] = c;
          near.push_back(changed);
        }
        near.push_back(name.substr(0, i) + c + name.substr(i));
        for (const std::string &n : near)
          if (n != name && !stackless_evaluator::all_special_forms().count(n))
            EXPECT_EQ(detail::special_form_of(n), special_form::none) << n;
      }
      if (i < name.size()) {
        const std::string shorter = name.substr(0, i) + name.substr(i + 1);
        if (!stackless_evaluator::all_special_forms().count(shorter))
          EXPECT_EQ(detail::special_form_of(shorter), special_form::none) << shorter;
      }
    }
  }
  EXPECT_EQ(seen.size(), stackless_evaluator::all_special_forms().size());
  for (const char *n : {"", "i", "IF", "If", "begin ", "defmacrox", "defclas", "defxlass", "rot"})
    EXPECT_EQ(detail::special_form_of(n), special_form::none) << n;
}

TEST_F(StacklessEvaluatorTest, SpecialFormsComeBeforeMacrosAndBindings) {
  // A macro or a binding can't take a special form's name; a name that only looks like one is an
  // ordinary function.
  EXPECT_EQ(eval_int("(begin (defmacro if (a b c) c) (if t 1 2))"), 1);
  EXPECT_EQ(eval_int("(begin (set while 5) (set i 0) (while (< i 3) (set i (+ i 1))) i)"), 3);
  EXPECT_EQ(eval_int("(begin (defun bxgin (x) (* x 2)) (bxgin 21))"), 42);
  EXPECT_EQ(eval_int("(begin (defun defclasx (x) (+ x 1)) (defclasx 41))"), 42);
  EXPECT_EQ(eval_int("(begin (defmacro sett (v) (set v 7)) (sett y) y)"), 7);
}

TEST_F(StacklessEvaluatorTest, RestrictedFormsStillGateOnlySpecialForms) {
  ev->restrict_special_forms(
      std::make_shared<const std::set<std::string>>(std::set<std::string>{"begin", "defun", "if"}));
  EXPECT_EQ(eval_int("(begin (defun whilex (x) (if x 1 2)) (whilex t))"), 1);
  EXPECT_THROW(eval("(begin (while nil 1))"), std::exception);
  EXPECT_THROW(eval("(set x 1)"), std::exception);
}

// ─── Step counts ───────────────────────────────────────────────────────────
//
// Step counts are part of the semantics (max_steps budgets, the async step rate): a faster
// evaluator must take exactly the steps it took before, of each kind. These were taken from the
// evaluator before the single-thread performance work (and must not change).

namespace {
struct step_counts {
  uint64_t all, eval, data;
};

step_counts count_steps(const std::string &script, std::size_t data_step_nodes) {
  stackless_evaluator ev(builtins::make_default_environment());
  evaluator_state st = ev.create_state(parse(script));
  st.data_step_nodes = data_step_nodes;
  while (!ev.step(st)) {
  }
  return {st.stats.get_step_count(), st.stats.steps(step_kind::eval),
          st.stats.steps(step_kind::data)};
}
} // namespace

TEST(StacklessStepCounts, ArePinned) {
  struct pinned {
    const char *script;
    std::size_t data_step_nodes;
    step_counts expected;
  };
  const std::vector<pinned> programs = {
      {"(+ 1 2)", 0, {6, 6, 0}},
      {"(begin (set s 0) (set i 0) (while (< i 10) (begin (set s (+ s i)) (set i (+ i 1)))) s)",
       0,
       {290, 290, 0}},
      {"(begin (defun fib (n) (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) (fib 8))",
       0,
       {1371, 1371, 0}},
      {"(begin (defun f (a b) (set c (+ a b)) (set d (* c 2)) (- d a)) (set k 0) (set tot 0) "
       "(while (< k 5) (begin (set tot (+ tot (f k 1))) (set k (+ k 1)))) tot)",
       0,
       {312, 312, 0}},
      {"(begin (set s 0) (for x (list 1 2 3 4) (set s (+ s x))) s)", 0, {56, 56, 0}},
      {"(let ((a 1) (b (+ 1 1))) (set c (+ a b)) (* c 2))", 0, {26, 26, 0}},
      {"(begin (set r 0) (set r (+ r (try (raise (ValueError \"x\")) (catch (ValueError e) 1)))) "
       "(set r (+ r (try 2 (catch (ValueError e) 0) (finally 3)))) r)",
       0,
       {39, 39, 0}},
      {"(begin (defmacro inc (v) (set v (+ v 1))) (set z 0) (inc z) (inc z) z)", 0, {29, 29, 0}},
      {"(begin (if nil 1) (list) (quote (a b)) (begin))", 0, {12, 12, 0}},
      {"(begin (defclass A () (init (self) (set-attr self \"v\" 1) self) (get (self) "
       "(get-attr self \"v\"))) (defclass B (A) (get (self) (+ 1 (super self get)))) "
       "(set o (B)) (send o \"get\"))",
       0,
       {47, 43, 4}},
      {"(begin (set s 0) (for x (range 5) (set s (+ s x))) s)", 0, {59, 59, 0}},
      {"(begin (set s 0) (for x (generator (lambda () (yield 1) (yield 2))) (set s (+ s x))) s)",
       0,
       {47, 47, 0}},
      {"(begin (set l (list 1 2 3 4 5 6 7 8 9 10)) (set m (list l l l)) (set q (str m)) "
       "(set l nil) (set m nil) (length q))",
       4,
       {67, 57, 10}},
      {"(begin (defun mk (n) (list n n n n n n)) (set i 0) (while (< i 4) (begin (mk i) "
       "(set i (+ i 1)))) (collections.map (list 1 2 3) (lambda (x) (* x 2))))",
       3,
       {200, 196, 4}},
  };
  for (const pinned &p : programs) {
    const step_counts got = count_steps(p.script, p.data_step_nodes);
    EXPECT_EQ(got.all, p.expected.all) << p.script;
    EXPECT_EQ(got.eval, p.expected.eval) << p.script;
    EXPECT_EQ(got.data, p.expected.data) << p.script;
  }
}

TEST(StacklessStepCounts, CountedFromAnotherThreadWhileTheEvaluatorSteps) {
  // pycvc wraps evaluation_stats::increment_step() and releases the GIL while an evaluator
  // steps: a count another thread makes meanwhile adds to the evaluator's own, neither losing the
  // other's.
  const char *const script =
      "(begin (set s 0) (set i 0) (while (< i 20000) (begin (set s (+ s i)) (set i (+ i 1)))) s)";
  const uint64_t alone = count_steps(script, 0).all;
  stackless_evaluator ev(builtins::make_default_environment());
  evaluator_state st = ev.create_state(parse(script));
  std::atomic<bool> stepping{true};
  std::atomic<uint64_t> counted{0};
  std::thread other([&] {
    while (stepping.load(std::memory_order_relaxed)) {
      st.stats.increment_step();
      counted.fetch_add(1, std::memory_order_relaxed);
    }
  });
  while (counted.load() == 0) // stepping overlaps the other thread's counting
    std::this_thread::yield();
  while (!ev.step(st)) {
  }
  stepping = false;
  other.join();
  EXPECT_EQ(st.stats.get_step_count(), alone + counted.load());
  EXPECT_EQ(st.stats.steps(step_kind::eval), alone + counted.load());
}

TEST(StacklessStepCounts, CountedFromAnotherThreadWhileTheEvaluatorReadsAndWalksData) {
  // The evaluator counts its read and data steps with increment_step() too, on the counter
  // another thread's increment_step() adds to: both are locked adds, so a count made meanwhile
  // isn't lost either.
  std::string script = "(set l (list 1 2 3 4 5 6 7 8)) (set n 0)";
  for (int i = 0; i < 2000; ++i)
    script += " (set n (+ n (length (str (list l l)))))";
  parse_options popts;
  popts.step_tokens = 1; // a read step per token
  struct counts {
    uint64_t all, eval, read, data;
  };
  const auto run = [&] {
    stackless_evaluator ev(builtins::make_default_environment());
    evaluator_state st = ev.create_state(script, nullptr, popts);
    st.data_step_nodes = 2;
    while (!ev.step(st)) {
    }
    return counts{st.stats.get_step_count(), st.stats.steps(step_kind::eval),
                  st.stats.steps(step_kind::read), st.stats.steps(step_kind::data)};
  };
  const counts alone = run();
  ASSERT_GT(alone.read, 10000u);
  ASSERT_GT(alone.data, 10000u);

  stackless_evaluator ev(builtins::make_default_environment());
  evaluator_state st = ev.create_state(script, nullptr, popts);
  st.data_step_nodes = 2;
  std::atomic<uint64_t> counted{0};
  const auto counting_while = [&](const std::function<void()> &steps) {
    std::atomic<bool> stepping{true};
    const uint64_t before = counted.load();
    std::thread other([&] {
      while (stepping.load(std::memory_order_relaxed)) {
        st.stats.increment_step();
        counted.fetch_add(1, std::memory_order_relaxed);
      }
    });
    while (counted.load() == before) // the steps overlap the other thread's counting
      std::this_thread::yield();
    steps();
    stepping = false;
    other.join();
  };
  // Every read step but the last, which ends the reading by moving the stats into the program's
  // state, copying the counts (evaluation_stats::increment_step()); then the program's steps.
  counting_while([&] {
    for (uint64_t i = 1; i < alone.read; ++i)
      ASSERT_FALSE(ev.step(st));
  });
  ASSERT_TRUE(st.reading);
  ASSERT_FALSE(ev.step(st));
  ASSERT_FALSE(st.reading);
  counting_while([&] {
    while (!ev.step(st)) {
    }
  });
  EXPECT_EQ(st.stats.get_step_count(), alone.all + counted.load());
  EXPECT_EQ(st.stats.steps(step_kind::eval), alone.eval + counted.load());
  EXPECT_EQ(st.stats.steps(step_kind::read), alone.read);
  EXPECT_EQ(st.stats.steps(step_kind::data), alone.data);
}

TEST_F(StacklessEvaluatorTest, PoppedFramesKeepNoValues) {
  // A popped frame's vectors are kept for the next frames, empty: what the frame held is dropped
  // when it pops, as before.
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
  stackless_evaluator local(env);
  EXPECT_FALSE(
      local.evaluate_script("(begin (length (mk)) (if t (length (mk))) (alive))").is_truthy());
  EXPECT_TRUE(local.evaluate_script("(begin (set keep (mk)) (alive))").is_truthy());
}

// ─── A call's function, found once ─────────────────────────────────────────

TEST_F(StacklessEvaluatorTest, CallSeesItsFunctionRebound) {
  // The apply step uses the function its init found only while that is what a lookup would
  // find: rebound in place, shadowed in a nearer scope, or bound for the first time meanwhile.
  EXPECT_EQ(eval_int("(begin (defun f (x) 1) (f (begin (defun f (x) 2) 0)))"), 2);
  EXPECT_EQ(eval_int("(begin (defun g (x) 1) "
                     "(defun h () (g (begin (set g (lambda (y) 7)) 0))) (h))"),
            7);
  EXPECT_EQ(eval_int("(begin (defun k () 1) (defun m () (k)) "
                     "(+ (m) (begin (defun k () 10) 0) (m)))"),
            11);
  try {
    eval("(begin (defun g (x) 1) (defun h () (g (begin (set g 5) 0))) (h))");
    FAIL() << "expected a TypeError";
  } catch (const std::exception &e) {
    EXPECT_NE(std::string(e.what()).find("apply: not callable: g"), std::string::npos) << e.what();
  }
}

#include "../state/state_exec/scope_mark.h"

TEST(StacklessCallee, NoteWhereTheFunctionIsAndCheckIt) {
  // The frame a call's init leaves holds the binding it found and marks the scopes it walked; a
  // frame whose scopes gained a binding since looks the name up again.
  auto env = builtins::make_default_environment();
  stackless_evaluator ev(env);
  evaluator_state st = ev.create_state(parse("(+ 1 2)"));
  ASSERT_FALSE(ev.step(st)); // init: (+ 1 2) waits for its arguments
  const eval_frame &call = st.stack.front();
  ASSERT_NE(call.callee, nullptr);
  EXPECT_EQ(call.callee->first, "+");
  EXPECT_TRUE(std::holds_alternative<native_fn>(call.callee->second.v));
  ASSERT_GE(call.callee_depth, 1u);
  const environment *last = call.env.get();
  for (std::uint32_t i = 1; i < call.callee_depth; ++i)
    last = last->outer.get();
  EXPECT_EQ(last, env.get()); // the walk ended at the scope that binds +
  EXPECT_TRUE(detail::mark_holds(call.env.get(), {call.callee_depth, call.callee_stamp}));
  // A binding of + in the process scope, nearer than the builtins: the apply step finds it.
  st.global_env->set("+", value_t{native_fn{[](std::span<const value_t>) -> value_t {
                       return value_t(int64_t(99));
                     }}});
  EXPECT_FALSE(detail::mark_holds(call.env.get(), {call.callee_depth, call.callee_stamp}));
  while (!ev.step(st)) {
  }
  EXPECT_EQ(std::get<int64_t>(st.result.v), 99);
}

TEST(StacklessCallee, ARewiredChainIsLookedUpAgain) {
  // E -> X -> G, G binding f, and the call (f ...) initialized in E. A host then makes E's
  // enclosing scope M, which binds f too and has as many bindings as X: the apply step finds M's.
  auto G = builtins::make_default_environment();
  stackless_evaluator ev(G);
  ev.evaluate_script("(defun f (x) 1)", G);
  const value_t two = ev.evaluate_script("(lambda (x) 2)", G);
  auto X = environment::extend(G);
  X->set("pad", value_t(int64_t(0)));
  auto M = environment::extend(G);
  M->set("f", two);
  auto E = environment::extend(X);
  evaluator_state st = ev.create_state(parse("(f (+ 0 0))"), E);
  while (st.stack.size() < 2)
    ASSERT_FALSE(ev.step(st)); // past the call's init: its argument is being evaluated
  E->outer = M;
  while (!ev.step(st)) {
  }
  EXPECT_EQ(std::get<int64_t>(st.result.v), 2);
}

TEST(StacklessCallee, AScopeMadeWhereADroppedOneWasIsLookedUpAgain) {
  // E -> O -> G, O binding f, and the call (f ...) initialized in E. A host (as pycvc can) then
  // drops O, setting E's outer to None, makes O2 in G with as many bindings, f among them (a dict
  // assigned to its bindings: nodes of its own), and makes it E's outer. O2 is likely in O's
  // block, at O's address (extend() hands out the block the last dropped scope left), while the
  // node that bound O's f is kept for reuse, or freed: the apply step looks f up again.
  auto G = builtins::make_default_environment();
  stackless_evaluator ev(G);
  const auto returns = [](int64_t r) {
    return value_t{native_fn{[r](std::span<const value_t>) -> value_t { return value_t(r); }}};
  };
  std::vector<environment_ptr> taken; // room in this thread's pool for O
  for (int i = 0; i < 64; ++i)
    taken.push_back(environment::extend(nullptr));
  auto E = environment::extend(environment::extend(G));
  E->outer->set("f", returns(1));
  E->outer->set("pad", value_t(int64_t(0)));
  evaluator_state st = ev.create_state(parse("(f (+ 0 0))"), E);
  while (st.stack.size() < 2)
    ASSERT_FALSE(ev.step(st)); // past the call's init: its argument is being evaluated
  ASSERT_NE(st.stack.front().callee, nullptr);
  const environment *was = E->outer.get();
  const bool kept =
      was->bindings.bucket_count() == decltype(environment::bindings)().bucket_count();
  E->outer = nullptr; // O goes
  auto O2 = environment::extend(G);
  O2->bindings.emplace("f", returns(2));
  O2->bindings.emplace("pad", value_t(int64_t(0)));
  if (kept)
    EXPECT_EQ(O2.get(), was); // what this is about: O2 in O's block (on MSVC)
  E->outer = O2;
  while (!ev.step(st)) {
  }
  EXPECT_EQ(std::get<int64_t>(st.result.v), 2);
}

// ─── A function passed as a parameter, called in tail position ────────────
//
// The call's frame is then the only owner of the scope that binds the function: the call must
// take what it needs from the closure before that frame goes.

namespace {
const char *const kTailCalls[][2] = {
    {"(begin (defun inc (n) (+ n 1)) (defun twice (f x) (f (f x))) (twice inc 5))", "7"},
    {"(begin (defun call-it (h) (h)) (call-it (lambda () 42)))", "42"},
    {"(begin (defun call-it (h) (h 1)) (call-it (lambda (a) (set b (+ a 1)) (* b 21))))", "42"},
    {"(begin (defun app (g v) (g v)) (app (lambda (v) (app (lambda (w) (* w 6)) v)) 7))", "42"},
};
} // namespace

TEST_F(StacklessEvaluatorTest, FunctionParameterCalledInTailPosition) {
  for (const auto &[script, want] : kTailCalls)
    EXPECT_EQ(std::to_string(eval_int(script)), want) << script;
}

TEST(StacklessTailCall, FunctionParameterCalledInTailPositionInAScheduler) {
  for (const auto &[script, want] : kTailCalls) {
    {
      scheduler sched;
      const int pid = sched.execute(std::string(script));
      auto results = sched.run();
      EXPECT_EQ(std::to_string(std::get<int64_t>(results.at(pid).v)), want) << script;
    }
    {
      async_scheduler sched;
      const int pid = sched.execute(std::string(script));
      auto results = sched.sync_run();
      EXPECT_EQ(std::to_string(std::get<int64_t>(results.at(pid).v)), want) << script;
    }
  }
}

// ─── The last frame's parts go before the state's old result ──────────────

TEST(StacklessLastFrame, PartsAreDroppedBeforeTheOldResult) {
  // As pop_frame drops them: what the last frame held first, then the result it replaces (with
  // a release backlog, that order decides how freeing is split across data steps).
  std::vector<std::string> dropped;
  struct note {
    std::vector<std::string> *log = nullptr;
    std::string name;
    ~note() {
      if (log)
        log->push_back(name);
    }
  };
  auto noted = [&](const char *name) {
    auto n = std::make_shared<note>();
    n->log = &dropped;
    n->name = name;
    return value_t{native_fn{[n](std::span<const value_t>) -> value_t { return value_t(); }}};
  };
  auto env = builtins::make_default_environment();
  env->set("mk",
           value_t{native_fn{[&](std::span<const value_t>) -> value_t { return noted("frame"); }}});
  stackless_evaluator ev(env);
  for (const char *script : {"(begin (mk) 1)", "(if (mk) 1 2)"}) {
    dropped.clear();
    evaluator_state st = ev.create_state(parse(script));
    st.result = noted("result"); // as a generator's last yielded value would be
    while (!ev.step(st)) {
    }
    EXPECT_EQ(std::get<int64_t>(st.result.v), 1) << script;
    EXPECT_EQ(dropped, (std::vector<std::string>{"frame", "result"})) << script;
  }
}

// ─── A closure's body, built once ──────────────────────────────────────────

TEST(StacklessClosureBody, MadeOnceAndIgnoredWhenStale) {
  stackless_evaluator ev(builtins::make_default_environment());
  evaluator_state st =
      ev.create_state(parse("(begin (defun f (x) (set y x) (* y 2)) (defun g (x) (* x 3)) f)"));
  while (!ev.step(st)) {
  }
  const closure_ptr f = std::get<closure_ptr>(st.result.v);
  ASSERT_EQ(f->body.size(), 2u);
  const list_ptr *made = &f->body_expr;
  ASSERT_NE(*made, nullptr);
  ASSERT_EQ((*made)->size(), 3u);
  EXPECT_EQ(std::get<symbol>((**made)[0].v).name, "begin");
  EXPECT_EQ(std::get<list_ptr>((**made)[1].v), std::get<list_ptr>(f->body[0].v));
  EXPECT_TRUE(std::get<closure_ptr>(st.global_env->lookup("g")->v)->body_expr == nullptr);

  auto call = [&](const char *script) {
    evaluator_state run = ev.create_state(parse(script), st.global_env);
    while (!ev.step(run)) {
    }
    return std::get<int64_t>(run.result.v);
  };
  EXPECT_EQ(call("(f 4)"), 8);
  // A body changed after the closure was made: the call runs the body it has now.
  f->body[1] = parse("(* y 5)");
  EXPECT_EQ(call("(f 4)"), 20);
  f->body.push_back(parse("(+ y 1)"));
  EXPECT_EQ(call("(f 4)"), 5);
}

TEST(StacklessClosureBody, SharedByTheClosuresOneFormMakes) {
  // Closures made in a loop from one lambda form share one (begin ...): a closure costs no list
  // of its own. Another form's closures have their own.
  stackless_evaluator ev(builtins::make_default_environment());
  const value_t made = ev.evaluate_script(
      "(begin (set fs (list)) (set i 0) "
      "(while (< i 5) (begin (append fs (lambda (x) (set y x) (+ y i))) (set i (+ i 1)))) "
      "(append fs (lambda (x) (set y x) (+ y 1))) fs)");
  const auto &fs = *std::get<list_ptr>(made.v);
  ASSERT_EQ(fs.size(), 6u);
  const list_ptr first = std::get<closure_ptr>(fs[0].v)->body_expr;
  ASSERT_NE(first, nullptr);
  for (std::size_t i = 1; i < 5; ++i)
    EXPECT_EQ(std::get<closure_ptr>(fs[i].v)->body_expr, first) << i;
  const list_ptr other = std::get<closure_ptr>(fs[5].v)->body_expr;
  ASSERT_NE(other, nullptr);
  EXPECT_NE(other, first);
}

TEST(StacklessClosureBody, EachGeneratorHasABodyOfItsOwn) {
  // (root) in a generator is its body; one generator changing it changes no other's.
  stackless_evaluator ev(builtins::make_default_environment());
  const value_t got =
      ev.evaluate_script("(defun body () (append (root) (quote (yield 99))) (yield 1))"
                         " (set a (generator body)) (set b (generator body))"
                         " (next a) (list (next b) (next b))");
  const auto &l = *std::get<list_ptr>(got.v);
  ASSERT_EQ(l.size(), 2u);
  EXPECT_EQ(std::get<int64_t>(l[0].v), 1);
  EXPECT_TRUE(l[1].is_nil()) << to_string(l[1]);
}

#include "../state/state_exec/closure_body.h"

#include <cmath>

TEST(StacklessClosureBody, FormsAreTheSameOnlyIfTheyEvaluateTheSame) {
  // A number is the same form only bit for bit: 0.0 and -0.0 are equal numbers but different
  // forms, and a NaN is the same form as itself.
  EXPECT_TRUE(detail::same_form(value_t(0.0), value_t(0.0)));
  EXPECT_FALSE(detail::same_form(value_t(0.0), value_t(-0.0)));
  EXPECT_FALSE(detail::same_form(value_t(-0.0), value_t(0.0)));
  EXPECT_TRUE(detail::same_form(value_t(std::nan("")), value_t(std::nan(""))));
  EXPECT_FALSE(detail::same_form(value_t(1.0), value_t(int64_t(1))));
  EXPECT_TRUE(detail::same_form(value_t(int64_t(3)), value_t(int64_t(3))));
  EXPECT_TRUE(detail::same_form(value_t(symbol{"x"}), value_t(symbol{"x"})));
  EXPECT_TRUE(detail::same_form(value_t(std::string("s")), value_t(std::string("s"))));
  EXPECT_FALSE(detail::same_form(parse("(a)"), parse("(a)"))); // lists: the same object only
}

TEST(StacklessClosureBody, ABodyIsSharedOnlyWithFormsThatEvaluateTheSame) {
  // A lambda form at the address of a freed one, whose body differs only in the sign of a zero,
  // gets a (begin ...) of its own, not the freed form's.
  const std::vector<value_t> negative = {value_t(), value_t(-0.0)};
  const std::vector<value_t> positive = {value_t(), value_t(0.0)};
  const int form = 0; // stands in for the lambda form's list: both at this address
  const list_ptr made = detail::shared_body_expr(&form, negative);
  ASSERT_NE(made, nullptr);
  EXPECT_EQ(detail::shared_body_expr(&form, negative).get(), made.get());
  const list_ptr other = detail::shared_body_expr(&form, positive);
  ASSERT_NE(other, nullptr);
  EXPECT_NE(other.get(), made.get());
  ASSERT_EQ(other->size(), 3u);
  EXPECT_FALSE(std::signbit(std::get<double>((*other)[2].v)));

  // The same through a program: each expansion of the macro is a new lambda form, likely at the
  // address of the last one, freed by then.
  stackless_evaluator ev(builtins::make_default_environment());
  const value_t got = ev.evaluate_script(
      "(defmacro k (v) (lambda () nil v)) (set a (k -0.0)) (set b (k 0.0)) (list (a) (b))");
  const auto &l = *std::get<list_ptr>(got.v);
  ASSERT_EQ(l.size(), 2u);
  EXPECT_TRUE(std::signbit(std::get<double>(l[0].v)));
  EXPECT_FALSE(std::signbit(std::get<double>(l[1].v)));
}

// ─── and / or ──────────────────────────────────────────────────────────────

// (and ...) and (or ...) are special forms in every evaluator: they stop at the first argument
// that decides them, and give what the builtins of the same names give.
TEST(AndOr, EveryEvaluatorStopsAtTheArgumentThatDecides) {
  const std::pair<const char *, const char *> cases[] = {
      // A later argument isn't evaluated: its side effect doesn't happen, its error isn't raised.
      {"(begin (set n 0) (and (set n 1) nil (set n 2)) n)", "1"},
      {"(begin (set n 0) (or nil (set n 1) (set n 2)) n)", "1"},
      {"(begin (set n 0) (and (set n 1) (set n 2)) n)", "2"},
      {"(and nil (no-such-function))", "#f"},
      {"(or 7 (error \"never\"))", "7"},
      {"(begin (set calls 0) (defun load () (set calls (+ calls 1)))"
       " (if (and nil (load)) 1 2) calls)",
       "0"},
      // The values: and is #f at a falsy argument, else the last (#t with none); or is the
      // first truthy one, else #f.
      {"(and)", "#t"},
      {"(or)", "#f"},
      {"(and 1 2 3)", "3"},
      {"(and 1 nil 3)", "#f"},
      {"(and 1 false 3)", "#f"},
      {"(or nil 2 3)", "2"},
      {"(or nil nil)", "#f"},
      {"(or false (not true) \"x\")", "\"x\""},
      // Nested, and in a function body.
      {"(and (or nil 1) (and 2 (or nil nil 3)))", "3"},
      {"(begin (defun f (x) (or (and (> x 1) \"big\") \"small\")) (list (f 5) (f 0)))",
       "(\"big\" \"small\")"},
      // Passed as values, `and` and `or` are the builtins: every argument already evaluated.
      {"(apply and (list 1 2))", "2"},
      {"(apply and (list 1 nil))", "#f"},
      {"(apply or (list nil 5))", "5"},
  };
  const auto env = [] { return builtins::make_default_environment(); };
  stackless_evaluator stackless(env());
  evaluator recursive(env());
  async_evaluator async(env());
  for (const auto &[script, want] : cases) {
    SCOPED_TRACE(script);
    EXPECT_EQ(to_string(stackless.evaluate_script(script)), want) << "stackless";
    EXPECT_EQ(to_string(recursive.evaluate_script(script)), want) << "recursive";
    EXPECT_EQ(to_string(async.sync_evaluate_script(script)), want) << "async";
  }
}

TEST(AndOr, AStacklessAndTakesAStepAnArgument) {
  // 2000 arguments, then a falsy one, then one that would raise: a step each, none native.
  std::string script = "(and";
  for (int i = 0; i < 2000; ++i)
    script += " 1";
  script += " nil (error \"never\"))";
  stackless_evaluator ev(builtins::make_default_environment());
  evaluator_state st = ev.create_state(script);
  int steps = 0;
  while (!ev.step(st))
    ++steps;
  EXPECT_GE(steps, 2000);
  EXPECT_EQ(to_string(st.result), "#f");
  // Nested 5000 deep, on the heap as any form is.
  std::string deep;
  for (int i = 0; i < 5000; ++i)
    deep += "(and 1 (or nil ";
  deep += "7";
  for (int i = 0; i < 5000; ++i)
    deep += "))";
  EXPECT_EQ(to_string(ev.evaluate_script(deep)), "7");
}

TEST(AndOr, ARestrictedEvaluatorGatesThemAsSpecialForms) {
  EXPECT_TRUE(stackless_evaluator::all_special_forms().count("and"));
  EXPECT_TRUE(stackless_evaluator::all_special_forms().count("or"));
  stackless_evaluator ev(builtins::make_default_environment());
  ev.restrict_special_forms(
      std::make_shared<const std::set<std::string>>(std::set<std::string>{"if", "begin"}));
  EXPECT_THROW(ev.evaluate_script("(and 1 2)"), std::exception);
  ev.restrict_special_forms(
      std::make_shared<const std::set<std::string>>(std::set<std::string>{"if", "and", "or"}));
  EXPECT_EQ(to_string(ev.evaluate_script("(if (and 1 (or nil 2)) 3 4)")), "3");
}
