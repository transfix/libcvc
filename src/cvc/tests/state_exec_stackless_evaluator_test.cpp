#include <chrono>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <memory>
#include <set>
#include <thread>

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
