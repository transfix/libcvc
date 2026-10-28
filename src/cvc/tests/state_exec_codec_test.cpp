#include <chrono>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/state/state_list.h>
#include <deque>
#include <gtest/gtest.h>
#include <optional>
#include <thread>

using namespace cvc::state_exec;

// ===========================================================================
// Value round-trip tests
// ===========================================================================

class ValueCodecTest : public ::testing::Test {
protected:
  cvc::app ctx;
  int counter_ = 0;

  // Round-trip a value_t through encode/decode using a fresh node each time
  value_t roundtrip(const value_t &val) {
    std::string name = "__codec_test_" + std::to_string(counter_++) + "__";
    auto &node = ctx.root()(name);
    encode_value(node, val);
    return decode_value(node);
  }
};

TEST_F(ValueCodecTest, Nil) {
  auto result = roundtrip(nil_value);
  EXPECT_TRUE(result.is_nil());
}

TEST_F(ValueCodecTest, BoolTrue) {
  auto result = roundtrip(true_value);
  ASSERT_TRUE(std::holds_alternative<bool>(result.v));
  EXPECT_TRUE(std::get<bool>(result.v));
}

TEST_F(ValueCodecTest, BoolFalse) {
  auto result = roundtrip(false_value);
  ASSERT_TRUE(std::holds_alternative<bool>(result.v));
  EXPECT_FALSE(std::get<bool>(result.v));
}

TEST_F(ValueCodecTest, Integer) {
  auto result = roundtrip(value_t{int64_t(42)});
  ASSERT_TRUE(std::holds_alternative<int64_t>(result.v));
  EXPECT_EQ(std::get<int64_t>(result.v), 42);
}

TEST_F(ValueCodecTest, IntegerNegative) {
  auto result = roundtrip(value_t{int64_t(-999)});
  ASSERT_TRUE(std::holds_alternative<int64_t>(result.v));
  EXPECT_EQ(std::get<int64_t>(result.v), -999);
}

TEST_F(ValueCodecTest, IntegerLarge) {
  auto result = roundtrip(value_t{int64_t(9223372036854775807LL)});
  ASSERT_TRUE(std::holds_alternative<int64_t>(result.v));
  EXPECT_EQ(std::get<int64_t>(result.v), 9223372036854775807LL);
}

TEST_F(ValueCodecTest, Double) {
  auto result = roundtrip(value_t{3.14159265358979});
  ASSERT_TRUE(std::holds_alternative<double>(result.v));
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), 3.14159265358979);
}

TEST_F(ValueCodecTest, DoubleNegative) {
  auto result = roundtrip(value_t{-1.5e10});
  ASSERT_TRUE(std::holds_alternative<double>(result.v));
  EXPECT_DOUBLE_EQ(std::get<double>(result.v), -1.5e10);
}

TEST_F(ValueCodecTest, String) {
  auto result = roundtrip(value_t{std::string("hello world")});
  ASSERT_TRUE(std::holds_alternative<std::string>(result.v));
  EXPECT_EQ(std::get<std::string>(result.v), "hello world");
}

TEST_F(ValueCodecTest, StringEmpty) {
  auto result = roundtrip(value_t{std::string("")});
  ASSERT_TRUE(std::holds_alternative<std::string>(result.v));
  EXPECT_EQ(std::get<std::string>(result.v), "");
}

TEST_F(ValueCodecTest, Symbol) {
  auto result = roundtrip(value_t{symbol{"my-var"}});
  ASSERT_TRUE(std::holds_alternative<symbol>(result.v));
  EXPECT_EQ(std::get<symbol>(result.v).name, "my-var");
}

TEST_F(ValueCodecTest, ListEmpty) {
  auto result = roundtrip(make_list());
  ASSERT_TRUE(std::holds_alternative<list_ptr>(result.v));
  EXPECT_TRUE(std::get<list_ptr>(result.v)->empty());
}

TEST_F(ValueCodecTest, ListOfIntegers) {
  auto result = roundtrip(make_list({value_t{1}, value_t{2}, value_t{3}}));
  ASSERT_TRUE(std::holds_alternative<list_ptr>(result.v));
  auto &elems = *std::get<list_ptr>(result.v);
  ASSERT_EQ(elems.size(), 3u);
  EXPECT_EQ(std::get<int64_t>(elems[0].v), 1);
  EXPECT_EQ(std::get<int64_t>(elems[1].v), 2);
  EXPECT_EQ(std::get<int64_t>(elems[2].v), 3);
}

TEST_F(ValueCodecTest, ListNested) {
  auto inner = make_list({value_t{10}, value_t{20}});
  auto outer = make_list({value_t{1}, inner, value_t{3}});
  auto result = roundtrip(outer);
  ASSERT_TRUE(std::holds_alternative<list_ptr>(result.v));
  auto &elems = *std::get<list_ptr>(result.v);
  ASSERT_EQ(elems.size(), 3u);
  EXPECT_EQ(std::get<int64_t>(elems[0].v), 1);
  ASSERT_TRUE(std::holds_alternative<list_ptr>(elems[1].v));
  auto &inner_r = *std::get<list_ptr>(elems[1].v);
  ASSERT_EQ(inner_r.size(), 2u);
  EXPECT_EQ(std::get<int64_t>(inner_r[0].v), 10);
  EXPECT_EQ(std::get<int64_t>(inner_r[1].v), 20);
  EXPECT_EQ(std::get<int64_t>(elems[2].v), 3);
}

TEST_F(ValueCodecTest, Dict) {
  auto d = make_dict({{"a", value_t{1}}, {"b", value_t{std::string("two")}}});
  auto result = roundtrip(d);
  ASSERT_TRUE(std::holds_alternative<dict_ptr>(result.v));
  auto &entries = *std::get<dict_ptr>(result.v);
  ASSERT_EQ(entries.size(), 2u);
  EXPECT_EQ(entries[0].first, "a");
  EXPECT_EQ(std::get<int64_t>(entries[0].second.v), 1);
  EXPECT_EQ(entries[1].first, "b");
  EXPECT_EQ(std::get<std::string>(entries[1].second.v), "two");
}

TEST_F(ValueCodecTest, Closure) {
  auto env = std::make_shared<environment>();
  env->set("x", value_t{42});
  auto cls = std::make_shared<closure>();
  cls->params = {symbol{"a"}, symbol{"b"}};
  cls->variadic = false;
  cls->body = {make_list({value_t{symbol{"+"}}, value_t{symbol{"a"}}, value_t{symbol{"b"}}})};
  cls->env_snapshot = env;

  auto result = roundtrip(value_t{cls});
  ASSERT_TRUE(std::holds_alternative<closure_ptr>(result.v));
  auto &rc = *std::get<closure_ptr>(result.v);
  ASSERT_EQ(rc.params.size(), 2u);
  EXPECT_EQ(rc.params[0].name, "a");
  EXPECT_EQ(rc.params[1].name, "b");
  EXPECT_FALSE(rc.variadic);
  ASSERT_EQ(rc.body.size(), 1u);
  ASSERT_TRUE(rc.env_snapshot);
  auto *val = rc.env_snapshot->lookup("x");
  ASSERT_TRUE(val);
  EXPECT_EQ(std::get<int64_t>(val->v), 42);
}

TEST_F(ValueCodecTest, ClosureVariadic) {
  auto cls = std::make_shared<closure>();
  cls->params = {symbol{"args"}};
  cls->variadic = true;
  cls->body = {value_t{symbol{"args"}}};
  auto result = roundtrip(value_t{cls});
  ASSERT_TRUE(std::holds_alternative<closure_ptr>(result.v));
  EXPECT_TRUE(std::get<closure_ptr>(result.v)->variadic);
}

TEST_F(ValueCodecTest, NativeFnBecomesNil) {
  native_fn fn = [](std::span<const value_t>) -> value_t { return nil_value; };
  auto result = roundtrip(value_t{fn});
  // Native functions can't be serialized; should decode as nil
  EXPECT_TRUE(result.is_nil());
}

TEST_F(ValueCodecTest, MixedList) {
  auto lst = make_list(
      {value_t{42}, value_t{std::string("hello")}, value_t{true}, nil_value, value_t{3.14}});
  auto result = roundtrip(lst);
  auto &elems = *std::get<list_ptr>(result.v);
  ASSERT_EQ(elems.size(), 5u);
  EXPECT_EQ(std::get<int64_t>(elems[0].v), 42);
  EXPECT_EQ(std::get<std::string>(elems[1].v), "hello");
  EXPECT_TRUE(std::get<bool>(elems[2].v));
  EXPECT_TRUE(elems[3].is_nil());
  EXPECT_DOUBLE_EQ(std::get<double>(elems[4].v), 3.14);
}

// ===========================================================================
// Environment round-trip tests
// ===========================================================================

class EnvironmentCodecTest : public ::testing::Test {
protected:
  cvc::app ctx;
};

TEST_F(EnvironmentCodecTest, NullEnv) {
  auto &node = ctx.root()("__env_test_null__");
  encode_environment(node, nullptr);
  auto result = decode_environment(node);
  EXPECT_EQ(result, nullptr);
}

TEST_F(EnvironmentCodecTest, SingleScope) {
  auto env = std::make_shared<environment>();
  env->set("x", value_t{10});
  env->set("y", value_t{std::string("hello")});

  auto &node = ctx.root()("__env_test_single__");
  encode_environment(node, env);
  auto result = decode_environment(node);

  ASSERT_TRUE(result);
  auto *x = result->lookup("x");
  ASSERT_TRUE(x);
  EXPECT_EQ(std::get<int64_t>(x->v), 10);
  auto *y = result->lookup("y");
  ASSERT_TRUE(y);
  EXPECT_EQ(std::get<std::string>(y->v), "hello");
}

TEST_F(EnvironmentCodecTest, ScopeChain) {
  auto outer = std::make_shared<environment>();
  outer->set("a", value_t{1});
  auto inner = environment::extend(outer);
  inner->set("b", value_t{2});

  auto &node = ctx.root()("__env_test_chain__");
  encode_environment(node, inner);
  auto result = decode_environment(node);

  ASSERT_TRUE(result);
  auto *b = result->lookup("b");
  ASSERT_TRUE(b);
  EXPECT_EQ(std::get<int64_t>(b->v), 2);
  // Should find 'a' in outer scope
  auto *a = result->lookup("a");
  ASSERT_TRUE(a);
  EXPECT_EQ(std::get<int64_t>(a->v), 1);
}

TEST_F(EnvironmentCodecTest, NativeFnBindingsSkipped) {
  auto env = std::make_shared<environment>();
  env->set("x", value_t{42});
  native_fn fn = [](std::span<const value_t>) -> value_t { return nil_value; };
  env->set("my-builtin", value_t{fn});

  auto &node = ctx.root()("__env_test_native__");
  encode_environment(node, env);
  auto result = decode_environment(node);

  ASSERT_TRUE(result);
  auto *x = result->lookup("x");
  ASSERT_TRUE(x);
  EXPECT_EQ(std::get<int64_t>(x->v), 42);
  // Native function binding should be absent
  EXPECT_EQ(result->lookup("my-builtin"), nullptr);
}

// The builtins a decoded scope chain lost (native functions are not written), put back in its
// outermost scope.
static void restore_builtins(const environment_ptr &env) {
  environment_ptr outermost = env;
  while (outermost && outermost->outer)
    outermost = outermost->outer;
  const environment_ptr defaults = builtins::make_default_environment();
  for (auto &[name, val] : defaults->bindings)
    if (!outermost->bindings.count(name))
      outermost->bindings[name] = val;
}

TEST_F(EnvironmentCodecTest, AFunctionItsOwnScopeBinds) {
  // `defun` binds the function in the scope it captures: encoding used to recurse through the
  // two forever. Each scope is now written once and referred to.
  stackless_evaluator ev(builtins::make_default_environment());
  value_t fact =
      ev.evaluate(parse("(begin (defun fact (n) (if (< n 2) 1 (* n (fact (- n 1))))) fact)"));
  auto &node = ctx.root()("__env_test_self__");
  encode_value(node, fact);
  value_t back = decode_value(node);

  ASSERT_TRUE(std::holds_alternative<closure_ptr>(back.v));
  const closure_ptr &f = std::get<closure_ptr>(back.v);
  ASSERT_TRUE(f->env_snapshot);
  value_t *bound = f->env_snapshot->lookup("fact");
  ASSERT_TRUE(bound);
  ASSERT_TRUE(std::holds_alternative<closure_ptr>(bound->v));
  EXPECT_EQ(std::get<closure_ptr>(bound->v)->env_snapshot, f->env_snapshot); // the same scope

  restore_builtins(f->env_snapshot);
  auto global = builtins::make_default_environment();
  stackless_evaluator run(global);
  auto env = environment::extend(global);
  env->set("g", back);
  EXPECT_EQ(std::get<int64_t>(run.evaluate(parse("(g 5)"), env).v), 120);
  f->env_snapshot->bindings.clear(); // break the scope -> closure -> scope cycle
}

TEST_F(EnvironmentCodecTest, ClosuresThatShareAScopeStillShareIt) {
  auto outer = std::make_shared<environment>();
  outer->set("base", value_t{100});
  auto scope = environment::extend(outer);
  scope->set("n", value_t{1});
  auto make = [&](const char *body) {
    auto c = std::make_shared<closure>();
    c->body = {parse(body)};
    c->env_snapshot = scope;
    return value_t{c};
  };
  value_t pair = make_list({make("(+ n base)"), make("(set n (+ n 1))")});

  auto &node = ctx.root()("__env_test_shared__");
  encode_value(node, pair);
  EXPECT_EQ(cvc::state_list(node("__envs__")).size(), 2u); // scope and outer, once each

  value_t back = decode_value(node);
  const auto &l = *std::get<list_ptr>(back.v);
  ASSERT_EQ(l.size(), 2u);
  const environment_ptr &a = std::get<closure_ptr>(l[0].v)->env_snapshot;
  const environment_ptr &b = std::get<closure_ptr>(l[1].v)->env_snapshot;
  ASSERT_TRUE(a);
  EXPECT_EQ(a, b);
  ASSERT_TRUE(a->outer);
  EXPECT_EQ(std::get<int64_t>(a->lookup("base")->v), 100);
}

TEST_F(EnvironmentCodecTest, OlderSnapshotsStillDecode) {
  // The layout written before scopes got their own table: the chain inside the closure (an
  // env_chain, innermost scope first), and each body expression a walk of its own, with its
  // own __spill__ table.
  auto &node = ctx.root()("__env_test_older__");
  auto set_type = [](cvc::state &n, const char *t) { n("__type__").value(std::string(t)); };
  set_type(node, "closure");
  cvc::state_list(node("__params__")).push_back().value(std::string("x"));
  node("__variadic__").value(std::string("0"));
  cvc::state_list body(node("__body__"));
  cvc::state &expr0 = body.push_back();
  set_type(expr0, "symbol");
  expr0.value(std::string("x"));
  cvc::state &expr1 = body.push_back(); // spilled into its own table
  set_type(expr1, "spill_ref");
  expr1.value(std::string("0"));
  cvc::state &spilled = cvc::state_list(expr1("__spill__")).push_back();
  set_type(spilled, "list");
  (void)cvc::state_list(spilled("__elems__"));
  cvc::state &env = node("__env__");
  set_type(env, "env_chain");
  cvc::state_list scopes(env("__scopes__"));
  for (auto [key, val] : {std::pair<const char *, int>{"y", 5}, {"z", 6}}) {
    cvc::state &entry = cvc::state_list(scopes.push_back()("__bindings__")).push_back();
    entry("__key__").value(std::string(key));
    set_type(entry("__val__"), "int");
    entry("__val__").value(std::to_string(val));
  }

  value_t back = decode_value(node);
  ASSERT_TRUE(std::holds_alternative<closure_ptr>(back.v));
  const closure &c = *std::get<closure_ptr>(back.v);
  ASSERT_EQ(c.params.size(), 1u);
  EXPECT_EQ(c.params[0].name, "x");
  ASSERT_EQ(c.body.size(), 2u);
  EXPECT_EQ(std::get<symbol>(c.body[0].v).name, "x");
  EXPECT_TRUE(std::get<list_ptr>(c.body[1].v)->empty());
  ASSERT_TRUE(c.env_snapshot);
  EXPECT_EQ(std::get<int64_t>(c.env_snapshot->lookup("y")->v), 5);
  EXPECT_EQ(std::get<int64_t>(c.env_snapshot->lookup("z")->v), 6); // from the outer scope
}

TEST_F(EnvironmentCodecTest, BadScopeReferencesThrow) {
  auto outer = std::make_shared<environment>();
  outer->set("a", value_t{1});
  auto scope = environment::extend(outer);
  auto c = std::make_shared<closure>();
  c->env_snapshot = scope;

  auto &node = ctx.root()("__env_test_bad__");
  encode_value(node, value_t{c});
  cvc::state &ref = node("__env__");
  ASSERT_EQ(ref("__type__").value(), "env_ref");
  ref.value(std::string("7")); // past the table
  EXPECT_THROW(decode_value(node), std::runtime_error);
  ref.value(std::string("0"));
  EXPECT_NO_THROW(decode_value(node));

  // Entry 1 (the outer scope) made to enclose entry 0 again: lookups would loop forever.
  cvc::state &outer_ref = cvc::state_list(node("__envs__")).at(1)("__outer__");
  outer_ref("__type__").value(std::string("env_ref"));
  outer_ref.value(std::string("0"));
  EXPECT_THROW(decode_value(node), std::runtime_error);
}

// ===========================================================================
// Evaluator-state round-trip tests
// ===========================================================================

class EvaluatorStateCodecTest : public ::testing::Test {
protected:
  cvc::app ctx;
  std::unique_ptr<stackless_evaluator> ev;

  void SetUp() override {
    ev = std::make_unique<stackless_evaluator>(builtins::make_default_environment());
  }
};

TEST_F(EvaluatorStateCodecTest, CompletedState) {
  auto state = ev->create_state(std::string("(+ 1 2)"));
  ev->run(state);
  ASSERT_TRUE(state.done);
  EXPECT_EQ(std::get<int64_t>(state.result.v), 3);

  auto &node = ctx.root()("__es_completed__");
  encode_evaluator_state(node, state);
  auto restored = decode_evaluator_state(node);

  EXPECT_TRUE(restored.done);
  EXPECT_EQ(std::get<int64_t>(restored.result.v), 3);
  EXPECT_TRUE(restored.stack.empty());
}

TEST_F(EvaluatorStateCodecTest, StateStillReadingItsScript) {
  // parser.step_tokens: a process snapshotted mid-read keeps its source and settings, and the
  // restored state reads the script again and evaluates it.
  parse_options popts;
  popts.step_tokens = 2;
  popts.max_depth = 64;
  auto state = ev->create_state(std::string("(set x 20) (+ x 22)"), nullptr, popts);
  ev->step(state); // part way through reading
  ASSERT_TRUE(state.reading);

  auto &node = ctx.root()("__es_reading__");
  encode_evaluator_state(node, state);
  auto restored = decode_evaluator_state(node);

  ASSERT_TRUE(restored.reading);
  EXPECT_EQ(restored.reading->source(), "(set x 20) (+ x 22)");
  EXPECT_EQ(restored.reading->options().step_tokens, 2u);
  EXPECT_EQ(restored.reading->options().max_depth, 64u);

  // Native builtins aren't serialized; put them back on the restored environment's root, as
  // PauseResumeProducesCorrectResult does. Nothing is on the frame stack yet.
  ASSERT_TRUE(restored.global_env);
  environment_ptr root_env = restored.global_env;
  while (root_env->outer)
    root_env = root_env->outer;
  const auto builtins_env = builtins::make_default_environment(); // must outlive the loop
  for (auto &[name, val] : builtins_env->bindings)
    root_env->set(name, val);
  EXPECT_EQ(std::get<int64_t>(ev->run(restored).v), 42);
}

TEST_F(EvaluatorStateCodecTest, PausedState) {
  // Run a program partially (10 steps of a loop)
  auto state =
      ev->create_state(std::string("(begin (set sum 0) (set i 0) "
                                   "(while (< i 100) (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                                   "sum)"));
  ev->run(state, 10); // 10 steps only
  ASSERT_FALSE(state.done);

  auto &node = ctx.root()("__es_paused__");
  encode_evaluator_state(node, state);
  auto restored = decode_evaluator_state(node);

  EXPECT_FALSE(restored.done);
  EXPECT_FALSE(restored.stack.empty());

  // The restored state should have a frame stack
  EXPECT_EQ(restored.stack.size(), state.stack.size());
}

TEST_F(EvaluatorStateCodecTest, PauseResumeProducesCorrectResult) {
  // Pause/resume test using deep nested arithmetic that doesn't depend
  // on shared environment mutation across frames.
  // NOTE: A more sophisticated codec that deduplicates environment_ptr
  //       identity (not just value) is needed to support pausing loops
  //       with mutable variables.  This is tracked as a future Phase 3 item.
  auto state = ev->create_state(std::string("(+ 1 2 3 4 5 6 7 8 9 10)"));

  // Run 3 steps — enough to start arg evaluation but not finish
  ev->run(state, 3);
  if (state.done) {
    EXPECT_EQ(std::get<int64_t>(state.result.v), 55);
    return;
  }

  // Serialize
  auto &node = ctx.root()("__es_resume__");
  encode_evaluator_state(node, state);
  auto restored = decode_evaluator_state(node);

  // Re-register builtins in ALL environment chains
  auto builtins_env = builtins::make_default_environment();
  auto patch_env = [&](environment_ptr env) {
    if (!env)
      return;
    while (env->outer)
      env = env->outer;
    for (auto &[name, val] : builtins_env->bindings)
      env->set(name, val);
  };
  patch_env(restored.global_env);
  for (auto &frame : restored.stack) {
    patch_env(frame.env);
    patch_env(frame.extra_env);
  }

  // Resume with a new evaluator
  auto ev2 = std::make_unique<stackless_evaluator>(builtins::make_default_environment());
  ev2->run(restored);
  ASSERT_TRUE(restored.done);
  EXPECT_EQ(std::get<int64_t>(restored.result.v), 55);
}

TEST_F(EvaluatorStateCodecTest, WithMacros) {
  auto state = ev->create_state(std::string("(begin "
                                            "  (defmacro double-it (x) (* x 2)) "
                                            "  (double-it 21))"));
  ev->run(state);
  ASSERT_TRUE(state.done);
  EXPECT_EQ(std::get<int64_t>(state.result.v), 42);

  auto &node = ctx.root()("__es_macros__");
  encode_evaluator_state(node, state);
  auto restored = decode_evaluator_state(node);

  EXPECT_TRUE(restored.done);
  EXPECT_EQ(std::get<int64_t>(restored.result.v), 42);
  EXPECT_EQ(restored.user_macros.size(), 1u);
  EXPECT_TRUE(restored.user_macros.count("double-it"));
}

// ===========================================================================
// Cross-evaluator consistency tests
// ===========================================================================

class CrossEvaluatorTest : public ::testing::Test {
protected:
  std::unique_ptr<evaluator> sync_eval;
  std::unique_ptr<stackless_evaluator> stackless_eval;

  void SetUp() override {
    sync_eval = std::make_unique<evaluator>(builtins::make_default_environment());
    stackless_eval = std::make_unique<stackless_evaluator>(builtins::make_default_environment());
  }

  // Run the same program on both evaluators and compare results
  void expect_consistent(const std::string &program) {
    value_t sync_result = sync_eval->evaluate_script(program);
    value_t stackless_result = stackless_eval->evaluate_script(program);
    EXPECT_TRUE(values_equal(sync_result, stackless_result))
        << "Mismatch for: " << program << "\n  sync:      " << to_string(sync_result)
        << "\n  stackless: " << to_string(stackless_result);
  }

  int64_t sync_int(const std::string &s) {
    return std::get<int64_t>(sync_eval->evaluate_script(s).v);
  }

  int64_t stackless_int(const std::string &s) {
    return std::get<int64_t>(stackless_eval->evaluate_script(s).v);
  }
};

// --- Atoms ---

TEST_F(CrossEvaluatorTest, IntegerLiteral) { expect_consistent("42"); }

TEST_F(CrossEvaluatorTest, DoubleLiteral) { expect_consistent("3.14"); }

TEST_F(CrossEvaluatorTest, StringLiteral) { expect_consistent("\"hello\""); }

TEST_F(CrossEvaluatorTest, NilLiteral) { expect_consistent("nil"); }

TEST_F(CrossEvaluatorTest, TrueLiteral) { expect_consistent("t"); }

// --- Arithmetic ---

TEST_F(CrossEvaluatorTest, Addition) { expect_consistent("(+ 1 2 3)"); }

TEST_F(CrossEvaluatorTest, Subtraction) { expect_consistent("(- 10 3)"); }

TEST_F(CrossEvaluatorTest, Multiplication) { expect_consistent("(* 4 5)"); }

TEST_F(CrossEvaluatorTest, Division) { expect_consistent("(/ 10 3)"); }

TEST_F(CrossEvaluatorTest, NestedArithmetic) { expect_consistent("(+ (* 2 3) (- 10 4))"); }

// --- Comparison ---

TEST_F(CrossEvaluatorTest, LessThan) {
  expect_consistent("(< 1 2)");
  expect_consistent("(< 2 1)");
}

TEST_F(CrossEvaluatorTest, Equality) {
  expect_consistent("(= 5 5)");
  expect_consistent("(= 5 6)");
}

// --- Special forms ---

TEST_F(CrossEvaluatorTest, IfTrue) { expect_consistent("(if t 1 2)"); }

TEST_F(CrossEvaluatorTest, IfFalse) { expect_consistent("(if nil 1 2)"); }

TEST_F(CrossEvaluatorTest, Begin) { expect_consistent("(begin 1 2 3)"); }

TEST_F(CrossEvaluatorTest, SetAndRead) { expect_consistent("(begin (set x 42) x)"); }

TEST_F(CrossEvaluatorTest, Quote) { expect_consistent("(quote (1 2 3))"); }

TEST_F(CrossEvaluatorTest, While) {
  expect_consistent("(begin (set i 0) (set sum 0) "
                    "  (while (< i 5) (begin (set sum (+ sum i)) (set i (+ i 1)))) "
                    "  sum)");
}

TEST_F(CrossEvaluatorTest, For) {
  expect_consistent("(begin (set sum 0) "
                    "  (for x (list 1 2 3 4 5) (set sum (+ sum x))) "
                    "  sum)");
}

// --- Lambda / closures ---

TEST_F(CrossEvaluatorTest, LambdaBasic) {
  // Direct application ((lambda ...) x) is not supported; bind first
  expect_consistent("(begin (set sq (lambda (x) (* x x))) (sq 5))");
}

TEST_F(CrossEvaluatorTest, LambdaClosure) {
  expect_consistent("(begin "
                    "  (set make-adder (lambda (n) (lambda (x) (+ x n)))) "
                    "  (set add5 (make-adder 5)) "
                    "  (add5 10))");
}

// --- Defun ---

TEST_F(CrossEvaluatorTest, DefunRecursive) {
  expect_consistent("(begin "
                    "  (defun fact (n) "
                    "    (if (< n 2) 1 (* n (fact (- n 1))))) "
                    "  (fact 10))");
}

// --- Let ---

TEST_F(CrossEvaluatorTest, LetBasic) { expect_consistent("(let ((a 10) (b 20)) (+ a b))"); }

TEST_F(CrossEvaluatorTest, LetNested) {
  expect_consistent("(let ((x 10)) "
                    "  (let ((y 20)) (+ x y)))");
}

// --- Return ---

TEST_F(CrossEvaluatorTest, Return) {
  expect_consistent("(begin (set f (lambda () (begin (return 42) 99))) (f))");
}

// --- Macros ---

TEST_F(CrossEvaluatorTest, Defmacro) {
  expect_consistent("(begin "
                    "  (defmacro double-it (x) (* x 2)) "
                    "  (double-it 21))");
}

// --- Eval ---

TEST_F(CrossEvaluatorTest, Eval) { expect_consistent("(eval (quote (+ 1 2)))"); }

// --- String operations ---

TEST_F(CrossEvaluatorTest, StrConcat) {
  expect_consistent("(str-concat \"hello\" \" \" \"world\")");
}

// --- List operations ---

TEST_F(CrossEvaluatorTest, CarCdrCons) {
  expect_consistent("(car (list 1 2 3))");
  expect_consistent("(cdr (list 1 2 3))");
}

TEST_F(CrossEvaluatorTest, ListLength) { expect_consistent("(length (list 1 2 3 4))"); }

// --- Dict ---

TEST_F(CrossEvaluatorTest, DictOps) {
  expect_consistent("(begin "
                    "  (set d (dict)) "
                    "  (set-attr d \"key\" 42) "
                    "  (get-attr d \"key\"))");
}

// --- Fibonacci (complex) ---

TEST_F(CrossEvaluatorTest, Fibonacci) {
  EXPECT_EQ(sync_int("(begin "
                     "  (defun fib (n) "
                     "    (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
                     "  (fib 10))"),
            55);
  EXPECT_EQ(stackless_int("(begin "
                          "  (defun fib (n) "
                          "    (if (< n 2) n (+ (fib (- n 1)) (fib (- n 2))))) "
                          "  (fib 10))"),
            55);
}

// ===========================================================================
// with-lock in snapshots (docs/STATE_EXEC_LOCKS.md 5.8; locks plan L1.6, L1.7, L1.10): the lock
// phases, the lock_ops marker, the holder refusal, rebind_natives, the wait's handle and the
// remaining timeout.
// ===========================================================================

namespace {

// A sync scheduler with one intrinsics context per program (chroot optional).
struct lock_codec_world {
  cvc::app app;
  cvc::state &tree;
  scheduler sched;
  std::deque<intrinsics_context> contexts;

  lock_codec_world() : tree(cvc::state::instance(app)) { sched.set_watch_root(&tree); }

  environment_ptr natives(const std::string &chroot = "") {
    intrinsics_context &ctx = contexts.emplace_back();
    ctx.sched = &sched;
    ctx.root = &tree;
    if (!chroot.empty())
      apply_chroot(ctx, tree, chroot);
    ctx.proc = make_process();
    environment_ptr env = builtins::make_default_environment();
    register_intrinsics(env, &ctx);
    return env;
  }
  int submit(const std::string &script, environment_ptr env = nullptr) {
    execute_options opts;
    opts.env = env ? env : natives();
    return sched.execute(script, opts);
  }
  template <class Pred> bool step_until(Pred done, double seconds = 10.0) {
    const auto until = std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(static_cast<int>(seconds * 1000));
    while (!done()) {
      if (std::chrono::steady_clock::now() >= until)
        return false;
      if (!sched.step())
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
  }
  process_status status(int pid) { return sched.get_process_info(pid)->status; }
  bool finished(int pid) { return is_finished(status(pid)); }
  std::string waiting_on(int pid) {
    const auto i = sched.get_process_info(pid);
    return i && i->waiting_on ? *i->waiting_on : std::string();
  }
  bool lock_waiting(int pid) {
    return status(pid) == process_status::waiting && waiting_on(pid).rfind("lock ", 0) == 0;
  }
  bool inside(int pid) {
    return status(pid) == process_status::waiting && sched.get_process_info(pid)->locks_held > 0 &&
           waiting_on(pid).rfind("msg ", 0) == 0;
  }
  void send(const std::string &channel) {
    sched.deliver_to_receivers(channel, value_t(std::string("go")));
  }
  std::vector<std::string> held_keys() {
    std::vector<std::string> out;
    for (const lock_info &i : sched.locks()->manager().query("", nullptr, "").held)
      out.push_back(i.key);
    return out;
  }
  // Every node under `node` whose data_object type is `type`.
  std::vector<cvc::state *> data_objects(cvc::state &node, const std::string &type) {
    static const std::string tag = ".__data_type__";
    std::vector<cvc::state *> out;
    for (const std::string &full : node.children())
      if (full.size() > tag.size() &&
          full.compare(full.size() - tag.size(), tag.size(), tag) == 0 &&
          tree(full).value() == type)
        out.push_back(&tree(full.substr(0, full.size() - tag.size())));
    return out;
  }
};

eval_frame lock_frame(eval_phase phase) {
  eval_frame f;
  f.phase = phase;
  f.expr = parse("(with-lock \"k\" 1)");
  f.extra_vals.assign(7, nil_value);
  f.extra_vals.push_back(value_t(int64_t(1))); // the body
  return f;
}

} // namespace

TEST(LockCodec, PhasesRoundTrip) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("snap");
  evaluator_state st;
  st.root_expr = parse("(with-lock \"k\" 1)");
  st.global_env = builtins::make_default_environment();
  st.stack.push_back(lock_frame(eval_phase::lock_enter));
  st.stack.push_back(lock_frame(eval_phase::lock_body));
  for (eval_frame &f : st.stack)
    f.env = st.global_env;
  encode_evaluator_state(node, st);
  const evaluator_state back = decode_evaluator_state(node);
  ASSERT_EQ(back.stack.size(), 2u);
  EXPECT_EQ(back.stack[0].phase, eval_phase::lock_enter);
  EXPECT_EQ(back.stack[1].phase, eval_phase::lock_body);
  EXPECT_EQ(back.stack[1].extra_vals.size(), 8u);
}

TEST(LockCodec, LockOpsDecodesNil) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("snap");
  intrinsics_context ctx;
  const value_t ops = detail::make_lock_ops(&ctx);
  ASSERT_NE(detail::lock_ops_of(ops), nullptr);
  encode_value(node, ops);
  EXPECT_EQ(node("__data_type__").value(), "lock_ops");
  EXPECT_TRUE(node.data().empty()); // no payload: the context pointer isn't written
  EXPECT_TRUE(decode_value(node).is_nil());
  // Inside a list too.
  cvc::state &node2 = cvc::state::instance(app)("snap2");
  encode_value(node2, make_list({value_t(int64_t(1)), ops}));
  const value_t back = decode_value(node2);
  const auto &l = *std::get<list_ptr>(back.v);
  ASSERT_EQ(l.size(), 2u);
  EXPECT_TRUE(l[1].is_nil());
}

// A process with the intrinsics, snapshotted where it never locked, and restored with another
// context's natives: no node holds a lock_ops payload, and the restored with-lock is the
// restoring context's (its chroot names the key).
TEST(LockCodec, NoLockOpsPayload) {
  lock_codec_world w;
  const int pid = w.submit(R"((begin (msg-recv "go") (with-lock "k" (msg-recv "go2"))))");
  ASSERT_TRUE(w.step_until([&] { return w.status(pid) == process_status::waiting; }));
  // (The search finds an encoded lock_ops.)
  encode_value(w.tree("probe"), make_list({detail::make_lock_ops(&w.contexts.front())}));
  ASSERT_EQ(w.data_objects(w.tree("probe"), "lock_ops").size(), 1u);
  cvc::state &node = w.tree("snap");
  ASSERT_TRUE(w.sched.snapshot(pid, node));
  for (cvc::state *n : w.data_objects(node, "lock_ops"))
    EXPECT_TRUE(n->data().empty()) << n->fullName();
  const int r = w.sched.restore(node, w.natives("r2"));
  w.sched.kill(pid);
  w.send(resolve_channel_key("r2", "go"));
  ASSERT_TRUE(w.step_until([&] { return w.inside(r); }));
  EXPECT_EQ(w.held_keys(), (std::vector<std::string>{"r2.k"}));
  w.send(resolve_channel_key("r2", "go2"));
  ASSERT_TRUE(w.step_until([&] { return w.finished(r); }));
  EXPECT_TRUE(w.held_keys().empty());
}

TEST(LockCodec, HolderRefused) {
  lock_codec_world w;
  const int pid = w.submit(R"((with-lock (list "m.n" "k") (msg-recv "go")))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(pid); }));
  cvc::state &node = w.tree("snap");
  try {
    w.sched.snapshot(pid, node);
    ADD_FAILURE() << "a holder was snapshotted";
  } catch (const std::runtime_error &e) {
    EXPECT_EQ(std::string(e.what()),
              "snapshot: process " + std::to_string(pid) + " holds locks: /k, /m.n");
  }
  EXPECT_EQ(node.numChildren(), 0u); // nothing written for the process
  EXPECT_EQ(node.value(), "");
  w.send("go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(pid); }));
}

TEST(LockCodec, RebindReplacesStaleLockOps) {
  lock_codec_world w;
  intrinsics_context stale;
  const environment_ptr natives = w.natives();
  const detail::lock_ops *fresh = detail::lock_ops_of(*natives->lookup("with-lock"));
  ASSERT_NE(fresh, nullptr);
  stackless_evaluator ev(builtins::make_default_environment());
  const value_t closure = ev.evaluate_script("(lambda (k b) 0)");
  struct row {
    const char *name;
    std::optional<value_t> bound; // nullopt: unbound
    bool replaced;
  };
  for (const row &r : {row{"unbound", std::nullopt, true}, row{"nil", nil_value, true},
                       row{"stale", detail::make_lock_ops(&stale), true},
                       row{"a program's own", closure, false}}) {
    SCOPED_TRACE(r.name);
    evaluator_state st;
    st.global_env = std::make_shared<environment>();
    if (r.bound)
      st.global_env->set("with-lock", *r.bound);
    rebind_natives(st, natives);
    const value_t *now = st.global_env->lookup("with-lock");
    ASSERT_NE(now, nullptr);
    const detail::lock_ops *ops = detail::lock_ops_of(*now);
    if (r.replaced) {
      ASSERT_NE(ops, nullptr);
      EXPECT_EQ(ops->ctx, fresh->ctx);
    } else {
      EXPECT_EQ(ops, nullptr);
      EXPECT_TRUE(std::holds_alternative<closure_ptr>(now->v));
    }
  }
}

// A parked waiter's snapshot writes its wait's handle as nil, in the await frame and in the
// lock_enter frame's slot, with no payload anywhere; restored (L1.10), the copy queues afresh, and
// the original is still queued and granted on release.
TEST(LockCodec, LockWaitHandleEncodesNil) {
  lock_codec_world w;
  const int h = w.submit(R"((with-lock "k" (msg-recv "go")))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(h); }));
  const environment_ptr env = w.natives();
  const int q = w.submit(R"((with-lock "k" "q"))", env);
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(q); }));
  cvc::state &node = w.tree("snap");
  ASSERT_TRUE(w.sched.snapshot(q, node));
  EXPECT_TRUE(w.data_objects(node, "lock_wait").empty());
  const process_ptr pp = decode_process(node);
  const auto &st = pp->state.stack;
  ASSERT_GE(st.size(), 2u);
  ASSERT_EQ(st.back().phase, eval_phase::await_call);
  EXPECT_TRUE(st.back().extra_vals.at(0).is_nil()); // the handle
  ASSERT_EQ(st[st.size() - 2].phase, eval_phase::lock_enter);
  EXPECT_TRUE(st[st.size() - 2].extra_vals.at(3).is_nil()); // wait_or_outcome
  const int r = w.sched.restore(node, env);
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(r); })); // queued afresh
  EXPECT_TRUE(w.lock_waiting(q));
  EXPECT_EQ(w.sched.locks()->manager().stats().queued, 2u);
  w.send("go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(q) && w.finished(r); }));
  EXPECT_EQ(to_string(*w.sched.get_result(q)), R"("q")");
  EXPECT_EQ(to_string(*w.sched.get_result(r)), R"("q")");
  EXPECT_TRUE(w.sched.locks()->manager().table().empty());
}

TEST(LockCodec, RemainingTimeoutWritten) {
  lock_codec_world w;
  const int h = w.submit(R"((with-lock "k" (msg-recv "go")))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(h); }));
  const int t30 = w.submit(R"((with-lock "k" :timeout 30 1))");
  const int forever = w.submit(R"((with-lock "k" 1))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(t30) && w.lock_waiting(forever); }));
  const auto timeout_written = [&](int pid) {
    cvc::state &node = w.tree("snap." + std::to_string(pid));
    EXPECT_TRUE(w.sched.snapshot(pid, node));
    const process_ptr pp = decode_process(node);
    const eval_frame &wait = pp->state.stack.back();
    EXPECT_EQ(wait.phase, eval_phase::await_call);
    EXPECT_EQ(wait.extra_lists.size(), 1u);
    return wait.extra_lists.at(0).at(2);
  };
  const value_t left = timeout_written(t30);
  ASSERT_TRUE(std::holds_alternative<double>(left.v)) << to_string(left);
  EXPECT_GT(std::get<double>(left.v), 0.0);
  EXPECT_LE(std::get<double>(left.v), 30.0);
  EXPECT_TRUE(timeout_written(forever).is_nil());
  w.send("go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(t30) && w.finished(forever); }));
}

// ===========================================================================
// The lock phases' fields (docs/STATE_EXEC_LOCKS.md)
// ===========================================================================

// The priority floor (7.2; locks plan L2.6) and whether a host gave it: written beside the
// priority and read back, through a scheduler's snapshot and restore too. A snapshot without them
// (older than the floor, or of a process no scheduler submitted) reads the decoded priority as its
// floor, not an explicit one.
TEST(LockCodec, FloorRoundTrips) {
  cvc::app ctx;
  scheduler sched;
  execute_options explicit_floor;
  explicit_floor.priority = 4;
  explicit_floor.priority_floor = 2;
  const int pid = sched.execute(std::string("(+ 1 2)"), explicit_floor);
  execute_options submit_floor;
  submit_floor.priority = 5;
  const int pid2 = sched.execute(std::string("(+ 1 2)"), submit_floor);

  auto &node = ctx.root()("__floor_explicit__");
  ASSERT_TRUE(sched.snapshot(pid, node));
  EXPECT_EQ(node("priority").value(), "4");
  EXPECT_EQ(node("priority_floor").value(), "2");
  EXPECT_EQ(node("priority_floor_explicit").value(), "1");
  const process_ptr decoded = decode_process(node);
  EXPECT_EQ(decoded->priority, 4);
  EXPECT_EQ(decoded->priority_floor, std::optional<int>(2));
  EXPECT_TRUE(decoded->priority_floor_explicit);
  const int restored = sched.restore(node);
  EXPECT_EQ(sched.get_priority(restored)->floor, 2);

  auto &node2 = ctx.root()("__floor_submit__");
  ASSERT_TRUE(sched.snapshot(pid2, node2));
  EXPECT_EQ(node2("priority_floor").value(), "5");
  EXPECT_EQ(node2("priority_floor_explicit").value(), "0");
  const process_ptr decoded2 = decode_process(node2);
  EXPECT_EQ(decoded2->priority_floor, std::optional<int>(5));
  EXPECT_FALSE(decoded2->priority_floor_explicit);

  // No floor: a process no scheduler submitted writes none, and reads its priority back.
  process_ptr bare = make_process();
  bare->priority = 7;
  auto &node3 = ctx.root()("__floor_none__");
  encode_process(node3, *bare);
  EXPECT_EQ(node3.findDescendant("priority_floor"), nullptr);
  const process_ptr decoded3 = decode_process(node3);
  EXPECT_EQ(decoded3->priority, 7);
  EXPECT_EQ(decoded3->priority_floor, std::optional<int>(7));
  EXPECT_FALSE(decoded3->priority_floor_explicit);
}
