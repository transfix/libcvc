#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/state/state_list.h>
#include <gtest/gtest.h>

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
