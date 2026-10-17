/// @file state_exec_stdlib_test.cpp
/// @brief The stdlib modules (string, bytes, math, collections) reached from DSL programs, the
/// shared frozen scope they live in, and the per-call allocation limit.

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stdlib.h>
#include <gtest/gtest.h>
#include <stdexcept>
#include <string>
#include <vector>

using namespace cvc::state_exec;

namespace {

// A result as text: a string bare, anything else as to_string() prints it.
std::string show(const std::optional<value_t> &r) {
  if (!r)
    return "<no result>";
  if (auto *s = std::get_if<std::string>(&r->v))
    return *s;
  return to_string(*r);
}

// Run a script to completion on the sync scheduler with its default environment. A runtime
// error propagates out of run().
std::string run_sync(const std::string &script, execute_options opts = {},
                     cvc::state *root = nullptr, const std::string &id = {}) {
  scheduler sched;
  if (root) {
    sched.set_watch_root(root);
    sched.set_id(id);
    sched.load_settings();
  }
  const int pid = sched.execute(script, opts);
  sched.run();
  return show(sched.get_result(pid));
}

// Same on the async scheduler (the app's exec scheduler type). A runtime error kills the
// process; it comes back through `error`.
std::string run_async(const std::string &script, std::string *error, execute_options opts = {},
                      cvc::state *root = nullptr) {
  async_scheduler sched;
  if (root)
    sched.set_watch_root(root);
  const int pid = sched.execute(script, opts);
  sched.sync_run();
  *error = sched.get_exit_error(pid).value_or("");
  return show(sched.get_result(pid));
}

// The message of the runtime_error `f` throws, or "" when it doesn't throw.
template <class F> std::string thrown(F &&f) {
  try {
    f();
  } catch (const std::runtime_error &e) {
    return e.what();
  }
  return "";
}

// A list just over the 16 MiB default, as a collections.range length.
std::string over_default_elements() {
  return std::to_string(kDefaultMaxAllocBytes / sizeof(value_t) + 1);
}

value_t call(const char *fn, std::vector<value_t> args) {
  const native_fn *f = stdlib_registry::standard().lookup_qualified(fn);
  if (!f)
    throw std::logic_error(std::string("no stdlib function ") + fn);
  return (*f)(std::span<const value_t>(args.data(), args.size()));
}

} // namespace

// --- reachable from programs ---------------------------------------------------------------

TEST(StdlibReachTest, SyncSchedulerCallsStringLength) {
  EXPECT_EQ(run_sync("(string.length \"x\")"), "1");
  EXPECT_EQ(run_sync("(string.length \"caf\xC3\xA9\")"), "4"); // codepoints, not bytes
}

TEST(StdlibReachTest, AsyncSchedulerCallsStringLength) {
  std::string err;
  EXPECT_EQ(run_async("(string.length \"caf\xC3\xA9\")", &err), "4");
  EXPECT_EQ(err, "");
}

TEST(StdlibReachTest, EveryModuleIsReachable) {
  EXPECT_EQ(run_sync("(string.upper \"go\")"), "GO");
  EXPECT_EQ(run_sync("(bytes.length (string.encode \"caf\xC3\xA9\"))"), "5");
  EXPECT_EQ(run_sync("(bytes.decode (string.encode \"ok\"))"), "ok");
  EXPECT_EQ(run_sync("(math.floor 3.7)"), "3");
  EXPECT_EQ(run_sync("(collections.reverse (list 1 2 3))"), "(3 2 1)");
  // map/filter/reduce take a native function (a builtin or a stdlib function).
  EXPECT_EQ(run_sync("(collections.reduce (list 1 2 3) + 0)"), "6");
  EXPECT_EQ(run_sync("(collections.map (list -1 2 -3) math.abs)"), "(1 2 3)");
}

TEST(StdlibReachTest, BuiltinsKeepTheirUnqualifiedNames) {
  // The stdlib names are all module-qualified, so the builtins of the same short name stay.
  EXPECT_EQ(run_sync("(length (list 1 2 3))"), "3");
  EXPECT_EQ(run_sync("(collect (range 3))"), "(0 1 2)");
}

TEST(StdlibReachTest, ExplicitEnvironmentFromMakeDefaultEnvironment) {
  // Hosts that build their own environment (Ariadne, pycvc) start from
  // make_default_environment(), so they reach the stdlib the same way.
  scheduler sched;
  execute_options opts;
  opts.env = builtins::make_default_environment();
  const int pid = sched.execute(std::string("(string.substring \"caf\xC3\xA9s\" 3 1)"), opts);
  sched.run();
  EXPECT_EQ(show(sched.get_result(pid)), "\xC3\xA9");
}

// --- one shared, frozen scope ---------------------------------------------------------------

TEST(StdlibSharedScopeTest, DefaultEnvironmentsLinkOneScope) {
  const environment_ptr scope = stdlib_registry::shared_scope();
  ASSERT_TRUE(scope);
  EXPECT_TRUE(scope->frozen);
  EXPECT_EQ(stdlib_registry::shared_scope(), scope); // built once

  auto a = builtins::make_default_environment();
  auto b = builtins::make_default_environment();
  EXPECT_EQ(a->outer, scope);
  EXPECT_EQ(b->outer, scope);
  // Not copied: neither environment binds the name itself, both find the one shared value.
  EXPECT_EQ(a->bindings.count("string.length"), 0u);
  ASSERT_NE(a->lookup("string.length"), nullptr);
  EXPECT_EQ(a->lookup("string.length"), b->lookup("string.length"));

  // Every function of every module is in it.
  const stdlib_registry &reg = stdlib_registry::standard();
  for (const std::string &module : {"string", "bytes", "math", "collections"})
    for (const std::string &fn : reg.list_functions(module))
      EXPECT_NE(scope->lookup(fn), nullptr) << fn;
}

TEST(StdlibSharedScopeTest, SetShadowsInsteadOfWritingTheSharedScope) {
  EXPECT_EQ(run_sync("(begin (set string.length 42) string.length)"), "42");
  // Another program (another scheduler) still sees the real function.
  EXPECT_EQ(run_sync("(string.length \"ab\")"), "2");
  ASSERT_NE(stdlib_registry::shared_scope()->lookup("string.length"), nullptr);
  EXPECT_TRUE(std::holds_alternative<native_fn>(
      stdlib_registry::shared_scope()->lookup("string.length")->v));
}

TEST(StdlibSharedScopeTest, DefunShadowsAStdlibName) {
  EXPECT_EQ(run_sync("(begin (defun string.upper (s) \"mine\") (string.upper \"x\"))"), "mine");
  EXPECT_EQ(run_sync("(string.upper \"x\")"), "X");
}

TEST(StdlibSharedScopeTest, SetExistingShadowsInOutermostUnfrozenScope) {
  auto env = builtins::make_default_environment();
  auto inner = environment::extend(env);
  inner->set_existing("math.pi", value_t{int64_t{3}});
  EXPECT_EQ(inner->bindings.count("math.pi"), 0u); // not in the scope it was called on
  ASSERT_EQ(env->bindings.count("math.pi"), 1u);   // shadowed one level below the shared scope
  EXPECT_EQ(std::get<int64_t>(env->bindings["math.pi"].v), 3);
  EXPECT_TRUE(
      std::holds_alternative<native_fn>(stdlib_registry::shared_scope()->lookup("math.pi")->v));
  // A name bound in an unfrozen scope is still updated where it is.
  inner->set("x", value_t{int64_t{1}});
  inner->set_existing("x", value_t{int64_t{2}});
  EXPECT_EQ(std::get<int64_t>(inner->bindings["x"].v), 2);
}

TEST(StdlibSharedScopeTest, WritingAFrozenScopeThrows) {
  EXPECT_THROW(stdlib_registry::shared_scope()->set_existing("math.pi", value_t{}),
               std::logic_error);
  EXPECT_THROW(stdlib_registry::shared_scope()->set("math.pi", value_t{}), std::logic_error);
  EXPECT_THROW(stdlib_registry::standard().import_module("math", stdlib_registry::shared_scope()),
               std::logic_error);
}

// --- per-call allocation limit --------------------------------------------------------------

TEST(StdlibAllocLimitTest, DefaultLimitStopsAHugeRange) {
  const std::string err = thrown([] { run_sync("(collections.range 1000000000)"); });
  EXPECT_NE(err.find("collections.range"), std::string::npos) << err;
  EXPECT_NE(err.find("allocation limit"), std::string::npos) << err;

  std::string async_err;
  run_async("(collections.range 1000000000)", &async_err);
  EXPECT_NE(async_err.find("collections.range"), std::string::npos) << async_err;

  // Just over the default fails, a normal size works.
  EXPECT_NE(thrown([] { run_sync("(collections.range " + over_default_elements() + ")"); }), "");
  EXPECT_EQ(run_sync("(collections.range 5)"), "(0 1 2 3 4)");
}

TEST(StdlibAllocLimitTest, CheckedAgainstWhatIsLeftOfMaxMemory) {
  execute_options opts;
  opts.max_memory = 64 * 1024; // the whole budget is under the size of 10000 elements
  const std::string err = thrown([&] { run_sync("(collections.range 10000)", opts); });
  EXPECT_NE(err.find("allocation limit"), std::string::npos) << err;
  std::string async_err;
  run_async("(collections.range 10000)", &async_err, opts);
  EXPECT_NE(async_err.find("allocation limit"), std::string::npos) << async_err;
  EXPECT_EQ(run_sync("(collections.range 3)", opts), "(0 1 2)");
}

TEST(StdlibAllocLimitTest, TreeSettingResolvesPerSchedulerThenDefaults) {
  cvc::app app;
  auto &root = cvc::state::instance(app);
  root("state_exec.defaults.limits.max_alloc_bytes").value("4096");
  EXPECT_EQ(max_alloc_bytes_from_state(root, "s1"), 4096u);
  root("state_exec.schedulers.s1.limits.max_alloc_bytes").value("8192");
  EXPECT_EQ(max_alloc_bytes_from_state(root, "s1"), 8192u);
  EXPECT_EQ(max_alloc_bytes_from_state(root, "s2"), 4096u);
  root("state_exec.schedulers.s2.limits.max_alloc_bytes").value("lots"); // skipped
  EXPECT_EQ(max_alloc_bytes_from_state(root, "s2"), 4096u);

  // The sync scheduler reads it in load_settings() and publishes the effective value.
  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("s1");
  sched.load_settings();
  EXPECT_EQ(sched.max_alloc_bytes(), 8192u);
  EXPECT_EQ(root("state_exec.schedulers.s1.limits.max_alloc_bytes").value(), "8192");
  EXPECT_NE(thrown([&] { run_sync("(collections.range 1000)", {}, &root, "s1"); }), "");

  // The async scheduler reads the defaults key on every submission.
  std::string err;
  run_async("(collections.range 1000)", &err, {}, &root);
  EXPECT_NE(err.find("allocation limit"), std::string::npos) << err;
  root("state_exec.defaults.limits.max_alloc_bytes").value(std::to_string(1 << 20));
  EXPECT_EQ(run_async("(length (collections.range 1000))", &err, {}, &root), "1000");
  EXPECT_EQ(err, "");
}

TEST(StdlibAllocLimitTest, ZeroMeansNoLimitOfItsOwn) {
  cvc::app app;
  auto &root = cvc::state::instance(app);
  root("state_exec.defaults.limits.max_alloc_bytes").value("0");
  const std::string n = over_default_elements();
  EXPECT_EQ(run_sync("(length (collections.range " + n + "))", {}, &root), n);
  std::string err;
  EXPECT_EQ(run_async("(length (collections.range " + n + "))", &err, {}, &root), n);
  EXPECT_EQ(err, "");
  // max_memory still bounds it.
  execute_options opts;
  opts.max_memory = 1 << 20;
  EXPECT_NE(thrown([&] { run_sync("(collections.range " + n + ")", opts, &root); }), "");
}

TEST(StdlibAllocLimitTest, GuardsNestAndOnlyTighten) {
  EXPECT_EQ(eval_alloc_limit(), std::optional<std::size_t>(kDefaultMaxAllocBytes));
  {
    eval_alloc_guard unlimited(std::nullopt);
    EXPECT_EQ(eval_alloc_limit(), std::nullopt);
    {
      eval_alloc_guard outer(100);
      EXPECT_EQ(eval_alloc_limit(), std::optional<std::size_t>(100));
      eval_alloc_guard looser(1000);
      EXPECT_EQ(eval_alloc_limit(), std::optional<std::size_t>(100));
      eval_alloc_guard none(std::nullopt);
      EXPECT_EQ(eval_alloc_limit(), std::optional<std::size_t>(100));
    }
    EXPECT_EQ(eval_alloc_limit(), std::nullopt);
  }
  EXPECT_EQ(eval_alloc_limit(), std::optional<std::size_t>(kDefaultMaxAllocBytes));
}

TEST(StdlibAllocLimitTest, EveryGrowingResultIsChecked) {
  // Each of these builds a result bigger than any one argument; under a 64-byte limit each
  // throws naming itself, before allocating.
  const value_t s(std::string(40, 'a'));
  const value_t big(std::string(100, 'a'));
  const value_t three = make_list({int64_t(1), int64_t(2), int64_t(3)});
  const value_t nested = make_list({three, three, three});
  auto d1 = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  d1->emplace_back("a", value_t(int64_t(1)));
  auto d2 = std::make_shared<std::vector<std::pair<std::string, value_t>>>();
  d2->emplace_back("b", value_t(int64_t(2)));
  const auto concat = [](std::vector<value_t> args) {
    return builtin_str_concat(std::span<const value_t>(args.data(), args.size()));
  };

  eval_alloc_guard tight(64);
  EXPECT_NE(
      thrown([&] { call("string.split", {big, value_t(std::string(""))}); }).find("string.split"),
      std::string::npos);
  EXPECT_NE(thrown([&] {
              call("string.join", {make_list({s, s}), value_t(std::string(""))});
            }).find("string.join"),
            std::string::npos);
  EXPECT_NE(thrown([&] {
              call("string.replace", {s, value_t(std::string("a")), s});
            }).find("string.replace"),
            std::string::npos);
  EXPECT_NE(thrown([&] {
              call("bytes.concat",
                   {make_bytes(std::string(40, 'x')), make_bytes(std::string(40, 'x'))});
            }).find("bytes.concat"),
            std::string::npos);
  EXPECT_NE(
      thrown([&] { call("collections.range", {value_t(int64_t(100))}); }).find("collections.range"),
      std::string::npos);
  EXPECT_NE(thrown([&] { call("collections.zip", {three, three}); }).find("collections.zip"),
            std::string::npos);
  EXPECT_NE(thrown([&] { call("collections.flatten", {nested}); }).find("collections.flatten"),
            std::string::npos);
  EXPECT_NE(thrown([&] {
              call("collections.dict-merge", {value_t(d1), value_t(d2)});
            }).find("collections.dict-merge"),
            std::string::npos);
  EXPECT_NE(thrown([&] { concat({s, s}); }).find("str-concat"), std::string::npos);
  const auto env = builtins::make_default_environment();
  const value_t hundred(int64_t(100));
  const value_t gen = std::get<native_fn>(env->lookup("range")->v)(std::span(&hundred, 1));
  const native_fn &collect = std::get<native_fn>(env->lookup("collect")->v);
  EXPECT_NE(thrown([&] { collect(std::span(&gen, 1)); }).find("collect"), std::string::npos);

  // Results that fit still work under the same limit.
  EXPECT_EQ(show(call("string.replace", {value_t(std::string("hello")), value_t(std::string("l")),
                                         value_t(std::string("r"))})),
            "herro");
  EXPECT_EQ(show(concat({value_t(std::string("ab")), value_t(std::string("cd"))})), "abcd");
}

// --- behaviour kept or fixed by the rewrites -------------------------------------------------

TEST(StdlibBehaviourTest, ReplaceIsNonOverlappingLeftToRight) {
  EXPECT_EQ(run_sync("(string.replace \"hello\" \"l\" \"r\")"), "herro");
  EXPECT_EQ(run_sync("(string.replace \"aaa\" \"a\" \"bb\")"), "bbbbbb");
  EXPECT_EQ(run_sync("(string.replace \"aaaa\" \"aa\" \"b\")"), "bb");
  EXPECT_EQ(run_sync("(string.replace \"abc\" \"x\" \"y\")"), "abc");
  EXPECT_EQ(run_sync("(string.replace \"abc\" \"\" \"y\")"), "abc");
}

TEST(StdlibBehaviourTest, SplitAndJoinRoundTrip) {
  EXPECT_EQ(run_sync("(string.split \"a,b,,c\" \",\")"), "(\"a\" \"b\" \"\" \"c\")");
  EXPECT_EQ(run_sync("(string.join (string.split \"a,b,,c\" \",\") \"-\")"), "a-b--c");
  EXPECT_EQ(run_sync("(length (string.split \"caf\xC3\xA9\" \"\"))"), "4");
}

TEST(StdlibBehaviourTest, RangeCountsWithoutOverflow) {
  EXPECT_EQ(run_sync("(collections.range 2 5)"), "(2 3 4)");
  EXPECT_EQ(run_sync("(collections.range 5 0 -2)"), "(5 3 1)");
  EXPECT_EQ(run_sync("(collections.range 0 10 3)"), "(0 3 6 9)");
  EXPECT_EQ(run_sync("(collections.range 5 5)"), "()");
  EXPECT_EQ(run_sync("(collections.range 5 0)"), "()");
  // The old loop overflowed past INT64_MAX here; the count-based one stops.
  EXPECT_EQ(run_sync("(collections.range 9223372036854775806 9223372036854775807 10)"),
            "(9223372036854775806)");
  EXPECT_EQ(run_sync("(collections.range -9223372036854775807 -9223372036854775808 -5)"),
            "(-9223372036854775807)");
}

TEST(StdlibBehaviourTest, FlattenAndZip) {
  EXPECT_EQ(run_sync("(collections.flatten (list (list 1 2) 3 (list 4)))"), "(1 2 3 4)");
  EXPECT_EQ(run_sync("(collections.zip (list 1 2 3) (list \"a\" \"b\"))"), "((1 \"a\") (2 \"b\"))");
}
