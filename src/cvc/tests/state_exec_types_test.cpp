#include <chrono>
#include <cstdint>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <memory>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

TEST(StateExecTypesTest, NilValue) {
  value_t v;
  EXPECT_TRUE(v.is_nil());
  EXPECT_FALSE(v.is_truthy());
  EXPECT_EQ(v.type_name(), "nil");
  EXPECT_EQ(to_string(v), "nil");
}

TEST(StateExecTypesTest, BoolValues) {
  value_t t(true);
  value_t f(false);
  EXPECT_FALSE(t.is_nil());
  EXPECT_TRUE(t.is_truthy());
  EXPECT_FALSE(f.is_truthy());
  EXPECT_EQ(t.type_name(), "bool");
  EXPECT_EQ(to_string(t), "#t");
  EXPECT_EQ(to_string(f), "#f");
}

TEST(StateExecTypesTest, IntValue) {
  value_t v(int64_t(42));
  EXPECT_TRUE(v.is_truthy());
  EXPECT_EQ(v.type_name(), "int");
  EXPECT_EQ(to_string(v), "42");
}

TEST(StateExecTypesTest, IntFromInt) {
  value_t v(7);
  EXPECT_EQ(std::get<int64_t>(v.v), 7);
}

TEST(StateExecTypesTest, DoubleValue) {
  value_t v(3.14);
  EXPECT_EQ(v.type_name(), "float");
  auto s = to_string(v);
  EXPECT_NE(s.find("3.14"), std::string::npos);
}

TEST(StateExecTypesTest, StringValue) {
  value_t v(std::string("hello"));
  EXPECT_EQ(v.type_name(), "string");
  EXPECT_EQ(to_string(v), "\"hello\"");
}

TEST(StateExecTypesTest, StringFromCStr) {
  value_t v("world");
  EXPECT_EQ(std::get<std::string>(v.v), "world");
}

TEST(StateExecTypesTest, SymbolValue) {
  value_t v(symbol{"foo"});
  EXPECT_EQ(v.type_name(), "symbol");
  EXPECT_EQ(to_string(v), "foo");
}

TEST(StateExecTypesTest, SymbolEquality) {
  symbol a{"x"}, b{"x"}, c{"y"};
  EXPECT_EQ(a, b);
  EXPECT_NE(a, c);
}

TEST(StateExecTypesTest, ListValue) {
  auto list = make_list({value_t(int64_t(1)), value_t(int64_t(2)), value_t(int64_t(3))});
  EXPECT_EQ(list.type_name(), "list");
  EXPECT_EQ(to_string(list), "(1 2 3)");
}

TEST(StateExecTypesTest, EmptyList) {
  auto list = make_list();
  EXPECT_EQ(to_string(list), "()");
}

TEST(StateExecTypesTest, NestedList) {
  auto inner = make_list({value_t("a"), value_t("b")});
  auto outer = make_list({value_t(int64_t(1)), inner});
  EXPECT_EQ(to_string(outer), "(1 (\"a\" \"b\"))");
}

TEST(StateExecTypesTest, DictValue) {
  auto dict = make_dict({{"name", value_t("Alice")}, {"age", value_t(int64_t(30))}});
  EXPECT_EQ(dict.type_name(), "dict");
  auto s = to_string(dict);
  EXPECT_NE(s.find("\"name\""), std::string::npos);
  EXPECT_NE(s.find("\"Alice\""), std::string::npos);
}

TEST(StateExecTypesTest, EmptyDict) {
  auto dict = make_dict();
  EXPECT_EQ(to_string(dict), "{}");
}

TEST(StateExecTypesTest, ClosureValue) {
  auto env = std::make_shared<environment>();
  auto c = std::make_shared<closure>();
  c->params = {symbol{"x"}};
  c->body = {value_t(symbol{"x"})};
  c->env_snapshot = env;
  value_t v(c);
  EXPECT_EQ(v.type_name(), "closure");
  EXPECT_EQ(to_string(v), "<closure>");
}

TEST(StateExecTypesTest, NativeFnValue) {
  native_fn fn = [](std::span<const value_t>) -> value_t { return value_t(int64_t(42)); };
  value_t v(fn);
  EXPECT_EQ(v.type_name(), "native_fn");
  EXPECT_EQ(to_string(v), "<native_fn>");
}

TEST(StateExecTypesTest, DataObjectValue) {
  auto obj = std::make_shared<data_object>();
  obj->payload = 42;
  obj->type_name = "int";
  value_t v(obj);
  EXPECT_EQ(v.type_name(), "data_object");
  EXPECT_TRUE(obj->is_type("int"));
  EXPECT_FALSE(obj->is_type("string"));
  auto s = to_string(v);
  EXPECT_NE(s.find("data_object:int"), std::string::npos);
}

TEST(StateExecTypesTest, EnvironmentLookup) {
  auto env = std::make_shared<environment>();
  env->set("x", value_t(int64_t(10)));

  auto *result = env->lookup("x");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(std::get<int64_t>(result->v), 10);
  EXPECT_EQ(env->lookup("y"), nullptr);
}

TEST(StateExecTypesTest, EnvironmentScopeChain) {
  auto outer = std::make_shared<environment>();
  outer->set("a", value_t(int64_t(1)));

  auto inner = environment::extend(outer);
  inner->set("b", value_t(int64_t(2)));

  // Inner sees both
  EXPECT_NE(inner->lookup("a"), nullptr);
  EXPECT_NE(inner->lookup("b"), nullptr);
  EXPECT_EQ(std::get<int64_t>(inner->lookup("a")->v), 1);
  EXPECT_EQ(std::get<int64_t>(inner->lookup("b")->v), 2);

  // Outer doesn't see inner
  EXPECT_EQ(outer->lookup("b"), nullptr);
}

TEST(StateExecTypesTest, EnvironmentSetExisting) {
  auto outer = std::make_shared<environment>();
  outer->set("x", value_t(int64_t(1)));

  auto inner = environment::extend(outer);

  // set_existing should modify outer
  inner->set_existing("x", value_t(int64_t(99)));
  EXPECT_EQ(std::get<int64_t>(outer->lookup("x")->v), 99);
}

TEST(StateExecTypesTest, EnvironmentSetExistingFallback) {
  auto env = std::make_shared<environment>();
  // Name not found — should set in current scope
  env->set_existing("new_var", value_t("hello"));
  EXPECT_NE(env->lookup("new_var"), nullptr);
}

TEST(StateExecTypesTest, ValuesEqual) {
  EXPECT_TRUE(values_equal(nil_value, nil_value));
  EXPECT_TRUE(values_equal(true_value, true_value));
  EXPECT_TRUE(values_equal(value_t(int64_t(42)), value_t(int64_t(42))));
  EXPECT_FALSE(values_equal(value_t(int64_t(42)), value_t(int64_t(43))));
  EXPECT_TRUE(values_equal(value_t("abc"), value_t("abc")));
  EXPECT_FALSE(values_equal(value_t("abc"), value_t("xyz")));
}

TEST(StateExecTypesTest, ValuesEqualLists) {
  auto a = make_list({value_t(int64_t(1)), value_t(int64_t(2))});
  auto b = make_list({value_t(int64_t(1)), value_t(int64_t(2))});
  auto c = make_list({value_t(int64_t(1)), value_t(int64_t(3))});
  EXPECT_TRUE(values_equal(a, b));
  EXPECT_FALSE(values_equal(a, c));
}

TEST(StateExecTypesTest, ValuesEqualDicts) {
  auto a = make_dict({{"k", value_t(int64_t(1))}});
  auto b = make_dict({{"k", value_t(int64_t(1))}});
  auto c = make_dict({{"k", value_t(int64_t(2))}});
  EXPECT_TRUE(values_equal(a, b));
  EXPECT_FALSE(values_equal(a, c));
}

TEST(StateExecTypesTest, ValuesEqualCrossType) {
  EXPECT_FALSE(values_equal(value_t(int64_t(1)), value_t(1.0)));
  EXPECT_FALSE(values_equal(nil_value, false_value));
}

TEST(StateExecTypesTest, Truthiness) {
  EXPECT_FALSE(nil_value.is_truthy());
  EXPECT_FALSE(false_value.is_truthy());
  EXPECT_TRUE(true_value.is_truthy());
  EXPECT_TRUE(value_t(int64_t(0)).is_truthy()); // 0 is truthy (not false)
  EXPECT_TRUE(value_t("").is_truthy());         // empty string is truthy
  EXPECT_TRUE(make_list().is_truthy());         // empty list is truthy
}

// -- bounded / cycle-safe walks (values_equal, to_string) --------------------

namespace {
// A shared-pointer DAG: `depth` doublings => `depth` PHYSICAL nodes but 2^depth logical.
value_t make_dag(int depth) {
  value_t node{int64_t{1}};
  for (int i = 0; i < depth; ++i)
    node = make_list({node, node});
  return node;
}
} // namespace

TEST(StateExecTypesTest, ValuesEqualMemoizedOnSharedDag) {
  // Two independently-built 40-deep DAGs (distinct pointers at every level) unfold to 2^40
  // logical nodes. Memoized values_equal compares them in physical time; a naive walk hangs.
  EXPECT_TRUE(values_equal(make_dag(40), make_dag(40)));
  // A shape/leaf difference is detected, still bounded.
  value_t a = make_dag(30);
  value_t b = make_list({make_dag(29), value_t{int64_t{2}}});
  EXPECT_FALSE(values_equal(a, b));
}

TEST(StateExecTypesTest, ToStringBoundedOnSharedDag) {
  const std::string s = to_string(make_dag(40)); // 2^40 logical -> truncated, never OOM/hang
  EXPECT_LT(s.size(), std::size_t{1} << 17);
}

TEST(StateExecTypesTest, WalksTerminateOnCyclicStructure) {
  auto l = std::make_shared<std::vector<value_t>>();
  l->push_back(value_t{int64_t{1}});
  value_t cyc{l};
  l->push_back(cyc); // l = [1, <-self]  (cyclic) — a naive walk would recurse forever
  const std::string s = to_string(cyc);
  EXPECT_FALSE(s.empty());
  EXPECT_LT(s.size(), std::size_t{1} << 17);
  EXPECT_TRUE(values_equal(cyc, cyc)); // must terminate, not overflow the stack
}

// A time_budget charges its first half on the steady clock and its second on the thread's CPU
// clock. A sleeping thread stands in for one the OS has descheduled on a loaded machine (neither
// runs): three times the budget passes without spending it, and only running -- spinning here --
// spends the second half.
TEST(StateExecTypesTest, TimeBudgetSecondHalfIsTheThreadsOwnRunningTime) {
  time_budget b(0.02);
  b.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  const double c0 = thread_cpu_seconds();
  EXPECT_FALSE(b.spent());
  while (!b.spent()) {
  }
  EXPECT_GE(thread_cpu_seconds() - c0, 0.01);
  b.stop();
  EXPECT_TRUE(b.spent());
  EXPECT_EQ(b.remaining(), 0.0);
  EXPECT_TRUE(time_budget(0.0).spent()); // an empty budget is spent from the start
}

// In that second half a check charges at most a sixteenth of it, so a stall between two checks
// -- here the thread runs for twice the half without one, as a virtual machine's CPU clock can
// count time its hypervisor stole -- cannot spend the budget: only running, check by check, can.
TEST(StateExecTypesTest, TimeBudgetChargesAStallAtMostASixteenthOfTheSecondHalf) {
  time_budget b(0.02);
  b.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_FALSE(b.spent()); // the first half is spent: the second is charged on the CPU clock
  const double c0 = thread_cpu_seconds();
  while (thread_cpu_seconds() - c0 < 0.02) {
  }
  EXPECT_FALSE(b.spent()); // one check: a sixteenth
  const double c1 = thread_cpu_seconds();
  while (!b.spent()) {
  }
  EXPECT_GE(thread_cpu_seconds() - c1, 0.009); // the other fifteen sixteenths
}

// The evaluation deadline is such a budget, and a nested guard tightens it.
TEST(StateExecTypesTest, EvalDeadlineIsSpentOnlyByRunning) {
  EXPECT_FALSE(eval_deadline_expired()); // none armed
  eval_deadline_guard guard(0.02);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_FALSE(eval_deadline_expired());
  {
    eval_deadline_guard none(std::nullopt); // arms nothing
    eval_deadline_guard spent(0.0);
    EXPECT_TRUE(eval_deadline_expired());
  }
  while (!eval_deadline_expired()) {
  }
}

// ─── Scopes from extend(): recycled per thread, looked up with one hash ───────

TEST(StateExecTypesTest, ExtendedScopeComesBackEmpty) {
  auto global = std::make_shared<environment>();
  global->set("g", value_t(int64_t(1)));
  // Take every scope this thread kept, so that the pool has room for the one dropped below.
  std::vector<environment_ptr> taken;
  for (int i = 0; i < 64; ++i)
    taken.push_back(environment::extend(nullptr));
  const std::size_t fresh = decltype(environment::bindings)().bucket_count();
  const environment *first = nullptr;
  bool keepable = false;
  {
    auto scope = environment::extend(global);
    first = scope.get();
    scope->set("a", value_t(int64_t(2)));
    scope->set("b", value_t(std::string(40, 'x'))); // a string past the small buffer
    scope->set("a", value_t(int64_t(3)));           // set again: one binding, the new value
    EXPECT_EQ(scope->bindings.size(), 2u);
    EXPECT_EQ(std::get<int64_t>(scope->lookup("a")->v), 3);
    scope->frozen = true; // a kept scope is unfrozen again
    // Kept only with a new map's buckets: on MSVC (8 buckets up to 8 bindings), not where a new
    // map has none and the first binding allocates some (libstdc++, libc++).
    keepable = scope->bindings.bucket_count() == fresh;
  }
  // The next scope made on this thread is empty, unfrozen and in the given scope, and nothing
  // the dropped one bound is visible through it: when the dropped scope was kept, it is that
  // scope.
  auto again = environment::extend(global);
  if (keepable)
    EXPECT_EQ(again.get(), first);
  EXPECT_TRUE(again->bindings.empty());
  EXPECT_FALSE(again->frozen);
  EXPECT_EQ(again->outer, global);
  EXPECT_EQ(again->lookup("a"), nullptr);
  EXPECT_EQ(again->lookup("b"), nullptr);
  EXPECT_EQ(std::get<int64_t>(again->lookup("g")->v), 1);
  again->set("c", value_t(int64_t(4)));
  EXPECT_EQ(std::get<int64_t>(again->lookup("c")->v), 4);
  EXPECT_EQ(global->lookup("c"), nullptr);
}

TEST(StateExecTypesTest, ScopeDropsItsBindingsAsADestructorWould) {
  // A last owner a recycled scope drops goes where a destroyed scope's would: to the release
  // backlog routing this thread's drops (a slicing step frees it a slice at a time).
  detail::release_backlog backlog;
  std::weak_ptr<std::vector<value_t>> held;
  {
    auto scope = environment::extend(nullptr);
    value_t list = make_list({value_t(int64_t(1)), value_t(int64_t(2))});
    held = std::get<list_ptr>(list.v);
    scope->set("l", std::move(list));
    detail::backlog_scope routing(&backlog);
    scope.reset();
  }
  EXPECT_FALSE(backlog.empty());
  EXPECT_FALSE(held.expired());
  backlog.drain(0);
  EXPECT_TRUE(held.expired());
}

TEST(StateExecTypesTest, ScopeLetsGoOfWhatANativeFunctionBoundInItCaptured) {
  // A recycled scope keeps its binding nodes, not their values: a native function bound in it
  // drops what it captured when the scope goes. (libc++ moves a std::function whose callable
  // fits in it, as this one does, by copying the callable: a node left holding the moved-from
  // value would keep the capture alive until the node was reused.)
  auto captured = std::make_shared<int>(7);
  const std::weak_ptr<int> watched = captured;
  {
    auto scope = environment::extend(nullptr);
    scope->set("f", value_t{native_fn{[captured](std::span<const value_t>) -> value_t {
                 return value_t(int64_t(*captured));
               }}});
    captured.reset();
    EXPECT_FALSE(watched.expired());
  }
  EXPECT_TRUE(watched.expired());
}

TEST(StateExecTypesTest, AScopeMadeInADroppedOnesBlockIsAnotherScope) {
  // extend() hands out the block the scope dropped last left, so a new scope is likely at that
  // scope's address: its serial tells the two apart (a call record checks scopes by both).
  std::vector<environment_ptr> taken; // room in this thread's pool for the scope dropped below
  for (int i = 0; i < 64; ++i)
    taken.push_back(environment::extend(nullptr));
  const environment *was = nullptr;
  std::uint32_t serial = 0;
  {
    auto scope = environment::extend(nullptr); // never bound: kept on every platform
    was = scope.get();
    serial = scope->serial.value;
  }
  auto again = environment::extend(nullptr);
  EXPECT_EQ(again.get(), was);
  EXPECT_NE(again->serial.value, serial);
  // A scope made, copied or assigned has a serial no other has.
  auto made = std::make_shared<environment>();
  EXPECT_NE(made->serial.value, again->serial.value);
  EXPECT_NE(made->serial.value, serial);
  detail::scope_serial copied = made->serial;
  EXPECT_NE(copied.value, made->serial.value);
  const std::uint32_t before = copied.value;
  copied = again->serial;
  EXPECT_NE(copied.value, before);
  EXPECT_NE(copied.value, again->serial.value);
}

TEST(StateExecTypesTest, ScopeChainOutlivesItsLinks) {
  // A long chain of scopes is freed a link at a time, recycled or not.
  environment_ptr e = std::make_shared<environment>();
  for (int i = 0; i < 200000; ++i)
    e = environment::extend(e);
  e.reset();
  SUCCEED();
}

TEST(StateExecTypesTest, LookupFindsEveryNameInLargeAndNestedScopes) {
  // Lookups hash a name once and walk each scope's bucket: check that against many names, scopes
  // that grew through several rehashes, empty scopes in between, and shadowing.
  auto outer = std::make_shared<environment>();
  for (int i = 0; i < 5000; ++i)
    outer->set("name_" + std::to_string(i), value_t(int64_t(i)));
  auto empty = environment::extend(outer);
  auto inner = environment::extend(empty);
  for (int i = 0; i < 300; i += 3)
    inner->set("name_" + std::to_string(i), value_t(int64_t(-i)));
  inner->set("a_name_well_past_the_small_string_buffer", value_t(int64_t(7)));
  for (int i = 0; i < 5000; ++i) {
    const std::string n = "name_" + std::to_string(i);
    const value_t *v = inner->lookup(n);
    ASSERT_NE(v, nullptr) << n;
    EXPECT_EQ(std::get<int64_t>(v->v), i < 300 && i % 3 == 0 ? -i : i) << n;
    const environment &c = *inner;
    EXPECT_EQ(c.lookup(n), v);
  }
  EXPECT_EQ(std::get<int64_t>(inner->lookup("a_name_well_past_the_small_string_buffer")->v), 7);
  for (const char *missing : {"", "name_", "name_5000", "Name_1", "a_name_well_past_the_small"})
    EXPECT_EQ(inner->lookup(missing), nullptr) << missing;
}

TEST(StateExecTypesTest, ScopeDroppedAtThreadExitAfterThePool) {
  // A scope a thread_local destructor drops after the thread's pool is gone is freed, not kept.
  struct holder {
    environment_ptr scope;
    ~holder() { scope.reset(); }
  };
  std::weak_ptr<environment> watched;
  std::thread t([&] {
    thread_local holder h; // made before the pool below, so destroyed after it
    h.scope = nullptr;
    auto scope = environment::extend(nullptr);
    scope->set("x", value_t(int64_t(1)));
    watched = scope;
    h.scope = std::move(scope);
  });
  t.join();
  EXPECT_TRUE(watched.expired());
}

TEST(StateExecTypesTest, KeptScopeHasANewScopesBuckets) {
  // A scope is kept for the next extend() only while its map has the buckets a new map has, so
  // a scope's bucket count (which heap measurement counts) never depends on earlier scopes.
  const std::size_t fresh = decltype(environment::bindings)().bucket_count();
  {
    auto grown = environment::extend(nullptr);
    for (int i = 0; i < 40; ++i)
      grown->set("v" + std::to_string(i), value_t(int64_t(i)));
    EXPECT_GT(grown->bindings.bucket_count(), fresh);
  }
  for (int i = 0; i < 3; ++i) {
    auto scope = environment::extend(nullptr);
    EXPECT_EQ(scope->bindings.bucket_count(), fresh) << i;
    EXPECT_TRUE(scope->bindings.empty());
    scope->set("a", value_t(int64_t(1)));
  }
}

TEST(StateExecTypesTest, ScopeIsKeptOnlyOnceNoWeakReferenceRemains) {
  // A scope and its control block are one block: kept for reuse only when both are done with.
  std::weak_ptr<environment> watched;
  const environment *was = nullptr;
  {
    auto scope = environment::extend(nullptr);
    watched = scope;
    was = scope.get();
  }
  EXPECT_TRUE(watched.expired());
  auto other = environment::extend(nullptr);
  EXPECT_NE(other.get(), was); // still the weak reference's
  watched.reset();
  auto again = environment::extend(nullptr);
  EXPECT_EQ(again.get(), was); // kept once the weak reference went, and taken next
}
