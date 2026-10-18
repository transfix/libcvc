// Deeply nested state_exec values must not overflow the native stack.
//
// The parser's nesting limit (parser.max_depth on the state tree) exists because the reader
// recurses; it can rise only once nothing else does either. These tests build values 100000
// levels deep at runtime (not through the parser) and drop, print, compare and snapshot them
// (the deep copy behind state-data-get is in state_exec_intrinsics_test). Each body runs on a
// thread with a small fixed stack (kStackKiB), so a walker that recurses once per level fails
// here on every platform, whatever the main thread's stack is.
//
// For probing, CVC_DEEP_DATA_DEPTH overrides the depths and CVC_DEEP_DATA_STACK_KIB the stack
// (0 runs the body on the calling thread).

#include <algorithm>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_exec/types.h>
#include <exception>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace cvc::state_exec;

namespace {

constexpr std::size_t kDepth = 100000;
// The snapshot codec writes cvc::state nodes, and each write signals every ancestor, so a level
// costs tens of microseconds: 10000 levels keep the codec tests to seconds, and are still far
// past the 441 levels the recursive codec survived on a 1 MiB stack (104 on 256 KiB).
constexpr std::size_t kCodecDepth = 10000;
constexpr std::size_t kStackKiB = 256;

std::size_t env_size(const char *name, std::size_t fallback) {
  const char *s = std::getenv(name);
  return s && *s ? static_cast<std::size_t>(std::strtoull(s, nullptr, 10)) : fallback;
}

std::size_t depth() { return env_size("CVC_DEEP_DATA_DEPTH", kDepth); }
std::size_t codec_depth() { return env_size("CVC_DEEP_DATA_DEPTH", kCodecDepth); }

// Run `body` on a thread with a kStackKiB stack and rethrow what it throws.
void on_small_stack(const std::function<void()> &body) {
  const std::size_t kib = env_size("CVC_DEEP_DATA_STACK_KIB", kStackKiB);
  if (kib == 0) {
    body();
    return;
  }
  std::exception_ptr error;
  boost::thread::attributes attrs;
  attrs.set_stack_size(kib * 1024);
  boost::thread t(attrs, [&] {
    try {
      body();
    } catch (...) {
      error = std::current_exception();
    }
  });
  t.join();
  if (error)
    std::rethrow_exception(error);
}

// (((... (leaf) ...))): `n` lists, the innermost holding `leaf`.
value_t nested_list(std::size_t n, value_t leaf = value_t{int64_t{1}}) {
  value_t v = make_list({std::move(leaf)});
  for (std::size_t i = 1; i < n; ++i)
    v = make_list({std::move(v)});
  return v;
}

// {"k": {"k": ... {"k": 1} ...}}: `n` dicts.
value_t nested_dict(std::size_t n) {
  value_t v = make_dict({{"k", value_t{int64_t{1}}}});
  for (std::size_t i = 1; i < n; ++i)
    v = make_dict({{"k", std::move(v)}});
  return v;
}

// Lists and dicts alternating, each level also holding a scalar sibling.
value_t nested_mixed(std::size_t n) {
  value_t v = value_t{int64_t{0}};
  for (std::size_t i = 0; i < n; ++i)
    v = i % 2 ? make_dict({{"i", value_t{int64_t(i)}}, {"v", std::move(v)}})
              : make_list({value_t{std::string("s")}, std::move(v)});
  return v;
}

// The innermost list of nested_list(), as a weak pointer: expired once the value is freed.
std::weak_ptr<std::vector<value_t>> innermost(const value_t &v) {
  const value_t *cur = &v;
  list_ptr last;
  while (auto *l = std::get_if<list_ptr>(&cur->v)) {
    last = *l;
    if ((*l)->empty())
      break;
    cur = &(*l)->front();
  }
  return last;
}

// How many lists deep a nested_list() is, walked iteratively.
std::size_t list_depth(const value_t &v) {
  std::size_t n = 0;
  const value_t *cur = &v;
  while (auto *l = std::get_if<list_ptr>(&cur->v)) {
    ++n;
    if (!*l || (*l)->empty())
      break;
    cur = &(*l)->front();
  }
  return n;
}

// The paths of every node under `node`, relative to it (state::children() lists all
// descendants, by full name).
std::vector<std::string> descendants(cvc::state &node) {
  const std::size_t prefix = node.fullName().size() + 1;
  std::vector<std::string> paths;
  for (const std::string &full : node.children())
    paths.push_back(full.substr(prefix));
  return paths;
}

// The depth of the state subtree under `node`.
std::size_t state_depth(cvc::state &node) {
  std::size_t deepest = 0;
  for (const std::string &path : descendants(node))
    deepest = std::max<std::size_t>(deepest, 1 + std::count(path.begin(), path.end(), '.'));
  return deepest;
}

// ---- drop ------------------------------------------------------------------

TEST(StateExecDeepData, DropList) {
  on_small_stack([] {
    value_t v = nested_list(depth());
    auto last = innermost(v);
    v = value_t{};
    EXPECT_TRUE(last.expired());
  });
}

TEST(StateExecDeepData, DropListOutOfScope) {
  on_small_stack([] {
    std::weak_ptr<std::vector<value_t>> last;
    {
      value_t v = nested_list(depth());
      last = innermost(v);
    }
    EXPECT_TRUE(last.expired());
  });
}

TEST(StateExecDeepData, DropDict) {
  on_small_stack([] { value_t v = nested_dict(depth()); });
}

TEST(StateExecDeepData, DropMixed) {
  on_small_stack([] { value_t v = nested_mixed(depth()); });
}

TEST(StateExecDeepData, DropBareListPtr) {
  // The last owner is a list_ptr, not a value_t: the vector's elements free themselves.
  on_small_stack([] {
    list_ptr l = std::get<list_ptr>(nested_list(depth()).v);
    l.reset();
  });
}

TEST(StateExecDeepData, DropKeepsSharedTail) {
  on_small_stack([] {
    const std::size_t n = depth();
    value_t tail = nested_list(n);
    value_t a = make_list({tail});
    value_t b = make_list({tail});
    tail = value_t{};
    a = value_t{};
    EXPECT_EQ(list_depth(b), n + 1); // still whole: `a` only dropped its reference
    auto last = innermost(b);
    b = value_t{};
    EXPECT_TRUE(last.expired());
  });
}

TEST(StateExecDeepData, DropClosureChain) {
  // closure -> env_snapshot -> binding -> closure -> ...: the chain a recursive function that
  // returns a closure over its inner call's closure builds at runtime.
  on_small_stack([] {
    value_t v;
    for (std::size_t i = 0; i < depth(); ++i) {
      auto env = std::make_shared<environment>();
      env->set("next", std::move(v));
      auto c = std::make_shared<closure>();
      c->env_snapshot = std::move(env);
      v = value_t{c};
    }
  });
}

TEST(StateExecDeepData, DropEnvironmentChain) {
  on_small_stack([] {
    environment_ptr env;
    for (std::size_t i = 0; i < depth(); ++i)
      env = environment::extend(std::move(env));
    std::weak_ptr<environment> inner = env;
    env.reset();
    EXPECT_TRUE(inner.expired());
  });
}

// ---- to_string -------------------------------------------------------------

TEST(StateExecDeepData, ToStringIsBounded) {
  on_small_stack([] {
    value_t v = nested_list(depth());
    const std::string s = to_string(v);
    // Output is capped at 64 KiB (plus the closing parens of what was opened), never per
    // level of depth.
    EXPECT_LE(s.size(), std::size_t{2} * 65536 + 16);
    EXPECT_EQ(s.front(), '(');
    EXPECT_NE(s.find("\xE2\x80\xA6"), std::string::npos);
  });
}

TEST(StateExecDeepData, ToStringPrintsDeepValueWithinLengthCap) {
  // 5000 levels print in ~10 KB, under the length cap, so they print whole: depth alone
  // never truncates.
  on_small_stack([] {
    const std::size_t n = 5000;
    EXPECT_EQ(to_string(nested_list(n)), std::string(n, '(') + "1" + std::string(n, ')'));
  });
}

TEST(StateExecDeepData, ToStringDeepDict) {
  on_small_stack([] {
    const std::string s = to_string(nested_dict(depth()));
    EXPECT_LE(s.size(), std::size_t{3} * 65536);
    EXPECT_EQ(s.rfind("{\"k\": {\"k\": ", 0), 0u);
  });
}

// ---- values_equal ----------------------------------------------------------

TEST(StateExecDeepData, ValuesEqualDeepLists) {
  on_small_stack([] {
    const std::size_t n = depth();
    EXPECT_TRUE(values_equal(nested_list(n), nested_list(n)));
    EXPECT_FALSE(values_equal(nested_list(n), nested_list(n, value_t{int64_t{2}})));
    EXPECT_FALSE(values_equal(nested_list(n), nested_list(n - 1)));
  });
}

TEST(StateExecDeepData, ValuesEqualDeepDicts) {
  on_small_stack([] {
    const std::size_t n = depth();
    EXPECT_TRUE(values_equal(nested_dict(n), nested_dict(n)));
    EXPECT_TRUE(values_equal(nested_mixed(n), nested_mixed(n)));
    EXPECT_FALSE(values_equal(nested_mixed(n), nested_mixed(n - 1)));
  });
}

// ---- macro expansion -------------------------------------------------------

TEST(StateExecDeepData, MacroExpandsDeepTemplate) {
  // (begin (defmacro m (x) (quote (((... x ...))))) (m 7)), the template built at runtime.
  on_small_stack([] {
    const std::size_t n = depth();
    auto sym = [](const char *s) { return value_t{symbol{s}}; };
    value_t body = make_list({sym("quote"), nested_list(n, sym("x"))});
    value_t expr = make_list({sym("begin"),
                              make_list({sym("defmacro"), sym("m"), make_list({sym("x")}), body}),
                              make_list({sym("m"), value_t{int64_t{7}}})});
    stackless_evaluator ev(builtins::make_default_environment());
    value_t r = ev.evaluate(expr);
    EXPECT_EQ(list_depth(r), n);
    EXPECT_TRUE(values_equal(r, nested_list(n, value_t{int64_t{7}})));
  });
}

// ---- snapshot codec --------------------------------------------------------

class StateExecDeepDataCodec : public ::testing::Test {
protected:
  cvc::app ctx;

  // A node as deep as a process snapshot's would be.
  cvc::state &snapshot_node(const std::string &name) {
    return ctx.root()("state_exec.processes.p1.snapshot." + name);
  }

  // Round-trip `v` through `node`. The state tree written stays shallow, whatever the value's
  // depth: cvc::state's own walks (fullName(), childChanged up to the root) recurse once per
  // tree level.
  void round_trip(cvc::state &node, const value_t &v) {
    on_small_stack([&] {
      encode_value(node, v);
      EXPECT_TRUE(values_equal(v, decode_value(node)));
    });
    EXPECT_LE(state_depth(node), 32u);
  }
};

TEST_F(StateExecDeepDataCodec, ListRoundTrip) {
  const std::size_t n = codec_depth();
  value_t v = nested_list(n);
  round_trip(snapshot_node("deep_list"), v);
  EXPECT_EQ(list_depth(decode_value(snapshot_node("deep_list"))), n);
}

TEST_F(StateExecDeepDataCodec, DictRoundTrip) {
  round_trip(snapshot_node("deep_dict"), nested_dict(codec_depth()));
}

TEST_F(StateExecDeepDataCodec, MixedRoundTrip) {
  round_trip(snapshot_node("deep_mixed"), nested_mixed(codec_depth()));
}

TEST_F(StateExecDeepDataCodec, ShallowLayoutUnchanged) {
  // Values shallower than the spill depth encode exactly as before (no __spill__ table), so
  // snapshots of ordinary values are unchanged and old snapshots still decode.
  auto &node = ctx.root()("shallow");
  encode_value(node, make_list({make_list({value_t{int64_t{7}}}), make_dict({{"a", value_t{}}})}));
  EXPECT_EQ(node("__type__").value(), "list");
  EXPECT_EQ(node("__elems__.000000.__type__").value(), "list");
  EXPECT_EQ(node("__elems__.000000.__elems__.000000.__type__").value(), "int");
  EXPECT_EQ(node("__elems__.000000.__elems__.000000").value(), "7");
  EXPECT_EQ(node("__elems__.000001.__type__").value(), "dict");
  EXPECT_EQ(node("__elems__.000001.__entries__.000000.__key__").value(), "a");
  EXPECT_EQ(node("__elems__.000001.__entries__.000000.__val__.__type__").value(), "nil");
  EXPECT_EQ(node.findDescendant("__spill__"), nullptr);
}

TEST_F(StateExecDeepDataCodec, CyclicValueThrows) {
  auto l = std::make_shared<std::vector<value_t>>();
  l->push_back(value_t{int64_t{1}});
  l->push_back(value_t{l});
  EXPECT_THROW(encode_value(ctx.root()("cycle"), value_t{l}), std::runtime_error);
  l->clear(); // break the cycle so the list is freed
}

TEST_F(StateExecDeepDataCodec, ClosureChainRoundTrip) {
  // Each closure's scope binds the closure made before it: the codec used to recurse twice per
  // closure (into its scope, and from the binding into the next closure).
  const std::size_t n = codec_depth();
  value_t chain = nil_value;
  for (std::size_t i = 0; i < n; ++i) {
    auto scope = std::make_shared<environment>();
    scope->set("prev", chain);
    auto c = std::make_shared<closure>();
    c->body = {value_t{symbol{"prev"}}};
    c->env_snapshot = scope;
    chain = value_t{c};
  }
  cvc::state &node = snapshot_node("closures");
  on_small_stack([&] {
    encode_value(node, chain);
    value_t back = decode_value(node);
    std::size_t depth = 0;
    for (const value_t *v = &back; std::holds_alternative<closure_ptr>(v->v); ++depth)
      v = std::get<closure_ptr>(v->v)->env_snapshot->lookup("prev");
    EXPECT_EQ(depth, n);
  });
  EXPECT_LE(state_depth(node), 32u);
}

TEST_F(StateExecDeepDataCodec, SharedValueEncodesTwice) {
  // A DAG (the same list reached twice) is not a cycle: it encodes, and decodes as two copies.
  value_t inner = make_list({value_t{int64_t{3}}});
  value_t v = make_list({inner, inner});
  auto &node = ctx.root()("dag");
  encode_value(node, v);
  EXPECT_TRUE(values_equal(v, decode_value(node)));
}

TEST_F(StateExecDeepDataCodec, BadSpillReferenceThrows) {
  auto &node = ctx.root()("bad_ref");
  encode_value(node, nested_list(200));
  ASSERT_NE(node.findDescendant("__spill__"), nullptr);
  std::vector<cvc::state *> refs;
  for (const std::string &path : descendants(node)) {
    const std::string suffix = ".__type__";
    if (path.size() > suffix.size() && path.ends_with(suffix) && node(path).value() == "spill_ref")
      refs.push_back(&node(path.substr(0, path.size() - suffix.size())));
  }
  ASSERT_GE(refs.size(), 2u);
  const std::string first = refs[0]->value();
  // Past the end of the table.
  refs[0]->value("999999");
  EXPECT_THROW(decode_value(node), std::runtime_error);
  // Not a number.
  refs[0]->value("x");
  EXPECT_THROW(decode_value(node), std::runtime_error);
  // Every reference to entry 0, which then holds a reference to itself: decoding must stop
  // at the second use, not expand forever.
  std::vector<std::string> saved;
  for (cvc::state *r : refs) {
    saved.push_back(r->value());
    r->value("0");
  }
  EXPECT_THROW(decode_value(node), std::runtime_error);
  for (std::size_t i = 0; i < refs.size(); ++i)
    refs[i]->value(saved[i]);
  refs[0]->value(first);
  EXPECT_TRUE(values_equal(decode_value(node), nested_list(200)));
}

// ---- native nesting ----------------------------------------------------------
//
// A native function that calls another function (apply, send, a class constructor running init,
// an inherited init running its superclass's) or runs a nested evaluator (a class method, a
// generator) nests on the native stack. detail::native_nesting refuses a level past
// kMaxNativeNesting levels or with less than kNativeStackReserve bytes of stack left, so a value or
// class that refers to itself throws instead of overflowing. The runaway cases run on a stack only
// 64 KiB bigger than the reserve, where the stack runs out first, and must throw, not crash; the
// legitimate ones run on 1 MiB.

constexpr std::size_t kSmallStackKiB = kNativeStackReserve / 1024 + 64;
constexpr std::size_t kThreadStackKiB = 1024;
constexpr std::size_t kWasmStackKiB = 64; // wasm's default stack

// Run `body` on a thread with a `kib` stack.
void on_stack(std::size_t kib, const std::function<void()> &body) {
  std::exception_ptr error;
  boost::thread::attributes attrs;
  attrs.set_stack_size(kib * 1024);
  boost::thread t(attrs, [&] {
    try {
      body();
    } catch (...) {
      error = std::current_exception();
    }
  });
  t.join();
  if (error)
    std::rethrow_exception(error);
}

value_t run_script(const std::string &script) {
  stackless_evaluator ev(builtins::make_default_environment());
  return ev.evaluate(parse(script));
}

// Run `script` on a `kib` stack, expecting the nesting limit's error; returns the levels it
// reports. Afterwards no level is left counted.
std::size_t expect_nesting_limit(const std::string &script, std::size_t kib = kSmallStackKiB) {
  std::size_t levels = 0;
  on_stack(kib, [&] {
    try {
      run_script(script);
      ADD_FAILURE() << "expected the native nesting limit: " << script;
    } catch (const std::runtime_error &e) {
      const std::string what = e.what();
      const std::size_t at = what.find("nested too deeply (");
      EXPECT_NE(at, std::string::npos) << what;
      if (at != std::string::npos)
        levels = std::strtoull(what.c_str() + at + 19, nullptr, 10);
    }
    EXPECT_EQ(detail::native_nesting::depth(), 0u);
  });
  return levels;
}

TEST(StateExecNativeNesting, ApplyOnASelfContainingList) {
  // (apply apply L) with L = (apply L): apply calls itself once per trip around the cycle.
  const std::string loop = "(begin (set L (list apply nil)) (set-nth L 1 L) (apply apply L))";
  // On a small stack the stack left decides; on a big one, the level count.
  EXPECT_LT(expect_nesting_limit(loop), kMaxNativeNesting);
  EXPECT_EQ(expect_nesting_limit(loop, 4096), kMaxNativeNesting);
}

TEST(StateExecNativeNesting, SendThroughItself) {
  // A class whose method "m" is `send` itself: (send o "m" "m" ...) re-sends once per "m".
  std::string ms;
  for (int i = 0; i < 5000; ++i)
    ms += " \"m\"";
  expect_nesting_limit("(begin (defclass C) (set-attr C.__cls_dict__ \"m\" send) (set o (C))"
                       " (send o" +
                       ms + "))");
}

TEST(StateExecNativeNesting, ConstructorThatIsItsOwnInit) {
  expect_nesting_limit("(begin (defclass P) (set-attr P.__cls_dict__ \"init\" P) (P))");
}

// A chain of `n` classes, each the superclass of the next, defining no init of their own, so
// constructing the last runs every inherited init up the chain. Leaves `begin` open.
std::string class_chain(std::size_t n) {
  return "(begin (defclass P) (set i 0)"
         " (while (< i " +
         std::to_string(n) +
         ") (begin (defclass K P) (set P.__cls_dict__ K.__cls_dict__) (set i (+ i 1))))";
}

TEST(StateExecNativeNesting, InheritedInitUpALongChain) {
  on_stack(kThreadStackKiB, [] {
    value_t obj = run_script(class_chain(100) + " (K))");
    EXPECT_TRUE(std::holds_alternative<dict_ptr>(obj.v));
  });
  expect_nesting_limit(class_chain(10000) + " (K))");
}

TEST(StateExecNativeNesting, MethodsCallingMethodsAreFrames) {
  // A method is a closure: send calls it through call_function, so its body runs as frames of
  // the calling evaluator, on the heap, and methods calling methods do not nest natively.
  const std::string cls = "(begin (defclass C (down (self n) (if (= n 0) 0 (+ 1 (send self "
                          "\"down\" (- n 1)))))) (set o (C)) ";
  on_stack(kSmallStackKiB, [&] {
    value_t r = run_script(cls + "(send o \"down\" 10000))");
    EXPECT_EQ(std::get<int64_t>(r.v), 10000);
    EXPECT_EQ(detail::native_nesting::depth(), 0u);
  });
}

TEST(StateExecNativeNesting, GeneratorsOfGenerators) {
  // (mk n) yields from (mk (- n 1)): advancing it advances n nested generators.
  const std::string mk = "(begin (set mk nil) (set mk (lambda (n) (generator (lambda ()"
                         " (if (= n 0) (yield 7) (for x (mk (- n 1)) (yield x))))))) ";
  on_stack(kThreadStackKiB, [&] {
    value_t r = run_script(mk + "(next (mk 50)))");
    EXPECT_EQ(std::get<int64_t>(r.v), 7);
  });
  expect_nesting_limit(mk + "(next (mk 10000)))");
}

TEST(StateExecNativeNesting, FreeingALongClassChain) {
  // Each class holds its superclass's dict inside its inherited init and finalize (native
  // functions, not values): freeing the chain must not take a native frame per class.
  on_stack(kWasmStackKiB, [] {
    stackless_evaluator ev(builtins::make_default_environment());
    ev.evaluate(parse(class_chain(depth() / 10) + ")"));
  });
}

} // namespace
