/// @file state_exec_locks_test.cpp
/// @brief The state_exec lock phases' tests (docs/STATE_EXEC_LOCKS.md, section 13). L0: the
///        atomic natives state-incr, state-cas and state-swap (section 4.4, tests AT-1..AT-13).

#include <charconv>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/scheduler.h>
#include <deque>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

namespace {

// The sanitizer CI jobs (ci.yml, `sanitizers`) set CVC_LOCKS_SANITIZER=1: the heavy cases then take
// their smaller sizes.
bool under_sanitizer() {
  const char *v = std::getenv("CVC_LOCKS_SANITIZER");
  return v && *v && std::string(v) != "0";
}

// A program's closure bound in the scope it captured (a defun, a lambda set at top level) and that
// scope hold each other, and only the cycle collector frees them
// (docs/STATE_EXEC_DEVELOPER_GUIDE.md, "Reference cycles"). These tests step the schedulers by
// hand, so nothing collects; once every world is gone, one collection leaves LeakSanitizer only
// real leaks to report.
class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

// One app and one scheduler of the given kind ("sync" or "async"), with one intrinsics context per
// program. state-watch registers its handler on ctx->pid and takes the watch id from ctx->proc
// (intrinsics.cpp), so programs must not share a context; each gets its own, with its pid set
// before the program's first step. The contexts live as long as the world.
struct locks_world {
  cvc::app app;
  cvc::state &tree;
  std::unique_ptr<scheduler> sync;
  std::unique_ptr<async_scheduler> async;
  std::deque<intrinsics_context> contexts; // stable addresses: the natives point at them

  explicit locks_world(const std::string &kind) : tree(cvc::state::instance(app)) {
    if (kind == "sync")
      sync = std::make_unique<scheduler>();
    else
      async = std::make_unique<async_scheduler>();
    base().set_watch_root(&tree);
  }
  locks_world(const locks_world &) = delete;
  locks_world &operator=(const locks_world &) = delete;

  scheduler_base &base() { return sync ? static_cast<scheduler_base &>(*sync) : *async; }

  // Submit `script` in a default environment of its own, with its own intrinsics context.
  int submit(const std::string &script, execute_options opts = {}) {
    intrinsics_context &ctx = contexts.emplace_back();
    ctx.sched = &base();
    ctx.root = &tree;
    ctx.proc = make_process();
    ctx.uid = "test";
    ctx.cluster_id = "local";
    ctx.node_id = "local";
    environment_ptr env = builtins::make_default_environment();
    register_intrinsics(env, &ctx);
    opts.env = env;
    const int pid = base().execute(script, opts);
    ctx.pid = pid;
    ctx.proc->pid = pid;
    return pid;
  }

  // Step until no process is left to run, or `seconds` pass. True when nothing is left.
  bool drive(double seconds = 30.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (std::chrono::steady_clock::now() < deadline) {
      if (sync) {
        if (!sync->has_runnable())
          return true;
        sync->step();
      } else {
        async->drain_ingress();
        if (!async->has_runnable())
          return true;
        async->sync_step();
      }
    }
    return false;
  }

  // `script` submitted and driven to its end; its result.
  value_t run(const std::string &script) {
    const int pid = submit(script);
    EXPECT_TRUE(drive()) << "did not finish: " << script;
    return result(pid);
  }

  value_t result(int pid) {
    const auto info = base().get_process_info(pid);
    EXPECT_TRUE(info.has_value());
    if (info && info->exit_error)
      ADD_FAILURE() << "pid " << pid << " ended with " << *info->exit_error;
    const std::optional<value_t> r = sync ? sync->get_result(pid) : async->get_result(pid);
    EXPECT_TRUE(r.has_value()) << "pid " << pid << " has no result";
    return r ? *r : nil_value;
  }

  std::string node(const std::string &path) { return tree(path).value(); }
};

int64_t as_int(const value_t &v) {
  const auto *i = std::get_if<int64_t>(&v.v);
  EXPECT_NE(i, nullptr) << "expected an int, got " << to_string(v);
  return i ? *i : 0;
}

double as_float(const value_t &v) {
  const auto *d = std::get_if<double>(&v.v);
  EXPECT_NE(d, nullptr) << "expected a float, got " << to_string(v);
  return d ? *d : 0.0;
}

bool as_bool(const value_t &v) {
  const auto *b = std::get_if<bool>(&v.v);
  if (b)
    return *b;
  EXPECT_TRUE(v.is_nil()) << "expected a boolean, got " << to_string(v);
  return false;
}

std::string as_str(const value_t &v) {
  const auto *s = std::get_if<std::string>(&v.v);
  EXPECT_NE(s, nullptr) << "expected a string, got " << to_string(v);
  return s ? *s : std::string();
}

// The elements of a list result.
std::vector<value_t> as_list(const value_t &v) {
  const auto *l = std::get_if<list_ptr>(&v.v);
  EXPECT_NE(l, nullptr) << "expected a list, got " << to_string(v);
  return l ? std::vector<value_t>((*l)->begin(), (*l)->end()) : std::vector<value_t>{};
}

// Counts a node's valueChanged signals for the life of the counter.
struct change_counter {
  int count = 0;
  boost::signals2::scoped_connection conn;
  explicit change_counter(cvc::state &node)
      : conn(node.valueChanged.connect([this] { ++count; })) {}
};

class Atomics : public ::testing::TestWithParam<std::string> {
protected:
  locks_world w{GetParam()};
};

INSTANTIATE_TEST_SUITE_P(Schedulers, Atomics, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });

} // namespace

// AT-1: an absent node counts as 0; int + int stays an int; a float delta makes a float.
TEST_P(Atomics, AT01_IncrAbsentThenIntThenFloat) {
  const auto r =
      as_list(w.run(R"((list (state-incr "n") (state-incr "n" 5) (state-incr "n" 0.5)))"));
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(as_int(r[0]), 1);
  EXPECT_EQ(as_int(r[1]), 6);
  EXPECT_DOUBLE_EQ(as_float(r[2]), 6.5);
  EXPECT_EQ(w.node("n"), "6.5");
}

// AT-2: a node that is not a number raises ValueError and is left as it was, firing nothing.
TEST_P(Atomics, AT02_IncrNotANumber) {
  w.tree("n").value("abc");
  change_counter changes(w.tree("n"));
  const value_t r = w.run(R"((try (state-incr "n")
                   (catch (ValueError e) (str-concat "caught: " (get-attr e "message")))))");
  EXPECT_EQ(as_str(r).rfind("caught: ", 0), 0u) << to_string(r);
  EXPECT_NE(as_str(r).find("not a number"), std::string::npos) << to_string(r);
  EXPECT_EQ(w.node("n"), "abc");
  EXPECT_EQ(changes.count, 0);
}

// AT-3: a nil expected matches an absent node once; the second claim sees "a".
TEST_P(Atomics, AT03_CasNilTwice) {
  const auto r = as_list(w.run(R"((list (state-cas "k" nil "a") (state-cas "k" nil "a")))"));
  ASSERT_EQ(r.size(), 2u);
  EXPECT_TRUE(as_bool(r[0]));
  EXPECT_FALSE(as_bool(r[1]));
  EXPECT_EQ(w.node("k"), "a");
}

// AT-4: state-swap returns what it replaced.
TEST_P(Atomics, AT04_SwapReturnsOld) {
  w.tree("k").value("a");
  EXPECT_EQ(as_str(w.run(R"((state-swap "k" "b"))")), "a");
  EXPECT_EQ(w.node("k"), "b");
}

// AT-5: through a writable transparent link each writes the link's target; on a read-only node
// each raises NativeError (native_type read_only_error), leaving it unchanged.
TEST_P(Atomics, AT05_ThroughWritableLinkAndReadOnly) {
  w.tree("target.n").value("1");
  w.tree("lnk").linkTo("target.n", cvc::state::link_mode::transparent);
  w.tree("lnk").setLinkWritable(true);
  const auto r = as_list(
      w.run(R"((list (state-incr "lnk") (state-cas "lnk" "2" "7") (state-swap "lnk" "9")))"));
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(as_int(r[0]), 2);
  EXPECT_TRUE(as_bool(r[1]));
  EXPECT_EQ(as_str(r[2]), "7");
  EXPECT_EQ(w.node("target.n"), "9");
  EXPECT_EQ(w.tree("lnk").value(), "");

  w.tree("ro").value("5");
  w.tree("ro").readOnly(true);
  for (const char *call :
       {R"((state-incr "ro"))", R"((state-cas "ro" "5" "6"))", R"((state-swap "ro" "6"))"}) {
    const value_t got = w.run(std::string("(try ") + call +
                              R"( (catch (NativeError e) (get-attr e "native_type"))))");
    EXPECT_NE(as_str(got).find("read_only_error"), std::string::npos)
        << call << ": " << to_string(got);
  }
  EXPECT_EQ(w.node("ro"), "5");
}

// AT-6 (C++): two threads updating one node through updateValue lose no update.
TEST(AtomicsThreads, AT06_TwoThreadsUpdateValueExact) {
  cvc::app app;
  cvc::state &node = cvc::state::instance(app)("count");
  const int per = under_sanitizer() ? 10000 : 1000000;
  const auto bump = [&node, per] {
    for (int i = 0; i < per; ++i)
      node.updateValue([](const std::string &old) {
        return std::to_string((old.empty() ? 0 : std::stoll(old)) + 1);
      });
  };
  std::thread a(bump), b(bump);
  a.join();
  b.join();
  EXPECT_EQ(node.value(), std::to_string(2LL * per));
}

// AT-7: a changing state-cas or state-incr fires the program's watch once and valueChanged once.
TEST_P(Atomics, AT07_WatchAndSignalOnce) {
  change_counter changes(w.tree("w.n"));
  // Each change is followed by a spin until the handler has run (it fires at a step boundary).
  const value_t r = w.run(R"(
      (begin
        (state-watch "w.n" (lambda (path val) (state-incr "w.fired")))
        (defun wait-fired (k)
          (begin (set tries 0)
                 (while (and (< (if (state-exists "w.fired") (int (state-get "w.fired")) 0) k)
                             (< tries 100000))
                   (set tries (+ tries 1)))))
        (state-cas "w.n" nil "0")
        (wait-fired 1)
        (state-incr "w.n")
        (wait-fired 2)
        (set tries 0)
        (while (< tries 2000) (set tries (+ tries 1)))
        (state-get "w.fired")))");
  EXPECT_EQ(as_str(r), "2");
  EXPECT_EQ(changes.count, 2);
  EXPECT_EQ(w.node("w.n"), "1");
}

// AT-8: a compare-and-set or swap whose new value equals the old one succeeds and fires nothing.
TEST_P(Atomics, AT08_EqualNewFiresNothing) {
  w.tree("p").value("x");
  change_counter changes(w.tree("p"));
  const value_t r = w.run(R"(
      (begin
        (state-watch "p" (lambda (path val) (state-incr "p-fired")))
        (set ok (list (state-cas "p" "x" "x") (state-swap "p" "x")))
        (set tries 0)
        (while (< tries 3000) (set tries (+ tries 1)))
        (append ok (state-exists "p-fired"))
        ok))");
  const auto l = as_list(r);
  ASSERT_EQ(l.size(), 3u);
  EXPECT_TRUE(as_bool(l[0]));
  EXPECT_EQ(as_str(l[1]), "x");
  EXPECT_FALSE(as_bool(l[2]));
  EXPECT_EQ(changes.count, 0);
}

// AT-9: swapping into a node never set returns nil.
TEST_P(Atomics, AT09_SwapNeverSet) {
  EXPECT_TRUE(w.run(R"((state-swap "fresh" "v"))").is_nil());
  EXPECT_EQ(w.node("fresh"), "v");
}

// AT-10: claim with nil, release by setting nil back, claim again.
TEST_P(Atomics, AT10_ClaimReleaseClaim) {
  const auto r = as_list(w.run(R"(
      (list (state-cas "jobs.j7.owner" nil "me")
            (state-cas "jobs.j7.owner" "me" nil)
            (state-cas "jobs.j7.owner" nil "you")))"));
  ASSERT_EQ(r.size(), 3u);
  for (const value_t &v : r)
    EXPECT_TRUE(as_bool(v));
  EXPECT_EQ(w.node("jobs.j7.owner"), "you");
}

// AT-11: a parent made by creating its child was never set; a nil expected matches it.
TEST_P(Atomics, AT11_CasParentNeverSet) {
  w.tree("a.b").value("child");
  EXPECT_TRUE(as_bool(w.run(R"((state-cas "a" nil "x"))")));
  EXPECT_EQ(w.node("a"), "x");
}

// AT-12: a transparent link that is not writable refuses all three, changing neither node.
TEST_P(Atomics, AT12_NonWritableLink) {
  w.tree("t.v").value("1");
  w.tree("ro-link").linkTo("t.v", cvc::state::link_mode::transparent);
  ASSERT_FALSE(w.tree("ro-link").linkWritable());
  for (const char *call : {R"((state-incr "ro-link"))", R"((state-cas "ro-link" "1" "2"))",
                           R"((state-swap "ro-link" "2"))"}) {
    const value_t got = w.run(std::string("(try ") + call +
                              R"( (catch (NativeError e) (get-attr e "native_type"))))");
    EXPECT_NE(as_str(got).find("read_only_error"), std::string::npos)
        << call << ": " << to_string(got);
  }
  EXPECT_EQ(w.node("t.v"), "1");
  EXPECT_EQ(w.tree("ro-link").value(), "");
}

// AT-13: a float sum is stored in its shortest round-trip form, not six digits.
TEST_P(Atomics, AT13_IncrShortestFloat) {
  w.tree("n").value("1000000");
  EXPECT_DOUBLE_EQ(as_float(w.run(R"((state-incr "n" 0.5))")), 1000000.5);
  EXPECT_EQ(w.node("n"), "1000000.5");
}

// Errors the design names besides AT's: a compound value is a TypeError; an int overflow is an
// ArithmeticError that leaves the node; a non-number delta is a ValueError.
TEST_P(Atomics, ArgumentErrors) {
  const auto caught = [&](const std::string &call, const std::string &cls) {
    return as_str(w.run("(try " + call + " (catch (" + cls + " e) \"" + cls + "\"))"));
  };
  EXPECT_EQ(caught(R"((state-cas "c" nil (list 1 2)))", "TypeError"), "TypeError");
  EXPECT_EQ(caught(R"((state-swap "c" (dict "a" 1)))", "TypeError"), "TypeError");
  EXPECT_EQ(caught(R"((state-incr "c" "1"))", "ValueError"), "ValueError");
  w.tree("big").value(std::to_string(std::numeric_limits<int64_t>::max()));
  EXPECT_EQ(caught(R"((state-incr "big"))", "ArithmeticError"), "ArithmeticError");
  EXPECT_EQ(w.node("big"), std::to_string(std::numeric_limits<int64_t>::max()));
  EXPECT_FALSE(w.tree("c").isLink());
}

// The DSL's own number text: what state-incr reads is what the parser reads.
TEST_P(Atomics, IncrReadsDslNumberText) {
  w.tree("f").value("2.5");
  w.tree("e").value("1e3");
  w.tree("neg").value("-4");
  w.tree("sp").value(" 4");
  EXPECT_DOUBLE_EQ(as_float(w.run(R"((state-incr "f"))")), 3.5);
  EXPECT_DOUBLE_EQ(as_float(w.run(R"((state-incr "e"))")), 1001.0);
  EXPECT_EQ(w.node("e"), "1001");
  EXPECT_EQ(as_int(w.run(R"((state-incr "neg" 4))")), 0);
  EXPECT_EQ(as_str(w.run(R"((try (state-incr "sp") (catch (ValueError e) "no")))")), "no");
}

// detail::format_number_shortest_fallback writes std::to_chars's text, on every platform.
TEST(AtomicsFormat, FormatShortestFallback) {
  struct row {
    double d;
    const char *text; // std::to_chars(double), as MSVC writes it
  };
  const row rows[] = {
      {0.1, "0.1"},
      {0.0001, "1e-04"},
      {1e-05, "1e-05"},
      {1e21, "1e+21"},
      {1e22, "1e+22"},
      {5e-324, "5e-324"},
      {2.2250738585072014e-308, "2.2250738585072014e-308"},
      {1.7976931348623157e308, "1.7976931348623157e+308"},
      {-0.0, "-0"},
      {0.0, "0"},
      {1000000.5, "1000000.5"},
      {1.0 / 3, "0.3333333333333333"},
      {9007199254740994.0, "9007199254740994"},
      {123456789012345680.0, "123456789012345680"},
      {100.0, "100"},
      {1e5, "1e+05"},
      {6.5, "6.5"},
      {-2.5e-7, "-2.5e-07"},
      {1e100, "1e+100"},
      {123.456, "123.456"},
      {1152921504606846976.0, "1152921504606846976"}, // 2^60: fixed, the exact digits
  };
  for (const row &r : rows) {
    const std::string got = detail::format_number_shortest_fallback(r.d);
    EXPECT_EQ(got, r.text) << "for " << r.text;
    if (got != "-0" && got != "0")
      EXPECT_EQ(std::strtod(got.c_str(), nullptr), r.d) << got;
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
    char buf[64];
    const auto tc = std::to_chars(buf, buf + sizeof(buf), r.d);
    EXPECT_EQ(got, std::string(buf, tc.ptr)) << "for " << r.text;
#endif
  }
#if defined(__cpp_lib_to_chars) && __cpp_lib_to_chars >= 201611L
  // Against to_chars itself on values spread over the whole range, powers of two (lopsided
  // rounding intervals) included.
  uint64_t x = 0x9E3779B97F4A7C15ull;
  for (int i = 0; i < 20000; ++i) {
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    double d;
    if (i % 4 == 0)
      d = std::ldexp(1.0, static_cast<int>(x % 2098) - 1074); // a power of two
    else {
      std::memcpy(&d, &x, sizeof d);
      if (!std::isfinite(d))
        continue;
    }
    char buf[64];
    const auto tc = std::to_chars(buf, buf + sizeof(buf), d);
    ASSERT_EQ(detail::format_number_shortest_fallback(d), std::string(buf, tc.ptr));
  }
#endif
}
