/// @file state_exec_locks_test.cpp
/// @brief The state_exec lock phases' tests (docs/STATE_EXEC_LOCKS.md, section 13). L0: the
///        atomic natives state-incr, state-cas and state-swap (section 4.4, tests AT-1..AT-13).
///        L1: with-lock on one scheduler (sections 13.2-13.5: LK, LX, LW, LD), the lock home
///        over a fake scheduler (LockHome), and the random-operations stress (LocksStress).
///        L3: several schedulers' homes on one shared manager, on threads of their own
///        (LocksThreads; LT, LD-6, PI-7, PI-10 and the posted LX cases), and the cross-thread
///        stress at full size (LocksStress).

#include <algorithm>
#include <atomic>
#include <barrier>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/detail/lock_table.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/lock_manager.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/process_table.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <deque>
#include <functional>
#include <future>
#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
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

  // Step until `done()` holds or `seconds` pass, sleeping briefly whenever nothing stepped (a lock
  // deadline or a sleeper's comes due meanwhile). True when done() held.
  template <class Pred> bool step_until(Pred done, double seconds = 10.0) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!done()) {
      if (std::chrono::steady_clock::now() >= deadline)
        return false;
      int stepped;
      if (sync) {
        stepped = sync->step();
      } else {
        async->drain_ingress();
        stepped = async->sync_step();
      }
      if (!stepped)
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    return true;
  }

  process_status status(int pid) { return base().get_process_info(pid)->status; }
  bool finished(int pid) { return is_finished(status(pid)); }
  bool parked(int pid) { return status(pid) == process_status::waiting; }

  // `script` submitted and driven to its end; its result.
  value_t run(const std::string &script) {
    const int pid = submit(script);
    EXPECT_TRUE(step_until([&] { return finished(pid); }, 30.0)) << "did not finish: " << script;
    return result(pid);
  }

  value_t result(int pid) {
    const auto info = base().get_process_info(pid);
    EXPECT_TRUE(info.has_value());
    if (info && info->exit_error)
      ADD_FAILURE() << "pid " << pid << " ended with " << *info->exit_error;
    const std::optional<value_t> r = sync ? sync->get_result(pid) : async->get_result(pid);
    EXPECT_TRUE(r.has_value()) << "pid " << pid << " has no result";
    return r ? *r : nil_value();
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

// ===========================================================================
// with-lock end to end (docs/STATE_EXEC_LOCKS.md 4.1): a first pass over the form on both
// schedulers; the LK/LX/LW/LD suites follow the design's section 13 in full.
// ===========================================================================

namespace {
class LocksSmoke : public ::testing::TestWithParam<std::string> {
protected:
  locks_world w{GetParam()};
};
INSTANTIATE_TEST_SUITE_P(Schedulers, LocksSmoke, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });
} // namespace

TEST_P(LocksSmoke, ValueAndLocksHeld) {
  const auto r = as_list(w.run(R"((list (with-lock "k" (list (locks-held) 7)) (locks-held)))"));
  ASSERT_EQ(r.size(), 2u);
  const auto inside = as_list(r[0]);
  ASSERT_EQ(inside.size(), 2u);
  EXPECT_EQ(as_int(inside[1]), 7);
  const auto held = as_list(inside[0]);
  ASSERT_EQ(held.size(), 1u);
  EXPECT_EQ(to_string(held[0]),
            R"({"key": "k", "mode": "exclusive", "count": 1, "generation": 1})");
  EXPECT_TRUE(as_list(r[1]).empty());
}

TEST_P(LocksSmoke, WaiterParksAndIsGrantedOnRelease) {
  const int holder = w.submit(R"((with-lock "k" (begin (msg-recv "go") 1)))");
  ASSERT_TRUE(w.step_until([&] { return w.parked(holder); }));
  const int waiter = w.submit(R"((with-lock "k" 42))");
  ASSERT_TRUE(w.step_until([&] { return w.parked(waiter); }));
  const auto info = w.base().get_process_info(waiter);
  ASSERT_TRUE(info->waiting_on.has_value());
  EXPECT_EQ(*info->waiting_on, "lock /k");
  const int sender = w.submit(R"((msg-send "go" "x"))");
  ASSERT_TRUE(w.step_until([&] { return w.finished(holder) && w.finished(waiter); }));
  EXPECT_EQ(as_int(w.result(holder)), 1);
  EXPECT_EQ(as_int(w.result(waiter)), 42);
  (void)sender;
  const value_t q = w.run(R"((lock-query))");
  EXPECT_EQ(to_string(q), R"({"held": (), "waiting": ()})");
}

TEST_P(LocksSmoke, DeadlockIsRaisedInTheCloser) {
  const int a = w.submit(R"((with-lock "a" (begin (msg-recv "a1") (with-lock "b" 1))))");
  const int b = w.submit(R"((with-lock "b" (begin (msg-recv "b1")
                              (try (with-lock "a" 2) (catch (DeadlockError e) "deadlock")))))");
  ASSERT_TRUE(w.step_until([&] { return w.parked(a) && w.parked(b); }));
  w.submit(R"((msg-send "a1" "x"))");
  ASSERT_TRUE(w.step_until([&] {
    return w.parked(a) &&
           w.base().get_process_info(a)->waiting_on == std::optional<std::string>("lock /b");
  }));
  w.submit(R"((msg-send "b1" "x"))");
  ASSERT_TRUE(w.step_until([&] { return w.finished(a) && w.finished(b); }));
  EXPECT_EQ(as_str(w.result(b)), "deadlock");
  EXPECT_EQ(as_int(w.result(a)), 1);
}

TEST_P(LocksSmoke, TimeoutAndIfAvailable) {
  const int holder = w.submit(R"((with-lock "k" (msg-recv "go")))");
  ASSERT_TRUE(w.step_until([&] { return w.parked(holder); }));
  const auto t0 = std::chrono::steady_clock::now();
  const int waiter =
      w.submit(R"((try (with-lock "k" :timeout 0.2 1) (catch (TimeoutError e) "timed out")))");
  ASSERT_TRUE(w.step_until([&] { return w.finished(waiter); }));
  const double took = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("timeout after %.3f s\n", took);
  EXPECT_GE(took, 0.2);
  EXPECT_EQ(as_str(w.result(waiter)), "timed out");
  EXPECT_EQ(as_str(w.run(R"((with-lock "k" :if-available :else "busy" 1))")), "busy");
  EXPECT_EQ(as_str(w.run(R"((try (with-lock "k" :timeout 0 1) (catch (TimeoutError e) "now")))")),
            "now");
  w.submit(R"((msg-send "go" "x"))");
  ASSERT_TRUE(w.step_until([&] { return w.finished(holder); }));
  EXPECT_EQ(as_int(w.run(R"((with-lock "k" :if-available :else "busy" 1))")), 1);
}

TEST_P(LocksSmoke, OptionErrorsAndCovered) {
  for (const char *form :
       {R"((with-lock "k" :bogus 1))", R"((with-lock "k" :timeout 1 :timeout 2 1))",
        R"((with-lock "k" :shared :exclusive 1))", R"((with-lock "k" :else 2 1))",
        R"((with-lock "k" :timeout))", R"((with-lock "k" :as 3 1))"}) {
    EXPECT_EQ(as_str(w.run(std::string("(try ") + form + R"( (catch (ValueError e) "bad")))")),
              "bad")
        << form;
  }
  // A nested request under an own X hold is covered: one hold, count 2.
  const auto r = as_list(w.run(R"((with-lock "a" (with-lock "a.b" :shared (locks-held))))"));
  ASSERT_EQ(r.size(), 1u);
  EXPECT_NE(to_string(r[0]).find(R"("count": 2)"), std::string::npos) << to_string(r[0]);
  // Upgrade refused.
  EXPECT_EQ(as_str(w.run(R"((with-lock "a" :shared
                               (try (with-lock "a.b" 1) (catch (LockError e) (get-attr e "reason")))))")),
            "upgrade");
  // :as: the canonical keys; the second X grant of this world (an S grant takes the last X's).
  EXPECT_EQ(to_string(w.run(R"((with-lock (list "b" "a" "a.x") :as g g))")),
            R"({"key": ("a" "b"), "mode": "exclusive", "generation": 2, "abandoned": #f})");
}

// ===========================================================================
// ===========================================================================
// What the L1 reviews found (R1a, R1b, R1d): restored lock frames, a restored wait under the
// restoring context's rules, the order KEY and :timeout are read in, a '/' key on a cache hit,
// and a cache eviction for a request.
// ===========================================================================

namespace {
class LocksReview : public ::testing::TestWithParam<std::string> {
protected:
  locks_world w{GetParam()};

  // An environment of its own over a context of its own (kept by the world), `setup` applied to
  // the context before the intrinsics are registered.
  environment_ptr make_env(const std::function<void(intrinsics_context &)> &setup = {}) {
    intrinsics_context &ctx = w.contexts.emplace_back();
    ctx.sched = &w.base();
    ctx.root = &w.tree;
    ctx.proc = make_process();
    ctx.uid = "test";
    if (setup)
      setup(ctx);
    environment_ptr env = builtins::make_default_environment();
    register_intrinsics(env, &ctx);
    return env;
  }

  int submit(const std::string &script, const environment_ptr &env) {
    execute_options opts;
    opts.env = env;
    return w.base().execute(script, opts);
  }

  // Snapshot `pid`, kill it, and restore the snapshot with `natives`: the copy's pid.
  int snapshot_and_restore(int pid, const environment_ptr &natives) {
    cvc::state &node = w.tree("snap.p" + std::to_string(pid));
    EXPECT_TRUE(w.sync ? w.sync->snapshot(pid, node) : w.async->snapshot(pid, node));
    w.base().kill(pid);
    return w.sync ? w.sync->restore(node, natives) : w.async->restore(node, natives);
  }
};
INSTANTIATE_TEST_SUITE_P(Schedulers, LocksReview, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });
} // namespace

// R1b: a lock frame restored while KEY is still being evaluated reads its lock form from the
// restoring environment (the codec writes the slot as nil).
TEST_P(LocksReview, RestoredWhileKeyEvaluates) {
  const environment_ptr env = make_env();
  const int p = submit(R"S((with-lock (begin (sleep 0.05) "k") 42))S", env);
  ASSERT_TRUE(w.step_until([&] { return w.parked(p); }));
  const int q = snapshot_and_restore(p, env);
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }));
  EXPECT_EQ(as_int(w.result(q)), 42);
}

// R1b: a lock frame restored with a failed outcome raises it at the form.
TEST_P(LocksReview, RestoredWithAFailedOutcome) {
  const int h = w.submit(R"S((with-lock "k" (msg-recv "never")))S");
  ASSERT_TRUE(w.step_until([&] { return w.parked(h); }));
  const environment_ptr env = make_env();
  const int p = submit(
      R"S((try (with-lock "k" :timeout 0.05 1) (catch (TimeoutError e) "timed out")))S", env);
  ASSERT_TRUE(w.step_until([&] { return w.parked(p); }));
  ASSERT_TRUE(w.step_until([&] { return !w.parked(p); })); // its deadline passed: the outcome
  ASSERT_FALSE(w.finished(p));
  const int q = snapshot_and_restore(p, env);
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }));
  EXPECT_EQ(as_str(w.result(q)), "timed out");
  w.base().kill(h);
}

// R1d: a restored wait follows the restoring context's rules. Where no wait is allowed (as on
// Ariadne's init lane), the request is tried once and refused, not queued again.
TEST_P(LocksReview, RestoredWaitFollowsTheRestoringContext) {
  const int h = w.submit(R"S((with-lock "k" (msg-recv "never")))S");
  ASSERT_TRUE(w.step_until([&] { return w.parked(h); }));
  const environment_ptr env = make_env();
  const int p = submit(R"S((try (with-lock "k" 1) (catch (TimeoutError e) "refused")))S", env);
  ASSERT_TRUE(w.step_until([&] { return w.parked(p); }));
  const environment_ptr strict =
      make_env([](intrinsics_context &c) { c.lock_wait_allowed = false; });
  const int q = snapshot_and_restore(p, strict);
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }, 5.0));
  EXPECT_EQ(as_str(w.result(q)), "refused");
  w.base().kill(h);
}

// td B28: (fork) inside a step forks the running process. The parent's call returns the child's
// pid and the child's returns 0; the child goes on after the call, so it does not fork again.
TEST_P(LocksReview, ForkInAStepReturnsZeroInTheChild) {
  const int p = w.submit(R"S((begin (set c (fork)) (if (= c 0) "child" c)))S");
  ASSERT_TRUE(w.step_until([&] { return w.finished(p); }));
  const int child = static_cast<int>(as_int(w.result(p)));
  ASSERT_TRUE(w.step_until([&] { return w.finished(child); }));
  EXPECT_EQ(as_str(w.result(child)), "child");
  EXPECT_EQ(w.base().list_processes().size(), 2u); // no runaway chain of forks
}

// R1b: KEY is evaluated before :timeout's symbol is looked up (left to right).
TEST_P(LocksReview, TimeoutIsReadAfterKey) {
  EXPECT_EQ(as_int(w.run(R"S((with-lock (begin (set tmo 5) "k") :timeout tmo 42))S")), 42);
}

// R1a: the key cache is keyed by chroot and text, not by channel policy, so a '/' key found in
// it is still checked against the requesting context's policy (gap G-15).
TEST_P(LocksReview, GlobalKeyIsCheckedOnACacheHit) {
  EXPECT_EQ(as_int(w.run(R"S((with-lock "/g" 1))S")), 1); // no policy: allowed, and cached
  channel_policy strict;
  strict.enforce = true; // declares no global
  const environment_ptr env = make_env([&](intrinsics_context &c) { c.channels = &strict; });
  const int p = submit(R"S((try (with-lock "/g" 1) (catch (PermissionError e) "denied")))S", env);
  ASSERT_TRUE(w.step_until([&] { return w.finished(p); }));
  EXPECT_EQ(as_str(w.result(p)), "denied");
}

// R1a: a miss's eviction never names a key of its own request (protocol 4.15), even through
// another spelling: "x" and "x " name one node. A cache of two entries and a manager that checks
// every operation, which reports such a drop as a violation.
TEST_P(LocksReview, EvictionNeverDropsAKeyOfTheRequest) {
  // The manager's hooks (static: the manager lives on in the world after the test's body).
  static const detail::lock_test_hooks hooks{.home_cache_capacity = 2};
  lock_manager::options o;
  o.self_check = true;
  const auto mgr = std::make_shared<lock_manager>(o);
  mgr->set_test_hooks(&hooks); // before the home attaches: it reads the capacity then
  if (w.sync)
    w.sync->set_lock_manager(mgr);
  else
    w.async->set_lock_manager(mgr);
  EXPECT_EQ(as_int(w.run(R"S((with-lock "x" 1))S")), 1);
  EXPECT_EQ(as_int(w.run(R"S((with-lock "x " 2))S")), 2);           // a second entry, the same node
  EXPECT_EQ(as_int(w.run(R"S((with-lock (list "x" "c") 3))S")), 3); // c's miss may evict neither
  EXPECT_NO_THROW(mgr->check());
}

// ===========================================================================
// Phase L1: with-lock on one scheduler (docs/STATE_EXEC_LOCKS.md 13.2-13.5), on the checked
// driver of the locks plan (L1.md section 3): one step at a time, every invariant checked after
// each step.
// ===========================================================================

namespace {

using steady = std::chrono::steady_clock;

void set_env_var(const char *name, const char *value) {
#ifdef _WIN32
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif
}

// Every manager operation checks the manager's invariants when CVC_STATE_EXEC_CHECK=1 (gap G-4),
// which the library reads once, when the first manager is made. Set during static
// initialization, so it precedes every manager this executable makes.
[[maybe_unused]] const bool g_self_check_forced = [] {
  set_env_var("CVC_STATE_EXEC_CHECK", "1");
  return true;
}();

double seconds_since(steady::time_point t0) {
  return std::chrono::duration<double>(steady::now() - t0).count();
}

steady::time_point deadline_after(double seconds) {
  return steady::now() +
         std::chrono::duration_cast<steady::duration>(std::chrono::duration<double>(seconds));
}

// A dict's entry, or nil (with a failure) when it has none.
value_t entry_of(const value_t &d, const std::string &key) {
  const auto *dp = std::get_if<dict_ptr>(&d.v);
  if (dp && *dp)
    for (const auto &[k, v] : **dp)
      if (k == key)
        return v;
  ADD_FAILURE() << "no '" << key << "' in " << to_string(d);
  return nil_value();
}

// A string value's text, any other value's display.
std::string text_of(const value_t &v) {
  if (const auto *s = std::get_if<std::string>(&v.v))
    return *s;
  return to_string(v);
}

// The exception's class name and its "reason" field, from a dsl_exception.
std::string reason_of(const dsl_exception &e) {
  const auto *dp = std::get_if<dict_ptr>(&e.value().v);
  if (dp && *dp)
    for (const auto &[k, v] : **dp)
      if (k == "reason")
        return text_of(v);
  return "";
}

// The lock suites' test hooks (detail::lock_test_hooks) are plain function pointers: what they
// act on lives here. Installed only on a manager (lock_manager::set_test_hooks: hooks_scope's),
// and removed by every test that installs them.
struct hook_state {
  int throw_in_enter_pid = -1;     // in_step_lock_enter throws std::runtime_error for this pid
  int interrupt_in_enter_pid = -1; // ... throws evaluation_interrupted (an internal step error)
  int deadline_in_enter_pid = -1;  // ... runs past the step's deadline, then checks it
  int throw_after_grant_pid = -1;  // after_grant throws for this pid
  int leak_pid = -1;               // before_terminate_end takes a hold "leak" for this pid
  detail::lock_home *home = nullptr;
  int after_grant_calls = 0;
};
hook_state g_hook;

void hook_in_step_lock_enter(int pid) {
  if (pid == g_hook.throw_in_enter_pid)
    throw std::runtime_error("injected into the lock_enter step");
  if (pid == g_hook.interrupt_in_enter_pid)
    throw evaluation_interrupted("injected: an internal step error");
  if (pid == g_hook.deadline_in_enter_pid) {
    // Busy (a time budget charges CPU time once half spent), until the step's deadline passes.
    const auto until = steady::now() + std::chrono::seconds(5);
    while (!eval_deadline_expired() && steady::now() < until) {
    }
    eval_deadline_check("the lock_enter step");
  }
}

void hook_after_grant(int pid) {
  ++g_hook.after_grant_calls;
  if (pid == g_hook.throw_after_grant_pid)
    throw std::runtime_error("injected after the grant");
}

void hook_before_terminate_end(process &p) {
  if (p.pid != g_hook.leak_pid || !g_hook.home)
    return;
  // A hold left at termination (gap G-6: not reachable from the DSL): taken here, as a handler's
  // frames would have left it.
  intrinsics_context ctx;
  g_hook.home->begin_request();
  // The request views its key's text, so the key outlives it.
  const value_t key(std::string("leak"));
  detail::home_request req;
  detail::build_request(req, *g_hook.home, ctx, key, lock_mode::exclusive);
  g_hook.home->try_enter(p, req, "");
}

// A manager with a set of hooks installed for its scope, for the worlds made in it to attach to
// (checked_world(kind, hooks.mgr)): the hooks are that manager's alone. Every test that installs
// hooks resets them on the way out.
struct hooks_scope {
  detail::lock_test_hooks hooks;
  std::shared_ptr<lock_manager> mgr = std::make_shared<lock_manager>();
  explicit hooks_scope(std::size_t cache_capacity = 0) {
    g_hook = hook_state{};
    hooks.in_step_lock_enter = hook_in_step_lock_enter;
    hooks.after_grant = hook_after_grant;
    hooks.before_terminate_end = hook_before_terminate_end;
    hooks.home_cache_capacity = cache_capacity;
    mgr->set_test_hooks(&hooks);
  }
  ~hooks_scope() {
    mgr->set_test_hooks(nullptr);
    g_hook = hook_state{};
  }
  hooks_scope(const hooks_scope &) = delete;
  hooks_scope &operator=(const hooks_scope &) = delete;
};

// How checked_world submits a program. At namespace scope, not nested: GCC refuses a `= {}`
// default argument of a nested class with member initializers inside the class that encloses it.
struct checked_submit_options {
  execute_options opts;
  std::string chroot;
  const channel_policy *policy = nullptr;
  std::optional<double> lock_default_timeout;
  bool lock_wait_allowed = true;
  std::optional<int> ctx_pid; // what the context's pid says (a host's placeholder); else the pid
  bool spawn = false;         // the context installs make_spawn_context (a child gets the natives)
};

// What check_all() checks after each round (L3: a home among several).
enum class world_checks : std::uint8_t {
  all,        // the manager, the home's quiescent clauses, the process table (one home: L1, L2)
  sequenced,  // the same, the quiescent clauses only while the home's port is empty: a home
              // among several whose actions run one at a time in an order the test fixes, so
              // every delivery is done when an action returns and only a post is in flight
  concurrent, // the manager and the process table: a home stepping while another home's thread
              // acts, whose delivery may be under way (the quiescent clauses need every home
              // delivered and drained, protocol 6)
  stepping,   // the process table only: LT-4's and LT-10's drivers, where CVC_STATE_EXEC_CHECK=1
              // checks the manager after every operation, and the quiescent clauses wait for
              // the stops
};

// How checked_world is made (L3). At namespace scope, as checked_submit_options is.
struct checked_world_options {
  std::shared_ptr<lock_manager> mgr; // the manager to attach to; else a private one
  lock_attach attach;                // cross_thread: a home pumped on a thread of its own
  scheduling_policy policy = scheduling_policy::round_robin;
  cvc::state *tree = nullptr; // the programs' tree, shared with other worlds; else the world's own
  world_checks checks = world_checks::all;
};

// One app and one scheduler of the given kind, its lock home made at once (on a private manager
// unless one is given), one intrinsics context per program, and the checked driver.
struct checked_world {
  using submit_options = checked_submit_options;

  std::string kind;
  std::unique_ptr<cvc::app> app; // the world's own, unless its options give a tree
  cvc::state &tree;
  std::unique_ptr<scheduler> sync;
  std::unique_ptr<async_scheduler> async;
  std::shared_ptr<lock_manager> mgr;
  std::deque<intrinsics_context> contexts; // stable addresses: the natives point at them
  std::deque<channel_policy> policies;
  std::unordered_map<int, environment_ptr> envs;
  std::unordered_map<int, std::string> channels; // a locker's release channel (resolved)
  std::vector<std::string> trace;                // (mark x) appends x
  int next_channel = 0;
  bool broken = false; // a check failed: the drivers stop
  std::uint64_t rounds = 0;
  world_checks checks = world_checks::all;
  std::function<void(const std::string &)> on_hook; // (hook x) calls it with x, in the step

  explicit checked_world(const std::string &k, std::shared_ptr<lock_manager> m = nullptr)
      : checked_world(k, checked_world_options{std::move(m)}) {}

  checked_world(const std::string &k, const checked_world_options &o)
      : kind(k), tree(tree_of(app, o.tree)), checks(o.checks) {
    if (kind == "sync")
      sync = std::make_unique<scheduler>(o.policy);
    else
      async = std::make_unique<async_scheduler>(o.policy);
    base().set_watch_root(&tree);
    if (o.mgr) {
      if (sync)
        sync->set_lock_manager(o.mgr, o.attach);
      else
        async->set_lock_manager(o.mgr, o.attach);
    }
    mgr = base().locks()->manager_ptr();
    EXPECT_TRUE(mgr->table().opts().self_check)
        << "CVC_STATE_EXEC_CHECK=1 did not reach the library";
  }
  static cvc::state &tree_of(std::unique_ptr<cvc::app> &app, cvc::state *given) {
    if (given)
      return *given;
    app = std::make_unique<cvc::app>();
    return cvc::state::instance(*app);
  }
  checked_world(const checked_world &) = delete;
  checked_world &operator=(const checked_world &) = delete;

  scheduler_base &base() { return sync ? static_cast<scheduler_base &>(*sync) : *async; }
  detail::lock_home &home() { return *base().locks(); }
  lock_stats stats() const { return mgr->stats(); }

  // The test's own natives, bound in every program's environment.
  void register_test_natives(const environment_ptr &env) {
    builtins::register_fn(env, "mark", [this](std::span<const value_t> a) -> value_t {
      trace.push_back(a.empty() ? std::string() : text_of(a[0]));
      return nil_value();
    });
    // The test's own code, inside the step of the process calling it (L3: another home's action
    // placed at an exact point of this one's).
    builtins::register_fn(env, "hook", [this](std::span<const value_t> a) -> value_t {
      if (on_hook)
        on_hook(a.empty() ? std::string() : text_of(a[0]));
      return nil_value();
    });
    // A nested evaluator, inside the step of the process calling it (design 5.5).
    std::weak_ptr<environment> weak = env;
    builtins::register_fn(env, "nested-eval", [weak](std::span<const value_t> a) -> value_t {
      environment_ptr e = weak.lock();
      stackless_evaluator ev(e);
      return ev.evaluate_script(a.empty() ? std::string() : text_of(a[0]));
    });
    // An internal step error: not a program's error (async_scheduler kills the process).
    builtins::register_fn(env, "boom", [](std::span<const value_t>) -> value_t {
      throw evaluation_interrupted("boom: an internal step error");
    });
    // One long step: runs until the step's deadline cuts it (a scheduler's max_time or
    // max_step_time), as a long native loop would (types.h: eval_deadline_check).
    builtins::register_fn(env, "spin-step", [](std::span<const value_t>) -> value_t {
      const auto until = steady::now() + std::chrono::seconds(5);
      while (steady::now() < until)
        eval_deadline_check("spin-step");
      return nil_value();
    });
  }

  // A program's environment: the intrinsics over a context of its own, and the test's natives.
  environment_ptr make_env(submit_options &so, intrinsics_context *&out) {
    intrinsics_context &ctx = contexts.emplace_back();
    ctx.sched = &base();
    ctx.root = &tree;
    if (!so.chroot.empty()) {
      apply_chroot(ctx, tree, so.chroot);
      so.opts.root_path = so.chroot;
    }
    ctx.proc = make_process();
    ctx.uid = "test";
    ctx.cluster_id = "local";
    ctx.node_id = "local";
    ctx.channels = so.policy;
    ctx.lock_default_timeout = so.lock_default_timeout;
    ctx.lock_wait_allowed = so.lock_wait_allowed;
    if (so.spawn)
      ctx.spawn_context = make_spawn_context();
    environment_ptr env = builtins::make_default_environment();
    register_intrinsics(env, &ctx);
    register_test_natives(env);
    out = &ctx;
    return env;
  }

  // The natives a restore rebinds from: an environment of this world's, running nothing.
  environment_ptr natives(submit_options so = {}) {
    intrinsics_context *ctx = nullptr;
    return make_env(so, ctx);
  }

  int submit(const std::string &script, submit_options so = {}) {
    intrinsics_context *ctx = nullptr;
    environment_ptr env = make_env(so, ctx);
    so.opts.env = env;
    const int pid = base().execute(script, so.opts);
    ctx->pid = so.ctx_pid ? *so.ctx_pid : pid;
    ctx->proc->pid = pid;
    envs[pid] = env;
    return pid;
  }

  // -- the checked driver (L1.md section 3) --------------------------------------------------

  // The manager's invariants, the home's quiescent clauses and the process table's indexes, as
  // `checks` says. A violation is a failure, and stops the drivers.
  bool check_all() {
    if (broken)
      return false;
    try {
      if (checks != world_checks::stepping)
        mgr->check();
      if (checks == world_checks::all ||
          (checks == world_checks::sequenced && !home().port_pending()))
        home().check_quiescent();
    } catch (const detail::lock_violation &v) {
      broken = true;
      ADD_FAILURE() << "after round " << rounds << ": " << v.what();
      return false;
    }
    if (!base().check_tables()) {
      broken = true;
      ADD_FAILURE() << "after round " << rounds << ": the process table's check failed";
      return false;
    }
    return true;
  }

  // One round: one step, then check_all(). The async scheduler is driven as pycvc's run() drives
  // it (drain_ingress, sync_step, wait_for_work when nothing stepped), the sync one by step()
  // (whose step_once services the timers), sleeping 0.2 ms when nothing stepped. True when a
  // process stepped.
  bool round(steady::time_point until, bool wait = true) {
    ++rounds;
    int stepped;
    if (sync) {
      stepped = sync->step();
      if (!stepped && wait)
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    } else {
      async->drain_ingress();
      stepped = async->sync_step();
      if (!stepped && wait)
        async->wait_for_work(std::min(until, steady::now() + std::chrono::milliseconds(10)));
    }
    check_all();
    return stepped != 0;
  }

  // Rounds until `done()` holds (true), or `seconds` pass or a check fails (false).
  template <class Pred> bool step_until(Pred done, double seconds = 10.0) {
    const steady::time_point until = deadline_after(seconds);
    while (!done()) {
      if (broken || steady::now() >= until)
        return false;
      round(until);
    }
    return true;
  }

  // Rounds until one steps nothing (every process waits or has ended), without waiting for a
  // deadline. True when it settled.
  bool settle(double seconds = 10.0) {
    const steady::time_point until = deadline_after(seconds);
    while (round(until, false))
      if (broken || steady::now() >= until)
        return false;
    return !broken;
  }

  // -- processes --------------------------------------------------------------------------

  std::optional<process_info> info(int pid) { return base().get_process_info(pid); }
  process_status status(int pid) { return info(pid)->status; }
  bool finished(int pid) { return is_finished(status(pid)); }
  bool parked(int pid) { return status(pid) == process_status::waiting; }
  std::size_t held(int pid) { return info(pid)->locks_held; }
  std::string waiting_on(int pid) {
    const auto i = info(pid);
    return i && i->waiting_on ? *i->waiting_on : std::string();
  }
  bool lock_waiting(int pid) { return parked(pid) && waiting_on(pid).rfind("lock ", 0) == 0; }
  // Parked inside its section (a locker's body waits on its release channel).
  bool inside(int pid) { return parked(pid) && held(pid) > 0 && !lock_waiting(pid); }
  bool all_finished(const std::vector<int> &pids) {
    for (int p : pids)
      if (!finished(p))
        return false;
    return true;
  }
  std::optional<std::string> exit_error(int pid) { return info(pid)->exit_error; }

  // `script` submitted and driven to its end; its result.
  value_t run(const std::string &script, submit_options so = {}) {
    const int pid = submit(script, std::move(so));
    EXPECT_TRUE(step_until([&] { return finished(pid); }, 30.0)) << "did not finish: " << script;
    return result(pid);
  }

  value_t result(int pid) {
    const auto i = info(pid);
    EXPECT_TRUE(i.has_value());
    if (i && i->exit_error)
      ADD_FAILURE() << "pid " << pid << " ended with " << *i->exit_error;
    const std::optional<value_t> r = sync ? sync->get_result(pid) : async->get_result(pid);
    EXPECT_TRUE(r.has_value()) << "pid " << pid << " has no result";
    return r ? *r : nil_value();
  }

  // A process that takes KEY (a DSL form) with OPTIONS, marks TAG, then parks inside its section
  // until release(pid); its value is TAG. Not stepped.
  int locker(const std::string &key, const std::string &options, const std::string &tag,
             submit_options so = {}, const std::string &before = "") {
    const std::string ch = "release." + std::to_string(next_channel++);
    const std::string chroot = so.chroot;
    const int pid = submit("(with-lock " + key + " " + options + " (begin (mark \"" + tag + "\") " +
                               before + " (msg-recv \"" + ch + "\") \"" + tag + "\"))",
                           std::move(so));
    channels[pid] = resolve_channel_key(chroot, ch);
    return pid;
  }
  // A locker driven until it holds, parked inside.
  int holder(const std::string &key, const std::string &options = "", const std::string &tag = "H",
             submit_options so = {}, const std::string &before = "") {
    const int pid = locker(key, options, tag, std::move(so), before);
    EXPECT_TRUE(step_until([&] { return inside(pid); })) << "holder " << key << " never got in";
    return pid;
  }
  // A locker driven until it waits for its lock.
  int waiter(const std::string &key, const std::string &options = "", const std::string &tag = "W",
             submit_options so = {}) {
    const int pid = locker(key, options, tag, std::move(so));
    EXPECT_TRUE(step_until([&] { return lock_waiting(pid); }))
        << "waiter " << key << " never queued";
    return pid;
  }
  // A process whose section only marks TAG, driven until it waits for its lock.
  int queued(const std::string &key, const std::string &options = "", const std::string &tag = "Q",
             submit_options so = {}) {
    const int pid = submit("(with-lock " + key + " " + options + " (begin (mark \"" + tag +
                               "\") \"" + tag + "\"))",
                           std::move(so));
    EXPECT_TRUE(step_until([&] { return lock_waiting(pid); }))
        << "request " << key << " never queued";
    return pid;
  }
  // Wakes a locker parked inside its section.
  void release(int pid) { send(channels.at(pid)); }
  void send(const std::string &channel) {
    base().deliver_to_receivers(channel, value_t(std::string("go")));
    check_all();
  }
  bool kill(int pid) {
    const bool ok = base().kill(pid);
    check_all();
    return ok;
  }
  bool send_signal(int pid, const std::string &sig) {
    const bool ok = sync ? sync->send_signal(pid, sig) : async->send_signal(pid, sig);
    check_all();
    return ok;
  }
  bool snapshot(int pid, cvc::state &node) {
    return sync ? sync->snapshot(pid, node) : async->snapshot(pid, node);
  }
  int restore(cvc::state &node, environment_ptr natives) {
    return sync ? sync->restore(node, std::move(natives))
                : async->restore(node, std::move(natives));
  }
  int restore(process_ptr p) { return sync ? sync->restore(std::move(p)) : async->restore(p); }

  // What the next X grant on `key` reports as abandoned (a program that takes it at once).
  bool next_x_abandoned(const std::string &key) {
    return as_bool(run("(with-lock \"" + key + "\" :as g (get-attr g \"abandoned\"))"));
  }

  // The keys held now (absolute), as the manager reports them to a host.
  std::vector<std::string> held_keys() {
    std::vector<std::string> out;
    for (const lock_info &i : mgr->query("", nullptr, "").held)
      out.push_back(i.key);
    return out;
  }

  // The manager's mathematical state without its key-cache references, which a refused request
  // may change (I13).
  std::string state_without_cache() const {
    std::istringstream in(mgr->table().canonical());
    std::string line, out;
    bool skip = false;
    while (std::getline(in, line)) {
      if (line == "cache_refs")
        skip = true;
      else if (line.rfind("counters", 0) == 0)
        skip = false;
      if (!skip)
        out += line + "\n";
    }
    return out;
  }
};

// The I13 comparison: every counter but the cache's (and the trie's size, which cached keys keep).
void expect_same_lock_state(const lock_stats &a, const lock_stats &b) {
  EXPECT_EQ(a.holds, b.holds);
  EXPECT_EQ(a.queued, b.queued);
  EXPECT_EQ(a.grants, b.grants);
  EXPECT_EQ(a.fast_grants, b.fast_grants);
  EXPECT_EQ(a.waits, b.waits);
  EXPECT_EQ(a.timeouts, b.timeouts);
  EXPECT_EQ(a.deadlocks, b.deadlocks);
  EXPECT_EQ(a.abandoned, b.abandoned);
  EXPECT_EQ(a.ends, b.ends);
}

class Locks : public ::testing::TestWithParam<std::string> {
protected:
  checked_world w{GetParam()};
  void TearDown() override { EXPECT_FALSE(w.broken) << "an invariant check failed"; }
};

INSTANTIATE_TEST_SUITE_P(Schedulers, Locks, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });

const char *const kVariants[] = {"queued", "granted"};

} // namespace

// ---------------------------------------------------------------------------
// LK (13.2)
// ---------------------------------------------------------------------------

TEST_P(Locks, LK01_ValueAndLocksHeld) {
  const auto r = as_list(w.run(R"((list (with-lock "k" (list (locks-held) 7)) (locks-held)))"));
  ASSERT_EQ(r.size(), 2u);
  const auto inside = as_list(r[0]);
  ASSERT_EQ(inside.size(), 2u);
  EXPECT_EQ(as_int(inside[1]), 7);
  const auto held = as_list(inside[0]);
  ASSERT_EQ(held.size(), 1u);
  EXPECT_EQ(as_str(entry_of(held[0], "key")), "k");
  EXPECT_EQ(as_str(entry_of(held[0], "mode")), "exclusive");
  EXPECT_EQ(as_int(entry_of(held[0], "count")), 1);
  EXPECT_GE(as_int(entry_of(held[0], "generation")), 1);
  EXPECT_TRUE(as_list(r[1]).empty());
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LK02_ExclusiveExcludesSharedShares) {
  const int h = w.holder("\"k\"", "", "H");
  const int x = w.waiter("\"k\"", "", "X");
  const int s = w.waiter("\"k\"", ":shared", "S");
  EXPECT_EQ(w.waiting_on(x), "lock /k");
  EXPECT_EQ(w.waiting_on(s), "lock /k");
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.inside(x); }));
  EXPECT_TRUE(w.lock_waiting(s)); // X excludes S
  w.release(x);
  ASSERT_TRUE(w.step_until([&] { return w.inside(s); }));
  w.release(s);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, x, s}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"H", "X", "S"}));

  // Two S overlap: both inside at once, as a counter both bump shows.
  const int s1 = w.holder("\"r\"", ":shared", "S1", {}, "(state-incr \"inside\")");
  const int s2 = w.holder("\"r\"", ":shared", "S2", {}, "(state-incr \"inside\")");
  EXPECT_EQ(w.tree("inside").value(), "2");
  EXPECT_EQ(w.held(s1), 1u);
  EXPECT_EQ(w.held(s2), 1u);
  w.release(s1);
  w.release(s2);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({s1, s2}); }));
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LK03_FiveWritersInOrder) {
  std::vector<std::vector<std::string>> traces;
  for (int run = 0; run < 3; ++run) {
    checked_world v(GetParam());
    const int h = v.holder("\"k\"", "", "H");
    std::vector<int> writers;
    for (int i = 0; i < 5; ++i)
      writers.push_back(v.queued("\"k\"", "", "W" + std::to_string(i)));
    v.release(h);
    writers.push_back(h);
    ASSERT_TRUE(v.step_until([&] { return v.all_finished(writers); }));
    EXPECT_EQ(v.trace, (std::vector<std::string>{"H", "W0", "W1", "W2", "W3", "W4"}));
    std::vector<std::string> pids;
    for (int i = 0; i < 5; ++i)
      pids.push_back(std::to_string(writers[i]));
    traces.push_back(pids);
    EXPECT_FALSE(v.broken);
  }
  EXPECT_EQ(traces[0], traces[1]);
  EXPECT_EQ(traces[1], traces[2]);
}

TEST_P(Locks, LK04_WriterBlocksLaterReaders) {
  const int r1 = w.holder("\"k\"", ":shared", "R1");
  const int r2 = w.holder("\"k\"", ":shared", "R2");
  const int wr = w.queued("\"k\"", "", "W");
  const int r3 = w.queued("\"k\"", ":shared", "R3"); // behind the writer, though only S is held
  ASSERT_TRUE(w.settle());
  EXPECT_TRUE(w.lock_waiting(r3));
  w.release(r1);
  ASSERT_TRUE(w.step_until([&] { return w.finished(r1); }));
  EXPECT_TRUE(w.lock_waiting(wr));
  w.release(r2);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({r1, r2, wr, r3}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"R1", "R2", "W", "R3"}));
}

TEST_P(Locks, LK05_NoBargeOnRelease) {
  const int p = w.submit(R"((begin (with-lock "k" (msg-recv "p.go")) (with-lock "k" (mark "P"))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(p); }));
  const int q = w.queued("\"k\"", "", "W");
  w.send("p.go");
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({p, q}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"W", "P"}));
}

TEST_P(Locks, LK06_IfAvailableAndTimeoutZeroNeverBarge) {
  const int h = w.holder("\"k\"", ":shared", "H");
  const int x = w.queued("\"k\"", "", "X");
  const lock_stats s0 = w.stats();
  const std::string state0 = w.state_without_cache();
  // S on k is compatible with the S holder, but the queued X came first: neither barges.
  EXPECT_EQ(as_str(w.run(R"((with-lock "k" :shared :if-available
                               :else (begin (state-incr "else") "else") 1))")),
            "else");
  EXPECT_EQ(w.tree("else").value(), "1"); // :else evaluated once
  EXPECT_EQ(as_str(w.run(R"((try (with-lock "k" :shared :timeout 0 1)
                               (catch (TimeoutError e) "timeout")))")),
            "timeout");
  expect_same_lock_state(s0, w.stats());
  EXPECT_EQ(w.state_without_cache(), state0);
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, x}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"H", "X"}));
}

TEST_P(Locks, LK07_CoveredUpgradeBroaden) {
  // X a then S a.b: covered, count 2; the manager sees only the outer request.
  lock_stats s0 = w.stats();
  const auto r = as_list(w.run(R"((with-lock "a" (with-lock "a.b" :shared (locks-held))))"));
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(as_str(entry_of(r[0], "key")), "a");
  EXPECT_EQ(as_int(entry_of(r[0], "count")), 2);
  EXPECT_EQ(w.stats().grants - s0.grants, 1u);
  EXPECT_EQ(w.stats().waits, s0.waits);
  // S a then X a.b: the upgrade is refused.
  EXPECT_EQ(as_str(w.run(R"((with-lock "a" :shared
                               (try (with-lock "a.b" 1) (catch (LockError e) (get-attr e "reason")))))")),
            "upgrade");
  // X a.b then X a: granted, a second hold, when no other process holds under a.
  s0 = w.stats();
  const auto r2 = as_list(w.run(R"((with-lock "a.b" (with-lock "a" (locks-held))))"));
  ASSERT_EQ(r2.size(), 2u);
  EXPECT_EQ(as_str(entry_of(r2[0], "key")), "a.b");
  EXPECT_EQ(as_str(entry_of(r2[1], "key")), "a");
  EXPECT_EQ(w.stats().grants - s0.grants, 2u);
  EXPECT_EQ(w.stats().waits, s0.waits);
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LK08_ConflictOnSegments) {
  const auto probe = [&](const std::string &key, const std::string &mode) {
    return as_str(w.run("(try (with-lock \"" + key + "\" " + mode +
                        " :timeout 0 \"granted\") (catch (TimeoutError e) \"busy\"))"));
  };
  const int h = w.holder("\"a.b\"", "", "H");
  EXPECT_EQ(probe("a", ":shared"), "busy");
  EXPECT_EQ(probe("a", ""), "busy");
  EXPECT_EQ(probe("a.b.c", ":shared"), "busy");
  EXPECT_EQ(probe("a.c", ""), "granted");
  EXPECT_EQ(probe("a.bc", ""), "granted");
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.finished(h); }));

  // A queued X on a against a later X on a.b: no hold conflicts with a.b, the earlier waiter does.
  const int h2 = w.holder("\"a.c\"", "", "H2");
  const int qa = w.queued("\"a\"", "", "A");
  EXPECT_EQ(probe("a.b", ""), "busy");
  const int qb = w.queued("\"a.b\"", "", "AB");
  w.release(h2);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h2, qa, qb}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"H", "H2", "A", "AB"}));
}

TEST_P(Locks, LK09_MultiKeyCanonical) {
  const lock_stats s0 = w.stats();
  const auto r = as_list(w.run(R"((with-lock (list "b" "a" "a.x") (locks-held)))"));
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(to_string(entry_of(r[0], "key")), R"(("a" "b"))");
  EXPECT_EQ(w.stats().grants - s0.grants, 1u);
}

TEST_P(Locks, LK10_KeyResolution) {
  // Segments trimmed, empties dropped.
  const auto r = as_list(w.run(R"((with-lock " a..b " (locks-held)))"));
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(as_str(entry_of(r[0], "key")), "a.b");

  // Two chroots, one relative key: two locks.
  const int a = w.holder("\"x\"", "", "A", {.chroot = "docs.A"});
  const int b = w.holder("\"x\"", "", "B", {.chroot = "docs.B"});
  EXPECT_EQ(w.held_keys(), (std::vector<std::string>{"docs.A.x", "docs.B.x"}));
  // The same absolute key, unchrooted: the same lock (no "channels" segment).
  EXPECT_EQ(as_str(w.run(R"((try (with-lock "docs.A.x" :timeout 0 "granted")
                               (catch (TimeoutError e) "busy")))")),
            "busy");
  // Seen from the chroot, the key is relative; unchrooted, absolute.
  const auto q =
      w.run(R"((get-attr (car (get-attr (lock-query) "held")) "key"))", {.chroot = "docs.A"});
  EXPECT_EQ(as_str(q), "x");
  w.release(a);
  w.release(b);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({a, b}); }));
}

TEST_P(Locks, LK11_GlobalKeyNeedsPolicy) {
  channel_policy &pol = w.policies.emplace_back();
  pol.enforce = true;
  pol.global = {"shared.jobs"};
  const auto run = [&](const std::string &s) { return w.run(s, {.policy = &pol}); };
  EXPECT_EQ(as_str(run(R"((try (with-lock "/jobs" 1) (catch (PermissionError e) "denied")))")),
            "denied");
  EXPECT_EQ(as_str(run(R"((try (lock-query "/jobs") (catch (PermissionError e) "denied")))")),
            "denied");
  EXPECT_EQ(as_int(run(R"((with-lock "/shared.jobs" 1))")), 1);
  EXPECT_EQ(as_int(run(R"((with-lock "/shared.jobs.x" 2))")), 2);
  EXPECT_EQ(as_int(run(R"((with-lock "jobs" 3))")), 3); // relative: no permission needed
  // Without an enforcing policy a '/' key is allowed.
  EXPECT_EQ(as_int(w.run(R"((with-lock "/jobs" 4))")), 4);
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LK13_UserDefinitionsWin) {
  // The user's definitions run, and cost what the same program costs under another name (which
  // is what it cost before with-lock existed: an unbound name like any other).
  const auto steps_of = [&](const std::string &script, value_t &out) {
    const int pid = w.submit(script);
    EXPECT_TRUE(w.step_until([&] { return w.finished(pid); }));
    out = w.result(pid);
    return w.info(pid)->step_count;
  };
  const lock_stats s0 = w.stats();
  // The step counts are pinned as measured on this build, where they equal the renamed program's.
  struct user_case {
    const char *prog;
    const char *result;
    uint64_t steps;
  };
  for (const user_case &c :
       {user_case{R"((begin (defun with-lock (k v) (list "mine" k v)) (with-lock "k" 5)))",
                  R"(("mine" "k" 5))", 18},
        user_case{R"((begin (defmacro with-lock (k body) body) (with-lock "k" (+ 2 3))))", "5", 11},
        user_case{R"((begin (set with-lock (lambda (k v) (list "set" k v))) (with-lock "k" 6)))",
                  R"(("set" "k" 6))", 20}}) {
    const std::string prog = c.prog;
    value_t mine, other;
    const uint64_t steps = steps_of(prog, mine);
    std::string renamed = prog;
    for (std::size_t at; (at = renamed.find("with-lock")) != std::string::npos;)
      renamed.replace(at, 9, "user-lock");
    const uint64_t baseline = steps_of(renamed, other);
    EXPECT_EQ(to_string(mine), c.result) << prog;
    EXPECT_EQ(to_string(other), c.result) << renamed;
    EXPECT_EQ(steps, baseline) << prog;
    EXPECT_EQ(steps, c.steps) << prog;
  }
  expect_same_lock_state(s0, w.stats()); // no manager operation
}

TEST_P(Locks, LK14_AsBinding) {
  const auto r = as_list(w.run(
      R"((list (with-lock "a" :as g g) (with-lock "b" :as g g) (with-lock "s" :shared :as g g)))"));
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(as_str(entry_of(r[0], "key")), "a");
  EXPECT_EQ(as_str(entry_of(r[0], "mode")), "exclusive");
  EXPECT_FALSE(as_bool(entry_of(r[0], "abandoned")));
  const int64_t g1 = as_int(entry_of(r[0], "generation"));
  const int64_t g2 = as_int(entry_of(r[1], "generation"));
  EXPECT_GT(g2, g1);
  EXPECT_EQ(as_str(entry_of(r[2], "mode")), "shared");
  EXPECT_EQ(as_int(entry_of(r[2], "generation")), g2); // an S grant reports the last X generation

  // A covered section nested in a grant that reported abandoned: the outer generation, #f.
  const int k = w.holder("\"z\"", "", "K");
  w.kill(k);
  const auto r2 = as_list(w.run(R"((with-lock "z" :as g (list g (with-lock "z" :as g2 g2))))"));
  ASSERT_EQ(r2.size(), 2u);
  EXPECT_TRUE(as_bool(entry_of(r2[0], "abandoned")));
  EXPECT_GT(as_int(entry_of(r2[0], "generation")), g2);
  EXPECT_EQ(as_int(entry_of(r2[1], "generation")), as_int(entry_of(r2[0], "generation")));
  EXPECT_FALSE(as_bool(entry_of(r2[1], "abandoned")));

  // :if-available taking :else: the name is nil there.
  const int q = w.holder("\"q\"", "", "Q");
  EXPECT_TRUE(w.run(R"((with-lock "q" :if-available :as g :else g 1))").is_nil());
  w.release(q);
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }));

  // A chrooted multi-key grant shows its keys as locks-held does.
  EXPECT_EQ(to_string(w.run(R"((with-lock (list "b" "a") :as g (get-attr g "key")))",
                            {.chroot = "docs.A"})),
            R"(("a" "b"))");
}

TEST_P(Locks, LK14b_OptionErrors) {
  for (const char *form :
       {R"((with-lock "k" :bogus 1))", R"((with-lock "k" :timeout 1 :timeout 2 1))",
        R"((with-lock "k" :shared :exclusive 1))", R"((with-lock "k" :else 2 1))",
        R"((with-lock "k" :timeout))", R"((with-lock "k" :as 3 1))",
        R"((with-lock "k" :timeout (state-set "t" "x") :timeout 2 1))"}) {
    const lock_stats s0 = w.stats();
    const std::string state0 = w.state_without_cache();
    const auto r = as_list(w.run(std::string("(list (try ") + form +
                                 R"( (catch (ValueError e) "bad")) (locks-held)))"));
    ASSERT_EQ(r.size(), 2u);
    EXPECT_EQ(as_str(r[0]), "bad") << form;
    EXPECT_TRUE(as_list(r[1]).empty()) << form; // no mirror entry
    expect_same_lock_state(s0, w.stats());
    EXPECT_EQ(w.state_without_cache(), state0) << form;
  }
  EXPECT_FALSE(as_bool(w.run(R"((state-exists "t"))"))); // the :timeout never evaluated
}

TEST_P(Locks, LK15_QueryHidesOtherOwners) {
  const int other = w.holder("\"/k\"", "", "O", {.opts = {.owner = "other"}});
  const int mine = w.holder("\"/j\"", "", "M", {.opts = {.owner = "me"}});
  const auto r = as_list(w.run(R"((list (lock-query "/k") (lock-query "/j")))",
                               {.opts = {.owner = "me"}, .chroot = "docs.A"}));
  ASSERT_EQ(r.size(), 2u);
  const auto k = as_list(entry_of(r[0], "held"));
  ASSERT_EQ(k.size(), 1u);
  EXPECT_EQ(as_str(entry_of(k[0], "key")), "/k");
  EXPECT_EQ(as_int(entry_of(k[0], "count")), 1);        // counted
  EXPECT_TRUE(as_list(entry_of(k[0], "pids")).empty()); // its pid left out
  const auto j = as_list(entry_of(r[1], "held"));
  ASSERT_EQ(j.size(), 1u);
  const auto jp = as_list(entry_of(j[0], "pids"));
  ASSERT_EQ(jp.size(), 1u);
  EXPECT_EQ(as_int(jp[0]), mine);
  w.release(other);
  w.release(mine);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({other, mine}); }));
}

TEST_P(Locks, LK16_NoMessageWakesALockWaiter) {
  const int h = w.holder("\"k\"", "", "H");
  const int q = w.queued("\"k\"", "", "Q");
  for (const char *ch : {"#lock", "#", "k#", "#k", "k", "lock", "lock /k", "with-lock"}) {
    w.run(std::string("(try (msg-send \"") + ch + "\" \"x\") (catch (e) nil))");
    ASSERT_TRUE(w.settle());
    EXPECT_TRUE(w.lock_waiting(q)) << ch;
    EXPECT_EQ(w.waiting_on(q), "lock /k") << ch;
  }
  EXPECT_EQ(as_str(w.run("(get-attr (inspect " + std::to_string(q) + ") \"waiting_on\")")),
            "lock k");
  EXPECT_EQ(as_str(w.run("(get-attr (inspect " + std::to_string(q) + ") \"waiting_on\")",
                         {.chroot = "docs"})),
            "lock /k");
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q}); }));
}

// LK-18: random S/X single- and multi-key requests with timeouts and kills; every invariant after
// every step, and the manager empty at the end (I9).
namespace {

// LK-18's sizes (L1.md section 3, "Sizes, by lane"): CVC_LOCKS_RANDOM_OPS, else by build.
std::size_t random_ops_regular() {
  if (const char *v = std::getenv("CVC_LOCKS_RANDOM_OPS"); v && *v)
    return static_cast<std::size_t>(std::strtoull(v, nullptr, 10));
  // Every invariant is checked after every step (the check walks the trie). On a CI runner 3000
  // operations take 5-13 s in Release, and a Debug build is 25-50 times slower: 2000 took 150-240 s
  // there, and past ctest's 300 s on a loaded runner. The deep sizes are LocksStress's. By the
  // optimizer, not NDEBUG, which a Debug build can define too: CVC_ENABLE_MESHER's did, and the
  // Debug package lanes ran 3000 at 12 a second.
  if (under_sanitizer())
    return 1000;
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  return 3000;
#else
  return 500;
#endif
}
constexpr std::size_t kRandomOpsStress = 100000;

// Seeded random programs: sections on paths of depth 1-8 in a 3-ary tree, single- and multi-key,
// S and X, with and without timeouts, some nested. The paths come from a pool drawn once (20 of
// each depth), so requests meet often at every depth, and the default key cache holds them all
// while a cache of 4 evicts on most requests. (Fresh random paths would fill the default cache
// and the trie with keys used once, whose checks after every step then dominate the run.)
struct random_programs {
  std::mt19937_64 rng;
  std::size_t ops = 0; // lock requests generated so far (executed or not)
  std::vector<std::string> pool;
  explicit random_programs(std::uint64_t seed) : rng(seed) {
    static const char *const seg[] = {"a", "b", "c"};
    for (int i = 0; i < 160; ++i) {
      std::string k = seg[pick(3)];
      for (int d = 1, depth = 1 + i % 8; d < depth; ++d)
        k += std::string(".") + seg[pick(3)];
      pool.push_back("\"" + k + "\"");
    }
  }
  int pick(int n) { return std::uniform_int_distribution<int>(0, n - 1)(rng); }
  std::string key() { return pool[static_cast<std::size_t>(pick(static_cast<int>(pool.size())))]; }
  std::string keys() {
    if (pick(4) != 0)
      return key();
    std::string l = "(list";
    for (int i = 0, n = 2 + pick(2); i < n; ++i)
      l += " " + key();
    return l + ")";
  }
  std::string options() {
    std::string o = pick(2) ? ":shared" : "";
    const int t = pick(10);
    if (t < 3)
      return o; // waits as long as it takes
    if (t < 4)
      return o + " :timeout 0";
    if (t < 5)
      return o + " :if-available :else (state-incr \"else\")";
    char buf[32];
    std::snprintf(buf, sizeof buf, " :timeout %.3f", 0.001 * (1 + pick(20)));
    return o + buf;
  }
  std::string body(int depth) {
    switch (pick(depth < 2 ? 5 : 3)) {
    case 0:
      return "(sleep 0)";
    case 1:
      return "(state-incr \"n\")";
    case 2:
      return "(locks-held)";
    default:
      return section(depth + 1);
    }
  }
  std::string section(int depth) {
    ++ops;
    return "(try (with-lock " + keys() + " " + options() + " " + body(depth) + " " + body(depth) +
           ") (catch (TimeoutError e) nil) (catch (LockError e) nil))";
  }
  std::string program() {
    std::string p = "(begin";
    for (int i = 0, n = 3 + pick(6); i < n; ++i)
      p += " " + section(0);
    return p + ")";
  }
};

void run_random_operations(const std::string &kind, std::size_t target_ops,
                           std::size_t cache_capacity, std::uint64_t seed, double max_seconds) {
  hooks_scope hooks(cache_capacity); // before the world: its home reads the capacity once
  const auto t0 = steady::now();
  checked_world v(kind, hooks.mgr);
  EXPECT_EQ(v.home().cache_capacity(), cache_capacity ? cache_capacity : 1024u);
  random_programs gen(seed);
  std::vector<int> live;
  const auto spawn = [&] { live.push_back(v.submit(gen.program())); };
  for (int i = 0; i < 16; ++i)
    spawn();
  // An operation is a request that came to an outcome in the manager: a grant, a timeout or a
  // deadlock (the requests of killed programs that never ran don't count).
  const auto done_ops = [&] {
    const lock_stats s = v.stats();
    return static_cast<std::size_t>(s.grants + s.timeouts + s.deadlocks);
  };
  const auto until = deadline_after(max_seconds);
  std::uint64_t rounds = 0, kills = 0, ended = 0;
  while (!live.empty() && !v.broken && steady::now() < until) {
    v.round(until);
    if (++rounds % 50 == 0) {
      v.kill(live[static_cast<std::size_t>(gen.pick(static_cast<int>(live.size())))]);
      ++kills;
    }
    std::size_t replace = 0;
    for (auto it = live.begin(); it != live.end();) {
      const int pid = *it;
      if (!v.finished(pid)) {
        ++it;
        continue;
      }
      const auto info = v.info(pid);
      EXPECT_NE(info->status, process_status::failed)
          << "pid " << pid << ": " << info->exit_error.value_or("");
      ++ended;
      it = live.erase(it);
      if (v.sync)
        v.sync->reap(pid);
      else
        v.async->reap(pid);
      v.envs.erase(pid);
      if (done_ops() < target_ops)
        ++replace;
    }
    for (std::size_t i = 0; i < replace; ++i)
      spawn();
  }
  EXPECT_TRUE(live.empty()) << live.size() << " processes still running after " << max_seconds
                            << " s";
  const std::size_t trie_at_end = v.stats().trie_nodes;
  for (int pid : live)
    v.kill(pid);
  const lock_stats s = v.stats();
  std::printf("LK-18 %s cache %zu: %zu operations (%zu generated), %llu rounds, %llu kills, %llu "
              "ended; grants %llu "
              "(fast %llu), waits %llu, timeouts %llu, deadlocks %llu, abandoned %llu; trie %zu "
              "nodes; %.2f s\n",
              kind.c_str(), v.home().cache_capacity(), done_ops(), gen.ops,
              static_cast<unsigned long long>(rounds), static_cast<unsigned long long>(kills),
              static_cast<unsigned long long>(ended), static_cast<unsigned long long>(s.grants),
              static_cast<unsigned long long>(s.fast_grants),
              static_cast<unsigned long long>(s.waits), static_cast<unsigned long long>(s.timeouts),
              static_cast<unsigned long long>(s.deadlocks),
              static_cast<unsigned long long>(s.abandoned), trie_at_end, seconds_since(t0));
  EXPECT_GE(done_ops(), target_ops);
  EXPECT_GT(s.waits, 0u);
  EXPECT_GT(s.timeouts, 0u);
  // I9: every process ended, the manager is empty.
  EXPECT_TRUE(v.mgr->table().empty()) << v.mgr->table().canonical();
  EXPECT_EQ(s.holds, 0u);
  EXPECT_EQ(s.queued, 0u);
  EXPECT_TRUE(v.check_all());
}

} // namespace

TEST_P(Locks, LK18_RandomOperationsKeepInvariants) {
  const std::size_t ops = random_ops_regular();
  run_random_operations(GetParam(), ops, 0, 18, 240.0);
  run_random_operations(GetParam(), ops, 4, 1818, 240.0); // evictions on most requests
}

namespace {
class LocksStress : public ::testing::TestWithParam<std::string> {};
INSTANTIATE_TEST_SUITE_P(Schedulers, LocksStress, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });
} // namespace

TEST_P(LocksStress, LK18_RandomOperationsKeepInvariants) {
  run_random_operations(GetParam(), kRandomOpsStress, 0, 180, 3000.0);
}

TEST_P(LocksStress, LK18_SmallCache) {
  run_random_operations(GetParam(), kRandomOpsStress, 4, 18180, 3000.0);
}

TEST_P(Locks, LK19_MultiKeyWaitsHoldingNothing) {
  const int q = w.holder("\"a.y\"", "", "Q");
  const int p = w.queued("(list \"a.x\" \"a.y\")", "", "P");
  EXPECT_EQ(w.held_keys(), (std::vector<std::string>{"a.y"})); // a.x is not held while P waits
  EXPECT_EQ(w.held(p), 0u);
  const int r = w.queued("\"a\"", "", "R");
  const lock_stats s0 = w.stats();
  w.release(q);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({q, p, r}); }));
  EXPECT_EQ(as_str(w.result(p)), "P"); // no DeadlockError
  EXPECT_EQ(w.stats().deadlocks, s0.deadlocks);
  EXPECT_EQ(w.trace, (std::vector<std::string>{"Q", "P", "R"}));
}

TEST_P(Locks, LK20_NestedSiblingPassesQueuedParentWriter) {
  const int p = w.submit(R"((with-lock "a.x" (begin (msg-recv "p.go")
                              (list (with-lock "a.y" 1)
                                    (with-lock "a.y" :if-available :else "else" 2)
                                    (try (with-lock "a.y" :timeout 0 3)
                                      (catch (TimeoutError e) "timeout")
                                      (catch (DeadlockError e) "deadlock"))))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(p); }));
  const int q = w.queued("\"a\"", "", "Q");
  const lock_stats s0 = w.stats();
  w.send("p.go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(p); }));
  EXPECT_EQ(to_string(w.result(p)), "(1 2 3)");
  EXPECT_EQ(w.stats().waits, s0.waits);
  EXPECT_EQ(w.stats().deadlocks, s0.deadlocks);
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }));
}

TEST_P(Locks, LK21_GrantAllInOrder) {
  const int h = w.holder("\"a\"", "", "H");
  const int s1 = w.waiter("\"a\"", ":shared", "S1");
  const int s2 = w.waiter("\"a\"", ":shared", "S2");
  const int x3 = w.waiter("\"a\"", "", "X3");
  const int s4 = w.waiter("\"a\"", ":shared", "S4");
  w.release(h);
  // The two first S together.
  ASSERT_TRUE(w.step_until([&] { return w.inside(s1) && w.inside(s2); }));
  EXPECT_TRUE(w.lock_waiting(x3));
  EXPECT_TRUE(w.lock_waiting(s4));
  w.release(s1);
  ASSERT_TRUE(w.step_until([&] { return w.finished(s1); }));
  EXPECT_TRUE(w.lock_waiting(x3)); // after both
  w.release(s2);
  ASSERT_TRUE(w.step_until([&] { return w.inside(x3); }));
  EXPECT_TRUE(w.lock_waiting(s4)); // the last S after the X
  w.release(x3);
  ASSERT_TRUE(w.step_until([&] { return w.inside(s4); }));
  w.release(s4);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, s1, s2, x3, s4}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"H", "S1", "S2", "X3", "S4"}));
}

namespace {
// LK-22 and LK-25: H holds S on `held`; W waits for X on `held` with a short timeout; R waits for S
// on `reader` behind W. W's timeout re-examines the waiters behind it: R is granted in the very
// round W's wait ends.
void writer_timeout_lets_readers_in(checked_world &w, const std::string &held,
                                    const std::string &reader) {
  const int h = w.holder("\"" + held + "\"", ":shared", "H");
  const int wr = w.submit("(try (with-lock \"" + held +
                          "\" :timeout 0.3 1) (catch (TimeoutError e) \"timeout\"))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(wr); }));
  const int r = w.waiter("\"" + reader + "\"", ":shared", "R");
  ASSERT_TRUE(w.step_until([&] { return !w.lock_waiting(wr); }));
  EXPECT_EQ(w.held(r), 1u); // granted at once, in the round of the timeout
  EXPECT_EQ(w.held(wr), 0u);
  ASSERT_TRUE(w.step_until([&] { return w.finished(wr) && w.inside(r); }));
  EXPECT_EQ(as_str(w.result(wr)), "timeout");
  w.release(r);
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, r}); }));
}
} // namespace

TEST_P(Locks, LK22_WriterTimeoutLetsReadersIn) { writer_timeout_lets_readers_in(w, "a.b", "a"); }

TEST_P(Locks, LK25_SharedWaiterBehindTimedOutWriter) {
  writer_timeout_lets_readers_in(w, "k", "k");
}

// LK-23's C++ twin: the with-lock binding a host re-wraps keeps its context alive (pycvc's
// register_intrinsics does this for the Python owner); the context's owner drops it, and the form
// still works.
TEST_P(Locks, LK23_ContextDroppedCxx) {
  auto ctx = std::make_shared<intrinsics_context>();
  ctx->sched = &w.base();
  ctx->root = &w.tree;
  ctx->proc = make_process();
  environment_ptr env = builtins::make_default_environment();
  register_intrinsics(env, ctx.get());
  env->set("with-lock", detail::make_lock_ops(ctx.get(), ctx));
  const std::weak_ptr<intrinsics_context> weak = ctx;
  ctx.reset();
  EXPECT_FALSE(weak.expired()); // the binding keeps it
  execute_options opts;
  opts.env = env;
  const int pid = w.base().execute(std::string(R"((with-lock "k" (+ 0 1)))"), opts);
  ASSERT_TRUE(w.step_until([&] { return w.finished(pid); }));
  EXPECT_EQ(as_int(w.result(pid)), 1);
}

TEST_P(Locks, LK26_BroadenAfterOtherReleases) {
  const int p =
      w.submit(R"((with-lock "a.b" (begin (msg-recv "p.go") (with-lock "a" (mark "P-a")))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(p); }));
  const int r = w.holder("\"a.c\"", ":shared", "R");
  w.send("p.go");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(p); }));
  EXPECT_EQ(w.waiting_on(p), "lock /a");
  w.release(r);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({p, r}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"R", "P-a"}));
}

namespace {
// LK-27 and LK-28: P holds `outer` (MODE), Q's X on a queues, P then asks for `inner`: granted at
// once (the conversion rule); Q after P releases both.
void conversion_rule(checked_world &w, const std::string &outer, const std::string &mode,
                     const std::string &inner) {
  const int p =
      w.submit("(with-lock \"" + outer + "\" " + mode +
               " (begin (msg-recv \"p.go\") (with-lock \"" + inner + "\" (mark \"P-inner\"))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(p); }));
  const int q = w.queued("\"a\"", "", "Q");
  const lock_stats s0 = w.stats();
  w.send("p.go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(p); }));
  EXPECT_EQ(w.stats().waits, s0.waits); // granted at once
  EXPECT_EQ(w.stats().deadlocks, s0.deadlocks);
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"P-inner", "Q"}));
}
} // namespace

TEST_P(Locks, LK27_ConversionRuleNested) { conversion_rule(w, "a.x", ":shared", "a.y"); }

TEST_P(Locks, LK28_ConversionRuleBroaden) { conversion_rule(w, "a.b", "", "a"); }

TEST_P(Locks, LK30_MultiKeyTimeout) {
  const int a = w.submit(R"((with-lock "a" (sleep 0.45)))");
  ASSERT_TRUE(w.step_until([&] { return w.held(a) == 1; }));
  const int b = w.holder("\"b\"", "", "B");
  const lock_stats s0 = w.stats();
  const auto t0 = steady::now();
  const int p = w.submit(
      R"((try (with-lock (list "a" "b") :timeout 0.5 1) (catch (TimeoutError e) "timeout")))");
  std::size_t most = 0;
  ASSERT_TRUE(w.step_until(
      [&] {
        most = std::max(most, w.held(p));
        return w.finished(p);
      },
      20.0));
  const double took = seconds_since(t0);
  std::printf("LK-30: the multi-key request timed out after %.3f s\n", took);
  EXPECT_EQ(as_str(w.result(p)), "timeout");
  EXPECT_GE(took, 0.5); // a deadline never fires early
  EXPECT_LT(took, 10.0);
  EXPECT_EQ(w.stats().timeouts - s0.timeouts, 1u);
  EXPECT_EQ(most, 0u); // nothing ever held
  w.release(b);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({a, b}); }));
}

TEST_P(Locks, LK32_NonConflictingPassFreely) {
  const int h = w.holder("\"a\"", "", "H");
  const int q = w.queued("\"a.b\"", ":shared", "W");
  const lock_stats s0 = w.stats();
  EXPECT_EQ(as_int(w.run(R"((with-lock "b" 1))")), 1);
  EXPECT_EQ(as_int(w.run(R"((with-lock "ab" 2))")), 2);
  EXPECT_EQ(w.stats().waits, s0.waits); // both granted at once
  EXPECT_TRUE(w.lock_waiting(q));       // W still queued
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q}); }));
}

TEST_P(Locks, LK33_CoveredMultiKey) {
  const lock_stats s0 = w.stats();
  const auto r =
      as_list(w.run(R"((with-lock "a" (with-lock (list "a.b" "a.c") :shared (locks-held))))"));
  ASSERT_EQ(r.size(), 1u);
  EXPECT_EQ(as_str(entry_of(r[0], "key")), "a");
  EXPECT_EQ(as_int(entry_of(r[0], "count")), 2);
  EXPECT_EQ(w.stats().grants - s0.grants, 1u); // the covered request made no manager call
}

// ---------------------------------------------------------------------------
// LX (13.3): every way out. Design 13.3 runs each case with the process queued and with it
// granted but not yet resumed. Where the way out needs the process to run its body (LX-1's
// exits, LX-6's termination), "queued" is a section entered after a wait and "granted" one
// entered at once; elsewhere the way out lands while the request is queued, or between its grant
// and the process's next step.
// ---------------------------------------------------------------------------

TEST_P(Locks, LX01_EveryNormalExit) {
  struct exit_case {
    const char *name;
    std::string program; // CH: the channel the body parks on before it leaves
    std::optional<int64_t> value;
  };
  const std::vector<exit_case> cases = {
      {"value", R"((with-lock "k" (begin (mark "P") (msg-recv "CH") 1)))", 1},
      {"return",
       R"((begin (defun f () (with-lock "k" (begin (mark "P") (msg-recv "CH") (return 2) 99))) (f)))",
       2},
      {"break",
       R"((begin (set r 0) (while t (with-lock "k" (begin (mark "P") (msg-recv "CH") (set r 3) (break)))) r))",
       3},
      {"caught raise",
       R"((try (with-lock "k" (begin (mark "P") (msg-recv "CH") (raise (ValueError "x"))))
            (catch (ValueError e) 4)))",
       4},
      {"uncaught raise",
       R"((with-lock "k" (begin (mark "P") (msg-recv "CH") (raise (ValueError "x")))))",
       std::nullopt},
  };
  for (const char *variant : kVariants) {
    for (const exit_case &c : cases) {
      SCOPED_TRACE(std::string(variant) + ", " + c.name);
      checked_world v(GetParam());
      int h = -1;
      if (std::string(variant) == "queued")
        h = v.holder("\"k\"", "", "H");
      std::string prog = c.program;
      prog.replace(prog.find("CH"), 2, "p.go");
      // An outer finally sees the lock free: it runs after the release.
      const int p =
          v.submit("(try " + prog + " (finally (state-set \"fin\" (str (length (locks-held))))))");
      if (h >= 0) {
        ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(p); }));
        v.release(h);
      }
      ASSERT_TRUE(v.step_until([&] { return v.inside(p); }));
      const int q = v.queued("\"k\"", "", "W"); // the next waiter, granted by P's way out
      v.send("p.go");
      ASSERT_TRUE(v.step_until([&] { return v.finished(p) && v.finished(q); }));
      if (c.value) {
        EXPECT_EQ(as_int(v.result(p)), *c.value);
      } else {
        EXPECT_EQ(v.status(p), process_status::failed);
        EXPECT_NE(v.exit_error(p).value_or("").find("ValueError"), std::string::npos);
      }
      EXPECT_EQ(v.tree("fin").value(), "0");
      EXPECT_EQ(v.trace.back(), "W");
      EXPECT_FALSE(v.next_x_abandoned("k")); // released, not abandoned
      EXPECT_TRUE(v.mgr->table().empty());
      EXPECT_FALSE(v.broken);
    }
  }
}

namespace {
// LX-2's ways out. `external` ones act from outside (the process may be parked); the others are
// the process's budgets and errors, which act when it steps.
struct death {
  const char *name;
  bool async_only;
  bool external;
  const char *reason; // the exit error (a prefix)
};
const death kDeaths[] = {
    {"kill", false, true, "killed_by_user"},
    {"SIGKILL", false, true, "SIGKILL"},
    {"kill_owner", true, true, "owner_reaped"},
    {"max_steps", false, false, "max_steps_exceeded"},
    {"max_time", false, false, "time_limit_exceeded"},
    {"max_memory", false, false, "memory_limit_exceeded"},
    {"step error", false, false, ""},
    {"max_step_time", true, false, "time_limit_exceeded"},
};
} // namespace

// LX-2: every kill and budget of design 5.6 ends a process inside its body on "p.k" (X or S)
// while its request for "q" (held by Q) is queued, or granted but not resumed. Its X key is
// abandoned (the next X grant on it says so once; one on its parent does not); its S key and its
// unstarted grant are released clean; the manager ends empty.
TEST_P(Locks, LX02_KillsAndBudgetsRelease) {
  const bool is_async = GetParam() == "async";
  for (const death &d : kDeaths) {
    if (d.async_only && !is_async)
      continue;
    for (const char *variant : kVariants) {
      const bool queued = std::string(variant) == "queued";
      if (!queued && std::string(d.name) == "max_memory")
        continue; // a memory kill follows a heap measurement over later steps: it can't land
                  // between a grant and the process's next step
      for (const bool exclusive : {true, false}) {
        SCOPED_TRACE(std::string(d.name) + ", " + variant + (exclusive ? ", X" : ", S"));
        hooks_scope hooks;
        checked_world v(GetParam(), hooks.mgr);
        const int q = v.holder("\"q\"", "", "Q");
        // What runs inside q's section once P is in it: a budget runs out there (queued variant).
        std::string burn = "(msg-recv \"never\")";
        checked_world::submit_options so;
        so.opts.owner = "victim";
        const std::string name = d.name;
        if (!d.external && queued) {
          if (name == "max_steps") {
            so.opts.max_steps = 400;
            burn = "(while t (set i 1))";
          } else if (name == "max_time") {
            so.opts.max_time = 0.05;
            burn = "(while t (set i 1))";
          } else if (name == "max_memory") {
            so.opts.max_memory = 64 * 1024;
            burn =
                R"((begin (set l (list)) (while t (set l (cons "xxxxxxxxxxxxxxxxxxxxxxxx" l)))))";
          } else if (name == "step error") {
            burn = is_async ? "(boom)" : "(spin-step)";
            if (!is_async)
              so.opts.max_time = 0.05; // the sync scheduler's step deadline (design 5.6)
          } else if (name == "max_step_time") {
            so.opts.max_step_time = 0.02;
            burn = "(spin-step)";
          }
        } else if (!d.external) {
          if (name == "max_time" || (name == "step error" && !is_async))
            so.opts.max_time = 0.05;
          if (name == "max_step_time")
            so.opts.max_step_time = 0.02;
        }
        const int p = v.submit("(with-lock \"p.k\" " + std::string(exclusive ? "" : ":shared") +
                                   " (begin (mark \"P\") (with-lock \"q\" (begin (mark \"P-q\") " +
                                   burn + "))))",
                               so);
        ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(p); }));
        ASSERT_EQ(v.held(p), 1u);
        bool q_started = false;
        if (!queued) {
          v.release(q);
          ASSERT_TRUE(v.step_until([&] { return v.held(p) == 2; })); // granted, not resumed
          ASSERT_EQ(v.status(p), process_status::ready);
        } else if (!d.external) {
          v.release(q);
          q_started = true;
        }
        // The way out.
        if (name == "kill")
          v.kill(p);
        else if (name == "SIGKILL")
          v.send_signal(p, "SIGKILL");
        else if (name == "kill_owner")
          EXPECT_EQ(v.async->kill_owner("victim"), 1);
        else if (!queued) {
          if (name == "max_steps")
            (v.sync ? v.sync->set_max_steps(p, v.info(p)->step_count + 1)
                    : v.async->set_max_steps(p, v.info(p)->step_count + 1));
          else if (name == "step error" && is_async)
            g_hook.interrupt_in_enter_pid = p;
          else
            g_hook.deadline_in_enter_pid = p; // max_time, max_step_time, the sync step deadline
        }
        ASSERT_TRUE(v.step_until([&] { return v.finished(p); }, 20.0));
        EXPECT_EQ(v.status(p), process_status::killed) << v.exit_error(p).value_or("");
        const std::string why = v.exit_error(p).value_or("");
        if (name == "step error" && is_async)
          EXPECT_NE(why.find("internal step error"), std::string::npos) << why;
        else
          EXPECT_EQ(why.rfind(d.reason, 0), 0u) << why;
        EXPECT_EQ(v.held(p), 0u);
        if (q_started)
          EXPECT_NE(std::find(v.trace.begin(), v.trace.end(), "P-q"), v.trace.end());
        else
          EXPECT_EQ(std::find(v.trace.begin(), v.trace.end(), "P-q"), v.trace.end());
        if (!v.finished(q)) {
          v.release(q);
          ASSERT_TRUE(v.step_until([&] { return v.finished(q); }));
        }
        EXPECT_FALSE(v.next_x_abandoned("p")); // the parent: not the key
        EXPECT_EQ(v.next_x_abandoned("p.k"), exclusive);
        EXPECT_FALSE(v.next_x_abandoned("p.k")); // once
        // q: abandoned only when P's body in it had started.
        EXPECT_EQ(v.next_x_abandoned("q"), q_started);
        EXPECT_TRUE(v.mgr->table().empty()) << v.mgr->table().canonical();
        EXPECT_FALSE(v.broken);
      }
    }
  }
}

// The sync scheduler's internal step error is not the program's: it propagates out of step() to
// the host, as every such error does there (scheduler.cpp: "An error that isn't the program's
// ... propagates"), with the process left running. The host's kill then releases it, abandoned.
TEST(LocksSync, LX02_InternalStepErrorPropagatesThenTheKillReleases) {
  checked_world v("sync");
  const int p = v.submit(R"((with-lock "k" (begin (mark "P") (boom))))");
  bool thrown = false;
  const auto until = deadline_after(10.0);
  while (!thrown && steady::now() < until) {
    try {
      v.round(until);
    } catch (const evaluation_interrupted &) {
      thrown = true;
    }
  }
  ASSERT_TRUE(thrown);
  EXPECT_FALSE(v.finished(p));
  EXPECT_EQ(v.held(p), 1u);
  v.kill(p);
  EXPECT_TRUE(v.next_x_abandoned("k"));
  EXPECT_TRUE(v.mgr->table().empty());
  EXPECT_FALSE(v.broken);
}

TEST_P(Locks, LX03_KillGrantedBeforeItSteps) {
  for (const char *variant : kVariants) {
    const bool queued = std::string(variant) == "queued";
    for (const bool after_abandonment : {false, true}) {
      SCOPED_TRACE(std::string(variant) + (after_abandonment ? ", after an abandonment" : ""));
      checked_world v(GetParam());
      const int h = v.holder("\"k\"", "", "H");
      const int p = v.queued("\"k\"", "", "P");
      const int q =
          v.submit(R"((with-lock "k" :as g (begin (mark "W") (get-attr g "abandoned"))))");
      ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(q); }));
      if (queued) {
        v.kill(p);
        if (after_abandonment)
          v.kill(h); // the key flagged; W takes the flag
        else
          v.release(h);
      } else {
        if (after_abandonment)
          v.kill(h); // flagged; P's grant takes the flag
        else
          v.release(h);
        ASSERT_TRUE(v.step_until([&] { return v.held(p) == 1; })); // granted, not resumed
        v.kill(p); // released clean: an unstarted grant gives the flag back
      }
      ASSERT_TRUE(v.step_until([&] { return v.finished(q); }));
      EXPECT_EQ(as_bool(v.result(q)), after_abandonment); // the next X to enter its body
      EXPECT_EQ(std::count(v.trace.begin(), v.trace.end(), "P"), 0);
      EXPECT_EQ(v.trace.back(), "W");
      EXPECT_FALSE(v.next_x_abandoned("k"));
      EXPECT_TRUE(v.mgr->table().empty());
      EXPECT_FALSE(v.broken);
    }
  }
}

// LX-4: a pending signal handler runs before the lock_enter step of a waiter that was granted. One
// that raises: the raise unwinds the lock_enter frame, which gives the granted hold back (clean).
// One that asks for the granted key: LockError("handler"), raised in the program the same way.
TEST_P(Locks, LX04_HandlerBeforeLockEnter) {
  for (const char *variant : kVariants) {
    for (const auto &[handler, expect] :
         {std::pair<std::string, std::string>{R"((raise (RuntimeError "from the handler")))",
                                              "raised"},
          {R"((with-lock "k" 1))", "handler"}}) {
      SCOPED_TRACE(std::string(variant) + ", " + handler);
      checked_world v(GetParam());
      const int h = v.holder("\"k\"", "", "H");
      checked_world::submit_options so;
      so.opts.signal_handlers["SIGUSR1"] = parse(handler);
      const int p = v.submit(R"((try (with-lock "k" (mark "P-body"))
                                   (catch (LockError e) (get-attr e "reason"))
                                   (catch (RuntimeError e) "raised")))",
                             so);
      ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(p); }));
      const int q = v.queued("\"k\"", "", "W");
      if (std::string(variant) == "queued") {
        v.send_signal(p, "SIGUSR1");
        ASSERT_TRUE(v.settle());
        EXPECT_TRUE(v.lock_waiting(p)); // a waiter's handlers run after it is woken
        v.release(h);
      } else {
        v.release(h);
        ASSERT_TRUE(v.step_until([&] { return v.held(p) == 1; })); // granted, not resumed
        v.send_signal(p, "SIGUSR1");
      }
      ASSERT_TRUE(v.step_until([&] { return v.all_finished({h, p, q}); }));
      EXPECT_EQ(as_str(v.result(p)), expect);
      EXPECT_EQ(std::count(v.trace.begin(), v.trace.end(), "P-body"), 0);
      EXPECT_EQ(v.trace.back(), "W");
      EXPECT_FALSE(v.next_x_abandoned("k")); // the unstarted grant went back clean
      EXPECT_TRUE(v.mgr->table().empty());
      EXPECT_FALSE(v.broken);
    }
  }
}

// LX-5a: a timeout or a kill racing a grant made on the same home before the waiter steps. One
// outcome either way, and every check passes.
TEST_P(Locks, LX05a_TimeoutOrKillRacingSameHomeGrant) {
  for (const char *variant : kVariants) {
    const bool queued = std::string(variant) == "queued";
    {
      SCOPED_TRACE(std::string(variant) + ", timeout");
      checked_world v(GetParam());
      const int h = v.holder("\"k\"", "", "H");
      const int p = v.submit(R"((try (with-lock "k" :timeout 0.2 (begin (mark "P-body") "body"))
                                   (catch (TimeoutError e) "timeout")))");
      ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(p); }));
      if (queued) {
        // The deadline passes, then the holder's release is ready too: both are due at the next
        // step, before P has stepped.
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        v.release(h);
      } else {
        v.release(h);
        ASSERT_TRUE(v.step_until([&] { return v.held(p) == 1; }));   // granted, not resumed
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // past the old deadline
      }
      ASSERT_TRUE(v.step_until([&] { return v.all_finished({h, p}); }));
      const std::string out = as_str(v.result(p));
      const auto bodies = std::count(v.trace.begin(), v.trace.end(), "P-body");
      EXPECT_TRUE((out == "timeout" && bodies == 0) || (out == "body" && bodies == 1)) << out;
      if (!queued)
        EXPECT_EQ(out, "body"); // a grant is kept: its deadline left with the wait
      std::printf("LX-5a %s/%s: the racing timeout's outcome: %s\n", GetParam().c_str(), variant,
                  out.c_str());
      EXPECT_TRUE(v.mgr->table().empty());
      EXPECT_FALSE(v.broken);
    }
    {
      SCOPED_TRACE(std::string(variant) + ", kill");
      checked_world v(GetParam());
      const int h = v.holder("\"k\"", "", "H");
      const int p = v.queued("\"k\"", "", "P");
      const int q = v.queued("\"k\"", "", "W");
      v.release(h); // ready, not stepped
      if (!queued)
        ASSERT_TRUE(v.step_until([&] { return v.held(p) == 1; }));
      v.kill(p);
      ASSERT_TRUE(v.step_until([&] { return v.all_finished({h, p, q}); }));
      EXPECT_EQ(v.trace, (std::vector<std::string>{"H", "W"}));
      EXPECT_FALSE(v.next_x_abandoned("k"));
      EXPECT_TRUE(v.mgr->table().empty());
      EXPECT_FALSE(v.broken);
    }
  }
}

// LX-6: the end hook runs before on_complete, which here reaps the process; a hold left at a
// normal termination (made by the test hook, gap G-6) is counted in leaked_at_exit and released.
TEST_P(Locks, LX06_OnCompleteAndLeakedAtExit) {
  for (const char *variant : kVariants) {
    SCOPED_TRACE(variant);
    hooks_scope hooks;
    checked_world v(GetParam(), hooks.mgr);
    g_hook.home = &v.home();
    int h = -1;
    if (std::string(variant) == "queued")
      h = v.holder("\"k\"", "", "H");
    std::optional<std::size_t> holds_at_complete;
    std::vector<std::string> keys_at_complete;
    int pid = -1;
    checked_world::submit_options so;
    so.opts.on_complete = [&](value_t) {
      holds_at_complete = v.stats().holds;
      keys_at_complete = v.held_keys();
      if (v.sync)
        v.sync->reap(pid);
      else
        v.async->reap(pid);
    };
    pid = v.submit(R"((with-lock "k" (begin (mark "P") 5)))", so);
    g_hook.leak_pid = pid;
    if (h >= 0) {
      ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(pid); }));
      v.release(h);
    }
    const lock_stats s0 = v.stats();
    ASSERT_TRUE(v.step_until([&] { return holds_at_complete.has_value(); }));
    ASSERT_TRUE(v.settle());
    EXPECT_EQ(*holds_at_complete, 0u); // P's locks (its leaked one too) went before on_complete
    EXPECT_TRUE(std::find(keys_at_complete.begin(), keys_at_complete.end(), "leak") ==
                keys_at_complete.end());
    EXPECT_TRUE(std::find(keys_at_complete.begin(), keys_at_complete.end(), "k") ==
                keys_at_complete.end());
    EXPECT_FALSE(v.info(pid).has_value()); // reaped by on_complete
    EXPECT_EQ(v.stats().leaked_at_exit - s0.leaked_at_exit, 1u);
    if (h >= 0)
      ASSERT_TRUE(v.step_until([&] { return v.finished(h); }));
    EXPECT_TRUE(v.mgr->table().empty());
    EXPECT_TRUE(v.held_keys().empty());
    EXPECT_FALSE(v.broken);
  }
}

// LX-7: a scheduler destroyed with holders and waiters (one granted, not resumed, in the second
// variant): its home's one detach releases and cancels everything, on a private manager (kept
// here past the scheduler) and on one the host gave it.
TEST_P(Locks, LX07_SchedulerDestroyed) {
  for (const bool host_manager : {false, true}) {
    for (const char *variant : kVariants) {
      SCOPED_TRACE(std::string(host_manager ? "host manager, " : "private manager, ") + variant);
      std::shared_ptr<lock_manager> mgr;
      lock_stats s0;
      {
        checked_world v(GetParam(), host_manager ? std::make_shared<lock_manager>() : nullptr);
        mgr = v.mgr;
        v.holder("\"a\"", "", "A");
        v.holder("\"b\"", ":shared", "B");
        v.queued("\"a\"", "", "WA");
        v.queued("\"b\"", "", "WB");
        if (std::string(variant) == "granted") {
          const int c = v.holder("\"c\"", "", "C");
          const int wc = v.queued("\"c\"", "", "WC");
          v.release(c);
          ASSERT_TRUE(v.step_until([&] { return v.held(wc) == 1; })); // granted, not resumed
        }
        s0 = mgr->stats();
        EXPECT_GT(s0.holds, 0u);
        EXPECT_EQ(s0.queued, 2u);
        EXPECT_FALSE(v.broken);
        if (v.sync)
          v.sync.reset();
        else
          v.async.reset();
      }
      EXPECT_NO_THROW(mgr->check());
      EXPECT_TRUE(mgr->table().empty()) << mgr->table().canonical();
      const lock_stats s1 = mgr->stats();
      EXPECT_EQ(s1.holds, 0u);
      EXPECT_EQ(s1.queued, 0u);
      EXPECT_EQ(s1.abandoned - s0.abandoned, 1u); // A's X section had started; B's is S
      EXPECT_EQ(s1.cache_refs, 0u);               // the home's cache references dropped
    }
  }
}

// LX-8: a request whose second key would raise is refused before the manager sees any key; an
// error injected into the lock_enter step (after a wait) or into the step that granted at once
// fails that step, whose unwind gives the grant back: nothing is held after.
TEST_P(Locks, LX08_ErrorsBeforeAndAfterTheGrant) {
  {
    SCOPED_TRACE("upgrade on the second key");
    checked_world v(GetParam());
    const lock_stats s0 = v.stats();
    EXPECT_EQ(as_str(v.run(R"((with-lock "b" :shared
                                 (try (with-lock (list "a" "b.c") 1)
                                   (catch (LockError e) (get-attr e "reason")))))")),
              "upgrade");
    const lock_stats s1 = v.stats();
    EXPECT_EQ(s1.grants - s0.grants, 1u); // b only: a was never granted
    EXPECT_EQ(s1.waits, s0.waits);
    EXPECT_TRUE(v.mgr->table().empty());
  }
  {
    SCOPED_TRACE("the handler rule on the second key");
    checked_world v(GetParam());
    checked_world::submit_options so;
    so.opts.signal_handlers["SIGUSR1"] = parse(R"((state-set "h"
        (try (with-lock (list "a" "b.x") "granted") (catch (LockError e) (get-attr e "reason")))))");
    const int p = v.submit(
        R"((with-lock "b" (begin (mark "P") (while (not (state-exists "stop")) (set i 1)) "done")))",
        so);
    ASSERT_TRUE(v.step_until([&] { return v.held(p) == 1; }));
    const lock_stats s0 = v.stats();
    v.send_signal(p, "SIGUSR1");
    ASSERT_TRUE(v.step_until([&] { return v.tree("h").value() != ""; }));
    EXPECT_EQ(v.tree("h").value(), "handler");
    EXPECT_EQ(v.stats().grants, s0.grants); // a was never granted
    v.tree("stop").value("1");
    ASSERT_TRUE(v.step_until([&] { return v.finished(p); }));
    EXPECT_TRUE(v.mgr->table().empty());
  }
  for (const char *variant : kVariants) {
    SCOPED_TRACE(std::string("injected, ") + variant);
    hooks_scope hooks;
    checked_world v(GetParam(), hooks.mgr);
    const bool queued = std::string(variant) == "queued";
    int h = -1;
    if (queued)
      h = v.holder("\"k\"", "", "H");
    const int p = v.submit(R"((list (try (with-lock "k" (mark "P-body"))
                                       (catch (RuntimeError e) (get-attr e "message")))
                                     (locks-held)))");
    if (queued) {
      // Into the lock_enter step, after the wait's grant.
      g_hook.throw_in_enter_pid = p;
      ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(p); }));
      v.release(h);
    } else {
      // Into the step that grants at once, once the grant has joined the mirror.
      g_hook.throw_after_grant_pid = p;
    }
    ASSERT_TRUE(v.step_until([&] { return v.finished(p); }));
    const auto r = as_list(v.result(p));
    ASSERT_EQ(r.size(), 2u);
    EXPECT_NE(as_str(r[0]).find("injected"), std::string::npos);
    EXPECT_TRUE(as_list(r[1]).empty()) << "the injected step's grant is still held after the catch";
    EXPECT_EQ(std::count(v.trace.begin(), v.trace.end(), "P-body"), 0);
    EXPECT_TRUE(v.held_keys().empty()) << "the injected step's grant was not given back";
    EXPECT_EQ(v.stats().leaked_at_exit, 0u);
    if (h >= 0)
      ASSERT_TRUE(v.step_until([&] { return v.finished(h); }));
    EXPECT_FALSE(v.next_x_abandoned("k"));
    EXPECT_TRUE(v.mgr->table().empty());
    EXPECT_FALSE(v.broken);
  }
}

// LX-11 (same home): a wake for a ticket the process no longer waits on is ignored. The ticket was
// adopted by its timeout (queued variant) or collected by its grant (granted variant); the process
// then queues a new request, and the stale wake, applied to the home, leaves it queued until its
// own grant.
TEST_P(Locks, LX11_StaleWakeIgnored) {
  for (const char *variant : kVariants) {
    SCOPED_TRACE(variant);
    checked_world v(GetParam());
    const bool queued = std::string(variant) == "queued";
    const int h = v.holder("\"k\"", "", "H");
    const int p = v.submit(std::string(R"((begin
        (try (with-lock "k" :timeout )") +
                           (queued ? "0.1" : "30") + R"( (msg-recv "p.first"))
          (catch (TimeoutError e) nil))
        (with-lock "k" (mark "P-second"))))");
    ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(p); }));
    const detail::lock_pid P{v.home().id(), p};
    const std::uint64_t old_tid = *v.mgr->table().rec_of(P)->ticket;
    if (queued) {
      // Times out; the second request queues behind H.
      ASSERT_TRUE(v.step_until([&] {
        const auto rec = v.mgr->table().rec_of(P);
        return v.lock_waiting(p) && rec && rec->ticket && *rec->ticket != old_tid;
      }));
    } else {
      v.release(h);
      ASSERT_TRUE(v.step_until([&] { return v.inside(p); })); // the grant collected
      // The second request must wait: X takes k when P's first section ends.
      const int x = v.submit(R"((with-lock "k" (msg-recv "x.go")))");
      ASSERT_TRUE(v.step_until([&] { return v.lock_waiting(x); }));
      v.send("p.first");
      ASSERT_TRUE(v.step_until([&] { return v.inside(x) && v.lock_waiting(p); }));
      detail::lock_notes stale;
      stale.push_back({v.home().id(), nullptr, {detail::lock_post::kind::wake, p, old_tid}});
      v.home().deliver(stale);
      ASSERT_TRUE(v.check_all());
      EXPECT_TRUE(v.lock_waiting(p));
      v.send("x.go");
      ASSERT_TRUE(v.step_until([&] { return v.finished(p) && v.finished(x); }));
      EXPECT_EQ(v.trace.back(), "P-second");
      EXPECT_FALSE(v.broken);
      continue;
    }
    detail::lock_notes stale;
    stale.push_back({v.home().id(), nullptr, {detail::lock_post::kind::wake, p, old_tid}});
    v.home().deliver(stale);
    ASSERT_TRUE(v.check_all());
    EXPECT_TRUE(v.lock_waiting(p)); // the new request stays queued
    EXPECT_EQ(v.held(p), 0u);
    ASSERT_TRUE(v.settle());
    EXPECT_TRUE(v.lock_waiting(p));
    v.release(h); // its own grant
    ASSERT_TRUE(v.step_until([&] { return v.finished(p); }));
    EXPECT_EQ(v.trace, (std::vector<std::string>{"H", "P-second"}));
    EXPECT_FALSE(v.broken);
  }
}

// ---------------------------------------------------------------------------
// LW (13.4): where waits are allowed, handlers, fork, snapshots
// ---------------------------------------------------------------------------

// LW-1: where a process cannot park (a nested evaluator, park_forbidden_scope), an uncontended
// with-lock works, a contended one raises LockError("cannot wait here") without waiting, and
// :if-available takes :else. Nothing is queued. (Ariadne's reactive lane: ariadne_runtime_test.)
TEST_P(Locks, LW01_CannotWaitHere) {
  // (MSVC's preprocessor misreads a raw string with a delimiter inside a macro's arguments.)
  const std::string nested = R"lw((nested-eval "(with-lock \"k\" 1)"))lw";
  const std::string nested_caught = R"lw((try (nested-eval "(with-lock \"k\" 1)")
                                           (catch (LockError e) (get-attr e "reason"))))lw";
  const std::string nested_else =
      R"lw((nested-eval "(with-lock \"k\" :if-available :else \"else\" 1)"))lw";
  EXPECT_EQ(as_int(w.run(nested)), 1);
  const int h = w.holder("\"k\"", "", "H");
  const lock_stats s0 = w.stats();
  EXPECT_EQ(as_str(w.run(nested_caught)), "cannot wait here");
  EXPECT_EQ(as_str(w.run(nested_else)), "else");
  {
    const park_forbidden_scope no_parking;
    EXPECT_EQ(
        as_str(w.run(R"((try (with-lock "k" 1) (catch (LockError e) (get-attr e "reason"))))")),
        "cannot wait here");
    EXPECT_EQ(as_str(w.run(R"((with-lock "k" :if-available :else "else" 1))")), "else");
    EXPECT_EQ(as_int(w.run(R"((with-lock "j" 2))")), 2);
  }
  EXPECT_EQ(w.stats().waits, s0.waits); // nothing queued
  EXPECT_EQ(w.stats().queued, 0u);
  EXPECT_EQ(w.held_keys(), (std::vector<std::string>{"k"}));
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.finished(h); }));
  EXPECT_TRUE(w.mgr->table().empty());
}

// LW-1b: a wait inside a DSL function a native called runs as frames of the calling process: it
// parks there, is granted on release, and the native's job completes.
TEST_P(Locks, LW01b_WaitInsideCollectionsMap) {
  const int h = w.holder("\"k\"", "", "H");
  const int p = w.submit(R"((collections.map (list 1 2) (lambda (x) (with-lock "k" x))))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(p); }));
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, p}); }));
  EXPECT_EQ(to_string(w.result(p)), "(1 2)");
  EXPECT_TRUE(w.mgr->table().empty());
}

// LW-2: with-lock with no stepping process; %lock-wait called by a program (no lock form's token):
// at top level, as with-lock's KEY, as its :timeout. LockError each time; nothing queued, held or
// mirrored.
TEST_P(Locks, LW02_NoProcessAndDirectLockWait) {
  {
    environment_ptr env = w.natives();
    stackless_evaluator ev(env);
    try {
      ev.evaluate_script(R"((with-lock "k" 1))");
      ADD_FAILURE() << "with-lock ran with no process stepping";
    } catch (const dsl_exception &e) {
      EXPECT_EQ(e.class_name(), "LockError");
      EXPECT_EQ(reason_of(e), "no process");
    }
  }
  const int h = w.holder("\"k\"", "", "H");
  const lock_stats s0 = w.stats();
  const std::string state0 = w.state_without_cache();
  for (const char *form :
       {R"((%lock-wait "k" "exclusive" nil))", R"((with-lock (%lock-wait "k" "exclusive" nil) 1))",
        R"((with-lock "j" :timeout (%lock-wait "k" "exclusive" nil) 1))"}) {
    const auto r = as_list(w.run(std::string("(list (try ") + form +
                                 R"( (catch (LockError e) "refused")) (locks-held)))"));
    ASSERT_EQ(r.size(), 2u);
    EXPECT_EQ(as_str(r[0]), "refused") << form;
    EXPECT_TRUE(as_list(r[1]).empty()) << form;
    expect_same_lock_state(s0, w.stats());
    EXPECT_EQ(w.state_without_cache(), state0) << form;
  }
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.finished(h); }));
}

namespace {
// LW-3 and LW-4: a program holding "k" (MODE) spins in its section; a handler of it (signal or
// watch) asks for KEY and stores what came of it in "h".
struct handler_case {
  const char *program_mode;
  const char *request; // the handler's with-lock key and options
  const char *expect;
};
const handler_case kHandlerCases[] = {
    {"", R"("k" :shared)", "handler"},        // the program's key, covered by its X hold
    {"", R"("k.x")", "handler"},              // under its X hold: covered and conflicting
    {":shared", R"("k.x")", "handler"},       // conflicting with its S hold (not covered)
    {":shared", R"("k" :shared)", "handler"}, // covered by its S hold
    {"", R"("z")", "granted"},                // unrelated: granted
    {"", R"((list "z" "y"))", "granted"},
};
std::string handler_body(const handler_case &c) {
  return std::string(R"((state-set "h" (try (with-lock )") + c.request +
         R"( "granted") (catch (LockError e) (get-attr e "reason")))))";
}
} // namespace

TEST_P(Locks, LW03_SignalHandlerRule) {
  for (const handler_case &c : kHandlerCases) {
    SCOPED_TRACE(std::string(c.program_mode) + " / " + c.request);
    checked_world v(GetParam());
    checked_world::submit_options so;
    so.opts.signal_handlers["SIGUSR1"] = parse(handler_body(c));
    const int p =
        v.submit(std::string(R"((with-lock "k" )") + c.program_mode +
                     R"( (begin (mark "P") (while (not (state-exists "stop")) (set i 1)) "done")))",
                 so);
    ASSERT_TRUE(v.step_until([&] { return v.held(p) == 1; }));
    const lock_stats s0 = v.stats();
    v.send_signal(p, "SIGUSR1");
    ASSERT_TRUE(v.step_until([&] { return v.tree("h").value() != ""; }));
    EXPECT_EQ(v.tree("h").value(), c.expect);
    if (std::string(c.expect) == "handler")
      EXPECT_EQ(v.stats().grants, s0.grants); // refused before the manager saw it
    EXPECT_EQ(v.held(p), 1u);                 // the handler's own hold went with its frames
    v.tree("stop").value("1");
    ASSERT_TRUE(v.step_until([&] { return v.finished(p); }));
    EXPECT_EQ(as_str(v.result(p)), "done");
    EXPECT_TRUE(v.mgr->table().empty());
    EXPECT_FALSE(v.broken);
  }
}

// LW-4: the same with a watch handler, registered by the process under test through its own
// context, so the handler runs in that process.
TEST_P(Locks, LW04_WatchHandlerRule) {
  for (const handler_case &c : kHandlerCases) {
    SCOPED_TRACE(std::string(c.program_mode) + " / " + c.request);
    checked_world v(GetParam());
    const int p = v.submit(std::string("(begin (state-watch \"w.n\" (lambda (path val) ") +
                           handler_body(c) + ")) (with-lock \"k\" " + c.program_mode +
                           R"( (begin (state-set "ready" "1")
                                 (while (not (state-exists "stop")) (set i 1)) "done"))))");
    ASSERT_TRUE(v.step_until([&] { return v.tree("ready").value() == "1"; }));
    ASSERT_EQ(v.held(p), 1u);
    v.tree("w.n").value("changed");
    ASSERT_TRUE(v.step_until([&] { return v.tree("h").value() != ""; }))
        << "the watch handler never ran";
    EXPECT_EQ(v.tree("h").value(), c.expect);
    EXPECT_EQ(v.held(p), 1u);
    v.tree("stop").value("1");
    ASSERT_TRUE(v.step_until([&] { return v.finished(p); }));
    EXPECT_EQ(as_str(v.result(p)), "done");
    EXPECT_TRUE(v.mgr->table().empty());
    EXPECT_FALSE(v.broken);
  }
}

TEST_P(Locks, LW05_PauseHolderAndWaiter) {
  const int h = w.submit(
      R"((with-lock "k" (begin (mark "H") (while (not (state-exists "stop")) (set i 1)) "done")))");
  ASSERT_TRUE(w.step_until([&] { return w.held(h) == 1; }));
  ASSERT_TRUE(w.base().pause(h));
  const int q = w.queued("\"k\"", "", "W");
  ASSERT_TRUE(w.settle());
  EXPECT_EQ(w.status(h), process_status::paused);
  EXPECT_EQ(w.held(h), 1u); // a paused holder keeps its lock
  EXPECT_TRUE(w.lock_waiting(q));
  EXPECT_FALSE(w.base().pause(q)); // a waiter can't be paused
  EXPECT_TRUE(w.lock_waiting(q));
  ASSERT_TRUE(w.base().resume(h));
  w.tree("stop").value("1");
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"H", "W"}));
}

// LW-6: fork inside with-lock is refused; outside, it forks the running process whatever the
// context's pid says (a host's placeholder: pycvc's 0, Ariadne's -1), and the child keeps the
// parent's owner, so kill_owner reaps it.
//
// The child's (fork) returns 0 and it goes on from there (td B28:
// LocksReview.ForkInAStepReturnsZeroInTheChild), with its own copies of the parent's scopes
// (detail::fork_scopes: a set in the child never lands in the parent's). Here the child is checked
// and reaped before it steps.
TEST_P(Locks, LW06_Fork) {
  const std::size_t before = w.base().list_processes().size();
  EXPECT_EQ(
      as_str(w.run(R"((with-lock "k" (try (fork) (catch (LockError e) (get-attr e "reason")))))")),
      "fork");
  EXPECT_EQ(w.base().list_processes().size(), before + 1); // no child
  EXPECT_TRUE(w.mgr->table().empty());
  for (const std::optional<int> ctx_pid :
       {std::optional<int>(), std::optional<int>(0), std::optional<int>(-1)}) {
    SCOPED_TRACE(ctx_pid ? "ctx->pid " + std::to_string(*ctx_pid) : std::string("ctx->pid set"));
    checked_world v(GetParam());
    checked_world::submit_options so;
    so.opts.owner = "docA";
    so.ctx_pid = ctx_pid;
    const int p = v.submit(R"((begin (set c (fork)) (msg-recv "never")))", so);
    ASSERT_TRUE(v.step_until([&] { return v.base().list_processes().size() == 2; }));
    std::optional<process_info> child;
    for (const process_info &i : v.base().list_processes())
      if (i.pid != p)
        child = i;
    ASSERT_TRUE(child.has_value());
    EXPECT_EQ(child->parent_pid, p); // the running process forked
    cvc::state &node = v.tree("snap.fork");
    ASSERT_TRUE(v.snapshot(child->pid, node));
    EXPECT_EQ(node("owner").value(), "docA");
    if (v.async) {
      EXPECT_EQ(v.async->kill_owner("docA"), 2); // the parent and the child
      EXPECT_EQ(v.exit_error(child->pid).value_or(""), "owner_reaped");
    } else {
      v.kill(child->pid);
      v.kill(p);
    }
    EXPECT_TRUE(v.finished(child->pid));
    EXPECT_FALSE(v.broken);
  }
}

// LW-7: snapshots. A holder's is refused, naming its keys. A waiter's (holding nothing) restores
// as a new process that queues afresh at the tail: in place, through a serialized round trip, and
// into another scheduler, whose locks it then uses; restored and snapshotted again before its
// next step, it queues afresh the same way. The snapshot keeps the wait's remaining timeout.
TEST_P(Locks, LW07_Snapshots) {
  const int h = w.holder("(list \"k\" \"m.n\")", "", "H");
  try {
    w.snapshot(h, w.tree("snap.holder"));
    ADD_FAILURE() << "a holder was snapshotted";
  } catch (const std::runtime_error &e) {
    const std::string what = e.what();
    EXPECT_NE(what.find("holds locks"), std::string::npos) << what;
    EXPECT_NE(what.find("/k"), std::string::npos) << what;
    EXPECT_NE(what.find("/m.n"), std::string::npos) << what;
  }
  const std::string waiter =
      R"((begin (with-lock "k" (mark (str-concat "k " (str (self-pid)))))
                (with-lock "after" (mark (str-concat "after " (str (self-pid)))))))";
  const int q = w.submit(waiter);
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(q); }));

  // In place.
  cvc::state &n1 = w.tree("snap.w1");
  ASSERT_TRUE(w.snapshot(q, n1));
  const int r1 = w.restore(n1, w.envs.at(q));
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(r1); }));
  // Through a serialized round trip.
  cvc::state &n2 = w.tree("snap.w2");
  n2.json(n1.json());
  const int r2 = w.restore(n2, w.envs.at(q));
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(r2); }));
  // Restored, snapshotted again before its first step, and that snapshot restored.
  const int r3a = w.restore(n1, w.envs.at(q));
  ASSERT_TRUE(w.snapshot(r3a, w.tree("snap.w3")));
  w.kill(r3a);
  const int r3 = w.restore(w.tree("snap.w3"), w.envs.at(q));
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(r3); }));
  EXPECT_EQ(w.stats().queued, 4u);

  // Into another scheduler: it queues there (k is free there: granted), and its next with-lock
  // uses that scheduler's locks too.
  {
    checked_world other(GetParam());
    const int r = other.restore(n1, other.natives());
    ASSERT_TRUE(other.step_until([&] { return other.finished(r); }));
    EXPECT_EQ(other.status(r), process_status::terminated);
    EXPECT_EQ(other.trace,
              (std::vector<std::string>{"k " + std::to_string(r), "after " + std::to_string(r)}));
    EXPECT_EQ(other.stats().grants, 2u);
    EXPECT_FALSE(other.broken);
  }
  EXPECT_EQ(w.stats().queued, 4u); // nothing of it here

  // The remaining timeout, structurally: a waiter with :timeout 30, snapshotted after at least
  // 1 s, writes a TIMEOUT above 0 and at most 29 in its wait child.
  const int t30 =
      w.submit(R"((try (with-lock "k" :timeout 30 1) (catch (TimeoutError e) "timeout")))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(t30); }));
  std::this_thread::sleep_for(std::chrono::milliseconds(1050));
  cvc::state &n30 = w.tree("snap.t30");
  ASSERT_TRUE(w.snapshot(t30, n30));
  {
    const process_ptr pp = decode_process(n30);
    ASSERT_FALSE(pp->state.stack.empty());
    const eval_frame &wait = pp->state.stack.back();
    ASSERT_EQ(wait.phase, eval_phase::await_call);
    ASSERT_EQ(wait.extra_lists.size(), 1u);
    ASSERT_EQ(wait.extra_lists[0].size(), 3u);
    const double left = as_float(wait.extra_lists[0][2]);
    std::printf("LW-7: a :timeout 30 wait snapshotted after 1 s keeps %.3f s\n", left);
    EXPECT_GT(left, 0.0);
    EXPECT_LE(left, 29.0);
  }
  w.kill(t30);
  // A waiter with :timeout 1.0, snapshotted after 0.5 s and restored, times out no earlier than
  // the encoded time after the restore.
  const int t1 =
      w.submit(R"((try (with-lock "k" :timeout 1.0 1) (catch (TimeoutError e) "timeout")))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(t1); }));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  cvc::state &n1s = w.tree("snap.t1");
  ASSERT_TRUE(w.snapshot(t1, n1s));
  const double encoded = as_float(decode_process(n1s)->state.stack.back().extra_lists[0][2]);
  w.kill(t1);
  const auto t0 = steady::now();
  const int rt = w.restore(n1s, w.envs.at(t1));
  ASSERT_TRUE(w.step_until([&] { return w.finished(rt); }, 20.0));
  const double took = seconds_since(t0);
  std::printf("LW-7: encoded %.3f s left; the restored wait timed out %.3f s after the restore\n",
              encoded, took);
  EXPECT_EQ(as_str(w.result(rt)), "timeout");
  EXPECT_GE(took, encoded);
  EXPECT_LT(took, 10.0);

  // The queue: q, then the restored copies, each at the tail.
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q, r1, r2, r3}); }));
  std::vector<std::string> k_order;
  for (const std::string &t : w.trace)
    if (t.rfind("k ", 0) == 0)
      k_order.push_back(t.substr(2));
  EXPECT_EQ(k_order, (std::vector<std::string>{std::to_string(q), std::to_string(r1),
                                               std::to_string(r2), std::to_string(r3)}));
  EXPECT_TRUE(w.mgr->table().empty());
}

// LW-7b (gap G-44): a parked waiter restored where with-lock is not the lock form: a natives scope
// without the lock intrinsics, or the program's own with-lock shadowing it. LockError("restore")
// at its first step, caught by a try around the form; nothing queued, held or mirrored.
TEST_P(Locks, LW07b_RestoreWithoutTheLockForm) {
  const int h = w.holder("\"k\"", "", "H");
  const int q =
      w.submit(R"((list (try (with-lock "k" 1) (catch (LockError e) (get-attr e "reason"))) 2))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(q); }));
  cvc::state &node = w.tree("snap.q");
  ASSERT_TRUE(w.snapshot(q, node));
  const lock_stats s0 = w.stats();
  // A natives scope with no lock intrinsics.
  const int r1 = w.restore(node, builtins::make_default_environment());
  ASSERT_TRUE(w.step_until([&] { return w.finished(r1); }));
  EXPECT_EQ(to_string(w.result(r1)), R"(("restore" 2))");
  // The program's own with-lock shadowing the form where it runs (a function of that name in
  // the lock frame's scope).
  const process_ptr pp = decode_process(node);
  rebind_natives(pp->state, w.envs.at(q));
  ASSERT_GE(pp->state.stack.size(), 2u);
  eval_frame &enter = pp->state.stack[pp->state.stack.size() - 2];
  ASSERT_EQ(enter.phase, eval_phase::lock_enter);
  stackless_evaluator ev(builtins::make_default_environment());
  enter.env->set("with-lock", ev.evaluate_script("(lambda (k b) 0)"));
  const int r2 = w.restore(pp);
  ASSERT_TRUE(w.step_until([&] { return w.finished(r2); }))
      << "status " << int(w.status(r2)) << ", waiting on " << w.waiting_on(r2) << ", held "
      << w.held(r2);
  EXPECT_EQ(to_string(w.result(r2)), R"(("restore" 2))");
  const lock_stats s1 = w.stats();
  EXPECT_EQ(s1.waits, s0.waits);
  EXPECT_EQ(s1.queued, 1u); // q only
  EXPECT_EQ(s1.grants, s0.grants);
  EXPECT_TRUE(w.check_all());
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q}); }));
  EXPECT_TRUE(w.mgr->table().empty());
}

// LW-10: with-lock in a closure generator's body is refused, even uncontended (a generator can be
// dropped with its frames alive); around (next g) it works.
TEST_P(Locks, LW10_GeneratorBodies) {
  EXPECT_EQ(as_str(w.run(R"((begin (set g (generator (lambda () (with-lock "k" (yield 1)))))
                               (try (next g) (catch (LockError e) (get-attr e "reason")))))")),
            "generator");
  EXPECT_EQ(
      as_str(w.run(R"((begin (set g (generator (lambda () (with-lock "k" (yield 1) (yield 2)))))
                               (set r (try (next g) (catch (LockError e) (get-attr e "reason"))))
                               (set g nil)
                               r))")),
      "generator");
  // Shared across a fork (made here by the host between steps, which puts the fork at a known
  // point): whichever process steps the generator first is refused, and the other finds it ended.
  const int p = w.submit(R"((begin (set g (generator (lambda () (with-lock "k" (yield 1)))))
                              (state-set "ready" "1")
                              (while (not (state-exists "go")) (set i 1))
                              (try (next g) (catch (LockError e) (get-attr e "reason")))))");
  ASSERT_TRUE(w.step_until([&] { return w.tree("ready").value() == "1"; }));
  const int child = w.sync ? w.sync->fork(p) : w.async->fork(p);
  ASSERT_GT(child, 0);
  w.tree("go").value("1");
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({p, child}); }));
  const std::string rp = to_string(w.result(p)), rc = to_string(w.result(child));
  EXPECT_TRUE(rp == R"("generator")" || rc == R"("generator")") << rp << " " << rc;
  EXPECT_TRUE(w.held_keys().empty());
  EXPECT_TRUE(w.mgr->table().empty());
  EXPECT_EQ(as_int(w.run(R"((begin (set g (generator (lambda () (yield 1) (yield 2))))
                              (with-lock "k" (next g))))")),
            1);
}

// LW-11 (C++): the recursive evaluator, with the intrinsics bound, refuses with-lock before it
// evaluates the body.
TEST(Locks, LW11_RecursiveEvaluator) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  scheduler sched;
  intrinsics_context ctx;
  ctx.sched = &sched;
  ctx.root = &tree;
  ctx.proc = make_process();
  environment_ptr env = builtins::make_default_environment();
  register_intrinsics(env, &ctx);
  evaluator ev(env);
  try {
    ev.evaluate_script(R"((with-lock "k" (state-set "x" 1)))");
    ADD_FAILURE() << "the recursive evaluator ran with-lock";
  } catch (const dsl_exception &e) {
    EXPECT_EQ(e.class_name(), "TypeError");
    EXPECT_NE(std::string(e.what()).find("with-lock needs a scheduler"), std::string::npos)
        << e.what();
  }
  EXPECT_EQ(tree.findDescendant("x"), nullptr);
}

// LW-12: a restore of a stack whose lock frame recorded a hold (no snapshot makes one; built by
// hand from a waiter's) is refused; a lock_body of :else, holding nothing, runs.
TEST_P(Locks, LW12_RestoreOfAHeldBody) {
  const int h = w.holder("\"k\"", "", "H");
  const int q = w.submit(R"((with-lock "k" 7))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(q); }));
  cvc::state &node = w.tree("snap.q");
  ASSERT_TRUE(w.snapshot(q, node));
  const auto lock_frame_of = [&](const process_ptr &pp) -> eval_frame & {
    auto &st = pp->state.stack;
    EXPECT_GE(st.size(), 2u);
    EXPECT_EQ(st.back().phase, eval_phase::await_call);
    st.pop_back(); // the wait child
    EXPECT_EQ(st.back().phase, eval_phase::lock_enter);
    return st.back();
  };
  {
    const process_ptr pp = decode_process(node);
    eval_frame &f = lock_frame_of(pp);
    f.phase = eval_phase::lock_body;
    f.extra_vals[6] = value_t(int64_t(99)); // lock_frame_slot::hold
    EXPECT_THROW(w.restore(pp), std::invalid_argument);
  }
  {
    const process_ptr pp = decode_process(node);
    eval_frame &f = lock_frame_of(pp);
    f.extra_vals[3] = value_t(int64_t(99)); // a lock_enter woken with a grant
    EXPECT_THROW(w.restore(pp), std::invalid_argument);
  }
  {
    const process_ptr pp = decode_process(node);
    rebind_natives(pp->state, w.envs.at(q));
    eval_frame &f = lock_frame_of(pp);
    f.phase = eval_phase::lock_body; // :else's, holding nothing
    f.extra_vals.resize(7);
    f.extra_vals.push_back(value_t(std::string("else")));
    f.index = -1;
    f.results.clear();
    const int r = w.restore(pp);
    ASSERT_TRUE(w.step_until([&] { return w.finished(r); }));
    EXPECT_EQ(as_str(w.result(r)), "else");
  }
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q}); }));
  EXPECT_TRUE(w.mgr->table().empty());
}

// ---------------------------------------------------------------------------
// LD (13.5): deadlocks
// ---------------------------------------------------------------------------

namespace {
// A DeadlockError's cycle: its pids (nil as -1) and keys, in order.
struct cycle_view {
  std::vector<int> pids;
  std::vector<std::string> keys;
};
cycle_view cycle_of(const value_t &cycle) {
  cycle_view c;
  for (const value_t &e : as_list(cycle)) {
    const value_t pid = entry_of(e, "pid");
    c.pids.push_back(pid.is_nil() ? -1 : static_cast<int>(as_int(pid)));
    c.keys.push_back(as_str(entry_of(e, "key")));
  }
  return c;
}
std::string show_cycle(const cycle_view &c) {
  std::string out;
  for (std::size_t i = 0; i < c.pids.size(); ++i)
    out += "{" + std::to_string(c.pids[i]) + " " + c.keys[i] + "} ";
  return out;
}
template <class T> std::set<T> as_set(const std::vector<T> &v) { return {v.begin(), v.end()}; }
} // namespace

TEST_P(Locks, LD01_TwoInOppositeOrder) {
  const int a =
      w.submit(R"((with-lock "a" (begin (msg-recv "a1") (with-lock "b" (mark "A-b")) "a-done")))");
  const int b = w.submit(R"((with-lock "b" (begin (msg-recv "b1")
                              (try (with-lock "a" (mark "B-a"))
                                (catch (DeadlockError e) (state-set "caught" "1") "deadlock")
                                (finally (state-set "fin" "1"))))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(a) && w.inside(b); }));
  const lock_stats s0 = w.stats();
  w.send("a1");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(a); }));
  w.send("b1"); // b closes the cycle
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({a, b}); }));
  EXPECT_EQ(as_str(w.result(a)), "a-done");
  EXPECT_EQ(as_str(w.result(b)), "deadlock");
  EXPECT_EQ(w.tree("caught").value(), "1");
  EXPECT_EQ(w.tree("fin").value(), "1");
  EXPECT_EQ(w.stats().deadlocks - s0.deadlocks, 1u);
  EXPECT_EQ(w.trace, (std::vector<std::string>{"A-b"}));
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LD02_CycleOfEightAndThroughShared) {
  // P_i holds k_i, then asks for k_(i+1 mod 8); P_7 closes the cycle.
  std::vector<int> ps;
  for (int i = 0; i < 8; ++i)
    ps.push_back(w.submit("(with-lock \"k" + std::to_string(i) + "\" (begin (msg-recv \"go" +
                          std::to_string(i) + "\") (try (with-lock \"k" +
                          std::to_string((i + 1) % 8) +
                          "\" \"ok\") (catch (DeadlockError e) (get-attr e \"cycle\")))))"));
  ASSERT_TRUE(w.step_until(
      [&] { return std::all_of(ps.begin(), ps.end(), [&](int p) { return w.inside(p); }); }));
  for (int i = 0; i < 7; ++i) {
    w.send("go" + std::to_string(i));
    ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(ps[i]); }));
  }
  w.send("go7");
  ASSERT_TRUE(w.step_until([&] { return w.all_finished(ps); }));
  for (int i = 0; i < 7; ++i)
    EXPECT_EQ(as_str(w.result(ps[i])), "ok");
  const cycle_view c = cycle_of(w.result(ps[7]));
  std::printf("LD-2: the cycle of 8: %s\n", show_cycle(c).c_str());
  // From the requester round the cycle, each process with the key it waits for; the same owner,
  // so every pid shown.
  std::vector<int> pids{ps[7]};
  std::vector<std::string> keys;
  for (int i = 0; i < 8; ++i) {
    keys.push_back("k" + std::to_string(i));
    if (i < 7)
      pids.push_back(ps[i]);
  }
  EXPECT_EQ(c.pids, pids);
  EXPECT_EQ(c.keys, keys);

  // Through an S lock with two holders: A and B hold S s; C holds X t and waits for X s (behind
  // both); B asks for X t and closes B -> C -> B.
  const int a = w.holder("\"s\"", ":shared", "A");
  const int b = w.submit(R"((with-lock "s" :shared (begin (msg-recv "b.go")
                              (try (with-lock "t" "ok") (catch (DeadlockError e) (get-attr e "cycle"))))))");
  const int cc = w.submit(R"((with-lock "t" (begin (msg-recv "c.go") (with-lock "s" "C-s"))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(b) && w.inside(cc); }));
  w.send("c.go");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(cc); }));
  w.send("b.go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(b); }));
  const cycle_view c2 = cycle_of(w.result(b));
  std::printf("LD-2: the cycle through S: %s\n", show_cycle(c2).c_str());
  EXPECT_EQ(c2.pids, (std::vector<int>{b, cc}));
  EXPECT_EQ(c2.keys, (std::vector<std::string>{"t", "s"}));
  w.release(a);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({a, cc}); }));
  EXPECT_EQ(as_str(w.result(cc)), "C-s");
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LD03_TwoBroadeners) {
  const int p = w.submit(R"((with-lock "a.b" (begin (msg-recv "p.go")
                              (try (with-lock "a" "P-a") (catch (DeadlockError e) "deadlock")))))");
  const int q = w.submit(R"((with-lock "a.c" (begin (msg-recv "q.go")
                              (try (with-lock "a" "Q-a") (catch (DeadlockError e) "deadlock")))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(p) && w.inside(q); }));
  w.send("p.go");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(p); }));
  w.send("q.go");
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({p, q}); }));
  EXPECT_EQ(as_str(w.result(q)), "deadlock"); // the closer
  EXPECT_EQ(as_str(w.result(p)), "P-a");      // granted once the victim unwound
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LD04_LongChainNoCycle) {
  constexpr int n = 32;
  std::vector<int> ps;
  for (int i = 0; i < n; ++i) {
    const std::string k = "c" + std::to_string(i), go = "go" + std::to_string(i);
    ps.push_back(
        i + 1 < n ? w.submit("(with-lock \"" + k + "\" (begin (msg-recv \"" + go +
                             "\") (with-lock \"c" + std::to_string(i + 1) + "\" \"ok\")))")
                  : w.submit("(with-lock \"" + k + "\" (begin (msg-recv \"" + go + "\") \"ok\"))"));
  }
  ASSERT_TRUE(w.step_until(
      [&] { return std::all_of(ps.begin(), ps.end(), [&](int p) { return w.inside(p); }); }));
  const lock_stats s0 = w.stats();
  for (int i = 0; i + 1 < n; ++i) {
    w.send("go" + std::to_string(i));
    ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(ps[i]); }));
  }
  EXPECT_EQ(w.stats().deadlocks, s0.deadlocks); // no false positive
  EXPECT_EQ(w.stats().queued, static_cast<std::size_t>(n - 1));
  w.send("go" + std::to_string(n - 1));
  ASSERT_TRUE(w.step_until([&] { return w.all_finished(ps); }));
  for (int p : ps)
    EXPECT_EQ(as_str(w.result(p)), "ok");
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LD07_ConversionRuleSharedParent) {
  const int q = w.submit(R"((with-lock "a.b.c" :shared (begin (msg-recv "q.go")
                              (with-lock "a" :shared (mark "Q-a")) "q")))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(q); }));
  const int p = w.queued("\"a.b.c\"", "", "P");
  const lock_stats s0 = w.stats();
  w.send("q.go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(q); }));
  EXPECT_EQ(w.stats().waits, s0.waits); // granted at once
  EXPECT_EQ(w.stats().deadlocks, s0.deadlocks);
  ASSERT_TRUE(w.step_until([&] { return w.finished(p); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"Q-a", "P"}));
}

TEST_P(Locks, LD09_CycleThroughWaiterEdge) {
  const int h = w.submit(R"((with-lock "a.b" (begin (msg-recv "h.go")
                              (try (with-lock "z" "H-z") (catch (DeadlockError e) (get-attr e "cycle"))))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(h); }));
  const int v = w.queued("\"a\"", ":shared", "V");
  const int wz = w.submit(R"((with-lock "z" (begin (msg-recv "w.go") (with-lock "a.c" "W-ac"))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(wz); }));
  w.send("w.go");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(wz); })); // behind V (a waiter edge)
  w.send("h.go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(h); }));
  const cycle_view c = cycle_of(w.result(h));
  std::printf("LD-9: %s\n", show_cycle(c).c_str());
  EXPECT_EQ(c.pids, (std::vector<int>{h, wz, v})); // (H, W, V): W's edge to V is a waiter's
  EXPECT_EQ(c.keys, (std::vector<std::string>{"z", "a.c", "a"}));
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({v, wz}); }));
  EXPECT_EQ(as_str(w.result(wz)), "W-ac");
  EXPECT_TRUE(w.mgr->table().empty());
}

TEST_P(Locks, LD10_PairwiseConversionLimit) {
  const int p = w.submit(R"((with-lock "a.x" :shared (begin (msg-recv "p.go")
                              (try (with-lock "a.q.w" "P") (catch (DeadlockError e) (get-attr e "cycle"))))))");
  ASSERT_TRUE(w.step_until([&] { return w.inside(p); }));
  const int r = w.queued("\"a\"", "", "R");
  const int q = w.queued("\"a.q\"", ":shared", "Q");
  w.send("p.go");
  ASSERT_TRUE(w.step_until([&] { return w.finished(p); }));
  const cycle_view c = cycle_of(w.result(p));
  std::printf("LD-10: %s\n", show_cycle(c).c_str());
  EXPECT_EQ(c.pids, (std::vector<int>{p, q, r})); // R is exempt for P, Q is not
  EXPECT_EQ(c.keys, (std::vector<std::string>{"a.q.w", "a.q", "a"}));
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({q, r}); }));
  EXPECT_EQ(w.trace, (std::vector<std::string>{"R", "Q"}));
  EXPECT_TRUE(w.mgr->table().empty());
}

// ===========================================================================
// LockHome: a real lock_home over a fake scheduler (lock_sched_ops) that owns a process table and
// records the home's calls; processes and their lock frames are built by hand. Every action is
// followed by the manager's check(), the home's check_quiescent() and the table's check().
// ===========================================================================

namespace {

struct fake_sched : detail::lock_sched_ops {
  detail::process_table table;
  std::vector<int> parks, readies, relevels;

  bool park_lock(process &p) override {
    parks.push_back(p.pid);
    table.set_status(p, process_status::waiting); // its deadline, if any, a timer
    return true;
  }
  void ready_lock_waiter(process &p) override {
    readies.push_back(p.pid);
    table.set_status(p, process_status::ready);
  }
  void relevel(process &p) override {
    relevels.push_back(p.pid);
    table.update(p);
  }
  std::size_t lock_expired_count() const override { return table.lock_expired_count(); }
  void take_lock_expired(std::vector<process *> &out) override { table.take_lock_expired(out); }
  process *find_process(int pid) override { return table.find(pid); }
  void for_each_process(const std::function<void(process &)> &fn) override {
    for (const auto &[pid, p] : table.all())
      fn(*p);
  }
};

struct home_world {
  hooks_scope hooks; // the cache capacity, read once when the home is made
  std::shared_ptr<lock_manager> mgr = hooks.mgr;
  fake_sched sched;
  std::unique_ptr<detail::lock_home> home; // goes before the processes (protocol 5.7)
  intrinsics_context ctx;                  // build_request reads its chroot and policy only
  // The keys of the requests made: a request's keys are views into its KEY value (as a lock
  // frame's are into the frame's), so the values live as long as the world.
  std::deque<value_t> request_keys;

  explicit home_world(std::size_t cache_capacity = 0) : hooks(cache_capacity) {
    home = std::make_unique<detail::lock_home>(mgr, sched, scheduling_policy::round_robin);
  }

  process &add(int pid) {
    process_ptr p = make_process();
    p->pid = pid;
    p->status = process_status::ready;
    return *sched.table.insert(std::move(p));
  }
  void set_status(process &p, process_status s) { sched.table.set_status(p, s); }

  detail::home_request request(const std::vector<std::string> &keys, lock_mode mode) {
    home->begin_request();
    detail::home_request req;
    value_t key;
    if (keys.size() == 1) {
      key = value_t(keys[0]);
    } else {
      std::vector<value_t> l;
      for (const std::string &k : keys)
        l.emplace_back(k);
      key = make_list(std::move(l));
    }
    detail::build_request(req, *home, ctx, request_keys.emplace_back(std::move(key)), mode);
    return req;
  }
  detail::enter_result enter(process &p, const std::vector<std::string> &keys, lock_mode mode) {
    detail::home_request req = request(keys, mode);
    return home->try_enter(p, req, "");
  }

  // A waiter's frames (design 8.5): its lock_enter frame, and the wait child's await frame above.
  static void wait_frames(process &p) {
    p.state.stack.clear();
    eval_frame enter;
    enter.phase = eval_phase::lock_enter;
    enter.index = 2;
    enter.extra_vals.assign(7, nil_value());
    eval_frame wait;
    wait.phase = eval_phase::await_call;
    wait.extra_vals.assign(1, nil_value());
    wait.extra_strs = {"", "with-lock", "", ""};
    p.state.stack.push_back(std::move(enter));
    p.state.stack.push_back(std::move(wait));
  }
  // P (stepping) waits for KEYS: protocol 5.2. Its outcome when granted or failed at once.
  std::optional<detail::frame_outcome> wait(process &p, const std::vector<std::string> &keys,
                                            lock_mode mode, std::optional<double> timeout) {
    wait_frames(p);
    set_status(p, process_status::running);
    detail::home_request req = request(keys, mode);
    return home->wait(p, req, timeout, "");
  }
  static const value_t &outcome_slot(const process &p) {
    return p.state.stack.at(0).extra_vals.at(3);
  }

  void check() {
    try {
      mgr->check();
      home->check_quiescent();
    } catch (const detail::lock_violation &v) {
      ADD_FAILURE() << v.what();
    }
    EXPECT_TRUE(sched.table.check());
  }
  // The deadlines due by `at`, through wake_due's vector.
  std::vector<process *> due(steady::time_point at) {
    std::vector<process *> v;
    sched.table.wake_due(at, &v);
    return v;
  }
};

std::string class_of(const value_t &exc) { return dsl_exception(exc).class_name(); }

} // namespace

TEST(LockHome, SameRequestKeysStayPinned) {
  for (const std::size_t capacity : {2u, 3u, 4u}) {
    SCOPED_TRACE("capacity " + std::to_string(capacity));
    home_world hw(capacity);
    ASSERT_EQ(hw.home->cache_capacity(), capacity);
    process &p = hw.add(1);
    // A sliding window of 4 keys: each request hits some keys and misses others, and a request
    // of 4 keys never fits a cache of 2 or 3.
    for (int round = 0; round < 8; ++round) {
      std::vector<std::string> keys;
      for (int i = 0; i < 4; ++i)
        keys.push_back("k" + std::to_string(round + i));
      std::sort(keys.begin(), keys.end(),
                [](const std::string &a, const std::string &b) { return detail::key_less(a, b); });
      const detail::enter_result r = hw.enter(p, keys, lock_mode::exclusive);
      ASSERT_EQ(r.what, detail::enter_result::kind::granted);
      const detail::mhold &h = p.locks->holds.back();
      ASSERT_EQ(h.keys.size(), 4u);
      for (std::size_t i = 0; i < 4; ++i)
        EXPECT_EQ(h.keys[i]->key, keys[i]); // the node each key was granted on is that key's
      EXPECT_LE(hw.home->cache_size(), capacity);
      EXPECT_EQ(hw.mgr->table().home_cache_refs(hw.home->id()), hw.home->cache_size());
      EXPECT_EQ(hw.mgr->stats().cache_refs, hw.home->cache_size());
      hw.check();
      hw.home->release_frame(p, r.hold, false);
      hw.check();
    }
    EXPECT_TRUE(hw.mgr->table().empty());
  }
}

TEST(LockHome, EvictionAndRefs) {
  home_world hw(2);
  process &p = hw.add(1);
  const char *const keys[] = {"x1", "x2", "x3"};
  for (int i = 0; i < 12; ++i) {
    const std::string k = keys[i % 3];
    SCOPED_TRACE(k);
    const lock_stats before = hw.mgr->stats();
    const detail::enter_result r = hw.enter(p, {k}, lock_mode::exclusive);
    ASSERT_EQ(r.what, detail::enter_result::kind::granted);
    hw.check();
    hw.home->release_frame(p, r.hold, false);
    hw.check();
    const lock_stats after = hw.mgr->stats();
    EXPECT_EQ(after.cache_refs, hw.home->cache_size());
    EXPECT_EQ(hw.home->cache_size(), std::min<std::size_t>(i + 1, 2));
    // With nothing held, the trie keeps the root and the cached keys' nodes only: an evicted
    // key's references reached 0 and its node is freed.
    EXPECT_EQ(after.trie_nodes, 1 + hw.home->cache_size());
    if (i >= 2) // full: each new use evicts one entry (one reference dropped, one taken)
      EXPECT_EQ(after.cache_refs, before.cache_refs);
  }
  EXPECT_TRUE(hw.mgr->table().empty());
}

TEST(LockHome, StaleWakeIgnored) {
  home_world hw;
  process &h = hw.add(1);
  process &p = hw.add(2);
  const detail::enter_result rh = hw.enter(h, {"k"}, lock_mode::exclusive);
  ASSERT_EQ(rh.what, detail::enter_result::kind::granted);
  ASSERT_FALSE(hw.wait(p, {"k"}, lock_mode::exclusive, 30.0).has_value());
  EXPECT_EQ(hw.sched.parks, (std::vector<int>{2}));
  EXPECT_TRUE(p.lock_waiting);
  const std::uint64_t t1 = *p.locks->ticket;
  hw.check();
  // Its deadline passes first: the wait ends with TimeoutError.
  std::vector<process *> expired = hw.due(steady::now() + std::chrono::hours(1));
  ASSERT_EQ(expired.size(), 1u);
  hw.home->wait_expired(expired);
  EXPECT_EQ(class_of(home_world::outcome_slot(p)), "TimeoutError");
  EXPECT_EQ(hw.sched.readies, (std::vector<int>{2}));
  hw.check();
  // It makes a new request, which queues; then the wake for its old ticket arrives.
  ASSERT_FALSE(hw.wait(p, {"k"}, lock_mode::exclusive, std::nullopt).has_value());
  const std::uint64_t t2 = *p.locks->ticket;
  ASSERT_NE(t1, t2);
  detail::lock_notes stale;
  stale.push_back({hw.home->id(), nullptr, {detail::lock_post::kind::wake, 2, t1}});
  hw.home->deliver(stale);
  EXPECT_TRUE(p.lock_waiting);
  EXPECT_EQ(p.locks->ticket, t2);
  EXPECT_TRUE(home_world::outcome_slot(p).is_nil()); // no outcome: still waiting
  EXPECT_EQ(hw.sched.readies.size(), 1u);
  hw.check();
  // Its own grant.
  hw.home->release_frame(h, rh.hold, false);
  ASSERT_TRUE(std::holds_alternative<int64_t>(home_world::outcome_slot(p).v));
  const auto hold = static_cast<std::uint64_t>(std::get<int64_t>(home_world::outcome_slot(p).v));
  EXPECT_EQ(hw.sched.readies, (std::vector<int>{2, 2}));
  hw.check();
  hw.home->start_body(p, hold);
  hw.home->release_frame(p, hold, false);
  hw.check();
  EXPECT_TRUE(hw.mgr->table().empty());
}

TEST(LockHome, TimeoutAdoptsGrant) {
  home_world hw;
  process &h = hw.add(1);
  process &p = hw.add(2);
  const detail::enter_result rh = hw.enter(h, {"k"}, lock_mode::exclusive);
  ASSERT_FALSE(hw.wait(p, {"k"}, lock_mode::exclusive, 30.0).has_value());
  hw.check();
  // H's release grants P, but its wake is not applied yet (as a post in flight would be).
  detail::lock_notes notes;
  hw.mgr->table().release({hw.home->id(), 1}, rh.hold, notes);
  h.locks->holds.clear(); // H's mirror, as release_frame leaves it
  ASSERT_EQ(notes.size(), 1u);
  EXPECT_EQ(notes[0].post.pid, 2);
  // P's deadline is processed first: its cancel finds the grant, which the wait adopts.
  std::vector<process *> expired = hw.due(steady::now() + std::chrono::hours(1));
  hw.home->wait_expired(expired);
  ASSERT_TRUE(std::holds_alternative<int64_t>(home_world::outcome_slot(p).v))
      << to_string(home_world::outcome_slot(p));
  ASSERT_EQ(p.locks->holds.size(), 1u);
  EXPECT_EQ(p.locks->holds[0].count, 1u);
  EXPECT_EQ(hw.sched.readies, (std::vector<int>{2}));
  hw.check();
  // The late wake is stale.
  hw.home->deliver(notes);
  EXPECT_EQ(hw.sched.readies.size(), 1u);
  hw.check();
  const auto hold = static_cast<std::uint64_t>(std::get<int64_t>(home_world::outcome_slot(p).v));
  hw.home->start_body(p, hold);
  hw.home->release_frame(p, hold, false);
  hw.check();
  EXPECT_TRUE(hw.mgr->table().empty());
}

TEST(LockHome, TryEnterHandlerUpgradeCovered) {
  home_world hw;
  process &p = hw.add(1);
  process &q = hw.add(2);
  const detail::enter_result r = hw.enter(p, {"a"}, lock_mode::exclusive);
  ASSERT_EQ(r.what, detail::enter_result::kind::granted);
  const detail::enter_result rq = hw.enter(q, {"b"}, lock_mode::shared);
  ASSERT_EQ(rq.what, detail::enter_result::kind::granted);
  hw.check();
  const lock_stats s0 = hw.mgr->stats();
  const std::string c0 = hw.mgr->table().canonical();
  const auto refused = [&](process &who, const std::vector<std::string> &keys, lock_mode m) {
    try {
      hw.enter(who, keys, m);
    } catch (const dsl_error &e) {
      EXPECT_EQ(e.class_name(), "LockError");
      for (const auto &[k, v] : e.fields())
        if (k == "reason")
          return text_of(v);
      return std::string("no reason");
    }
    return std::string("granted");
  };
  // 1. The handler rule: P's handler asks for a key under the hold of the program it interrupted.
  p.in_signal_handler = true;
  EXPECT_EQ(refused(p, {"a.b"}, lock_mode::shared), "handler");
  EXPECT_EQ(refused(p, {"z", "a.c"}, lock_mode::exclusive), "handler"); // on its second key
  p.in_signal_handler = false;
  // 2. The upgrade: Q holds S b and asks for X b.c.
  EXPECT_EQ(refused(q, {"b.c"}, lock_mode::exclusive), "upgrade");
  // 3. Covered: P asks for S a.b under its X a: the hold's count goes to 2.
  const detail::enter_result rc = hw.enter(p, {"a.b"}, lock_mode::shared);
  EXPECT_EQ(rc.what, detail::enter_result::kind::covered);
  EXPECT_EQ(rc.hold, r.hold);
  EXPECT_EQ(p.locks->holds.at(0).count, 2u);
  // None of them reached the manager.
  expect_same_lock_state(s0, hw.mgr->stats());
  EXPECT_EQ(hw.mgr->table().canonical(), c0);
  hw.check();
  hw.home->release_frame(p, r.hold, false); // the covered section
  EXPECT_EQ(p.locks->holds.at(0).count, 1u);
  hw.home->release_frame(p, r.hold, false);
  hw.home->release_frame(q, rq.hold, false);
  hw.check();
  EXPECT_TRUE(hw.mgr->table().empty());
}

TEST(LockHome, ReleaseFrameDefensiveWithdraw) {
  home_world hw;
  process &h = hw.add(1);
  process &p = hw.add(2);
  const detail::enter_result rh = hw.enter(h, {"k"}, lock_mode::exclusive);
  ASSERT_FALSE(hw.wait(p, {"k"}, lock_mode::exclusive, 30.0).has_value());
  hw.check();
  // P's lock_enter frame unwinds with its ticket still queued (protocol 5.4's defensive path).
  hw.set_status(p, process_status::running);
  hw.home->release_frame(p, std::nullopt, true);
  EXPECT_FALSE(p.locks->ticket.has_value());
  EXPECT_FALSE(p.lock_waiting);
  EXPECT_FALSE(p.lock_deadline.has_value());
  EXPECT_FALSE(hw.mgr->table().rec_of({hw.home->id(), 2}).has_value());
  EXPECT_EQ(hw.mgr->stats().queued, 0u);
  hw.sched.table.update(p);
  hw.check();
  hw.home->release_frame(h, rh.hold, false);
  hw.check();
  EXPECT_TRUE(hw.mgr->table().empty());
}

TEST(LockHome, DestructorClearsEverything) {
  home_world hw;
  process &a = hw.add(1);
  process &b = hw.add(2);
  process &wa = hw.add(3);
  process &wb = hw.add(4);
  process &c = hw.add(5);
  process &wc = hw.add(6);
  ASSERT_EQ(hw.enter(a, {"a"}, lock_mode::exclusive).what, detail::enter_result::kind::granted);
  ASSERT_EQ(hw.enter(b, {"b"}, lock_mode::shared).what, detail::enter_result::kind::granted);
  ASSERT_FALSE(hw.wait(wa, {"a"}, lock_mode::shared, std::nullopt).has_value());
  ASSERT_FALSE(hw.wait(wb, {"b"}, lock_mode::exclusive, 30.0).has_value());
  const detail::enter_result rc = hw.enter(c, {"c"}, lock_mode::exclusive);
  ASSERT_FALSE(hw.wait(wc, {"c"}, lock_mode::exclusive, std::nullopt).has_value());
  hw.home->release_frame(c, rc.hold, false); // wc granted, not resumed
  ASSERT_TRUE(std::holds_alternative<int64_t>(home_world::outcome_slot(wc).v));
  hw.check();
  const std::shared_ptr<lock_manager> mgr = hw.mgr;
  EXPECT_GT(mgr->stats().holds, 0u);
  EXPECT_EQ(mgr->stats().queued, 2u);
  hw.home.reset();
  for (process *p : {&a, &b, &wa, &wb, &c, &wc}) {
    SCOPED_TRACE(p->pid);
    EXPECT_TRUE(!p->locks || (p->locks->holds.empty() && !p->locks->ticket));
    EXPECT_FALSE(p->lock_waiting);
    EXPECT_FALSE(p->lock_deadline.has_value());
    EXPECT_EQ(p->inherited_priority, INT_MAX);
  }
  EXPECT_NO_THROW(mgr->check());
  EXPECT_TRUE(mgr->table().empty()) << mgr->table().canonical();
  EXPECT_EQ(mgr->stats().holds, 0u);
  EXPECT_EQ(mgr->stats().queued, 0u);
  EXPECT_EQ(mgr->stats().cache_refs, 0u);
}

// L1.7: a lock waiter flagged by a wake_due with no vector (which leaves no timer) is collected by
// the next wait_expired, with the ones the vector brings.
TEST(LockHome, WaitExpiredTakesFlagged) {
  home_world hw;
  process &h = hw.add(1);
  process &p1 = hw.add(2);
  process &p2 = hw.add(3);
  const detail::enter_result rh = hw.enter(h, {"k"}, lock_mode::exclusive);
  ASSERT_FALSE(hw.wait(p1, {"k"}, lock_mode::exclusive, 0.001).has_value());
  ASSERT_FALSE(hw.wait(p2, {"k"}, lock_mode::exclusive, 3600.0).has_value());
  hw.check();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
  // P1's deadline, by a wake_due with no vector: flagged and counted.
  EXPECT_EQ(hw.sched.table.wake_due(steady::now(), nullptr), 1u);
  EXPECT_TRUE(p1.lock_expired_pending);
  EXPECT_EQ(hw.sched.table.lock_expired_count(), 1u);
  hw.check();
  // P2's, through the vector.
  std::vector<process *> expired = hw.due(steady::now() + std::chrono::hours(2));
  ASSERT_EQ(expired.size(), 1u);
  EXPECT_EQ(expired[0], &p2);
  hw.home->wait_expired(expired); // one call times out both
  EXPECT_TRUE(expired.empty());
  EXPECT_EQ(class_of(home_world::outcome_slot(p1)), "TimeoutError");
  EXPECT_EQ(class_of(home_world::outcome_slot(p2)), "TimeoutError");
  EXPECT_FALSE(p1.lock_expired_pending);
  EXPECT_EQ(hw.sched.table.lock_expired_count(), 0u);
  std::vector<int> readied = hw.sched.readies;
  std::sort(readied.begin(), readied.end());
  EXPECT_EQ(readied, (std::vector<int>{2, 3}));
  EXPECT_EQ(hw.mgr->stats().timeouts, 2u);
  hw.check();
  hw.home->release_frame(h, rh.hold, false);
  hw.check();
  EXPECT_TRUE(hw.mgr->table().empty());
}

// ---------------------------------------------------------------------------
// The L1 reviews' findings, as tests (the fixes are the lead's, on perf/locks-l1)
// ---------------------------------------------------------------------------

// Finding 4: a '/' key found in the home's key cache still obeys the requester's channel policy.
// An unrestricted program caches "/jobs" first; a program under a policy that doesn't declare it
// is refused all the same.
TEST_P(Locks, LK11b_CachedGlobalKeyStillNeedsPolicy) {
  EXPECT_EQ(as_int(w.run(R"((with-lock "/jobs" 1))")), 1); // cached now
  channel_policy &pol = w.policies.emplace_back();
  pol.enforce = true;
  pol.global = {"shared"};
  EXPECT_EQ(as_str(w.run(R"((try (with-lock "/jobs" 2) (catch (PermissionError e) "denied")))",
                         {.policy = &pol})),
            "denied");
  EXPECT_TRUE(w.mgr->table().empty());
}

// Finding 3: KEY is evaluated before :timeout, so a :timeout symbol is read after a KEY call ran.
TEST_P(Locks, LK14c_TimeoutReadAfterKey) {
  EXPECT_EQ(as_int(w.run(R"((with-lock (begin (set t 0.5) "k") :timeout t 1))")), 1);
}

// Finding 1: a lock frame restored while its KEY call was being evaluated (its lock form decoded
// as nil) takes the restoring environment's lock form, or refuses with LockError("restore"); and
// one restored with a failed outcome raises it (or the same refusal). Never a null lock form.
TEST_P(Locks, LW07c_RestoredLockFramesOutsideTheWait) {
  {
    const int p = w.submit(R"((try (with-lock (begin (msg-recv "key.go") "k") (+ 1 0))
                      (catch (LockError e) (get-attr e "reason"))))");
    ASSERT_TRUE(w.step_until([&] { return w.parked(p); }));
    cvc::state &node = w.tree("snap.key");
    ASSERT_TRUE(w.snapshot(p, node));
    const int r = w.restore(node, w.envs.at(p));
    w.kill(p);
    // A restored process doesn't re-arm a msg-recv it was parked in (its nil placeholder stands),
    // so the copy may go straight on to KEY; if it parks again, the message releases it.
    ASSERT_TRUE(w.step_until([&] { return w.parked(r) || w.finished(r); }));
    if (w.parked(r))
      w.send("key.go");
    ASSERT_TRUE(w.step_until([&] { return w.finished(r); }));
    const std::string out = to_string(w.result(r));
    EXPECT_TRUE(out == "1" || out == R"("restore")") << out;
  }
  {
    const int h = w.holder("\"k\"", "", "H");
    const int p = w.submit(R"((try (with-lock "k" :timeout 0.1 1)
                                 (catch (TimeoutError e) "timeout")
                                 (catch (LockError e) (get-attr e "reason"))))");
    ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(p); }));
    ASSERT_TRUE(w.step_until([&] { return !w.lock_waiting(p); })); // timed out, not stepped
    cvc::state &node = w.tree("snap.failed");
    ASSERT_TRUE(w.snapshot(p, node));
    const int r = w.restore(node, w.envs.at(p));
    ASSERT_TRUE(w.step_until([&] { return w.finished(r) && w.finished(p); }));
    const std::string out = to_string(w.result(r));
    EXPECT_TRUE(out == R"("timeout")" || out == R"("restore")") << out;
    EXPECT_EQ(as_str(w.result(p)), "timeout");
    w.release(h);
    ASSERT_TRUE(w.step_until([&] { return w.finished(h); }));
  }
  EXPECT_TRUE(w.mgr->table().empty());
}

// Finding 2: a restored wait obeys the restoring context: where lock waits are not allowed (as
// in Ariadne's init lane), the busy request raises TimeoutError at once instead of queueing.
TEST_P(Locks, LW07d_RestoredWaitObeysTheRestoringContext) {
  const int h = w.holder("\"k\"", "", "H");
  const int q = w.submit(R"((try (with-lock "k" 1) (catch (TimeoutError e) "timeout")))");
  ASSERT_TRUE(w.step_until([&] { return w.lock_waiting(q); }));
  cvc::state &node = w.tree("snap.q");
  ASSERT_TRUE(w.snapshot(q, node));
  checked_world::submit_options so;
  so.lock_wait_allowed = false;
  const int r = w.restore(node, w.natives(so));
  ASSERT_TRUE(w.step_until([&] { return w.finished(r) || w.lock_waiting(r); }));
  EXPECT_TRUE(w.finished(r)) << "the restored wait queued where waits are not allowed";
  if (w.finished(r))
    EXPECT_EQ(as_str(w.result(r)), "timeout");
  w.kill(r);
  w.release(h);
  ASSERT_TRUE(w.step_until([&] { return w.all_finished({h, q}); }));
  EXPECT_TRUE(w.mgr->table().empty());
}

// ===========================================================================
// Locks under the spawn context (locks plan L1b; td 4.15)
// ===========================================================================

namespace {
class LocksSpawn : public ::testing::TestWithParam<std::string> {
protected:
  locks_world w{GetParam()};

  // Submit `script` as a parent chrooted at `chroot` whose context installs make_spawn_context.
  int submit_chrooted(const std::string &script, const std::string &chroot) {
    intrinsics_context &ctx = w.contexts.emplace_back();
    ctx.sched = &w.base();
    ctx.proc = make_process();
    ctx.uid = "test";
    apply_chroot(ctx, w.tree, chroot);
    ctx.spawn_context = make_spawn_context();
    environment_ptr env = builtins::make_default_environment();
    register_intrinsics(env, &ctx);
    execute_options opts;
    opts.env = env;
    const int pid = w.base().execute(script, opts);
    ctx.pid = pid;
    ctx.proc->pid = pid;
    return pid;
  }
};
INSTANTIATE_TEST_SUITE_P(Schedulers, LocksSpawn, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });

// `s` as a DSL string literal.
std::string dsl_quote(const std::string &s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '"' || c == '\\')
      out += '\\';
    out += c;
  }
  return out + "\"";
}
} // namespace

TEST_P(LocksSpawn, LK24_SpawnedChildrenLockUnderTheParentsChroot) {
  // k children of a parent chrooted at doc, each `per` sections of an atomic increment and a
  // read-modify-write that sleeps in the middle, so the children interleave inside the lock.
  const int k = 8;
  const int per = under_sanitizer() ? 200 : 1000;
  const std::string child = "(begin (set i 0) (while (< i " + std::to_string(per) +
                            R"S() (begin (with-lock "q" (begin (state-incr "q.n")
                                 (set v (if (state-exists "q.m") (int (state-get "q.m")) 0))
                                 (sleep 0)
                                 (state-set "q.m" (str (+ v 1)))))
                               (set i (+ i 1)))) i))S";
  std::string parent = "(list";
  for (int i = 0; i < k; ++i)
    parent += " (spawn " + dsl_quote(child) + ")";
  parent += ")";
  const int pid = submit_chrooted(parent, "doc");
  ASSERT_TRUE(w.step_until([&] { return w.finished(pid); }));
  const auto pids = as_list(w.result(pid));
  ASSERT_EQ(pids.size(), static_cast<std::size_t>(k));
  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_TRUE(w.step_until(
      [&] {
        for (const value_t &c : pids)
          if (!w.finished(static_cast<int>(as_int(c))))
            return false;
        return true;
      },
      120.0));
  ASSERT_NE(w.base().locks(), nullptr);
  const lock_stats st = w.base().locks()->manager().stats();
  std::printf("LK-24 %s: %d x %d sections in %.3f s, %llu waits\n", GetParam().c_str(), k, per,
              std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
              static_cast<unsigned long long>(st.waits));
  EXPECT_GT(st.waits, 0u); // the children did meet inside the lock
  for (const value_t &c : pids)
    EXPECT_EQ(as_int(w.result(static_cast<int>(as_int(c)))), per);
  const std::string total = std::to_string(k * per);
  EXPECT_EQ(w.node("doc.q.n"), total);
  EXPECT_EQ(w.node("doc.q.m"), total);
  EXPECT_EQ(w.tree.findDescendant("q"), nullptr);
}

// ===========================================================================
// Phase L3: one app, several threads (locks plan L3.md section 3). Several schedulers' homes on
// one shared manager: posts between homes (protocol 2.3, 4.14, 5.9), the races between their
// threads, and the stops where every home has delivered and drained its port (protocol 6).
// ===========================================================================

namespace {

// A shared-mode manager (L3.1), as app::lock_manager() makes one when threads are about.
std::shared_ptr<lock_manager> shared_manager() {
  lock_manager::options o;
  o.threading = lock_threading::shared;
  return std::make_shared<lock_manager>(o);
}

// A world whose home is attached to `mgr` as a home pumped on a thread of its own.
std::unique_ptr<checked_world> home_on(const std::string &kind,
                                       const std::shared_ptr<lock_manager> &mgr,
                                       scheduling_policy policy = scheduling_policy::round_robin,
                                       world_checks checks = world_checks::sequenced,
                                       cvc::state *tree = nullptr) {
  checked_world_options o;
  o.mgr = mgr;
  o.attach.cross_thread = true;
  o.policy = policy;
  o.tree = tree;
  o.checks = checks;
  return std::make_unique<checked_world>(kind, o);
}

// A home's own thread: it runs the closures it is given one at a time, each to its end, so a test
// fixes the order of the homes' actions while each runs on its home's thread.
class home_thread {
public:
  home_thread() : thread_([this] { loop(); }) {}
  ~home_thread() {
    {
      std::lock_guard<std::mutex> g(mu_);
      done_ = true;
    }
    cv_.notify_all();
    thread_.join();
  }
  home_thread(const home_thread &) = delete;
  home_thread &operator=(const home_thread &) = delete;

  // `fn` runs on the thread after the closures given before it; the future is ready when it
  // returns.
  std::future<void> post(std::function<void()> fn) {
    std::packaged_task<void()> task(std::move(fn));
    std::future<void> done = task.get_future();
    {
      std::lock_guard<std::mutex> g(mu_);
      queue_.push_back(std::move(task));
    }
    cv_.notify_all();
    return done;
  }
  // `fn` run on the thread, and waited for.
  void run(std::function<void()> fn) { post(std::move(fn)).get(); }

private:
  void loop() {
    for (;;) {
      std::packaged_task<void()> task;
      {
        std::unique_lock<std::mutex> g(mu_);
        cv_.wait(g, [this] { return done_ || !queue_.empty(); });
        if (queue_.empty())
          return;
        task = std::move(queue_.front());
        queue_.pop_front();
      }
      task();
    }
  }

  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<std::packaged_task<void()>> queue_;
  bool done_ = false;
  std::thread thread_; // last: it starts once the members it uses are made
};

// A cross-thread case's size (L1.md section 3, "Sizes, by lane"): `var` when it is set, else by
// lane: an optimized build, an unoptimized one (Debug and the coverage lanes, 25-50 times
// slower), or a sanitizer job (CVC_LOCKS_SANITIZER=1). By the optimizer, as LK-18's sizes are.
std::size_t lane_size(const char *var, std::size_t release, std::size_t debug,
                      std::size_t sanitizer) {
  if (const char *v = std::getenv(var); v && *v)
    return static_cast<std::size_t>(std::strtoull(v, nullptr, 10));
  if (under_sanitizer())
    return sanitizer;
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  (void)debug;
  return release;
#else
  (void)release;
  return debug;
#endif
}

// The sync and async schedulers' own control queue: `fn` runs on the scheduler's thread at its
// next step, after the step has drained the lock port and before it services the timers.
void post_query(checked_world &v, int pid, std::function<void()> fn) {
  control_cmd cmd;
  cmd.what = control_cmd::kind::query;
  cmd.query = [fn = std::move(fn)](scheduler_base &) { fn(); };
  if (v.sync)
    v.sync->post_control(pid, std::move(cmd));
  else
    v.async->post_control(pid, std::move(cmd));
}

void reap(checked_world &v, int pid) {
  if (v.sync)
    v.sync->reap(pid);
  else
    v.async->reap(pid);
  v.envs.erase(pid);
}

// A wake for `pid`'s ticket `tid`, for `to`'s port: a note as another home's delivery posts it.
detail::lock_notes wake_note(checked_world &to, int pid, std::uint64_t tid) {
  detail::lock_notes notes;
  notes.push_back({to.home().id(), to.home().port(), {detail::lock_post::kind::wake, pid, tid}});
  return notes;
}

class LocksThreads : public ::testing::TestWithParam<std::string> {};
INSTANTIATE_TEST_SUITE_P(Schedulers, LocksThreads, ::testing::Values("sync", "async"),
                         [](const auto &info) { return info.param; });

} // namespace

// LT-2: a grant made on another thread wakes the waiter's scheduler wherever it waits: an
// async_scheduler in wait_for_work, and a sync run() in idle_wait beside a 5 s sleeper, once the
// wake is posted; a sync run(N) too short for the wait returns on its budget and the next run(N)
// applies the wake; a sync run(10^7) left with only the waiter and no timer returns at once
// (design 6.1), and the next run() applies the wake. What each proves is structural (the sleeper
// still asleep, the waiter still queued); the times are printed.
TEST(LocksThreads, LT02_CrossThreadGrantWakes) {
  {
    SCOPED_TRACE("an async_scheduler in wait_for_work");
    const auto mgr = shared_manager();
    home_thread ta;
    auto a = home_on("sync", mgr);
    auto b = home_on("async", mgr);
    int h = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    const int w = b->queued("\"k\"", "", "W");
    ASSERT_TRUE(b->settle());
    const lock_stats s0 = mgr->stats();
    std::atomic<steady::rep> released_at{0};
    std::future<void> release = ta.post([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      released_at = steady::now().time_since_epoch().count();
      a->release(h);
      EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
    });
    const auto t0 = steady::now();
    const bool work = b->async->wait_for_work(deadline_after(30.0));
    const auto woke = steady::now();
    release.get();
    const double waited = std::chrono::duration<double>(woke - t0).count();
    const double latency = std::chrono::duration<double>(woke.time_since_epoch() -
                                                         steady::duration(released_at.load()))
                               .count();
    std::printf("LT-2 async: wait_for_work(30 s) returned after %.3f s, %.1f us after the "
                "release began\n",
                waited, latency * 1e6);
    EXPECT_TRUE(work);
    EXPECT_LT(waited, 10.0);
    EXPECT_EQ(mgr->stats().posts - s0.posts, 1u); // the wake, through B's port
    b->async->drain_ingress();                    // applies it
    EXPECT_FALSE(b->lock_waiting(w));
    EXPECT_TRUE(b->async->has_runnable());
    ASSERT_TRUE(b->step_until([&] { return b->finished(w); }));
    EXPECT_EQ(as_str(b->result(w)), "W");
    EXPECT_TRUE(mgr->table().empty());
  }
  {
    SCOPED_TRACE("a sync run() in idle_wait beside a 5 s sleeper");
    const auto mgr = shared_manager();
    home_thread ta;
    auto a = home_on("async", mgr);
    auto b = home_on("sync", mgr);
    int h = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    const int s = b->submit("(begin (sleep 5) \"slept\")");
    std::optional<process_status> sleeper_at_end;
    steady::time_point ended;
    checked_world::submit_options so;
    so.opts.on_complete = [&](value_t) {
      ended = steady::now();
      sleeper_at_end = b->status(s);
      b->sync->stop();
    };
    const int w = b->submit(R"((with-lock "k" "W"))", so);
    ASSERT_TRUE(b->step_until([&] { return b->lock_waiting(w) && b->parked(s); }));
    std::atomic<steady::rep> released_at{0};
    std::future<void> release = ta.post([&] {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      released_at = steady::now().time_since_epoch().count();
      a->release(h);
      EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
    });
    const auto t0 = steady::now();
    b->sync->run(std::nullopt, 30.0);
    const double took = seconds_since(t0);
    release.get();
    ASSERT_TRUE(sleeper_at_end.has_value()) << "the waiter never ended";
    const double latency = std::chrono::duration<double>(ended.time_since_epoch() -
                                                         steady::duration(released_at.load()))
                               .count();
    std::printf("LT-2 sync idle_wait: run() returned after %.3f s; the waiter ended %.1f us after "
                "the release began\n",
                took, latency * 1e6);
    // The post ended the wait: the sleeper was still asleep when the waiter ended.
    EXPECT_EQ(*sleeper_at_end, process_status::waiting);
    EXPECT_LT(took, 10.0);
    EXPECT_EQ(as_str(b->result(w)), "W");
    EXPECT_TRUE(b->check_all());
    b->kill(s);
    EXPECT_TRUE(mgr->table().empty());
  }
  {
    SCOPED_TRACE("a sync run(N) too short for the wait, then another run(N)");
    const auto mgr = shared_manager();
    home_thread ta;
    auto a = home_on("async", mgr);
    auto b = home_on("sync", mgr);
    int h = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    const int s = b->submit("(begin (sleep 5) \"slept\")");
    const int w = b->queued("\"k\"", "", "W");
    ASSERT_TRUE(b->step_until([&] { return b->parked(s); }));
    const auto t0 = steady::now();
    b->sync->run(1000); // its budget runs out long before the sleeper's 5 s
    const double took = seconds_since(t0);
    std::printf("LT-2 sync run(1000) beside a 5 s sleeper returned after %.4f s\n", took);
    EXPECT_TRUE(b->check_all());
    EXPECT_TRUE(b->lock_waiting(w)); // returned on its budget, the grant not made yet
    EXPECT_TRUE(b->parked(s));
    ta.run([&] {
      a->release(h);
      EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
    });
    EXPECT_TRUE(b->home().port_pending());
    b->sync->run(1000); // its first step drains the port
    EXPECT_TRUE(b->check_all());
    EXPECT_TRUE(b->finished(w));
    EXPECT_EQ(as_str(b->result(w)), "W");
    b->kill(s);
    EXPECT_TRUE(mgr->table().empty());
  }
  {
    SCOPED_TRACE("a sync run(10^7) left with only the waiter");
    const auto mgr = shared_manager();
    home_thread ta;
    auto a = home_on("sync", mgr);
    auto b = home_on("sync", mgr);
    int h = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    const int w = b->queued("\"k\"", "", "W");
    const auto t0 = steady::now();
    b->sync->run(10000000); // no timer: nothing in this run can be counted on to wake it
    const double took = seconds_since(t0);
    std::printf("LT-2 sync run(10^7) with only a lock waiter returned after %.4f s\n", took);
    EXPECT_TRUE(b->check_all());
    EXPECT_TRUE(b->lock_waiting(w));
    EXPECT_LT(took, 10.0);
    ta.run([&] {
      a->release(h);
      EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
    });
    b->sync->run(std::nullopt, 30.0); // applies the wake, and ends with the waiter
    EXPECT_TRUE(b->check_all());
    EXPECT_TRUE(b->finished(w));
    EXPECT_EQ(as_str(b->result(w)), "W");
    EXPECT_TRUE(mgr->table().empty());
  }
}

// LT-3 (C++; pycvc's test_lt03_exec_destroyed is its Exec twin): a holder killed on its thread
// while another home's thread pumps a waiter: the waiter is granted through its port, and sees
// the abandoned flag of the killed X holder; a scheduler destroyed on its thread with a wake in
// its port: nothing is held by its processes, the post is discarded, check() clean.
TEST_P(LocksThreads, LT03_HolderKilledAndSchedulerDestroyed) {
  const std::string kind = GetParam();
  {
    SCOPED_TRACE("the holder killed");
    const auto mgr = shared_manager();
    home_thread ta, tb;
    auto a = home_on(kind, mgr, scheduling_policy::round_robin, world_checks::concurrent);
    auto b = home_on(kind, mgr, scheduling_policy::round_robin, world_checks::concurrent);
    int h = -1, w = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    tb.run([&] {
      w = b->submit(R"((with-lock "k" :as g (get-attr g "abandoned")))");
      EXPECT_TRUE(b->step_until([&] { return b->lock_waiting(w); }));
    });
    const lock_stats s0 = mgr->stats();
    // B's thread pumps, as a host's run() would, while A's kills the holder.
    std::future<void> pump =
        tb.post([&] { EXPECT_TRUE(b->step_until([&] { return b->finished(w); }, 30.0)); });
    ta.run([&] { EXPECT_TRUE(a->kill(h)); });
    pump.get();
    tb.run([&] { EXPECT_TRUE(as_bool(b->result(w))); }); // an X body had started: abandoned
    const lock_stats s1 = mgr->stats();
    EXPECT_EQ(s1.posts - s0.posts, 1u);
    EXPECT_EQ(s1.abandoned - s0.abandoned, 1u);
    EXPECT_FALSE(a->broken);
    EXPECT_FALSE(b->broken);
    EXPECT_NO_THROW(mgr->check());
    EXPECT_TRUE(mgr->table().empty());
  }
  {
    SCOPED_TRACE("the scheduler destroyed with a wake in its port");
    const auto mgr = shared_manager();
    home_thread ta, tb;
    auto a = home_on(kind, mgr);
    auto b = home_on(kind, mgr);
    int h = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    tb.run([&] {
      b->queued("\"k\"", "", "W");
      b->holder("\"x\"", "", "X"); // a hold of its own as well
    });
    ta.run([&] {
      a->release(h);
      EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
    });
    const lock_stats s0 = mgr->stats();
    EXPECT_TRUE(b->home().port_pending());
    EXPECT_EQ(s0.holds, 2u); // W's grant, its wake not applied, and X's hold
    tb.run([&] { b.reset(); });
    const lock_stats s1 = mgr->stats();
    EXPECT_EQ(s1.holds, 0u); // nothing held by B's processes
    EXPECT_EQ(s1.queued, 0u);
    EXPECT_EQ(s1.posts_discarded - s0.posts_discarded, 1u);
    EXPECT_EQ(s1.abandoned - s0.abandoned, 1u); // X's body had started; W's had not
    EXPECT_NO_THROW(mgr->check());
    EXPECT_TRUE(mgr->table().empty()) << mgr->table().canonical();
    ta.run([&] {
      EXPECT_TRUE(a->next_x_abandoned("x"));
      EXPECT_FALSE(a->next_x_abandoned("k"));
    });
  }
}

namespace {

// LT-3b's count of the notes deliver hands to another home's port (lock_test_hooks).
std::atomic<std::uint64_t> g_cross_home_notes{0};
void count_cross_home_note(const detail::lock_note &) {
  g_cross_home_notes.fetch_add(1, std::memory_order_relaxed);
}

// Installs a set of hooks on `mgr` for its scope.
struct test_hooks_scope {
  lock_manager &mgr;
  test_hooks_scope(lock_manager &m, const detail::lock_test_hooks &h) : mgr(m) {
    mgr.set_test_hooks(&h);
  }
  ~test_hooks_scope() { mgr.set_test_hooks(nullptr); }
  test_hooks_scope(const test_hooks_scope &) = delete;
  test_hooks_scope &operator=(const test_hooks_scope &) = delete;
};

} // namespace

// LT-3b: a post in flight racing the destruction of its scheduler. A's thread releases a key B's
// waiter waits for, while B's thread steps B a few rounds and lets go of its reference, and a
// third thread drops the last one, so ~lock_home (protocol 5.7) runs there. Each post lands
// before the close or is dropped: over the run, posts + posts_dropped equals the cross-home notes
// delivered (counted through lock_test_hooks), and every post that landed was applied by B's
// drain or discarded at the close. ASan and TSan (ci.yml's sanitizer jobs) watch for a use after
// free and a race. 10^4 runs (10^3 in an unoptimized build; CVC_LOCKS_LT03B_RUNS), within 90 s
// unless CVC_LOCKS_LT03B_RUNS sets the count, and never fewer than a twentieth of them: on macOS
// sched_yield gives up the core for up to a quantum (10 ms) while other threads are runnable, and
// the spins, the dropper's wait and libc++'s barrier all yield, so a run on a loaded runner takes
// 20-30 ms there against about 1 ms on Linux.
TEST_P(LocksThreads, LT03b_PostRacesDestruction) {
  const std::string kind = GetParam();
  const std::size_t runs = lane_size("CVC_LOCKS_LT03B_RUNS", 10000, 1000, 10000);
  const char *sized = std::getenv("CVC_LOCKS_LT03B_RUNS");
  const bool budgeted = !(sized && *sized);
  const steady::time_point budget = deadline_after(90.0);
  detail::lock_test_hooks hooks;
  hooks.cross_home_note = count_cross_home_note;
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  const auto mgr = shared_manager();
  test_hooks_scope installed(*mgr, hooks);
  auto a = home_on(kind, mgr, scheduling_policy::round_robin, world_checks::stepping, &tree);

  // A run's parts: set by this thread before the run starts, read by the others once it has.
  std::shared_ptr<checked_world> b_pump, b_last; // B's two references
  int h = -1, w = -1;
  int release_spin = 0, pump_spin = 0, pump_rounds = 0, drop_spin = 0;
  bool applied = false;
  bool quit = false;
  std::atomic<bool> pumped{false};
  std::barrier<> sync(4);
  const auto spin = [](int n) {
    for (int i = 0; i < n; ++i)
      std::this_thread::yield();
  };

  std::thread releaser([&] { // A's thread
    for (;;) {
      sync.arrive_and_wait();
      if (quit)
        return;
      spin(release_spin);
      a->release(h);
      EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
      reap(*a, h);
      sync.arrive_and_wait();
    }
  });
  std::thread pump([&] { // B's thread
    for (;;) {
      sync.arrive_and_wait();
      if (quit)
        return;
      const steady::time_point end = deadline_after(30.0);
      spin(pump_spin);
      for (int r = 0; r < pump_rounds; ++r)
        b_pump->round(end, false);
      applied = !b_pump->lock_waiting(w); // only a drain of its port ends the wait
      EXPECT_FALSE(b_pump->broken);
      b_pump.reset();
      pumped.store(true, std::memory_order_release);
      sync.arrive_and_wait();
    }
  });
  std::thread dropper([&] { // the third thread
    for (;;) {
      sync.arrive_and_wait();
      if (quit)
        return;
      while (!pumped.load(std::memory_order_acquire))
        std::this_thread::yield();
      spin(drop_spin);
      b_last.reset(); // the last reference: B is destroyed here
      sync.arrive_and_wait();
    }
  });

  std::mt19937_64 rng(31);
  const auto pick = [&](int n) { return std::uniform_int_distribution<int>(0, n - 1)(rng); };
  std::uint64_t no_note = 0, landed_applied = 0, landed_discarded = 0, dropped = 0;
  const auto t0 = steady::now();
  std::size_t run = 0;
  for (; run < runs && !::testing::Test::HasFailure() && !(budgeted && steady::now() >= budget);
       ++run) {
    auto b = std::make_shared<checked_world>(
        kind, checked_world_options{mgr, lock_attach{true}, scheduling_policy::round_robin, &tree,
                                    world_checks::stepping});
    h = a->holder("\"k\"");
    w = b->submit(R"((with-lock "k" 1))");
    if (!b->step_until([&] { return b->lock_waiting(w); })) {
      ADD_FAILURE() << "run " << run << ": the waiter never queued";
      break;
    }
    release_spin = pick(64);
    pump_spin = pick(256);
    pump_rounds = pick(4);
    drop_spin = pick(64);
    pumped.store(false, std::memory_order_relaxed);
    b_pump = b;
    b_last = std::move(b);
    const std::uint64_t n0 = g_cross_home_notes.load();
    const lock_stats s0 = mgr->stats();
    sync.arrive_and_wait(); // the run starts
    sync.arrive_and_wait(); // and ends
    const std::uint64_t notes = g_cross_home_notes.load() - n0;
    const lock_stats s1 = mgr->stats();
    const std::uint64_t posts = s1.posts - s0.posts, drops = s1.posts_dropped - s0.posts_dropped,
                        discarded = s1.posts_discarded - s0.posts_discarded;
    // One note at most: H's release grants W, unless B's detach withdrew W's ticket first.
    EXPECT_LE(notes, 1u) << "run " << run;
    EXPECT_EQ(posts + drops, notes) << "run " << run;
    EXPECT_EQ(posts, (applied ? 1u : 0u) + discarded) << "run " << run;
    EXPECT_EQ(s1.holds, 0u) << "run " << run;
    EXPECT_EQ(s1.queued, 0u) << "run " << run;
    if (notes == 0)
      ++no_note;
    else if (drops)
      ++dropped;
    else if (applied)
      ++landed_applied;
    else
      ++landed_discarded;
  }
  quit = true;
  sync.arrive_and_wait();
  releaser.join();
  pump.join();
  dropper.join();
  std::printf("LT-3b %s: %zu runs in %.2f s: the post landed and was applied %llu, landed and was "
              "discarded %llu, was dropped %llu; no post (the detach came first) %llu%s\n",
              kind.c_str(), run, seconds_since(t0), static_cast<unsigned long long>(landed_applied),
              static_cast<unsigned long long>(landed_discarded),
              static_cast<unsigned long long>(dropped), static_cast<unsigned long long>(no_note),
              run < runs && !::testing::Test::HasFailure() ? " (the 90 s budget ended it)" : "");
  if (!::testing::Test::HasFailure())
    EXPECT_GE(run, runs / 20) << "the 90 s budget ended LT-3b after " << run << " of " << runs
                              << " runs";
  EXPECT_FALSE(a->broken);
  EXPECT_NO_THROW(mgr->check());
  EXPECT_TRUE(mgr->table().empty()) << mgr->table().canonical();
}

// LD-6: a cycle across two homes, both on one shared manager, stepped alternately by one thread
// in a fixed order: the same victim, with the same cycle, in 20 runs. The victim is the request
// that closes the cycle (I7); the other home's pid shows as nil in it (protocol 4.6).
TEST_P(LocksThreads, LD06_CrossHomeCycleDeterministic) {
  const std::string kind = GetParam();
  const auto program = [](const char *first, const char *second) {
    return std::string("(try (with-lock \"") + first +
           "\" (begin (sleep 0) (sleep 0) (with-lock \"" + second +
           "\" \"ok\"))) (catch (DeadlockError e) (get-attr e \"cycle\")))";
  };
  std::optional<std::string> first_run;
  for (int run = 0; run < 20; ++run) {
    SCOPED_TRACE("run " + std::to_string(run));
    const auto mgr = shared_manager();
    auto a = home_on(kind, mgr);
    auto b = home_on(kind, mgr);
    const int pa = a->submit(program("a", "b"));
    const int pb = b->submit(program("b", "a"));
    const steady::time_point until = deadline_after(10.0);
    while (!(a->finished(pa) && b->finished(pb)) && steady::now() < until &&
           !(a->broken || b->broken)) {
      a->round(until, false);
      b->round(until, false);
    }
    ASSERT_TRUE(a->finished(pa) && b->finished(pb));
    const value_t ra = a->result(pa), rb = b->result(pb);
    const bool a_victim = !std::holds_alternative<std::string>(ra.v);
    const bool b_victim = !std::holds_alternative<std::string>(rb.v);
    ASSERT_NE(a_victim, b_victim) << to_string(ra) << " / " << to_string(rb);
    const cycle_view c = cycle_of(a_victim ? ra : rb);
    const std::string seen = std::string(a_victim ? "A's" : "B's") + " pid " +
                             std::to_string(a_victim ? pa : pb) + ": " + show_cycle(c);
    // From the victim round the cycle: its own pid, then the other home's, shown as nil.
    EXPECT_EQ(c.pids, (std::vector<int>{a_victim ? pa : pb, -1}));
    EXPECT_EQ(c.keys,
              (a_victim ? std::vector<std::string>{"b", "a"} : std::vector<std::string>{"a", "b"}));
    if (!first_run) {
      first_run = seen;
      std::printf("LD-6 %s: the victim is %s\n", kind.c_str(), seen.c_str());
    }
    EXPECT_EQ(seen, *first_run);
    EXPECT_EQ(mgr->stats().deadlocks, 1u);
    EXPECT_TRUE(mgr->table().empty());
  }
}

// PI-7: a relevel post for a holder on another home that arrives after the holder released: its
// apply re-reads the manager (protocol 5.9), so the holder stays at its base. H (base 10, a
// priority_rr home A) holds k; inside H's step, through the hook, B's thread queues W (base -10,
// priority_rr home B) behind it, whose block posts a relevel for H to A's port; the hook then
// raises, and the unwind releases k in that same step. A's next step drains the post.
TEST_P(LocksThreads, PI07_LateRelevelRereads) {
  const std::string kind = GetParam();
  const auto mgr = shared_manager();
  home_thread tb;
  auto a = home_on(kind, mgr, scheduling_policy::priority_rr);
  auto b = home_on(kind, mgr, scheduling_policy::priority_rr);
  checked_world::submit_options hs;
  hs.opts.priority = 10;
  const int h = a->submit(R"((begin
      (try (with-lock "k" (begin (msg-recv "h.go") (hook "queue W") "body"))
        (catch (RuntimeError e) "unwound"))
      (msg-recv "h.after")))",
                          hs);
  ASSERT_TRUE(a->step_until([&] { return a->inside(h); }));
  int w = -1;
  tb.run([&] {
    checked_world::submit_options ws;
    ws.opts.priority = -10;
    w = b->submit(R"((with-lock "k" "W"))", ws);
  });
  bool hooked = false;
  a->on_hook = [&](const std::string &) {
    tb.run([&] { EXPECT_TRUE(b->step_until([&] { return b->lock_waiting(w); })); });
    hooked = true;
    throw std::runtime_error("the section unwinds in this step");
  };
  const lock_stats s0 = mgr->stats();
  a->send("h.go");
  const steady::time_point until = deadline_after(10.0);
  while (!hooked && !a->broken && steady::now() < until)
    a->round(until);
  ASSERT_TRUE(hooked);
  // The relevel was emitted while H held k, and is still in A's port now that H holds nothing.
  const lock_stats s1 = mgr->stats();
  EXPECT_GT(s1.relevels, s0.relevels);
  EXPECT_EQ(a->held(h), 0u);
  EXPECT_TRUE(a->home().port_pending()) << "A drained the relevel before H released";
  EXPECT_EQ(a->info(h)->effective_priority, 10);
  ASSERT_TRUE(a->step_until([&] { return !a->home().port_pending(); })); // drained: applied
  ASSERT_TRUE(a->step_until([&] { return a->parked(h); }));
  EXPECT_EQ(a->info(h)->effective_priority, 10); // at its base
  EXPECT_EQ(a->info(h)->priority, 10);
  EXPECT_EQ(as_int(entry_of(a->run("(get-priority " + std::to_string(h) + ")"), "effective")), 10);
  EXPECT_TRUE(a->check_all());
  tb.run([&] {
    EXPECT_TRUE(b->step_until([&] { return b->finished(w); }));
    EXPECT_EQ(as_str(b->result(w)), "W");
  });
  a->send("h.after");
  ASSERT_TRUE(a->step_until([&] { return a->finished(h); }));
  EXPECT_EQ(as_str(a->result(h)), "go");
  EXPECT_TRUE(mgr->table().empty());
}

// PI-10: a round-robin home lends nothing (design 6.2). Its child, spawned at -20, waits behind a
// priority_rr home's holder at 10: the holder is not boosted (get-priority's effective is 10), and
// no relevel is emitted or posted while it waits (stats().relevels and posts unchanged).
TEST_P(LocksThreads, PI10_RoundRobinChildLendsNothing) {
  const std::string kind = GetParam();
  const auto mgr = shared_manager();
  auto a = home_on(kind, mgr, scheduling_policy::priority_rr);
  auto b = home_on(kind, mgr, scheduling_policy::round_robin);
  checked_world::submit_options hs;
  hs.opts.priority = 10;
  const int h = a->holder("\"k\"", "", "H", hs);
  checked_world::submit_options ps;
  ps.spawn = true;
  const int parent = b->submit(
      R"S((spawn "(begin (msg-recv \"kid.go\") (with-lock \"k\" \"kid\"))" "kid" -20))S", ps);
  ASSERT_TRUE(b->step_until([&] { return b->finished(parent); }));
  const int kid = static_cast<int>(as_int(b->result(parent)));
  ASSERT_TRUE(b->step_until([&] { return b->parked(kid); }));
  EXPECT_EQ(b->info(kid)->priority, -20);
  const lock_stats s0 = mgr->stats();
  b->send("kid.go");
  ASSERT_TRUE(b->step_until([&] { return b->lock_waiting(kid); }));
  ASSERT_TRUE(a->settle());
  const lock_stats s1 = mgr->stats();
  EXPECT_EQ(s1.waits - s0.waits, 1u);
  EXPECT_EQ(s1.relevels, s0.relevels);
  EXPECT_EQ(s1.posts, s0.posts);
  EXPECT_FALSE(a->home().port_pending());
  EXPECT_EQ(a->info(h)->effective_priority, 10);
  EXPECT_EQ(as_int(entry_of(a->run("(get-priority " + std::to_string(h) + ")"), "effective")), 10);
  a->release(h);
  ASSERT_TRUE(a->step_until([&] { return a->finished(h); }));
  EXPECT_EQ(mgr->stats().posts - s1.posts, 1u); // the kid's wake
  ASSERT_TRUE(b->step_until([&] { return b->finished(kid); }));
  EXPECT_EQ(as_str(b->result(kid)), "kid");
  EXPECT_TRUE(mgr->table().empty());
}

// LX-5: a timeout and a kill, each racing a grant made on another home's thread, before and after
// the wake is applied: one outcome. The timeout adopts the grant (protocol 5.5) and the late wake
// is ignored; the kill releases the grant, the next waiter is granted, and the late wake is
// ignored. "Before" places A's release inside B's step, after its port was drained and before its
// timers (the schedulers' control queue), so B's timeout finds the ticket granted with the wake
// still in its port.
TEST_P(LocksThreads, LX05_TimeoutAndKillRaceAnotherHomesGrant) {
  const std::string kind = GetParam();
  for (const char *variant : {"before", "after"}) {
    const bool before = std::string(variant) == "before";
    {
      SCOPED_TRACE(std::string(variant) + " the wake, a timeout");
      const auto mgr = shared_manager();
      home_thread ta;
      auto a = home_on(kind, mgr);
      auto b = home_on(kind, mgr);
      int h = -1;
      ta.run([&] { h = a->holder("\"k\""); });
      const int p = b->submit(R"((try (with-lock "k" :timeout 0.2 (begin (mark "P-body") "body"))
                                   (catch (TimeoutError e) "timeout")))");
      ASSERT_TRUE(b->step_until([&] { return b->lock_waiting(p); }));
      const auto release = [&] {
        ta.run([&] {
          a->release(h);
          EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
        });
      };
      const lock_stats s0 = mgr->stats();
      if (before) {
        post_query(*b, p, release);
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // past P's deadline
        // B's next drain: its port (empty), then the query (A's release posts the wake), then
        // the timers, where P's deadline finds its ticket granted and adopts it.
        if (b->sync)
          b->sync->step();
        else
          b->async->drain_ingress();
        EXPECT_FALSE(b->lock_waiting(p));
        EXPECT_TRUE(b->home().port_pending()) << "the wake was applied before the timeout";
      } else {
        release();
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); // past P's deadline
        EXPECT_TRUE(b->home().port_pending());
      }
      ASSERT_TRUE(b->step_until([&] { return b->finished(p); }));
      EXPECT_FALSE(b->home().port_pending());
      EXPECT_EQ(as_str(b->result(p)), "body"); // the grant kept: one outcome
      EXPECT_EQ(std::count(b->trace.begin(), b->trace.end(), "P-body"), 1);
      const lock_stats s1 = mgr->stats();
      EXPECT_EQ(s1.timeouts, s0.timeouts);
      EXPECT_EQ(s1.posts - s0.posts, 1u);
      EXPECT_TRUE(mgr->table().empty());
      EXPECT_FALSE(a->broken || b->broken);
    }
    {
      SCOPED_TRACE(std::string(variant) + " the wake, a kill");
      const auto mgr = shared_manager();
      home_thread ta;
      auto a = home_on(kind, mgr);
      auto b = home_on(kind, mgr);
      int h = -1, q = -1;
      ta.run([&] { h = a->holder("\"k\""); });
      const int p = b->queued("\"k\"", "", "P");
      ta.run([&] { q = a->queued("\"k\"", "", "Q"); });
      ta.run([&] {
        a->release(h);
        EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
      });
      EXPECT_TRUE(b->home().port_pending());
      if (!before) {
        b->home().drain_port(); // applied: granted, not resumed
        EXPECT_FALSE(b->lock_waiting(p));
        EXPECT_EQ(b->held(p), 1u);
      }
      b->kill(p);
      ASSERT_TRUE(b->settle()); // drains whatever is left: a late wake is ignored
      EXPECT_FALSE(b->home().port_pending());
      EXPECT_EQ(b->status(p), process_status::killed);
      EXPECT_EQ(b->trace, std::vector<std::string>{});
      ta.run([&] {
        EXPECT_TRUE(a->step_until([&] { return a->finished(q); }));
        EXPECT_EQ(a->trace, (std::vector<std::string>{"H", "Q"}));
        EXPECT_FALSE(a->next_x_abandoned("k")); // P's body never began
      });
      EXPECT_TRUE(mgr->table().empty());
      EXPECT_FALSE(a->broken || b->broken);
    }
  }
}

// LX-7 (posted): a scheduler destroyed with holders and waiters, one of them granted by another
// home with the wake still in its port: one detach releases every hold and cancels every ticket,
// the post is discarded, and the other home's waiters are granted through their port.
TEST_P(LocksThreads, LX07_DestroyedWithAWakeInItsPort) {
  const std::string kind = GetParam();
  const auto mgr = shared_manager();
  home_thread ta, tb;
  auto a = home_on(kind, mgr);
  auto b = home_on(kind, mgr);
  int hx = -1, hy = -1, wa = -1, wb = -1;
  ta.run([&] {
    hx = a->holder("\"x\"", "", "HX");
    hy = a->holder("\"y\"", "", "HY");
  });
  tb.run([&] {
    b->holder("\"a\"", "", "A");        // X, its body begun
    b->holder("\"b\"", ":shared", "B"); // S
    b->queued("\"x\"", "", "WX");       // granted below, its wake left in the port
    b->queued("\"y\"", "", "WY");       // still queued at the end
  });
  ta.run([&] {
    wa = a->submit(R"((with-lock "a" :as g (get-attr g "abandoned")))");
    EXPECT_TRUE(a->step_until([&] { return a->lock_waiting(wa); }));
    wb = a->queued("\"b\"", "", "WB");
    a->release(hx);
    EXPECT_TRUE(a->step_until([&] { return a->finished(hx); }));
  });
  const lock_stats s0 = mgr->stats();
  EXPECT_TRUE(b->home().port_pending());
  EXPECT_EQ(s0.holds, 4u);  // HY, A, B, and WX's grant
  EXPECT_EQ(s0.queued, 3u); // WY, WA, WB
  tb.run([&] { b.reset(); });
  const lock_stats s1 = mgr->stats();
  EXPECT_EQ(s1.posts_discarded - s0.posts_discarded, 1u); // WX's wake
  EXPECT_EQ(s1.posts - s0.posts, 2u);                     // WA's and WB's grants, to A's port
  EXPECT_EQ(s1.abandoned - s0.abandoned, 1u);             // A's X section had begun
  EXPECT_EQ(s1.holds, 3u);                                // HY, WA, WB
  EXPECT_EQ(s1.queued, 0u);
  EXPECT_NO_THROW(mgr->check());
  ta.run([&] {
    EXPECT_TRUE(a->home().port_pending());
    EXPECT_TRUE(a->step_until([&] { return a->finished(wa) && a->finished(wb); }));
    EXPECT_TRUE(as_bool(a->result(wa))); // B's X section on "a" had begun
    a->release(hy);
    EXPECT_TRUE(a->step_until([&] { return a->finished(hy); }));
    EXPECT_FALSE(a->next_x_abandoned("x")); // WX's body never began
  });
  EXPECT_TRUE(mgr->table().empty()) << mgr->table().canonical();
}

// LX-10: a grant to a waiter on another home, then the waiter killed and reaped before the wake
// is applied: its end releases the granted hold, the next waiter (on a third home) is granted,
// and the wake, drained later, finds no process and is ignored.
TEST_P(LocksThreads, LX10_GrantThenWaiterKilledBeforeWake) {
  const std::string kind = GetParam();
  const auto mgr = shared_manager();
  home_thread ta, tc;
  auto a = home_on(kind, mgr);
  auto b = home_on(kind, mgr);
  auto c = home_on(kind, mgr);
  int h = -1, q = -1;
  ta.run([&] { h = a->holder("\"k\""); });
  const int p = b->queued("\"k\"", "", "P");
  tc.run([&] { q = c->queued("\"k\"", "", "Q"); });
  const lock_stats s0 = mgr->stats();
  ta.run([&] {
    a->release(h);
    EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
  });
  EXPECT_TRUE(b->home().port_pending());
  EXPECT_TRUE(b->kill(p));
  reap(*b, p);
  EXPECT_FALSE(b->info(p).has_value());
  EXPECT_TRUE(c->home().port_pending()); // P's end granted Q
  EXPECT_EQ(mgr->stats().posts - s0.posts, 2u);
  ASSERT_TRUE(b->settle()); // the wake for P: no such process
  EXPECT_FALSE(b->home().port_pending());
  EXPECT_TRUE(b->trace.empty());
  tc.run([&] {
    EXPECT_TRUE(c->step_until([&] { return c->finished(q); }));
    EXPECT_EQ(as_str(c->result(q)), "Q");
    EXPECT_FALSE(c->next_x_abandoned("k")); // P's body never began
  });
  EXPECT_TRUE(mgr->table().empty());
}

// LX-11 (posted): a wake for a ticket its process no longer waits on, posted by another home's
// thread: one adopted by its timeout (queued variant), or collected by its grant (granted
// variant), then a new request queued. The stale wake, drained from the port, leaves the new
// request queued until its own grant.
TEST_P(LocksThreads, LX11_StalePostedWake) {
  const std::string kind = GetParam();
  for (const char *variant : kVariants) {
    SCOPED_TRACE(variant);
    const bool queued = std::string(variant) == "queued";
    const auto mgr = shared_manager();
    home_thread ta;
    auto a = home_on(kind, mgr);
    auto b = home_on(kind, mgr);
    int h = -1, x = -1;
    ta.run([&] { h = a->holder("\"k\""); });
    const int p = b->submit(std::string(R"((begin
        (try (with-lock "k" :timeout )") +
                            (queued ? "0.1" : "30") + R"( (msg-recv "p.first"))
          (catch (TimeoutError e) nil))
        (with-lock "k" (mark "P-second"))))");
    ASSERT_TRUE(b->step_until([&] { return b->lock_waiting(p); }));
    const detail::lock_pid P{b->home().id(), p};
    const std::uint64_t old_tid = *mgr->table().rec_of(P)->ticket;
    if (queued) {
      // Times out; the second request queues behind H.
      ASSERT_TRUE(b->step_until([&] {
        const auto rec = mgr->table().rec_of(P);
        return b->lock_waiting(p) && rec && rec->ticket && *rec->ticket != old_tid;
      }));
    } else {
      ta.run([&] {
        a->release(h);
        EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
        // X takes k when P's first section ends, before P's second request.
        x = a->submit(R"((with-lock "k" (msg-recv "x.go")))");
        EXPECT_TRUE(a->step_until([&] { return a->lock_waiting(x); }));
      });
      ASSERT_TRUE(b->step_until([&] { return b->inside(p); })); // the grant collected
      b->send("p.first");
      ASSERT_TRUE(b->step_until([&] { return b->lock_waiting(p); }));
      ta.run([&] { EXPECT_TRUE(a->step_until([&] { return a->inside(x); })); });
    }
    const lock_stats s0 = mgr->stats();
    ta.run([&] { a->home().deliver(wake_note(*b, p, old_tid)); }); // the stale wake, posted
    EXPECT_EQ(mgr->stats().posts - s0.posts, 1u);
    EXPECT_TRUE(b->home().port_pending());
    ASSERT_TRUE(b->settle()); // drained, and ignored
    EXPECT_FALSE(b->home().port_pending());
    EXPECT_TRUE(b->lock_waiting(p)); // the new request stays queued
    EXPECT_EQ(b->held(p), 0u);
    ta.run([&] {
      if (queued) {
        a->release(h); // its own grant
        EXPECT_TRUE(a->step_until([&] { return a->finished(h); }));
      } else {
        a->send("x.go");
        EXPECT_TRUE(a->step_until([&] { return a->finished(x); }));
      }
    });
    ASSERT_TRUE(b->step_until([&] { return b->finished(p); }));
    EXPECT_EQ(b->trace.back(), "P-second");
    EXPECT_TRUE(mgr->table().empty());
    EXPECT_FALSE(a->broken || b->broken);
  }
}

// The progress store (L3.3; design 8.4): after a step on another home, blocking_sinks for a
// waiter reports that home's cell as moved; a step of the waiter's own home leaves it as it was.
TEST_P(LocksThreads, ProgressStoreMovesTheSinksCell) {
  const std::string kind = GetParam();
  const auto mgr = shared_manager();
  auto a = home_on(kind, mgr);
  auto b = home_on(kind, mgr);
  const int h = a->holder("\"k\"");
  const int spinner = a->submit("(begin (msg-recv \"spin\") (sleep 0) (sleep 0) 1)");
  ASSERT_TRUE(a->settle());
  const int w = b->queued("\"k\"", "", "W");
  const int other = b->submit("(begin (msg-recv \"other\") (sleep 0) 1)");
  ASSERT_TRUE(b->settle());
  const auto sinks = [&] { return b->home().blocking_sinks(w); };
  const auto s0 = sinks();
  ASSERT_EQ(s0.size(), 1u);
  EXPECT_EQ(s0[0].first, (detail::lock_pid{a->home().id(), h}));
  // A step on the waiter's home stores its own cell, not the sink's.
  std::this_thread::sleep_for(std::chrono::milliseconds(2)); // the clock moves between readings
  b->send("other");
  ASSERT_TRUE(b->round(deadline_after(10.0), false));
  const auto s1 = sinks();
  ASSERT_EQ(s1.size(), 1u);
  EXPECT_EQ(s1[0].second, s0[0].second);
  // A step on the sink's home moves it.
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  a->send("spin");
  ASSERT_TRUE(a->round(deadline_after(10.0), false));
  const auto s2 = sinks();
  ASSERT_EQ(s2.size(), 1u);
  EXPECT_GT(s2[0].second, s1[0].second);
  std::printf(
      "ProgressStore %s: the sink's cell moved by %.1f us\n", kind.c_str(),
      std::chrono::duration<double, std::micro>(steady::duration(s2[0].second - s1[0].second))
          .count());
  a->release(h);
  ASSERT_TRUE(a->step_until([&] { return a->finished(h) && a->finished(spinner); }));
  ASSERT_TRUE(b->step_until([&] { return b->finished(w) && b->finished(other); }));
  EXPECT_TRUE(sinks().empty());
  EXPECT_TRUE(mgr->table().empty());
}

// ---------------------------------------------------------------------------
// LT-4 and LT-10: random programs on several homes, each home's scheduler stepped by a thread of
// its own one step at a time (as Exec::run drives its scheduler), every operation checked by the
// manager (CVC_STATE_EXEC_CHECK=1), and stops where every home has delivered and drained its
// port, for the quiescent clauses of protocol 6 (I6's second half, I8's quiescent clause).
// ---------------------------------------------------------------------------

namespace {

// LT-4's and LT-10's programs (gap G-42): sections over 16 keys k0..k15, of one key or of two or
// three, S and X, waiting, with :timeout 0, with :if-available, or with a timeout of 1-20 ms, some
// nested, and set-priority between and inside them. An X section at top level counts itself into
// in.<key> for each of its keys and out again, and a count other than 1 inside is an overlap
// (state-incr "overlap"): the exclusion as the programs see it. A holder killed inside leaves its
// count at 1; the key's abandoned flag (I11) tells the next X holder, which resets it.
struct thread_programs {
  std::mt19937_64 rng;
  explicit thread_programs(std::uint64_t seed) : rng(seed) {}
  int pick(int n) { return std::uniform_int_distribution<int>(0, n - 1)(rng); }

  std::vector<int> keys() {
    std::vector<int> ks{pick(16)};
    for (int i = 0, n = pick(4) == 0 ? 1 + pick(2) : 0; i < n; ++i)
      if (const int k = pick(16); std::find(ks.begin(), ks.end(), k) == ks.end())
        ks.push_back(k);
    return ks;
  }
  static std::string key_text(const std::vector<int> &ks) {
    if (ks.size() == 1)
      return "\"k" + std::to_string(ks[0]) + "\"";
    std::string l = "(list";
    for (int k : ks)
      l += " \"k" + std::to_string(k) + "\"";
    return l + ")";
  }
  std::string options() {
    const int t = pick(10);
    if (t < 3)
      return ""; // waits as long as it takes
    if (t < 4)
      return " :timeout 0";
    if (t < 5)
      return " :if-available :else (state-incr \"else\")";
    char buf[32];
    std::snprintf(buf, sizeof buf, " :timeout %.3f", 0.001 * (1 + pick(20)));
    return buf;
  }
  std::string priority() { return "(set-priority " + std::to_string(pick(40) - 20) + ")"; }
  // `held`: the keys of the sections around this one, which the same process holds.
  std::string body(int depth, const std::vector<int> &held) {
    switch (pick(depth < 2 ? 6 : 4)) {
    case 0:
      return "(sleep 0)";
    case 1:
      return "(state-incr \"n\")";
    case 2:
      return "(locks-held)";
    case 3:
      return priority();
    default:
      return section(depth + 1, held);
    }
  }
  std::string section(int depth, const std::vector<int> &held) {
    const std::vector<int> ks = keys();
    const bool x = pick(2) == 0;
    std::string reset, in, out;
    if (x)
      for (int k : ks) {
        const std::string count = "\"in.k" + std::to_string(k) + "\"";
        // A grant reports `abandoned` for any of its keys. A key the process already holds
        // carries no flag (its own X grant took it, and an X request on its own S key is
        // refused), and its count belongs to the section around this one: left alone.
        if (std::find(held.begin(), held.end(), k) == held.end())
          reset += " (state-set " + count + " \"0\")";
        if (depth == 0) {
          in += " (begin (set c (state-incr " + count +
                ")) (if (!= c 1) (begin (state-incr \"overlap\") (hook (list \"overlap\" \"k" +
                std::to_string(k) + "\" (self-pid) c))) nil))";
          out += " (state-incr " + count + " -1)";
        }
      }
    std::vector<int> inner = held;
    inner.insert(inner.end(), ks.begin(), ks.end());
    std::string b = "(begin";
    if (!reset.empty())
      b += " (if (get-attr g \"abandoned\") (begin" + reset + ") nil)";
    b += in + " " + body(depth, inner) + " " + body(depth, inner) + out + ")";
    const std::string lock =
        "(with-lock " + key_text(ks) + (x ? "" : " :shared") + options() + " :as g " + b + ")";
    return "(try " + lock +
           " (catch (TimeoutError e) nil) (catch (DeadlockError e) nil) (catch (LockError e) nil))";
  }
  std::string program() {
    std::string p = "(begin";
    if (pick(2))
      p += " " + priority();
    for (int i = 0, n = 3 + pick(6); i < n; ++i)
      p += " " + section(0, {});
    return p + ")";
  }
};

// How a run_threads run is made.
struct threads_config {
  std::string name;
  std::vector<std::string> kinds; // one home per entry, each on a thread of its own
  std::vector<scheduling_policy> policies;
  std::size_t target_ops = 0; // operations: grants, timeouts and deadlocks
  std::size_t stop_every = 0; // the drivers stop every this many operations
  int per_home = 4;           // programs live on each home at a time
  std::uint64_t seed = 0;
  std::uint64_t fuzz_seed = 0; // the manager's CVC_EXEC_FUZZ seed (0: off)
  double max_seconds = 120.0;
  bool print = true;
};

struct threads_result {
  std::size_t ops = 0;
  std::uint64_t stops = 0, kills = 0, ended = 0;
  lock_stats stats;
  std::uint64_t fuzz_hits = 0; // the manager's fuzzed protocol points
};

// A home's thread kills one of its programs every this many rounds: with 4 programs a home, about
// as often per program as LK-18's 50 rounds over 16.
constexpr std::uint64_t kKillEvery = 200;

// Each home's thread: steps its scheduler a round at a time (step, then check_tables(); the async
// one as Exec::run does: drain_ingress, sync_step, wait_for_work when nothing stepped), kills one
// of its programs every kKillEvery rounds, replaces each that ends while the operations are short
// of the target, and stops whenever the operations reach the next stop: every thread waits at a
// barrier (every delivery is done, since each operation delivers before its action returns),
// drains its home's port, waits again (every post applied), checks its home's quiescent clauses
// and process table while one checks the manager, and waits a third time before it steps again
// (a home's clauses read the manager, which another home's step would change). At the end every
// program has ended: the ports are drained once more, every home checked, and the manager is
// empty (I9).
threads_result run_threads(const threads_config &c) {
  const auto t0 = steady::now();
  cvc::app app; // the programs' tree, shared: in.<key>, n, else, overlap
  cvc::state &tree = cvc::state::instance(app);
  const auto mgr = shared_manager();
  mgr->set_fuzz_seed(c.fuzz_seed);
  const std::size_t n = c.kinds.size();
  std::vector<std::unique_ptr<checked_world>> worlds;
  for (std::size_t i = 0; i < n; ++i)
    worlds.push_back(home_on(c.kinds[i], mgr, c.policies[i], world_checks::stepping, &tree));
  const auto done_ops = [&] {
    const lock_stats s = mgr->stats();
    return static_cast<std::size_t>(s.grants + s.timeouts + s.deadlocks);
  };

  std::mutex errors_mu;
  std::vector<std::string> errors;
  std::atomic<bool> failed{false};
  const auto fail = [&](const std::string &what) {
    std::lock_guard<std::mutex> g(errors_mu);
    errors.push_back(what);
    failed = true;
  };
  std::atomic<std::size_t> stop_at{c.stop_every};
  std::atomic<std::uint64_t> kills{0}, ended{0};
  std::uint64_t phases = 0, stops = 0; // the barrier's completion only
  auto completion = [&]() noexcept {
    switch (++phases % 3) {
    case 2: // every port drained, nothing stepping
      try {
        mgr->check();
      } catch (const std::exception &e) {
        fail(std::string("at stop ") + std::to_string(stops + 1) + ": " + e.what());
      }
      break;
    case 0: // every home checked: on to the next stop
      ++stops;
      stop_at.fetch_add(c.stop_every);
      break;
    default:
      break;
    }
  };
  std::barrier<decltype(completion)> sync(static_cast<std::ptrdiff_t>(n), completion);
  const steady::time_point until = deadline_after(c.max_seconds);

  const auto drive = [&](std::size_t i) {
    checked_world &v = *worlds[i];
    const std::string who = c.kinds[i] + " home " + std::to_string(i);
    // A program that saw an overlap says which key, its pid and the count it found.
    v.on_hook = [&, who](const std::string &what) { fail(who + ": " + what); };
    thread_programs gen(c.seed * 7919 + i);
    std::vector<int> live;
    const auto spawn = [&] {
      checked_world::submit_options so;
      so.opts.priority = gen.pick(16) - 5;
      so.opts.priority_floor = -20; // a host's floor: set-priority may take any level
      live.push_back(v.submit(gen.program(), so));
    };
    const auto stop = [&] {
      sync.arrive_and_wait();
      v.home().drain_port();
      sync.arrive_and_wait();
      try {
        v.home().check_quiescent();
      } catch (const detail::lock_violation &e) {
        fail(who + " at a stop: " + e.what());
      }
      if (!v.base().check_tables())
        fail(who + " at a stop: the process table's check failed");
      sync.arrive_and_wait();
    };
    try {
      for (int k = 0; k < c.per_home; ++k)
        spawn();
      std::uint64_t rounds = 0;
      while (!live.empty() && !failed) {
        if (steady::now() >= until) {
          fail(who + ": " + std::to_string(live.size()) + " programs still running after " +
               std::to_string(c.max_seconds) + " s");
          break;
        }
        v.round(until);
        if (v.broken) {
          fail(who + ": a check failed after a step");
          break;
        }
        if (++rounds % kKillEvery == 0 &&
            v.kill(live[static_cast<std::size_t>(gen.pick(static_cast<int>(live.size())))]))
          ++kills;
        std::size_t replace = 0;
        for (auto it = live.begin(); it != live.end();) {
          const int pid = *it;
          if (!v.finished(pid)) {
            ++it;
            continue;
          }
          const auto info = v.info(pid);
          if (info->status == process_status::failed)
            fail(who + ": pid " + std::to_string(pid) +
                 " failed: " + info->exit_error.value_or(""));
          ++ended;
          it = live.erase(it);
          reap(v, pid);
          if (done_ops() < c.target_ops)
            ++replace;
        }
        for (std::size_t k = 0; k < replace; ++k)
          spawn();
        if (done_ops() >= stop_at.load())
          stop();
      }
      for (int pid : live) // left on a failure
        v.kill(pid);
    } catch (const std::exception &e) {
      fail(who + ": " + e.what());
    }
    sync.arrive_and_drop();
  };
  std::vector<std::thread> threads;
  for (std::size_t i = 0; i < n; ++i)
    threads.emplace_back(drive, i);
  for (std::thread &t : threads)
    t.join();
  for (const std::string &e : errors)
    ADD_FAILURE() << c.name << ": " << e;

  // The end: what is left in the ports (posts for programs that ended) drained, then the quiescent
  // clauses of every home, and the manager empty (I9).
  for (std::size_t i = 0; i < n; ++i) {
    checked_world &v = *worlds[i];
    v.home().drain_port();
    try {
      v.home().check_quiescent();
    } catch (const detail::lock_violation &e) {
      ADD_FAILURE() << c.name << ": home " << i << " at the end: " << e.what();
    }
    EXPECT_TRUE(v.base().check_tables()) << c.name << ": home " << i;
  }
  threads_result r;
  r.ops = done_ops();
  r.stops = stops;
  r.kills = kills;
  r.ended = ended;
  r.stats = mgr->stats();
  EXPECT_NO_THROW(mgr->check()) << c.name;
  EXPECT_TRUE(mgr->table().empty()) << c.name << ": " << mgr->table().canonical();
  EXPECT_EQ(r.stats.holds, 0u) << c.name;
  EXPECT_EQ(r.stats.queued, 0u) << c.name;
  EXPECT_EQ(tree.findDescendant("overlap"), nullptr)
      << c.name << ": two X sections were inside one key at once";
  if (!failed)
    EXPECT_GE(r.ops, c.target_ops) << c.name;
  if (c.print) {
    const lock_stats &s = r.stats;
    std::printf(
        "%s: %zu operations, %llu stops, %llu kills, %llu ended; grants %llu (fast %llu), "
        "waits %llu, timeouts %llu, deadlocks %llu, abandoned %llu, relevels %llu; posts "
        "%llu, dropped %llu, discarded %llu; %.2f s\n",
        c.name.c_str(), r.ops, static_cast<unsigned long long>(r.stops),
        static_cast<unsigned long long>(r.kills), static_cast<unsigned long long>(r.ended),
        static_cast<unsigned long long>(s.grants), static_cast<unsigned long long>(s.fast_grants),
        static_cast<unsigned long long>(s.waits), static_cast<unsigned long long>(s.timeouts),
        static_cast<unsigned long long>(s.deadlocks), static_cast<unsigned long long>(s.abandoned),
        static_cast<unsigned long long>(s.relevels), static_cast<unsigned long long>(s.posts),
        static_cast<unsigned long long>(s.posts_dropped),
        static_cast<unsigned long long>(s.posts_discarded), seconds_since(t0));
  }
  r.fuzz_hits = mgr->fuzz_hits();
  return r;
}

// LT-4: CVC_EXEC_FUZZ's seeds 1..seeds (the run's manager's, threads_config::fuzz_seed: with a
// seed, each protocol point runs on, yields its thread, or pauses it 0-50 us), each a run of 4
// homes on 4 threads (sync and async, round robin and priority_rr) for `ops` operations, stopping
// every 100.
void fuzzed_protocol_points(std::size_t seeds, std::size_t ops_per_seed,
                            double max_seconds_per_seed) {
  const auto t0 = steady::now();
  std::size_t ops = 0;
  std::uint64_t stops = 0, posts = 0, relevels = 0, hits = 0;
  std::size_t seed = 1;
  for (; seed <= seeds && !::testing::Test::HasFailure(); ++seed) {
    threads_config c;
    c.fuzz_seed = seed;
    c.name = "LT-4 seed " + std::to_string(seed);
    c.kinds = {"sync", "async", "sync", "async"};
    c.policies = {scheduling_policy::round_robin, scheduling_policy::priority_rr,
                  scheduling_policy::priority_rr, scheduling_policy::round_robin};
    c.target_ops = ops_per_seed;
    c.stop_every = 100;
    c.seed = seed;
    c.max_seconds = max_seconds_per_seed;
    c.print = false;
    const threads_result r = run_threads(c);
    ops += r.ops;
    stops += r.stops;
    posts += r.stats.posts;
    relevels += r.stats.relevels;
    // The seed was in effect: its runs reached the protocol points.
    EXPECT_GT(r.fuzz_hits, 0u) << c.name;
    hits += r.fuzz_hits;
  }
  std::printf("LT-4: %zu seeds, %zu operations, %llu stops, %llu posts, %llu relevels, %llu fuzzed "
              "protocol points; %.2f s\n",
              seed - 1, ops, static_cast<unsigned long long>(stops),
              static_cast<unsigned long long>(posts), static_cast<unsigned long long>(relevels),
              static_cast<unsigned long long>(hits), seconds_since(t0));
  EXPECT_GT(posts, 0u); // the homes did meet
}

// LT-10: 8 priority_rr async_schedulers on 8 threads, attached with cross_thread, stopping every
// 1000 operations.
void eight_schedulers_random(std::size_t ops, double max_seconds) {
  threads_config c;
  c.name = "LT-10";
  c.kinds.assign(8, "async");
  c.policies.assign(8, scheduling_policy::priority_rr);
  c.target_ops = ops;
  c.stop_every = 1000;
  c.seed = 10;
  c.max_seconds = max_seconds;
  const threads_result r = run_threads(c);
  EXPECT_GT(r.stats.posts, 0u);
  EXPECT_GT(r.stats.waits, 0u);
  EXPECT_GT(r.stats.timeouts, 0u);
  EXPECT_GT(r.stats.relevels, 0u);
  EXPECT_GE(r.stops + 1, ops / 1000);
}

} // namespace

// LT-4: fuzzed yields at the protocol points (CVC_EXEC_FUZZ, L3.7), 100 seeds (20 unoptimized or
// under a sanitizer; CVC_LOCKS_FUZZ_SEEDS) of 200 operations each (100 unoptimized, where an
// operation costs 30-40 times what it does in Release: LK-18's measure; CVC_LOCKS_FUZZ_OPS): no
// lost wakeup (every program ends), no double grant (I1 after every operation, and no overlap the
// programs see), FIFO kept (I5, asserted at each grant by the self-check), check() after every
// operation, check_quiescent() at each stop and at the end, the manager empty at the end (I9).
TEST(LocksThreads, LT04_FuzzedProtocolPoints) {
  fuzzed_protocol_points(lane_size("CVC_LOCKS_FUZZ_SEEDS", 100, 20, 20),
                         lane_size("CVC_LOCKS_FUZZ_OPS", 200, 100, 200), 120.0);
}

// LT-10 (gap G-42): 10^4 operations (2*10^3 unoptimized or under a sanitizer;
// CVC_LOCKS_THREAD_OPS).
TEST(LocksThreads, LT10_EightSchedulersRandom) {
  eight_schedulers_random(lane_size("CVC_LOCKS_THREAD_OPS", 10000, 2000, 2000), 240.0);
}

TEST(LocksStress, LT04_FuzzedProtocolPoints) { fuzzed_protocol_points(1000, 200, 120.0); }

TEST(LocksStress, LT10_EightSchedulersRandom) { eight_schedulers_random(100000, 3000.0); }
