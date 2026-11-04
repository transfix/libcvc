// Threaded execution, Stage 0 (docs/STATE_EXEC_THREADED.md 7, 8.1): the process-global hazards a
// second thread would hit today, each case one HZ test of td 8.1 that needs threads or a whole
// host rather than one intrinsic. The sanitizer jobs run this binary (ci.yml's SAN_CXX_TESTS) and
// require each case's line. No case name contains "Stress": run.sh filters those out.

#include <atomic>
#include <boost/any.hpp>
#include <boost/signals2/connection.hpp>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/process.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::state_exec;

namespace {

using clock_type = std::chrono::steady_clock;

double seconds_since(clock_type::time_point t) {
  return std::chrono::duration<double>(clock_type::now() - t).count();
}

// The sanitizer jobs set CVC_LOCKS_SANITIZER=1. Their builds run 5-15x slower, so the cases on
// several threads run fewer iterations there.
bool under_sanitizer() {
  const char *v = std::getenv("CVC_LOCKS_SANITIZER");
  return v && *v && std::string(v) != "0";
}

// A process's backstop: a case's processes end long before this, unless a hazard hangs one.
constexpr double kProcessCeiling = 300.0;

// One program's world, as a pycvc Exec or an Ariadne document has it: its own async_scheduler,
// slicing deep data `step_nodes` nodes a step, and an intrinsics context on the shared tree,
// chrooted at `chroot`. A world is driven by one thread.
struct tree_program {
  async_scheduler sched;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();

  explicit tree_program(cvc::state &tree, std::size_t step_nodes = 0,
                        const std::string &chroot = {}) {
    data_options data;
    data.step_nodes = step_nodes;
    sched.set_data_defaults(data);
    ictx.sched = &sched;
    ictx.proc = proc;
    apply_chroot(ictx, tree, chroot);
    register_intrinsics(env, &ictx);
  }
  tree_program(const tree_program &) = delete;
  tree_program &operator=(const tree_program &) = delete;

  // Each process gets a scope of its own over the world's environment, so two programs' top-level
  // variables don't meet.
  int submit(const std::string &script) {
    execute_options opts;
    opts.env = environment::extend(env);
    opts.max_time = kProcessCeiling;
    const int pid = sched.execute(script, opts);
    ictx.pid = pid;
    proc->pid = pid;
    return pid;
  }

  process_info info(int pid) const { return sched.get_process_info(pid).value(); }
  bool finished(int pid) const { return is_finished(info(pid).status); }

  // What `pid` came to: its result (a string bare), or its status and exit error.
  std::string outcome(int pid) const {
    if (const auto r = sched.get_result(pid))
      return std::holds_alternative<std::string>(r->v) ? std::get<std::string>(r->v)
                                                       : to_string(*r);
    const auto i = sched.get_process_info(pid);
    return std::string("<") + (i ? to_string(i->status) : "gone") + " " +
           sched.get_exit_error(pid).value_or("") + ">";
  }

  // Run `script` to its end on this thread.
  std::string run(const std::string &script) {
    const int pid = submit(script);
    sched.sync_run();
    return outcome(pid);
  }

  // Step until `done()` or `max_steps` steps; whether done() held.
  template <class Done> bool step_until(Done done, int max_steps = 1000000) {
    for (int i = 0; i < max_steps && !done(); ++i)
      sched.sync_step();
    return done();
  }
};

// Run each script on a thread of its own, each in its own world on `tree`, all starting together.
// The outcomes, in order; the time it took is printed under `label`.
std::vector<std::string> run_on_threads(cvc::state &tree, const std::vector<std::string> &scripts,
                                        const char *label, std::size_t step_nodes = 0) {
  std::vector<std::string> out(scripts.size());
  std::atomic<std::size_t> ready{0};
  std::vector<std::thread> threads;
  const auto t0 = clock_type::now();
  for (std::size_t k = 0; k < scripts.size(); ++k)
    threads.emplace_back([&, k] {
      tree_program prog(tree, step_nodes);
      ready.fetch_add(1);
      while (ready.load() < scripts.size())
        std::this_thread::yield();
      out[k] = prog.run(scripts[k]);
    });
  for (std::thread &t : threads)
    t.join();
  std::printf("%s: %zu threads took %.3f s\n", label, scripts.size(), seconds_since(t0));
  return out;
}

// (begin (set i 0) (while (< i n) (begin body (set i (+ i 1)))) result)
std::string loop(int n, const std::string &body, const std::string &result = "i") {
  return "(begin (set i 0) (while (< i " + std::to_string(n) + ") (begin " + body +
         " (set i (+ i 1)))) " + result + ")";
}

} // namespace

// --- Stage 0 hazards (td 8.1) ---

// HZ-6 (td 8.1, S0.3, S0.4; ASan and TSan): one thread deletes and re-creates p.x while another
// reads its siblings, the parent's children and p.x.y, and a third writes p.x.y, each in its own
// world on one tree. A read gives a value or nil, the siblings never change, and the sanitizers
// report nothing. The subtree reads are what fail today: they walk nodes the delete frees.
TEST(Hazards, HZ06_DeleteRecreateWhileReading) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  tree("p.s1").value(std::string("one"));
  tree("p.s2").value(std::string("two"));
  tree("p.x.y").value(std::string("0"));
  const int n = under_sanitizer() ? 2000 : 20000;
  const std::string deleter = loop(n, R"((state-delete "p.x") (state-set "p.x.y" "a"))");
  const std::string reader = loop(n, R"(
      (if (!= (state-get "p.s1") "one") (set bad (+ bad 1)))
      (if (!= (state-get "p.s2") "two") (set bad (+ bad 1)))
      (if (< (length (state-children "p")) 2) (set bad (+ bad 1)))
      (set v (state-get "p.x.y"))
      (if (not (or (is-null v) (is-string v))) (set bad (+ bad 1))))",
                                  "bad");
  const std::string writer = loop(n, R"((state-set "p.x.y" (str i)))");
  const std::vector<std::string> out =
      run_on_threads(tree, {deleter, "(begin (set bad 0) " + reader + ")", writer}, "HZ-6");
  EXPECT_EQ(out[0], std::to_string(n));
  EXPECT_EQ(out[1], "0"); // no read saw a sibling change or a value that was neither
  EXPECT_EQ(out[2], std::to_string(n));
  EXPECT_EQ(tree("p.s1").value(), "one");
  EXPECT_EQ(tree("p.s2").value(), "two");
}

// HZ-6b (td 8.1, S0.4; ASan, one thread): a state-set-tree sliced over many steps (data.step_nodes
// 1) whose target's ancestor another process deletes between two of its slices. The job keeps the
// target's chain, so its later slices write to the unlinked subtree and nothing it uses is freed.
TEST(Hazards, HZ06b_SetTreeSurvivesAncestorDelete) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  tree_program prog(tree, 1);
  const int writer =
      prog.submit("(begin (set d (dict)) " +
                  loop(100, R"((set-attr d (str i) (dict "v" i "w" (list i (str i)))))", "d") +
                  R"( (state-set-tree "p.t" d) "done"))");
  // The writer alone, until its job has written part of the tree.
  ASSERT_TRUE(prog.step_until([&] { return tree.findDescendant("p.t.5.w") != nullptr; }));
  ASSERT_FALSE(prog.finished(writer));
  ASSERT_EQ(tree.findDescendant("p.t.99"), nullptr) << "the job wasn't sliced";
  const int deleter = prog.submit(R"((begin (state-delete "p") "deleted"))");
  ASSERT_TRUE(prog.step_until([&] { return prog.finished(deleter); }));
  EXPECT_EQ(prog.outcome(deleter), "deleted");
  EXPECT_EQ(tree.findDescendant("p.t"), nullptr);
  ASSERT_FALSE(prog.finished(writer)) << "the delete didn't land between the job's slices";
  std::printf("HZ-6b: p deleted after %llu of the writer's data steps\n",
              static_cast<unsigned long long>(prog.info(writer).data_steps));
  std::fflush(stdout);
  ASSERT_TRUE(prog.step_until([&] { return prog.finished(writer); }));
  EXPECT_EQ(prog.outcome(writer), "done");
  std::printf("HZ-6b: after the job, p is %s\n", tree.findDescendant("p") ? "there" : "gone");
}

// HZ-3 (td 8.1, S0.2), in a program: a list changed after state-data-set stored it doesn't change
// what state-data-get returns, whether the copy is made whole in the step (data.step_nodes 0) or a
// node a step (1).
TEST(Hazards, HZ03_StateDataSetStoresACopy) {
  for (const std::size_t step_nodes : {std::size_t{0}, std::size_t{1}}) {
    SCOPED_TRACE("data.step_nodes " + std::to_string(step_nodes));
    cvc::app app;
    tree_program prog(cvc::state::instance(app), step_nodes);
    EXPECT_EQ(prog.run(R"((begin (set l (list 1 2)) (state-data-set "k" l) (set-nth l 0 9)
                                 (state-data-get "k")))"),
              "(1 2)");
    EXPECT_EQ(prog.run(R"((begin (set l (list 1 (list 2))) (state-data-set "k" l)
                                 (append (nth l 1) 3) (state-data-get "k")))"),
              "(1 (2))");
  }
}

// HZ-3b (S0.2): the copy is the writer's deep-data work. Storing a 1000-element list adds at least
// 1000 data nodes to the writer's count; at data.step_nodes 1 it adds at least 900 data steps (a
// slice a node, the last ones possibly merged), and at 0 none. A process stepped beside the writer
// is charged nothing for it. The writer is compared with the same program storing nothing.
TEST(Hazards, HZ03b_StateDataSetChargesTheWriter) {
  const std::string build = loop(1000, "(append l i)", "(length l)");
  for (const std::size_t step_nodes : {std::size_t{0}, std::size_t{1}}) {
    SCOPED_TRACE("data.step_nodes " + std::to_string(step_nodes));
    cvc::app app;
    tree_program prog(cvc::state::instance(app), step_nodes);
    const int plain = prog.submit("(begin (set l (list)) " + build + ")");
    prog.sched.sync_run();
    const int writer =
        prog.submit("(begin (set l (list)) " + build + R"( (state-data-set "k" l) (length l)))");
    const int other = prog.submit(loop(2000, "(set x (+ i 1))"));
    prog.sched.sync_run();
    ASSERT_EQ(prog.outcome(plain), "1000");
    ASSERT_EQ(prog.outcome(writer), "1000");
    ASSERT_EQ(prog.outcome(other), "2000");
    const process_info p = prog.info(plain), w = prog.info(writer), o = prog.info(other);
    std::printf("HZ-3b at step_nodes %zu: data_nodes %llu -> %llu, data_steps %llu -> %llu\n",
                step_nodes, static_cast<unsigned long long>(p.data_nodes),
                static_cast<unsigned long long>(w.data_nodes),
                static_cast<unsigned long long>(p.data_steps),
                static_cast<unsigned long long>(w.data_steps));
    EXPECT_GE(w.data_nodes, p.data_nodes + 1000);
    if (step_nodes == 1)
      EXPECT_GE(w.data_steps, p.data_steps + 900);
    else
      EXPECT_EQ(w.data_steps, p.data_steps);
    EXPECT_EQ(o.data_nodes, 0u);
    EXPECT_EQ(o.data_steps, 0u);
  }
}

// HZ-12 (td 8.1, 4.11; S0.10): print writes whole lines. 8 threads, each with its own scheduler,
// print 1000 lines each with a 512-character payload. Every other line has a list argument, so
// the line is built by the concat job, and half the threads slice deep data a node a step. The
// captured output is exactly 8000 lines, each of them whole, and each thread's in its own order.
TEST(Hazards, HZ12_PrintWritesWholeLines) {
  constexpr int kThreads = 8;
  constexpr int kLines = 1000;
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  const auto payload = [](int t) { return std::string(512, static_cast<char>('a' + t)); };
  const auto expected = [&](int t, int i) {
    const std::string p = "\"" + payload(t) + "\"";
    return std::to_string(t) + " " + std::to_string(i) + " " + (i % 2 == 0 ? p : "(" + p + ")");
  };
  std::vector<std::string> outcomes(kThreads);
  std::atomic<int> ready{0};
  std::vector<std::thread> threads;
  testing::internal::CaptureStdout();
  const auto t0 = clock_type::now();
  for (int t = 0; t < kThreads; ++t)
    threads.emplace_back([&, t] {
      tree_program prog(tree, t % 2 == 0 ? 0 : 1);
      const std::string script =
          "(begin (set p \"" + payload(t) + "\") " +
          loop(kLines, "(if (= (% i 2) 0) (print " + std::to_string(t) + " i p) (print " +
                           std::to_string(t) + " i (list p)))");
      ready.fetch_add(1);
      while (ready.load() < kThreads)
        std::this_thread::yield();
      outcomes[t] = prog.run(script + ")");
    });
  for (std::thread &th : threads)
    th.join();
  const double took = seconds_since(t0);
  const std::string out = testing::internal::GetCapturedStdout();
  std::printf("HZ-12: %d threads x %d lines took %.3f s, %zu bytes\n", kThreads, kLines, took,
              out.size());
  for (int t = 0; t < kThreads; ++t)
    EXPECT_EQ(outcomes[t], std::to_string(kLines)) << "thread " << t;

  std::vector<int> next(kThreads, 0);
  std::size_t lines = 0, broken = 0;
  std::size_t start = 0;
  while (start < out.size()) {
    std::size_t end = out.find('\n', start);
    if (end == std::string::npos)
      end = out.size();
    std::string line = out.substr(start, end - start);
    start = end + 1;
    if (!line.empty() && line.back() == '\r')
      line.pop_back();
    ++lines;
    // "<t> <i> ...": the thread's next line, whole.
    const std::size_t s1 = line.find(' '),
                      s2 = s1 == std::string::npos ? s1 : line.find(' ', s1 + 1);
    int t = -1, i = -1;
    if (s2 != std::string::npos) {
      try {
        t = std::stoi(line.substr(0, s1));
        i = std::stoi(line.substr(s1 + 1, s2 - s1 - 1));
      } catch (const std::exception &) {
        t = -1;
      }
    }
    if (t < 0 || t >= kThreads || i != next[t] || line != expected(t, i)) {
      if (++broken <= 3)
        ADD_FAILURE() << "line " << lines << " is not whole or out of order: " << line.substr(0, 80)
                      << (line.size() > 80 ? "..." : "");
      continue;
    }
    ++next[t];
  }
  EXPECT_EQ(lines, static_cast<std::size_t>(kThreads * kLines));
  EXPECT_EQ(broken, 0u);
  for (int t = 0; t < kThreads; ++t)
    EXPECT_EQ(next[t], kLines) << "thread " << t;
}

// --- R0-mid: the tree's memory fixes ---

namespace {

// Set when the node holding it as data has had its members destroyed: its ~state() is done.
struct gone_flag {
  std::shared_ptr<std::atomic<bool>> gone;
  ~gone_flag() {
    if (gone)
      gone->store(true);
  }
};

std::shared_ptr<std::atomic<bool>> flag_when_freed(cvc::state &node) {
  auto flag = std::make_shared<std::atomic<bool>>(false);
  auto g = std::make_shared<gone_flag>();
  g->gone = flag;
  node.data(boost::any(g));
  return flag;
}

// Whether `node` died, and whether its parent was already freed then: ~state() promises a node
// its ancestors while it dies (a destroyed() slot may walk _parent).
struct death_watch {
  bool died = false, parent_gone = false;
  boost::signals2::scoped_connection conn;
  death_watch(cvc::state &node, std::shared_ptr<std::atomic<bool>> parent_freed) {
    conn = node.destroyed.connect([this, parent_freed] {
      died = true;
      parent_gone = parent_freed->load();
    });
  }
};

// {"k0": "0", ...}: a document of `n` values, as the body of a DSL string literal.
std::string json_values(int n) {
  std::string doc = "{";
  for (int i = 0; i < n; ++i)
    doc += (i ? ", " : "") + std::string("\\\"k") + std::to_string(i) + "\\\": \\\"" +
           std::to_string(i) + "\\\"";
  return doc + "}";
}

} // namespace

// R0m-memory-2 (S0.4; ASan and TSan): state-from-json writes each value through a walk that holds
// the node it writes, and a dotted key's nodes above it, until the write is done. One thread
// loads a document naming a child and two dotted paths again and again while another deletes what
// it writes, each in its own world on one tree. Handing the reader each node unpinned let the delete
// free it before the write (a crash within seconds); here both loops finish.
TEST(Hazards, FromJsonAgainstDeletingWhatItWrites) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  const int n = under_sanitizer() ? 20000 : 100000;
  const std::string writer =
      loop(n, R"((state-from-json "p" "{\"k\": \"v\", \"a.b\": \"w\", \"a.c.d\": \"x\"}"))");
  const std::string deleter = loop(n, R"((state-delete "p.k") (state-delete "p.a"))");
  const std::vector<std::string> out =
      run_on_threads(tree, {writer, deleter}, "state-from-json against delete");
  EXPECT_EQ(out[0], std::to_string(n));
  EXPECT_EQ(out[1], std::to_string(n));
}

// R0m-memory-4 (S0.4): a context's root_pins are released leaf first. The context outlives the
// document it is chrooted in; once both are gone, the chroot node dies before its parent.
TEST(Hazards, ContextRootPinsReleaseTheChrootFirst) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  auto ctx = std::make_unique<intrinsics_context>();
  apply_chroot(*ctx, tree, "a.b");
  const auto a_freed = flag_when_freed(tree("a"));
  death_watch b(tree("a.b"), a_freed);
  ASSERT_TRUE(tree.removeChild("a")); // the document goes; the context still holds it
  EXPECT_FALSE(a_freed->load());
  EXPECT_FALSE(b.died);
  ctx.reset(); // root_pins [root, a, a.b] go
  EXPECT_TRUE(b.died);
  EXPECT_FALSE(b.parent_gone) << "a.b died after its parent a was freed";
  EXPECT_TRUE(a_freed->load());
}

// R0m-memory-4 (S0.4): a data job's chain is released leaf first. A state-set-tree sliced a node a
// step (HZ-6b) whose target's parent another process deletes between slices: when the job ends,
// it is the last owner of the deleted p and its target p.t.
TEST(Hazards, SetTreeJobPinsReleaseTheTargetFirst) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  tree_program prog(tree, 1);
  const int writer =
      prog.submit("(begin (set d (dict)) " + loop(100, "(set-attr d (str i) i)", "d") +
                  R"( (state-set-tree "p.t" d) "done"))");
  ASSERT_TRUE(prog.step_until([&] { return tree.findDescendant("p.t.5") != nullptr; }));
  ASSERT_FALSE(prog.finished(writer));
  const auto p_freed = flag_when_freed(*tree.findDescendant("p"));
  death_watch t(*tree.findDescendant("p.t"), p_freed);
  const int deleter = prog.submit(R"((begin (state-delete "p") "deleted"))");
  ASSERT_TRUE(prog.step_until([&] { return prog.finished(deleter); }));
  ASSERT_FALSE(prog.finished(writer)) << "the delete didn't land between the job's slices";
  ASSERT_TRUE(prog.step_until([&] { return prog.finished(writer); }));
  EXPECT_EQ(prog.outcome(writer), "done");
  prog.step_until([&] { return t.died; }, 10);
  EXPECT_TRUE(t.died) << "the job's pins were not released";
  EXPECT_FALSE(t.parent_gone) << "p.t died after its parent p was freed";
}

// The same for a state-from-json sliced a value a step: its job holds the target's chain, the
// target and the reader's own hold on it.
TEST(Hazards, FromJsonJobPinsReleaseTheTargetFirst) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  tree_program prog(tree, 1);
  const int writer =
      prog.submit(R"((begin (state-from-json "p.t" ")" + json_values(100) + R"(") "done"))");
  // The reader checks the whole document first, then sets it: wait for a value it has set.
  ASSERT_TRUE(prog.step_until([&] { return tree.findDescendant("p.t.k5") != nullptr; }));
  ASSERT_FALSE(prog.finished(writer));
  const auto p_freed = flag_when_freed(*tree.findDescendant("p"));
  death_watch t(*tree.findDescendant("p.t"), p_freed);
  const int deleter = prog.submit(R"((begin (state-delete "p") "deleted"))");
  ASSERT_TRUE(prog.step_until([&] { return prog.finished(deleter); }));
  ASSERT_FALSE(prog.finished(writer)) << "the delete didn't land between the job's slices";
  ASSERT_TRUE(prog.step_until([&] { return prog.finished(writer); }));
  EXPECT_EQ(prog.outcome(writer), "done");
  prog.step_until([&] { return t.died; }, 10);
  EXPECT_TRUE(t.died) << "the job's pins were not released";
  EXPECT_FALSE(t.parent_gone) << "p.t died after its parent p was freed";
}

// --- R0-mid: the schedulers' settings walks (implementer S) ---

// R0m-memory-3 (R0-mid's probe M5): a scheduler reads its settings from its watch root (each
// submission re-reads them once something under state_exec changed: settings_changes) and
// publishes them there (load_settings), on its own thread, while another thread's program deletes
// and re-makes state_exec under the same root. Every read holds the nodes it walks, and the
// version's connection the node it is connected to, so neither walks a node the delete frees.
// Before, the readers stalled within 2 s on a freed node's mutex. HZ-6 missed it: its worlds have
// no watch root, so they read no settings.
TEST(Hazards, SettingsWalksSurviveADeleteOfStateExec) {
  cvc::app app;
  cvc::state &tree = cvc::state::instance(app);
  tree("state_exec.defaults.data.step_nodes").value(std::string("0"));
  const int n = under_sanitizer() ? 300 : 3000;
  const std::string deleter = loop(n, R"((state-delete "state_exec")
      (state-set "state_exec.defaults.limits.max_alloc_bytes" "0")
      (state-set "state_exec.defaults.data.step_nodes" "0"))");
  std::atomic<bool> deleting{true}, hung{false};
  std::atomic<std::size_t> submits{0}, loads{0};
  std::string deleted, wrong;
  const auto t0 = clock_type::now();
  std::thread reader([&] { // submissions: settings_for_submit's reads, per change
    async_scheduler sched;
    sched.set_watch_root(&tree);
    while (deleting.load()) {
      const int pid = sched.execute(std::string("1"));
      sched.sync_run();
      const auto r = sched.get_result(pid);
      if (!r || to_string(*r) != "1")
        wrong = r ? to_string(*r) : "<none>";
      sched.reap(pid);
      submits.fetch_add(1);
    }
  });
  std::thread publisher([&] { // load_settings: reads, then publishes under state_exec.schedulers
    scheduler sched;
    sched.set_watch_root(&tree);
    sched.set_id("hz_settings");
    while (deleting.load()) {
      sched.load_settings();
      loads.fetch_add(1);
    }
  });
  std::thread writer([&] {
    tree_program prog(tree);
    deleted = prog.run(deleter);
    deleting.store(false);
  });
  // A hang is the failure this guards against: report it rather than wait for good.
  std::thread watchdog([&] {
    while (deleting.load()) {
      if (seconds_since(t0) > kProcessCeiling) {
        hung.store(true);
        std::printf("Hazards.SettingsWalksSurviveADeleteOfStateExec: no end after %.0f s (%zu "
                    "submissions, %zu loads)\n",
                    kProcessCeiling, submits.load(), loads.load());
        std::fflush(stdout);
        std::_Exit(3);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  });
  writer.join();
  reader.join();
  publisher.join();
  watchdog.join();
  std::printf("settings walks: %d deletes of state_exec, %zu submissions, %zu load_settings, "
              "%.3f s\n",
              n, submits.load(), loads.load(), seconds_since(t0));
  EXPECT_FALSE(hung.load());
  EXPECT_EQ(deleted, std::to_string(n));
  EXPECT_EQ(wrong, "");
  EXPECT_GT(submits.load(), 0u);
  EXPECT_GT(loads.load(), 0u);
}
