// Threaded execution, Stage 0 (docs/STATE_EXEC_THREADED.md 7, 8.1): the process-global hazards a
// second thread would hit today, each case one HZ test of td 8.1 that needs threads or a whole
// host rather than one intrinsic. The sanitizer jobs run this binary (ci.yml's SAN_CXX_TESTS) and
// require each case's line. No case name contains "Stress": run.sh filters those out.

#include <atomic>
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

  int submit(const std::string &script) {
    execute_options opts;
    opts.env = env;
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
