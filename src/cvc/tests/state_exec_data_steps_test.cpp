// Deep-data work on the executor ("Deep data on the executor" in docs/STATE_EXEC_GRAMMAR.md):
// walks that run a slice at a time (data_job), the stackless evaluator slicing them
// (evaluator_state::data_step_nodes), the schedulers taking data.step_nodes from the state tree,
// and the budget split by kind (read / eval / data) that process_info, inspect and budget-usage
// report.

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/memory_tracker.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_exec/stdlib.h>
#include <cvc/state/state_exec/types.h>
#include <gtest/gtest.h>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace cvc::state_exec;

namespace {

value_t sym(const char *s) { return value_t{symbol{s}}; }

value_t nested_list(std::size_t n, value_t leaf = value_t{int64_t{1}}) {
  value_t v = make_list({std::move(leaf)});
  for (std::size_t i = 1; i < n; ++i)
    v = make_list({std::move(v)});
  return v;
}

// Lists and dicts alternating, each level also holding a string and a number.
value_t nested_mixed(std::size_t n) {
  value_t v = value_t{int64_t{0}};
  for (std::size_t i = 0; i < n; ++i)
    v = i % 2 ? make_dict({{"i", value_t{int64_t(i)}}, {"v", std::move(v)}})
              : make_list({value_t{std::string("s")}, std::move(v), value_t{1.5}});
  return v;
}

// Run `job` in slices of `slice` nodes. Returns its result and how many step() calls it took.
std::pair<value_t, int> run_sliced(data_job &job, std::size_t slice) {
  int calls = 1;
  while (!job.step(slice))
    ++calls;
  return {job.result(), calls};
}

// The script that builds two separate n-deep lists and compares them.
std::string compare_script(int n) {
  return "(begin (set a 1) (set b 1) (set i 0)"
         " (while (< i " +
         std::to_string(n) +
         ") (begin (set a (list a)) (set b (list b)) (set i (+ i 1))))"
         " (= a b))";
}

// Step `st` until it is done; returns the steps it took.
int step_to_done(stackless_evaluator &ev, evaluator_state &st) {
  int steps = 0;
  while (!ev.step(st))
    ++steps;
  return steps + 1;
}

} // namespace

// ---- the jobs ---------------------------------------------------------------

TEST(DataJob, SlicedWalksMatchWholeOnes) {
  const value_t a = nested_mixed(300), b = nested_mixed(300), shorter = nested_mixed(299);
  for (std::size_t slice : {1u, 2u, 7u, 100000u}) {
    SCOPED_TRACE("slice " + std::to_string(slice));
    auto eq = make_equal_job(a, b);
    auto [equal, calls] = run_sliced(*eq, slice);
    EXPECT_TRUE(std::get<bool>(equal.v));
    if (slice == 1)
      EXPECT_GE(calls, 600); // a node per call
    auto ne = make_equal_job(a, shorter);
    EXPECT_FALSE(std::get<bool>(run_sliced(*ne, slice).first.v));

    auto print = make_print_job(a);
    EXPECT_EQ(std::get<std::string>(run_sliced(*print, slice).first.v), to_string(a));

    auto copy = make_copy_job(a);
    value_t c = run_sliced(*copy, slice).first;
    EXPECT_TRUE(values_equal(c, a));
    EXPECT_NE(std::get<dict_ptr>(c.v).get(), std::get<dict_ptr>(a.v).get()); // 300 levels: a dict

    auto concat = make_concat_job({value_t{std::string("<")}, a, value_t{int64_t{3}}}, " ", "test");
    EXPECT_EQ(std::get<std::string>(run_sliced(*concat, slice).first.v),
              "< " + to_string(a) + " 3");

    const value_t tmpl = make_list({sym("x"), nested_list(50, sym("y")), sym("z")});
    auto sub =
        make_substitute_job(tmpl, {{"x", value_t{int64_t{1}}}, {"y", value_t{std::string("Y")}}});
    EXPECT_TRUE(values_equal(
        run_sliced(*sub, slice).first,
        make_list({value_t{int64_t{1}}, nested_list(50, value_t{std::string("Y")}), sym("z")})));
  }
}

TEST(DataJob, ConcatChecksTheAllocationLimit) {
  // str-concat's sliced path (a list or dict among its parts) is held to the same allocation
  // limit as its fast path: the same string passed many times can outgrow every part.
  eval_alloc_guard limit(std::size_t{100});
  const value_t big{std::string(60, 'x')};
  auto job = make_concat_job({big, make_list({value_t{int64_t{1}}}), big}, "", "str-concat");
  try {
    run_sliced(*job, 1);
    FAIL() << "expected the allocation limit to stop it";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("str-concat"), std::string::npos) << e.what();
  }
}

TEST(DataJob, NodesDoNotDependOnTheSlice) {
  const value_t v = nested_mixed(200);
  auto whole = make_print_job(v);
  whole->step(0);
  auto sliced = make_print_job(v);
  run_sliced(*sliced, 3);
  EXPECT_EQ(whole->nodes(), sliced->nodes());
  EXPECT_GE(whole->nodes(), 200u);
}

TEST(DataJob, CopyKeepsSharingAndCycles) {
  value_t inner = make_list({value_t{int64_t{3}}});
  auto copy = make_copy_job(make_list({inner, inner}));
  value_t c = run_to_completion(*copy);
  auto &l = *std::get<list_ptr>(c.v);
  EXPECT_EQ(std::get<list_ptr>(l[0].v).get(), std::get<list_ptr>(l[1].v).get());
  EXPECT_NE(std::get<list_ptr>(l[0].v).get(), std::get<list_ptr>(inner.v).get());

  auto cyclic = std::make_shared<std::vector<value_t>>();
  cyclic->push_back(value_t{cyclic});
  auto cc = make_copy_job(value_t{cyclic});
  value_t cy = run_sliced(*cc, 1).first;
  auto copied = std::get<list_ptr>(cy.v);
  EXPECT_EQ(std::get<list_ptr>((*copied)[0].v).get(), copied.get());
  copied->clear(); // break both cycles
  cyclic->clear();
}

TEST(DataJob, ValueChangedBetweenSlices) {
  // A job reads what it walks by index, holding it by owning pointer: a change between two
  // slices (another process's) changes the answer, never memory safety.
  list_ptr inner = std::make_shared<std::vector<value_t>>();
  for (int i = 0; i < 10; ++i)
    inner->push_back(value_t{int64_t(i)});
  value_t a = make_list({value_t{inner}});
  value_t b = make_list({make_list(std::vector<value_t>(*inner))});
  auto eq = make_equal_job(a, b);
  EXPECT_FALSE(eq->step(3));
  inner->clear();
  inner->shrink_to_fit();
  EXPECT_FALSE(std::get<bool>(run_sliced(*eq, 3).first.v));

  list_ptr printed = std::make_shared<std::vector<value_t>>(20, value_t{int64_t{7}});
  auto print = make_print_job(value_t{printed});
  EXPECT_FALSE(print->step(4));
  printed->clear();
  printed->shrink_to_fit();
  const std::string out = std::get<std::string>(run_sliced(*print, 4).first.v);
  EXPECT_EQ(out.front(), '(');
  EXPECT_EQ(out.back(), ')');
}

TEST(DataJob, RunDataJobOutsideAStepRunsToCompletion) {
  value_t r = run_data_job(make_equal_job(nested_list(100), nested_list(100)));
  EXPECT_TRUE(std::get<bool>(r.v));
  r = run_data_job(make_print_job(nested_list(3)),
                   [](value_t s) { return value_t{std::get<std::string>(s.v) + "!"}; });
  EXPECT_EQ(std::get<std::string>(r.v), "(((1)))!");
}

// ---- the evaluator ------------------------------------------------------------

class DataStepsEvaluator : public ::testing::Test {
protected:
  environment_ptr env = builtins::make_default_environment();
  stackless_evaluator ev{env};

  // A state evaluating `script`, with data_step_nodes = `slice`, in an environment where `a` and
  // `b` are separately built n-deep lists.
  evaluator_state state(const std::string &script, std::size_t slice, std::size_t n = 1000) {
    auto run_env = environment::extend(env);
    run_env->set("a", nested_list(n));
    run_env->set("b", nested_list(n));
    evaluator_state st = ev.create_state(parse(script), run_env);
    st.data_step_nodes = slice;
    st.stats.start();
    return st;
  }
};

TEST_F(DataStepsEvaluator, NativeCallRunsASlicePerStep) {
  evaluator_state whole = state("(= a b)", 0);
  const int whole_steps = step_to_done(ev, whole);
  EXPECT_TRUE(std::get<bool>(whole.result.v));
  EXPECT_EQ(whole.stats.steps(step_kind::data), 0u);
  EXPECT_GE(whole.stats.data_nodes(), 1000u); // counted, though done in one step

  evaluator_state sliced = state("(= a b)", 64);
  const int sliced_steps = step_to_done(ev, sliced);
  EXPECT_TRUE(std::get<bool>(sliced.result.v));
  EXPECT_GE(sliced.stats.steps(step_kind::data), 1000u / 64 - 1);
  EXPECT_EQ(sliced_steps, whole_steps + static_cast<int>(sliced.stats.steps(step_kind::data)));
  EXPECT_EQ(sliced.stats.steps(step_kind::eval), whole.stats.steps(step_kind::eval));
}

TEST_F(DataStepsEvaluator, PrintingAndNotEqualSlice) {
  evaluator_state s = state("(str a)", 50);
  step_to_done(ev, s);
  EXPECT_EQ(std::get<std::string>(s.result.v), to_string(nested_list(1000)));
  EXPECT_GT(s.stats.steps(step_kind::data), 10u);

  evaluator_state ne = state("(!= a b)", 50);
  step_to_done(ev, ne);
  EXPECT_FALSE(std::get<bool>(ne.result.v));
  EXPECT_GT(ne.stats.steps(step_kind::data), 10u);

  evaluator_state cat = state("(str-concat \"a=\" a)", 50);
  step_to_done(ev, cat);
  EXPECT_EQ(std::get<std::string>(cat.result.v), "a=" + to_string(nested_list(1000)));
}

TEST_F(DataStepsEvaluator, SmallWorkTakesNoExtraSteps) {
  evaluator_state few = state("(= (list 1 2 3) (list 1 2 3))", 64);
  evaluator_state whole = state("(= (list 1 2 3) (list 1 2 3))", 0);
  EXPECT_EQ(step_to_done(ev, few), step_to_done(ev, whole));
  EXPECT_EQ(few.stats.steps(step_kind::data), 0u);
}

TEST_F(DataStepsEvaluator, MacroExpansionSlices) {
  const std::size_t n = 2000;
  value_t expr = make_list({sym("begin"),
                            make_list({sym("defmacro"), sym("m"), make_list({sym("x")}),
                                       make_list({sym("quote"), nested_list(n, sym("x"))})}),
                            make_list({sym("m"), value_t{int64_t{7}}})});
  evaluator_state st = ev.create_state(expr);
  st.data_step_nodes = 64;
  step_to_done(ev, st);
  EXPECT_TRUE(values_equal(st.result, nested_list(n, value_t{int64_t{7}})));
  EXPECT_GE(st.stats.steps(step_kind::data), n / 64 - 1);
}

TEST_F(DataStepsEvaluator, FreeingADeepValueSpreadsOverSteps) {
  const std::size_t n = 10000;
  auto run_env = environment::extend(env);
  run_env->set("big", nested_list(n)); // its only owner
  evaluator_state st = ev.create_state(parse("(begin (set big nil) 1)"), run_env);
  st.data_step_nodes = 100;
  st.stats.start();
  const int steps = step_to_done(ev, st);
  EXPECT_EQ(std::get<int64_t>(st.result.v), 1);
  EXPECT_GE(st.stats.steps(step_kind::data), n / 100 - 2);
  EXPECT_GE(st.stats.data_nodes(), n);
  // Evaluation waits while the backlog is freed, so the last step is an evaluation step.
  EXPECT_EQ(static_cast<uint64_t>(steps), st.stats.get_step_count());
}

TEST_F(DataStepsEvaluator, StepBudgetStopsDeepWorkMidway) {
  evaluator_state st = state("(= a b)", 16);
  ev.run(st, 8);
  EXPECT_FALSE(st.done);
  EXPECT_NE(st.data_work, nullptr); // waiting in the comparison
  ev.run(st);
  EXPECT_TRUE(st.done);
  EXPECT_TRUE(std::get<bool>(st.result.v));
}

TEST_F(DataStepsEvaluator, SnapshotMidWorkRunsTheCallAgain) {
  cvc::app ctx;
  evaluator_state st = state("(= a b)", 16, 300);
  ev.run(st, 8);
  ASSERT_NE(st.data_work, nullptr); // in the middle of the comparison

  auto &node = cvc::state::instance(ctx)("snapshot");
  encode_evaluator_state(node, st);
  evaluator_state back = decode_evaluator_state(node);
  EXPECT_EQ(back.data_step_nodes, 16u);
  EXPECT_EQ(back.data_work, nullptr); // work in flight is not kept...
  // (native functions are not serialized: put the builtins back, as the codec tests do)
  auto patch = [&](environment_ptr e) {
    for (; e && e->outer; e = e->outer) {
    }
    if (e)
      for (auto &[name, val] : env->bindings)
        e->set(name, val);
  };
  patch(back.global_env);
  for (auto &f : back.stack)
    patch(f.env);
  ev.run(back); // ...the waiting call runs again
  ASSERT_TRUE(back.done);
  EXPECT_TRUE(std::get<bool>(back.result.v));
  EXPECT_GT(back.stats.steps(step_kind::data), 0u);
}

// ---- jobs that call functions -----------------------------------------------------

// A job asks for a call and the evaluator makes it: a native function in the data step, a DSL
// function as frames above the call waiting on the job, so its body runs as the program's own
// evaluation steps.
TEST_F(DataStepsEvaluator, ALambdaAJobCallsRunsAsFramesAboveTheCall) {
  evaluator_state st = ev.create_state(
      parse("(+ 1 (length (collections.map (collections.range 40) (lambda (x) (* x 2)))))"));
  st.data_step_nodes = 4;
  st.stats.start();
  bool saw_call = false;
  while (!ev.step(st)) {
    // While the lambda runs its frames are on top; once it returns (stack back at base) the
    // next step hands its value to the job.
    if (st.data_work && st.data_work->calling && st.stack.size() > st.data_work->base) {
      saw_call = true;
      EXPECT_EQ(st.stack[st.data_work->base - 1].phase, eval_phase::apply); // the waiting call
    }
  }
  EXPECT_TRUE(saw_call);
  EXPECT_EQ(std::get<int64_t>(st.result.v), 41);
  EXPECT_GE(st.stats.steps(step_kind::data), 40u);
}

TEST_F(DataStepsEvaluator, SnapshotWhileAJobsLambdaRunsRunsTheCallAgain) {
  cvc::app ctx;
  evaluator_state st = ev.create_state(
      parse("(+ 1 (length (collections.map (collections.range 40) (lambda (x) (* x 2)))))"));
  st.data_step_nodes = 4;
  st.stats.start();
  while (!(st.data_work && st.data_work->calling && st.stack.size() > st.data_work->base &&
           st.stats.steps(step_kind::data) >= 10))
    ASSERT_FALSE(ev.step(st));
  const std::size_t base = st.data_work->base;
  const std::size_t args = st.data_work->frame_args;
  ASSERT_GT(st.stack.size(), base);

  auto &node = cvc::state::instance(ctx)("snapshot");
  encode_evaluator_state(node, st);
  evaluator_state back = decode_evaluator_state(node);
  EXPECT_EQ(back.data_work, nullptr);
  EXPECT_EQ(back.stack.size(), base);                // the lambda's frames are left out...
  EXPECT_EQ(back.stack.back().results.size(), args); // ...and the waiting call keeps its args
  // Native functions are not serialized: give each decoded scope chain the builtins and the
  // stdlib again.
  const auto patch = [&](environment_ptr e) {
    for (; e && e->outer && !e->outer->frozen; e = e->outer) {
    }
    if (!e || e->outer)
      return; // no scope, or patched already
    for (auto &[name, val] : env->bindings)
      e->set(name, val);
    e->outer = stdlib_registry::shared_scope();
  };
  patch(back.global_env);
  for (auto &f : back.stack) {
    patch(f.env);
    for (auto &v : f.results)
      if (auto *c = std::get_if<closure_ptr>(&v.v))
        patch((*c)->env_snapshot);
  }
  ev.run(back); // the map runs again from its first element
  ASSERT_TRUE(back.done);
  EXPECT_EQ(std::get<int64_t>(back.result.v), 41);
  ev.run(st);
  EXPECT_EQ(std::get<int64_t>(st.result.v), 41);
}

// A native function a job calls may leave deep-data work of its own (str of a value bigger than
// the slice): its result is that call's value, not the whole job's. Before jobs made their calls
// through the evaluator, (collections.map (list a b) str) came back as one string.
TEST_F(DataStepsEvaluator, ANativeFunctionsOwnWorkIsItsCallsValue) {
  const std::string expected =
      "(\"" + to_string(nested_list(1000)) + "\" \"" + to_string(nested_list(1000)) + "\")";
  for (const char *script :
       {"(collections.map (list a b) str)", "(collections.filter (list a b) (lambda (x) (= x b)))",
        "(collections.reduce (list a b) (lambda (acc x) (cons (str x) acc)) "
        "(list))"}) {
    SCOPED_TRACE(script);
    evaluator_state whole = state(script, 0);
    step_to_done(ev, whole);
    evaluator_state sliced = state(script, 16);
    step_to_done(ev, sliced);
    EXPECT_EQ(to_string(sliced.result), to_string(whole.result));
    EXPECT_GT(sliced.stats.steps(step_kind::data), 10u);
  }
  evaluator_state mapped = state("(collections.map (list a b) str)", 16);
  step_to_done(ev, mapped);
  ASSERT_TRUE(std::holds_alternative<list_ptr>(mapped.result.v));
  EXPECT_EQ(std::get<list_ptr>(mapped.result.v)->size(), 2u);
  EXPECT_EQ(to_string(mapped.result), expected);
}

TEST_F(DataStepsEvaluator, ALambdaAJobCallsIsGatedLikeTheProgram) {
  // The special-form gate of a restricted evaluator (Ariadne's predicates) covers the body of a
  // function a job calls, since that body runs on the same evaluator.
  stackless_evaluator gated(env);
  gated.restrict_special_forms(std::make_shared<const std::set<std::string>>(
      std::set<std::string>{"lambda", "if", "quote"}));
  for (std::size_t slice : {0u, 16u}) {
    evaluator_state st =
        gated.create_state(parse("(collections.map (list 1) (lambda (x) (defmacro m () 1)))"));
    st.data_step_nodes = slice;
    std::string err;
    try {
      gated.run(st);
    } catch (const std::runtime_error &e) {
      err = e.what();
    }
    EXPECT_NE(err.find("special form 'defmacro' is not permitted"), std::string::npos) << err;
  }
}

// ---- the schedulers -------------------------------------------------------------

class DataStepsScheduler : public ::testing::Test {
protected:
  cvc::app app_ctx;
  cvc::state &root = cvc::state::instance(app_ctx);
};

TEST_F(DataStepsScheduler, SettingsComeFromTheTree) {
  scheduler plain;
  plain.set_watch_root(&root);
  plain.set_id("plain");
  plain.load_settings();
  EXPECT_EQ(plain.data_settings().step_nodes, 0u);
  EXPECT_EQ(root("state_exec.schedulers.plain.data.step_nodes").value(), "0"); // published

  root("state_exec.defaults.data.step_nodes").value("32");
  scheduler sched;
  sched.set_watch_root(&root);
  sched.set_id("ds");
  sched.load_settings();
  EXPECT_EQ(sched.data_settings().step_nodes, 32u);
  EXPECT_EQ(root("state_exec.schedulers.ds.data.step_nodes").value(), "32");
  root("state_exec.schedulers.ds.data.step_nodes").value("8");
  sched.load_settings();
  EXPECT_EQ(sched.data_settings().step_nodes, 8u);
  root("state_exec.schedulers.ds.data.step_nodes").value("lots"); // skipped
  sched.load_settings();
  EXPECT_EQ(sched.data_settings().step_nodes, 32u);

  async_scheduler async;
  EXPECT_EQ(async.data_settings().step_nodes, 0u);
  async.set_data_defaults(async_scheduler::host_data_defaults());
  EXPECT_EQ(async.data_settings().step_nodes, async_scheduler::host_data_defaults().step_nodes);
  async.set_watch_root(&root); // state_exec.defaults.data.step_nodes = 32 overrides it
  EXPECT_EQ(async.data_settings().step_nodes, 32u);
}

TEST_F(DataStepsScheduler, DeepWorkCountsAgainstMaxSteps) {
  root("state_exec.defaults.data.step_nodes").value("16");
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();

  const int done = sched.execute(compare_script(500));
  sched.run();
  auto info = sched.get_process_info(done);
  ASSERT_TRUE(info);
  EXPECT_EQ(info->status, process_status::terminated);
  EXPECT_TRUE(std::get<bool>(sched.get_result(done)->v));
  EXPECT_GE(info->data_steps, 500u / 16 - 1);
  EXPECT_GT(info->eval_steps, 0u);
  EXPECT_EQ(info->read_steps, 0u);
  EXPECT_GE(info->data_nodes, 500u);
  EXPECT_EQ(info->eval_steps + info->data_steps + info->read_steps, info->step_count);
  EXPECT_GT(info->eval_time, 0.0);
  EXPECT_GT(info->data_time, 0.0);

  // The same program with room for its evaluation but not its comparison.
  execute_options opts;
  opts.max_steps = info->eval_steps + 2;
  const int killed = sched.execute(compare_script(500), opts);
  sched.run();
  EXPECT_EQ(sched.get_process_info(killed)->status, process_status::killed);
}

TEST_F(DataStepsScheduler, OtherProcessesRunBetweenSlices) {
  root("state_exec.defaults.data.step_nodes").value("4");
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  const int slow = sched.execute(compare_script(400));
  // Let it reach its comparison.
  while (sched.get_process_info(slow)->data_steps == 0)
    sched.step();
  const int quick = sched.execute(std::string("(begin 1 2 3)"));
  while (sched.get_process_info(quick)->status != process_status::terminated)
    sched.step();
  EXPECT_NE(sched.get_process_info(slow)->status, process_status::terminated);
  sched.run();
  EXPECT_TRUE(std::get<bool>(sched.get_result(slow)->v));
}

TEST_F(DataStepsScheduler, NoForkWhileAJobsFunctionRuns) {
  // The job is not copied, so a child could not finish the call: fork refuses until it is done.
  scheduler sched;
  const int pid =
      sched.execute(std::string("(collections.map (collections.range 50) (lambda (x) (* x 2)))"));
  while (sched.get_process_info(pid)->data_steps < 5)
    sched.step();
  EXPECT_EQ(sched.fork(pid), -1);
  sched.run();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::terminated);
  EXPECT_EQ(sched.list_processes().size(), 1u);
}

TEST_F(DataStepsScheduler, SignalsWaitForDeepWork) {
  root("state_exec.defaults.data.step_nodes").value("4");
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  execute_options opts;
  opts.signal_handlers["SIGUSR1"] = make_list({sym("begin"), value_t{int64_t{42}}});
  const int pid = sched.execute(compare_script(200), opts);
  while (sched.get_process_info(pid)->data_steps == 0)
    sched.step();
  ASSERT_TRUE(sched.send_signal(pid, "SIGUSR1"));
  const uint64_t before = sched.get_process_info(pid)->data_steps;
  sched.step();
  EXPECT_EQ(sched.get_process_info(pid)->data_steps, before + 1); // the work went on
  sched.run();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::terminated);
  EXPECT_TRUE(std::get<bool>(sched.get_result(pid)->v));
}

TEST_F(DataStepsScheduler, BudgetUsageAndInspectReportTheKinds) {
  async_scheduler sched;
  sched.set_watch_root(&root);
  root("state_exec.defaults.data.step_nodes").value("16");
  memory_tracker tracker;
  intrinsics_context ictx;
  ictx.sched = &sched;
  ictx.root = &root;
  ictx.tracker = &tracker;
  auto env = builtins::make_default_environment();
  register_intrinsics(env, &ictx);
  execute_options opts;
  opts.env = env;
  const int pid = sched.execute(
      std::string("(begin (set a 1) (set b 1) (set i 0)"
                  " (while (< i 300) (begin (set a (list a)) (set b (list b)) (set i (+ i 1))))"
                  " (= a b) (set used (budget-usage)) (set me (inspect (self-pid)))"
                  " (list used me))"),
      opts);
  sched.sync_run();
  auto result = sched.get_result(pid);
  ASSERT_TRUE(result);
  auto &pair = *std::get<list_ptr>(result->v);
  for (const value_t &d : pair) {
    std::unordered_map<std::string, value_t> fields;
    for (auto &[k, v] : *std::get<dict_ptr>(d.v))
      fields[k] = v;
    for (const char *key : {"read_steps", "eval_steps", "data_steps", "data_nodes", "read_time",
                            "eval_time", "data_time", "steps"})
      EXPECT_TRUE(fields.count(key)) << key;
    EXPECT_GE(std::get<int64_t>(fields["data_steps"].v), 300 / 16 - 1);
    EXPECT_GE(std::get<int64_t>(fields["data_nodes"].v), 300);
    EXPECT_GT(std::get<int64_t>(fields["eval_steps"].v), 0);
  }
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_GT(info->data_time, 0.0);
}

TEST_F(DataStepsScheduler, StateSetOfADeepValueSlices) {
  async_scheduler sched;
  sched.set_watch_root(&root);
  root("state_exec.defaults.data.step_nodes").value("16");
  memory_tracker tracker;
  intrinsics_context ictx;
  ictx.sched = &sched;
  ictx.root = &root;
  ictx.tracker = &tracker;
  auto env = builtins::make_default_environment();
  register_intrinsics(env, &ictx);
  execute_options opts;
  opts.env = env;
  const int pid =
      sched.execute(std::string("(begin (set a 1) (set i 0)"
                                " (while (< i 300) (begin (set a (list a)) (set i (+ i 1))))"
                                " (state-set \"deep.text\" a) (state-set \"deep.n\" 7)"
                                " (state-get \"deep.text\"))"),
                    opts);
  // The list prints a slice per step, and the node is written once the whole string is ready.
  while (sched.get_process_info(pid)->data_steps == 0)
    sched.sync_step();
  EXPECT_EQ(root.findDescendant("deep.text"), nullptr);
  sched.sync_run();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  EXPECT_EQ(info->status, process_status::terminated);
  EXPECT_EQ(root("deep.text").value(), to_string(nested_list(300)));
  EXPECT_EQ(root("deep.n").value(), "7");
  EXPECT_EQ(std::get<std::string>(sched.get_result(pid)->v), root("deep.text").value());
  EXPECT_GE(info->data_steps, 300u / 16 - 1);
  EXPECT_GE(info->data_nodes, 300u);
}

// ---- state-json / state-from-json ------------------------------------------------------
//
// cvc::state::json() and json(j) as data jobs: the same writer and reader (state_json.h), a
// slice of data.step_nodes units per step.

class DataStepsStateJson : public ::testing::Test {
protected:
  cvc::app app_ctx;
  cvc::state &root = cvc::state::instance(app_ctx);
  async_scheduler sched;
  memory_tracker tracker;
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();

  void SetUp() override {
    root("state_exec.defaults.data.step_nodes").value("64");
    sched.set_watch_root(&root);
    ictx.sched = &sched;
    ictx.root = &root;
    ictx.tracker = &tracker;
    register_intrinsics(env, &ictx);
    // deep: a 1000-level chain with a value on every node; wide: 20000 children.
    cvc::state *node = &root("deep");
    node->value("0");
    for (int i = 1; i <= 1000; ++i) {
      node = &(*node)("n");
      node->value(std::to_string(i));
    }
    cvc::state &wide = root("wide");
    for (int i = 0; i < 20000; ++i)
      wide("k" + std::to_string(i)).value("v" + std::to_string(i));
  }

  int submit(const std::string &script, execute_options opts = {}) {
    opts.env = environment::extend(env);
    return sched.execute(script, opts);
  }

  std::string error(int pid) const { return sched.get_exit_error(pid).value_or(""); }
};

TEST_F(DataStepsStateJson, StateJsonWritesWhatJsonWritesASliceAStep) {
  for (const char *path : {"deep", "wide"}) {
    SCOPED_TRACE(path);
    const int pid = submit(std::string("(state-json \"") + path + "\")");
    sched.sync_run();
    const auto info = sched.get_process_info(pid);
    ASSERT_TRUE(info);
    ASSERT_EQ(info->status, process_status::terminated) << error(pid);
    const std::string doc = root(path).json();
    EXPECT_EQ(std::get<std::string>(sched.get_result(pid)->v), doc);
    // A unit per node and per 256 bytes of text, 64 units a step (a slice overruns by at most its
    // last node's bytes: deep's longest key is ~2 KB, 8 units).
    const std::size_t units = root(path).children().size() + 1 + doc.size() / kDataNodeBytes;
    EXPECT_GE(info->data_nodes, units);
    EXPECT_GE(info->data_steps, units / (64 + 8) - 1);
    EXPECT_GT(info->data_steps, 50u);
  }
  // A missing node is nil, like state-get.
  const int missing = submit("(state-json \"no.such.node\")");
  sched.sync_run();
  EXPECT_TRUE(sched.get_result(missing)->is_nil());
}

TEST_F(DataStepsStateJson, StateFromJsonLoadsWhatJsonLoadsASliceAStep) {
  const int pid = submit("(state-from-json \"copy\" (state-json \"wide\"))");
  sched.sync_run();
  const auto info = sched.get_process_info(pid);
  ASSERT_EQ(info->status, process_status::terminated) << error(pid);
  EXPECT_EQ(root("copy").json(), root("wide").json());
  EXPECT_GT(info->data_steps, 200u);

  // Keys are relative to the node, so a document goes back into the node it came from.
  const std::string doc = root("deep").json();
  root("saved").value(doc);
  root("deep").reset(false); // its value and the whole chain
  const int restore = submit("(state-from-json \"deep\" (state-get \"saved\"))");
  sched.sync_run();
  ASSERT_EQ(sched.get_process_info(restore)->status, process_status::terminated) << error(restore);
  EXPECT_EQ(root("deep").json(), doc);
}

TEST_F(DataStepsStateJson, OtherProcessesRunBetweenSlices) {
  root("state_exec.defaults.data.step_nodes").value("4");
  const int slow = submit("(state-json \"wide\")");
  while (sched.get_process_info(slow)->data_steps == 0)
    sched.sync_step();
  const int quick = submit("(begin 1 2 3)");
  while (sched.get_process_info(quick)->status != process_status::terminated)
    sched.sync_step();
  EXPECT_NE(sched.get_process_info(slow)->status, process_status::terminated);
  sched.sync_run();
  EXPECT_EQ(std::get<std::string>(sched.get_result(slow)->v), root("wide").json());
}

TEST_F(DataStepsStateJson, TheWorkCountsAgainstMaxSteps) {
  execute_options opts;
  opts.max_steps = 20;
  const int pid = submit("(state-json \"wide\")", opts);
  sched.sync_run();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
}

TEST_F(DataStepsStateJson, AMalformedDocumentSetsNothingThoughItsCheckSpansSteps) {
  // Thousands of good entries, then an error: the first pass finds it before any is set.
  std::string doc = "{";
  for (int i = 0; i < 5000; ++i)
    doc += "\"k" + std::to_string(i) + "\": \"v\", ";
  doc += "\"bad\": }";
  root("doc").value(doc);
  const int pid = submit("(state-from-json \"target\" (state-get \"doc\"))");
  sched.sync_run();
  const auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
  EXPECT_NE(error(pid).find("state-from-json: line 1: expected value"), std::string::npos)
      << error(pid);
  EXPECT_GT(info->data_steps, 10u);
  EXPECT_EQ(root.findDescendant("target.k0"), nullptr);
}

TEST_F(DataStepsStateJson, TheDocumentIsHeldToTheAllocationLimit) {
  root("state_exec.defaults.limits.max_alloc_bytes").value("4096");
  const int pid = submit("(state-json \"wide\")");
  sched.sync_run();
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::killed);
  EXPECT_NE(error(pid).find("state-json"), std::string::npos) << error(pid);
}

TEST_F(DataStepsStateJson, NodesRemovedBetweenSlicesAreStillWritten) {
  // The walk holds the children it has yet to visit, so dropping them from the tree mid-walk
  // neither crashes it nor loses them: they are written as they were.
  root("state_exec.defaults.data.step_nodes").value("16");
  const std::string before = root("wide").json();
  const int pid = submit("(state-json \"wide\")");
  while (sched.get_process_info(pid)->data_steps < 3)
    sched.sync_step();
  root("wide").reset(false); // destroys the children the tree owned
  sched.sync_run();
  ASSERT_EQ(sched.get_process_info(pid)->status, process_status::terminated) << error(pid);
  EXPECT_EQ(std::get<std::string>(sched.get_result(pid)->v), before);
  EXPECT_EQ(root("wide").numChildren(), 0u);
}

// ---- methods and generators inside the process ----------------------------------
//
// A class method is a closure run as frames of the process that sends to it, and a DSL
// generator's body is advanced as a job (make_generator_job): both are the process's own steps,
// counted against its budget, where each used to run to completion on an evaluator of its own
// inside one step.

TEST_F(DataStepsScheduler, MethodStepsCountAgainstMaxSteps) {
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  const std::string script = "(begin (defclass C (spin (self n) (begin (set i 0)"
                             " (while (< i n) (set i (+ i 1))) i))) (set o (C))"
                             " (send o \"spin\" 2000))";
  const int done = sched.execute(script);
  sched.run();
  auto info = sched.get_process_info(done);
  ASSERT_TRUE(info);
  EXPECT_EQ(info->status, process_status::terminated);
  EXPECT_EQ(std::get<int64_t>(sched.get_result(done)->v), 2000);
  EXPECT_GT(info->eval_steps, 2000u); // the method's loop, step by step

  execute_options opts;
  opts.max_steps = 500;
  const int killed = sched.execute(script, opts);
  sched.run();
  EXPECT_EQ(sched.get_process_info(killed)->status, process_status::killed);
}

TEST_F(DataStepsScheduler, GeneratorBodiesRunASlicePerStep) {
  root("state_exec.defaults.data.step_nodes").value("16");
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  const std::string script = "(begin (set g (generator (lambda () (begin (set k 0)"
                             " (while (< k 500) (set k (+ k 1))) (yield k) (yield 7)))))"
                             " (list (next g) (next g) (next g) (length (collect (range 0 100)))))";
  const int done = sched.execute(script);
  sched.run();
  auto info = sched.get_process_info(done);
  ASSERT_TRUE(info);
  ASSERT_EQ(info->status, process_status::terminated);
  const auto &r = *std::get<list_ptr>(sched.get_result(done)->v);
  ASSERT_EQ(r.size(), 4u);
  EXPECT_EQ(std::get<int64_t>(r[0].v), 500);
  EXPECT_EQ(std::get<int64_t>(r[1].v), 7);
  EXPECT_TRUE(r[2].is_nil()); // exhausted
  EXPECT_EQ(std::get<int64_t>(r[3].v), 100);
  EXPECT_GT(info->data_nodes, 2000u); // the body's steps, and range's values
  EXPECT_GT(info->data_steps, 100u);

  execute_options opts;
  opts.max_steps = info->eval_steps + 20; // room for the program, not the generator's body
  const int killed = sched.execute(script, opts);
  sched.run();
  EXPECT_EQ(sched.get_process_info(killed)->status, process_status::killed);
}

TEST_F(DataStepsScheduler, ForOverAGeneratorGivesTheSameSlicedOrNot) {
  const std::string script = "(begin (set g (generator (lambda () (begin (set j 0)"
                             " (while (< j 50) (begin (yield j) (set j (+ j 1))))))))"
                             " (set s 0) (for x g (set s (+ s x))) s)";
  for (const char *slice : {"0", "4"}) {
    root("state_exec.defaults.data.step_nodes").value(slice);
    scheduler sched;
    sched.set_watch_root(&root);
    sched.load_settings();
    const int pid = sched.execute(script);
    sched.run();
    ASSERT_EQ(sched.get_process_info(pid)->status, process_status::terminated) << slice;
    EXPECT_EQ(std::get<int64_t>(sched.get_result(pid)->v), 49 * 50 / 2) << slice;
  }
}

TEST_F(DataStepsScheduler, ConstructorReturnsTheInstanceAfterInitDefers) {
  // K inherits P's init, a closure that compares two deep lists: the comparison runs as data
  // steps inside init's frames, and the constructor's value is still the instance, not init's.
  root("state_exec.defaults.data.step_nodes").value("16");
  scheduler sched;
  sched.set_watch_root(&root);
  sched.load_settings();
  const int pid = sched.execute(
      std::string("(begin (set a 1) (set b 1) (set i 0)"
                  " (while (< i 300) (begin (set a (list a)) (set b (list b)) (set i (+ i 1))))"
                  " (defclass P (init (self x) (begin (set-attr self \"same\" (= a b)) 42)))"
                  " (defclass K P) (set o (K 1)) (get-attr o \"same\"))"));
  sched.run();
  auto info = sched.get_process_info(pid);
  ASSERT_TRUE(info);
  ASSERT_EQ(info->status, process_status::terminated);
  EXPECT_TRUE(std::get<bool>(sched.get_result(pid)->v));
  EXPECT_GT(info->data_steps, 0u);
}
