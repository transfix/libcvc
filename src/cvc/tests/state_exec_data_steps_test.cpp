// Deep-data work on the executor ("Deep data on the executor" in docs/STATE_EXEC_GRAMMAR.md):
// walks that run a slice at a time (data_job), the stackless evaluator slicing them
// (evaluator_state::data_step_nodes), the schedulers taking data.step_nodes from the state tree,
// and the budget split by kind (read / eval / data) that process_info, inspect and budget-usage
// report.

#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/generator.h>
#include <cvc/state/state_exec/heap.h>
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

// A program's closure bound in the scope it captured (a defun, a lambda set at top level) and that
// scope hold each other, and only the cycle collector frees them
// (docs/STATE_EXEC_DEVELOPER_GUIDE.md, "Reference cycles"). Nothing here runs long enough for a
// collection to be due, so once every test is done one collection leaves LeakSanitizer only real
// leaks to report.
class collect_cycles_at_exit : public ::testing::Environment {
public:
  void TearDown() override { collect_cycles(); }
};
[[maybe_unused]] ::testing::Environment *const g_collect_cycles =
    ::testing::AddGlobalTestEnvironment(new collect_cycles_at_exit);

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

// ---- the isolating copy (state-data-set's, td 4.3) -----------------------------------

namespace {

// One value of each kind the isolating copy refuses (S0.2, S0-Q4), with the kind its message
// names.
std::vector<std::pair<std::string, value_t>> refused_kinds() {
  const auto call = std::make_shared<async_call>("f", 1);
  auto wait = std::make_shared<data_object>();
  wait->type_name = detail::kLockWaitType;
  const value_t box = make_dict({{"__name__", value_t{std::string("Box")}}});
  // A user class derived from a built-in one is still a user class.
  const value_t derived = make_dict({{"__name__", value_t{std::string("V")}},
                                     {"__super__", value_t{exception_class("ValueError")}}});
  return {
      {"closure", value_t{std::make_shared<closure>()}},
      {"native function",
       value_t{native_fn{[](std::span<const value_t>) -> value_t { return nil_value; }}}},
      {"generator", value_t{std::make_shared<generator>(std::make_shared<environment>())}},
      {"future", make_async_future(call)},
      {"future", detail::make_awaitable_handle(call)},
      {"lock form", detail::make_lock_ops(nullptr)},
      {"lock wait", value_t{wait}},
      {"object instance", make_dict({{"__class__", box}, {"n", value_t{int64_t{1}}}})},
      {"object instance", make_dict({{"__class__", derived}, {"message", value_t{"m"}}})},
  };
}

// state-data-set's refusal of `kind`, at the root.
std::string refusal_text(const std::string &kind) {
  return "state-data-set: cannot store " + std::string(kind[0] == 'o' ? "an " : "a ") + kind;
}

// The isolating copy of `v`, made in slices of `slice` nodes.
value_t isolate(const value_t &v, std::size_t slice = 0) {
  auto job = make_copy_job(v, copy_mode::isolate, "state-data-set");
  return run_sliced(*job, slice).first;
}

// What the isolating copy of `v` throws, or "" when it copies it.
std::string isolate_refusal(const value_t &v, std::size_t slice = 0) {
  try {
    isolate(v, slice);
  } catch (const std::runtime_error &e) {
    return e.what();
  }
  return "";
}

} // namespace

TEST(DataJob, IsolatingCopyRefusesAtAnyDepth) {
  // Each kind at the root, in a list and in a dict, and deep: the message names the kind and the
  // container it was found in.
  for (const auto &[kind, v] : refused_kinds()) {
    SCOPED_TRACE(kind);
    const std::string refused = refusal_text(kind);
    for (std::size_t slice : {0u, 1u}) {
      EXPECT_EQ(isolate_refusal(v, slice), refused);
      EXPECT_EQ(isolate_refusal(make_list({value_t{int64_t{1}}, v}), slice),
                refused + " (nested in a list)");
      EXPECT_EQ(isolate_refusal(make_dict({{"a", value_t{int64_t{1}}}, {"k", v}}), slice),
                refused + " (nested in a dict)");
      EXPECT_EQ(isolate_refusal(nested_list(40, v), slice), refused + " (nested in a list)");
      EXPECT_EQ(isolate_refusal(nested_list(40, make_dict({{"k", v}})), slice),
                refused + " (nested in a dict)");
    }
  }
}

TEST(DataJob, IsolatingCopyRefusesInALaterSlice) {
  // The slice that reaches the value refuses it; the slices before it built only the job's own
  // copy, and a refused job gives no result, so state-data-set's write (its job's `then`) never
  // runs.
  for (const auto &[kind, v] : refused_kinds()) {
    SCOPED_TRACE(kind);
    std::vector<value_t> elems(100, value_t{int64_t{7}});
    elems.push_back(make_dict({{"k", v}}));
    auto job = make_copy_job(make_list(std::move(elems)), copy_mode::isolate, "state-data-set");
    for (int i = 0; i < 10; ++i)
      ASSERT_FALSE(job->step(5));
    EXPECT_EQ(job->nodes(), 50u);
    try {
      run_sliced(*job, 5);
      ADD_FAILURE() << "not refused";
    } catch (const std::runtime_error &e) {
      EXPECT_EQ(std::string(e.what()), refusal_text(kind) + " (nested in a dict)");
    }
    EXPECT_EQ(job->nodes(), 102u); // the 101 elements and the dict's "k"

    bool written = false;
    EXPECT_THROW(run_data_job(make_copy_job(make_list({value_t{int64_t{1}}, v}), copy_mode::isolate,
                                            "state-data-set"),
                              [&](value_t) {
                                written = true;
                                return nil_value;
                              }),
                 std::runtime_error);
    EXPECT_FALSE(written);
  }
}

TEST(DataJob, IsolatingCopySharesDataObjectsAndBuiltinClasses) {
  // An ordinary data_object is shared under its read-only contract, at the root and nested.
  auto thing = std::make_shared<data_object>();
  thing->type_name = "thing";
  thing->payload = 42;
  EXPECT_EQ(std::get<data_object_ptr>(isolate(value_t{thing}).v), thing);
  const value_t holder = make_list({value_t{thing}, make_dict({{"t", value_t{thing}}})});
  const value_t held = isolate(holder, 1);
  const auto &hl = *std::get<list_ptr>(held.v);
  EXPECT_NE(std::get<list_ptr>(held.v), std::get<list_ptr>(holder.v));
  EXPECT_EQ(std::get<data_object_ptr>(hl[0].v), thing);
  EXPECT_EQ(std::get<data_object_ptr>(field(hl[1], "t")->v), thing);

  // Every built-in class dict is shared as it is: they are read-only (S0.1).
  ASSERT_EQ(exception_class_names().size(), 19u);
  for (const std::string &name : exception_class_names()) {
    const dict_ptr cls = exception_class(name);
    EXPECT_EQ(std::get<dict_ptr>(isolate(value_t{cls}).v), cls) << name;
  }

  // A built-in exception's instance is copied with its class shared: still a ValueError, its
  // own fields copied.
  const value_t exc =
      make_exception(exception_class("ValueError"), "bad", make_list({value_t{int64_t{1}}}), "dsl");
  const value_t copied = isolate(make_list({exc, exc}), 1);
  const auto &cl = *std::get<list_ptr>(copied.v);
  EXPECT_EQ(std::get<dict_ptr>(cl[0].v), std::get<dict_ptr>(cl[1].v)); // reached twice, one copy
  EXPECT_NE(std::get<dict_ptr>(cl[0].v), std::get<dict_ptr>(exc.v));
  EXPECT_EQ(class_of_instance(cl[0]), exception_class("ValueError"));
  EXPECT_NE(std::get<list_ptr>(field(cl[0], "data")->v), std::get<list_ptr>(field(exc, "data")->v));
  EXPECT_TRUE(is_instance(cl[0], exception_class("ValueError")));
  EXPECT_TRUE(values_equal(cl[0], exc));

  // Plain data that only looks like an instance or a future is copied: a `__class__` that is no
  // dict, a channel's future (a one-key dict naming the channel).
  for (const value_t &plain :
       {make_dict({{"__class__", value_t{std::string("Box")}}}), make_future("reply.1")}) {
    const value_t c = isolate(plain);
    EXPECT_NE(std::get<dict_ptr>(c.v), std::get<dict_ptr>(plain.v));
    EXPECT_TRUE(values_equal(c, plain)) << to_string(plain);
  }
}

TEST(DataJob, IsolatingCopyKeepsSharingAndCycles) {
  const value_t inner = make_list({value_t{int64_t{3}}});
  const value_t shared = make_dict({{"x", value_t{int64_t{1}}}});
  value_t c = isolate(make_list({inner, inner, shared, shared}));
  const auto &l = *std::get<list_ptr>(c.v);
  EXPECT_EQ(std::get<list_ptr>(l[0].v), std::get<list_ptr>(l[1].v));
  EXPECT_NE(std::get<list_ptr>(l[0].v), std::get<list_ptr>(inner.v));
  EXPECT_EQ(std::get<dict_ptr>(l[2].v), std::get<dict_ptr>(l[3].v));
  EXPECT_NE(std::get<dict_ptr>(l[2].v), std::get<dict_ptr>(shared.v));

  // A cycle through a list and a dict is copied as a cycle, each node once.
  auto cyclic = std::make_shared<std::vector<value_t>>();
  auto back = std::make_shared<ordered_map>();
  back->emplace_back("back", value_t{cyclic});
  cyclic->push_back(value_t{back});
  auto job = make_copy_job(value_t{cyclic}, copy_mode::isolate, "state-data-set");
  const value_t cy = run_sliced(*job, 1).first;
  EXPECT_EQ(job->nodes(), 2u);
  const list_ptr copied = std::get<list_ptr>(cy.v);
  const dict_ptr copied_back = std::get<dict_ptr>((*copied)[0].v);
  EXPECT_NE(copied, cyclic);
  EXPECT_NE(copied_back, back);
  EXPECT_EQ(std::get<list_ptr>((*copied_back)[0].second.v), copied);
  copied->clear(); // break both cycles
  cyclic->clear();
}

TEST(DataJob, IsolatingCopyKeepsWhatItMemoizedAlive) {
  // The memo is keyed by each source list's address. A source another process drops between two
  // slices stays allocated until the copy is done, or its address could come back as a new list
  // that the copy would take for it (R0a-mem-2). Share mode too: async arguments use it.
  for (copy_mode mode : {copy_mode::isolate, copy_mode::share}) {
    SCOPED_TRACE(mode == copy_mode::isolate ? "isolate" : "share");
    auto big = std::make_shared<std::vector<value_t>>();
    for (int i = 0; i < 100; ++i)
      big->push_back(make_list({value_t{int64_t{i}}}));
    auto job = make_copy_job(value_t{big}, mode, "state-data-set");
    for (int i = 0; i < 20; ++i)
      ASSERT_FALSE(job->step(1));
    const std::weak_ptr<std::vector<value_t>> first = std::get<list_ptr>((*big)[0].v);
    (*big)[0] = nil_value; // copied, and in the memo
    EXPECT_FALSE(first.expired()) << "a memoized source was freed mid-copy";
    const value_t c = run_sliced(*job, 1).first;
    EXPECT_EQ(to_string((*std::get<list_ptr>(c.v))[0]), "(0)");
    job.reset();
    EXPECT_TRUE(first.expired()); // and freed with the job
  }
}

TEST(DataJob, IsolatingCopyRefusesOnlyAnInstancesShape) {
  // An object instance has the shape defclass and make_exception give it: "__class__" first, and
  // its class's "__name__" first, a class of any size. Any other dict holding a "__class__" key is
  // data, and is copied (R0-mid semantics 4).
  const value_t box = make_dict({{"__name__", value_t{std::string("Box")}}});
  // JSON's {"__class__": {"a": 1}}; "__class__" not first; an empty "class".
  for (const value_t &plain : {make_dict({{"__class__", make_dict({{"a", value_t{1}}})}}),
                               make_dict({{"a", value_t{1}}, {"__class__", box}}),
                               make_dict({{"__class__", make_dict()}})}) {
    SCOPED_TRACE(to_string(plain));
    const value_t c = isolate(plain);
    EXPECT_NE(std::get<dict_ptr>(c.v), std::get<dict_ptr>(plain.v));
    EXPECT_TRUE(values_equal(c, plain));
  }
  // A user class with many methods is still a class.
  const value_t big = make_dict({{"__name__", value_t{std::string("Big")}},
                                 {"a", value_t{1}},
                                 {"b", value_t{2}},
                                 {"c", value_t{3}},
                                 {"d", value_t{4}}});
  EXPECT_EQ(isolate_refusal(make_dict({{"__class__", big}})), refusal_text("object instance"));
  EXPECT_EQ(isolate_refusal(make_list({make_dict({{"__class__", box}, {"x", value_t{1}}})})),
            refusal_text("object instance") + " (nested in a list)");
}

TEST(DataJob, ShareModeStillSharesClosures) {
  // Share mode (state-data-get's copy, async arguments) is as before, but for the built-in class
  // dicts: it refuses nothing, shares everything that is not a list or dict, copies every other
  // dict, and shares a built-in class's (read-only since S0.1), so a read-back exception can be
  // stored again.
  const auto clo = std::make_shared<closure>();
  const auto gen = std::make_shared<generator>(std::make_shared<environment>());
  const auto call = std::make_shared<async_call>("f", 1);
  const value_t future = make_async_future(call);
  const value_t lock = detail::make_lock_ops(nullptr);
  const value_t box = make_dict(
      {{"__class__", make_dict({{"__name__", value_t{std::string("Box")}}})}, {"n", value_t{1}}});
  const value_t exc = make_exception(exception_class("KeyError"), "k", nil_value, "dsl");
  const value_t source =
      make_list({value_t{clo}, value_t{gen},
                 value_t{native_fn{[](std::span<const value_t>) -> value_t { return nil_value; }}},
                 future, lock, box, exc});
  for (bool explicit_mode : {false, true}) {
    SCOPED_TRACE(explicit_mode ? "copy_mode::share" : "the defaults");
    auto job = explicit_mode ? make_copy_job(source, copy_mode::share, "state-data-get")
                             : make_copy_job(source);
    const value_t c = run_sliced(*job, 1).first;
    const auto &l = *std::get<list_ptr>(c.v);
    ASSERT_EQ(l.size(), 7u);
    EXPECT_EQ(std::get<closure_ptr>(l[0].v), clo);
    EXPECT_EQ(std::get<generator_ptr>(l[1].v), gen);
    EXPECT_TRUE(std::holds_alternative<native_fn>(l[2].v));
    EXPECT_NE(std::get<dict_ptr>(l[3].v), std::get<dict_ptr>(future.v)); // the dict is copied,
    EXPECT_TRUE(is_async_future(l[3]));                                  // its handle shared
    EXPECT_EQ(std::get<data_object_ptr>(field(l[3], "__async_call__")->v),
              std::get<data_object_ptr>(field(future, "__async_call__")->v));
    EXPECT_EQ(std::get<data_object_ptr>(l[4].v), std::get<data_object_ptr>(lock.v));
    EXPECT_NE(std::get<dict_ptr>(l[5].v), std::get<dict_ptr>(box.v)); // an instance and its class
    EXPECT_NE(class_of_instance(l[5]), class_of_instance(box));       // are copied
    EXPECT_EQ(class_of_instance(l[6]), exception_class("KeyError"));  // a built-in one is shared
    EXPECT_TRUE(is_instance(l[6], exception_class("KeyError")));
    EXPECT_EQ(isolate_refusal(c), "state-data-set: cannot store a closure (nested in a list)");
    EXPECT_EQ(isolate_refusal(l[6]), ""); // the read-back exception is stored again
  }
}

TEST(DataJob, CopyNamesItsCallerAtTheDeadline) {
  // The copy's deadline names its caller: state-data-get by default, as before.
  eval_deadline_guard spent(0.0);
  const value_t v = make_list({value_t{int64_t{1}}});
  const auto timeout_of = [](data_job &job) -> std::string {
    try {
      job.step(0);
    } catch (const evaluation_timeout &e) {
      return e.what();
    }
    return "";
  };
  EXPECT_EQ(timeout_of(*make_copy_job(v)), "state-data-get: evaluation exceeded timeout");
  EXPECT_EQ(timeout_of(*make_copy_job(v, copy_mode::isolate, "state-data-set")),
            "state-data-set: evaluation exceeded timeout");
}

// ---- values restored from a snapshot ---------------------------------------------------

namespace {

// An async scheduler with the state intrinsics on the app tree, as a host has one.
struct intrinsics_world {
  async_scheduler sched;
  process_ptr proc = make_process();
  intrinsics_context ictx;
  environment_ptr env = builtins::make_default_environment();

  explicit intrinsics_world(cvc::state &tree, std::size_t step_nodes = 0) {
    data_options data;
    data.step_nodes = step_nodes;
    sched.set_data_defaults(data);
    ictx.sched = &sched;
    ictx.proc = proc;
    apply_chroot(ictx, tree, "");
    register_intrinsics(env, &ictx);
  }
  int submit(const std::string &script) {
    execute_options opts;
    opts.env = environment::extend(env);
    opts.max_time = 60.0;
    const int pid = sched.execute(script, opts);
    adopt(pid);
    return pid;
  }
  void adopt(int pid) {
    ictx.pid = pid;
    proc->pid = pid;
  }
  // Step until `done`, at most `max` steps.
  template <class Done> bool step_until(Done done, int max = 100000) {
    for (int i = 0; i < max && !done(); ++i) {
      sched.drain_ingress();
      sched.sync_step();
    }
    return done();
  }
  bool finished(int pid) const {
    const auto info = sched.get_process_info(pid);
    return !info || is_finished(info->status);
  }
  std::string outcome(int pid) const {
    if (const auto r = sched.get_result(pid))
      return to_string(*r);
    return "<" + sched.get_exit_error(pid).value_or("no result") + ">";
  }
};

std::string value_at(cvc::state &root, const std::string &path) {
  cvc::state *node = root.findDescendant(path);
  return node ? node->value() : std::string();
}

} // namespace

TEST(DataJob, ARestoredBuiltinClassIsTheBuiltinOne) {
  // A snapshot writes a built-in class dict as data, its init as a native that decodes as nil.
  // Decoded, it is that class again: read-only, and shared by the isolating copy, so a restored
  // exception is stored as one made here would be (R0-mid semantics 3).
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  for (const std::string &name : exception_class_names()) {
    SCOPED_TRACE(name);
    const value_t exc = make_exception(exception_class(name), "m");
    encode_value(root("snap.each." + name), exc);
    const value_t restored = decode_value(root("snap.each." + name));
    EXPECT_NE(std::get<dict_ptr>(restored.v), std::get<dict_ptr>(exc.v));
    EXPECT_EQ(class_of_instance(restored), exception_class(name));
    const value_t stored = isolate(make_list({restored}));
    EXPECT_EQ(class_of_instance((*std::get<list_ptr>(stored.v))[0]), exception_class(name));
  }
  // A class dict alone (an environment's ValueError.__cls_dict__), and one deep enough to be
  // written into the spill table.
  encode_value(root("snap.cls"), value_t{exception_class("DeadlockError")});
  EXPECT_EQ(std::get<dict_ptr>(decode_value(root("snap.cls")).v), exception_class("DeadlockError"));
  encode_value(root("snap.deep"), nested_list(20, value_t{exception_class("KeyError")}));
  value_t at = decode_value(root("snap.deep"));
  for (int i = 0; i < 20; ++i)
    at = value_t{(*std::get<list_ptr>(at.v))[0]};
  EXPECT_EQ(std::get<dict_ptr>(at.v), exception_class("KeyError"));
}

TEST(DataJob, ARestoredUserClassNamedLikeABuiltinStaysAUserClass) {
  // Only the built-in shape is relinked: a class with an init or finalize of its own, or whose
  // superclass is not the built-in one, stays the user class it was, and its instance is refused.
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  const value_t own_init{std::make_shared<closure>()};
  const value_t native{native_fn{[](std::span<const value_t>) -> value_t { return nil_value; }}};
  const value_t exception{exception_class("Exception")};
  const value_t named{std::string("ValueError")};
  const std::vector<value_t> classes = {
      // (defclass ValueError Exception (init (self m) ...))
      make_dict({{"__name__", named}, {"__super__", exception}, {"init", own_init}}),
      // (defclass ValueError Exception): inherited init and finalize
      make_dict(
          {{"__name__", named}, {"__super__", exception}, {"init", native}, {"finalize", native}}),
      // Exception's keys, but not its name
      make_dict({{"__name__", named}, {"init", native}}),
      // its superclass a user class named Exception
      make_dict({{"__name__", named},
                 {"__super__",
                  make_dict({{"__name__", value_t{std::string("Exception")}}, {"init", own_init}})},
                 {"init", native}}),
      // Exception's shape, but an init that is no native's nil
      make_dict({{"__name__", value_t{std::string("Exception")}}, {"init", value_t{1}}}),
  };
  for (std::size_t i = 0; i < classes.size(); ++i) {
    SCOPED_TRACE(i);
    const std::string path = "snap.user." + std::to_string(i);
    encode_value(root(path), make_dict({{"__class__", classes[i]}, {"message", value_t{"m"}}}));
    const value_t restored = decode_value(root(path));
    const dict_ptr cls = class_of_instance(restored);
    ASSERT_NE(cls, nullptr);
    EXPECT_FALSE(is_builtin_class_dict(cls.get()));
    EXPECT_EQ(isolate_refusal(restored), refusal_text("object instance"));
  }
}

TEST(DataStepsStateDataSet, ARestoredProcessStoresItsCaughtException) {
  // A process holding a caught ValueError is snapshotted on one scheduler and restored on another
  // (a migration's path). It still catches it by class, and state-data-set stores it. A user class
  // named ValueError comes back as that user class, and its instance is still refused.
  const struct {
    const char *define;
    const char *expected;
  } programs[] = {
      {"", R"(("caught" "ValueError" "read-only" "stored"))"},
      {R"((defclass ValueError Exception (init (self m) (set-attr self "message" m) self)))",
       R"(("caught" "ValueError" "changed" "state-data-set: cannot store an object instance"))"},
  };
  for (const auto &p : programs) {
    SCOPED_TRACE(p.define);
    cvc::app app;
    cvc::state &root = cvc::state::instance(app);
    intrinsics_world a(root), b(root);
    const int pid = a.submit("(begin " + std::string(p.define) + R"(
        (set e (try (raise (ValueError "v")) (catch (x) x)))
        (state-set "ready" "1")
        (while (not (state-exists "go")) nil)
        (list (try (raise e) (catch (ValueError y) "caught"))
              (get-attr (get-attr e "__class__") "__name__")
              (try (begin (set-attr (get-attr e "__class__") "x" 1) "changed")
                   (catch (TypeError t) "read-only"))
              (try (begin (state-data-set "k" e) "stored") (catch (z) (get-attr z "message"))))))");
    ASSERT_TRUE(a.step_until([&] { return value_at(root, "ready") == "1"; }));
    ASSERT_TRUE(a.sched.snapshot(pid, root("snap")));
    a.sched.kill(pid);
    const int moved = b.sched.restore(root("snap"), b.env);
    b.adopt(moved);
    root("go").value(std::string("1"));
    ASSERT_TRUE(b.step_until([&] { return b.finished(moved); }));
    EXPECT_EQ(b.outcome(moved), p.expected);
  }
}

TEST(DataStepsStateDataSet, ASlicedStoreKeepsWhatItCopiedAlive) {
  // state-data-set copying a slice a step (a host's data.step_nodes, here 1) while another process
  // sharing the value (a common environment's binding) drops an element it copied already: that
  // element stays allocated to the end of the copy, which stores what it copied (R0a-mem-2).
  cvc::app app;
  intrinsics_world w(cvc::state::instance(app), 1);
  auto big = std::make_shared<std::vector<value_t>>();
  for (int i = 0; i < 400; ++i)
    big->push_back(make_list({value_t{std::string("old")}, value_t{int64_t{i}}}));
  w.env->set("big", value_t{big});
  const int pid = w.submit(R"((begin (state-data-set "k" big) (state-data-get "k")))");
  for (int s = 0; s < 600 && !w.finished(pid); ++s) {
    w.sched.drain_ingress();
    w.sched.sync_step();
  }
  ASSERT_FALSE(w.finished(pid)) << "the copy ended before the drop";
  const std::weak_ptr<std::vector<value_t>> first = std::get<list_ptr>((*big)[0].v);
  (*big)[0] = nil_value; // the other process's set-nth
  EXPECT_FALSE(first.expired()) << "a copied source was freed mid-copy";
  ASSERT_TRUE(w.step_until([&] { return w.finished(pid); }));
  const value_t got = w.sched.get_result(pid).value_or(nil_value);
  ASSERT_TRUE(std::holds_alternative<list_ptr>(got.v)) << w.outcome(pid);
  EXPECT_EQ(to_string((*std::get<list_ptr>(got.v))[0]), R"(("old" 0))");
  EXPECT_EQ(std::get<list_ptr>(got.v)->size(), 400u);
}

TEST(DataStepsStateDataSet, AJsonDictWithAClassKeyIsStored) {
  // Parsed JSON with a "__class__" key is data, whatever that key holds (R0-mid semantics 4).
  cvc::app app;
  intrinsics_world w(cvc::state::instance(app));
  const int pid = w.submit(R"((begin
      (state-data-set "a" (json-parse "{\"__class__\": {\"a\": 1}}"))
      (state-data-set "b" (json-parse "{\"x\": 1, \"__class__\": {\"__name__\": \"Box\"}}"))
      (list (state-data-get "a") (state-data-get "b"))))");
  ASSERT_TRUE(w.step_until([&] { return w.finished(pid); }));
  EXPECT_EQ(w.outcome(pid),
            R"(({"__class__": {"a": 1}} {"x": 1, "__class__": {"__name__": "Box"}}))");
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
  EXPECT_EQ(info->status, process_status::failed); // the native's error, uncaught
  EXPECT_NE(error(pid).find("state-from-json: line 1: expected value"), std::string::npos)
      << error(pid);
  EXPECT_GT(info->data_steps, 10u);
  EXPECT_EQ(root.findDescendant("target.k0"), nullptr);
}

TEST_F(DataStepsStateJson, TheDocumentIsHeldToTheAllocationLimit) {
  root("state_exec.defaults.limits.max_alloc_bytes").value("4096");
  const int pid = submit("(state-json \"wide\")");
  sched.sync_run();
  // The refused allocation is a MemoryError the program didn't catch.
  EXPECT_EQ(sched.get_process_info(pid)->status, process_status::failed);
  EXPECT_NE(error(pid).find("MemoryError: "), std::string::npos) << error(pid);
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

// ---- JSON: json-parse, json-encode, state-set-tree ------------------------------------

namespace {

// A feed reply: `n` items, each with a string `i` bytes long, numbers, a nested list; and a
// deep tail.
std::string feed_json(int n) {
  std::string doc = "{\"items\": [";
  for (int i = 0; i < n; ++i)
    doc += std::string(i ? ", " : "") + "{\"id\": " + std::to_string(i) + ", \"text\": \"" +
           std::string(static_cast<std::size_t>(i), 'x') +
           "\\u00e9\", \"tags\": [1, 2.5, null, true, false]}";
  doc += "], \"title\": \"feed\", \"deep\": " + std::string(100, '[') + std::string(100, ']') + "}";
  return doc;
}

} // namespace

// A scheduler slicing deep data 64 nodes a step, as DataStepsStateJson's, without its trees.
class DataStepsJson : public ::testing::Test {
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
  }

  int submit(const std::string &script) {
    execute_options opts;
    opts.env = environment::extend(env);
    return sched.execute(script, opts);
  }

  std::string error(int pid) const { return sched.get_exit_error(pid).value_or(""); }
};

TEST(DataJob, JsonJobsMatchWholeRunsInAnySlice) {
  const std::string doc = feed_json(300);
  auto whole = make_json_parse_job(doc);
  const value_t expect = run_sliced(*whole, 0).first;
  auto whole_text = make_json_encode_job(expect);
  const std::string text = std::get<std::string>(run_sliced(*whole_text, 0).first.v);
  auto whole_pretty = make_json_encode_job(expect, 2);
  const std::string pretty = std::get<std::string>(run_sliced(*whole_pretty, 0).first.v);
  for (std::size_t slice : {1u, 7u, 100000u}) {
    SCOPED_TRACE("slice " + std::to_string(slice));
    auto parse = make_json_parse_job(doc);
    auto [v, calls] = run_sliced(*parse, slice);
    EXPECT_TRUE(values_equal(v, expect));
    if (slice == 1)
      EXPECT_GE(calls, 300 * 8); // a node per value, and per 256 bytes
    auto encode = make_json_encode_job(v);
    EXPECT_EQ(std::get<std::string>(run_sliced(*encode, slice).first.v), text);
    auto indented = make_json_encode_job(v, 2);
    EXPECT_EQ(std::get<std::string>(run_sliced(*indented, slice).first.v), pretty);
  }
  auto back = make_json_parse_job(pretty);
  EXPECT_TRUE(values_equal(run_sliced(*back, 3).first, expect));
}

TEST(DataJob, JsonJobsAreHeldToTheAllocationLimit) {
  const std::string doc = feed_json(100);
  {
    eval_alloc_guard limit(std::size_t{4096});
    auto parse = make_json_parse_job(doc);
    EXPECT_THROW(run_sliced(*parse, 16), std::runtime_error);
  }
  auto parse = make_json_parse_job(doc);
  const value_t v = run_sliced(*parse, 0).first;
  eval_alloc_guard limit(std::size_t{4096});
  auto encode = make_json_encode_job(v);
  EXPECT_THROW(run_sliced(*encode, 16), std::runtime_error);
}

TEST_F(DataStepsJson, JsonParseAndEncodeRunASliceAStep) {
  const std::string doc = feed_json(300);
  root("doc").value(doc);
  const int pid = submit("(json-encode (json-parse (state-get \"doc\")) 2)");
  sched.sync_run();
  const auto info = sched.get_process_info(pid);
  ASSERT_EQ(info->status, process_status::terminated) << error(pid);
  auto parse = make_json_parse_job(doc);
  auto encode = make_json_encode_job(run_to_completion(*parse), 2);
  EXPECT_EQ(std::get<std::string>(sched.get_result(pid)->v),
            std::get<std::string>(run_to_completion(*encode).v));
  EXPECT_GT(info->data_steps, 100u);
}

TEST_F(DataStepsJson, AJsonErrorIsCatchableThoughItsParseSpansSteps) {
  std::string doc = "[";
  for (int i = 0; i < 5000; ++i)
    doc += "1,\n";
  doc += "]";
  root("doc").value(doc);
  const int pid = submit("(try (json-parse (state-get \"doc\"))"
                         " (catch (ValueError e) (list (get-attr e \"line\") (get-attr e \"col\")"
                         " (get-attr e \"message\"))))");
  sched.sync_run();
  const auto info = sched.get_process_info(pid);
  ASSERT_EQ(info->status, process_status::terminated) << error(pid);
  const value_t r = *sched.get_result(pid);
  ASSERT_TRUE(std::holds_alternative<list_ptr>(r.v)) << to_string(r);
  const auto &l = *std::get<list_ptr>(r.v);
  EXPECT_EQ(std::get<int64_t>(l.at(0).v), 5001);
  EXPECT_EQ(std::get<int64_t>(l.at(1).v), 1);
  EXPECT_EQ(std::get<std::string>(l.at(2).v), "json-parse: line 5001, column 1: expected a value");
  EXPECT_GT(info->data_steps, 10u);
}

TEST_F(DataStepsJson, StateSetTreeWritesAParsedReplyASliceAStep) {
  root("doc").value(feed_json(300));
  const int pid = submit("(state-set-tree \"feed\" (json-parse (state-get \"doc\")))");
  sched.sync_run();
  const auto info = sched.get_process_info(pid);
  ASSERT_EQ(info->status, process_status::terminated) << error(pid);
  EXPECT_GT(info->data_steps, 100u);
  EXPECT_EQ(root("feed").value(), "");
  EXPECT_EQ(root("feed.title").value(), "feed");
  EXPECT_EQ(root("feed.items").value(), "300");
  EXPECT_EQ(root("feed.items.0.id").value(), "0");
  EXPECT_EQ(root("feed.items.299.text").value(), std::string(299, 'x') + "\xC3\xA9");
  EXPECT_EQ(root("feed.items.5.tags.1").value(), "2.5");
  EXPECT_EQ(root("feed.items.5.tags.2").value(), "");
  EXPECT_EQ(root("feed.items.5.tags.4").value(), "false");
  EXPECT_EQ(root("feed.deep").value(), "1");

  // The next reply replaces it.
  root("doc").value(R"({"items": [{"id": 7}], "error": "rate limited"})");
  const int again = submit("(state-set-tree \"feed\" (json-parse (state-get \"doc\")))");
  sched.sync_run();
  ASSERT_EQ(sched.get_process_info(again)->status, process_status::terminated) << error(again);
  EXPECT_EQ(root("feed.items").value(), "1");
  EXPECT_EQ(root("feed.items").numChildren(), 1u);
  EXPECT_EQ(root("feed.items.0").childNames(), std::vector<std::string>{"id"});
  EXPECT_EQ(root.findDescendant("feed.title"), nullptr);
  EXPECT_EQ(root("feed.error").value(), "rate limited");
}

TEST_F(DataStepsJson, StateSetTreeChecksTheWholeValueThoughTheCheckSpansSteps) {
  std::string doc = "[";
  for (int i = 0; i < 3000; ++i)
    doc += "{\"k\": " + std::to_string(i) + "}, ";
  doc += "{\"a.b\": 1}]";
  root("doc").value(doc);
  const int pid = submit("(state-set-tree \"target\" (json-parse (state-get \"doc\")))");
  sched.sync_run();
  const auto info = sched.get_process_info(pid);
  EXPECT_EQ(info->status, process_status::failed);
  EXPECT_NE(error(pid).find("ValueError: state-set-tree: the key \"a.b\" can't name a node"),
            std::string::npos)
      << error(pid);
  EXPECT_GT(info->data_steps, 40u);
  EXPECT_EQ(root.findDescendant("target"), nullptr);
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
