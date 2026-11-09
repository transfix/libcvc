// The scripting engines' seam (script_engine.h; docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §3.1,
// §8.3–8.7), through a stub engine: registration and its block key, a document's block handed to
// the engine with the document's lane, unload on a reload, activations run as processes of the
// program lane (slices charged as steps under the lane's caps, frames yielded, the language's
// frame budget, calls into the lane's functions and their narrowing, awaits, errors, stop and
// teardown), and the hooks (an on: event, every frame, a value that changes).

#include <chrono>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/loader.h> // has_ari_block
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/script_engine.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/exceptions.h>
#include <cvc/state/state_exec/intrinsics.h> // make_future, resolve_channel_key
#include <cvc/state/state_exec/types.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using namespace cvc::ariadne;
namespace se = cvc::state_exec;
namespace fs = std::filesystem;

namespace {

#define REQUIRE_YAML()                                                                             \
  do {                                                                                             \
    if (!AppRuntime::have_yaml())                                                                  \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

constexpr const char *kLanguage = "stubscript";

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

fs::path write_doc(const std::string &name, const std::string &text) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_script_engine_test";
  fs::create_directories(dir);
  const fs::path p = dir / (name + ".ari");
  std::ofstream(p) << text;
  return p;
}

bool any_contains(const std::vector<std::string> &v, const std::string &needle) {
  for (const std::string &s : v)
    if (s.find(needle) != std::string::npos)
      return true;
  return false;
}

std::string text_of(const se::value_t &v) {
  const auto *s = std::get_if<std::string>(&v.v);
  return s ? *s : std::string();
}

std::int64_t int_of(const se::value_t &v) {
  const auto *i = std::get_if<std::int64_t>(&v.v);
  return i ? *i : -1;
}

// A field of an event dict, as text.
std::string field_text(const se::value_t &dict, const char *key) {
  const se::value_t *f = se::field(dict, key);
  return f ? text_of(*f) : std::string();
}

// An activation whose slices the test scripts: slice n (from 0) runs body(slice, n).
class ScriptedActivation : public ScriptActivation {
public:
  using Body = std::function<void(ScriptSlice &, int)>;
  explicit ScriptedActivation(Body body) : body_(std::move(body)) {}
  void step(ScriptSlice &slice) override { body_(slice, steps++); }
  void cancel() override { cancelled = true; }
  int steps = 0;
  bool cancelled = false;

private:
  Body body_;
};

// A language engine that records what it is handed, and runs a test's on_load in load_block.
class StubEngine : public ScriptEngine {
public:
  std::string language() const override { return kLanguage; }
  bool has_module(const std::string &module) const override { return module == "present"; }
  void load_block(const ScriptBlock &block, ScriptLane &lane,
                  std::vector<std::string> &warnings) override {
    blocks.push_back(block);
    lanes.push_back(&lane);
    if (on_load)
      on_load(block, lane, warnings);
  }
  void unload(ScriptLane &) override { ++unloads; }
  std::vector<ScriptBlock> blocks;
  std::vector<ScriptLane *> lanes;
  int unloads = 0;
  std::function<void(const ScriptBlock &, ScriptLane &, std::vector<std::string> &)> on_load;
};

// Each test registers the stub for the process, as a scripting host registers its engine, and
// unregisters it after.
class ScriptEngineTest : public ::testing::Test {
protected:
  void SetUp() override {
    engine = std::make_shared<StubEngine>();
    register_script_engine(engine);
  }
  void TearDown() override { unregister_script_engine(kLanguage); }

  // Drain until `done` holds, at most `max` times; how many drains it took (max + 1: never).
  static int drain_until(AppRuntime &art, const std::function<bool()> &done, int max = 50) {
    for (int i = 1; i <= max; ++i) {
      art.drain();
      if (done())
        return i;
    }
    return max + 1;
  }

  std::shared_ptr<StubEngine> engine;
};

const char kDoc[] = "requires: [stub-add]\n"
                    "functions: |\n"
                    "  (defun twice (x) (* 2 x))\n"
                    "stubscript:\n"
                    "  source: hi\n"
                    "init: (state-set \"x\" (stub-add 2 3))\n"
                    "root:\n"
                    "  - text: hi\n";

// The stub's block exports (stub-add a b) through its lane, and warns.
void export_stub_add(const ScriptBlock &, ScriptLane &lane, std::vector<std::string> &warnings) {
  lane.export_function("stub-add", [](std::span<const se::value_t> a) -> se::value_t {
    return se::value_t(int_of(a[0]) + int_of(a[1]));
  });
  warnings.push_back("from the stub's block");
}

} // namespace

// An engine is registered by its language, which becomes a block key the loader keeps; its tokens
// are its language's (python's are the documented ones, so a stub's language answers
// has_script_engine but is no capability token).
TEST_F(ScriptEngineTest, AnEngineIsRegisteredByItsLanguage) {
  EXPECT_EQ(find_script_engine(kLanguage), engine);
  EXPECT_TRUE(has_script_engine(kLanguage));
  EXPECT_TRUE(has_ari_block(kLanguage));
  EXPECT_EQ(find_script_engine("no-such-language"), nullptr);

  auto other = std::make_shared<StubEngine>();
  register_script_engine(other); // the same language: it replaces the first
  EXPECT_EQ(find_script_engine(kLanguage), other);
  unregister_script_engine(kLanguage);
  EXPECT_FALSE(has_script_engine(kLanguage));
  EXPECT_EQ(find_script_engine(kLanguage), nullptr);
}

// A runtime that uses the engine hands it the document's block at load, with the document's
// path, directory and prefix and the runtime's lane for the engine; what it exports there is a
// host function that requires: and init: see, and its warnings are the load's.
TEST_F(ScriptEngineTest, ADocumentsBlockReachesItsEngineWithTheLane) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  EXPECT_EQ(art.script_lane(kLanguage), &lane);
  EXPECT_EQ(art.script_lane("python"), nullptr);
  EXPECT_EQ(lane.language(), kLanguage);
  EXPECT_EQ(lane.prefix(), "ui.se");
  EXPECT_EQ(&lane.app(), &app);
  engine->on_load = export_stub_add;

  const fs::path doc = write_doc("block", kDoc);
  std::vector<std::string> warns;
  ASSERT_NO_THROW(warns = art.load(doc.string()));
  ASSERT_EQ(engine->blocks.size(), 1u);
  const ScriptBlock &b = engine->blocks[0];
  EXPECT_EQ(b.language, kLanguage);
  ASSERT_TRUE(b.content.is_map());
  ASSERT_NE(b.content.find("source"), nullptr);
  EXPECT_EQ(b.content.find("source")->as_string(), "hi");
  EXPECT_EQ(b.document, doc.string());
  EXPECT_EQ(fs::path(b.dir), fs::absolute(doc).parent_path());
  EXPECT_EQ(b.prefix, "ui.se");
  EXPECT_EQ(b.module, "");
  EXPECT_EQ(engine->lanes[0], &lane);
  EXPECT_EQ(val(app, "ui.se.x"), "5") << "init: called the block's export";
  EXPECT_TRUE(any_contains(warns, "from the stub's block"));
  EXPECT_FALSE(any_contains(warns, "stub-add")) << "the load's scan saw the export";
}

// A runtime that does not use the engine leaves the block alone (the scripting hosts attach their
// engine to each runtime they run a document in).
TEST_F(ScriptEngineTest, ARuntimeThatDoesNotUseTheEngineLeavesItsBlockAlone) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.plain");
  const fs::path doc = write_doc("unused", "stubscript: hi\nroot:\n  - text: hi\n");
  ASSERT_NO_THROW(art.load(doc.string()));
  EXPECT_TRUE(engine->blocks.empty());
}

// A block that throws fails the load, with what it threw.
TEST_F(ScriptEngineTest, ABlockThatThrowsFailsTheLoad) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.se");
  art.use_script_engine(engine);
  engine->on_load = [](const ScriptBlock &, ScriptLane &, std::vector<std::string> &) {
    throw std::runtime_error("stubscript: the block is broken");
  };
  const fs::path doc = write_doc("throws", "stubscript: hi\nroot:\n  - text: hi\n");
  try {
    art.load(doc.string());
    FAIL() << "the load should fail";
  } catch (const std::runtime_error &e) {
    EXPECT_NE(std::string(e.what()).find("the block is broken"), std::string::npos) << e.what();
  }
}

// From the second load on, the engine is unloaded before its block runs again, and the hooks it
// set go with the document.
TEST_F(ScriptEngineTest, AReloadUnloadsTheLastDocumentFirst) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  int frames = 0;
  engine->on_load = [&](const ScriptBlock &, ScriptLane &l, std::vector<std::string> &) {
    if (engine->blocks.size() == 1) // the first load sets a frame hook; the reload doesn't
      l.on_frame([&](const se::value_t &) {
        ++frames;
        return nullptr;
      });
  };
  const fs::path doc = write_doc("reload", "stubscript: hi\nroot:\n  - text: hi\n");
  art.load(doc.string());
  art.drain();
  EXPECT_EQ(frames, 1);
  EXPECT_EQ(engine->unloads, 0);
  art.load(doc.string());
  EXPECT_EQ(engine->unloads, 1);
  EXPECT_EQ(engine->blocks.size(), 2u);
  art.drain();
  EXPECT_EQ(frames, 1) << "the first document's hook went with it";
  (void)lane;
}

// An activation runs as a process of the document's program lane, named as (ps) shows it, a
// slice at a time; slices that need no frame run in the drain that starts them, and its result
// reaches on_done.
TEST_F(ScriptEngineTest, AnActivationRunsInSlicesInOneDrain) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  auto a = std::make_shared<ScriptedActivation>([](ScriptSlice &s, int n) {
    EXPECT_EQ(s.budget(), 64u) << "ariadne.limits.stubscript_slice_steps' default";
    EXPECT_FALSE(s.has_reply());
    s.charge(10);
    if (n == 4)
      s.finish(se::value_t(std::int64_t(42)));
  });
  ActivationOptions opt;
  opt.name = "stubscript/count";
  bool ended = false;
  ActivationOutcome out;
  opt.on_done = [&](const ActivationOutcome &o) {
    ended = true;
    out = o;
  };
  const int pid = lane.start(a, std::move(opt));
  const auto info = app.exec_scheduler().get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->name, "stubscript/count");
  EXPECT_TRUE(lane.running(pid));

  EXPECT_EQ(drain_until(art, [&] { return ended; }), 1) << "quick work finishes in its drain";
  EXPECT_TRUE(out.ok) << out.error;
  EXPECT_EQ(int_of(out.value), 42);
  EXPECT_EQ(a->steps, 5);
  EXPECT_FALSE(a->cancelled);
  EXPECT_FALSE(lane.running(pid));
}

// on_done may start the next activation: the runtime's sweep is still walking its processes when
// it calls on_done, and the new one joins them after.
TEST_F(ScriptEngineTest, AnOnDoneMayStartTheNextActivation) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  int finished = 0;
  std::function<void()> next = [&] {
    ActivationOptions opt;
    opt.on_done = [&](const ActivationOutcome &o) {
      EXPECT_TRUE(o.ok) << o.error;
      if (++finished < 3)
        next();
    };
    lane.start(std::make_shared<ScriptedActivation>([](ScriptSlice &s, int) { s.finish(); }),
               std::move(opt));
  };
  next();
  EXPECT_LE(drain_until(art, [&] { return finished == 3; }), 3);
  EXPECT_EQ(finished, 3);
}

// What a slice charges is DSL steps of its process, so the lane's step cap is the activation's gas
// cap: an action's ariadne.limits.action_max_steps (5000), lowered by its own max_steps, never
// raised; a resident has no total.
TEST_F(ScriptEngineTest, TheLanesStepCapBoundsWhatTheSlicesCharge) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  const auto run = [&](ProgramLane which, std::uint64_t own_cap, int finish_at) {
    auto a = std::make_shared<ScriptedActivation>([finish_at](ScriptSlice &s, int n) {
      s.charge(1000);
      if (n == finish_at)
        s.finish();
    });
    ActivationOptions opt;
    opt.lane = which;
    opt.max_steps = own_cap;
    bool ended = false;
    ActivationOutcome out;
    opt.on_done = [&](const ActivationOutcome &o) {
      ended = true;
      out = o;
    };
    lane.start(a, std::move(opt));
    drain_until(art, [&] { return ended; });
    EXPECT_TRUE(ended);
    return std::make_tuple(out, a->steps, a->cancelled);
  };

  auto [capped, slices, cancelled] = run(ProgramLane::action, 0, 1000);
  EXPECT_FALSE(capped.ok);
  EXPECT_NE(capped.error.find("max_steps_exceeded"), std::string::npos) << capped.error;
  EXPECT_LE(slices, 6) << "5000 steps at 1000 a slice";
  EXPECT_TRUE(cancelled) << "an activation that never finished is cancelled";

  auto [lowered, few, c2] = run(ProgramLane::action, 2500, 1000);
  EXPECT_FALSE(lowered.ok);
  EXPECT_LE(few, 3) << "its own cap lowers the lane's";
  (void)c2;

  auto [raised, all, c3] = run(ProgramLane::action, 100000, 9);
  EXPECT_FALSE(raised.ok) << "its own cap never raises the lane's";
  EXPECT_LE(all, 6);
  (void)c3;

  auto [resident, ten, c4] = run(ProgramLane::resident, 0, 9);
  EXPECT_TRUE(resident.ok) << resident.error;
  EXPECT_EQ(ten, 10) << "a resident has no total";
  EXPECT_FALSE(c4);
}

// A slice that yields the frame resumes in the next one.
TEST_F(ScriptEngineTest, AYieldResumesInTheNextFrame) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  auto a = std::make_shared<ScriptedActivation>([](ScriptSlice &s, int n) {
    if (n < 3)
      s.yield_frame();
    else
      s.finish();
  });
  bool ended = false;
  ActivationOptions opt;
  opt.on_done = [&](const ActivationOutcome &o) {
    ended = true;
    EXPECT_TRUE(o.ok) << o.error;
  };
  lane.start(a, std::move(opt));
  art.drain();
  EXPECT_EQ(a->steps, 1);
  art.drain();
  EXPECT_EQ(a->steps, 2);
  EXPECT_EQ(drain_until(art, [&] { return ended; }), 2);
  EXPECT_EQ(a->steps, 4);
}

// A language's activations share ariadne.limits.<language>_frame_budget of a frame: once it is
// spent, the rest yield to the next frame. With no budget, the same work fits in one drain.
TEST_F(ScriptEngineTest, ALanguagesActivationsShareAFrameBudget) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  const auto busy = [](ScriptSlice &s, int n) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::microseconds(1500)) {
    }
    if (n == 3)
      s.finish();
  };
  const auto run_two = [&] {
    int ended = 0;
    for (int i = 0; i < 2; ++i) {
      ActivationOptions opt;
      opt.on_done = [&](const ActivationOutcome &o) {
        ++ended;
        EXPECT_TRUE(o.ok) << o.error;
      };
      lane.start(std::make_shared<ScriptedActivation>(busy), std::move(opt));
    }
    return drain_until(art, [&] { return ended == 2; });
  };
  auto &st = cvc::state::instance(app);
  st("ariadne.limits.stubscript_frame_budget").value(std::string("0.002"));
  EXPECT_GE(run_two(), 3) << "8 slices of 1.5 ms, about 2 ms of them a frame";
  st("ariadne.limits.stubscript_frame_budget").value(std::string("0"));
  EXPECT_EQ(run_two(), 1) << "no budget: all in the drain that starts them";
}

// call() calls what the lane holds, in the activation's process: the state natives under the
// document's chroot, a host function, the document's functions:. Its value or its exception comes
// back to the next slice, and a name that isn't there is a NameError.
TEST_F(ScriptEngineTest, ACallReachesTheLanesFunctionsInItsChroot) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  engine->on_load = export_stub_add;
  ASSERT_NO_THROW(art.load(write_doc("calls", kDoc).string()));

  std::vector<std::string> got;
  auto a = std::make_shared<ScriptedActivation>([&](ScriptSlice &s, int n) {
    if (n > 0) {
      EXPECT_TRUE(s.has_reply());
      got.push_back(s.failed() ? "error " + s.error() : text_of(s.reply()));
      if (!s.failed() && std::get_if<std::int64_t>(&s.reply().v))
        got.back() = std::to_string(int_of(s.reply()));
    }
    switch (n) {
    case 0:
      s.call("state-set", {se::value_t(std::string("y")), se::value_t(std::string("7"))});
      break;
    case 1:
      s.call("state-get", {se::value_t(std::string("y"))});
      break;
    case 2:
      s.call("twice", {se::value_t(std::int64_t(21))});
      break;
    case 3:
      s.call("stub-add", {se::value_t(std::int64_t(1)), se::value_t(std::int64_t(2))});
      break;
    case 4:
      s.call("state-get", {});
      break;
    case 5:
      s.call("no-such-function", {});
      break;
    default:
      s.finish();
    }
  });
  bool ended = false;
  ActivationOptions opt;
  opt.on_done = [&](const ActivationOutcome &o) {
    ended = true;
    EXPECT_TRUE(o.ok) << o.error;
  };
  lane.start(a, std::move(opt));
  EXPECT_EQ(drain_until(art, [&] { return ended; }), 1);
  ASSERT_EQ(got.size(), 6u);
  EXPECT_EQ(val(app, "ui.se.y"), "7") << "written under the document's prefix";
  EXPECT_EQ(got[1], "7");
  EXPECT_EQ(got[2], "42") << "a document function";
  EXPECT_EQ(got[3], "3") << "a host function";
  EXPECT_EQ(got[4].rfind("error ", 0), 0u) << got[4];
  EXPECT_NE(got[5].find("NameError"), std::string::npos) << got[5];
}

// ActivationOptions::may_call narrows what call() reaches (a guest's imports:): a name outside it
// is a PermissionError, which the activation sees and may go on from.
TEST_F(ScriptEngineTest, MayCallNarrowsWhatACallReaches) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  cvc::state::instance(app)("ui.se.z").value(std::string("zed"));
  std::vector<std::string> got;
  auto a = std::make_shared<ScriptedActivation>([&](ScriptSlice &s, int n) {
    if (n > 0)
      got.push_back(s.failed() ? s.error() : text_of(s.reply()));
    if (n == 0)
      s.call("state-set", {se::value_t(std::string("z")), se::value_t(std::string("no"))});
    else if (n == 1)
      s.call("state-get", {se::value_t(std::string("z"))});
    else
      s.finish();
  });
  ActivationOptions opt;
  opt.may_call = [](const std::string &f) { return f == "state-get"; };
  bool ended = false;
  opt.on_done = [&](const ActivationOutcome &) { ended = true; };
  lane.start(a, std::move(opt));
  drain_until(art, [&] { return ended; });
  ASSERT_EQ(got.size(), 2u);
  EXPECT_NE(got[0].find("PermissionError"), std::string::npos) << got[0];
  EXPECT_EQ(got[1], "zed");
  EXPECT_EQ(val(app, "ui.se.z"), "zed");
}

// await() parks the process until its future settles, costing nothing meanwhile; the next slice
// gets the value.
TEST_F(ScriptEngineTest, AnAwaitParksUntilItsFutureSettles) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  const std::string channel = se::resolve_channel_key(std::string(), "script_engine_test#1");
  std::string got;
  auto a = std::make_shared<ScriptedActivation>([&](ScriptSlice &s, int n) {
    if (n == 0) {
      s.await(se::make_future(channel));
      return;
    }
    got = text_of(s.reply());
    s.finish();
  });
  bool ended = false;
  ActivationOptions opt;
  opt.on_done = [&](const ActivationOutcome &o) {
    ended = true;
    EXPECT_TRUE(o.ok) << o.error;
  };
  lane.start(a, std::move(opt));
  art.drain();
  art.drain();
  EXPECT_EQ(a->steps, 1) << "parked";
  app.exec_scheduler().post_message(channel, se::value_t(std::string("hello")));
  EXPECT_LE(drain_until(art, [&] { return ended; }), 2);
  EXPECT_EQ(got, "hello");
}

// A step that throws fails the activation with what it threw, as an uncaught exception fails a
// program: a dsl_error keeps its class.
TEST_F(ScriptEngineTest, AStepThatThrowsFailsTheActivation) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  auto a = std::make_shared<ScriptedActivation>(
      [](ScriptSlice &, int) { throw se::dsl_error("ValueError", "the guest trapped"); });
  bool ended = false;
  ActivationOutcome out;
  ActivationOptions opt;
  opt.on_done = [&](const ActivationOutcome &o) {
    ended = true;
    out = o;
  };
  lane.start(a, std::move(opt));
  drain_until(art, [&] { return ended; });
  EXPECT_FALSE(out.ok);
  EXPECT_NE(out.error.find("ValueError"), std::string::npos) << out.error;
  EXPECT_NE(out.error.find("the guest trapped"), std::string::npos) << out.error;
  EXPECT_TRUE(a->cancelled);
}

// stop() kills an activation; one still running when its runtime goes is cancelled with it.
TEST_F(ScriptEngineTest, AnActivationEndsWithStopOrItsDocument) {
  cvc::app app;
  const auto forever = [](ScriptSlice &s, int) { s.yield_frame(); };
  {
    AppRuntime art(app, "ui.se");
    ScriptLane &lane = art.use_script_engine(engine);
    auto a = std::make_shared<ScriptedActivation>(forever);
    bool ended = false;
    ActivationOutcome out;
    ActivationOptions opt;
    opt.on_done = [&](const ActivationOutcome &o) {
      ended = true;
      out = o;
    };
    const int pid = lane.start(a, std::move(opt));
    art.drain();
    EXPECT_TRUE(lane.running(pid));
    lane.stop(pid);
    drain_until(art, [&] { return ended; });
    EXPECT_FALSE(out.ok);
    EXPECT_TRUE(a->cancelled);
  }
  auto b = std::make_shared<ScriptedActivation>(forever);
  {
    AppRuntime art(app, "ui.se2");
    ScriptLane &lane = art.use_script_engine(engine);
    lane.start(b, {});
    art.drain();
    art.drain();
    EXPECT_FALSE(b->cancelled);
  }
  EXPECT_TRUE(b->cancelled) << "the document went, and its activation with it";
}

// on_event: an on: event the document fires (here a key binding's) is delivered to the
// activation the hook makes, as an action, in the drain that fires it.
TEST_F(ScriptEngineTest, AnEventHookRunsAnActivationPerEvent) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write_doc("event", "keys:\n"
                                              "  replan:\n"
                                              "    key: R\n"
                                              "    on: replan\n"
                                              "root:\n"
                                              "  - text: hi\n")
                               .string()));
  std::vector<std::string> events;
  int finished = 0;
  ActivationOptions opt;
  opt.on_done = [&](const ActivationOutcome &o) {
    EXPECT_TRUE(o.ok) << o.error;
    ++finished;
  };
  lane.on_event(
      "replan",
      [&](const se::value_t &event) {
        events.push_back(field_text(event, "event"));
        return std::make_shared<ScriptedActivation>([](ScriptSlice &s, int) { s.finish(); });
      },
      opt);
  art.post_key(true, "R");
  art.drain();
  EXPECT_EQ(events, std::vector<std::string>{"replan"});
  EXPECT_EQ(finished, 1) << "finished in the drain that fired it";
  art.post_key(true, "R");
  art.drain();
  EXPECT_EQ(events.size(), 2u);
  EXPECT_EQ(finished, 2);
}

// on_frame: a delivery at the end of every drain, one in flight, so a slow one coalesces the
// frames it spans.
TEST_F(ScriptEngineTest, AFrameHookDeliversOneAtATime) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  std::vector<std::int64_t> frames;
  lane.on_frame([&](const se::value_t &event) {
    const se::value_t *f = se::field(event, "frame");
    frames.push_back(f ? int_of(*f) : -1);
    return std::make_shared<ScriptedActivation>([](ScriptSlice &s, int n) {
      if (n < 2)
        s.yield_frame(); // spans three frames
      else
        s.finish();
    });
  });
  for (int i = 0; i < 6; ++i)
    art.drain();
  EXPECT_GE(frames.size(), 2u);
  EXPECT_LE(frames.size(), 3u) << "one in flight: the frames it spans are coalesced";
  for (std::size_t i = 1; i < frames.size(); ++i)
    EXPECT_GT(frames[i], frames[i - 1]);
}

// on_watch: a value under the document's prefix that changed since the last delivery (or since
// the hook was set) is delivered at the end of the drain, with its path and value.
TEST_F(ScriptEngineTest, AWatchHookDeliversAChangedValue) {
  cvc::app app;
  AppRuntime art(app, "ui.se");
  ScriptLane &lane = art.use_script_engine(engine);
  auto &st = cvc::state::instance(app);
  st("ui.se.w").value(std::string("a"));
  std::vector<std::pair<std::string, std::string>> seen;
  lane.on_watch("w", [&](const se::value_t &event) {
    seen.emplace_back(field_text(event, "path"), field_text(event, "value"));
    return std::make_shared<ScriptedActivation>([](ScriptSlice &s, int) { s.finish(); });
  });
  art.drain();
  EXPECT_TRUE(seen.empty()) << "unchanged since the hook was set";
  st("ui.se.w").value(std::string("b"));
  art.drain();
  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0].first, "w");
  EXPECT_EQ(seen[0].second, "b");
  art.drain();
  EXPECT_EQ(seen.size(), 1u);
}
