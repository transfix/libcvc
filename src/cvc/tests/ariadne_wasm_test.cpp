// WebAssembly guests in documents (src/ariadne_wasm; docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
// §8.2-8.7, phase G1), on wasmtime: the CBOR codec; a wasm: block whose WAT guest's exports the
// document's programs call (each call an activation, parked on by the caller); calls back into
// the lane under the document's chroot, and their narrowing; slices of fuel charged as steps
// under the action caps; a yield; traps and fail() as WasmError; _initialize; re-entry, direct
// or through a document function, as a RecursionError; one activation per instance, the others
// waiting as with-lock waits; a frame hook; the block's load errors; and host objects by handle
// (§8.4.1): volumes, meshes and height fields, read a region at a time, their handles dropped,
// narrowed by imports:, and a guest out of memory as a MemoryError.

#include "value_cbor.h"
#include "wasm_engine.h"

#include <boost/any.hpp>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/script_engine.h>
#include <cvc/core/app.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/height_field.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
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

fs::path dir() {
  const fs::path d = fs::temp_directory_path() / "ariadne_wasm_test";
  fs::create_directories(d);
  return d;
}

fs::path write(const std::string &name, const std::string &text) {
  const fs::path p = dir() / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

std::string val(cvc::app &app, const std::string &path) {
  return cvc::state::instance(app)(path).value();
}

// The test guest. Values cross as CBOR; an export takes (ptr, len) of its argument list and
// returns its result's (ptr << 32 | len).
const char kGuest[] = R"WAT(
(module
  (import "ari" "call" (func $call (param i32 i32 i32 i32) (result i64)))
  (import "ari" "reply" (func $reply (param i32)))
  (import "ari" "yield" (func $yield))
  (import "ari" "fail" (func $fail (param i32 i32)))
  (memory (export "memory") 1)
  (global $heap (mut i32) (i32.const 4096))
  (global $inits (mut i32) (i32.const 0))
  (func (export "_initialize") (global.set $inits (i32.add (global.get $inits) (i32.const 1))))
  (func $alloc (export "ari_alloc") (param $n i32) (result i32) (local $p i32)
    (local.set $p (global.get $heap))
    (global.set $heap (i32.add (global.get $heap) (local.get $n)))
    (local.get $p))
  (func $pack (param $p i32) (param $n i32) (result i64)
    (i64.or (i64.shl (i64.extend_i32_u (local.get $p)) (i64.const 32))
            (i64.extend_i32_u (local.get $n))))
  (func (export "echo") (param $p i32) (param $n i32) (result i64)
    (call $pack (local.get $p) (local.get $n)))
  (func (export "nothing") (param i32 i32) (result i64) (i64.const 0))
  (func (export "spin") (param i32 i32) (result i64) (local $i i32)
    (loop $l
      (local.set $i (i32.add (local.get $i) (i32.const 1)))
      (br_if $l (i32.lt_u (local.get $i) (i32.const 3000000))))
    (i64.const 0))
  ;; Call the function named at [np, np+nl) with the arguments at [ap, ap+al); its result's CBOR,
  ;; or fail with its error's text.
  (func $ask (param $np i32) (param $nl i32) (param $ap i32) (param $al i32) (result i64)
    (local $r i64) (local $dst i32)
    (local.set $r (call $call (local.get $np) (local.get $nl) (local.get $ap) (local.get $al)))
    (local.set $dst (global.get $heap))
    (call $reply (local.get $dst))
    (if (i64.lt_s (local.get $r) (i64.const 0))
      (then (call $fail (local.get $dst) (i32.wrap_i64 (i64.sub (i64.const 0) (local.get $r))))
            (unreachable)))
    (global.set $heap (i32.add (global.get $heap) (i32.wrap_i64 (local.get $r))))
    (call $pack (local.get $dst) (i32.wrap_i64 (local.get $r))))
  (data (i32.const 16) "state-get")
  (data (i32.const 32) "\81\61x")
  (data (i32.const 48) "state-set")
  (data (i32.const 64) "\82\61y\62ok")
  (data (i32.const 80) "test-ping")
  (data (i32.const 96) "\80")
  (data (i32.const 112) "\07")
  (data (i32.const 128) "no route")
  (data (i32.const 144) "echo")
  (func (export "get-x") (param i32 i32) (result i64)
    (call $ask (i32.const 16) (i32.const 9) (i32.const 32) (i32.const 3)))
  (func (export "set-y") (param i32 i32) (result i64)
    (call $ask (i32.const 48) (i32.const 9) (i32.const 64) (i32.const 6)))
  (func (export "ping") (param i32 i32) (result i64)
    (call $ask (i32.const 80) (i32.const 9) (i32.const 96) (i32.const 1)))
  (func (export "nap") (param i32 i32) (result i64)
    (call $yield) (call $yield) (call $pack (i32.const 112) (i32.const 1)))
  (func (export "boom") (param i32 i32) (result i64) (unreachable))
  (func (export "refuse") (param i32 i32) (result i64)
    (call $fail (i32.const 128) (i32.const 8)) (i64.const 0))
  (func (export "inits") (param i32 i32) (result i64) (local $p i32)
    (local.set $p (call $alloc (i32.const 1)))
    (i32.store8 (local.get $p) (global.get $inits))
    (call $pack (local.get $p) (i32.const 1)))
  (func (export "self") (param i32 i32) (result i64)
    (call $ask (i32.const 144) (i32.const 4) (i32.const 96) (i32.const 1)))
  ;; At every load: (state-set "booted" "yes") through the host, refused or not (imports:).
  (data (i32.const 176) "\82\66booted\63yes")
  (func (export "ari_init") (param i32 i32) (result i64)
    (drop (call $call (i32.const 48) (i32.const 9) (i32.const 176) (i32.const 12)))
    (i64.const 0))
  ;; (relay): (bounce), the document's function, which may call back into the guest.
  (data (i32.const 192) "bounce")
  (func (export "relay") (param i32 i32) (result i64)
    (call $ask (i32.const 192) (i32.const 6) (i32.const 96) (i32.const 1)))
)
)WAT";

const char kExports[] =
    "[echo, nothing, spin, get-x, set-y, nap, boom, refuse, inits, self, relay]";

std::string doc(const std::string &wasm_extra = std::string()) {
  return "requires: [wasm]\n"
         "wasm:\n"
         "  module: guest.wat\n"
         "  export: " +
         std::string(kExports) + "\n" + wasm_extra +
         "root:\n"
         "  - text: hi\n";
}

class WasmEngineTest : public ::testing::Test {
protected:
  void SetUp() override {
    std::string error;
    engine = wasm::make_engine(error);
    ASSERT_TRUE(engine) << error;
    register_script_engine(engine);
    write("guest.wat", kGuest);
  }
  void TearDown() override { unregister_script_engine("wasm"); }

  // Run `program` as an action of `art`'s document, draining until it ends: "" when it finished,
  // else why not. `drains` says how many drains it took.
  static std::string run(AppRuntime &art, const std::string &program, int *drains = nullptr) {
    bool ended = false;
    std::string how;
    art.run_program(program, [&](const std::string &h) {
      ended = true;
      how = h;
    });
    int n = 0;
    while (!ended && n < 200) {
      art.drain();
      ++n;
    }
    if (drains)
      *drains = n;
    return ended ? how : "never ended";
  }

  std::shared_ptr<ScriptEngine> engine;
};

} // namespace

// ── the codec ──────────────────────────────────────────────────────────────────────────────────

TEST(WasmCbor, ValuesRoundTrip) {
  const se::value_t v =
      se::make_dict({{"n", se::value_t(std::int64_t(-1000000))},
                     {"big", se::value_t(std::int64_t(1) << 40)},
                     {"f", se::value_t(2.5)},
                     {"s", se::value_t(std::string("route"))},
                     {"b", se::make_bytes(std::string("\x00\x01\xff", 3))},
                     {"l", se::make_list({se::value_t(true), se::value_t(false), se::value_t()})}});
  const std::string c = wasm::to_cbor(v);
  const se::value_t back = wasm::from_cbor(c);
  EXPECT_EQ(wasm::to_cbor(back), c);
  const auto *d = std::get_if<se::dict_ptr>(&back.v);
  ASSERT_TRUE(d && *d);
  ASSERT_EQ((*d)->size(), 6u);
  EXPECT_EQ((**d)[0].first, "n") << "a map keeps its order";
  EXPECT_EQ(std::get<std::int64_t>((**d)[0].second.v), -1000000);
  EXPECT_EQ(std::get<double>((**d)[2].second.v), 2.5);
  EXPECT_EQ(std::get<se::bytes_value>((**d)[4].second.v).data, std::string("\x00\x01\xff", 3));

  EXPECT_EQ(wasm::to_cbor(se::value_t(std::int64_t(7))), std::string("\x07"));
  EXPECT_EQ(wasm::to_cbor(se::value_t(std::string("x"))), std::string("\x61x"));
  EXPECT_EQ(std::get<double>(wasm::from_cbor(std::string("\xf9\x3e\x00", 3)).v), 1.5)
      << "a float16";
  EXPECT_EQ(std::get<std::int64_t>(wasm::from_cbor(std::string("\xc1\x18\x2a", 3)).v), 42)
      << "a tag is read through";
}

// A host object crosses as a handle under the handle tag, given a table, and comes back as itself.
TEST(WasmCbor, HostObjectsCrossAsHandles) {
  struct Table : wasm::Handles {
    std::vector<se::data_object_ptr> objs;
    std::uint64_t put(const se::data_object_ptr &o) override {
      objs.push_back(o);
      return objs.size();
    }
    se::data_object_ptr get(std::uint64_t h) const override {
      return h && h <= objs.size() ? objs[h - 1] : nullptr;
    }
  } table;
  auto obj = std::make_shared<se::data_object>();
  obj->type_name = "thing";
  const se::value_t v = se::make_list({se::value_t(obj)});
  EXPECT_THROW(wasm::to_cbor(v), std::invalid_argument) << "without a table it doesn't cross";
  const std::string c = wasm::to_cbor(v, &table);
  EXPECT_EQ(c, std::string("\x81\xda\x61\x72\x69\x68\x01", 7));
  const se::value_t back = wasm::from_cbor(c, &table);
  EXPECT_EQ(std::get<se::data_object_ptr>((*std::get<se::list_ptr>(back.v))[0].v), obj);
  EXPECT_THROW(wasm::from_cbor(c), std::invalid_argument) << "a handle without a table";
  EXPECT_THROW(wasm::from_cbor(std::string("\xda\x61\x72\x69\x68\x02", 6), &table),
               std::invalid_argument)
      << "a handle that names nothing";
}

TEST(WasmCbor, WhatDoesNotCrossIsRefused) {
  EXPECT_THROW(wasm::to_cbor(se::value_t(
                   se::native_fn([](std::span<const se::value_t>) -> se::value_t { return {}; }))),
               std::invalid_argument);
  EXPECT_THROW(wasm::from_cbor(std::string("\x82\x01", 2)), std::invalid_argument) << "truncated";
  EXPECT_THROW(wasm::from_cbor(std::string("\x01\x02", 2)), std::invalid_argument)
      << "trailing bytes";
  EXPECT_THROW(wasm::from_cbor(std::string("\x9f\xff", 2)), std::invalid_argument)
      << "an indefinite length";
  EXPECT_THROW(wasm::from_cbor(std::string("\xa1\x01\x02", 3)), std::invalid_argument)
      << "a map key that isn't text";
  EXPECT_THROW(wasm::from_cbor(std::string("\x1b\xff\xff\xff\xff\xff\xff\xff\xff", 9)),
               std::invalid_argument)
      << "past int64";
}

// ── the engine ─────────────────────────────────────────────────────────────────────────────────

// The block's exports are functions of the document: a call parks the program on the guest's
// activation and returns its result. _initialize ran once; wasm is a token that holds.
TEST_F(WasmEngineTest, ExportsAreFunctionsOfTheDocument) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  std::vector<std::string> warns;
  ASSERT_NO_THROW(warns = art.load(write("calls.ari", doc()).string()));
  EXPECT_EQ(run(art, "(assert (= (echo 1 \"a\") (list 1 \"a\")) \"echo\")\n"
                     "(assert (= (nothing) nil) \"nil\")\n"
                     "(assert (= (inits) 1) \"_initialize ran once\")\n"
                     "(assert (has? \"wasm\") \"the token\")\n"
                     "(let ((f (echo-async 3))) (assert (= (await f) (list 3)) \"a future\"))"),
            "");
}

// ari_init runs at load, through the host, before the document's init:; and init:, which can't
// wait, calls an export that runs to its end there.
TEST_F(WasmEngineTest, AriInitAndInitCallsRunAtLoad) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  std::vector<std::string> warns;
  ASSERT_NO_THROW(warns =
                      art.load(write("boot.ari", "meta: {min_libcvc: 3.7.13}\n"
                                                 "requires: [wasm]\n"
                                                 "wasm:\n"
                                                 "  module: guest.wat\n"
                                                 "  export: [get-x]\n"
                                                 "init: |\n"
                                                 "  (state-set \"seen\" (state-get \"booted\"))\n"
                                                 "  (state-set \"x\" \"early\")\n"
                                                 "  (state-set \"got\" (get-x))\n"
                                                 "root:\n"
                                                 "  - text: hi\n")
                                   .string()));
  for (const std::string &w : warns)
    ADD_FAILURE() << w;
  EXPECT_EQ(val(app, "ui.wa.booted"), "yes");
  EXPECT_EQ(val(app, "ui.wa.seen"), "yes") << "ari_init ran before init:";
  EXPECT_EQ(val(app, "ui.wa.got"), "early") << "init: called the guest";
}

// A guest's call reaches the lane's functions in the calling program's process: the state
// natives under the document's prefix. A call that waits parks the process, not the thread.
TEST_F(WasmEngineTest, CallsReachTheLaneUnderTheChroot) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("lane.ari", doc()).string()));
  EXPECT_EQ(run(art, "(state-set \"x\" \"hi\")\n"
                     "(assert (= (get-x) \"hi\") \"state-get\")\n"
                     "(set-y)"),
            "");
  EXPECT_EQ(val(app, "ui.wa.y"), "ok") << "written under the document's prefix";
}

// imports: narrows what a guest's calls reach: state.read lets state-get through and refuses
// state-set as a PermissionError, which the guest here fails with.
TEST_F(WasmEngineTest, ImportsNarrowTheCalls) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("narrow.ari", doc("  imports: [state.read]\n")).string()));
  EXPECT_EQ(run(art, "(state-set \"x\" \"read\") (assert (= (get-x) \"read\") \"allowed\")"), "");
  EXPECT_EQ(run(art, "(state-set \"m\" (try (set-y) (catch (WasmError e) (get-attr e "
                     "\"message\"))))"),
            "");
  EXPECT_NE(val(app, "ui.wa.m").find("PermissionError"), std::string::npos) << val(app, "ui.wa.m");
}

// A guest runs in slices of fuel, charged to the calling program as DSL steps: with no step cap a
// long loop finishes, and under the action's 5000 it is stopped there.
TEST_F(WasmEngineTest, FuelIsChargedAsSteps) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("fuel.ari", doc()).string()));
  auto &st = cvc::state::instance(app);
  st("ariadne.limits.action_max_time").value(std::string("10"));
  st("ariadne.limits.action_max_steps").value(std::string("0"));
  EXPECT_EQ(run(art, "(spin)"), "") << "no cap: the loop finishes, a slice at a time";
  st("ariadne.limits.action_max_steps").value(std::string("5000"));
  const std::string capped = run(art, "(spin)");
  EXPECT_NE(capped.find("max_steps_exceeded"), std::string::npos) << capped;
}

// yield() gives the frame back: (nap) yields twice and returns in a later frame.
TEST_F(WasmEngineTest, AYieldGivesTheFrameBack) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("nap.ari", doc()).string()));
  int drains = 0;
  EXPECT_EQ(run(art, "(assert (= (nap) 7) \"nap\")", &drains), "");
  EXPECT_GE(drains, 3);
}

// A trap and fail() are WasmErrors a program can catch; a guest calling back into its own export
// is refused with a RecursionError, rather than waiting on itself, and fails with its text.
TEST_F(WasmEngineTest, TrapsAndFailuresAreWasmErrors) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("errors.ari", doc()).string()));
  EXPECT_EQ(run(art, "(state-set \"t\" (try (boom) (catch (WasmError e) (get-attr e \"trap\"))))\n"
                     "(state-set \"r\" (try (refuse) (catch (WasmError e) (get-attr e "
                     "\"message\"))))\n"
                     "(state-set \"s\" (try (self) (catch (WasmError e) (get-attr e "
                     "\"message\"))))"),
            "");
  EXPECT_NE(val(app, "ui.wa.t").find("unreachable"), std::string::npos) << val(app, "ui.wa.t");
  EXPECT_NE(val(app, "ui.wa.r").find("no route"), std::string::npos) << val(app, "ui.wa.r");
  EXPECT_NE(val(app, "ui.wa.s").find("RecursionError"), std::string::npos) << val(app, "ui.wa.s");
  EXPECT_NE(val(app, "ui.wa.s").find("re-enter"), std::string::npos) << val(app, "ui.wa.s");
  const std::string uncaught = run(art, "(boom)");
  EXPECT_NE(uncaught.find("WasmError"), std::string::npos) << uncaught;
}

// One activation per instance: two programs calling it at once both finish, one after the other.
TEST_F(WasmEngineTest, OneActivationAtATimePerInstance) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("two.ari", doc()).string()));
  int ended = 0;
  std::vector<std::string> how;
  for (int i = 0; i < 2; ++i)
    art.run_program("(assert (= (nap) 7) \"nap\")", [&](const std::string &h) {
      ++ended;
      how.push_back(h);
    });
  for (int n = 0; ended < 2 && n < 100; ++n)
    art.drain();
  ASSERT_EQ(ended, 2);
  EXPECT_EQ(how[0], "");
  EXPECT_EQ(how[1], "");
}

// The instance's lock is a with-lock lock: lock-query shows it held while a call naps, and a
// second call waits for it under the action's lock timeout (ariadne.exec.lock_timeout; 0 never
// waits), so here it times out.
TEST_F(WasmEngineTest, ACallWaitsForTheInstanceAsWithLockWaits) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("wait.ari", doc()).string()));
  cvc::state::instance(app)("ariadne.exec.lock_timeout").value(std::string("0"));
  int ended = 0;
  std::vector<std::string> how;
  const auto done = [&](const std::string &h) {
    ++ended;
    how.push_back(h);
  };
  art.run_program("(assert (= (nap) 7) \"nap\")", done);
  art.run_program("(await #f)\n" // the next frame: the nap holds the instance
                  "(state-set \"k\" (get-attr (car (get-attr (lock-query) \"held\")) \"key\"))\n"
                  "(state-set \"b\" (try (nap) (catch (TimeoutError e) \"timed out\")))",
                  done);
  for (int n = 0; ended < 2 && n < 100; ++n)
    art.drain();
  ASSERT_EQ(ended, 2);
  EXPECT_EQ(how[0], "");
  EXPECT_EQ(how[1], "");
  EXPECT_EQ(val(app, "ui.wa.k"), "wasm:guest");
  EXPECT_EQ(val(app, "ui.wa.b"), "timed out");
}

// A call back into the guest from inside one of its host calls, through a document function, is
// a RecursionError the function can catch, with `running` the export in flight; the guest gets it
// back as that call's value. So it is whether the call is the program's own or an activation of
// its own (-async), and the instance goes on working after.
TEST_F(WasmEngineTest, ACallBackIntoTheGuestIsARecursionError) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("bounce.ari", "requires: [wasm]\n"
                                               "functions: |\n"
                                               "  (defun bounce ()\n"
                                               "    (try (echo 1)\n"
                                               "      (catch (RecursionError e)\n"
                                               "        (list \"refused\" (get-attr e "
                                               "\"running\")))))\n"
                                               "wasm:\n"
                                               "  module: guest.wat\n"
                                               "  export: " +
                                                   std::string(kExports) +
                                                   "\n"
                                                   "root:\n"
                                                   "  - text: hi\n")
                               .string()));
  EXPECT_EQ(run(art, "(assert (= (relay) (list \"refused\" \"relay\")) \"the program's own\")\n"
                     "(assert (= (await (relay-async)) (list \"refused\" \"relay\")) \"-async\")\n"
                     "(assert (= (echo 2) (list 2)) \"it still works\")"),
            "");
}

// hooks: on_tick delivers every frame to the guest's export, which calls the host back.
TEST_F(WasmEngineTest, AFrameHookCallsTheGuestEachFrame) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  int pings = 0;
  art.register_function("test-ping", [&](std::span<const se::value_t>) -> se::value_t {
    ++pings;
    return se::value_t();
  });
  ASSERT_NO_THROW(art.load(write("tick.ari", doc("  hooks: {on_tick: ping}\n")).string()));
  for (int i = 0; i < 6; ++i)
    art.drain();
  EXPECT_GE(pings, 4);
}

// What fails the load: no such module, a module without a named export, text that doesn't
// compile; an unknown key only warns.
TEST_F(WasmEngineTest, TheBlocksMistakesFailTheLoad) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  const auto fails = [&](const std::string &name, const std::string &text) -> std::string {
    try {
      art.load(write(name, text).string());
    } catch (const std::exception &e) {
      return e.what();
    }
    return "loaded";
  };
  EXPECT_NE(fails("missing.ari", "wasm: no-such.wasm\nroot:\n  - text: hi\n").find("can't read"),
            std::string::npos);
  EXPECT_NE(fails("noexport.ari", "wasm:\n  module: guest.wat\n  export: [absent]\n"
                                  "root:\n  - text: hi\n")
                .find("exports no function 'absent'"),
            std::string::npos);
  write("bad.wat", "(module (func (export \"f\") (i32.const)))");
  EXPECT_NE(fails("bad.ari", "wasm: bad.wat\nroot:\n  - text: hi\n").find("bad.wat"),
            std::string::npos);
  std::vector<std::string> warns;
  ASSERT_NO_THROW(warns = art.load(write("unknown.ari", "wasm:\n  module: guest.wat\n"
                                                        "  colour: blue\nroot:\n  - text: hi\n")
                                       .string()));
  bool warned = false;
  for (const std::string &w : warns)
    warned = warned || w.find("unknown key 'colour'") != std::string::npos;
  EXPECT_TRUE(warned);
}

// ── host objects by handle ─────────────────────────────────────────────────────────────────────

namespace {

// A guest over host objects (§8.4.1). An export's first argument is a handle (its CBOR: the
// array's head, the tag's five bytes, then 0x1b and the handle's eight, big-endian), and the
// rest are small integers of a byte each.
const char kDataGuest[] = R"WAT(
(module
  (import "ari" "call" (func $call (param i32 i32 i32 i32) (result i64)))
  (import "ari" "reply" (func $reply (param i32)))
  (import "ari" "fail" (func $fail (param i32 i32)))
  (import "ari" "handle_drop" (func $drop (param i64)))
  (import "ari" "data_info" (func $info (param i64) (result i64)))
  (import "ari" "volume_read"
    (func $vread (param i64 i32 i32 i32 i32 i32 i32 i32 i32 i32) (result i64)))
  (import "ari" "volume_sample" (func $vsample (param i64 i32 i32 i32 i32) (result i64)))
  (import "ari" "mesh_read" (func $mread (param i64 i32 i32 i32 i32 i32 i32) (result i64)))
  (import "ari" "heightfield_read" (func $hread (param i64 i32 i32 i32 i32 i32 i32) (result i64)))
  (memory (export "memory") 1)
  (global $heap (mut i32) (i32.const 4096))
  (func $alloc (export "ari_alloc") (param $n i32) (result i32) (local $p i32)
    (local.set $p (global.get $heap))
    (global.set $heap (i32.add (global.get $heap) (local.get $n)))
    (local.get $p))
  (func $pack (param $p i32) (param $n i32) (result i64)
    (i64.or (i64.shl (i64.extend_i32_u (local.get $p)) (i64.const 32))
            (i64.extend_i32_u (local.get $n))))
  (func $h (param $p i32) (result i64) (local $i i32) (local $v i64)
    (loop $l
      (local.set $v (i64.or (i64.shl (local.get $v) (i64.const 8))
        (i64.load8_u (i32.add (local.get $p) (i32.add (i32.const 7) (local.get $i))))))
      (local.set $i (i32.add (local.get $i) (i32.const 1)))
      (br_if $l (i32.lt_u (local.get $i) (i32.const 8))))
    (local.get $v))
  (func $arg (param $p i32) (param $k i32) (result i32)
    (i32.load8_u (i32.add (local.get $p) (i32.add (i32.const 15) (local.get $k)))))
  ;; A data call's result; fail with its error's text when it is < 0.
  (func $ok (param $r i64) (result i32) (local $dst i32)
    (if (i64.lt_s (local.get $r) (i64.const 0))
      (then (local.set $dst (global.get $heap))
            (call $reply (local.get $dst))
            (call $fail (local.get $dst) (i32.wrap_i64 (i64.sub (i64.const 0) (local.get $r))))
            (unreachable)))
    (i32.wrap_i64 (local.get $r)))
  ;; CBOR unsigned n (< 65536) at p; the bytes it took.
  (func $uint (param $p i32) (param $n i32) (result i32)
    (if (i32.lt_u (local.get $n) (i32.const 24))
      (then (i32.store8 (local.get $p) (local.get $n)) (return (i32.const 1))))
    (if (i32.lt_u (local.get $n) (i32.const 256))
      (then (i32.store8 (local.get $p) (i32.const 0x18))
            (i32.store8 offset=1 (local.get $p) (local.get $n))
            (return (i32.const 2))))
    (i32.store8 (local.get $p) (i32.const 0x19))
    (i32.store8 offset=1 (local.get $p) (i32.shr_u (local.get $n) (i32.const 8)))
    (i32.store8 offset=2 (local.get $p) (local.get $n))
    (i32.const 3))
  ;; The list (a b).
  (func $two (param $a i32) (param $b i32) (result i64) (local $p i32) (local $n i32)
    (local.set $p (global.get $heap))
    (i32.store8 (local.get $p) (i32.const 0x82))
    (local.set $n (i32.add (i32.const 1)
      (call $uint (i32.add (local.get $p) (i32.const 1)) (local.get $a))))
    (local.set $n (i32.add (local.get $n)
      (call $uint (i32.add (local.get $p) (local.get $n)) (local.get $b))))
    (global.set $heap (i32.add (local.get $p) (local.get $n)))
    (call $pack (local.get $p) (local.get $n)))
  (func (export "echo") (param $p i32) (param $n i32) (result i64)
    (call $pack (local.get $p) (local.get $n)))
  ;; (info h): data_info's map.
  (func (export "info") (param $p i32) (param $n i32) (result i64) (local $r i32) (local $dst i32)
    (local.set $r (call $ok (call $info (call $h (local.get $p)))))
    (local.set $dst (global.get $heap))
    (call $reply (local.get $dst))
    (global.set $heap (i32.add (local.get $dst) (local.get $r)))
    (call $pack (local.get $dst) (local.get $r)))
  ;; (vread h x y z nx ny nz lod): the voxels (unsigned chars), as a list.
  (func (export "vread") (param $p i32) (param $n i32) (result i64)
    (local $dst i32) (local $r i32) (local $i i32) (local $out i32) (local $at i32)
    (local.set $dst (global.get $heap))
    (local.set $r (call $ok (call $vread (call $h (local.get $p))
      (call $arg (local.get $p) (i32.const 0)) (call $arg (local.get $p) (i32.const 1))
      (call $arg (local.get $p) (i32.const 2)) (call $arg (local.get $p) (i32.const 3))
      (call $arg (local.get $p) (i32.const 4)) (call $arg (local.get $p) (i32.const 5))
      (call $arg (local.get $p) (i32.const 6)) (local.get $dst) (i32.const 64))))
    (local.set $out (i32.add (local.get $dst) (local.get $r)))
    (i32.store8 (local.get $out) (i32.or (i32.const 0x80) (local.get $r)))
    (local.set $at (i32.const 1))
    (block $done
      (loop $l
        (br_if $done (i32.ge_u (local.get $i) (local.get $r)))
        (local.set $at (i32.add (local.get $at)
          (call $uint (i32.add (local.get $out) (local.get $at))
                      (i32.load8_u (i32.add (local.get $dst) (local.get $i))))))
        (local.set $i (i32.add (local.get $i) (i32.const 1)))
        (br $l)))
    (global.set $heap (i32.add (local.get $out) (local.get $at)))
    (call $pack (local.get $out) (local.get $at)))
  ;; (mread h attr first count): (bytes, the sum of the u32s) for attr 1, indices; (bytes, 0) for
  ;; attr 0, positions.
  (data (i32.const 16) "positions")
  (data (i32.const 32) "indices")
  (func (export "mread") (param $p i32) (param $n i32) (result i64)
    (local $dst i32) (local $r i32) (local $i i32) (local $sum i32) (local $ap i32) (local $al i32)
    (if (call $arg (local.get $p) (i32.const 0))
      (then (local.set $ap (i32.const 32)) (local.set $al (i32.const 7)))
      (else (local.set $ap (i32.const 16)) (local.set $al (i32.const 9))))
    (local.set $dst (global.get $heap))
    (local.set $r (call $ok (call $mread (call $h (local.get $p)) (local.get $ap) (local.get $al)
      (call $arg (local.get $p) (i32.const 1)) (call $arg (local.get $p) (i32.const 2))
      (local.get $dst) (i32.const 1024))))
    (global.set $heap (i32.add (local.get $dst) (local.get $r)))
    (block $done
      (loop $l
        (br_if $done (i32.ge_u (local.get $i) (local.get $r)))
        (local.set $sum (i32.add (local.get $sum) (i32.load (i32.add (local.get $dst) (local.get $i)))))
        (local.set $i (i32.add (local.get $i) (i32.const 4)))
        (br $l)))
    (call $two (local.get $r)
      (select (local.get $sum) (i32.const 0) (call $arg (local.get $p) (i32.const 0)))))
  ;; (hread h row col rows cols): (bytes, the first height).
  (func (export "hread") (param $p i32) (param $n i32) (result i64) (local $dst i32) (local $r i32)
    (local.set $dst (global.get $heap))
    (local.set $r (call $ok (call $hread (call $h (local.get $p))
      (call $arg (local.get $p) (i32.const 0)) (call $arg (local.get $p) (i32.const 1))
      (call $arg (local.get $p) (i32.const 2)) (call $arg (local.get $p) (i32.const 3))
      (local.get $dst) (i32.const 1024))))
    (global.set $heap (i32.add (local.get $dst) (local.get $r)))
    (call $two (local.get $r) (i32.trunc_f32_u (f32.load (local.get $dst)))))
  ;; (sample h): (bytes, the value at (1, 0, 0), rounded).
  (data (i32.const 48) "\00\00\00\00\00\00\f0\3f\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00\00")
  (func (export "sample") (param $p i32) (param $n i32) (result i64) (local $dst i32) (local $r i32)
    (local.set $dst (global.get $heap))
    (local.set $r (call $ok (call $vsample (call $h (local.get $p)) (i32.const 48) (i32.const 1)
      (local.get $dst) (i32.const 8))))
    (global.set $heap (i32.add (local.get $dst) (local.get $r)))
    (call $two (local.get $r)
      (i32.trunc_f64_u (f64.add (f64.load (local.get $dst)) (f64.const 0.5)))))
  ;; (dropped h): drop the handle, then ask about it.
  (func (export "dropped") (param $p i32) (param $n i32) (result i64)
    (call $drop (call $h (local.get $p)))
    (drop (call $ok (call $info (call $h (local.get $p)))))
    (i64.const 0))
  ;; (fetch): (state-data-get "vol") through the host, its handle back as the result.
  (data (i32.const 96) "state-data-get")
  (data (i32.const 112) "\81\63vol")
  (func (export "fetch") (param i32 i32) (result i64) (local $r i32) (local $dst i32)
    (local.set $r (call $ok (call $call (i32.const 96) (i32.const 14) (i32.const 112) (i32.const 5))))
    (local.set $dst (global.get $heap))
    (call $reply (local.get $dst))
    (global.set $heap (i32.add (local.get $dst) (local.get $r)))
    (call $pack (local.get $dst) (local.get $r)))
  ;; (hog): grow the memory until a grow is refused, then give up.
  (func (export "hog") (param i32 i32) (result i64)
    (loop $l (br_if $l (i32.ne (memory.grow (i32.const 1)) (i32.const -1))))
    (unreachable))
)
)WAT";

std::string data_doc(const std::string &extra = std::string()) {
  return "requires: [wasm]\n"
         "wasm:\n"
         "  module: data.wat\n"
         "  export: [echo, info, vread, mread, hread, sample, dropped, fetch, hog]\n" +
         extra + "root:\n  - text: hi\n";
}

// A 3x2x2 volume of unsigned chars 1..12, x fastest, its bounds its voxel indices.
std::shared_ptr<const cvc::volume> small_volume(cvc::app &app) {
  std::vector<unsigned char> v(12);
  for (int i = 0; i < 12; ++i)
    v[i] = static_cast<unsigned char>(i + 1);
  auto vol = std::make_shared<cvc::volume>(app, v.data(), cvc::dimension(3, 2, 2), cvc::UChar);
  vol->boundingBox() = cvc::bounding_box(0, 0, 0, 2, 1, 1);
  return vol;
}

// What a program stored at `path` with state-data-set.
se::value_t data_at(cvc::app &app, const std::string &path) {
  const boost::any d = cvc::state::instance(app)(path).data();
  const auto *v = boost::any_cast<se::value_t>(&d);
  return v ? *v : se::value_t();
}

se::value_t entry(const se::value_t &dict, const std::string &key) {
  if (const auto *d = std::get_if<se::dict_ptr>(&dict.v); d && *d)
    for (const auto &[k, v] : **d)
      if (k == key)
        return v;
  return se::value_t();
}

std::int64_t int_of(const se::value_t &v) {
  const auto *i = std::get_if<std::int64_t>(&v.v);
  return i ? *i : -1;
}

std::string text_of(const se::value_t &v) {
  const auto *s = std::get_if<std::string>(&v.v);
  return s ? *s : std::string();
}

// The volume a stored value's host object holds, if it holds one.
const cvc::volume *volume_in(const se::value_t &v) {
  const auto *o = std::get_if<se::data_object_ptr>(&v.v);
  if (!o || !*o)
    return nullptr;
  const auto *p = boost::any_cast<std::shared_ptr<const cvc::volume>>(&(*o)->payload);
  return p ? p->get() : nullptr;
}

class WasmDataTest : public WasmEngineTest {
protected:
  void SetUp() override {
    WasmEngineTest::SetUp();
    write("data.wat", kDataGuest);
  }
};

} // namespace

// A volume crosses as a handle: data_info describes it, volume_read copies a box (or every other
// voxel), volume_sample a value at a point, and the handle comes back as the same object, from an
// argument or from the guest's own state-data-get.
TEST_F(WasmDataTest, AVolumeCrossesAsAHandle) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("vol.ari", data_doc()).string()));
  const std::shared_ptr<const cvc::volume> vol = small_volume(app);
  cvc::state::instance(app)("ui.wa.vol").data(boost::any(vol));
  EXPECT_EQ(run(art, "(let ((v (state-data-get \"vol\")))\n"
                     "  (state-data-set \"info\" (info v))\n"
                     "  (assert (= (vread v 0 0 0 3 2 2 0) (list 1 2 3 4 5 6 7 8 9 10 11 12)) "
                     "\"all of it\")\n"
                     "  (assert (= (vread v 1 1 0 2 1 1 0) (list 5 6)) \"a box\")\n"
                     "  (assert (= (vread v 0 0 0 3 2 2 1) (list 1 3)) \"every other voxel\")\n"
                     "  (assert (= (sample v) (list 8 2)) \"a value at a point\")\n"
                     "  (state-data-set \"back\" (car (echo v)))\n"
                     "  (state-data-set \"fetched\" (fetch)))"),
            "");
  const se::value_t info = data_at(app, "ui.wa.info");
  EXPECT_EQ(text_of(entry(info, "type")), "volume");
  EXPECT_EQ(text_of(entry(info, "voxel")), "unsigned char");
  EXPECT_EQ(int_of(entry(info, "voxel_bytes")), 1);
  const se::value_t dims = entry(info, "dims");
  ASSERT_TRUE(std::holds_alternative<se::list_ptr>(dims.v));
  const auto &d = *std::get<se::list_ptr>(dims.v);
  ASSERT_EQ(d.size(), 3u);
  EXPECT_EQ(int_of(d[0]) * 100 + int_of(d[1]) * 10 + int_of(d[2]), 322);
  EXPECT_EQ(volume_in(data_at(app, "ui.wa.back")), vol.get()) << "the handle came back as itself";
  EXPECT_EQ(volume_in(data_at(app, "ui.wa.fetched")), vol.get()) << "a handle from a call";
}

// A mesh's indices and positions, and a height field's heights, a range at a time.
TEST_F(WasmDataTest, AMeshAndAHeightField) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("mesh.ari", data_doc()).string()));
  auto mesh = std::make_shared<cvc::geometry>();
  mesh->points().push_back(cvc::geometry::point_t{{0.0, 0.0, 0.0}});
  mesh->points().push_back(cvc::geometry::point_t{{1.0, 0.0, 0.0}});
  mesh->points().push_back(cvc::geometry::point_t{{0.0, 1.0, 0.0}});
  mesh->tris().push_back(cvc::geometry::tri_t{{0, 1, 2}});
  cvc::state::instance(app)("ui.wa.mesh")
      .data(boost::any(std::shared_ptr<const cvc::geometry>(mesh)));
  cvc::state::instance(app)("ui.wa.terrain")
      .data(boost::any(std::make_shared<const cvc::height_field>(
          2, 3, 0.0, 0.0, 1.0, 1.0, std::vector<float>{5, 6, 7, 8, 9, 10})));
  EXPECT_EQ(run(art, "(let ((m (state-data-get \"mesh\")) (h (state-data-get \"terrain\")))\n"
                     "  (assert (= (mread m 1 0 1) (list 12 3)) \"the triangle's indices\")\n"
                     "  (assert (= (car (mread m 0 0 3)) 72) \"three positions\")\n"
                     "  (assert (= (hread h 1 1 1 2) (list 8 9)) \"two heights\")\n"
                     "  (state-data-set \"minfo\" (info m))\n"
                     "  (state-data-set \"hinfo\" (info h)))"),
            "");
  const se::value_t m = data_at(app, "ui.wa.minfo");
  EXPECT_EQ(text_of(entry(m, "type")), "mesh");
  EXPECT_EQ(int_of(entry(m, "points")), 3);
  EXPECT_EQ(int_of(entry(m, "tris")), 1);
  const se::value_t h = data_at(app, "ui.wa.hinfo");
  EXPECT_EQ(text_of(entry(h, "type")), "heightfield");
  EXPECT_EQ(int_of(entry(h, "rows")), 2);
  EXPECT_EQ(int_of(entry(h, "cols")), 3);
}

// A dropped handle names nothing, and a region outside the object is refused; the guest fails
// with the error's text.
TEST_F(WasmDataTest, ADroppedHandleNamesNothing) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("drop.ari", data_doc()).string()));
  cvc::state::instance(app)("ui.wa.vol").data(boost::any(small_volume(app)));
  EXPECT_EQ(run(art, "(let ((v (state-data-get \"vol\")))\n"
                     "  (state-set \"gone\" (try (dropped v) (catch (WasmError e) "
                     "(get-attr e \"message\"))))\n"
                     "  (state-set \"out\" (try (vread v 2 0 0 2 1 1 0) (catch (WasmError e) "
                     "(get-attr e \"message\")))))"),
            "");
  EXPECT_NE(val(app, "ui.wa.gone").find("names nothing"), std::string::npos)
      << val(app, "ui.wa.gone");
  EXPECT_NE(val(app, "ui.wa.out").find("isn't inside"), std::string::npos) << val(app, "ui.wa.out");
}

// imports: without data.read refuses the data calls as a PermissionError; with it they go.
TEST_F(WasmDataTest, ImportsNarrowTheDataCalls) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime narrow(app, "ui.wa");
  narrow.use_script_engine(engine);
  ASSERT_NO_THROW(narrow.load(write("narrow.ari", data_doc("  imports: [state.read]\n")).string()));
  cvc::state::instance(app)("ui.wa.vol").data(boost::any(small_volume(app)));
  EXPECT_EQ(run(narrow, "(state-set \"m\" (try (info (state-data-get \"vol\")) "
                        "(catch (WasmError e) (get-attr e \"message\"))))"),
            "");
  EXPECT_NE(val(app, "ui.wa.m").find("PermissionError"), std::string::npos) << val(app, "ui.wa.m");

  AppRuntime wide(app, "ui.wb");
  wide.use_script_engine(engine);
  ASSERT_NO_THROW(
      wide.load(write("wide.ari", data_doc("  imports: [state.read, data.read]\n")).string()));
  cvc::state::instance(app)("ui.wb.vol").data(boost::any(small_volume(app)));
  EXPECT_EQ(run(wide, "(assert (= (vread (state-data-get \"vol\") 1 1 0 2 1 1 0) (list 5 6)) "
                      "\"data.read\")"),
            "");
}

// A guest whose memory.grow is refused at its limit, and which then traps, is out of memory: a
// MemoryError, not a WasmError.
TEST_F(WasmDataTest, AGuestOutOfMemoryIsAMemoryError) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_NO_THROW(art.load(write("hog.ari", data_doc("  limits: {memory_mb: 1}\n")).string()));
  EXPECT_EQ(run(art, "(state-set \"r\" (try (hog) (catch (MemoryError e) (get-attr e "
                     "\"message\")) (catch (WasmError e) \"a WasmError\")))"),
            "");
  EXPECT_NE(val(app, "ui.wa.r").find("out of memory"), std::string::npos) << val(app, "ui.wa.r");
}
