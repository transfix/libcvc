// WebAssembly guests in documents (src/ariadne_wasm; docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md
// §8.2-8.7, phase G1), on wasmtime: the CBOR codec; a wasm: block whose WAT guest's exports the
// document's programs call (each call an activation, parked on by the caller); calls back into
// the lane under the document's chroot, and their narrowing; slices of fuel charged as steps
// under the action caps; a yield; traps and fail() as WasmError; _initialize; a refused re-entry;
// one activation per instance; a frame hook; and the block's load errors.

#include "value_cbor.h"
#include "wasm_engine.h"

#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/requires.h>
#include <cvc/ariadne/script_engine.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
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
)
)WAT";

const char kExports[] = "[echo, nothing, spin, get-x, set-y, nap, boom, refuse, inits, self]";

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

// A guest's call reaches the lane's functions in the activation's process: the state natives
// under the document's prefix. A call that waits parks the process, not the thread.
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

// A guest runs in slices of fuel, charged to its activation as DSL steps: with no step cap a long
// loop finishes, and under the action's 5000 it is stopped there.
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
// is refused, rather than waiting on itself.
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
