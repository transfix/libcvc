// WebAssembly guests transformed for the browser (src/ariadne_wasm; docs/roadmap/
// ARIADNE_SCRIPTING_HOSTS.md §8.5, G2's first part). One file, both backends: under Emscripten
// (node; plain wasm, no pthreads) on the page's own WebAssembly (browser_runtime.cpp), where a
// guest pauses by Asyncify at a gas check or at an import that waits, as ariadne_wasm_browser_test;
// natively on wasmtime, which runs the same transformed guests, as ariadne_wasm_transform_test.
// The guests are tests/guest.wat and tests/page_guest.wat through cvc_ariadne_wasm_transform
// (CVC_WASM_GUESTS_DIR; without wasm-opt the WasmTransformed cases skip). Here: exports as the
// document's functions; calls that park the guest and resume it; a yield; a spin sliced at its
// gas checks and charged as steps, and killed by a step cap; recursions paused deep in the stack;
// traps and fail() as WasmErrors; state imports resumed in the slice; a memory.grow refused at
// the block's limit as a MemoryError; a volume read and sampled into the guest's memory; a frame
// hook. Also the memory cap's rewrite of a module's binary (wasm_binary.h), and what the browser
// refuses: a .wat module, an untransformed guest.

#include "wasm_binary.h"
#include "wasm_engine.h"

#include <boost/any.hpp>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/script_engine.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/volume/volume.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <initializer_list>
#include <memory>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <process.h>
#define CVC_GETPID() ::_getpid()
#else
#include <unistd.h>
#define CVC_GETPID() ::getpid()
#endif

using namespace cvc::ariadne;
namespace se = cvc::state_exec;
namespace fs = std::filesystem;

namespace {

#define REQUIRE_YAML()                                                                             \
  do {                                                                                             \
    if (!AppRuntime::have_yaml())                                                                  \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

#ifdef CVC_WASM_GUESTS_DIR
#define REQUIRE_GUESTS()                                                                           \
  do {                                                                                             \
  } while (0)
#else
#define REQUIRE_GUESTS() GTEST_SKIP() << "built without wasm-opt (CVC_WASM_OPT): no guests"
#endif

// This process's own directory, as ariadne_wasm_test's: ctest runs each case as its own process,
// several at once (under Emscripten, MEMFS's /tmp).
fs::path dir() {
  const fs::path d =
      fs::temp_directory_path() /
      ("ariadne_wasm_browser_test-" + std::to_string(static_cast<long long>(CVC_GETPID())));
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

std::string bytes(std::initializer_list<int> b) {
  std::string s;
  for (const int c : b)
    s.push_back(static_cast<char>(c));
  return s;
}

// A name in the binary format: its length, then its bytes.
std::string name(const std::string &n) { return bytes({static_cast<int>(n.size())}) + n; }

const std::string kHeader("\0asm\x01\0\0\0", 8);

// A module: the header, a type section (one type, () -> ()), `memories` (a memory section), and an
// export of memory 0 as "memory".
std::string module_with(const std::string &memories) {
  return kHeader + bytes({0x01, 0x04, 0x01, 0x60, 0x00, 0x00}) + memories +
         bytes({0x07, 0x0a, 0x01}) + name("memory") + bytes({0x02, 0x00});
}

const char kExports[] =
    "[echo, nothing, spin, get-x, set-y, ping, nap, boom, refuse, inits, self, relay]";

// A document over the transformed guest.wat.
std::string doc(const std::string &wasm_extra = std::string()) {
  return "requires: [wasm]\n"
         "wasm:\n"
         "  module: guest.wasm\n"
         "  export: " +
         std::string(kExports) + "\n" + wasm_extra +
         "root:\n"
         "  - text: hi\n";
}

// A document over the transformed page_guest.wat.
std::string page_doc(const std::string &wasm_extra = std::string()) {
  return "requires: [wasm]\n"
         "wasm:\n"
         "  module: page_guest.wasm\n"
         "  export: [get-n, get-many, hog, deep, climb, vread, sample]\n" +
         wasm_extra + "root:\n  - text: hi\n";
}

class WasmTransformed : public ::testing::Test {
protected:
  void SetUp() override {
    std::string error;
    engine = wasm::make_engine(error);
    ASSERT_TRUE(engine) << error;
    register_script_engine(engine);
#ifdef CVC_WASM_GUESTS_DIR
    for (const char *g : {"guest.wasm", "page_guest.wasm"}) {
      std::error_code ec;
      fs::copy_file(fs::path(CVC_WASM_GUESTS_DIR) / g, dir() / g,
                    fs::copy_options::overwrite_existing, ec);
      ASSERT_FALSE(ec) << g << ": " << ec.message();
    }
#endif
  }
  void TearDown() override { unregister_script_engine("wasm"); }

  // Load `text` as `name`: false, with the failure noted, when it doesn't load.
  static bool load(AppRuntime &art, const std::string &name, const std::string &text) {
    try {
      art.load(write(name, text).string());
      return true;
    } catch (const std::exception &e) {
      ADD_FAILURE() << name << ": " << e.what();
      return false;
    }
  }

  // Why `text` doesn't load as `name`: "loaded" when it does.
  static std::string refused(AppRuntime &art, const std::string &name, const std::string &text) {
    try {
      art.load(write(name, text).string());
    } catch (const std::exception &e) {
      return e.what();
    }
    return "loaded";
  }

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

// ── the memory cap ─────────────────────────────────────────────────────────────────────────────

// A memory without a maximum gets the cap's, in whole pages (a two- or three-byte LEB128 where it
// needs one), and the sections around it stay as they were. No memory: nothing changes.
TEST(WasmBinary, CapsTheMemoryMaximum) {
  std::string error;
  std::uint64_t pages = 0;
  const std::string one = module_with(bytes({0x05, 0x03, 0x01, 0x00, 0x01})); // (memory 1)
  EXPECT_EQ(wasm::cap_memory(one, 4 * 65536 + 100, error, &pages),
            module_with(bytes({0x05, 0x04, 0x01, 0x01, 0x01, 0x04})))
      << error;
  EXPECT_EQ(pages, 4u);
  EXPECT_EQ(wasm::cap_memory(one, 300 * 65536, error, &pages),
            module_with(bytes({0x05, 0x05, 0x01, 0x01, 0x01, 0xac, 0x02})))
      << error;
  EXPECT_EQ(pages, 300u);
  EXPECT_EQ(wasm::cap_memory(one, std::uint64_t(8) << 32, error, &pages),
            module_with(bytes({0x05, 0x06, 0x01, 0x01, 0x01, 0x80, 0x80, 0x04})))
      << "past 4 GiB, wasm32's own maximum: " << error;
  EXPECT_EQ(pages, 65536u);
  EXPECT_EQ(wasm::cap_memory(one, 0, error, &pages), one) << "0 caps nothing";
  EXPECT_EQ(pages, 65536u);
  const std::string none = kHeader + bytes({0x01, 0x04, 0x01, 0x60, 0x00, 0x00});
  EXPECT_EQ(wasm::cap_memory(none, 65536, error, &pages), none) << error;
  EXPECT_EQ(pages, 0u);
}

// A memory's own maximum is lowered to the cap, never raised.
TEST(WasmBinary, LowersAnExistingMaximumOnly) {
  std::string error;
  std::uint64_t pages = 0;
  const std::string eight =
      module_with(bytes({0x05, 0x04, 0x01, 0x01, 0x01, 0x08})); // (memory 1 8)
  EXPECT_EQ(wasm::cap_memory(eight, 4 * 65536, error, &pages),
            module_with(bytes({0x05, 0x04, 0x01, 0x01, 0x01, 0x04})))
      << error;
  EXPECT_EQ(pages, 4u);
  EXPECT_EQ(wasm::cap_memory(eight, 16 * 65536, error, &pages), eight) << error;
  EXPECT_EQ(pages, 8u);
  EXPECT_EQ(wasm::cap_memory(eight, 0, error, &pages), eight) << error;
  EXPECT_EQ(pages, 8u);
}

// What it can't cap: an imported memory (behind an imported function), memory64, a shared memory,
// a minimum above the cap, a section past the module's end, and what isn't a module.
TEST(WasmBinary, RefusesWhatItCantCap) {
  std::string error;
  const auto refused = [&](const std::string &module, std::uint64_t cap) {
    const std::string out = wasm::cap_memory(module, cap, error);
    EXPECT_TRUE(out.empty());
    return error;
  };
  const std::string imports = bytes({0x02}) + name("ari") + name("f") + bytes({0x00, 0x00}) +
                              name("ari") + name("mem") + bytes({0x02, 0x00, 0x01});
  const std::string imported = kHeader + bytes({0x01, 0x04, 0x01, 0x60, 0x00, 0x00}) +
                               bytes({0x02, static_cast<int>(imports.size())}) + imports;
  EXPECT_NE(refused(imported, 65536).find("imports its memory"), std::string::npos) << error;
  EXPECT_NE(refused(module_with(bytes({0x05, 0x03, 0x01, 0x04, 0x01})), 65536).find("memory64"),
            std::string::npos)
      << error;
  EXPECT_NE(refused(module_with(bytes({0x05, 0x04, 0x01, 0x03, 0x01, 0x02})), 65536).find("shared"),
            std::string::npos)
      << error;
  EXPECT_NE(
      refused(module_with(bytes({0x05, 0x03, 0x01, 0x00, 0x08})), 4 * 65536).find("starts at 8"),
      std::string::npos)
      << error;
  EXPECT_NE(refused(kHeader + bytes({0x05, 0x10, 0x01}), 65536).find("past the module's end"),
            std::string::npos)
      << error;
  EXPECT_NE(refused("not a module", 65536).find("isn't a WebAssembly module"), std::string::npos)
      << error;
}

// ── the transformed guests ─────────────────────────────────────────────────────────────────────

// The block's exports are functions of the document, on the transformed guest as on the original:
// _initialize ran once; a call's value and a future's.
TEST_F(WasmTransformed, ExportsAreFunctionsOfTheDocument) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "calls.ari", doc()));
  EXPECT_EQ(run(art, "(assert (= (echo 1 \"a\") (list 1 \"a\")) \"echo\")\n"
                     "(assert (= (nothing) nil) \"nil\")\n"
                     "(assert (= (inits) 1) \"_initialize ran once\")\n"
                     "(let ((f (echo-async 3))) (assert (= (await f) (list 3)) \"a future\"))"),
            "");
}

// A guest's call through ari.call parks it (in the browser: unwound into its memory) until the
// loop has made the call, then resumes it where it stopped (rewound), with the call's value.
TEST_F(WasmTransformed, CallsParkTheGuestAndResumeIt) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  int pings = 0;
  art.register_function("test-ping", [&](std::span<const se::value_t>) -> se::value_t {
    ++pings;
    return se::value_t();
  });
  ASSERT_TRUE(load(art, "lane.ari", doc()));
  EXPECT_EQ(run(art, "(state-set \"x\" \"hi\")\n"
                     "(assert (= (get-x) \"hi\") \"state-get\")\n"
                     "(set-y)\n"
                     "(ping) (ping)"),
            "");
  EXPECT_EQ(val(app, "ui.wa.y"), "ok") << "written under the document's prefix";
  EXPECT_EQ(pings, 2);
}

// yield() gives the frame back: (nap) yields twice and returns in a later frame.
TEST_F(WasmTransformed, AYieldGivesTheFrameBack) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "nap.ari", doc()));
  int drains = 0;
  EXPECT_EQ(run(art, "(assert (= (nap) 7) \"nap\")", &drains), "");
  EXPECT_GE(drains, 3);
}

// A loop with no import in it is sliced at its gas checks: with a frame budget no slice fits in,
// each slice ends a frame, and (spin) goes on over frames to its end.
TEST_F(WasmTransformed, ASpinIsSlicedAndChargedAsSteps) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "spin.ari", doc()));
  auto &st = cvc::state::instance(app);
  st("ariadne.limits.action_max_time").value(std::string("10"));
  st("ariadne.limits.action_max_steps").value(std::string("0"));
  st("ariadne.limits.wasm_slice_steps").value(std::string("10000"));
  st("ariadne.limits.wasm_frame_budget").value(std::string("0.000001"));
  int drains = 0;
  EXPECT_EQ(run(art, "(spin)", &drains), "");
  EXPECT_GE(drains, 3) << "the guest paused at its gas checks, a slice a frame";
}

// The slices are charged to the calling program as steps, so the action's step cap stops a spin.
TEST_F(WasmTransformed, AStepCapKillsASpin) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "cap.ari", doc()));
  auto &st = cvc::state::instance(app);
  st("ariadne.limits.action_max_time").value(std::string("10"));
  st("ariadne.limits.action_max_steps").value(std::string("5000"));
  const std::string capped = run(art, "(spin)");
  EXPECT_NE(capped.find("max_steps_exceeded"), std::string::npos) << capped;
  st("ariadne.limits.action_max_steps").value(std::string("0"));
  EXPECT_EQ(run(art, "(assert (= (echo 2) (list 2)) \"the guest still works\")"), "");
}

// A pause deep in the stack comes back to the same frames: (deep) yields 8 frames down, and
// (climb) is sliced at gas checks wherever its 20 frames are when a slice ends.
TEST_F(WasmTransformed, ADeepStackPausesAndResumes) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "deep.ari", page_doc()));
  int drains = 0;
  EXPECT_EQ(run(art, "(assert (= (deep) 9) \"deep\")", &drains), "");
  EXPECT_GE(drains, 2);
  auto &st = cvc::state::instance(app);
  st("ariadne.limits.action_max_time").value(std::string("10"));
  st("ariadne.limits.action_max_steps").value(std::string("0"));
  st("ariadne.limits.wasm_slice_steps").value(std::string("1000"));
  st("ariadne.limits.wasm_frame_budget").value(std::string("0.000001"));
  EXPECT_EQ(run(art, "(assert (= (climb) 21) \"climb\")", &drains), "");
  EXPECT_GE(drains, 3);
}

// A trap and fail() are WasmErrors a program can catch, and a call back into the guest from
// inside it is a RecursionError the guest fails with.
TEST_F(WasmTransformed, TrapsAndFailuresAreWasmErrors) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "errors.ari", doc()));
  EXPECT_EQ(run(art, "(state-set \"t\" (try (boom) (catch (WasmError e) (get-attr e \"trap\"))))\n"
                     "(state-set \"r\" (try (refuse) (catch (WasmError e) (get-attr e "
                     "\"message\"))))\n"
                     "(state-set \"s\" (try (self) (catch (WasmError e) (get-attr e "
                     "\"message\"))))"),
            "");
  EXPECT_NE(val(app, "ui.wa.t").find("unreachable"), std::string::npos) << val(app, "ui.wa.t");
  EXPECT_NE(val(app, "ui.wa.r").find("no route"), std::string::npos) << val(app, "ui.wa.r");
  EXPECT_NE(val(app, "ui.wa.s").find("RecursionError"), std::string::npos) << val(app, "ui.wa.s");
  const std::string uncaught = run(art, "(boom)");
  EXPECT_NE(uncaught.find("WasmError"), std::string::npos) << uncaught;
  EXPECT_EQ(run(art, "(assert (= (echo 3) (list 3)) \"after a trap, the guest still works\")"), "");
}

// A state import pauses the guest for the lane's state function, which is called in the slice
// (ScriptSlice::call_now), and the guest resumes there: 200 of them fit under 1000 steps, a step
// or two each, where a call through the loop costs more.
TEST_F(WasmTransformed, StateImportsResumeInTheSlice) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "state.ari", page_doc()));
  auto &st = cvc::state::instance(app);
  st("ariadne.limits.action_max_time").value(std::string("10"));
  st("ariadne.limits.action_max_steps").value(std::string("1000"));
  EXPECT_EQ(run(art, "(state-set \"n\" \"42\")\n"
                     "(assert (= (get-n) \"42\") \"a state import\")\n"
                     "(assert (= (get-many) \"42\") \"200 of them\")"),
            "");
}

// A memory.grow past the block's limit (memory_mb) is refused, and the guest giving up there is
// out of memory: a MemoryError. In the browser the limit is the module's own maximum, capped.
TEST_F(WasmTransformed, AGrowPastTheLimitIsAMemoryError) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "hog.ari", page_doc("  limits: {memory_mb: 1}\n")));
  EXPECT_EQ(run(art, "(state-set \"r\" (try (hog) (catch (MemoryError e) (get-attr e "
                     "\"message\")) (catch (WasmError e) \"a WasmError\")))"),
            "");
  EXPECT_NE(val(app, "ui.wa.r").find("out of memory, at its limit of 1 MiB"), std::string::npos)
      << val(app, "ui.wa.r");
}

// The data calls copy into the guest's memory and out of it (in the browser through a staging
// buffer, GuestOut and GuestIn): a volume read whole, every other voxel, and sampled at a point
// the guest's memory holds.
TEST_F(WasmTransformed, AVolumeIsReadIntoTheGuestsMemory) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  ASSERT_TRUE(load(art, "vol.ari", page_doc()));
  std::vector<unsigned char> v(12);
  for (int i = 0; i < 12; ++i)
    v[i] = static_cast<unsigned char>(i + 1);
  auto vol = std::make_shared<cvc::volume>(app, v.data(), cvc::dimension(3, 2, 2), cvc::UChar);
  vol->boundingBox() = cvc::bounding_box(0, 0, 0, 2, 1, 1);
  cvc::state::instance(app)("ui.wa.vol").data(boost::any(std::shared_ptr<const cvc::volume>(vol)));
  EXPECT_EQ(run(art, "(let ((v (state-data-get \"vol\")))\n"
                     "  (assert (= (vread v 0 0 0 3 2 2 0) (list 1 2 3 4 5 6 7 8 9 10 11 12)) "
                     "\"all of it\")\n"
                     "  (assert (= (vread v 0 0 0 3 2 2 1) (list 1 3)) \"every other voxel\")\n"
                     "  (assert (= (sample v) 2) \"a value at a point\"))"),
            "");
}

// hooks: on_tick delivers every frame to the guest's export, which calls the host back.
TEST_F(WasmTransformed, AFrameHookRunsTheGuest) {
  REQUIRE_YAML();
  REQUIRE_GUESTS();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  int pings = 0;
  art.register_function("test-ping", [&](std::span<const se::value_t>) -> se::value_t {
    ++pings;
    return se::value_t();
  });
  ASSERT_TRUE(load(art, "tick.ari", doc("  hooks: {on_tick: ping}\n")));
  for (int i = 0; i < 6; ++i)
    art.drain();
  EXPECT_GE(pings, 4);
}

#ifdef __EMSCRIPTEN__

// ── what the browser refuses ───────────────────────────────────────────────────────────────────

using WasmBrowser = WasmTransformed;

// The page compiles binaries only: a .wat module is refused, naming the transform.
TEST_F(WasmBrowser, AWatModuleIsRefused) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  write("text.wat", "(module)");
  const std::string why = refused(art, "text.ari", "wasm: text.wat\nroot:\n  - text: hi\n");
  EXPECT_NE(why.find("the browser runs .wasm only"), std::string::npos) << why;
  EXPECT_NE(why.find("cvc_ariadne_wasm_transform"), std::string::npos) << why;
}

// A module without the transform's gas checks and Asyncify exports can't be paused, or metered:
// refused, naming the transform.
TEST_F(WasmBrowser, AnUntransformedGuestIsRefused) {
  REQUIRE_YAML();
  cvc::app app;
  AppRuntime art(app, "ui.wa");
  art.use_script_engine(engine);
  write("plain.wasm", kHeader);
  const std::string why = refused(art, "plain.ari", "wasm: plain.wasm\nroot:\n  - text: hi\n");
  EXPECT_NE(why.find("isn't transformed for the browser"), std::string::npos) << why;
  EXPECT_NE(why.find("cvc_ariadne_wasm_transform"), std::string::npos) << why;
}

#endif // __EMSCRIPTEN__
