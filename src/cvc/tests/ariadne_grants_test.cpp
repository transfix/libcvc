// Asks and grants, input, signals and the command line of mounted documents
// (docs/roadmap/ARIADNE_SHELL.md 4.5, 4.6; SH7).
//
// A mounted document asks for what lies beyond its scope in its requires:, with the grant tokens
// (grants.h); the document that loads it grants them with grants: on its load: or mounts: slot.
// These tests check the arithmetic (grants.cpp), what the loader makes of a load:'s grants: and
// argv:, and what the Runtime does with them: a required token not granted refuses the mount, a
// key reaches only the focused granted application and none while a parent's field has the
// keyboard, a request to end reaches every granted application, a mount's argv is parsed against
// its args: and a bad one refused with its usage posted, and one document's tests pass alone and
// mounted. Each app keeps its own: two apps side by side do not see each other's focus.

#include <algorithm>
#include <chrono>
#include <cvc/ariadne/app_runtime.h>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/grants.h>
#include <cvc/ariadne/input.h>
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/core/notify.h>
#include <cvc/state/state.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <string>
#include <thread>
#include <vector>

using namespace cvc::ariadne;
namespace fs = std::filesystem;

namespace {

// A backend that draws nothing and records the windows and fields the walk asks for. It can play
// the focus (the window titled `focused`, when `reports` is on) and a text field holding the
// keyboard (the field whose state key is `typing_in`), as ImGui reports them.
struct FocusBackend : Backend {
  bool reports = false;
  std::string focused;   // the title of the window that has the focus
  std::string hovered;   // ... that the pointer is over
  std::string typing_in; // the state key of the text field being typed into
  std::vector<std::string> windows;
  bool last_focused = false, last_hovered = false, last_typing = false;

  Capabilities capabilities() const override {
    Capabilities c;
    c.windows = true;
    c.menubar = true;
    c.keyboard = true;
    c.mouse = true;
    return c;
  }
  bool reports_focus() const override { return reports; }
  bool window_focused() const override { return last_focused; }
  bool window_hovered() const override { return last_hovered; }
  bool item_has_keyboard() const override { return last_typing; }
  void begin_frame() override { windows.clear(); }
  bool begin_main_menu_bar() override { return true; }
  void end_main_menu_bar() override {}
  std::vector<std::string> menus; // the menus begun: a tray icon is one, with no tray
  bool begin_menu(const char *label) override {
    menus.emplace_back(label);
    return false;
  }
  bool began_menu_with(const std::string &part) const {
    return std::any_of(menus.begin(), menus.end(),
                       [&](const std::string &m) { return m.find(part) != std::string::npos; });
  }
  void end_menu() override {}
  bool begin_window(const char *title, const char *, const Size &, float, bool *) override {
    windows.emplace_back(title);
    last_focused = reports && focused == title;
    last_hovered = hovered == title;
    return true;
  }
  void end_window() override {}
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  std::vector<std::string> texts; // "label: value" for each bound text drawn
  bool drew(const std::string &t) const {
    return std::find(texts.begin(), texts.end(), t) != texts.end();
  }
  void text_line(const char *t) override { texts.emplace_back(t); }
  void text_value(const char *l, const std::string &v) override {
    texts.push_back(std::string(l) + ": " + v);
  }
  void separator() override {}
  bool button(const char *) override { return false; }
  bool menu_item_action(const char *) override { return false; }
  BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  BoolEdit checkbox(const char *, bool) override { return {}; }
  IntEdit slider_int(const char *, const char *, int, int, int) override { return {}; }
  DoubleEdit slider_double(const char *, const char *, double, double, double,
                           const char *) override {
    return {};
  }
  IndexEdit combo(const char *, int, const std::vector<std::string> &) override { return {}; }
  StringEdit input_text(const char *, const char *key, const std::string &current, bool) override {
    last_typing = !typing_in.empty() && typing_in == key;
    StringEdit e;
    e.value = current;
    return e;
  }
};

fs::path write_doc(const fs::path &dir, const std::string &name, const std::string &text) {
  fs::create_directories((dir / name).parent_path());
  const fs::path p = dir / name;
  std::ofstream(p, std::ios::binary) << text;
  return p;
}

// The value at `path`, or "" when there is none (it makes no node).
std::string peek(cvc::app &app, const std::string &path) {
  const cvc::state *n = cvc::state::instance(app).findDescendant(path);
  return n ? const_cast<cvc::state *>(n)->value() : std::string();
}

void put(cvc::app &app, const std::string &path, const std::string &value) {
  cvc::state::instance(app)(path).value(value);
}

template <class Pred> bool pump_until(Runtime &rt, Pred pred, int budget_ms = 5000) {
  using namespace std::chrono;
  const auto deadline = steady_clock::now() + milliseconds(budget_ms);
  while (steady_clock::now() < deadline) {
    rt.render();
    rt.drain();
    if (pred())
      return true;
    std::this_thread::sleep_for(milliseconds(2));
  }
  return pred();
}

void pump(Runtime &rt, int frames = 3) {
  for (int i = 0; i < frames; ++i) {
    rt.render();
    rt.drain();
  }
}

// Run `program` in the document's own scope, and wait for it to end: "" when it finished.
std::string run(Runtime &rt, const std::string &program) {
  bool ended = false;
  std::string how = "did not end";
  rt.run_program(program, [&](const std::string &h) {
    how = h;
    ended = true;
  });
  pump_until(rt, [&] { return ended; });
  return how;
}

void key(Runtime &rt, const std::string &name) {
  InputEvent down;
  down.kind = InputEvent::Kind::KeyDown;
  down.key = name;
  rt.post_input(down);
  InputEvent up = down;
  up.kind = InputEvent::Kind::KeyUp;
  rt.post_input(up);
}

bool posted(cvc::app &app, const std::string &needle) {
  for (const cvc::notification_entry &e : cvc::notifications(app))
    if (e.text.find(needle) != std::string::npos)
      return true;
  return false;
}

const Runtime::MountState *mount_named(const std::vector<Runtime::MountState> &all,
                                       const std::string &as) {
  for (const Runtime::MountState &m : all)
    if (m.as == as)
      return &m;
  return nullptr;
}

// An application that takes keys and signals: each key down it gets is appended to `keys`, each
// signal to `signals` (with a ';').
const char *const kTaker = R"(meta: { name: Taker, min_libcvc: "0.0.0" }
requires: [delegate.keys, delegate.signals]
on_key: |
  (if (= (get-attr event "kind") "key_down")
      (state-set "typed" (str-concat (if (is-null (state-get "typed")) "" (state-get "typed"))
                                    (get-attr event "key")))
      #f)
on_signal: |
  (state-set "signals" (str-concat (if (is-null (state-get "signals")) "" (state-get "signals"))
                                   (get-attr event "signal") ";"))
windows:
  - window: Taker
    id: taker
    children:
      - text: typed
        bind: typed
)";

// The same, asking for both only as optional.
const char *const kMaybeTaker = R"(meta: { name: Maybe, min_libcvc: "0.0.0" }
requires:
  - {name: delegate.keys, optional: true}
  - {name: delegate.signals, optional: true}
on_key: (state-set "typed" (get-attr event "key"))
on_signal: (state-set "signals" (get-attr event "signal"))
)";

std::vector<HostRequirement> asks(std::initializer_list<HostRequirement> l) { return l; }

} // namespace

// ── the arithmetic (grants.cpp) ──────────────────────────────────────────────────────────────────

TEST(AriadneGrants, TheTokensAndWhatADocumentAloneHolds) {
  EXPECT_TRUE(is_grant_token("delegate.keys"));
  EXPECT_TRUE(is_grant_token("net.origin"));
  EXPECT_TRUE(is_grant_token("access.settings.read"));
  EXPECT_FALSE(is_grant_token("gl.scene")) << "a host's capability, not a grant";
  EXPECT_FALSE(is_grant_token("delegate.nothing"));
  for (const char *t : {"delegate.keys", "delegate.pointer", "delegate.signals", "extend.menubar",
                        "extend.tray", "extend.context_menu", "extend.settings", "net.origin"})
    EXPECT_TRUE(held_alone(t)) << t;
  for (const char *t :
       {"access.settings", "access.settings.read", "request.main_view", "request.open"})
    EXPECT_FALSE(held_alone(t)) << t << ": no one shares or opens for the outermost document";
  EXPECT_EQ(default_grants(), (std::vector<std::string>{"extend.tray", "extend.settings"}));
  for (const GrantToken &t : grant_tokens()) {
    EXPECT_NE(std::string(t.meaning), "") << t.name;
    EXPECT_NE(std::string(t.words), "") << t.name;
  }
}

TEST(AriadneGrants, AskedAndGrantedHolds) {
  GrantBuilder b;
  EXPECT_EQ(b.add("delegate.keys"), "");
  EXPECT_NE(b.add("delegate.nothing"), "") << "not a token";
  EXPECT_NE(b.add("delegate.keys", nullptr), "x") << "a bare token again is fine";
  const std::vector<std::string> sigs{"term", "SIGHUP"};
  EXPECT_EQ(b.add("delegate.signals", &sigs), "");
  const std::vector<std::string> more{"x"};
  EXPECT_NE(b.add("extend.tray", &more), "") << "only delegate.signals and net.origin take a list";
  const GrantSet g = b.take();
  EXPECT_TRUE(g.has("delegate.keys"));
  EXPECT_FALSE(g.every_signal);
  EXPECT_EQ(g.signals, (std::vector<std::string>{"SIGTERM", "SIGHUP"}));

  const MountGrants m = hold_grants(g, asks({{"delegate.keys"},
                                             {"delegate.signals"},
                                             {"delegate.pointer", true},
                                             {"access.settings"}}));
  EXPECT_TRUE(m.holds("delegate.keys"));
  EXPECT_TRUE(m.holds("delegate.signals"));
  EXPECT_FALSE(m.holds("delegate.pointer")) << "optional, not granted";
  EXPECT_FALSE(m.holds("extend.tray")) << "not granted (the list replaced the defaults)";
  EXPECT_EQ(m.refused, (std::vector<std::string>{"access.settings"})) << "required, not granted";
  EXPECT_TRUE(m.takes_signal("SIGTERM"));
  EXPECT_TRUE(m.takes_signal("term"));
  EXPECT_FALSE(m.takes_signal("SIGINT")) << "the list limits it";

  // A token it did not ask for is not given; the defaults are, as every mount has had them.
  const MountGrants d = hold_grants(deny_tokens(default_grant_set(), {"extend.settings"}), {});
  EXPECT_TRUE(d.holds("extend.tray"));
  EXPECT_FALSE(d.holds("extend.settings")) << "denied";
  GrantSet keys;
  keys.tokens = {"delegate.keys"};
  EXPECT_FALSE(hold_grants(keys, {}).holds("delegate.keys")) << "granted, never asked";
}

TEST(AriadneGrants, AGrantNeverWidensDownTheTree) {
  GrantBuilder outer_b;
  outer_b.add("delegate.keys");
  const std::vector<std::string> one{"SIGTERM"};
  outer_b.add("delegate.signals", &one);
  const std::vector<std::string> api{"https://api.example.org"};
  outer_b.add("net.origin", &api);
  const MountGrants outer =
      hold_grants(outer_b.take(), asks({{"delegate.keys"},
                                        {"delegate.signals"},
                                        {"net.origin", false, {"https://api.example.org"}}}));
  ASSERT_TRUE(outer.holds("net.origin"));

  GrantBuilder inner_b; // it grants its own mount more than it holds
  for (const char *t : {"delegate.keys", "delegate.pointer", "delegate.signals", "net.origin"})
    inner_b.add(t);
  const GrantSet bounded = bound_grants(inner_b.take(), &outer);
  EXPECT_TRUE(bounded.has("delegate.keys"));
  EXPECT_FALSE(bounded.has("delegate.pointer")) << "the outer does not hold it";
  EXPECT_FALSE(bounded.every_signal);
  EXPECT_EQ(bounded.signals, (std::vector<std::string>{"SIGTERM"}));
  EXPECT_FALSE(bounded.every_origin);
  EXPECT_EQ(bounded.origins, (std::vector<std::string>{"https://api.example.org"}));
  const MountGrants inner = hold_grants(
      bounded, asks({{"net.origin", false, {"https://api.example.org", "https://evil.example"}}}));
  EXPECT_EQ(inner.net_origins, (std::vector<std::string>{"https://api.example.org"}));
  EXPECT_EQ(inner.refused, (std::vector<std::string>{"net.origin https://evil.example"}));

  // Alone, the outermost document passes on what it holds alone, and nothing of access.*.
  GrantBuilder top;
  top.add("delegate.keys");
  top.add("access.settings");
  const GrantSet from_top = bound_grants(top.take(), nullptr);
  EXPECT_TRUE(from_top.has("delegate.keys"));
  EXPECT_FALSE(from_top.has("access.settings"));
}

TEST(AriadneGrants, PolicyOriginsAndSignalNames) {
  EXPECT_EQ(policy_origin("https://API.Example.org:443/v1/x?y#z"), "https://api.example.org");
  EXPECT_EQ(policy_origin("http://localhost:8080"), "http://localhost:8080");
  EXPECT_EQ(policy_origin("http://example.org:80/"), "http://example.org");
  EXPECT_EQ(policy_origin("https://[::1]:8443/a"), "https://[::1]:8443");
  EXPECT_EQ(policy_origin("ftp://example.org"), "");
  EXPECT_EQ(policy_origin("https://user@example.org"), "") << "credentials in it";
  EXPECT_EQ(policy_origin("https://example.org:x"), "");
  EXPECT_EQ(policy_origin("example.org"), "");
  EXPECT_EQ(signal_name("term"), "SIGTERM");
  EXPECT_EQ(signal_name("SIGHUP"), "SIGHUP");
  EXPECT_EQ(signal_name("sigint"), "SIGINT");
}

// ── the loader: a load:'s grants:, deny:, argv: and its fragment's input ────────────────────────

TEST(AriadneGrantsLoader, ALoadsGrantsAreWhatItsFragmentHolds) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_loader";
  fs::remove_all(dir);
  write_doc(dir, "taker.ari", kTaker);
  write_doc(dir, "maybe.ari", kMaybeTaker);
  const fs::path doc = write_doc(dir, "doc.ari", R"(meta: { name: Doc, min_libcvc: "0.0.0" }
root:
  - load: taker.ari
    as: granted
    grants: [delegate.keys, {delegate.signals: [SIGTERM]}]
  - load: maybe.ari
    as: defaults
  - load: maybe.ari
    as: denied
    deny: extend.tray
)");
  cvc::app app;
  const LoadResult r = load_file(app, doc.string());
  ASSERT_TRUE(r.ok) << r.error;
  std::map<std::string, const Widget *> mounts;
  for (const Widget &w : r.root.children)
    if (!w.scope.empty())
      mounts[w.scope] = &w;
  ASSERT_EQ(mounts.size(), 3u);
  const Widget &granted = *mounts.at("includes.granted");
  ASSERT_TRUE(granted.grants);
  EXPECT_TRUE(granted.grants->holds("delegate.keys"));
  EXPECT_TRUE(granted.grants->holds("delegate.signals"));
  EXPECT_TRUE(granted.grants->takes_signal("SIGTERM"));
  EXPECT_FALSE(granted.grants->takes_signal("SIGINT"));
  EXPECT_FALSE(granted.grants->holds("extend.tray")) << "its grants: replaced the defaults";
  EXPECT_NE(granted.grants->on_key.find("key_down"), std::string::npos) << "its on_key: is kept";
  EXPECT_NE(granted.grants->on_signal.find("signals"), std::string::npos);

  const Widget &defaults = *mounts.at("includes.defaults");
  ASSERT_TRUE(defaults.grants);
  EXPECT_FALSE(defaults.grants->holds("delegate.keys")) << "optional, and not granted";
  EXPECT_TRUE(defaults.grants->holds("extend.tray"));
  EXPECT_TRUE(defaults.grants->holds("extend.settings"));
  EXPECT_TRUE(defaults.grants->on_key.empty()) << "ignored: it does not hold delegate.keys";
  const auto warned = [&](const std::string &needle) {
    return std::any_of(r.warnings.begin(), r.warnings.end(),
                       [&](const std::string &w) { return w.find(needle) != std::string::npos; });
  };
  EXPECT_TRUE(warned("on_key: is ignored — it does not hold delegate.keys"));
  EXPECT_FALSE(mounts.at("includes.denied")->grants->holds("extend.tray")) << "deny:";
  EXPECT_TRUE(mounts.at("includes.denied")->grants->holds("extend.settings"));
  fs::remove_all(dir);
}

TEST(AriadneGrantsLoader, AnOutOfTheWayGrantsListWarns) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_loader_warn";
  fs::remove_all(dir);
  write_doc(dir, "maybe.ari", kMaybeTaker);
  const fs::path doc = write_doc(dir, "doc.ari", R"(meta: { name: Doc, min_libcvc: "0.0.0" }
root:
  - load: maybe.ari
    grants: [delegate.kees, {extend.tray: [x]}, {net.origin: [ftp://nope]}]
)");
  cvc::app app;
  const LoadResult r = load_file(app, doc.string());
  ASSERT_TRUE(r.ok) << r.error;
  std::string all;
  for (const std::string &w : r.warnings)
    all += w + "\n";
  EXPECT_NE(all.find("'delegate.kees' is not a grant token"), std::string::npos) << all;
  EXPECT_NE(all.find("takes no list"), std::string::npos) << all;
  EXPECT_NE(all.find("not an http(s) origin"), std::string::npos) << all;
  fs::remove_all(dir);
}

TEST(AriadneGrantsLoader, NetOriginAsksAreHeldWhereGranted) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_loader_net";
  fs::remove_all(dir);
  write_doc(dir, "client.ari", R"(meta: { name: Client, min_libcvc: "0.0.0" }
requires:
  - {name: net.origin, origins: ["https://API.example.org:443/v1", "https://cdn.example.org"]}
)");
  const fs::path doc = write_doc(dir, "doc.ari", R"(meta: { name: Doc, min_libcvc: "0.0.0" }
root:
  - load: client.ari
    as: every
    grants: [net.origin]
)");
  cvc::app app;
  const LoadResult r = load_file(app, doc.string());
  ASSERT_TRUE(r.ok) << r.error;
  const Widget &m = r.root.children.at(0);
  ASSERT_TRUE(m.grants);
  EXPECT_TRUE(m.grants->holds("net.origin"));
  EXPECT_EQ(m.grants->net_origins,
            (std::vector<std::string>{"https://api.example.org", "https://cdn.example.org"}));
  ASSERT_EQ(m.requirements.size(), 1u);
  EXPECT_EQ(m.requirements[0].origins.size(), 2u);
  fs::remove_all(dir);
}

// docs/roadmap/ARIADNE_SHELL.md 4.6: a load:'s argv: is parsed by its fragment's args: block, and a
// bad one leaves the mount out, with its usage; a mounts: entry's (load_mount) is refused.
TEST(AriadneGrantsLoader, ArgvIsParsedAndABadOneRefusedWithItsUsage) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_loader_argv";
  fs::remove_all(dir);
  write_doc(dir, "greeter.ari",
            R"(meta: { name: Greeter, description: Says hello, min_libcvc: "0.0.0" }
args:
  greeting: { type: string, default: hello, help: What it says }
  loud: bool
  name: { type: string, required: true }
)");
  const fs::path doc = write_doc(dir, "doc.ari", R"(meta: { name: Doc, min_libcvc: "0.0.0" }
root:
  - load: greeter.ari
    as: good
    argv: [--name, Ada, --loud]
  - load: greeter.ari
    as: bad
    argv: [--nope]
)");
  cvc::app app;
  const LoadResult r = load_file(app, doc.string());
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *good = nullptr;
  for (const Widget &w : r.root.children)
    if (w.scope == "includes.good")
      good = &w;
  ASSERT_NE(good, nullptr);
  ASSERT_TRUE(good->command_line);
  EXPECT_TRUE(good->command_line->has_args_block);
  EXPECT_EQ(good->command_line->argv, (std::vector<std::string>{"--name", "Ada", "--loud"}));
  EXPECT_TRUE(good->command_line->parsed.errors.empty());
  std::map<std::string, std::string> values(good->command_line->parsed.values.begin(),
                                            good->command_line->parsed.values.end());
  EXPECT_EQ(values["name"], "Ada");
  EXPECT_EQ(values["greeting"], "hello");
  EXPECT_EQ(values["loud"], "true");
  std::string all;
  for (const std::string &w : r.warnings)
    all += w + "\n";
  EXPECT_NE(all.find("'greeter.ari' is not mounted"), std::string::npos) << all;
  EXPECT_NE(all.find("usage:"), std::string::npos) << "with its usage\n" << all;
  for (const Widget &w : r.root.children)
    EXPECT_NE(w.scope, "includes.bad") << "the bad one is left out";

  MountRequest rq;
  rq.uri = "greeter.ari";
  rq.as = "g";
  rq.base_dir = dir.string();
  const MountResult none = load_mount(app, rq); // no argv: its required name is missing
  EXPECT_FALSE(none.ok);
  EXPECT_NE(none.error.find("name"), std::string::npos) << none.error;
  EXPECT_NE(none.error.find("usage:"), std::string::npos) << none.error;
  rq.has_argv = true;
  rq.argv = {"--name", "Grace"};
  const MountResult ok = load_mount(app, rq);
  ASSERT_TRUE(ok.ok) << ok.error;
  ASSERT_TRUE(ok.wrapper.command_line);
  EXPECT_TRUE(ok.wrapper.command_line->parsed.errors.empty());
  fs::remove_all(dir);
}

// ── the Runtime ──────────────────────────────────────────────────────────────────────────────────

// A required token not granted refuses the mount, naming it, with the refusal in the feed; granted,
// it opens and its entry lists what it holds. A static load: of it fails the document's load.
TEST(AriadneGrantsRuntime, ARequiredTokenNotGrantedRefusesTheMount) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_refuse";
  fs::remove_all(dir);
  const fs::path taker = write_doc(dir, "taker.ari", kTaker);
  const fs::path stingy =
      write_doc(dir, "stingy.ari", R"(meta: { name: Stingy, min_libcvc: "0.0.0" }
root:
  - mounts: apps
)");
  {
    cvc::app app;
    AppRuntime art(app, "ui.shell");
    FocusBackend be;
    art.set_backend(be);
    art.load(stingy.string());
    Runtime &rt = art.runtime();
    put(app, "ui.shell.apps.t.uri", taker.generic_string());
    ASSERT_TRUE(pump_until(rt, [&] { return !peek(app, "ui.shell.apps.t.status").empty(); }));
    EXPECT_EQ(peek(app, "ui.shell.apps.t.status"), "refused");
    const std::string error = peek(app, "ui.shell.apps.t.error");
    EXPECT_NE(error.find("not granted"), std::string::npos) << error;
    EXPECT_NE(error.find("delegate.keys"), std::string::npos) << error;
    EXPECT_NE(error.find("delegate.signals"), std::string::npos) << error;
    EXPECT_TRUE(posted(app, "Not opened")) << "the refusal is in the feed";
    // An entry's own grants pick from the slot's: they cannot add what the slot does not grant.
    put(app, "ui.shell.apps.u.grants", "delegate.keys,delegate.signals");
    put(app, "ui.shell.apps.u.uri", taker.generic_string());
    ASSERT_TRUE(pump_until(rt, [&] { return !peek(app, "ui.shell.apps.u.status").empty(); }));
    EXPECT_EQ(peek(app, "ui.shell.apps.u.status"), "refused");
  }
  const fs::path generous =
      write_doc(dir, "generous.ari", R"(meta: { name: Generous, min_libcvc: "0.0.0" }
root:
  - mounts: apps
    grants: [delegate.keys, delegate.signals, extend.tray]
)");
  {
    cvc::app app;
    AppRuntime art(app, "ui.shell");
    FocusBackend be;
    art.set_backend(be);
    art.load(generous.string());
    Runtime &rt = art.runtime();
    put(app, "ui.shell.apps.t.uri", taker.generic_string());
    ASSERT_TRUE(pump_until(rt, [&] { return peek(app, "ui.shell.apps.t.status") == "open"; }))
        << peek(app, "ui.shell.apps.t.error");
    EXPECT_EQ(peek(app, "ui.shell.apps.t.holds"), "delegate.keys,delegate.signals,extend.tray");
    const std::vector<Runtime::MountState> all = rt.mounts();
    const Runtime::MountState *m = mount_named(all, "t");
    ASSERT_NE(m, nullptr);
    EXPECT_EQ(m->grants,
              (std::vector<std::string>{"delegate.keys", "delegate.signals", "extend.tray"}));
    // ... and an entry that picks less holds less: delegate.signals alone is not enough.
    put(app, "ui.shell.apps.v.grants", "delegate.signals");
    put(app, "ui.shell.apps.v.uri", taker.generic_string());
    ASSERT_TRUE(pump_until(rt, [&] { return !peek(app, "ui.shell.apps.v.status").empty(); }));
    EXPECT_EQ(peek(app, "ui.shell.apps.v.status"), "refused");
    EXPECT_NE(peek(app, "ui.shell.apps.v.error").find("delegate.keys"), std::string::npos);
  }
  // A static load: that does not grant it fails the document's load, as a missing entry does.
  const fs::path loads = write_doc(dir, "loads.ari", R"(meta: { name: Loads, min_libcvc: "0.0.0" }
root:
  - load: taker.ari
)");
  {
    cvc::app app;
    AppRuntime art(app, "ui.doc");
    FocusBackend be;
    art.set_backend(be);
    try {
      art.load(loads.string());
      ADD_FAILURE() << "loaded";
    } catch (const std::exception &e) {
      EXPECT_NE(std::string(e.what()).find("it requires what it is not granted: delegate.keys"),
                std::string::npos)
          << e.what();
    }
  }
  fs::remove_all(dir);
}

namespace {
// A shell with a field, a key binding of its own on Q, and a slot that grants keys and signals.
const char *const kShell = R"(meta: { name: Shell, min_libcvc: "0.0.0" }
keys:
  quit_app: { label: Shell key, key: Q, on: (state-set "shell_q" "1") }
root:
  - mounts: apps
    grants: [delegate.keys, delegate.signals]
  - input_text: Search
    bind: search
)";

struct ShellFixture {
  fs::path dir;
  cvc::app app;
  AppRuntime art{app, "ui.shell"};
  FocusBackend be;
  explicit ShellFixture(const std::string &name, const std::string &shell = kShell) {
    dir = fs::temp_directory_path() / name;
    fs::remove_all(dir);
    const fs::path taker = write_doc(dir, "taker.ari", kTaker);
    const fs::path maybe = write_doc(dir, "maybe.ari", kMaybeTaker);
    const fs::path doc = write_doc(dir, "shell.ari", shell);
    art.set_backend(be);
    art.load(doc.string());
    put(app, "ui.shell.apps.a.uri", taker.generic_string());
    put(app, "ui.shell.apps.b.uri", taker.generic_string());
    put(app, "ui.shell.apps.c.grants", ""); // it picks nothing of the slot's
    put(app, "ui.shell.apps.c.uri", maybe.generic_string());
  }
  ~ShellFixture() {
    std::error_code ec;
    fs::remove_all(dir, ec);
  }
  Runtime &rt() { return art.runtime(); }
  std::string keys(const std::string &as) {
    return peek(app, "ui.shell.includes." + as + ".typed");
  }
  std::string signals(const std::string &as) {
    return peek(app, "ui.shell.includes." + as + ".signals");
  }
  bool all_open() {
    for (const char *as : {"a", "b", "c"})
      if (peek(app, std::string("ui.shell.apps.") + as + ".status") != "open")
        return false;
    return true;
  }
};
} // namespace

// docs/roadmap/ARIADNE_SHELL.md 4.6: a key goes to at most one application, the focused one that
// holds delegate.keys; the document's own bindings match first, and while a text field outside the
// application has the keyboard no key reaches it.
TEST(AriadneGrantsRuntime, AKeyReachesOnlyTheFocusedGrantedApplication) {
  ShellFixture s("ariadne_grants_keys");
  Runtime &rt = s.rt();
  ASSERT_TRUE(pump_until(rt, [&] { return s.all_open(); })) << peek(s.app, "ui.shell.apps.c.error");
  EXPECT_EQ(peek(s.app, "ui.shell.apps.c.holds"), "") << "it picked nothing";

  // No application has the focus: no key reaches one.
  key(rt, "K");
  pump(rt);
  EXPECT_EQ(s.keys("a") + s.keys("b") + s.keys("c"), "");

  ASSERT_EQ(run(rt, R"((app-focus "a"))"), "");
  pump(rt);
  EXPECT_EQ(peek(s.app, "ui.shell.ariadne.focus"), "includes.a");
  key(rt, "K");
  ASSERT_TRUE(pump_until(rt, [&] { return s.keys("a") == "K"; })) << s.keys("a");
  EXPECT_EQ(s.keys("b"), "");

  ASSERT_EQ(run(rt, R"((app-focus "b"))"), "");
  key(rt, "J");
  ASSERT_TRUE(pump_until(rt, [&] { return s.keys("b") == "J"; }));
  EXPECT_EQ(s.keys("a"), "K") << "only the focused one";

  // An application that does not hold delegate.keys gets none, focused or not.
  ASSERT_EQ(run(rt, R"((app-focus "c"))"), "");
  key(rt, "Z");
  pump(rt, 5);
  EXPECT_EQ(s.keys("c"), "");

  // The shell's own binding has Q: it never reaches the application.
  ASSERT_EQ(run(rt, R"((app-focus "b"))"), "");
  key(rt, "Q");
  ASSERT_TRUE(pump_until(rt, [&] { return peek(s.app, "ui.shell.shell_q") == "1"; }));
  pump(rt);
  EXPECT_EQ(s.keys("b"), "J");

  // The shell's search field has the keyboard: no key reaches the application.
  s.be.typing_in = "ui.shell.search";
  pump(rt, 1);
  key(rt, "X");
  pump(rt, 5);
  EXPECT_EQ(s.keys("b"), "J") << "typed into the shell's field, not the application";
  s.be.typing_in.clear();
  pump(rt, 1);
  key(rt, "Y");
  ASSERT_TRUE(pump_until(rt, [&] { return s.keys("b") == "JY"; })) << s.keys("b");
  EXPECT_EQ(s.keys("a"), "K");
}

// The focus follows the window that has it, when the backend says which (ImGui does): an
// application's window, or the shell's own and none, which take it off every application.
TEST(AriadneGrantsRuntime, TheFocusFollowsTheFocusedWindow) {
  ShellFixture s("ariadne_grants_window_focus", R"(meta: { name: Shell, min_libcvc: "0.0.0" }
windows:
  - window: Shell window
    id: shellwin
    children:
      - text: The shell's own
root:
  - mounts: apps
    grants: [delegate.keys, delegate.signals]
)");
  Runtime &rt = s.rt();
  s.be.reports = true;
  ASSERT_TRUE(pump_until(rt, [&] { return s.all_open(); }));
  // Both applications' windows are titled Taker: the backend tells them apart by the order the
  // walk draws them, a's first. Focus a's by giving it a title of its own for one frame at a time
  // is not possible here, so focus the shell's window, then an application's, by title.
  s.be.focused = "Shell window";
  pump(rt, 2);
  EXPECT_EQ(peek(s.app, "ui.shell.ariadne.focus"), "");
  s.be.focused = "Taker";
  pump(rt, 2);
  const std::string focus = peek(s.app, "ui.shell.ariadne.focus");
  EXPECT_TRUE(focus == "includes.a" || focus == "includes.b") << focus;
  key(rt, "W");
  const std::string as = focus.substr(focus.rfind('.') + 1);
  ASSERT_TRUE(pump_until(rt, [&] { return s.keys(as) == "W"; })) << focus;
  // (app-focus) moves it until the window focus moves again.
  const std::string other = as == "a" ? "b" : "a";
  ASSERT_EQ(run(rt, "(app-focus \"" + other + "\")"), "");
  key(rt, "E");
  ASSERT_TRUE(pump_until(rt, [&] { return s.keys(other) == "E"; }));
  s.be.focused.clear(); // the focus left every window: the main view has it
  pump(rt, 2);
  EXPECT_EQ(peek(s.app, "ui.shell.ariadne.focus"), "");
  key(rt, "R");
  pump(rt, 5);
  EXPECT_EQ(s.keys(as), "W");
  EXPECT_EQ(s.keys(other), "E");
}

// docs/roadmap/ARIADNE_SHELL.md 4.6: without an on_signal: of its own, the shell passes a request
// to end on to every application that holds delegate.signals (and the host's default still ends
// it), an interrupt to the focused one only, which takes it over.
TEST(AriadneGrantsRuntime, SigtermReachesEveryGrantedApplication) {
  ShellFixture s("ariadne_grants_signals");
  Runtime &rt = s.rt();
  ASSERT_TRUE(pump_until(rt, [&] { return s.all_open(); }));
  EXPECT_FALSE(rt.post_signal("SIGTERM", "os")) << "the host's default still ends the shell";
  ASSERT_TRUE(
      pump_until(rt, [&] { return s.signals("a") == "SIGTERM;" && s.signals("b") == "SIGTERM;"; }))
      << s.signals("a") << " / " << s.signals("b");
  EXPECT_EQ(s.signals("c"), "") << "it does not hold delegate.signals";

  EXPECT_FALSE(rt.post_signal("SIGINT", "terminal")) << "no application has the focus";
  ASSERT_EQ(run(rt, R"((app-focus "a"))"), "");
  pump(rt, 1); // the next drain takes the request
  EXPECT_TRUE(rt.post_signal("SIGINT", "terminal")) << "the focused application takes it over";
  ASSERT_TRUE(pump_until(rt, [&] { return s.signals("a") == "SIGTERM;SIGINT;"; }));
  EXPECT_EQ(s.signals("b"), "SIGTERM;") << "an interrupt is the focused one's alone";
}

// A slot that grants only some signals passes on only those; a shell with an on_signal: of its own
// passes them on itself, with (app-signal AS NAME).
TEST(AriadneGrantsRuntime, ASignalListLimitsWhatIsPassedOn) {
  ShellFixture s("ariadne_grants_signal_list", R"(meta: { name: Shell, min_libcvc: "0.0.0" }
root:
  - mounts: apps
    grants: [delegate.keys, {delegate.signals: [SIGTERM]}]
)");
  Runtime &rt = s.rt();
  ASSERT_TRUE(pump_until(rt, [&] { return s.all_open(); }));
  rt.post_signal("SIGHUP", "os");
  pump(rt, 5);
  EXPECT_EQ(s.signals("a"), "") << "SIGHUP is not in the slot's list";
  rt.post_signal("SIGTERM", "os");
  ASSERT_TRUE(pump_until(rt, [&] { return s.signals("a") == "SIGTERM;"; }));

  ShellFixture t("ariadne_grants_app_signal", R"(meta: { name: Shell, min_libcvc: "0.0.0" }
on_signal: (state-set "shell_got" (get-attr event "signal"))
root:
  - mounts: apps
    grants: [delegate.keys, delegate.signals]
)");
  Runtime &rt2 = t.rt();
  ASSERT_TRUE(pump_until(rt2, [&] { return t.all_open(); }));
  EXPECT_TRUE(rt2.post_signal("SIGTERM", "os")) << "the shell's own handler takes it over";
  ASSERT_TRUE(pump_until(rt2, [&] { return peek(t.app, "ui.shell.shell_got") == "SIGTERM"; }));
  pump(rt2, 3);
  EXPECT_EQ(t.signals("a"), "") << "the shell passes signals on itself";
  ASSERT_EQ(run(rt2, R"((app-signal "a" "SIGHUP"))"), "");
  ASSERT_TRUE(pump_until(rt2, [&] { return t.signals("a") == "SIGHUP;"; }));
  EXPECT_EQ(t.signals("b"), "");
}

// docs/roadmap/ARIADNE_SHELL.md 4.6: an entry's argv is parsed by the document's args: and argv and
// args.* are published under its sub-prefix before its init: runs; a bad one refuses the mount and
// posts its usage to the feed.
TEST(AriadneGrantsRuntime, ArgvIsParsedAndABadOneRefusedWithTheUsagePosted) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_argv";
  fs::remove_all(dir);
  const fs::path greeter =
      write_doc(dir, "greeter.ari", R"(meta: { name: Greeter, min_libcvc: "0.0.0" }
args:
  greeting: { type: string, default: hello, help: What it says }
  loud: bool
init: (state-set "said" (state-get "args.greeting"))
)");
  const fs::path shell = write_doc(dir, "shell.ari", R"(meta: { name: Shell, min_libcvc: "0.0.0" }
root:
  - mounts: apps
)");
  cvc::app app;
  AppRuntime art(app, "ui.shell");
  FocusBackend be;
  art.set_backend(be);
  art.load(shell.string());
  Runtime &rt = art.runtime();
  put(app, "ui.shell.apps.d.argv", "--greeting hi --loud");
  put(app, "ui.shell.apps.d.uri", greeter.generic_string());
  put(app, "ui.shell.apps.n.uri", greeter.generic_string()); // no argv: its defaults
  ASSERT_TRUE(pump_until(rt, [&] {
    return peek(app, "ui.shell.includes.d.said") == "hi" &&
           peek(app, "ui.shell.includes.n.said") == "hello";
  })) << peek(app, "ui.shell.apps.d.error");
  EXPECT_EQ(peek(app, "ui.shell.includes.d.args.loud"), "true");
  EXPECT_EQ(peek(app, "ui.shell.includes.d.argv"), "--greeting hi --loud");
  EXPECT_EQ(peek(app, "ui.shell.includes.n.args.loud"), "") << "a bool not given has no default";

  put(app, "ui.shell.apps.e.argv", "--nope");
  put(app, "ui.shell.apps.e.uri", greeter.generic_string());
  ASSERT_TRUE(pump_until(rt, [&] { return !peek(app, "ui.shell.apps.e.status").empty(); }));
  EXPECT_EQ(peek(app, "ui.shell.apps.e.status"), "refused");
  const std::string error = peek(app, "ui.shell.apps.e.error");
  EXPECT_NE(error.find("--nope"), std::string::npos) << error;
  EXPECT_NE(error.find("usage:"), std::string::npos) << error;
  EXPECT_TRUE(posted(app, "usage:")) << "the usage is in the feed";
  EXPECT_EQ(peek(app, "ui.shell.includes.e.said"), "") << "nothing of it ran";

  // Another argv is another run: closed, opened again, its init: run again.
  put(app, "ui.shell.apps.d.argv", "--greeting bonjour");
  ASSERT_TRUE(pump_until(rt, [&] { return peek(app, "ui.shell.includes.d.said") == "bonjour"; }));
  fs::remove_all(dir);
}

// (has? NAME) answers a grant token per mount: a mounted application's programs, its init:, its
// reactive expressions see what it holds; the document's own see what it would hold alone.
TEST(AriadneGrantsRuntime, HasAnswersEachMountsGrants) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_has";
  fs::remove_all(dir);
  const fs::path asker = write_doc(dir, "asker.ari", R"(meta: { name: Asker, min_libcvc: "0.0.0" }
requires:
  - {name: delegate.keys, optional: true}
  - {name: request.open, optional: true}
init: (state-set "init_keys" (if (has? "delegate.keys") "yes" "no"))
on_tick: (state-set "tick_keys" (if (has? "delegate.keys") "yes" "no"))
root:
  - text: shown
    bind: '(if (has? "delegate.keys") "keys yes" "keys no")'
)");
  const fs::path shell = write_doc(dir, "shell.ari", R"(meta: { name: Shell, min_libcvc: "0.0.0" }
requires: [{name: access.settings, optional: true}]
init: |
  (state-set "own" (str-concat (if (has? "delegate.keys") "y" "n")
                               (if (has? "access.settings") "y" "n")))
root:
  - mounts: apps
    grants: [delegate.keys]
)");
  cvc::app app;
  AppRuntime art(app, "ui.shell");
  FocusBackend be;
  art.set_backend(be);
  art.load(shell.string());
  Runtime &rt = art.runtime();
  EXPECT_EQ(peek(app, "ui.shell.own"), "yn") << "alone: delegate.* yes, access.* no";
  put(app, "ui.shell.apps.yes.uri", asker.generic_string());
  put(app, "ui.shell.apps.no.grants", "extend.tray");
  put(app, "ui.shell.apps.no.uri", asker.generic_string());
  ASSERT_TRUE(pump_until(rt, [&] {
    return peek(app, "ui.shell.includes.yes.tick_keys") == "yes" &&
           peek(app, "ui.shell.includes.no.tick_keys") == "no";
  }));
  EXPECT_EQ(peek(app, "ui.shell.includes.yes.init_keys"), "yes");
  EXPECT_EQ(peek(app, "ui.shell.includes.no.init_keys"), "no");
  // The read lane answers per scope too: each mount's bound expression its own.
  be.texts.clear();
  pump(rt, 1);
  EXPECT_TRUE(be.drew("shown: keys yes")) << "the granted one";
  EXPECT_TRUE(be.drew("shown: keys no")) << "the one that picked extend.tray alone";
  EXPECT_EQ(run(rt, R"((state-set "doc_keys" (if (has? "delegate.keys") "y" "n")))"), "");
  EXPECT_EQ(peek(app, "ui.shell.doc_keys"), "y");
  fs::remove_all(dir);
}

// 4.12 the net.origin form of requires:: the origins a mount asks for and its slot grants are what
// it holds, on its entry (SH11 reaches them: ScopePrincipal::net_origins).
TEST(AriadneGrantsRuntime, AMountHoldsTheOriginsItsSlotGrants) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_origins";
  fs::remove_all(dir);
  const fs::path client =
      write_doc(dir, "client.ari", R"(meta: { name: Client, min_libcvc: "0.0.0" }
requires:
  - {name: net.origin, origins: [https://api.example.org, "https://cdn.example.org:443/x"]}
)");
  const fs::path optional =
      write_doc(dir, "optional.ari", R"(meta: { name: Optional, min_libcvc: "0.0.0" }
requires:
  - {name: net.origin, optional: true, origins: [https://api.example.org, https://other.example]}
)");
  const fs::path shell = write_doc(dir, "shell.ari", R"(meta: { name: Shell, min_libcvc: "0.0.0" }
root:
  - mounts: apps
    grants: [{net.origin: [https://api.example.org, https://cdn.example.org]}]
)");
  cvc::app app;
  AppRuntime art(app, "ui.shell");
  FocusBackend be;
  art.set_backend(be);
  art.load(shell.string());
  Runtime &rt = art.runtime();
  put(app, "ui.shell.apps.c.uri", client.generic_string());
  put(app, "ui.shell.apps.o.uri", optional.generic_string());
  ASSERT_TRUE(pump_until(rt, [&] {
    return peek(app, "ui.shell.apps.c.status") == "open" &&
           peek(app, "ui.shell.apps.o.status") == "open";
  })) << peek(app, "ui.shell.apps.c.error");
  EXPECT_EQ(peek(app, "ui.shell.apps.c.net_origins"),
            "https://api.example.org,https://cdn.example.org");
  EXPECT_EQ(peek(app, "ui.shell.apps.o.net_origins"), "https://api.example.org")
      << "optional: what was granted of it";
  const std::vector<Runtime::MountState> all = rt.mounts();
  const Runtime::MountState *m = mount_named(all, "c");
  ASSERT_NE(m, nullptr);
  EXPECT_EQ(m->net_origins.size(), 2u);

  // A slot that grants one of them fewer refuses a document that requires both.
  const fs::path narrow =
      write_doc(dir, "narrow.ari", R"(meta: { name: Narrow, min_libcvc: "0.0.0" }
root:
  - mounts: apps
    grants: [{net.origin: [https://api.example.org]}]
)");
  cvc::app app2;
  AppRuntime art2(app2, "ui.shell");
  FocusBackend be2;
  art2.set_backend(be2);
  art2.load(narrow.string());
  put(app2, "ui.shell.apps.c.uri", client.generic_string());
  ASSERT_TRUE(
      pump_until(art2.runtime(), [&] { return !peek(app2, "ui.shell.apps.c.status").empty(); }));
  EXPECT_EQ(peek(app2, "ui.shell.apps.c.status"), "refused");
  EXPECT_NE(peek(app2, "ui.shell.apps.c.error").find("net.origin https://cdn.example.org"),
            std::string::npos)
      << peek(app2, "ui.shell.apps.c.error");
  fs::remove_all(dir);
}

// A static load: granted delegate.keys takes keys too, focused by (app-focus) with its id, and its
// keys: bindings run in its scope at their own keys.
TEST(AriadneGrantsRuntime, AStaticLoadTakesKeysWhenGranted) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_static";
  fs::remove_all(dir);
  write_doc(dir, "taker.ari", std::string(kTaker) + R"(keys:
  bump: { label: Bump, key: B, on: (state-set "bumped" "1") }
)");
  const fs::path doc = write_doc(dir, "doc.ari", R"(meta: { name: Doc, min_libcvc: "0.0.0" }
root:
  - load: taker.ari
    as: t
    grants: [delegate.keys, delegate.signals]
)");
  cvc::app app;
  AppRuntime art(app, "ui.doc");
  FocusBackend be;
  art.set_backend(be);
  art.load(doc.string());
  Runtime &rt = art.runtime();
  pump(rt, 2);
  EXPECT_EQ(peek(app, "ui.doc.includes.t.keys.bump"), "B") << "its binding's default, its own";
  ASSERT_EQ(run(rt, R"((app-focus "t"))"), "");
  key(rt, "B");
  ASSERT_TRUE(pump_until(rt, [&] { return peek(app, "ui.doc.includes.t.bumped") == "1"; }));
  EXPECT_EQ(peek(app, "ui.doc.includes.t.typed"), "B") << "its on_key: saw it too";
  EXPECT_FALSE(rt.post_signal("SIGTERM", "os"));
  ASSERT_TRUE(pump_until(rt, [&] { return peek(app, "ui.doc.includes.t.signals") == "SIGTERM;"; }));
  fs::remove_all(dir);
}

// extend.tray: a loaded application's tray icons join the document's tray when it holds it, which
// it is granted by default; deny: takes it away.
TEST(AriadneGrantsRuntime, ALoadedApplicationsTrayIconsNeedExtendTray) {
  const fs::path dir = fs::temp_directory_path() / "ariadne_grants_tray";
  fs::remove_all(dir);
  for (const char *n : {"TrayA", "TrayB"})
    write_doc(dir, std::string(n) + ".ari",
              std::string("meta: { name: ") + n + ", min_libcvc: \"0.0.0\" }\n" +
                  "menubar:\n  - menu: " + n + " things\n    items:\n      - menu_item: Hi\n" +
                  "  - tray_icon: " + n + "Icon\n");
  const fs::path doc = write_doc(dir, "doc.ari", R"(meta: { name: Doc, min_libcvc: "0.0.0" }
menubar:
  - menu: File
    items:
      - menu_item: New
root:
  - load: TrayA.ari
  - load: TrayB.ari
    deny: extend.tray
)");
  cvc::app app;
  AppRuntime art(app, "ui.doc");
  FocusBackend be;
  art.set_backend(be);
  art.load(doc.string());
  pump(art.runtime(), 2);
  EXPECT_TRUE(be.began_menu_with("TrayAIcon")) << "granted by default";
  EXPECT_FALSE(be.began_menu_with("TrayBIcon")) << "denied";
  EXPECT_TRUE(be.began_menu_with("TrayB")) << "its own application menu needs no token";
  fs::remove_all(dir);
}

// One document's tests pass alone and mounted (what `ari test` and `ari test --mounted` run, here
// with this backend in place of FTXUI's): it holds the same tokens, its args take the same
// defaults, its init: reads them the same.
TEST(AriadneGrantsRuntime, ADocumentsTestsPassAloneAndMounted) {
  const fs::path here = fs::path(__FILE__).parent_path();
  const fs::path doc = here.parent_path().parent_path() / "ari" / "tests" / "grants_component.ari";
  ASSERT_TRUE(fs::exists(doc)) << doc.string();
  const auto run_all =
      [](Runtime &rt, const std::vector<std::pair<std::string, std::string>> &tests,
         const std::function<void(const std::string &, std::function<void(const std::string &)>)>
             &start) {
        std::vector<std::string> failures;
        for (const auto &[name, program] : tests) {
          bool ended = false;
          std::string how;
          start(program, [&](const std::string &h) {
            how = h;
            ended = true;
          });
          if (!pump_until(rt, [&] { return ended; }))
            failures.push_back(name + ": did not end");
          else if (!how.empty())
            failures.push_back(name + ": " + how);
        }
        return failures;
      };
  { // alone
    cvc::app app;
    AppRuntime art(app, "ui.docs.ari");
    FocusBackend be;
    art.set_backend(be);
    art.set_document_args({}, "ari test", /*validate=*/false);
    const std::vector<std::string> warnings = art.load(doc.string());
    EXPECT_TRUE(warnings.empty()) << warnings.front();
    Runtime &rt = art.runtime();
    pump(rt, 2);
    ASSERT_EQ(art.tests().size(), 2u);
    const std::vector<std::string> failed =
        run_all(rt, art.tests(),
                [&](const std::string &p, auto done) { rt.run_program(p, std::move(done)); });
    EXPECT_TRUE(failed.empty()) << "alone: " << failed.front();
  }
  { // mounted, as ari test --mounted makes it
    const fs::path dir = fs::temp_directory_path() / "ariadne_grants_mounted";
    fs::remove_all(dir);
    std::string grants;
    for (const GrantToken &t : grant_tokens())
      if (held_alone(t.name))
        grants += std::string(grants.empty() ? "" : ", ") + t.name;
    const fs::path shell = write_doc(dir, "shell.ari",
                                     "meta: { name: ari test --mounted, min_libcvc: \"0.0.0\" }\n"
                                     "root:\n  - mounts: ari_test.apps\n    grants: [" +
                                         grants + "]\n");
    cvc::app app;
    AppRuntime art(app, "ui.docs.ari");
    FocusBackend be;
    art.set_backend(be);
    put(app, "ui.docs.ari.ari_test.apps.doc.uri", doc.generic_string());
    art.load(shell.string());
    Runtime &rt = art.runtime();
    ASSERT_TRUE(pump_until(rt, [&] {
      const std::vector<Runtime::MountState> all = rt.mounts();
      const Runtime::MountState *m = mount_named(all, "doc");
      return m && (m->mounted || !m->refused.empty());
    }));
    const Runtime::MountState m = *mount_named(rt.mounts(), "doc");
    ASSERT_TRUE(m.mounted) << m.refused;
    EXPECT_TRUE(m.warnings.empty()) << m.warnings.front();
    ASSERT_EQ(m.tests.size(), 2u);
    const std::vector<std::string> failed =
        run_all(rt, m.tests, [&](const std::string &p, auto done) {
          EXPECT_TRUE(rt.run_program_in_mount("doc", p, std::move(done)));
        });
    EXPECT_TRUE(failed.empty()) << "mounted: " << failed.front();
    fs::remove_all(dir);
  }
}

// No singletons: two apps side by side, each with its shell and applications, keep their own focus,
// grants and input. A key and a signal posted to one never reach the other's applications.
TEST(AriadneGrantsRuntime, TwoAppsKeepTheirOwnFocusAndInput) {
  ShellFixture one("ariadne_grants_two_1");
  ShellFixture two("ariadne_grants_two_2");
  ASSERT_TRUE(pump_until(one.rt(), [&] { return one.all_open(); }));
  ASSERT_TRUE(pump_until(two.rt(), [&] { return two.all_open(); }));
  ASSERT_EQ(run(one.rt(), R"((app-focus "a"))"), "");
  ASSERT_EQ(run(two.rt(), R"((app-focus "b"))"), "");
  key(one.rt(), "1");
  key(two.rt(), "2");
  ASSERT_TRUE(pump_until(one.rt(), [&] { return one.keys("a") == "1"; }));
  ASSERT_TRUE(pump_until(two.rt(), [&] { return two.keys("b") == "2"; }));
  pump(one.rt());
  pump(two.rt());
  EXPECT_EQ(one.keys("b"), "");
  EXPECT_EQ(two.keys("a"), "");
  EXPECT_EQ(peek(one.app, "ui.shell.ariadne.focus"), "includes.a");
  EXPECT_EQ(peek(two.app, "ui.shell.ariadne.focus"), "includes.b");
  one.rt().post_signal("SIGTERM", "os");
  ASSERT_TRUE(pump_until(one.rt(), [&] { return one.signals("a") == "SIGTERM;"; }));
  pump(two.rt(), 5);
  EXPECT_EQ(two.signals("a") + two.signals("b"), "") << "the other app's applications";
}
