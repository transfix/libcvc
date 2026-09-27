// Comprehensive tests for the Ariadne .ari loader (cvc::ariadne): widget parsing,
// the meta/min_libcvc gate, semantic validation, and §3.0.3b size/layout/frame.
//
// Parsing tests are skipped when the build has no yaml-cpp (the loader is then a
// stub); the version/schema surface is tested unconditionally.

#include <boost/asio.hpp> // a localhost server for the http handler test
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cvc/ariadne/ariadne.h> // register_widget_type (customs gate tests)
#include <cvc/ariadne/loader.h>
#include <cvc/ariadne/state_io.h>  // §13.10 save_state / restore_state
#include <cvc/ariadne/uri.h>       // §13 resolver (import routes through it)
#include <cvc/ariadne/uri_http.h>  // §13.6 http:// handler
#include <cvc/ariadne/uri_state.h> // §13.3 state:// handler
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>   // cvc::app (state:// handler tests build a state tree)
#include <cvc/core/state.h> // cvc::state
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>

using namespace cvc::ariadne;

namespace {

// Depth-first search for the first widget of a kind, optionally matching a label.
const Widget *find(const Widget &w, Kind k, const std::string &label = std::string()) {
  if (w.kind == k && (label.empty() || w.label == label))
    return &w;
  for (const Widget &c : w.children)
    if (const Widget *r = find(c, k, label))
      return r;
  return nullptr;
}

int count(const Widget &w, Kind k) {
  int n = (w.kind == k) ? 1 : 0;
  for (const Widget &c : w.children)
    n += count(c, k);
  return n;
}

bool has_warning(const LoadResult &r, const std::string &needle) {
  for (const std::string &w : r.warnings)
    if (w.find(needle) != std::string::npos)
      return true;
  return false;
}

// First widget in the tree whose mount scope equals `scope` (§12 load: wrapper).
const Widget *find_scope(const Widget &w, const std::string &scope) {
  if (w.scope == scope)
    return &w;
  for (const Widget &c : w.children)
    if (const Widget *r = find_scope(c, scope))
      return r;
  return nullptr;
}

// Write a temp .ari file (under a shared test dir) and return its path — for §12 import tests.
std::string write_temp_ari(const std::string &name, const std::string &content) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_import_test";
  fs::create_directories(dir);
  const fs::path p = dir / name;
  std::ofstream(p) << content;
  return p.string();
}

#define SKIP_WITHOUT_YAML()                                                                        \
  do {                                                                                             \
    if (!have_yaml())                                                                              \
      GTEST_SKIP() << "libcvc built without yaml-cpp";                                             \
  } while (0)

// A throwaway one-shot localhost HTTP server: binds an OS-assigned port on 127.0.0.1 (listening
// immediately, so a client that connects before accept() is queued), then on a background thread
// serves ONE connection a fixed 200 response with `body`. Join it via done(). For the http tests.
class OneShotHttpServer {
public:
  explicit OneShotHttpServer(std::string body)
      : acceptor_(io_,
                  boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0)),
        body_(std::move(body)) {
    port_ = acceptor_.local_endpoint().port();
    thread_ = std::thread([this] {
      boost::system::error_code ec;
      boost::asio::ip::tcp::socket s(io_);
      acceptor_.accept(s, ec);
      if (ec)
        return;
      char buf[4096];
      s.read_some(boost::asio::buffer(buf), ec); // consume the request line/headers
      const std::string resp =
          "HTTP/1.1 200 OK\r\nContent-Length: " + std::to_string(body_.size()) +
          "\r\nConnection: close\r\n\r\n" + body_;
      boost::asio::write(s, boost::asio::buffer(resp), ec);
      s.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    });
  }
  unsigned short port() const { return port_; }
  ~OneShotHttpServer() {
    if (thread_.joinable())
      thread_.join();
  }

private:
  boost::asio::io_context io_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::string body_;
  unsigned short port_ = 0;
  std::thread thread_;
};

// A localhost HTTP/1.1 server that CAPTURES one request (method line, headers, and the full body
// per Content-Length) for the write/header tests, then replies 200. Unlike OneShotHttpServer it
// drains the whole request across TCP segments so the PUT/POST body is complete. Read request()
// only after take_request() has joined the server thread (the join is the happens-before for
// request_).
class CapturingHttpServer {
public:
  // `reply` is the 200 body. `raw_reply`, if non-empty, is sent VERBATIM instead of the 200 (used
  // to return a 3xx redirect and prove the handler refuses to follow it). Both are fixed at
  // construction — set before the server thread reads them, so there is no data race with the
  // accept loop.
  explicit CapturingHttpServer(std::string reply = "OK", std::string raw_reply = std::string())
      : acceptor_(io_,
                  boost::asio::ip::tcp::endpoint(boost::asio::ip::make_address("127.0.0.1"), 0)),
        reply_(std::move(reply)), raw_reply_(std::move(raw_reply)) {
    port_ = acceptor_.local_endpoint().port();
    thread_ = std::thread([this] { run(); });
  }
  unsigned short port() const { return port_; }
  // Join the server thread, then return the full raw request it captured.
  std::string take_request() {
    if (thread_.joinable())
      thread_.join();
    return request_;
  }
  ~CapturingHttpServer() {
    if (thread_.joinable())
      thread_.join();
  }

private:
  static std::size_t content_length_of(const std::string &head) {
    std::string low = head;
    for (char &c : low)
      c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    const std::size_t k = low.find("content-length:");
    if (k == std::string::npos)
      return 0;
    std::size_t p = k + std::string("content-length:").size();
    while (p < head.size() && (head[p] == ' ' || head[p] == '\t'))
      ++p;
    std::size_t len = 0;
    bool any = false;
    for (; p < head.size() && head[p] >= '0' && head[p] <= '9'; ++p) {
      len = len * 10 + static_cast<std::size_t>(head[p] - '0');
      any = true;
    }
    return any ? len : 0;
  }
  void run() {
    boost::system::error_code ec;
    boost::asio::ip::tcp::socket s(io_);
    acceptor_.accept(s, ec);
    if (ec)
      return;
    std::string data;
    std::size_t header_end = std::string::npos;
    std::size_t content_len = 0;
    bool have_headers = false;
    char buf[4096];
    for (;;) {
      const std::size_t n = s.read_some(boost::asio::buffer(buf), ec);
      if (n)
        data.append(buf, n);
      if (!have_headers) {
        header_end = data.find("\r\n\r\n");
        if (header_end != std::string::npos) {
          have_headers = true;
          content_len = content_length_of(data.substr(0, header_end));
        }
      }
      if (have_headers && data.size() - (header_end + 4) >= content_len)
        break;
      if (ec) // connection closed / error before we saw the whole message
        break;
    }
    request_ = std::move(data);
    const std::string resp =
        !raw_reply_.empty()
            ? raw_reply_
            : "HTTP/1.1 200 OK\r\nContent-Length: " + std::to_string(reply_.size()) +
                  "\r\nConnection: close\r\n\r\n" + reply_;
    boost::asio::write(s, boost::asio::buffer(resp), ec);
    s.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
  }
  boost::asio::io_context io_;
  boost::asio::ip::tcp::acceptor acceptor_;
  std::string reply_;
  std::string raw_reply_;
  std::string request_;
  unsigned short port_ = 0;
  std::thread thread_;
};

// The URI registry is process-global; a state:// handler captures a state root by pointer.
// Unregister it when the test scopes out — even on an ASSERT early-return — so a later test's
// resolve() cannot dispatch into a freed root. Declare AFTER the app so it destructs first.
struct StateHandlerGuard {
  ~StateHandlerGuard() { unregister_state_uri_handler(); } // tears down BOTH read + write handlers
};

// Tear down BOTH the http/https read and write handlers when a test scopes out (register_http_uri_
// handler now registers all four) — even on an ASSERT early-return — so the process-global store
// handlers do not leak into a later test.
struct HttpHandlerGuard {
  ~HttpHandlerGuard() {
    unregister_uri_handler("http");
    unregister_uri_handler("https");
    unregister_uri_store_handler("http");
    unregister_uri_store_handler("https");
  }
};

// Clear the process-global HTTP options provider on scope-out so a test's auth-header injection
// cannot bleed into another test's request.
struct HttpProviderGuard {
  ~HttpProviderGuard() { set_http_options_provider(nullptr); }
};

// Unregister an arbitrary test-registered scheme when the test scopes out (even on an ASSERT) —
// crucial for a "file" override, which would otherwise break every later file resolve.
struct SchemeGuard {
  std::string scheme;
  ~SchemeGuard() { unregister_uri_handler(scheme); }
};

// Snapshot + restore the process-global file byte caps, so a test that lowers a cap cannot leak
// it into later tests (which store/resolve real files at the default cap).
struct CapGuard {
  std::size_t r = resolve_file_byte_cap();
  std::size_t s = store_file_byte_cap();
  ~CapGuard() {
    set_resolve_file_byte_cap(r);
    set_store_file_byte_cap(s);
  }
};

} // namespace

// ---- version + schema surface (no yaml needed) ----------------------------

TEST(AriadneVersion, AtLeast) {
  EXPECT_TRUE(version_at_least("3.4.0", "3.4.0"));
  EXPECT_TRUE(version_at_least("3.4.0", "3.3.9"));
  EXPECT_TRUE(version_at_least("3.4.0", "3.4"));
  EXPECT_TRUE(version_at_least("3.4.1", "3.4.0"));
  EXPECT_TRUE(version_at_least("4.0.0", "3.9.9"));
  EXPECT_FALSE(version_at_least("3.4.0", "3.4.1"));
  EXPECT_FALSE(version_at_least("3.4.0", "9.0.0"));
  EXPECT_FALSE(version_at_least("3.4.0", "3.5"));
  // Tolerates a leading 'v' and a pre-release/build suffix.
  EXPECT_TRUE(version_at_least("v3.4.0", "3.4.0"));
  EXPECT_TRUE(version_at_least("3.4.0-rc1", "3.4.0"));
}

TEST(AriadneVersion, LibcvcAndSchema) {
  EXPECT_FALSE(libcvc_version().empty());
  EXPECT_NE(libcvc_version(), "3.0.0"); // the stale fallback is gone (§17/config.h fix)
  EXPECT_NE(ari_schema_json().find("min_libcvc"), std::string::npos);
  EXPECT_NE(ari_schema_json().find("$schema"), std::string::npos);
}

// ---- widget parsing --------------------------------------------------------

TEST(AriadneLoader, FullDocumentStructure) {
  SKIP_WITHOUT_YAML();
  const char *doc = R"(
meta: { name: T, min_libcvc: "3.4.0" }
menubar:
  - menu: Sim
    items:
      - menu_item: Paused
        bind: demo.paused
      - separator
      - menu_item: Quit
        on: quit
windows:
  - window: Controls
    id: ctrl
    children:
      - text: "hello"
      - slider_int: N
        bind: demo.n
        lo: 1
        hi: 100
        def: 10
      - combo: Belief
        bind: demo.b
        options: [a, b, c]
        default: b
      - checkbox: Wire
        bind: demo.w
      - button: Go
        on: go
)";
  LoadResult r = load_string(doc);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.meta.name, "T");
  EXPECT_EQ(r.meta.min_libcvc, "3.4.0");
  EXPECT_EQ(count(r.root, Kind::Menubar), 1);
  EXPECT_EQ(count(r.root, Kind::Menu), 1);
  EXPECT_EQ(count(r.root, Kind::MenuItemToggle), 1);
  EXPECT_EQ(count(r.root, Kind::MenuItemAction), 1);
  EXPECT_EQ(count(r.root, Kind::Separator), 1);
  EXPECT_EQ(count(r.root, Kind::Window), 1);
  EXPECT_EQ(count(r.root, Kind::SliderInt), 1);
  EXPECT_EQ(count(r.root, Kind::Combo), 1);
  EXPECT_EQ(count(r.root, Kind::Checkbox), 1);
  EXPECT_EQ(count(r.root, Kind::Button), 1);

  const Widget *win = find(r.root, Kind::Window);
  ASSERT_NE(win, nullptr);
  EXPECT_EQ(win->id, "ctrl");

  const Widget *si = find(r.root, Kind::SliderInt);
  ASSERT_NE(si, nullptr);
  EXPECT_EQ(si->bind, "demo.n");
  EXPECT_EQ(si->ilo, 1);
  EXPECT_EQ(si->ihi, 100);
  EXPECT_EQ(si->idef, 10);

  const Widget *cb = find(r.root, Kind::Combo);
  ASSERT_NE(cb, nullptr);
  ASSERT_EQ(cb->options.size(), 3u);
  EXPECT_EQ(cb->options[1], "b");
  EXPECT_EQ(cb->sdef, "b");

  const Widget *tg = find(r.root, Kind::MenuItemToggle);
  ASSERT_NE(tg, nullptr);
  EXPECT_EQ(tg->bind, "demo.paused");

  const Widget *bt = find(r.root, Kind::Button);
  ASSERT_NE(bt, nullptr);
  EXPECT_EQ(bt->on, "go");
}

// §4/§7 action lane: a program-shaped `on:` (an s-expression, like a computed bind:) is captured
// on Widget::on VERBATIM — quotes and all — so the runtime can run it through state_exec at drain.
// A bare event name stays a bare name; the runtime tells them apart by the leading '('.
TEST(AriadneLoader, ProgramOnCapturedVerbatim) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("windows:\n  - window: W\n    children:\n      - button: Toggle\n    "
                             "    on: (state-set \"paused\" \"true\")\n");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *b = find(r.root, Kind::Button);
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->on, "(state-set \"paused\" \"true\")"); // s-expr survives YAML intact
}

// G6: a combo `values:` list parses onto the widget parallel to `options:`; a length mismatch warns
// and drops the mapping (fail-safe to storing the label text).
TEST(AriadneLoader, ComboValuesParse) {
  SKIP_WITHOUT_YAML();
  LoadResult ok =
      load_string("windows:\n  - window: W\n    children:\n      - combo: Quality\n     "
                  "   bind: q\n        options: [Low, High]\n        values: [0, 2]\n");
  ASSERT_TRUE(ok.ok) << ok.error;
  const Widget *c = find(ok.root, Kind::Combo);
  ASSERT_NE(c, nullptr);
  ASSERT_EQ(c->values.size(), 2u);
  EXPECT_EQ(c->values[1], "2"); // a YAML int stringified
  // A mismatched length is ignored (with a warning) — the combo stays a text combo.
  LoadResult bad = load_string("windows:\n  - window: W\n    children:\n      - combo: Q\n        "
                               "bind: q\n        options: [a, b, c]\n        values: [0, 1]\n");
  ASSERT_TRUE(bad.ok) << bad.error;
  const Widget *cb = find(bad.root, Kind::Combo);
  ASSERT_NE(cb, nullptr);
  EXPECT_TRUE(cb->values.empty()); // mapping dropped on mismatch
}

// §G7: a `color:` widget parses to Kind::Color, bound to its "r,g,b" key with an optional default.
TEST(AriadneLoader, ColorWidgetParses) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("windows:\n  - window: W\n    children:\n      - color: Tint\n        "
                             "bind: mat.tint\n        default: \"1,0.5,0\"\n");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *c = find(r.root, Kind::Color);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->bind, "mat.tint");
  EXPECT_EQ(c->sdef, "1,0.5,0");
}

// §raster viewer: an `image:` widget parses to Kind::Image with a static src, a display size, and
// an optional bind (the image name from a key).
TEST(AriadneLoader, ImageWidgetParses) {
  SKIP_WITHOUT_YAML();
  LoadResult r =
      load_string("windows:\n  - window: W\n    children:\n      - image: Belief\n       "
                  " src: belief\n        size: 320\n        bind: raster.layer\n");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *im = find(r.root, Kind::Image);
  ASSERT_NE(im, nullptr);
  EXPECT_EQ(im->src, "belief");
  EXPECT_EQ(im->bind, "raster.layer");
  EXPECT_DOUBLE_EQ(im->img_size, 320.0);
}

TEST(AriadneLoader, BareScalarsAndLiteralVsBoundText) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    children:
      - separator
      - text: "a literal caption"
      - text: "value ="
        bind: demo.v
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(count(r.root, Kind::Separator), 1);
  // Two Text widgets: one literal, one bound.
  EXPECT_EQ(count(r.root, Kind::Text), 2);
  const Widget *win = find(r.root, Kind::Window);
  ASSERT_NE(win, nullptr);
  bool saw_literal = false, saw_bound = false;
  for (const Widget &c : win->children) {
    if (c.kind == Kind::Text && c.literal_text)
      saw_literal = true;
    if (c.kind == Kind::Text && !c.literal_text && c.bind == "demo.v")
      saw_bound = true;
  }
  EXPECT_TRUE(saw_literal);
  EXPECT_TRUE(saw_bound);
}

// ---- the min_libcvc gate ---------------------------------------------------

TEST(AriadneLoaderGate, Passes) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("meta: { min_libcvc: \"0.0.1\" }\nwindows: []\n");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.error.empty());
}

TEST(AriadneLoaderGate, FailsWhenTooNew) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("meta: { min_libcvc: \"999.0.0\" }\nwindows: []\n");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("requires libcvc"), std::string::npos);
  // A failed gate never builds a tree.
  EXPECT_TRUE(r.root.children.empty());
}

TEST(AriadneLoaderGate, MissingMinLibcvcIsWarningNotFatal) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("windows: [ { window: W, children: [] } ]\n");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "min_libcvc"));
}

// ---- semantic validation (Layer 3) ----------------------------------------

TEST(AriadneLoaderValidation, Warnings) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: Bad
    children:
      - slider_int: NoBind
        lo: 10
        hi: 5
      - combo: Empty
        bind: demo.c
      - button: Dud
      - frobnicate: Whatsit
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "no bind"));                 // slider with no bind
  EXPECT_TRUE(has_warning(r, "lo >= hi"));                // bad slider range
  EXPECT_TRUE(has_warning(r, "no options"));              // combo with no options
  EXPECT_TRUE(has_warning(r, "no on: action"));           // button with no action
  EXPECT_TRUE(has_warning(r, "unrecognized widget key")); // frobnicate
}

TEST(AriadneLoaderValidation, UnknownExplicitType) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
root:
  - type: hologram
    intensity: 0.7
)");
  ASSERT_TRUE(r.ok) << r.error;
  // An unknown `type:` is now PRESERVED as a Kind::Custom widget (not collapsed to a
  // group), carrying its props, with a warning that no handler is registered.
  EXPECT_TRUE(has_warning(r, "no registered handler"));
  const Widget *c = find(r.root, Kind::Custom);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->custom_type, "hologram");
  EXPECT_DOUBLE_EQ(c->props.num("intensity", -1.0), 0.7);
}

TEST(AriadneLoaderValidation, CustomWidgetKeepsChildrenAndTypeNamedProp) {
  SKIP_WITHOUT_YAML();
  // The anti-lossy guarantee: a custom widget's nested children survive (the old
  // fallback dropped them), and a prop whose key equals the type value is kept.
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
root:
  - type: gauge
    gauge: 0.8
    children:
      - text: "a"
      - button: Go
        on: x
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *c = find(r.root, Kind::Custom);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->custom_type, "gauge");
  EXPECT_DOUBLE_EQ(c->props.num("gauge", -1.0), 0.8); // prop named like the type kept
  ASSERT_EQ(c->children.size(), 2u);                  // children NOT dropped
  EXPECT_EQ(c->children[1].kind, Kind::Button);
}

// ---- errors + forward-compat ----------------------------------------------

TEST(AriadneLoaderErrors, MalformedYaml) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("windows: [ { window: W,,, ] : bad");
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.error.empty());
}

TEST(AriadneLoaderErrors, EmptyDocLoads) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string("");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.root.children.empty());
}

TEST(AriadneLoaderErrors, UnknownTopLevelKeysIgnored) {
  SKIP_WITHOUT_YAML();
  // meta carries an unknown field; a future top-level key is ignored (forward-compat).
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0", future_field: 42 }
some_future_section: { a: 1 }
windows: [ { window: W, children: [] } ]
)");
  EXPECT_TRUE(r.ok) << r.error;
  EXPECT_EQ(count(r.root, Kind::Window), 1);
}

// ---- §3.0.3b size / layout / frame ----------------------------------------

TEST(AriadneLayout, WindowSizePercentPxAutoAndMinMax) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: Inspector
    size: { hint: ["50%", "100%"], min: [280, 0], max: ["80%", 900] }
    frame: { border: 2 }
    children: []
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->size.w.unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->size.w.value, 50.0f);
  EXPECT_EQ(w->size.h.unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->size.h.value, 100.0f);
  EXPECT_EQ(w->size.min_w.unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->size.min_w.value, 280.0f);
  EXPECT_EQ(w->size.min_h.unit, Unit::Auto); // 0 with a min sequence -> px 0? see note
  EXPECT_EQ(w->size.max_w.unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->size.max_w.value, 80.0f);
  EXPECT_EQ(w->size.max_h.unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->size.max_h.value, 900.0f);
  EXPECT_FLOAT_EQ(w->frame_border, 2.0f);
}

TEST(AriadneLayout, SizeSequenceAndFractionPercent) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    size: [0.25, 300]
    children: []
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->size.w.unit, Unit::Percent); // 0.25 fraction -> 25%
  EXPECT_FLOAT_EQ(w->size.w.value, 25.0f);
  EXPECT_EQ(w->size.h.unit, Unit::Px); // 300 -> px
  EXPECT_FLOAT_EQ(w->size.h.value, 300.0f);
}

TEST(AriadneLayout, GridColTracksBordersResizable) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    layout:
      kind: grid
      col_widths: [200, "auto", "30%"]
      resizable: true
      borders: { show: inner, color: [0.3, 0.3, 0.35, 1.0] }
    children: [ { text: "a" }, { text: "b" }, { text: "c" } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->layout.kind, LayoutKind::Grid);
  ASSERT_EQ(w->layout.col_widths.size(), 3u);
  EXPECT_EQ(w->layout.col_widths[0].unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->layout.col_widths[0].value, 200.0f);
  EXPECT_EQ(w->layout.col_widths[1].unit, Unit::Auto);
  EXPECT_EQ(w->layout.col_widths[2].unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->layout.col_widths[2].value, 30.0f);
  EXPECT_TRUE(w->layout.resizable);
  EXPECT_EQ(w->layout.borders, BorderShow::Inner);
  EXPECT_TRUE(w->layout.has_border_color);
  EXPECT_FLOAT_EQ(w->layout.border_color[3], 1.0f);
  EXPECT_FALSE(w->layout.is_row_split()); // no row tracks
}

TEST(AriadneLayout, RowTracksAndRowSplit) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows:
  - window: W
    layout:
      kind: grid
      row_heights: ["30%", 120, "auto"]
      resizable: true
    children: [ { text: "top" }, { text: "mid" }, { text: "bot" } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  ASSERT_EQ(w->layout.row_heights.size(), 3u);
  EXPECT_EQ(w->layout.row_heights[0].unit, Unit::Percent);
  EXPECT_FLOAT_EQ(w->layout.row_heights[0].value, 30.0f);
  EXPECT_EQ(w->layout.row_heights[1].unit, Unit::Px);
  EXPECT_FLOAT_EQ(w->layout.row_heights[1].value, 120.0f);
  EXPECT_EQ(w->layout.row_heights[2].unit, Unit::Auto);
  // resizable + row tracks -> a row split-pane layout (§3.0.3b increment 2).
  EXPECT_TRUE(w->layout.is_row_split());
  EXPECT_TRUE(w->layout.is_set());
}

TEST(AriadneLayout, GroupCanBeAGrid) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
root:
  - type: group
    layout: { kind: grid, col_widths: ["50%", "50%"] }
    children: [ { text: "l" }, { text: "r" } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *g = find(r.root, Kind::Group, "");
  // The document root is itself a Group; find a nested grid Group.
  bool found_grid = false;
  std::function<void(const Widget &)> walk = [&](const Widget &w) {
    if (w.kind == Kind::Group && w.layout.kind == LayoutKind::Grid &&
        w.layout.col_widths.size() == 2)
      found_grid = true;
    for (const Widget &c : w.children)
      walk(c);
  };
  walk(r.root);
  (void)g;
  EXPECT_TRUE(found_grid);
}

TEST(AriadneLayout, NoLayoutMeansPlainVertical) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows: [ { window: W, children: [ { text: "x" } ] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *w = find(r.root, Kind::Window);
  ASSERT_NE(w, nullptr);
  EXPECT_EQ(w->layout.kind, LayoutKind::Vertical);
  EXPECT_FALSE(w->layout.is_set());
  EXPECT_FALSE(w->size.any());
  EXPECT_LT(w->frame_border, 0.0f); // unset
}

// ---------------------------------------------------------------------------
// §9 scene binding — the `scene:` block parses into LoadResult::scene.
// ---------------------------------------------------------------------------

namespace {
const SceneNode *find_scene_node(const std::vector<SceneNode> &nodes, const std::string &id) {
  for (const SceneNode &n : nodes) {
    if (n.id == id)
      return &n;
    if (const SceneNode *r = find_scene_node(n.children, id))
      return r;
  }
  return nullptr;
}
} // namespace

TEST(AriadneScene, NoSceneBlockIsEmpty) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
windows: [ { window: W, children: [ { text: "x" } ] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(r.scene.any());
  EXPECT_TRUE(r.scene.nodes.empty());
  EXPECT_TRUE(r.scene.lights.empty());
}

TEST(AriadneScene, GeometryNodeSourceMaterialTransform) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: bunny
      type: geometry
      source: { file: bunny.obj }
      material: { color: [0.9, 0.1, 0.2], ambient: 0.3, diffuse: 0.7 }
      transform: { position: [1, 2, 3], rotation: [0, 90, 0], scale: [2, 2, 2] }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_TRUE(r.scene.any());
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.id, "bunny");
  EXPECT_EQ(n.type, "geometry");
  EXPECT_EQ(n.source_file, "bunny.obj");
  ASSERT_TRUE(n.has_material);
  EXPECT_FLOAT_EQ(n.color[0], 0.9f);
  EXPECT_FLOAT_EQ(n.color[2], 0.2f);
  EXPECT_FLOAT_EQ(n.ambient, 0.3f);
  EXPECT_FLOAT_EQ(n.diffuse, 0.7f);
  ASSERT_TRUE(n.has_transform);
  EXPECT_FLOAT_EQ(n.position[0], 1.0f);
  EXPECT_FLOAT_EQ(n.position[2], 3.0f);
  EXPECT_FLOAT_EQ(n.rotation[1], 90.0f);
  EXPECT_FLOAT_EQ(n.scale[0], 2.0f);
  EXPECT_TRUE(n.visible_default); // default visible, no bind
  EXPECT_TRUE(n.visible_bind.empty());
}

// §9 enrichments: a procedural plane primitive, specular/single_color material, and a `fit:` block.
TEST(AriadneScene, PlanePrimitiveSpecularAndFit) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: ground
      type: geometry
      source: { plane: { size: 440 } }
      material: { color: [0.3, 0.34, 0.38], ambient: 0.35, diffuse: 0.85 }
    - node: bunny
      type: geometry
      source: { file: stanford.bunny }
      material: { color: [0.85, 0.82, 0.88], specular: 0.25, specular_power: 24, single_color: true }
      fit: { height: 100, up: y }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 2u);
  const SceneNode *g = find_scene_node(r.scene.nodes, "ground");
  ASSERT_NE(g, nullptr);
  EXPECT_EQ(g->source_primitive, "plane");
  EXPECT_FLOAT_EQ(g->plane_size, 440.0f);
  EXPECT_TRUE(g->source_file.empty()); // a primitive has no file
  const SceneNode *b = find_scene_node(r.scene.nodes, "bunny");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->source_file, "stanford.bunny");
  EXPECT_TRUE(b->has_specular);
  EXPECT_FLOAT_EQ(b->specular, 0.25f);
  EXPECT_FLOAT_EQ(b->specular_power, 24.0f);
  EXPECT_TRUE(b->use_single_color);
  ASSERT_TRUE(b->has_fit);
  EXPECT_FLOAT_EQ(b->fit_height, 100.0f);
  EXPECT_TRUE(b->fit_up_y);
}

// §9 enrichments: a tuned StageLighting rig and shadow resolution/update-interval.
TEST(AriadneScene, RigTuningAndShadowResolution) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  lights:
    - light: studio
      rig: three_point
      stage: { center: [0, 0, 50], radius: 62 }
      key: { intensity: 1.9, azimuth: -38, elevation: 52, cone: 34 }
      fill: 0.85
      back: 0.6
      warmth: 0.3
      environment: 0.7
      ambient: 0.4
  shadows: { enabled: true, resolution: 2048, update_interval: 1 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.lights.size(), 1u);
  const SceneLight &l = r.scene.lights[0];
  EXPECT_EQ(l.rig, "three_point");
  ASSERT_TRUE(l.has_stage);
  EXPECT_FLOAT_EQ(l.stage_center[2], 50.0f);
  EXPECT_FLOAT_EQ(l.stage_radius, 62.0f);
  ASSERT_TRUE(l.has_key);
  EXPECT_FLOAT_EQ(l.key_intensity, 1.9f);
  EXPECT_FLOAT_EQ(l.key_azimuth, -38.0f);
  EXPECT_FLOAT_EQ(l.key_cone, 34.0f);
  EXPECT_TRUE(l.has_fill);
  EXPECT_FLOAT_EQ(l.fill, 0.85f);
  EXPECT_TRUE(l.has_back);
  EXPECT_TRUE(l.has_warmth);
  EXPECT_TRUE(l.has_environment);
  EXPECT_TRUE(l.has_rig_ambient);
  EXPECT_FLOAT_EQ(l.rig_ambient, 0.4f);
  EXPECT_TRUE(r.scene.has_shadows);
  EXPECT_TRUE(r.scene.shadows_enabled);
  ASSERT_TRUE(r.scene.has_shadow_resolution);
  EXPECT_EQ(r.scene.shadow_resolution, 2048);
  ASSERT_TRUE(r.scene.has_shadow_interval);
  EXPECT_EQ(r.scene.shadow_interval, 1);
}

// §9 background: a solid colour or a top→bottom gradient (a view property the host applies).
TEST(AriadneScene, BackgroundSolidAndGradient) {
  SKIP_WITHOUT_YAML();
  LoadResult solid =
      load_string("meta: { min_libcvc: \"0.0.0\" }\nscene:\n  background: [0.2, 0.3, "
                  "0.4]\n  nodes:\n    - node: g\n      source: { plane: { size: 4 } "
                  "}\n");
  ASSERT_TRUE(solid.ok) << solid.error;
  ASSERT_TRUE(solid.scene.has_background);
  EXPECT_FALSE(solid.scene.background_gradient);
  EXPECT_FLOAT_EQ(solid.scene.background_top[0], 0.2f);
  EXPECT_FLOAT_EQ(solid.scene.background_bottom[2], 0.4f); // solid: top == bottom
  LoadResult grad = load_string(
      "meta: { min_libcvc: \"0.0.0\" }\nscene:\n  background: { top: [0.1, 0.12, 0.16], "
      "bottom: [0.0, 0.0, 0.02] }\n  nodes:\n    - node: g\n      source: { plane: { "
      "size: 4 } }\n");
  ASSERT_TRUE(grad.ok) << grad.error;
  ASSERT_TRUE(grad.scene.has_background);
  EXPECT_TRUE(grad.scene.background_gradient);
  EXPECT_FLOAT_EQ(grad.scene.background_top[1], 0.12f);
  EXPECT_FLOAT_EQ(grad.scene.background_bottom[2], 0.02f);
}

// §9 chrome: strip the SceneGraph diagnostic grid/axis for a clean capture.
TEST(AriadneScene, ChromeToggle) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  chrome: false
  nodes:
    - node: g
      source: { plane: { size: 4 } }
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.scene.any());
  ASSERT_TRUE(r.scene.has_chrome);
  EXPECT_FALSE(r.scene.chrome_visible);
}

// The reusable-component pattern: a `units:` window template `include:`d at the top level (the
// `windows:` list) expands to that window — the mechanism a component library leans on (a document
// `import:`s a component file then `include:`s its window unit). Same-document here; cross-file
// import is covered by AriadneModularity.ImportUnitsOverHttp.
TEST(AriadneModularity, IncludeWindowUnitAtTopLevel) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
units:
  controls:
    window: Controls
    id: controls
    children:
      - slider_float: Gain
        bind: lighting.key_intensity
        lo: 0
        hi: 4
windows:
  - include: controls
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *win = find(r.root, Kind::Window);
  ASSERT_NE(win, nullptr);
  EXPECT_EQ(win->label, "Controls"); // the unit expanded into a real window
  const Widget *sl = find(r.root, Kind::SliderFloat);
  ASSERT_NE(sl, nullptr);
  EXPECT_EQ(sl->bind, "lighting.key_intensity"); // the component's binding survived the include
}

// A PARAMETRIZED component: `include: <unit>` with `args:` substitutes `{token}` into the unit's
// binds at load time — so one component drives a chosen node/viewer (e.g. a per-volume-node volren
// panel). This is the reuse unlock: the same unit, retargeted per include.
TEST(AriadneModularity, ParametrizedIncludeSubstitutesBind) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
units:
  volren_shadows:
    window: Shadows
    children:
      - checkbox: Enabled
        bind: graphics.root.children.{node}.volren.shadows.enabled
      - slider_int: Map
        bind: graphics.root.children.{node}.volren.shadows.resolution
        lo: 256
        hi: 4096
windows:
  - include: volren_shadows
    args: { node: bunny_volume }
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *cb = find(r.root, Kind::Checkbox);
  ASSERT_NE(cb, nullptr);
  EXPECT_EQ(cb->bind,
            "graphics.root.children.bunny_volume.volren.shadows.enabled"); // {node} filled
  const Widget *sl = find(r.root, Kind::SliderInt);
  ASSERT_NE(sl, nullptr);
  EXPECT_EQ(sl->bind, "graphics.root.children.bunny_volume.volren.shadows.resolution");
}

TEST(AriadneScene, ScalarScaleIsUniform) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: s
      source: { file: s.obj }
      transform: { scale: 3 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  ASSERT_TRUE(n.has_transform);
  EXPECT_FLOAT_EQ(n.scale[0], 3.0f);
  EXPECT_FLOAT_EQ(n.scale[1], 3.0f);
  EXPECT_FLOAT_EQ(n.scale[2], 3.0f);
  EXPECT_EQ(n.type, "geometry"); // default type
}

TEST(AriadneScene, VisibleLiteralVsBind) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: hidden
      source: { file: a.obj }
      visible: false
    - node: bound
      source: { file: b.obj }
      visible: demo.show_mesh
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 2u);
  const SceneNode *hidden = find_scene_node(r.scene.nodes, "hidden");
  ASSERT_NE(hidden, nullptr);
  EXPECT_FALSE(hidden->visible_default);
  EXPECT_TRUE(hidden->visible_bind.empty());
  const SceneNode *bound = find_scene_node(r.scene.nodes, "bound");
  ASSERT_NE(bound, nullptr);
  EXPECT_EQ(bound->visible_bind, "demo.show_mesh");
}

TEST(AriadneScene, VisibleMapFormBindAndDefault) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: hiddenbound
      source: { file: a.obj }
      visible: { bind: demo.show, default: false }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.visible_bind, "demo.show"); // bound...
  EXPECT_FALSE(n.visible_default);        // ...AND starts hidden (map form)
}

TEST(AriadneScene, GroupNestsChildren) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: convoy
      type: group
      children:
        - node: truck1
          source: { file: truck.obj }
        - node: truck2
          source: { file: truck.obj }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &g = r.scene.nodes[0];
  EXPECT_EQ(g.type, "group");
  ASSERT_EQ(g.children.size(), 2u);
  EXPECT_EQ(g.children[0].id, "truck1");
  EXPECT_NE(find_scene_node(r.scene.nodes, "truck2"), nullptr);
}

TEST(AriadneScene, NestedChildCarriesLocalTransform) {
  SKIP_WITHOUT_YAML();
  // A child under a transformed parent keeps its OWN transform; the realizer nests
  // the child under the parent so this becomes a LOCAL transform (world = parent ∘
  // child). Here we assert the parse carries both independently.
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: convoy
      type: group
      transform: { position: [100, 0, 0] }
      children:
        - node: truck
          source: { file: truck.obj }
          transform: { position: [5, 0, 0] }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &convoy = r.scene.nodes[0];
  ASSERT_TRUE(convoy.has_transform);
  EXPECT_FLOAT_EQ(convoy.position[0], 100.0f);
  ASSERT_EQ(convoy.children.size(), 1u);
  const SceneNode &truck = convoy.children[0];
  ASSERT_TRUE(truck.has_transform);
  EXPECT_FLOAT_EQ(truck.position[0], 5.0f); // local to the convoy, not 105
}

TEST(AriadneScene, LightsAndShadows) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: g
      source: { file: g.obj }
  lights:
    - light: key
      kind: spot
      pos: [10, 20, 30]
      target: [1, 2, 3]
      cone: 30
      intensity: 1.5
    - light: soft
      rig: three_point
  shadows: { enabled: true }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.lights.size(), 2u);
  const SceneLight &key = r.scene.lights[0];
  EXPECT_EQ(key.id, "key");
  EXPECT_EQ(key.kind, "spot");
  EXPECT_FLOAT_EQ(key.pos[1], 20.0f);
  // Non-default target so a dropped/mistyped `target` key can't hide (the realizer
  // feeds these into LightNode::setTarget for spot/fill).
  EXPECT_FLOAT_EQ(key.target[0], 1.0f);
  EXPECT_FLOAT_EQ(key.target[1], 2.0f);
  EXPECT_FLOAT_EQ(key.target[2], 3.0f);
  EXPECT_FLOAT_EQ(key.cone, 30.0f);
  EXPECT_FLOAT_EQ(key.intensity, 1.5f);
  EXPECT_EQ(r.scene.lights[1].rig, "three_point");
  EXPECT_TRUE(r.scene.has_shadows);
  EXPECT_TRUE(r.scene.shadows_enabled);
}

TEST(AriadneScene, LightColorAzimuthElevation) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  lights:
    - light: sun
      kind: directional
      azimuth: 135
      elevation: 60
      color: [1.0, 0.9, 0.7]
      intensity: 0.8
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.lights.size(), 1u);
  const SceneLight &l = r.scene.lights[0];
  EXPECT_EQ(l.kind, "directional");
  EXPECT_FLOAT_EQ(l.azimuth, 135.0f);
  EXPECT_FLOAT_EQ(l.elevation, 60.0f);
  EXPECT_FLOAT_EQ(l.color[0], 1.0f);
  EXPECT_FLOAT_EQ(l.color[1], 0.9f);
  EXPECT_FLOAT_EQ(l.color[2], 0.7f);
  EXPECT_FLOAT_EQ(l.intensity, 0.8f);
}

TEST(AriadneScene, LightColorDefaultsWhite) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  lights: [ { light: fill, kind: fill, cone: 20 } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.lights.size(), 1u);
  const SceneLight &l = r.scene.lights[0];
  EXPECT_FLOAT_EQ(l.color[0], 1.0f); // default white
  EXPECT_FLOAT_EQ(l.color[1], 1.0f);
  EXPECT_FLOAT_EQ(l.color[2], 1.0f);
  EXPECT_FLOAT_EQ(l.elevation, 45.0f); // LightNode's own directional default
}

TEST(AriadneScene, VolumeNodeParses) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: vol
      type: volume
      source: { file: head.rawiv }
      transform: { scale: 2 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.type, "volume");
  EXPECT_EQ(n.source_file, "head.rawiv");
  ASSERT_TRUE(n.has_transform);
  EXPECT_FLOAT_EQ(n.scale[0], 2.0f);
  // An unstyled node must report has_material==false: the realizer applies
  // setAmbient/setDiffuse only then, so an unstyled volume keeps VolumeNode's tuned
  // transfer-function defaults. Pin the default so a parse regression can't silently
  // style every node.
  EXPECT_FALSE(n.has_material);
}

TEST(AriadneScene, VolRenNodeParses) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: vr
      type: volren
      source: { file: head.rawiv }
      volren:
        ambient: 0.3
        distance_field: true
        steps: 256
        backend: cpu
        isosurfaces:
          - { value: 0.5, opacity: 0.9, color: [0.9, 0.2, 0.1], shininess: 40 }
        transfer_function:
          window: [0.0, 1.0]
          points:
            - { value: 0.0, color: [0, 0, 0, 0] }
            - { value: 1.0, color: [1, 1, 1, 1] }
        lights:
          - { color: [1, 1, 1], direction: [0, 0, 1] }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.type, "volren");
  ASSERT_TRUE(n.has_volren);
  EXPECT_FLOAT_EQ(n.volren.ambient, 0.3f);
  EXPECT_TRUE(n.volren.distance_field);
  EXPECT_EQ(n.volren.steps, 256);
  ASSERT_EQ(n.volren.isosurfaces.size(), 1u);
  EXPECT_DOUBLE_EQ(n.volren.isosurfaces[0].value, 0.5); // value domain is double
  EXPECT_FLOAT_EQ(n.volren.isosurfaces[0].opacity, 0.9f);
  EXPECT_FLOAT_EQ(n.volren.isosurfaces[0].shininess, 40.0f);
  ASSERT_EQ(n.volren.tf.points.size(), 2u);
  EXPECT_TRUE(n.volren.tf.has_window);
  EXPECT_FALSE(n.volren.tf.auto_domain); // an explicit window fixes the domain
  EXPECT_DOUBLE_EQ(n.volren.tf.window_max, 1.0);
  EXPECT_FLOAT_EQ(n.volren.tf.points[1].color[3], 1.0f); // alpha
  ASSERT_EQ(n.volren.lights.size(), 1u);
  EXPECT_FLOAT_EQ(n.volren.lights[0].direction[2], 1.0f);
}

TEST(AriadneScene, VolSliceNodeParses) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: vs
      type: volslice
      source: { file: head.rawiv }
      volslice:
        quality: 0.75
        near_plane: 0.1
        filter: nearest
        opacity_correction: true
        transfer_function:
          points:
            - { value: 0.0, color: [1, 0, 0, 0] }
            - { value: 1.0, color: [1, 0, 0, 0.8] }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.type, "volslice");
  ASSERT_TRUE(n.has_volslice);
  EXPECT_FLOAT_EQ(n.volslice.quality, 0.75f);
  EXPECT_FLOAT_EQ(n.volslice.near_plane, 0.1f);
  EXPECT_TRUE(n.volslice.nearest_filter);
  EXPECT_TRUE(n.volslice.opacity_correction);
  ASSERT_EQ(n.volslice.tf.points.size(), 2u);
  EXPECT_TRUE(n.volslice.tf.auto_domain); // no window given
  EXPECT_FLOAT_EQ(n.volslice.tf.points[1].color[3], 0.8f);
}

TEST(AriadneScene, WidgetsAndSceneCoexist) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: mesh
      source: { file: m.obj }
windows:
  - window: Controls
    children:
      - checkbox: Show
        bind: demo.show_mesh
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.scene.nodes.size(), 1u);
  const Widget *cb = find(r.root, Kind::Checkbox);
  ASSERT_NE(cb, nullptr);
  EXPECT_EQ(cb->bind, "demo.show_mesh");
}

// ---------------------------------------------------------------------------
// Extensibility — custom scene-node props bag + custom top-level blocks.
// ---------------------------------------------------------------------------

TEST(AriadneScene, CustomNodeTypeCapturesProps) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: swarm1
      type: swarm
      count: 128
      color: [0.2, 0.4, 0.9]
      behavior: { mode: flock, speed: 2.5 }
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const SceneNode &n = r.scene.nodes[0];
  EXPECT_EQ(n.type, "swarm");
  // Unknown keys land in props (a Map); known keys (node/type/...) do NOT.
  ASSERT_TRUE(n.props.is_map());
  EXPECT_DOUBLE_EQ(n.props.num("count", 0), 128.0);
  EXPECT_EQ(n.props.find("type"), nullptr); // a known key is not duplicated
  EXPECT_EQ(n.props.find("node"), nullptr);
  const Value *behavior = n.props.find("behavior"); // nested map
  ASSERT_NE(behavior, nullptr);
  EXPECT_EQ(behavior->str("mode"), "flock");
  EXPECT_DOUBLE_EQ(behavior->num("speed", 0), 2.5);
  const Value *color = n.props.find("color"); // a sequence
  ASSERT_NE(color, nullptr);
  ASSERT_TRUE(color->is_seq());
  ASSERT_EQ(color->items.size(), 3u);
  EXPECT_DOUBLE_EQ(color->items[1].as_double(), 0.4);
}

TEST(AriadneScene, PropsAccessorsMatchBuiltinSemantics) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes:
    - node: g
      type: gadget
      good: 2.5
      bad: 10px
      caption: ""
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  const Value &p = r.scene.nodes[0].props;
  EXPECT_DOUBLE_EQ(p.num("good", -1.0), 2.5);
  EXPECT_DOUBLE_EQ(p.num("bad", -1.0),
                   -1.0); // "10px" is not a whole number -> default (strict, like num())
  EXPECT_DOUBLE_EQ(p.num("missing", 7.0), 7.0); // absent -> default
  EXPECT_EQ(p.str("caption", "fallback"), "");  // an explicit "" overrides the default (like str())
  EXPECT_EQ(p.str("missing", "fallback"), "fallback"); // absent -> default
}

TEST(AriadneScene, BuiltinNodeHasEmptyProps) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
scene:
  nodes: [ { node: m, type: geometry, source: { file: m.obj } } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.scene.nodes.size(), 1u);
  EXPECT_TRUE(r.scene.nodes[0].props.empty()); // all keys consumed by the built-in
}

TEST(AriadneLoader, CustomTopLevelBlockRegistered) {
  SKIP_WITHOUT_YAML();
  // Capture-free parser (the registry is process-global and outlives this test).
  register_ari_block("theme", [](const Value &content, LoadResult &out) {
    out.extras.emplace_back("theme", content);
    out.warnings.push_back("theme applied: " + content.str("name"));
  });
  EXPECT_TRUE(has_ari_block("theme"));
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
theme:
  name: midnight
  accent: [0.1, 0.2, 0.3]
windows: [ { window: W, children: [ { text: "x" } ] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.extras.size(), 1u);
  EXPECT_EQ(r.extras[0].first, "theme");
  EXPECT_EQ(r.extras[0].second.str("name"), "midnight");
  const Value *accent = r.extras[0].second.find("accent");
  ASSERT_NE(accent, nullptr);
  EXPECT_TRUE(accent->is_seq());
  bool warned = false;
  for (const std::string &w : r.warnings)
    if (w.find("theme applied: midnight") != std::string::npos)
      warned = true;
  EXPECT_TRUE(warned);
  EXPECT_NE(find(r.root, Kind::Window), nullptr); // the built-in windows block still parsed
}

TEST(AriadneLoader, RegisterBuiltinBlockIsIgnored) {
  register_ari_block("scene", [](const Value &, LoadResult &) {}); // built-in: ignored
  EXPECT_FALSE(has_ari_block("scene"));
}

// ---------------------------------------------------------------------------
// The `customs:` gate — declare custom types the doc uses; fail fast on a
// missing REQUIRED one, warn on a missing non-required one (default).
// ---------------------------------------------------------------------------

TEST(AriadneCustoms, MissingRequiredWidgetFailsLoad) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - widget: definitely_unregistered_xyz
    required: true
windows: [ { window: W, children: [] } ]
)");
  EXPECT_FALSE(r.ok); // fail fast
  EXPECT_NE(r.error.find("requires custom widget"), std::string::npos);
  EXPECT_TRUE(r.root.children.empty()); // no tree built on a hard custom failure
}

TEST(AriadneCustoms, MissingNonRequiredWidgetWarns) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - widget: some_optional_widget
windows: [ { window: W, children: [] } ]
)");
  ASSERT_TRUE(r.ok) << r.error; // non-required missing -> loads
  EXPECT_TRUE(has_warning(r, "optional custom widget"));
  ASSERT_EQ(r.customs.size(), 1u);
  EXPECT_EQ(r.customs[0].name, "some_optional_widget");
  EXPECT_FALSE(r.customs[0].required); // default is NOT required
}

TEST(AriadneCustoms, RegisteredWidgetSatisfiesRequirement) {
  SKIP_WITHOUT_YAML();
  register_widget_type("customs_ok_widget", [](const Widget &, const WidgetEmitContext &) {});
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - widget: customs_ok_widget
    required: true
windows: [ { window: W, children: [] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(has_warning(r, "custom widget")); // registered -> neither error nor warning
}

TEST(AriadneCustoms, NodeCustomIsDeferredNotCheckedAtLoad) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - node: unregistered_node_type
    required: true
)");
  // Node types register on the cvcGL side, so the loader records the requirement but
  // does NOT check it — cvc::gl::ariadne::verify_scene_customs does, before realize.
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_EQ(r.customs.size(), 1u);
  EXPECT_EQ(r.customs[0].kind, CustomRequirement::Kind::Node);
  EXPECT_TRUE(r.customs[0].required);
}

TEST(AriadneCustoms, MissingRequiredBlockFailsLoad) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - block: definitely_unregistered_block_xyz
    required: true
windows: [ { window: W, children: [] } ]
)");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("requires custom block"), std::string::npos);
}

TEST(AriadneCustoms, RegisteredBlockSatisfiesRequirement) {
  SKIP_WITHOUT_YAML();
  register_ari_block("customs_ok_block", [](const Value &, LoadResult &) {});
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - block: customs_ok_block
    required: true
windows: [ { window: W, children: [] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
}

TEST(AriadneCustoms, RequiredHonorsCanonicalYamlBooleanSpelling) {
  SKIP_WITHOUT_YAML();
  // `required: True` (capital) is a valid YAML boolean — it MUST still fail fast, not
  // silently downgrade to a warning (the safety-gate case-sensitivity fix).
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - widget: definitely_unregistered_qq
    required: True
)");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("requires custom widget"), std::string::npos);
}

TEST(AriadneCustoms, MalformedCustomsBlockWarnsNotSilent) {
  SKIP_WITHOUT_YAML();
  // A customs: written as a MAP (dropped the sequence dash) must not be silently
  // dropped — it warns, so a botched required: declaration is visible.
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  widget: labeled
  required: true
)");
  ASSERT_TRUE(r.ok) << r.error; // ignored, not fatal — but warned
  EXPECT_TRUE(has_warning(r, "not a sequence"));
  EXPECT_TRUE(r.customs.empty());
}

TEST(AriadneCustoms, MultipleKindKeysIgnoredWithWarning) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
customs:
  - widget: a
    node: b
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "more than one"));
  EXPECT_TRUE(r.customs.empty()); // the ambiguous entry is dropped
}

TEST(AriadneInit, BlockCapturedNotRun) {
  SKIP_WITHOUT_YAML();
  // The loader only CAPTURES the init: script verbatim (it has no app, never runs it).
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
init: |
  (state-set "demo.agents" "128")
windows: [ { window: W, children: [] } ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(r.init_script.find("state-set"), std::string::npos);
}

TEST(AriadneInit, NonScalarInitWarns) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
meta: { min_libcvc: "0.0.0" }
init: [ not, a, script ]
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(r.init_script.empty());
  EXPECT_TRUE(has_warning(r, "init: must be a scalar"));
}

// ---- §4 read-lane: visible_when parsing ------------------------------------

TEST(AriadneReactive, VisibleWhenParsedOntoWidget) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - text: hello
        visible_when: (> (int (state-get "n")) 5)
      - text: plain
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *hello = find(r.root, Kind::Text, "hello");
  ASSERT_NE(hello, nullptr);
  EXPECT_EQ(hello->visible_when, "(> (int (state-get \"n\")) 5)");
  const Widget *plain = find(r.root, Kind::Text, "plain");
  ASSERT_NE(plain, nullptr);
  EXPECT_TRUE(plain->visible_when.empty()); // absent -> empty (always visible)
}

TEST(AriadneReactive, EnabledAndDisabledWhenParsedOntoWidget) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - button: Go
        on: go
        enabled_when: (state-exists "ready")
        disabled_when: (state-exists "busy")
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *b = find(r.root, Kind::Button, "Go");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->enabled_when, "(state-exists \"ready\")");
  EXPECT_EQ(b->disabled_when, "(state-exists \"busy\")");
}

TEST(AriadneReactive, ComboOptionsScalarIsExpressionSequenceIsStatic) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - combo: Static
        bind: a
        options: [x, y, z]
      - combo: Computed
        bind: b
        options: (list "p" "q")
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *s = find(r.root, Kind::Combo, "Static");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->options.size(), 3u);
  EXPECT_TRUE(s->options_expr.empty());
  const Widget *c = find(r.root, Kind::Combo, "Computed");
  ASSERT_NE(c, nullptr);
  EXPECT_TRUE(c->options.empty());
  EXPECT_EQ(c->options_expr, "(list \"p\" \"q\")");
}

// ---- §12 modularization: units + include ----------------------------------

TEST(AriadneModularity, IncludeExpandsUnitWithArgs) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
units:
  aslider:
    slider_int: "{label}"
    bind: "{path}"
    lo: 0
    hi: 100
windows:
  - window: W
    children:
      - include: aslider
        args: { label: Agents, path: demo.agents }
      - include: aslider
        args: { label: Speed, path: demo.speed }
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(count(r.root, Kind::SliderInt), 2); // the unit instantiated twice
  const Widget *a = find(r.root, Kind::SliderInt, "Agents");
  ASSERT_NE(a, nullptr);
  EXPECT_EQ(a->bind, "demo.agents"); // args substituted per instance
  const Widget *s = find(r.root, Kind::SliderInt, "Speed");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->bind, "demo.speed");
}

TEST(AriadneModularity, IncludeUnknownUnitWarns) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - include: nope
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "unknown unit")); // surfaced, not silently dropped
}

TEST(AriadneModularity, NestedIncludeExpands) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
units:
  inner:
    checkbox: "{name}"
    bind: "flags.{name}"
  outer:
    group:
    children:
      - include: inner
        args: { name: wire }
windows:
  - window: W
    children:
      - include: outer
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *cb = find(r.root, Kind::Checkbox, "wire");
  ASSERT_NE(cb, nullptr); // a unit that includes another unit expands both
  EXPECT_EQ(cb->bind, "flags.wire");
}

TEST(AriadneModularity, RecursiveIncludeIsDepthGuarded) {
  SKIP_WITHOUT_YAML();
  // A unit that includes itself must terminate (depth guard), not hang or overflow.
  LoadResult r = load_string(R"(
units:
  loopy:
    group:
    children:
      - include: loopy
windows:
  - window: W
    children:
      - include: loopy
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "recursive unit")); // refused (name-based cycle guard), terminates
}

TEST(AriadneModularity, IncludedUnitReadLaneFieldsSurvive) {
  SKIP_WITHOUT_YAML();
  // A unit's own visible_when/repeat/… must NOT be erased by the include node (which has none).
  LoadResult r = load_string(R"(
units:
  gated:
    text: secret
    visible_when: (state-exists "show")
windows:
  - window: W
    children:
      - include: gated
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *t = find(r.root, Kind::Text, "secret");
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->visible_when, "(state-exists \"show\")"); // preserved, not clobbered
}

TEST(AriadneModularity, IncludeArgsHandleSpecialCharsAndNoDoubleSubstitution) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
units:
  row:
    text: "{a}"
    bind: "{b}"
windows:
  - window: W
    children:
      - include: row
        args: { a: '{b}', b: 'x"y' }
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *t = find(r.root, Kind::Text);
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->label, "{b}"); // single pass: {a}->'{b}', the value is NOT re-substituted to x"y
  EXPECT_EQ(t->bind, "x\"y"); // a value with a double-quote survives (no dump/reparse hazard)
}

// ---- §13 URI resolver (no yaml needed) -------------------------------------

TEST(AriadneUri, ParsesSchemesAndBarePaths) {
  Uri f = parse_uri("panels/rf.ari"); // bare -> file
  EXPECT_EQ(f.scheme, "file");
  EXPECT_EQ(f.path, "panels/rf.ari");
  Uri s = parse_uri("state://ui.nav.current?value");
  EXPECT_EQ(s.scheme, "state");
  EXPECT_EQ(s.path, "ui.nav.current");
  EXPECT_EQ(s.query, "value");
  Uri h = parse_uri("HTTPS://example.com/x.ari"); // scheme lowercased
  EXPECT_EQ(h.scheme, "https");
}

TEST(AriadneUri, FileHandlerReadsContent) {
  const std::string path = write_temp_ari("uri_probe.txt", "hello-uri");
  UriResult r = resolve(path); // bare path -> built-in file handler
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "hello-uri");
  EXPECT_FALSE(r.canonical.empty());
}

TEST(AriadneUri, UnknownSchemeHasNoHandler) {
  EXPECT_TRUE(has_uri_handler("file")); // built-in
  EXPECT_FALSE(has_uri_handler("nosuch"));
  UriResult r = resolve("nosuch://x");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("no handler"), std::string::npos);
}

TEST(AriadneUri, RegisterHandlerDispatches) {
  register_uri_handler("memtest", [](const Uri &u, const std::string &) {
    return UriResult{true, "content-for-" + u.path, "memtest:" + u.path, ""};
  });
  EXPECT_TRUE(has_uri_handler("memtest"));
  UriResult r = resolve("memtest://abc");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.content, "content-for-abc");
}

TEST(AriadneUri, RelativePathAnchorsToBaseElseCwd) {
  namespace fs = std::filesystem;
  // No base (a string-loaded document has no source location) -> anchor to the CWD, not left
  // dangling as a relative path.
  const std::string cwd_anchored = resolve_file_path("sub/x.ari", "");
  EXPECT_TRUE(fs::path(cwd_anchored).is_absolute());
  EXPECT_EQ(cwd_anchored, fs::weakly_canonical(fs::current_path() / "sub" / "x.ari").string());
  // With a base directory, anchor there (base == CWD here, so the two agree).
  const std::string base_anchored = resolve_file_path("sub/x.ari", fs::current_path().string());
  EXPECT_EQ(base_anchored, cwd_anchored);
  // An absolute path stands alone regardless of base.
  EXPECT_EQ(resolve_file_path("/abs/x.ari", "/some/dir"),
            fs::weakly_canonical(fs::path("/abs/x.ari")).string());
}

TEST(AriadneUri, FileNameWithQuestionMarkIsNotSplit) {
  // '?' is a legal POSIX filename byte, so a bare/file path is never split on it (a query
  // component belongs only to schemes that use one — state/http/custom).
  Uri u = parse_uri("data?v2.ari");
  EXPECT_EQ(u.scheme, "file");
  EXPECT_EQ(u.path, "data?v2.ari");
  EXPECT_TRUE(u.query.empty());
  // Round-trip through the file handler where the platform allows '?' in a name.
  const std::string p = write_temp_ari("q?mark.txt", "qm-content");
  if (std::filesystem::exists(p)) {
    UriResult r = resolve(p);
    ASSERT_TRUE(r.ok) << r.error;
    EXPECT_EQ(r.content, "qm-content");
  }
}

TEST(AriadneUri, FileHandlerRejectsDirectoryWithoutThrowing) {
  // A directory opens but throws on read (libstdc++); resolve() must return an error, not
  // terminate. The test process surviving this call is itself the assertion.
  const std::string probe = write_temp_ari("dir_probe.txt", "x"); // ensures the dir exists
  const std::string dir = std::filesystem::path(probe).parent_path().string();
  UriResult r = resolve(dir);
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("directory"), std::string::npos);
}

TEST(AriadneUri, FileExceedingCapIsRejectedByBoundedRead) {
  // The cap is enforced during the read, not by a pre-read stat, so a file larger than the cap
  // is rejected regardless of what stat reports. A sparse file makes this cheap to construct.
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_import_test";
  fs::create_directories(dir);
  const fs::path big = dir / "toobig.bin";
  {
    std::ofstream o(big, std::ios::binary);
    o.seekp(static_cast<std::streamoff>(17) * 1024 * 1024); // 17 MiB > the 16 MiB cap
    o.put('\0');
  }
  if (fs::exists(big) && fs::file_size(big) > 16u * 1024u * 1024u) {
    UriResult r = resolve(big.string());
    EXPECT_FALSE(r.ok);
    EXPECT_NE(r.error.find("exceeds"), std::string::npos);
  }
  std::error_code ec;
  fs::remove(big, ec);
}

// The cvc:// scheme resolves a component name against a search path (env + extra dirs + cwd), first
// hit wins — the location-independent import path for the shared .ari component library.
TEST(AriadneUri, CvcSchemeResolvesFromSearchPath) {
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() / "ariadne_cvc_test";
  const fs::path comps = root / "components";
  fs::create_directories(comps);
  {
    std::ofstream o(comps / "unit.ari");
    o << "units:\n  cvc_ctl: { text: FromCvc }\n";
  }
  register_cvc_uri_handler(
      {root.string()}); // root is a search dir -> cvc://components/... resolves
  SchemeGuard g{"cvc"};
  UriResult r = resolve("cvc://components/unit.ari");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(r.content.find("FromCvc"), std::string::npos);
  EXPECT_FALSE(
      r.canonical.empty()); // the found file's absolute path (relative imports resolve off it)
  UriResult miss = resolve("cvc://components/nope.ari");
  EXPECT_FALSE(miss.ok);
  EXPECT_NE(miss.error.find("not found"), std::string::npos); // a clear diagnostic, not a crash
  std::error_code ec;
  fs::remove_all(root, ec);
}

// End-to-end: a document imports a component by cvc:// URI and includes its unit — the
// library-import path a shipped .ari uses (import: cvc://components/foo.ari), location-independent.
TEST(AriadneModularity, ImportComponentViaCvcScheme) {
  SKIP_WITHOUT_YAML();
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() / "ariadne_cvc_import";
  const fs::path comps = root / "components";
  fs::create_directories(comps);
  {
    std::ofstream o(comps / "panel.ari");
    o << "units:\n  panel:\n    window: P\n    children:\n      - text: FromComponent\n";
  }
  register_cvc_uri_handler({root.string()});
  SchemeGuard g{"cvc"};
  LoadResult r = load_string("import: cvc://components/panel.ari\nwindows:\n  - include: panel\n");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *win = find(r.root, Kind::Window);
  ASSERT_NE(win, nullptr);
  EXPECT_EQ(win->label, "P");
  EXPECT_NE(find(r.root, Kind::Text, "FromComponent"), nullptr);
  std::error_code ec;
  fs::remove_all(root, ec);
}

TEST(AriadneUri, ResolveToFileFileSchemeIsInPlace) {
  const std::string p = write_temp_ari("srcprobe.off", "OFF-DATA");
  ResolvedFile rf = resolve_to_file(p);
  ASSERT_TRUE(rf.ok) << rf.error;
  // file:// (bare) -> the resolved path itself, no temp copy.
  EXPECT_NE(rf.path.find("srcprobe.off"), std::string::npos);
  EXPECT_TRUE(std::filesystem::exists(rf.path));
}

TEST(AriadneUri, ResolveToFileBytesSchemeSpillsToTempThenCleansUp) {
  register_uri_handler("membytes", [](const Uri &u, const std::string &) {
    return UriResult{true, "GEOM-BYTES", "membytes:" + u.path, std::string()};
  });
  std::string temp_path;
  {
    ResolvedFile rf = resolve_to_file("membytes://mesh.obj");
    ASSERT_TRUE(rf.ok) << rf.error;
    temp_path = rf.path;
    EXPECT_NE(temp_path.find(".obj"), std::string::npos); // source extension preserved
    std::ifstream in(temp_path, std::ios::binary);
    std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    EXPECT_EQ(content, "GEOM-BYTES");
  }
  EXPECT_FALSE(std::filesystem::exists(temp_path)); // temp removed on ResolvedFile destruction
}

TEST(AriadneUri, HttpHandlerFetchesOverLocalhost) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  OneShotHttpServer server("hello-over-http");
  register_http_uri_handler();
  EXPECT_TRUE(has_uri_handler("http"));
  EXPECT_TRUE(has_uri_handler("https"));
  EXPECT_TRUE(has_uri_store_handler("http")); // the write side registers alongside the reader
  EXPECT_TRUE(has_uri_store_handler("https"));
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/x";
  UriResult r = resolve(url);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "hello-over-http");
  EXPECT_FALSE(r.canonical.empty());
}

TEST(AriadneUri, HttpFetchFailureIsAnErrorNotACrash) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  register_http_uri_handler();
  // Port 1 has nothing listening -> connection refused -> a clean error, never a throw/crash.
  UriResult r = resolve("http://127.0.0.1:1/nope");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("http fetch"), std::string::npos);
}

TEST(AriadneModularity, ImportUnitsOverHttp) {
  SKIP_WITHOUT_YAML();
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  // A units library served over http, imported through the full §13 resolver.
  OneShotHttpServer server("units:\n  http_ctl: { text: FromHttp }\n");
  register_http_uri_handler();
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/lib.ari";
  LoadResult r = load_string(
      "import: " + url + "\nwindows:\n  - window: W\n    children:\n      - include: http_ctl\n");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find(r.root, Kind::Text, "FromHttp"), nullptr); // the http-served unit expanded
}

TEST(AriadneUri, HttpResolveSendsProviderAuthHeader) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  HttpProviderGuard pg;
  CapturingHttpServer server("body-back");
  register_http_uri_handler();
  // The host injects an Authorization header out-of-band (never from the .ari doc), keyed on URL.
  set_http_options_provider([](const std::string &url, bool for_write) {
    EXPECT_FALSE(for_write) << "a resolve() is a read";
    EXPECT_NE(url.find("/secret"), std::string::npos); // keyed on the request URL
    HttpRequestOptions o;
    o.headers.push_back("Authorization: Bearer read-token");
    return o;
  });
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/secret";
  UriResult r = resolve(url);
  const std::string req = server.take_request();
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "body-back");
  EXPECT_EQ(req.rfind("GET ", 0), 0u); // the read stays a GET
  EXPECT_NE(req.find("Authorization: Bearer read-token"), std::string::npos);
}

TEST(AriadneUri, HttpStorePutsBodyWithAuthHeader) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  HttpProviderGuard pg;
  CapturingHttpServer server("stored");
  register_http_uri_handler();
  set_http_options_provider([](const std::string &, bool for_write) {
    EXPECT_TRUE(for_write) << "a store() is a write";
    HttpRequestOptions o;
    o.headers.push_back("Authorization: Bearer write-token");
    return o;
  });
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/save";
  // A body with an embedded NUL proves the size-before-COPYPOSTFIELDS order (no strlen truncation).
  const std::string payload = std::string("PART-A\0PART-B", 13);
  StoreResult r = store(url, payload);
  const std::string req = server.take_request();
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_FALSE(r.canonical.empty());
  EXPECT_EQ(req.rfind("PUT ", 0), 0u); // store defaults to PUT
  EXPECT_NE(req.find("Authorization: Bearer write-token"), std::string::npos);
  // The provider gave no Content-Type, so the writer defaults a neutral one — NOT libcurl's POST
  // default of application/x-www-form-urlencoded (which would make a server form-parse the bytes).
  EXPECT_NE(req.find("Content-Type: application/octet-stream"), std::string::npos);
  EXPECT_EQ(req.find("application/x-www-form-urlencoded"), std::string::npos);
  const std::size_t body_at = req.find("\r\n\r\n");
  ASSERT_NE(body_at, std::string::npos);
  EXPECT_EQ(req.substr(body_at + 4), payload); // full body, embedded NUL and all
}

TEST(AriadneUri, HttpStoreProviderContentTypeWins) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  HttpProviderGuard pg;
  CapturingHttpServer server("stored");
  register_http_uri_handler();
  set_http_options_provider([](const std::string &, bool) {
    HttpRequestOptions o;
    o.headers.push_back(
        "Content-Type: application/json"); // an explicit type must not be overridden
    return o;
  });
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/save";
  StoreResult r = store(url, "{\"k\":1}");
  const std::string req = server.take_request();
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(req.find("Content-Type: application/json"), std::string::npos);
  EXPECT_EQ(req.find("application/octet-stream"), std::string::npos); // default not appended
}

TEST(AriadneUri, HttpStoreRefusesRedirect) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  // A 307 preserves method+body across the hop; a write must NOT be re-sent to another origin. The
  // Location target never exists — the point is that the handler does not follow it.
  CapturingHttpServer server("", "HTTP/1.1 307 Temporary Redirect\r\nLocation: http://127.0.0.1:1/"
                                 "moved\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
  register_http_uri_handler();
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/save";
  StoreResult r = store(url, "PAYLOAD");
  (void)server.take_request();
  EXPECT_FALSE(r.ok); // a 3xx on a write is an error, not a silent success
  EXPECT_NE(r.error.find("redirect"), std::string::npos);
}

TEST(AriadneUri, HttpResolveWithAuthDoesNotFollowRedirect) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  HttpProviderGuard pg;
  // A credentialed read must not follow a redirect (the auth header would cross an origin).
  CapturingHttpServer server("", "HTTP/1.1 302 Found\r\nLocation: http://127.0.0.1:1/"
                                 "elsewhere\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
  register_http_uri_handler();
  set_http_options_provider([](const std::string &, bool) {
    HttpRequestOptions o;
    o.headers.push_back(
        "X-Api-Key: secret"); // a custom auth header libcurl would NOT strip on redirect
    return o;
  });
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/data";
  UriResult r = resolve(url);
  (void)server.take_request();
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("redirect"), std::string::npos);
}

TEST(AriadneUri, HttpStoreHonorsPostMethodOverride) {
  if (!have_http_uri_handler())
    GTEST_SKIP() << "libcvc built without libcurl (http handler is a stub)";
  HttpHandlerGuard hg;
  HttpProviderGuard pg;
  CapturingHttpServer server("posted");
  register_http_uri_handler();
  set_http_options_provider([](const std::string &, bool) {
    HttpRequestOptions o;
    o.method = "POST"; // an RPC-style store endpoint
    return o;
  });
  const std::string url = "http://127.0.0.1:" + std::to_string(server.port()) + "/rpc";
  StoreResult r = store(url, "RPCPAYLOAD");
  const std::string req = server.take_request();
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(req.rfind("POST ", 0), 0u); // method override honoured
  const std::size_t body_at = req.find("\r\n\r\n");
  ASSERT_NE(body_at, std::string::npos);
  EXPECT_EQ(req.substr(body_at + 4), "RPCPAYLOAD");
}

TEST(AriadneUri, ResolveToFileExtractsExtFromFinalSegmentOnly) {
  register_uri_handler("memseg", [](const Uri &u, const std::string &) {
    return UriResult{true, "BODY", "memseg:" + u.path, std::string()};
  });
  SchemeGuard g{"memseg"};
  // Last '.' precedes the final '/': the ext must NOT capture "/…", which would put the temp file
  // in a nonexistent subdir and fail the open. Success (ok) is the discriminating check.
  ResolvedFile a = resolve_to_file("memseg://host/data.v2/model");
  ASSERT_TRUE(a.ok) << a.error;
  EXPECT_TRUE(std::filesystem::exists(a.path));
  // A real final-segment extension is preserved.
  ResolvedFile b = resolve_to_file("memseg://host/x/model.obj");
  ASSERT_TRUE(b.ok) << b.error;
  EXPECT_GE(b.path.size(), 4u);
  EXPECT_EQ(b.path.substr(b.path.size() - 4), ".obj");
}

TEST(AriadneUri, ResolveToFileHonorsRegisteredFileOverride) {
  register_uri_handler("file", [](const Uri &u, const std::string &) {
    return UriResult{true, "OVERRIDDEN-BYTES", "file:override:" + u.path, std::string()};
  });
  SchemeGuard g{"file"}; // MUST restore the built-in file handler for later tests
  ResolvedFile rf = resolve_to_file("file://whatever.off");
  ASSERT_TRUE(rf.ok) << rf.error;
  std::ifstream in(rf.path, std::ios::binary);
  std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  EXPECT_EQ(content, "OVERRIDDEN-BYTES"); // went through the override -> temp bridge, not in-place
}

TEST(AriadneUri, StoreFileWritesAndResolvesBack) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_store_test";
  fs::create_directories(dir);
  const std::string path = (dir / "out.json").string();
  EXPECT_TRUE(has_uri_store_handler("file"));
  const StoreResult s = store(path, "STORED-BYTES");
  ASSERT_TRUE(s.ok) << s.error;
  EXPECT_FALSE(s.canonical.empty());
  const UriResult r = resolve(path); // round-trips through the read side
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "STORED-BYTES");
  std::error_code ec;
  fs::remove(path, ec);
}

TEST(AriadneUri, StoreFileAtomicallyOverwrites) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_store_test";
  fs::create_directories(dir);
  const std::string path = (dir / "ow.json").string();
  ASSERT_TRUE(store(path, "first").ok);
  ASSERT_TRUE(store(path, "second-longer").ok); // rename-over replaces atomically
  const UriResult r = resolve(path);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "second-longer"); // fully replaced, no residue of "first"
  std::error_code ec;
  fs::remove(path, ec);
}

TEST(AriadneUri, StoreToDirectoryErrors) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_store_test";
  fs::create_directories(dir);
  const StoreResult s = store(dir.string(), "x");
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.error.find("directory"), std::string::npos);
}

TEST(AriadneUri, StoreUnknownSchemeErrors) {
  EXPECT_FALSE(has_uri_store_handler("nostore"));
  const StoreResult s = store("nostore://x", "y");
  EXPECT_FALSE(s.ok);
  EXPECT_NE(s.error.find("no store handler"), std::string::npos);
}

TEST(AriadneUri, StoreFileRejectsOversize) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_store_test";
  fs::create_directories(dir);
  const std::string path = (dir / "big.bin").string();
  const std::string big(17u * 1024u * 1024u, 'x'); // > the 16 MiB cap
  const StoreResult s = store(path, big);
  EXPECT_FALSE(s.ok); // rejected up front (symmetric with the read cap), so it can't become
  EXPECT_NE(s.error.find("exceeds"), std::string::npos); // a file the read side could not load
  EXPECT_FALSE(fs::exists(path));                        // nothing written
}

TEST(AriadneUri, ConfigurableStoreCap) {
  CapGuard cg;
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_store_test";
  fs::create_directories(dir);
  const std::string path = (dir / "cap.bin").string();
  set_store_file_byte_cap(8);
  EXPECT_EQ(store_file_byte_cap(), 8u);
  EXPECT_FALSE(store(path, "123456789").ok); // 9 > 8 -> rejected
  EXPECT_TRUE(store(path, "12345678").ok);   // 8 == cap -> ok
  set_store_file_byte_cap(0);                // 0 = unlimited
  EXPECT_TRUE(store(path, std::string(1000, 'x')).ok);
  std::error_code ec;
  fs::remove(path, ec);
}

TEST(AriadneUri, ConfigurableResolveCap) {
  CapGuard cg;
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / "ariadne_store_test";
  fs::create_directories(dir);
  const std::string path = (dir / "rcap.bin").string();
  set_store_file_byte_cap(0); // unlimited store so the write itself is not what fails
  ASSERT_TRUE(store(path, "0123456789").ok); // 10 bytes on disk
  set_resolve_file_byte_cap(4);
  UriResult r = resolve(path);
  EXPECT_FALSE(r.ok); // 10 > the 4-byte read cap
  EXPECT_NE(r.error.find("read cap"), std::string::npos);
  set_resolve_file_byte_cap(0); // unlimited
  EXPECT_TRUE(resolve(path).ok);
  std::error_code ec;
  fs::remove(path, ec);
}

TEST(AriadneUri, OkHandlerWithEmptyCanonicalFallsBackToRawUri) {
  // resolve() enforces the UriResult invariant: an ok result always has a non-empty canonical
  // (the dedup/cycle key), so a handler that forgets to set one cannot collapse the guard.
  register_uri_handler("emptycanon", [](const Uri &, const std::string &) {
    return UriResult{true, "body", std::string(), std::string()};
  });
  UriResult r = resolve("emptycanon://foo");
  ASSERT_TRUE(r.ok);
  EXPECT_EQ(r.canonical, "emptycanon://foo");
}

TEST(AriadneUri, ThrowingHandlerIsContainedNotPropagated) {
  register_uri_handler("boom", [](const Uri &, const std::string &) -> UriResult {
    throw std::runtime_error("kaboom");
  });
  UriResult r = resolve("boom://x"); // must not escape the UriResult boundary
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("kaboom"), std::string::npos);
}

TEST(AriadneStateUri, ValueChannelResolvesNodeValue) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("ui.libs.controls").value(std::string("hello-from-state"));
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  UriResult r = resolve("state://ui.libs.controls?value");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "hello-from-state");
  EXPECT_FALSE(r.canonical.empty());
  UriResult bare = resolve("state://ui.libs.controls"); // no query defaults to ?value
  ASSERT_TRUE(bare.ok) << bare.error;
  EXPECT_EQ(bare.content, "hello-from-state");
}

TEST(AriadneStateUri, TransparentLinkAliasCollapsesToTargetCanonical) {
  // An alias (transparent link) and its target hold identical content; their canonicals must
  // match too, so the loader's import dedup collapses the two spellings into one library.
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("libs.forms").value(std::string("units:\n  u: { text: X }\n"));
  root("libs.alias").linkTo("libs.forms", cvc::state::link_mode::transparent);
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  UriResult a = resolve("state://libs.alias");
  UriResult t = resolve("state://libs.forms");
  ASSERT_TRUE(a.ok) << a.error;
  ASSERT_TRUE(t.ok) << t.error;
  EXPECT_EQ(a.content, t.content);     // the link follows through to the target's value
  EXPECT_EQ(a.canonical, t.canonical); // and the canonical follows too -> they dedup as one
}

TEST(AriadneStateUri, MissingNodeErrors) {
  cvc::app app;
  register_state_uri_handler(cvc::state::instance(app));
  StateHandlerGuard guard;
  UriResult r = resolve("state://does.not.exist");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("not found"), std::string::npos);
}

TEST(AriadneStateUri, ChildrenChannelUnsupported) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("g.mesh").value(std::string("v"));
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  // ?value and ?data are served (see other tests); ?children still needs a richer handler.
  UriResult c = resolve("state://g.mesh?children");
  EXPECT_FALSE(c.ok);
  EXPECT_NE(c.error.find("?children"), std::string::npos);
}

TEST(AriadneStateUri, UnregisterRemovesHandler) {
  cvc::app app;
  register_state_uri_handler(cvc::state::instance(app));
  EXPECT_TRUE(has_uri_handler("state"));
  unregister_uri_handler("state");
  EXPECT_FALSE(has_uri_handler("state"));
  UriResult r = resolve("state://x");
  EXPECT_FALSE(r.ok);
  EXPECT_NE(r.error.find("no handler"), std::string::npos);
  unregister_uri_store_handler("state"); // also drop the write handler so its root can't dangle
}

TEST(AriadneStateUri, StoreWritesNodeValueAndRoundTrips) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  EXPECT_TRUE(has_uri_store_handler("state"));
  const StoreResult s = store("state://cfg.theme?value", "dark");
  ASSERT_TRUE(s.ok) << s.error;
  EXPECT_EQ(root("cfg.theme").value(), "dark"); // wrote (and created) the node
  const UriResult r = resolve("state://cfg.theme");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "dark"); // read back through the read handler
}

TEST(AriadneStateUri, StoreAndResolveDataChannel) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  const StoreResult s = store("state://blob.x?data", "BINARY-ISH");
  ASSERT_TRUE(s.ok) << s.error;
  const UriResult r = resolve("state://blob.x?data"); // ?data round-trips through the data channel
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "BINARY-ISH");
}

TEST(AriadneStateUri, StoreThroughNonWritableTransparentLinkRoundTrips) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("libs.forms").value(std::string("orig"));
  // A DEFAULT transparent link is NOT writable — value()'s write-routing would shadow a store on
  // the link node, but the read follows through, so store must write the effective target too.
  root("libs.alias").linkTo("libs.forms", cvc::state::link_mode::transparent);
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  const StoreResult s = store("state://libs.alias?value", "NEW");
  ASSERT_TRUE(s.ok) << s.error;
  const UriResult r = resolve("state://libs.alias?value");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_EQ(r.content, "NEW");                  // read-back through the link sees the stored bytes
  EXPECT_EQ(root("libs.forms").value(), "NEW"); // the store reached the effective target
  EXPECT_EQ(s.canonical, r.canonical);          // and store/resolve report the same canonical
}

TEST(AriadneStateUri, SyncResolverCapsFromState) {
  CapGuard cg;
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("sys.ariadne.resolver.read_cap_bytes").value(std::string("100"));
  root("sys.ariadne.resolver.store_cap_bytes").value(std::string("200"));
  sync_resolver_caps_from_state(root);
  EXPECT_EQ(resolve_file_byte_cap(), 100u);
  EXPECT_EQ(store_file_byte_cap(), 200u);
  // A non-numeric (or missing/empty) node leaves that cap unchanged — a typo can't zero the cap.
  root("sys.ariadne.resolver.read_cap_bytes").value(std::string("oops"));
  sync_resolver_caps_from_state(root);
  EXPECT_EQ(resolve_file_byte_cap(), 100u);
  // std::stoull is lenient: it would silently take "16MiB" as 16 and WRAP "-1" to ~16 EiB. Both are
  // rejected (whole-string + sign-free), so a fat-fingered human size can neither shrink the cap to
  // a few bytes (rejecting every fragment) nor uncap the OOM guard. Cap stays at its last good 100.
  for (const char *typo : {"16MiB", "10MB", "1,048,576", "-1", "  -5", "0x10", "16 "}) {
    root("sys.ariadne.resolver.read_cap_bytes").value(std::string(typo));
    sync_resolver_caps_from_state(root);
    EXPECT_EQ(resolve_file_byte_cap(), 100u) << "malformed cap should be ignored: " << typo;
  }
  // A clean, complete decimal still applies — including "0" (unlimited).
  root("sys.ariadne.resolver.read_cap_bytes").value(std::string("0"));
  sync_resolver_caps_from_state(root);
  EXPECT_EQ(resolve_file_byte_cap(), 0u);
}

TEST(AriadneStateUri, UnregisterRemovesBothReadAndWrite) {
  cvc::app app;
  register_state_uri_handler(cvc::state::instance(app));
  EXPECT_TRUE(has_uri_handler("state"));
  EXPECT_TRUE(has_uri_store_handler("state"));
  unregister_state_uri_handler();
  EXPECT_FALSE(has_uri_handler("state"));
  EXPECT_FALSE(has_uri_store_handler("state"));
}

TEST(AriadneStateIo, SaveRestoreRoundTripsViaFile) {
  namespace fs = std::filesystem;
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("doc.title").value(std::string("Hello"));
  root("doc.n").value(std::string("42"));
  const std::string path = (fs::temp_directory_path() / "ariadne_state_io.json").string();
  std::string err;
  ASSERT_TRUE(save_state(root("doc"), path, &err)) << err; // serializes keys "doc.title"/"doc.n"
  // Restore into a FRESH app's ROOT — the absolute paths reconstruct there (nested children survive
  // the round-trip, exercising the #430 ptree deserialize fix).
  cvc::app app2;
  cvc::state &root2 = cvc::state::instance(app2);
  ASSERT_TRUE(restore_state(root2, path, &err)) << err;
  EXPECT_EQ(root2("doc.title").value(), "Hello");
  EXPECT_EQ(root2("doc.n").value(), "42");
  std::error_code ec;
  fs::remove(path, ec);
}

TEST(AriadneStateIo, SaveRestoreRoundTripsViaStateScheme) {
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  root("cfg.color").value(std::string("red"));
  std::string err;
  // Snapshot the cfg subtree onto a state node (its JSON text lives on backup's value channel).
  ASSERT_TRUE(save_state(root("cfg"), "state://backup?value", &err)) << err;
  root("cfg.color").value(std::string("green")); // mutate after the snapshot
  // Restore into the app root reverts cfg.color — a full round-trip through state:// store+resolve.
  ASSERT_TRUE(restore_state(root, "state://backup?value", &err)) << err;
  EXPECT_EQ(root("cfg.color").value(), "red");
}

TEST(AriadneModularity, ImportUnitsFromAnotherFile) {
  SKIP_WITHOUT_YAML();
  // A reusable LIBRARY of units in one file, imported + used from another.
  write_temp_ari("lib.ari", R"(
units:
  labeled:
    slider_int: "{label}"
    bind: "{path}"
    lo: 0
    hi: 100
)");
  const std::string main = write_temp_ari("main.ari", R"(
import: lib.ari
windows:
  - window: W
    children:
      - include: labeled
        args: { label: Agents, path: demo.agents }
)");
  LoadResult r = load_file(main); // relative import resolves against main.ari's directory
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *s = find(r.root, Kind::SliderInt, "Agents");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->bind, "demo.agents"); // the library unit expanded with this program's args
}

TEST(AriadneModularity, ImportCycleTerminates) {
  SKIP_WITHOUT_YAML();
  // a imports b, b imports a — the resolved-path cycle guard must terminate, and both
  // libraries' units must still be available.
  const std::string a = write_temp_ari("a.ari", R"(
import: b.ari
units:
  ua: { text: fromA }
windows:
  - window: W
    children:
      - include: ub
)");
  write_temp_ari("b.ari", R"(
import: a.ari
units:
  ub: { text: fromB }
)");
  LoadResult r = load_file(a);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find(r.root, Kind::Text, "fromB"), nullptr); // b's unit reached through the cycle
}

TEST(AriadneModularity, ImportOfADirectoryWarnsNotFatal) {
  SKIP_WITHOUT_YAML();
  // A directory-valued import must degrade to a warning (the file handler rejects it cleanly),
  // never abort the whole document load with a fatal error.
  const std::string probe = write_temp_ari("dir_import_probe.txt", "x");
  const std::string dir = std::filesystem::path(probe).parent_path().string();
  const std::string main = write_temp_ari("main_dir_import.ari", "import: '" + dir +
                                                                     "'\n"
                                                                     "windows:\n"
                                                                     "  - window: W\n"
                                                                     "    children: []\n");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error; // NOT fatal
  EXPECT_TRUE(has_warning(r, "could not be resolved"));
}

TEST(AriadneModularity, ImportUnitsFromAStateNode) {
  SKIP_WITHOUT_YAML();
  // A reusable units LIBRARY living in the state tree (its .ari text on a node's value
  // channel), imported through the full §13 resolver via the state:// handler.
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("ui.libs.forms")
      .value(std::string("units:\n"
                         "  labeled:\n"
                         "    text: \"{label}\"\n"));
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  LoadResult r = load_string(R"(
import: state://ui.libs.forms
windows:
  - window: W
    children:
      - include: labeled
        args: { label: FromState }
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find(r.root, Kind::Text, "FromState"), nullptr);
}

TEST(AriadneModularity, DeepImportChainIsDepthCapped) {
  SKIP_WITHOUT_YAML();
  // A linear chain of DISTINCT libraries (f0 imports f1 imports … imports fN) recurses one C++
  // frame per link; the dedup set never trips (all distinct). The depth cap must stop it with a
  // warning, not a stack overflow. N exceeds kMaxImportDepth (32).
  const int N = 40;
  std::string f0;
  for (int i = 0; i <= N; ++i) {
    std::string content;
    if (i < N)
      content += "import: f" + std::to_string(i + 1) + ".ari\n";
    content += "units:\n  u" + std::to_string(i) + ": { text: T" + std::to_string(i) + " }\n";
    const std::string p = write_temp_ari("f" + std::to_string(i) + ".ari", content);
    if (i == 0)
      f0 = p;
  }
  LoadResult r = load_file(f0);
  ASSERT_TRUE(r.ok) << r.error; // capped, not crashed
  EXPECT_TRUE(has_warning(r, "maximum depth"));
}

TEST(AriadneModularity, ImportUnknownSchemeWarns) {
  SKIP_WITHOUT_YAML();
  // A scheme with no registered handler (no state/http installed) is surfaced, not fatal.
  LoadResult r = load_string(R"(
import: state://ui.libs.controls?value
windows:
  - window: W
    children: []
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "could not be resolved"));
}

TEST(AriadneModularity, FanOutBombIsRefusedNotExploded) {
  SKIP_WITHOUT_YAML();
  // A unit that includes itself N times would be N^depth expansions under a depth cap; the
  // name-based guard refuses re-entry, so this terminates immediately.
  LoadResult r = load_string(R"(
units:
  bomb:
    group:
    children:
      - include: bomb
      - include: bomb
      - include: bomb
windows:
  - window: W
    children:
      - include: bomb
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "recursive unit"));
}

TEST(AriadneMount, LoadMountsFragmentAsScopedModule) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("panel.ari", R"(
root:
  - slider_int: "{title}"
    bind: level
    lo: 0
    hi: 10
)");
  const std::string main = write_temp_ari("main_load.ari", R"(
windows:
  - window: W
    children:
      - load: panel.ari
        as: rf
        args: { title: Range }
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  // The fragment's widget mounted, with its arg substituted and its BARE bind kept (the Runtime
  // scopes it at the sub-prefix — proven in the runtime test).
  const Widget *s = find(r.root, Kind::SliderInt, "Range");
  ASSERT_NE(s, nullptr);
  EXPECT_EQ(s->bind, "level");
  // A scoped wrapper carries includes.<as>.
  const Widget *scoped = find_scope(r.root, "includes.rf");
  ASSERT_NE(scoped, nullptr);
}

TEST(AriadneMount, LoadDefaultsMountIdFromBasename) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("gauge.ari", "root: [ { text: hi } ]\n");
  const std::string main = write_temp_ari("main_default_as.ari", R"(
windows:
  - window: W
    children:
      - load: gauge.ari
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find_scope(r.root, "includes.gauge"), nullptr); // basename, extension stripped
}

TEST(AriadneMount, LoadUnitsAreIsolatedFromHost) {
  SKIP_WITHOUT_YAML();
  // The fragment defines + uses its OWN unit; the host must NOT see it (module boundary).
  write_temp_ari("lib_panel.ari", R"(
units:
  knob: { slider_int: Knob, bind: k, lo: 0, hi: 4 }
root:
  - include: knob
)");
  const std::string main = write_temp_ari("main_iso.ari", R"(
windows:
  - window: W
    children:
      - load: lib_panel.ari
        as: p
      - include: knob        # the host cannot see the fragment's unit
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find(r.root, Kind::SliderInt, "Knob"), nullptr); // the fragment expanded its own unit
  EXPECT_TRUE(has_warning(r, "unknown unit 'knob'"));        // the host's include did not
}

TEST(AriadneMount, LoadParsesLinkHoles) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("linked_panel.ari", "root: [ { text: hi } ]\n");
  const std::string main = write_temp_ari("main_link.ari", R"(
windows:
  - window: W
    children:
      - load: linked_panel.ari
        as: rf
        link:
          theme: /ui.theme
          fleet: { to: shared.fleet, mode: ro }
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *m = find_scope(r.root, "includes.rf");
  ASSERT_NE(m, nullptr);
  ASSERT_EQ(m->links.size(), 2u);
  const LinkHole *theme = nullptr;
  const LinkHole *fleet = nullptr;
  for (const LinkHole &h : m->links) {
    if (h.name == "theme")
      theme = &h;
    if (h.name == "fleet")
      fleet = &h;
  }
  ASSERT_NE(theme, nullptr);
  EXPECT_EQ(theme->target, "/ui.theme");
  EXPECT_TRUE(theme->writable); // scalar form defaults rw
  ASSERT_NE(fleet, nullptr);
  EXPECT_EQ(fleet->target, "shared.fleet");
  EXPECT_FALSE(fleet->writable); // mode: ro
}

TEST(AriadneMount, LinkModeFailsClosed) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("fc_panel.ari", "root: [ { text: x } ]\n");
  const std::string main = write_temp_ari("main_fc.ari", R"(
windows:
  - window: W
    children:
      - load: fc_panel.ari
        as: rf
        link:
          a: { to: p.a, mode: readonly }
          b: { to: p.b }
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *m = find_scope(r.root, "includes.rf");
  ASSERT_NE(m, nullptr);
  const LinkHole *a = nullptr, *b = nullptr;
  for (const LinkHole &h : m->links) {
    if (h.name == "a")
      a = &h;
    if (h.name == "b")
      b = &h;
  }
  ASSERT_NE(a, nullptr);
  EXPECT_FALSE(a->writable); // an unrecognized mode ('readonly') fails CLOSED to read-only
  ASSERT_NE(b, nullptr);
  EXPECT_TRUE(b->writable); // no mode -> rw default
  EXPECT_TRUE(has_warning(r, "unknown mode"));
}

TEST(AriadneMount, LinkNameLeadingSlashIsRejected) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("ls_panel.ari", "root: [ { text: x } ]\n");
  const std::string main = write_temp_ari("main_ls.ari", R"(
windows:
  - window: W
    children:
      - load: ls_panel.ari
        as: rf
        link:
          "/escape": /app.secret
          ok: p.val
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *m = find_scope(r.root, "includes.rf");
  ASSERT_NE(m, nullptr);
  ASSERT_EQ(m->links.size(), 1u); // the '/'-prefixed escape name was dropped
  EXPECT_EQ(m->links[0].name, "ok");
  EXPECT_TRUE(has_warning(r, "must not start with '/'"));
}

TEST(AriadneMount, LoadNonScalarInitWarns) {
  SKIP_WITHOUT_YAML();
  // A fragment's init: written as a list (a common mistake) is ignored WITH a warning, matching
  // the top-level document path — not silently dropped.
  write_temp_ari("bad_init_panel.ari",
                 "init: [ (state-set \"a\" \"1\") ]\nroot: [ { text: x } ]\n");
  const std::string main = write_temp_ari("main_bad_init.ari", R"(
windows:
  - window: W
    children:
      - load: bad_init_panel.ari
        as: rf
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "init: must be a scalar"));
}

TEST(AriadneMount, LoadNeedsPreflightWarnsOnUngranted) {
  SKIP_WITHOUT_YAML();
  // The fragment declares it needs two holes; the mount grants only one -> a warning names the
  // ungranted one, and the granted one is silent.
  write_temp_ari("needs_panel.ari", "needs: [theme, fleet]\nroot: [ { text: x } ]\n");
  const std::string main = write_temp_ari("main_needs.ari", R"(
windows:
  - window: W
    children:
      - load: needs_panel.ari
        as: rf
        link:
          theme: /ui.theme
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "needs link 'fleet'"));  // ungranted -> warned
  EXPECT_FALSE(has_warning(r, "needs link 'theme'")); // granted -> silent
}

TEST(AriadneMount, LoadTracksSourcesForHotReload) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("hr_lib.ari", "units:\n  u: { text: x }\n");
  write_temp_ari("hr_panel.ari", "root: [ { text: p } ]\n");
  const std::string main =
      write_temp_ari("hr_main.ari", "import: hr_lib.ari\n"
                                    "windows:\n  - window: W\n    children:\n"
                                    "      - load: hr_panel.ari\n        as: rf\n");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  const auto has_src = [&](const std::string &needle) {
    for (const std::string &s : r.sources)
      if (s.find(needle) != std::string::npos)
        return true;
    return false;
  };
  EXPECT_TRUE(has_src("hr_main.ari"));  // the main document
  EXPECT_TRUE(has_src("hr_lib.ari"));   // an imported library
  EXPECT_TRUE(has_src("hr_panel.ari")); // a mounted fragment
}

TEST(AriadneMount, SourcesChangedDetectsMtimeBump) {
  namespace fs = std::filesystem;
  const std::string p = write_temp_ari("hr_watch.ari", "root: []\n");
  std::vector<std::string> sources{p};
  std::map<std::string, std::int64_t> stamps;
  EXPECT_FALSE(sources_changed(sources, stamps)); // first sight = baseline, not a change
  EXPECT_FALSE(sources_changed(sources, stamps)); // still unchanged
  // Bump the mtime explicitly so the test is independent of filesystem timestamp granularity.
  const auto now = fs::last_write_time(p);
  fs::last_write_time(p, now + std::chrono::seconds(2));
  EXPECT_TRUE(sources_changed(sources, stamps));  // change detected
  EXPECT_FALSE(sources_changed(sources, stamps)); // stamp updated -> not reported again
  // A missing source is treated as unchanged (never throws).
  std::vector<std::string> missing{"/no/such/ariadne/file.ari"};
  std::map<std::string, std::int64_t> ms;
  EXPECT_FALSE(sources_changed(missing, ms));
}

TEST(AriadneMount, SourceStampsSeedTheHotReloadBaseline) {
  SKIP_WITHOUT_YAML();
  namespace fs = std::filesystem;
  const std::string main = write_temp_ari("ss_main.ari", "root: []\n");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  ASSERT_FALSE(r.source_stamps.empty()); // load-time mtimes captured
  // Seeding the poll with the LOAD-TIME stamps catches a change that happened after read but
  // before the first poll — which a fresh (first-sight = baseline) map would silently adopt.
  std::map<std::string, std::int64_t> stamps = r.source_stamps;
  const auto now = fs::last_write_time(main);
  fs::last_write_time(main, now + std::chrono::seconds(2));
  EXPECT_TRUE(sources_changed(r.sources, stamps));
}

TEST(AriadneMount, SourcesChangedPrunesRemovedSources) {
  const std::string a = write_temp_ari("prune_a.ari", "root: []\n");
  const std::string b = write_temp_ari("prune_b.ari", "root: []\n");
  std::map<std::string, std::int64_t> stamps;
  sources_changed({a, b}, stamps); // baseline both
  EXPECT_EQ(stamps.size(), 2u);
  sources_changed({a}, stamps); // b dropped from the graph
  EXPECT_EQ(stamps.size(), 1u); // its stamp is pruned (bounded growth; re-baseline if it returns)
  EXPECT_EQ(stamps.count(b), 0u);
}

TEST(AriadneMount, LoadMountCycleTerminates) {
  SKIP_WITHOUT_YAML();
  // a loads b loads a — the resolved-URI mount guard must terminate (warn), not recurse forever.
  const std::string a = write_temp_ari("cyc_a.ari", R"(
root:
  - load: cyc_b.ari
    as: b
)");
  write_temp_ari("cyc_b.ari", R"(
root:
  - load: cyc_a.ari
    as: a
)");
  LoadResult r = load_file(a);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "mount cycle"));
}

TEST(AriadneMount, LoadDuplicateMountIdIsDisambiguated) {
  SKIP_WITHOUT_YAML();
  write_temp_ari("dup_panel.ari", "root: [ { text: x } ]\n");
  const std::string main = write_temp_ari("main_dup.ari", R"(
windows:
  - window: W
    children:
      - load: dup_panel.ari
        as: p
      - load: dup_panel.ari
        as: p
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find_scope(r.root, "includes.p"), nullptr);   // first keeps the id
  EXPECT_NE(find_scope(r.root, "includes.p_2"), nullptr); // second disambiguated
  EXPECT_TRUE(has_warning(r, "duplicate mount id"));
}

TEST(AriadneMount, LoadStateSchemeDefaultMountIdKeepsDottedTail) {
  SKIP_WITHOUT_YAML();
  // A non-file scheme's path is a dotted IDENTITY, not a filename — the default mount id must
  // keep the whole tail, so two distinct state fragments do not collapse to one id.
  cvc::app app;
  cvc::state &root = cvc::state::instance(app);
  root("libs.forms").value(std::string("root: [ { text: F } ]\n"));
  root("libs.panels").value(std::string("root: [ { text: P } ]\n"));
  register_state_uri_handler(root);
  StateHandlerGuard guard;
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - load: state://libs.forms
      - load: state://libs.panels
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find_scope(r.root, "includes.libs_forms"), nullptr);
  EXPECT_NE(find_scope(r.root, "includes.libs_panels"), nullptr);
  EXPECT_FALSE(has_warning(r, "duplicate mount id")); // distinct tails -> no collision
}

TEST(AriadneMount, FailedMountDoesNotBurnItsMountId) {
  SKIP_WITHOUT_YAML();
  // A mount that fails to parse must not reserve its id, or a later valid sibling with the same
  // id would be spuriously disambiguated.
  write_temp_ari("bad_frag.ari", "root: *nope\n"); // undefined alias -> YAML parse error
  write_temp_ari("good_frag.ari", "root: [ { text: ok } ]\n");
  const std::string main = write_temp_ari("main_fail_id.ari", R"(
windows:
  - window: W
    children:
      - load: bad_frag.ari
        as: p
      - load: good_frag.ari
        as: p
)");
  LoadResult r = load_file(main);
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_NE(find_scope(r.root, "includes.p"), nullptr);   // the good mount keeps `p`
  EXPECT_EQ(find_scope(r.root, "includes.p_2"), nullptr); // not bumped
  EXPECT_FALSE(has_warning(r, "duplicate mount id"));
}

TEST(AriadneMount, BranchingMountGraphIsAggregateCapped) {
  SKIP_WITHOUT_YAML();
  // A branching graph (each level mounts the next TWICE) would fan out 2^depth under only a
  // per-chain depth cap; the aggregate mount cap must stop it with a warning, not exponentially.
  const int D = 8; // 2^8 = 256 leaf mounts -> total attempts exceed kMaxMounts (256)
  std::string l0;
  for (int k = D; k >= 0; --k) {
    std::string content;
    if (k < D) {
      const std::string next = "L" + std::to_string(k + 1) + ".ari";
      content = "root:\n  - load: " + next + "\n    as: a\n  - load: " + next + "\n    as: b\n";
    } else {
      content = "root: [ { text: leaf } ]\n";
    }
    const std::string p = write_temp_ari("L" + std::to_string(k) + ".ari", content);
    if (k == 0)
      l0 = p;
  }
  LoadResult r = load_file(l0);
  ASSERT_TRUE(r.ok) << r.error; // bounded, not exponential
  EXPECT_TRUE(has_warning(r, "mount limit reached"));
}

TEST(AriadneReactive, RepeatParsedOntoWidget) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - text: "Item {i}"
        repeat: (int (state-get "n"))
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *t = find(r.root, Kind::Text);
  ASSERT_NE(t, nullptr);
  EXPECT_EQ(t->repeat, "(int (state-get \"n\"))");
  EXPECT_EQ(t->label, "Item {i}"); // template label kept verbatim; {i} substituted at emit
}

TEST(AriadneReactive, NestedRepeatWarnsUnsupported) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - group:
        repeat: (int 2)
        children:
          - checkbox: on
            bind: rows.{i}.on
            repeat: (int 3)
)");
  ASSERT_TRUE(r.ok) << r.error;
  EXPECT_TRUE(has_warning(r, "nested 'repeat'")); // single {i} index -> surfaced, not silent
}

TEST(AriadneReactive, TooltipParsedOntoWidget) {
  SKIP_WITHOUT_YAML();
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - button: Go
        on: go
        tooltip: "run the thing"
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *b = find(r.root, Kind::Button, "Go");
  ASSERT_NE(b, nullptr);
  EXPECT_EQ(b->tooltip, "run the thing");
}

TEST(AriadneReactive, VisibleWhenOnCustomWidgetIsConsumedNotAProp) {
  SKIP_WITHOUT_YAML();
  // visible_when is a common field for EVERY kind, custom included — it must be captured
  // on the widget and NOT leak into a custom widget's props bag.
  LoadResult r = load_string(R"(
windows:
  - window: W
    children:
      - type: gauge
        gauge: 0.8
        visible_when: (state-exists "ready")
)");
  ASSERT_TRUE(r.ok) << r.error;
  const Widget *c = find(r.root, Kind::Custom);
  ASSERT_NE(c, nullptr);
  EXPECT_EQ(c->custom_type, "gauge");
  EXPECT_EQ(c->visible_when, "(state-exists \"ready\")");
  EXPECT_EQ(c->props.find("visible_when"), nullptr);  // consumed, not a prop
  EXPECT_DOUBLE_EQ(c->props.num("gauge", -1.0), 0.8); // real config still captured
}
