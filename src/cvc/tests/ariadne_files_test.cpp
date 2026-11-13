// Tests for picking files and sending them (docs/ARIADNE.md §10.8): (file-pick) with a host's own
// picker and with the Runtime's composed "Open file" window, the handles it gives (never bytes,
// never a path), and (http-upload) as multipart, as a body and in chunks that resume, blocking and
// started (a browser page without threads, L4), with the host's policy and credentials.

#include <algorithm>
#include <boost/any.hpp>
#include <chrono>
#include <cvc/ariadne/ariadne.h>
#include <cvc/ariadne/backend.h>
#include <cvc/ariadne/files.h>
#include <cvc/ariadne/ftxui_backend.h>
#include <cvc/ariadne/net_host.h>
#include <cvc/ariadne/tvision_backend.h>
#include <cvc/ariadne/widget.h>
#include <cvc/core/app.h>
#include <cvc/net/http_client.h>
#include <cvc/state/state.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/types.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

using namespace cvc::ariadne;
namespace fs = std::filesystem;

namespace {

// A backend that draws nothing and records what the core drew, with programmable answers: the
// button clicked, the message field's edit, the radio list's pick, and optionally a picker of its
// own (pick_files).
struct PickMock : Backend {
  std::vector<std::string> windows; // titles drawn this frame
  std::vector<std::string> texts;   // text lines and values drawn this frame
  std::vector<std::string> listed;  // the last radio's options
  std::string click;                // a button label reported clicked
  std::string close_title;          // a window whose close button is reported pressed
  MessageEdit message_ret{};        // the next message_input's edit
  IndexEdit radio_ret{};            // the next radio's edit
  int focus_window = 0, focus_widget = 0;

  bool own_picker = false; // answer pick_files itself
  FilePickRequest asked;
  FilePickDone pending;

  Capabilities capabilities() const override {
    Capabilities c;
    c.windows = true;
    c.keyboard = true;
    return c;
  }
  void begin_frame() override {
    windows.clear();
    texts.clear();
  }
  bool begin_main_menu_bar() override { return false; }
  void end_main_menu_bar() override {}
  bool begin_menu(const char *) override { return false; }
  void end_menu() override {}
  bool begin_window(const char *t, const char *, const Size &, float, bool *p_open) override {
    windows.emplace_back(t);
    if (p_open && close_title == t)
      *p_open = false;
    return true;
  }
  void end_window() override {}
  void focus_next_window() override { ++focus_window; }
  void focus_next_widget() override { ++focus_widget; }
  void push_id(const char *) override {}
  void pop_id() override {}
  bool begin_grid(const Layout &, const char *) override { return true; }
  void grid_next_cell() override {}
  void end_grid() override {}
  void text_line(const char *t) override { texts.emplace_back(t); }
  void text_value(const char *l, const std::string &v) override {
    texts.push_back(std::string(l) + ": " + v);
  }
  void separator() override {}
  bool button(const char *l) override { return click == l; }
  bool menu_item_action(const char *) override { return false; }
  BoolEdit menu_item_toggle(const char *, bool) override { return {}; }
  BoolEdit checkbox(const char *, bool) override { return {}; }
  IntEdit slider_int(const char *, const char *, int, int, int) override { return {}; }
  DoubleEdit slider_double(const char *, const char *, double, double, double,
                           const char *) override {
    return {};
  }
  IndexEdit combo(const char *, int, const std::vector<std::string> &) override { return {}; }
  IndexEdit radio(const char *, int, const std::vector<std::string> &options, bool) override {
    listed = options;
    return std::exchange(radio_ret, IndexEdit{});
  }
  StringEdit input_text(const char *, const char *, const std::string &, bool) override {
    return {};
  }
  MessageEdit message_input(const char *, const char *, const std::string &,
                            const MessageInputOptions &) override {
    return std::exchange(message_ret, MessageEdit{});
  }
  bool pick_files(const FilePickRequest &req, FilePickDone done) override {
    if (!own_picker)
      return false;
    asked = req;
    pending = std::move(done);
    return true;
  }
  bool shows(const std::string &title) const {
    return std::find(windows.begin(), windows.end(), title) != windows.end();
  }
  bool said(const std::string &text) const {
    for (const std::string &t : texts)
      if (t.find(text) != std::string::npos)
        return true;
    return false;
  }
};

std::string sval(cvc::app &app, const char *path) {
  return cvc::state::instance(app)(path).value();
}

// Every value and data payload in the app's state tree, as text.
std::string whole_state(cvc::app &app) {
  namespace se = cvc::state_exec;
  cvc::state &root = cvc::state::instance(app);
  std::string all;
  for (const std::string &name : root.children()) {
    cvc::state &n = root(name);
    all += name + "=" + n.value() + "\n";
    const boost::any d = n.data();
    if (const std::string *s = boost::any_cast<std::string>(&d))
      all += *s + "\n";
    else if (const se::value_t *v = boost::any_cast<se::value_t>(&d))
      all += (std::get_if<se::bytes_value>(&v->v) ? std::get<se::bytes_value>(v->v).data
                                                  : se::to_string(*v)) +
             "\n";
  }
  return all;
}

void frames(Runtime &rt, int n) {
  for (int i = 0; i < n; ++i) {
    rt.render();
    rt.drain();
  }
}

template <class Pred> bool pump_until(Runtime &rt, Pred pred, int budget_ms = 10000) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(budget_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    rt.render();
    rt.drain();
    if (pred())
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  return pred();
}

// Run `program` as a button's action: one frame with the button clicked.
void run(Runtime &rt, PickMock &mb, const std::string &program) {
  rt.set_root(group({button("Go", program)}));
  mb.click = "Go";
  rt.render();
  rt.drain();
  mb.click.clear();
}

// A folder of files, removed at the end.
struct TempDir {
  fs::path path;
  explicit TempDir(const std::string &tag) {
    path = fs::temp_directory_path() /
           ("cvc_files_test_" + tag + "_" +
            std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(path);
  }
  ~TempDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
  }
  void write(const std::string &name, const std::string &bytes) const {
    fs::create_directories((path / name).parent_path());
    std::ofstream(path / name, std::ios::binary) << bytes;
  }
  std::string str() const { return path_utf8(path); }
};

// A DSL string literal holding `s` (backslashes and quotes escaped).
std::string lit(const std::string &s) {
  std::string out = "\"";
  for (const char c : s) {
    if (c == '\\' || c == '"')
      out.push_back('\\');
    out.push_back(c);
  }
  return out + "\"";
}

// (file-pick OPTS), the reply's count and first file in state, then done = 1.
std::string pick_program(const std::string &opts) {
  return "(begin (set r (file-pick " + opts +
         "))"
         "  (state-set \"n\" (length r))"
         "  (if (> (length r) 0) (begin"
         "    (set f (nth r 0))"
         "    (state-set \"f.handle\" (get-attr f \"handle\"))"
         "    (state-set \"f.name\" (get-attr f \"name\"))"
         "    (state-set \"f.size\" (get-attr f \"size\"))"
         "    (state-set \"f.type\" (get-attr f \"type\"))))"
         "  (state-set \"done\" 1))";
}

} // namespace

TEST(AriadneFilePick, TheHostsPickerAnswersWithAHandleNotBytes) {
  cvc::app app;
  Runtime rt(app, "");
  PickMock mb;
  mb.own_picker = true;
  rt.set_backend(&mb);
  run(rt, mb,
      pick_program("(dict \"accept\" (list \".txt\" \"image/*\") \"multiple\" #t \"title\" "
                   "\"Attach\")"));
  frames(rt, 3);
  ASSERT_TRUE(mb.pending) << "the host's own picker was asked";
  EXPECT_EQ(mb.asked.accept, (std::vector<std::string>{".txt", "image/*"}));
  EXPECT_TRUE(mb.asked.multiple);
  EXPECT_EQ(mb.asked.title, "Attach");
  EXPECT_EQ(sval(app, "done"), "") << "the program waits for the user";
  EXPECT_FALSE(mb.shows("Attach")) << "no composed window where the host has a picker";

  const std::string secret = "the file's own bytes, never in state";
  mb.pending({memory_file_source("notes.txt", secret)}, "");
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "n"), "1");
  EXPECT_EQ(sval(app, "f.handle"), "file:1");
  EXPECT_EQ(sval(app, "f.name"), "notes.txt");
  EXPECT_EQ(sval(app, "f.size"), std::to_string(secret.size()));
  EXPECT_EQ(sval(app, "f.type"), "text/plain");
  EXPECT_EQ(whole_state(app).find(secret), std::string::npos);
  ASSERT_TRUE(picked_files(app).find("file:1"));

  // (file-info H) and (file-forget H).
  run(rt, mb,
      "(begin (state-set \"i.name\" (get-attr (file-info \"file:1\") \"name\"))"
      "  (state-set \"i.gone\" (file-forget \"file:1\"))"
      "  (state-set \"i.again\" (file-forget \"file:1\"))"
      "  (state-set \"i.nil\" (if (file-info \"file:1\") \"some\" \"none\")))");
  ASSERT_TRUE(pump_until(rt, [&] { return !sval(app, "i.nil").empty(); }));
  EXPECT_EQ(sval(app, "i.name"), "notes.txt");
  EXPECT_EQ(sval(app, "i.gone"), "true");
  EXPECT_EQ(sval(app, "i.again"), "false");
  EXPECT_EQ(sval(app, "i.nil"), "none");
  EXPECT_FALSE(picked_files(app).find("file:1"));
}

TEST(AriadneFilePick, ACancelIsAnEmptyListAndOnePickerAtATime) {
  cvc::app app;
  Runtime rt(app, "");
  PickMock mb;
  mb.own_picker = true;
  rt.set_backend(&mb);
  run(rt, mb, pick_program(""));
  frames(rt, 2);
  ASSERT_TRUE(mb.pending);
  FilePickDone first = std::move(mb.pending);
  mb.pending = nullptr;
  // A second pick while the first is open raises at its call.
  run(rt, mb, "(state-set \"second\" (try (file-pick) (catch (e) (get-attr e \"message\"))))");
  ASSERT_TRUE(pump_until(rt, [&] { return !sval(app, "second").empty(); }));
  EXPECT_NE(sval(app, "second").find("a file picker is open already"), std::string::npos)
      << sval(app, "second");
  first({}, "");
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "n"), "0");
  EXPECT_EQ(sval(app, "f.handle"), "");
  // Bad options are an error at the call.
  run(rt, mb,
      "(state-set \"bad\" (try (file-pick (dict \"accept\" \"png\")) (catch (e) (get-attr e "
      "\"message\"))))");
  ASSERT_TRUE(pump_until(rt, [&] { return !sval(app, "bad").empty(); }));
  EXPECT_NE(sval(app, "bad").find("\"type/*\""), std::string::npos) << sval(app, "bad");
}

TEST(AriadneFilePick, TheComposedWindowListsAFolderAndPicksAFile) {
  TempDir dir("composed");
  dir.write("notes.txt", "hello");
  dir.write("photo.png", "not really a png");
  dir.write("sub/inner.txt", "inside");
  dir.write(".hidden.txt", "x");
  cvc::app app;
  Runtime rt(app, "");
  PickMock mb; // no picker of its own: the Runtime draws one
  rt.set_backend(&mb);
  run(rt, mb, pick_program("(dict \"accept\" \".txt\" \"dir\" " + lit(dir.str()) + ")"));
  frames(rt, 2);
  ASSERT_TRUE(mb.shows("Open file"));
  EXPECT_GE(mb.focus_window, 1) << "the window is raised to take the keys";
  EXPECT_GE(mb.focus_widget, 1) << "the name field has the focus";
  EXPECT_TRUE(mb.said("Folder: " + dir.str()));
  // "..", the folder, then the files `accept` takes; no hidden files, no .png.
  ASSERT_EQ(mb.listed.size(), 3u);
  EXPECT_EQ(mb.listed[0], "../");
  EXPECT_EQ(mb.listed[1], "sub/");
  EXPECT_EQ(mb.listed[2], "notes.txt  (5 B)");

  // Into sub/ by the list and Open, then the file by name and Enter.
  mb.radio_ret = {true, true, 1};
  frames(rt, 1);
  mb.click = "Open";
  frames(rt, 1);
  mb.click.clear();
  frames(rt, 1);
  EXPECT_TRUE(mb.said("Folder: " + path_utf8(dir.path / "sub")));
  ASSERT_EQ(mb.listed.size(), 2u);
  EXPECT_EQ(mb.listed[1], "inner.txt  (6 B)");
  EXPECT_EQ(sval(app, "done"), "");

  // A name that isn't there says so and keeps the window.
  mb.message_ret.submitted = true;
  mb.message_ret.value = "missing.txt";
  frames(rt, 2);
  EXPECT_TRUE(mb.shows("Open file"));
  EXPECT_TRUE(mb.said("No such file or folder: missing.txt"));

  mb.message_ret.submitted = true;
  mb.message_ret.value = "inner.txt";
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "n"), "1");
  EXPECT_EQ(sval(app, "f.name"), "inner.txt");
  EXPECT_EQ(sval(app, "f.size"), "6");
  frames(rt, 1);
  EXPECT_FALSE(mb.shows("Open file")) << "the window closes with the pick";
  EXPECT_EQ(whole_state(app).find(dir.str()), std::string::npos) << "no path reaches the program";

  // The handle reads the file.
  std::string bytes, error;
  picked_files(app).find(sval(app, "f.handle"))->read(0, 100, [&](std::string b, std::string e) {
    bytes = std::move(b);
    error = std::move(e);
  });
  EXPECT_EQ(bytes, "inside");
  EXPECT_EQ(error, "");
}

TEST(AriadneFilePick, TheComposedWindowRefusesWhatAcceptDoesNotTakeAndCancels) {
  TempDir dir("refuse");
  dir.write("photo.png", "png");
  cvc::app app;
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb, pick_program("(dict \"accept\" (list \".txt\") \"dir\" " + lit(dir.str()) + ")"));
  frames(rt, 2);
  mb.message_ret.submitted = true;
  mb.message_ret.value = "photo.png"; // there, but not a .txt
  frames(rt, 2);
  EXPECT_TRUE(mb.said("photo.png is not one of: .txt"));
  EXPECT_EQ(sval(app, "done"), "");
  mb.click = "Cancel";
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  mb.click.clear();
  EXPECT_EQ(sval(app, "n"), "0");

  // The window's close button cancels too.
  run(rt, mb, pick_program("(dict \"title\" \"Pick one\" \"dir\" " + lit(dir.str()) + ")"));
  frames(rt, 2);
  ASSERT_TRUE(mb.shows("Pick one"));
  cvc::state::instance(app)("done").value("");
  mb.close_title = "Pick one";
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "n"), "0");
}

TEST(AriadneFilePick, AFileThatChangedAfterItWasPickedIsNotRead) {
  TempDir dir("changed");
  dir.write("a.bin", "12345");
  std::string why;
  std::shared_ptr<FileSource> f = local_file_source(path_utf8(dir.path / "a.bin"), &why);
  ASSERT_TRUE(f) << why;
  EXPECT_EQ(f->size(), 5u);
  EXPECT_EQ(f->type(), "application/octet-stream");
  dir.write("a.bin", "1234567");
  std::string error;
  f->read(0, 10, [&](std::string, std::string e) { error = std::move(e); });
  EXPECT_NE(error.find("changed since it was picked"), std::string::npos) << error;
  EXPECT_FALSE(local_file_source(path_utf8(dir.path / "nope"), &why));
  EXPECT_FALSE(local_file_source(dir.str(), &why)) << "a folder is not a file";
}

TEST(AriadneFilePick, AcceptMatchesExtensionsAndTypes) {
  EXPECT_TRUE(file_accepted({}, "any.thing", "application/octet-stream"));
  EXPECT_TRUE(file_accepted({".PNG"}, "a.png", "image/png"));
  EXPECT_TRUE(file_accepted({"image/*"}, "a.webp", "image/webp"));
  EXPECT_TRUE(file_accepted({"video/mp4"}, "clip.mp4", "video/mp4"));
  EXPECT_FALSE(file_accepted({"image/*", ".txt"}, "clip.mp4", "video/mp4"));
  const std::vector<std::string> ext = accepted_extensions({".TXT", "video/mp4"});
  EXPECT_EQ(ext, (std::vector<std::string>{"txt", "mp4", "m4v"}));
  EXPECT_EQ(guess_mime_type("Cat.JPG"), "image/jpeg");
  EXPECT_EQ(guess_mime_type("noext"), "application/octet-stream");
}

// The composed window in a terminal: the FTXUI backend draws it over the document and takes the
// keys there, and tvision hosts it as one of its windows (its layout needs no terminal).
TEST(AriadneFilePick, TheOpenFileWindowTakesTheKeysInAnFtxuiTerminal) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  TempDir dir("ftxui");
  dir.write("notes.txt", "hello");
  dir.write("sub/inner.txt", "inside");
  cvc::app app;
  FtxuiBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  rt.set_root(group({button("Attach", pick_program("(dict \"dir\" " + lit(dir.str()) + ")"))}));
  const auto draw = [&] { return be.render_to_string(rt, 100, 30); };
  const auto key = [&](const char *k) { be.feed(rt, TermEvent::key_event(k, false)); };
  const auto type = [&](const std::string &text) {
    for (const char c : text)
      be.feed(rt, TermEvent::text_event(std::string(1, c)));
  };
  draw();
  key("Tab");
  key("Return");
  std::string screen;
  ASSERT_TRUE(pump_until(rt, [&] {
    screen = draw();
    return screen.find("Open file") != std::string::npos;
  })) << screen;
  // The folder, as far as the window is wide: a long temp path (macOS's /private/var/folders/...)
  // is cut at its frame.
  const std::size_t at = screen.find("Folder: ");
  ASSERT_NE(at, std::string::npos) << screen;
  const std::size_t from = at + 8;
  const std::size_t to = std::min(screen.find("\xe2\x94\x83", from), screen.find('\n', from));
  std::string shown = screen.substr(from, to == std::string::npos ? std::string::npos : to - from);
  shown.erase(shown.find_last_not_of(' ') + 1);
  EXPECT_FALSE(shown.empty()) << screen;
  EXPECT_EQ(dir.str().compare(0, shown.size(), shown), 0) << shown << " vs " << dir.str();
  EXPECT_NE(screen.find("sub/"), std::string::npos) << screen;
  EXPECT_NE(screen.find("notes.txt  (5 B)"), std::string::npos) << screen;
  // The name field has the keys: a folder by name enters it, then a file picks it.
  type("sub");
  key("Return");
  ASSERT_TRUE(pump_until(rt, [&] {
    screen = draw();
    return screen.find("inner.txt  (6 B)") != std::string::npos;
  })) << screen;
  type("inner.txt");
  key("Return");
  ASSERT_TRUE(pump_until(rt, [&] {
    draw();
    return sval(app, "done") == "1";
  })) << draw();
  EXPECT_EQ(sval(app, "f.name"), "inner.txt");
  EXPECT_EQ(sval(app, "f.size"), "6");
  EXPECT_EQ(draw().find("Open file"), std::string::npos) << "the window is gone";

  // Again: a second opening is a new window, drawn over the document as the first was (the same
  // window again would come back behind whatever was raised since).
  cvc::state::instance(app)("done").value(std::string());
  key("Tab");
  key("Return");
  ASSERT_TRUE(pump_until(rt, [&] {
    screen = draw();
    return screen.find("Open file") != std::string::npos;
  })) << screen;
  type("notes.txt");
  key("Return");
  ASSERT_TRUE(pump_until(rt, [&] {
    draw();
    return sval(app, "done") == "1";
  })) << draw();
  EXPECT_EQ(sval(app, "f.name"), "notes.txt");
}

TEST(AriadneFilePick, TvisionHostsTheOpenFileWindow) {
  if (!FtxuiBackend::available())
    GTEST_SKIP() << "libcvc built without FTXUI";
  TempDir dir("tvision");
  dir.write("notes.txt", "hello");
  cvc::app app;
  TvisionBackend be;
  Runtime rt(app, "");
  rt.set_backend(&be);
  rt.set_root(group({button("Attach", pick_program("(dict \"title\" \"Attach a file\" \"dir\" " +
                                                   lit(dir.str()) + ")"))}));
  be.render_to_string(rt, 100, 30);
  be.feed_window(rt, "", TermEvent::key_event("Tab", false));
  be.feed_window(rt, "", TermEvent::key_event("Return", false));
  std::string id;
  ASSERT_TRUE(pump_until(rt, [&] {
    be.render_to_string(rt, 100, 30);
    for (const FtxuiBackend::HostedWindow &w : be.hosted_windows())
      if (w.title == "Attach a file" && w.closable)
        id = w.id;
    return !id.empty();
  }));
  EXPECT_EQ(id.rfind("ariadne.file_pick.", 0), 0u) << id;
  const TermRows rows = be.render_window(id, 70, 16);
  std::string body;
  for (const auto &row : rows) {
    for (const auto &cell : row)
      body += cell.ch;
    body += '\n';
  }
  EXPECT_NE(body.find("notes.txt"), std::string::npos) << body;
  EXPECT_NE(body.find("Cancel"), std::string::npos) << body;
  // Its close box cancels, as the window's X does elsewhere.
  be.close_window(rt, id);
  ASSERT_TRUE(pump_until(rt, [&] {
    be.render_to_string(rt, 100, 30);
    return sval(app, "done") == "1";
  }));
  EXPECT_EQ(sval(app, "n"), "0");
}

// ---------------------------------------------------------------------------------------------
// (http-upload URL FILE [OPTS])
// ---------------------------------------------------------------------------------------------
namespace {

cvc::net::HttpResponse reply(long status, std::string body, std::vector<std::string> headers = {},
                             std::string url = "") {
  cvc::net::HttpResponse r;
  r.ok = true;
  r.status = status;
  r.body = std::move(body);
  r.headers = std::move(headers);
  r.canonical_url = std::move(url);
  return r;
}

cvc::net::HttpResponse dropped() {
  cvc::net::HttpResponse r;
  r.ok = false;
  r.error = "connection reset";
  return r;
}

using Handler = std::function<cvc::net::HttpResponse(const cvc::net::HttpRequest &)>;

// A transport that answers in place through a handler, from the pool thread the kernel runs on.
class FnHttpClient : public cvc::net::HttpClient {
public:
  explicit FnHttpClient(Handler h) : handler_(std::move(h)) {}
  cvc::net::HttpResponse send(const cvc::net::HttpRequest &req) override {
    std::lock_guard<std::mutex> lock(m_);
    seen.push_back(req);
    return handler_(req);
  }
  std::vector<cvc::net::HttpRequest> requests() {
    std::lock_guard<std::mutex> lock(m_);
    return seen;
  }

private:
  std::mutex m_;
  std::vector<cvc::net::HttpRequest> seen;
  Handler handler_;
};

// A transport that starts requests and answers them when the test says (the browser's fetch).
class DeferredHttpClient : public cvc::net::HttpClient {
public:
  struct Transfer : cvc::net::HttpTransfer {
    bool aborted = false;
    void abort() override { aborted = true; }
  };
  struct Pending {
    cvc::net::HttpRequest req;
    cvc::net::HttpDone done;
    std::shared_ptr<Transfer> transfer;
  };
  explicit DeferredHttpClient(Handler h) : handler_(std::move(h)) {}
  std::vector<Pending> pending;
  int sends = 0;
  bool starts_async() const override { return true; }
  std::shared_ptr<cvc::net::HttpTransfer> start(const cvc::net::HttpRequest &req,
                                                cvc::net::HttpDone done) override {
    auto t = std::make_shared<Transfer>();
    pending.push_back({req, std::move(done), t});
    return t;
  }
  cvc::net::HttpResponse send(const cvc::net::HttpRequest &req) override {
    ++sends;
    return handler_(req);
  }
  bool answer() {
    while (!pending.empty()) {
      Pending p = std::move(pending.front());
      pending.erase(pending.begin());
      if (p.transfer->aborted)
        continue;
      p.done(handler_(p.req));
      return true;
    }
    return false;
  }
  std::size_t live() const {
    return static_cast<std::size_t>(std::count_if(
        pending.begin(), pending.end(), [](const Pending &p) { return !p.transfer->aborted; }));
  }

private:
  Handler handler_;
};

struct NetGuard {
  ~NetGuard() {
    disable_net();
    cvc::net::set_http_client(nullptr);
  }
};

NetHostConfig allow(std::initializer_list<std::string> origins) {
  NetHostConfig c;
  c.enabled = true;
  c.allow = origins;
  return c;
}

std::string header(const cvc::net::HttpRequest &req, const std::string &name) {
  return cvc::net::response_header(req.headers, name);
}

// A txwtf-like resumable endpoint (PATCH .../up with Upload-Offset): it keeps what it got, answers
// {"offset": N}, and a 409 with its own Upload-Offset when the client's is not where it is.
struct ChunkServer {
  std::mutex m;
  std::string stored;
  int chunks = 0;
  std::function<bool(int)> fail_chunk; // chunk n: answer 503 instead
  std::function<bool(int)> drop_ack;   // chunk n: keep it, but the reply is lost
  cvc::net::HttpResponse operator()(const cvc::net::HttpRequest &req) {
    std::lock_guard<std::mutex> lock(m);
    if (req.method == "GET" && req.url == "http://api.test/up")
      return reply(200,
                   "{\"upload_id\": \"u1\", \"offset\": " + std::to_string(stored.size()) + "}");
    if (req.method != "PATCH" || req.url != "http://api.test/up")
      return reply(404, "{}");
    const int n = ++chunks;
    if (fail_chunk && fail_chunk(n))
      return reply(503, "busy");
    const std::string at = header(req, "Upload-Offset");
    if (at != std::to_string(stored.size()))
      return reply(409, "{\"detail\": \"Offset mismatch\"}",
                   {"Upload-Offset: " + std::to_string(stored.size())});
    stored += req.body;
    if (drop_ack && drop_ack(n))
      return dropped();
    return reply(200, "{\"upload_id\": \"u1\", \"offset\": " + std::to_string(stored.size()) + "}",
                 {"Content-Type: application/json"});
  }
};

std::string some_bytes(std::size_t n) {
  std::string s(n, '\0');
  for (std::size_t i = 0; i < n; ++i)
    s[i] = static_cast<char>((i * 7 + 3) % 251);
  return s;
}

// An upload's reply in state: up.ok, up.status, up.done, up.offset, up.size, up.error; done = 1.
std::string upload_program(const std::string &call) {
  return "(begin (set r " + call +
         ")"
         "  (state-set \"up.ok\" (get-attr r \"ok\"))"
         "  (state-set \"up.status\" (get-attr r \"status\"))"
         "  (state-set \"up.done\" (get-attr r \"done\"))"
         "  (state-set \"up.offset\" (get-attr r \"offset\"))"
         "  (state-set \"up.size\" (get-attr r \"size\"))"
         "  (state-set \"up.error\" (get-attr r \"error\"))"
         "  (state-data-set \"up.body\" (get-attr r \"body\"))"
         "  (state-set \"done\" 1))";
}

} // namespace

TEST(AriadneHttpUpload, MultipartCarriesTheFileAndItsFields) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetGuard guard;
  auto *net = new FnHttpClient([](const cvc::net::HttpRequest &req) {
    return reply(200, "{\"id\": 42, \"url\": \"/f/42\"}", {"Content-Type: application/json"},
                 req.url);
  });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string bytes = some_bytes(3000);
  const std::string h = picked_files(app).add(memory_file_source("cat \"1\".png", bytes));
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb,
      upload_program("(http-upload \"http://api.test/social/files\" " + lit(h) +
                     " (dict \"form\" (dict \"alt_text\" \"a cat\") \"progress\" \"up.p\"))"));
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "up.status"), "200") << sval(app, "up.error");
  EXPECT_EQ(sval(app, "up.done"), "true");
  EXPECT_EQ(sval(app, "up.offset"), "3000");
  EXPECT_EQ(sval(app, "up.size"), "3000");
  EXPECT_EQ(sval(app, "up.p"), "1");
  const std::vector<cvc::net::HttpRequest> seen = net->requests();
  ASSERT_EQ(seen.size(), 1u);
  const cvc::net::HttpRequest &req = seen[0];
  EXPECT_EQ(req.method, "POST");
  const std::string ct = header(req, "Content-Type");
  ASSERT_EQ(ct.rfind("multipart/form-data; boundary=", 0), 0u) << ct;
  const std::string boundary = ct.substr(std::string("multipart/form-data; boundary=").size());
  const std::string want =
      "--" + boundary + "\r\nContent-Disposition: form-data; name=\"alt_text\"\r\n\r\na cat\r\n" +
      "--" + boundary +
      "\r\nContent-Disposition: form-data; name=\"file\"; filename=\"cat %221%22.png\"\r\n"
      "Content-Type: image/png\r\n\r\n" +
      bytes + "\r\n--" + boundary + "--\r\n";
  EXPECT_TRUE(req.body == want) << "the multipart body, byte for byte";
  EXPECT_EQ(whole_state(app).find(bytes.substr(100, 64)), std::string::npos)
      << "the file's bytes never reach the state tree";
}

TEST(AriadneHttpUpload, AsABodyItIsTheFileAsItIs) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetGuard guard;
  auto *net = new FnHttpClient([](const cvc::net::HttpRequest &) { return reply(201, ""); });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string h = picked_files(app).add(memory_file_source("clip.mp4", "MP4BYTES"));
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb,
      upload_program("(http-upload \"http://api.test/put/clip.mp4\" " + lit(h) +
                     " (dict \"as\" \"body\" \"method\" \"PUT\" \"query\" (dict \"v\" 2)))"));
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "up.done"), "true");
  const std::vector<cvc::net::HttpRequest> seen = net->requests();
  ASSERT_EQ(seen.size(), 1u);
  EXPECT_EQ(seen[0].method, "PUT");
  EXPECT_EQ(seen[0].url, "http://api.test/put/clip.mp4?v=2");
  EXPECT_EQ(header(seen[0], "Content-Type"), "video/mp4");
  EXPECT_EQ(seen[0].body, "MP4BYTES");
}

TEST(AriadneHttpUpload, ChunksResumeAfterAConflictAFailureAndALostReply) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetGuard guard;
  auto server = std::make_shared<ChunkServer>();
  server->fail_chunk = [](int n) { return n == 2; }; // a 503: tried again
  server->drop_ack = [](int n) { return n == 4; };   // kept, but the reply is lost
  auto *net = new FnHttpClient([server](const cvc::net::HttpRequest &r) { return (*server)(r); });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string bytes = some_bytes(5000);
  const std::string h = picked_files(app).add(memory_file_source("big.bin", bytes));
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb,
      upload_program("(http-upload \"http://api.test/up\" " + lit(h) +
                     " (dict \"as\" \"chunks\" \"chunk_size\" 1024 \"progress\" \"up.p\""
                     "       \"status_url\" \"http://api.test/up\"))"));
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }, 20000));
  EXPECT_EQ(sval(app, "up.done"), "true") << sval(app, "up.error");
  EXPECT_EQ(sval(app, "up.offset"), "5000");
  EXPECT_EQ(sval(app, "up.p"), "1");
  {
    std::lock_guard<std::mutex> lock(server->m);
    EXPECT_TRUE(server->stored == bytes) << "the server has the file, each byte once";
  }
  const std::vector<cvc::net::HttpRequest> seen = net->requests();
  for (const cvc::net::HttpRequest &r : seen)
    if (r.method == "PATCH") {
      EXPECT_EQ(header(r, "Content-Type"), "application/octet-stream");
      EXPECT_LE(r.body.size(), 1024u);
    }
  // A lost reply: the status GET after it found the server had the chunk.
  EXPECT_TRUE(std::any_of(seen.begin(), seen.end(),
                          [](const cvc::net::HttpRequest &r) { return r.method == "GET"; }));
}

TEST(AriadneHttpUpload, AFailedUploadSaysWhereToResumeFrom) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetGuard guard;
  auto server = std::make_shared<ChunkServer>();
  server->fail_chunk = [](int n) { return n >= 3; }; // down after two chunks
  auto *net = new FnHttpClient([server](const cvc::net::HttpRequest &r) { return (*server)(r); });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string bytes = some_bytes(4096);
  const std::string h = picked_files(app).add(memory_file_source("big.bin", bytes));
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb,
      upload_program("(http-upload \"http://api.test/up\" " + lit(h) +
                     " (dict \"as\" \"chunks\" \"chunk_size\" 1024 \"retries\" 0))"));
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "up.done"), "false");
  EXPECT_EQ(sval(app, "up.status"), "503");
  EXPECT_EQ(sval(app, "up.offset"), "2048");

  // Resumed from there once the server is back: only the rest goes.
  server->fail_chunk = nullptr;
  cvc::state::instance(app)("done").value("");
  run(rt, mb,
      upload_program("(http-upload \"http://api.test/up\" " + lit(h) +
                     " (dict \"as\" \"chunks\" \"chunk_size\" 1024 \"offset\" 2048))"));
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "up.done"), "true") << sval(app, "up.error");
  std::lock_guard<std::mutex> lock(server->m);
  EXPECT_TRUE(server->stored == bytes);
}

TEST(AriadneHttpUpload, OnlyAPickedFileGoesAndOnlyWhereThePolicyAllows) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetGuard guard;
  auto *net = new FnHttpClient([](const cvc::net::HttpRequest &) { return reply(200, "{}"); });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string h = picked_files(app).add(memory_file_source("a.txt", "abc"));
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb,
      "(begin"
      "  (state-set \"e.path\" (try (http-upload \"http://api.test/x\" \"/etc/passwd\")"
      "    (catch (e) (get-attr e \"message\"))))"
      "  (state-set \"e.opt\" (try (http-upload \"http://api.test/x\" " +
          lit(h) +
          " (dict \"body\" \"x\"))"
          "    (catch (e) (get-attr e \"message\"))))"
          "  (set r (http-upload \"http://evil.test/x\" " +
          lit(h) +
          "))"
          "  (state-set \"e.ok\" (get-attr r \"ok\"))"
          "  (state-set \"e.refused\" (get-attr r \"error\"))"
          "  (state-set \"done\" 1))");
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_NE(sval(app, "e.path").find("FILE is a handle file-pick returned"), std::string::npos)
      << sval(app, "e.path");
  EXPECT_NE(sval(app, "e.opt").find("the body is the file"), std::string::npos)
      << sval(app, "e.opt");
  EXPECT_EQ(sval(app, "e.ok"), "false");
  EXPECT_NE(sval(app, "e.refused").find("http://evil.test is not an origin this host allows"),
            std::string::npos)
      << sval(app, "e.refused");
  EXPECT_TRUE(net->requests().empty()) << "nothing was sent";
}

TEST(AriadneHttpUpload, TheCredentialGoesWithEachChunkAndNeverIntoState) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  const std::string token = "upload-token-0123456789abcdef";
  cvc::app app;
  NetGuard guard;
  auto server = std::make_shared<ChunkServer>();
  auto *net = new FnHttpClient([server, token](const cvc::net::HttpRequest &r) {
    if (r.url == "http://api.test/login")
      return reply(200, "{\"token\": \"" + token + "\"}", {"Content-Type: application/json"});
    if (header(r, "Authorization") != "Bearer " + token)
      return reply(401, "{}");
    return (*server)(r);
  });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string h = picked_files(app).add(memory_file_source("big.bin", some_bytes(2500)));
  Runtime rt(app, "");
  PickMock mb;
  rt.set_backend(&mb);
  run(rt, mb,
      "(begin (http-request \"POST\" \"http://api.test/login\""
      "    (dict \"capture_auth\" (dict \"name\" \"api\")))"
      "  " +
          upload_program("(http-upload \"http://api.test/up\" " + lit(h) +
                         " (dict \"as\" \"chunks\" \"chunk_size\" 1024 \"auth\" \"api\"))") +
          ")");
  ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
  EXPECT_EQ(sval(app, "up.done"), "true") << sval(app, "up.error");
  int chunks = 0;
  for (const cvc::net::HttpRequest &r : net->requests())
    if (r.method == "PATCH") {
      ++chunks;
      EXPECT_EQ(header(r, "Authorization"), "Bearer " + token);
    }
  EXPECT_EQ(chunks, 3);
  EXPECT_EQ(whole_state(app).find(token), std::string::npos);
}

TEST(AriadneHttpUpload, StartedChunksLetFramesGoOnAndATornDownDocumentAborts) {
  if (!cvc::net::have_http_backend())
    GTEST_SKIP() << "libcvc built without an HTTP backend";
  cvc::app app;
  NetGuard guard;
  auto server = std::make_shared<ChunkServer>();
  auto *net =
      new DeferredHttpClient([server](const cvc::net::HttpRequest &r) { return (*server)(r); });
  cvc::net::set_http_client(std::unique_ptr<cvc::net::HttpClient>(net));
  ASSERT_EQ(enable_net(app, allow({"http://api.test"})), "");
  const std::string bytes = some_bytes(3000);
  const std::string h = picked_files(app).add(memory_file_source("big.bin", bytes));
  {
    Runtime rt(app, "");
    PickMock mb;
    rt.set_backend(&mb);
    cvc::state::instance(app)("ui.frames").value("0");
    rt.set_tick_program("(state-set \"ui.frames\" (+ (int (state-get \"ui.frames\")) 1))");
    run(rt, mb,
        upload_program("(http-upload \"http://api.test/up\" " + lit(h) +
                       " (dict \"as\" \"chunks\" \"chunk_size\" 1024 \"progress\" \"up.p\"))"));
    std::vector<std::string> progress;
    for (int chunk = 0; chunk < 3; ++chunk) {
      ASSERT_TRUE(pump_until(rt, [&] { return net->live() == 1; })) << "chunk " << chunk;
      progress.push_back(sval(app, "up.p"));
      frames(rt, 10);
      EXPECT_EQ(sval(app, "done"), "") << "the program waits on the chunk";
      ASSERT_TRUE(net->answer());
    }
    ASSERT_TRUE(pump_until(rt, [&] { return sval(app, "done") == "1"; }));
    EXPECT_EQ(progress, (std::vector<std::string>{"0", "0.3413", "0.6827"}));
    EXPECT_EQ(sval(app, "up.p"), "1");
    EXPECT_EQ(sval(app, "up.done"), "true") << sval(app, "up.error");
    EXPECT_GE(std::stoi(sval(app, "ui.frames")), 30) << "on_tick ran meanwhile";
    EXPECT_EQ(net->sends, 0) << "nothing waited in place";
    std::lock_guard<std::mutex> lock(server->m);
    EXPECT_TRUE(server->stored == bytes);
  }
  {
    // A document torn down mid-upload aborts the chunk out.
    auto rt = std::make_unique<Runtime>(app, "");
    PickMock mb;
    rt->set_backend(&mb);
    run(*rt, mb,
        upload_program("(http-upload \"http://api.test/up2\" " + lit(h) +
                       " (dict \"as\" \"chunks\" \"chunk_size\" 1024))"));
    ASSERT_TRUE(pump_until(*rt, [&] { return net->live() == 1; }));
    rt.reset();
    EXPECT_EQ(net->live(), 0u) << "its request was aborted";
  }
}
