// python_engine.h — the embedded CPython of the scripting hosts, arix, ariglx and arisrvx
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §3 and §6.5, ARIADNE.md §9.6 and §9.8).
//
// One interpreter per process, booted once by the host and finalized once, never per document:
// numpy and the SWIG modules (pycvc, pycvc_gl) are not sub-interpreter safe. After boot the GIL is
// parked, so the host's frames run without it and every crossing into Python takes it (as pycvc's
// callbacks do), and a document's async Python functions run on the app's pool.
//
// What it gives the host:
//   - attach(): the live app, the document's AppRuntime and (a GL host) its window's objects, lent
//     to Python through the builtin module `_arihost`, which pycvc_gl.ariadne.Host() adopts (its
//     embedded mode). It also installs the document side on that AppRuntime: a document's
//     python: block runs at load, before requires: is checked, and the functions it exports become
//     host functions; `python` and `python.<module>` are capability tokens that hold.
//   - run_script(): a Python program as the entry point (`ariglx run app.py`), run as __main__.
//   - python_main(): `<host> python [args]`, the embedded interpreter as a plain python3.
//   - A server host (arisrvx, EngineConfig::server): serve(), a Python program run beside the
//     relay on a script thread of the host's, with one asyncio loop (pycvc.ariadne, built in as
//     ariadne_server.py). Its writes reach the relay's tree only through a ServerLink
//     (server_link.h), which the relay applies on its own thread.
//
// Where its Python lives: PYTHONHOME when set; else an installed prefix beside the executable
// (<prefix>/bin/<host> with the cvcpkg python312 in <prefix>); else the Python the host was built
// against. The cvcGL objects cross as void*, so the engine itself links no cvcGL.
#pragma once

#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
class AppRuntime;

namespace python {

class ServerLink;

struct EngineConfig {
  std::string host;              // the host's name, as its messages say it: "arix", "ariglx"
  std::vector<std::string> path; // --python-path entries: on sys.path as PYTHONPATH's are, first
  // A server host (arisrvx): pycvc.ariadne, the server side of the engine, is built in and
  // imported at boot, ahead of any pycvc installed beside the host.
  bool server = false;
};

// How a server host runs its program (Engine::serve).
struct ServeOptions {
  std::string http_listen;       // --http-listen ADDR:PORT: serve the program's `app` there
  double shutdown_timeout = 10.; // --shutdown-timeout: the ASGI servers' graceful stop, seconds
  bool check = false;            // arisrvx check: run the program without serving, and report
  // --script-slow-ms: a sync function (a client's call, a server document's export) that runs
  // longer warns with its name, since it holds up every other caller. 0: never.
  double slow_ms = 50.0;
  // --call-concurrency, --call-rate: per principal, the calls a client may have in flight and make
  // in a second (0: no limit); past either, a call is answered rate_limited.
  long call_concurrency = 8;
  double call_rate = 0.0;
  // --auth-timeout: how long the relay waits for the authenticate hook's verdict.
  double auth_timeout = 5.0;
  // The relay's own listeners, for check to probe: {flag, ADDR:PORT}.
  std::vector<std::pair<std::string, std::string>> ports;
};

// What a host lends its Python while it runs a document. The host owns every object; they are
// lent for the life of the Attachment that attach() returns.
struct HostHandles {
  cvc::app *app = nullptr;
  AppRuntime *runtime = nullptr; // null in a server host without a server document
  std::string mode;              // "window", "offscreen", "terminal", "headless" or "server"
  // A server host's: where Python's writes queue for the relay, and the relay's events for the
  // loop. With it, Python never writes the tree itself.
  ServerLink *server = nullptr;
  // A GL host's: cvc::gl::ariadne::AriRuntime, SceneRenderer, CameraController, ImGuiOverlay.
  void *ari_runtime = nullptr;
  void *view = nullptr;
  void *camera = nullptr;
  void *overlay = nullptr;
  // The host's frame loop, for a script's host.run(): runs the document until it closes and
  // returns the exit status. Null where the host has no loop to give (a check).
  std::function<int()> run_loop;
  // Whether the host's runtime is inside a frame now (a GL host's AriRuntime::in_frame): what
  // calls host.run() then is a frame hook, or a function or verb the frame runs, and it refuses.
  std::function<bool()> in_frame;
};

// The loan attach() makes: while it lives, _arihost hands Python the host's objects; when it goes,
// they are taken back. Destroy it before the objects it lends (declare it after them).
class Attachment {
public:
  Attachment() = default;
  Attachment(Attachment &&other) noexcept;
  Attachment &operator=(Attachment &&other) noexcept;
  ~Attachment();

private:
  friend class Engine;
  explicit Attachment(bool live) : live_(live) {}
  bool live_ = false;
};

class Engine {
public:
  // Boot CPython. ok() says whether it came up; error() says why not. The interpreter must outlive
  // every AppRuntime it is attached to (the Python functions a document registered hold references
  // the runtime drops when it dies), so a host constructs its Engine before its app and runtime.
  explicit Engine(EngineConfig config);
  ~Engine(); // finalizes the interpreter
  Engine(const Engine &) = delete;
  Engine &operator=(const Engine &) = delete;

  bool ok() const { return ok_; }
  const std::string &error() const { return error_; }
  std::string version() const; // "3.12.10"

  // Lend `handles` to Python (_arihost) and install the document side on handles.runtime: the
  // python: block, the default per-action caps. Call once per runtime, before it loads a document;
  // a later attach() lends another runtime's objects (ari test runs one AppRuntime per document).
  // A server host lends its app and ServerLink, and its server document's runtime when it has one:
  // that document's python: block is refused there (its Python would run on the relay's loop).
  [[nodiscard]] Attachment attach(const HostHandles &handles);

  // Register this engine on `app` (register_script_engine): its `python` and `python.<module>`
  // tokens hold for the app's documents, and their python: blocks are kept. attach() does it for a
  // runtime's app; a server host calls it as soon as its app exists, since it reads its server
  // document before it attaches. Nothing when the interpreter did not come up.
  void register_on(cvc::app &app);

  // Run the program at `path` as __main__, with sys.argv = [path, args...] and its directory first
  // on sys.path. Returns its exit status: 0, SystemExit's code, or 1 once an uncaught exception's
  // traceback is printed.
  int run_script(const std::string &path, const std::vector<std::string> &args);
  // Whether the host's loop has run (a program's host.run(), or run_loop), and the status the
  // program's host.run() returned.
  bool loop_ran() const;
  int loop_status() const;
  // Run the host's own frame loop (`loop`, the HostHandles::run_loop a program's host.run() runs)
  // for a document or after a program: returns loop()'s status. While a loop runs, here or through
  // host.run(), _arihost.run() refuses, so a plt.show() or host.run() from code the document runs
  // never starts a second loop inside the first. A host runs every loop of its own through this.
  int run_loop(const std::function<int()> &loop);

  // A server host's program (pycvc.ariadne's runner, ARIADNE_SCRIPTING_HOSTS.md §6.5): run `path`
  // as __main__ on the calling thread, the host's script thread, with an asyncio loop of its own;
  // then run that loop until the relay posts "shutdown" or the program stops it, and shut down its
  // ASGI servers. Call after attach() lent handles with a ServerLink. Returns the exit status. With
  // options.check it serves nothing: it reports what the program would serve and whether the ports
  // are free.
  int serve(const std::string &path, const std::vector<std::string> &args,
            const ServeOptions &options);

  // `<host> python [args]`: the embedded interpreter as a plain python3 (a REPL, -c, -m, a
  // script), with the same packages. Call INSTEAD of constructing an Engine.
  static int python_main(const EngineConfig &config, const std::vector<std::string> &args);

private:
  EngineConfig config_;
  bool ok_ = false;
  std::string error_;
  void *mainState_ = nullptr; // the PyThreadState parked after boot
};

} // namespace python
} // namespace ariadne
} // namespace cvc
