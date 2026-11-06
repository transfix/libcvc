// python_engine.h — the embedded CPython of the scripting hosts, arix and ariglx
// (docs/roadmap/ARIADNE_SCRIPTING_HOSTS.md §3, ARIADNE.md §9.6).
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
//
// Where its Python lives: PYTHONHOME when set; else an installed prefix beside the executable
// (<prefix>/bin/<host> with the cvcpkg python312 in <prefix>); else the Python the host was built
// against. The cvcGL objects cross as void*, so the engine itself links no cvcGL.
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace cvc {
class app;
namespace ariadne {
class AppRuntime;

namespace python {

struct EngineConfig {
  std::string host;              // the host's name, as its messages say it: "arix", "ariglx"
  std::vector<std::string> path; // --python-path entries: on sys.path as PYTHONPATH's are, first
};

// What a host lends its Python while it runs a document. The host owns every object; they are
// lent for the life of the Attachment that attach() returns.
struct HostHandles {
  cvc::app *app = nullptr;
  AppRuntime *runtime = nullptr;
  std::string mode; // "window", "offscreen", "terminal" or "headless"
  // A GL host's: cvc::gl::ariadne::AriRuntime, SceneRenderer, CameraController, ImGuiOverlay.
  void *ari_runtime = nullptr;
  void *view = nullptr;
  void *camera = nullptr;
  void *overlay = nullptr;
  // The host's frame loop, for a script's host.run(): runs the document until it closes and
  // returns the exit status. Null where the host has no loop to give (a check).
  std::function<int()> run_loop;
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
  [[nodiscard]] Attachment attach(const HostHandles &handles);

  // Run the program at `path` as __main__, with sys.argv = [path, args...] and its directory first
  // on sys.path. Returns its exit status: 0, SystemExit's code, or 1 once an uncaught exception's
  // traceback is printed.
  int run_script(const std::string &path, const std::vector<std::string> &args);
  // Whether the program ran the host's loop (host.run()), and the status it returned.
  bool loop_ran() const;
  int loop_status() const;

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
