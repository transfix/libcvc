// pycvc_ariadne.i — the GL-free half of a Python Ariadne host, in pycvc:
//
// - pycvc.host_function(name, fn, is_async=False, with_call=False, done_channel=""): a Python
//   callable as a host function with arguments and a value (pycvc::host_function, pycvc_value.h).
//   pycvc builds it because the value conversion lives here; pycvc_gl.AriRuntime and pycvc.AriApp
//   register it (register_host_function), and the document calls it as (name a b ...).
// - pycvc.AriApp(app, prefix="ari"): cvc::ariadne::AppRuntime with no backend and no scene, for
//   running a document's programs with no window and no GL (tests, CI, a server). Its init:,
//   residents, actions, host functions and state behave as under arigl; nothing is drawn, so a
//   computed bind: or text: is never evaluated, and a document that requires gl.scene does not
//   load. pycvc_gl.ariadne.Host(mode="headless") drives it.
// - pycvc.ari_app_attach(app, runtime): an AriApp over a scripting host's own AppRuntime (arix,
//   ariglx lend it as a "cvc.ariadne.AppRuntime" capsule, _arihost.runtime()), which it does not
//   own: the host's python: blocks register their functions through it, and
//   pycvc_gl.ariadne.Host() drives it in arix.
//
// The GIL: frame()/step()/load()/run_program's drain and wait_idle() release it, so a Python
// function the document calls (which takes it back) and an async function on a pool thread run.
// %include'd from pycvc.i after pycvc_exec.h. NOTE: a SWIG interface file — do NOT run clang-format.
%{
#include <cvc/ariadne/app_runtime.h>
#include <cvc/core/app.h>
#include <cvc/state/state_exec/async_host.h>
#include <stdexcept>

namespace pycvc {

// A Python callable as a C++ callback: it owns a reference (dropped under the GIL), takes the GIL
// for each call, and prints (never propagates) a Python exception, as pycvc_gl's handlers do.
static std::function<void()> ariadne_void_callback(PyObject *fn, const char *who) {
  if (fn == nullptr || !PyCallable_Check(fn))
    throw std::invalid_argument(std::string(who) + ": expected a callable");
  std::shared_ptr<PyObject> held = python_ref(fn);
  return [held]() {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallObject(held.get(), nullptr);
    if (r == nullptr)
      PyErr_Print();
    Py_XDECREF(r);
    PyGILState_Release(g);
  };
}

static std::function<void(const std::string &)> ariadne_done_callback(PyObject *fn,
                                                                     const char *who) {
  if (fn == nullptr || fn == Py_None)
    return nullptr;
  if (!PyCallable_Check(fn))
    throw std::invalid_argument(std::string(who) + ": done must be callable or None");
  std::shared_ptr<PyObject> held = python_ref(fn);
  return [held](const std::string &how) {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallFunction(held.get(), "s", how.c_str());
    if (r == nullptr)
      PyErr_Print();
    Py_XDECREF(r);
    PyGILState_Release(g);
  };
}

host_function make_host_function(const std::string &name, PyObject *fn, bool is_async,
                                 bool with_call, const std::string &done_channel) {
  if (name.empty())
    throw std::invalid_argument("pycvc.host_function: the name is empty");
  host_function f;
  f.name = name;
  if (is_async) {
    f.marshal = python_async_marshal("pycvc.host_function", name, fn, with_call);
    f.done_channel = done_channel;
  } else {
    if (with_call)
      throw std::invalid_argument("pycvc.host_function: with_call is for an async function");
    if (!done_channel.empty())
      throw std::invalid_argument("pycvc.host_function: done_channel is for an async function");
    f.call = python_native_fn("pycvc.host_function", name, fn);
  }
  return f;
}

// A GL-free Ariadne host: an AppRuntime over `app`'s state at `prefix`, with no backend and no
// scene. Its methods mirror pycvc_gl.AriRuntime's, so pycvc_gl.ariadne.Host drives either.
class AriApp {
public:
  AriApp(std::shared_ptr<cvc::app> app, const std::string &prefix)
      : app_(std::move(app)), prefix_(prefix) {
    if (!app_)
      throw std::invalid_argument("pycvc.AriApp: app is None");
    if (prefix_.empty())
      throw std::invalid_argument("pycvc.AriApp: the prefix is empty");
    // Build the app's lazily made singletons on this thread before a pool thread needs them.
    app_->exec_scheduler();
    app_->exec_async_host();
    owned_ = std::make_unique<cvc::ariadne::AppRuntime>(*app_, prefix_);
    rt_ = owned_.get();
  }
  // Over a runtime someone else owns (ari_app_attach): `rt` outlives this AriApp.
  AriApp(std::shared_ptr<cvc::app> app, cvc::ariadne::AppRuntime *rt)
      : app_(std::move(app)), prefix_(rt ? rt->prefix() : std::string()), rt_(rt) {
    if (!app_ || !rt_)
      throw std::invalid_argument("pycvc.ari_app_attach: no app or no runtime");
  }
  ~AriApp() { owned_.reset(); }

  void add_component_path(const std::string &dir) { rt_->add_component_path(dir); }
  void on(const std::string &event, PyObject *handler) {
    rt_->on(event, ariadne_void_callback(handler, "AriApp.on"));
  }
  void register_host_function(const host_function &f) {
    if (f.is_async())
      rt_->register_async_function(f.name, f.marshal, f.done_channel);
    else
      rt_->register_function(f.name, f.call);
  }
  void register_quit() { rt_->register_quit(); }
  void set_document_args(const std::vector<std::string> &argv, const std::string &invocation,
                         bool validate) {
    rt_->set_document_args(argv, invocation, validate);
  }
  std::vector<std::string> load(const std::string &path) { return rt_->load(path); }
  void frame(double dt_seconds) {
    (void)dt_seconds; // no camera, no scene clock: the drain is the frame
    if (inFrame_)
      throw std::logic_error("AriApp::frame: called from inside a frame (a host handler or "
                             "function)");
    inFrame_ = true;
    try {
      rt_->drain();
    } catch (...) {
      inFrame_ = false;
      throw;
    }
    inFrame_ = false;
  }
  void render() {} // nothing is drawn headless
  void step(double dt_seconds) { frame(dt_seconds); }
  bool should_close() const { return rt_->should_close(); }
  void request_close() { rt_->request_close(); }
  int exit_code() const { return rt_->exit_code(); }
  bool idle() const { return rt_->idle(); }
  void run_program(const std::string &script, PyObject *done) {
    rt_->run_program(script, ariadne_done_callback(done, "AriApp.run_program"));
  }
  bool post_signal(const std::string &signal, const std::string &source) {
    return rt_->post_signal(signal, source);
  }
  std::vector<std::string> take_warnings() { return rt_->take_warnings(); }
  bool reload_if_changed() { return rt_->reload_if_changed(); }
  // sim.step / sim.restart, as the document's Step and Restart buttons raise them
  // (AppRuntime::step_world / restart_world).
  void step_world(unsigned long long n = 1) { rt_->step_world(n); }
  bool restart_world() { return rt_->restart_world(); }
  std::string prefix() const { return prefix_; }
  bool has_function(const std::string &name) { return rt_->runtime().has_function(name); }
  void wait_idle() {
    if (inFrame_)
      throw std::logic_error("AriApp::wait_idle: called from inside a frame");
    rt_->wait_idle(); // the app's host and the document's own group's (S1.9)
  }
  static bool have_yaml() { return cvc::ariadne::AppRuntime::have_yaml(); }

private:
  std::shared_ptr<cvc::app> app_;
  std::string prefix_;
  std::unique_ptr<cvc::ariadne::AppRuntime> owned_; // null when attached
  cvc::ariadne::AppRuntime *rt_ = nullptr;
  bool inFrame_ = false;
};

// An AriApp over the AppRuntime in a "cvc.ariadne.AppRuntime" capsule (a scripting host's,
// _arihost.runtime()), not owning it.
AriApp *ari_app_attach(std::shared_ptr<cvc::app> app, PyObject *runtime) {
  if (!runtime || !PyCapsule_CheckExact(runtime))
    throw std::invalid_argument("pycvc.ari_app_attach: runtime is not a PyCapsule");
  void *p = PyCapsule_GetPointer(runtime, "cvc.ariadne.AppRuntime");
  if (!p) {
    PyErr_Clear();
    throw std::invalid_argument(
        "pycvc.ari_app_attach: the capsule is not a \"cvc.ariadne.AppRuntime\"");
  }
  return new AriApp(std::move(app), static_cast<cvc::ariadne::AppRuntime *>(p));
}

} // namespace pycvc
%}

// The long calls release the GIL (a Python function the document calls retakes it), and so does
// the destructor: ~AppRuntime ends the document's processes. pycvc::gil_release is pycvc_lod.i's.
// File scope, so the names are qualified.
%define PYCVC_ARIADNE_NOGIL(NAME)
%exception NAME {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  }
  PYCVC_STATE_EXEC_CATCH
}
%enddef
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::frame)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::step)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::load)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::wait_idle)
PYCVC_ARIADNE_NOGIL(pycvc::AriApp::~AriApp)

%feature("kwargs") pycvc::make_host_function;
%rename(host_function) pycvc::make_host_function;
%rename(HostFunction) pycvc::host_function;
%nodefaultctor pycvc::host_function;
%feature("kwargs") pycvc::AriApp::AriApp;
%newobject pycvc::ari_app_attach;
%feature("kwargs") pycvc::AriApp::run_program;
%feature("kwargs") pycvc::AriApp::post_signal;
%feature("kwargs") pycvc::AriApp::set_document_args;
// AriApp keeps its app alive in C++ (a shared_ptr), and its handlers / functions their callables.
// What SWIG sees: the data the C++ struct and class hold stay out of the proxy.
namespace pycvc {
struct host_function {
  %immutable;
  std::string name;
  std::string done_channel;
  %mutable;
  bool is_async() const;
};
host_function make_host_function(const std::string &name, PyObject *fn, bool is_async = false,
                                 bool with_call = false, const std::string &done_channel = "");

class AriApp {
public:
  AriApp(std::shared_ptr<cvc::app> app, const std::string &prefix = "ari");
  ~AriApp();
  void add_component_path(const std::string &dir);
  void on(const std::string &event, PyObject *handler);
  void register_host_function(const host_function &f);
  void register_quit();
  void set_document_args(const std::vector<std::string> &argv, const std::string &invocation = "",
                         bool validate = true);
  std::vector<std::string> load(const std::string &path);
  void frame(double dt_seconds);
  void render();
  void step(double dt_seconds);
  bool should_close() const;
  void request_close();
  int exit_code() const;
  bool idle() const;
  void run_program(const std::string &script, PyObject *done = nullptr);
  bool post_signal(const std::string &signal, const std::string &source = "host");
  std::vector<std::string> take_warnings();
  bool reload_if_changed();
  void step_world(unsigned long long n = 1);
  bool restart_world();
  std::string prefix() const;
  bool has_function(const std::string &name);
  void wait_idle();
  static bool have_yaml();
};
AriApp *ari_app_attach(std::shared_ptr<cvc::app> app, PyObject *runtime);
} // namespace pycvc
