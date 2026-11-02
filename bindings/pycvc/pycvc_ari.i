// pycvc_ari.i — Python binding for cvc::gl::ariadne::AriRuntime: open a window, load a .ari
// document, run its event/render loop, and register Python host verbs and functions — a full
// Ariadne app from Python. pycvc_gl.ariadne.Host (pymod_gl/ariadne.py) is the Python host built on
// it. %include'd LAST from pycvc_gl.i, after SceneRenderer / CameraController / ImGuiOverlay so
// their proxy types resolve, and after the file-scope std::function<void()> typemap (reused by on()
// and register_verb()). NOTE: this is a SWIG interface file — do NOT run clang-format on it.
%{
#include <cvc/gl/ariadne/AriRuntime.h>

#include "pycvc_value.h" // pycvc::python_exception

// The Python exception set now, taken (cleared) as a PythonError for the DSL
// (docs/STATE_EXEC_EXCEPTIONS.md): its type's name, its str(), and the exception itself.
static pycvc::python_exception pycvc_ari_take_exception(const char *what) {
  PyObject *type = nullptr, *val = nullptr, *tb = nullptr;
  PyErr_Fetch(&type, &val, &tb);
  PyErr_NormalizeException(&type, &val, &tb);
  if (val != nullptr && tb != nullptr)
    PyException_SetTraceback(val, tb);
  std::string msg = "error", type_name;
  if (val != nullptr)
    if (PyObject *s = PyObject_Str(val)) {
      if (const char *c = PyUnicode_AsUTF8(s))
        msg = c;
      Py_DECREF(s);
    }
  if (type != nullptr) {
    if (PyObject *q = PyObject_GetAttrString(type, "__qualname__")) {
      if (const char *c = PyUnicode_AsUTF8(q))
        type_name = c;
      Py_DECREF(q);
    }
    if (PyObject *m = PyObject_GetAttrString(type, "__module__")) {
      const char *c = PyUnicode_Check(m) ? PyUnicode_AsUTF8(m) : nullptr;
      if (c && std::string(c) != "builtins" && !type_name.empty())
        type_name = std::string(c) + "." + type_name;
      Py_DECREF(m);
    }
  }
  PyErr_Clear();
  std::shared_ptr<PyObject> held;
  if (val != nullptr) {
    Py_INCREF(val);
    held.reset(val, [](PyObject *p) {
      PyGILState_STATE g = PyGILState_Ensure();
      Py_DECREF(p);
      PyGILState_Release(g);
    });
  }
  Py_XDECREF(type);
  Py_XDECREF(val);
  Py_XDECREF(tb);
  return pycvc::python_exception(std::string(what) + ": " + msg, type_name, std::move(held));
}
%}

// register_async_verb takes a value-returning callable: it runs on the app's background task pool
// as the verb's async kernel, and str(result) is the verb's value. A Python exception raises at
// the call as a PythonError (docs/STATE_EXEC_EXCEPTIONS.md). Mirrors the std::function<void()>
// typemap (pycvc_gl.i).
%typemap(in) std::function<std::string()> {
  if (!PyCallable_Check($input))
    SWIG_exception_fail(SWIG_TypeError, "expected a callable for std::function<std::string()>");
  Py_INCREF($input);
  std::shared_ptr<PyObject> _cb($input, [](PyObject *p) {
    PyGILState_STATE g = PyGILState_Ensure();
    Py_DECREF(p);
    PyGILState_Release(g);
  });
  $1 = [_cb]() -> std::string {
    PyGILState_STATE g = PyGILState_Ensure();
    std::string out;
    PyObject *r = PyObject_CallObject(_cb.get(), nullptr);
    if (!r) {
      pycvc::python_exception error = pycvc_ari_take_exception("python async verb");
      PyGILState_Release(g);
      throw error;
    }
    PyObject *s = PyObject_Str(r);
    const char *c = s ? PyUnicode_AsUTF8(s) : nullptr;
    if (c)
      out = c;
    Py_XDECREF(s);
    Py_DECREF(r);
    PyErr_Clear();
    PyGILState_Release(g);
    return out;
  };
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) std::function<std::string()> {
  $1 = PyCallable_Check($input) ? 1 : 0;
}

// set_root takes a cvc::ariadne::Widget by value (the programmatic-tree path); not marshallable from
// Python — the Python surface loads .ari files instead.
%ignore cvc::gl::ariadne::AriRuntime::set_root;

// ── the GIL ──────────────────────────────────────────────────────────────────────────────────────
// A frame runs the document's programs, and they call Python: an on() handler or a function, on
// this thread, and an async function's kernel on a pool thread. Each takes the GIL for its call.
// So the calls that run a frame, a load (its init:), a wait for async work and the teardown release
// it: holding it there would stall a kernel until the frame returns (forever, under run()), and a
// teardown that waits for one would deadlock.
%{
namespace pycvc_ari {
// The Py_BEGIN_ALLOW_THREADS / Py_END_ALLOW_THREADS pair as a scope (a throw restores the GIL).
class nogil {
public:
  nogil() : ts_(PyEval_SaveThread()) {}
  ~nogil() { PyEval_RestoreThread(ts_); }
  nogil(const nogil &) = delete;
  nogil &operator=(const nogil &) = delete;

private:
  PyThreadState *ts_;
};
} // namespace pycvc_ari
%}
%define PYCVC_ARI_NOGIL(NAME)
%exception NAME {
  try {
    pycvc_ari::nogil pycvc_ari_nogil;
    $action
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "pycvc_gl: C++ exception (see cvcGL)");
  }
}
%enddef
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::frame)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::render)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::step)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::run)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::load)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::reload_if_changed)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::wait_idle)
PYCVC_ARI_NOGIL(cvc::gl::ariadne::AriRuntime::~AriRuntime)

// ── host functions with arguments and a value ────────────────────────────────────────────────────
// The C++ register_function / register_async_function take state_exec callables; Python passes a
// pycvc.host_function (pycvc_ariadne.i), built in pycvc where the value conversion lives and
// shared with this module through SWIG's type table.
%ignore cvc::gl::ariadne::AriRuntime::register_function;
%ignore cvc::gl::ariadne::AriRuntime::register_async_function;
%extend cvc::gl::ariadne::AriRuntime {
  void register_host_function(const pycvc::host_function &f) {
    if (f.is_async())
      $self->register_async_function(f.name, f.marshal, f.done_channel);
    else
      $self->register_function(f.name, f.call);
  }
}

// run_program(script, done=None): done(how) is called from a later frame, on its thread, with ""
// when the program finished, else why not. None (or nothing) passes no callback.
%typemap(in) std::function<void(const std::string &)> {
  // Always assign: SWIG holds a by-value std::function in a SwigValueWrapper, which is empty (a
  // null pointer) until assigned, so None must still pass an (empty) function.
  $1 = std::function<void(const std::string &)>();
  if ($input != Py_None) {
    if (!PyCallable_Check($input))
      SWIG_exception_fail(SWIG_TypeError, "expected a callable or None");
    Py_INCREF($input);
    std::shared_ptr<PyObject> _cb($input, [](PyObject *p) {
      PyGILState_STATE g = PyGILState_Ensure();
      Py_DECREF(p);
      PyGILState_Release(g);
    });
    $1 = [_cb](const std::string &how) {
      PyGILState_STATE g = PyGILState_Ensure();
      PyObject *r = PyObject_CallFunction(_cb.get(), "s", how.c_str());
      if (!r)
        PyErr_Print();
      Py_XDECREF(r);
      PyGILState_Release(g);
    };
  }
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) std::function<void(const std::string &)> {
  $1 = ($input == Py_None || PyCallable_Check($input)) ? 1 : 0;
}

// Keep the window/camera/overlay the AriRuntime borrows alive for its whole life (~AriRuntime
// resets the overlay's draw callback), through pycvc._keepalive. The ctor has a single
// signature, so its proxy takes NAMED parameters (no `args`; see the %pythonappend note in
// pycvc_gl.i).
%pythonappend cvc::gl::ariadne::AriRuntime::AriRuntime %{
    self._pycvc_view, self._pycvc_cam, self._pycvc_overlay = (
        pycvc._keepalive(view), pycvc._keepalive(cam), pycvc._keepalive(ui))
%}

%include "cvc/gl/ariadne/AriRuntime.h"
