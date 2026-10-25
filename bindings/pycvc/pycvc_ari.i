// pycvc_ari.i — Python binding for cvc::gl::ariadne::AriRuntime: open a window, load a .ari
// document, run its event/render loop, and register Python host verbs — a full Ariadne app from
// Python. %include'd LAST from pycvc_gl.i, after SceneRenderer / CameraController / ImGuiOverlay so
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

// Keep the window/camera/overlay the AriRuntime borrows alive for its whole life. The ctor has a
// single signature, so its proxy takes NAMED parameters (no `args`; see the %pythonappend note in
// pycvc_gl.i).
%pythonappend cvc::gl::ariadne::AriRuntime::AriRuntime %{
    self._pycvc_view, self._pycvc_cam, self._pycvc_overlay = view, cam, ui
%}

%include "cvc/gl/ariadne/AriRuntime.h"
