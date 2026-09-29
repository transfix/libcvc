// pycvc_ari.i — Python binding for cvc::gl::ariadne::AriRuntime: open a window, load a .ari
// document, run its event/render loop, and register Python host verbs — a full Ariadne app from
// Python. %include'd LAST from pycvc_gl.i, after SceneRenderer / CameraController / ImGuiOverlay so
// their proxy types resolve, and after the file-scope std::function<void()> typemap (reused by on()
// and register_verb()). NOTE: this is a SWIG interface file — do NOT run clang-format on it.
%{
#include <cvc/gl/ariadne/AriRuntime.h>
%}

// register_async_verb takes a value-returning callable; the string result is posted to the .ari
// program. Mirrors the std::function<void()> typemap (pycvc_gl.i), returning str(result).
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
      PyErr_Print();
    } else {
      PyObject *s = PyObject_Str(r);
      const char *c = s ? PyUnicode_AsUTF8(s) : nullptr;
      if (c)
        out = c;
      Py_XDECREF(s);
      Py_DECREF(r);
    }
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

// Keep the window/camera/overlay the AriRuntime borrows alive for its whole life.
%pythonappend cvc::gl::ariadne::AriRuntime::AriRuntime %{
    if len(args) >= 3:
        self._pycvc_view, self._pycvc_cam, self._pycvc_overlay = args[0], args[1], args[2]
%}

%include "cvc/gl/ariadne/AriRuntime.h"
