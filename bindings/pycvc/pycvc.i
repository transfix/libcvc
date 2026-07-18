// pycvc.i — SWIG interface for the pycvc Python bindings.
//
// Scope (v0): the general-purpose geometry builder. Volume/state bindings
// and numpy fast-paths follow. SWIG only ever sees this facade — never
// libcvc's headers — because pycvc_geometry.h forward-declares
// cvc::geometry.
%module pycvc

%{
#include "pycvc_geometry.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <exception.i>

// Surface C++ exceptions (e.g. bad array lengths, unreadable files) as
// Python exceptions instead of aborting the interpreter.
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

namespace std {
  %template(DoubleVector) vector<double>;
  %template(IndexVector) vector<unsigned long>;
}

// native() exposes cvc::geometry (an opaque, forward-declared type) for
// C++ host apps only — not meaningful from Python.
%ignore pycvc::Geometry::native;

%include "pycvc_geometry.h"
