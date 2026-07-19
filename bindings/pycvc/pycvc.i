// pycvc.i — SWIG interface for the pycvc Python bindings.
//
// Scope (v0): the general-purpose geometry builder. Volume/state bindings
// and numpy fast-paths follow. SWIG only ever sees this facade — never
// libcvc's headers — because pycvc_geometry.h forward-declares
// cvc::geometry.
%module pycvc

%{
#include "pycvc_geometry.h"
#include "pycvc_volume.h"
%}

%include <std_string.i>
%include <std_vector.i>
%include <exception.i>

// Surface C++ exceptions (e.g. bad array lengths, unreadable files) as
// Python exceptions instead of aborting the interpreter.
// cvc exceptions derive from boost::exception (NOT std::exception), so the
// catch(...) fallback is required to translate them into Python errors
// instead of std::terminate.
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "pycvc: C++ exception (see libcvc)");
  }
}

namespace std {
  %template(DoubleVector) vector<double>;
  %template(IndexVector) vector<unsigned long>;
}

// native() exposes an opaque, forward-declared cvc type for C++ host apps
// only — not meaningful from Python.
%ignore pycvc::Geometry::native;
%ignore pycvc::Volume::native;

%include "pycvc_geometry.h"
%include "pycvc_volume.h"
