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
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

// Capsule destructor: releases the shared_ptr<void> that keeps the C++
// storage alive for as long as any numpy view of it exists.
static void pycvc_owner_capsule_dtor(PyObject* cap) {
  void* p = PyCapsule_GetPointer(cap, "pycvc_owner");
  delete static_cast<std::shared_ptr<void>*>(p);
}
%}

%init %{
  import_array();
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

// ── Zero-copy numpy views ──────────────────────────────────────────
// A facade method returning a pycvc::ArrayView becomes a numpy array that
// VIEWS the C++ buffer (no data copy). The array's base is a capsule that
// owns a shared_ptr to the C++ storage, so the memory outlives the facade
// object for exactly as long as any view of it does — safe zero-copy.
namespace pycvc { struct ArrayView; }

%typemap(out) pycvc::ArrayView {
  const pycvc::ArrayView& _v = $1;
  int _nd = static_cast<int>(_v.shape.size());
  std::vector<npy_intp> _dims(_v.shape.begin(), _v.shape.end());
  int _npt = (_v.dtype == pycvc::DType::Float64)   ? NPY_DOUBLE
             : (_v.dtype == pycvc::DType::Float32) ? NPY_FLOAT
                                                   : NPY_UINT64;
  npy_intp _n = 1;
  for (npy_intp _d : _dims) _n *= _d;
  PyObject* _arr = nullptr;
  if (_n == 0 || _v.data == nullptr) {
    _arr = PyArray_EMPTY(_nd, _dims.data(), _npt, 0);  // empty, no view needed
  } else {
    _arr = PyArray_SimpleNewFromData(_nd, _dims.data(), _npt,
                                     const_cast<void*>(_v.data));
    if (_arr) {
      if (!_v.writable)
        PyArray_CLEARFLAGS(reinterpret_cast<PyArrayObject*>(_arr),
                           NPY_ARRAY_WRITEABLE);
      auto* _own = new std::shared_ptr<void>(_v.owner);
      PyObject* _cap =
          PyCapsule_New(_own, "pycvc_owner", pycvc_owner_capsule_dtor);
      if (!_cap || PyArray_SetBaseObject(reinterpret_cast<PyArrayObject*>(_arr),
                                         _cap) < 0) {
        delete _own;
        Py_XDECREF(_cap);
        Py_DECREF(_arr);
        SWIG_fail;
      }
    }
  }
  if (!_arr) SWIG_fail;
  $result = _arr;
}

%include "pycvc_geometry.h"
%include "pycvc_volume.h"
