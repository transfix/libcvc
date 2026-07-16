/*
  pycvc.i — top-level SWIG module for the libcvc Python bindings skeleton.

  Minimal first wrap surface (see cvc-engagement-docs modernization docs,
  "libcvc-reuse-for-dbg" §5 and CVC-modernization-plan Phase 13):

    app.i    — hidden cvc::app module context singleton
    volume.i — cvc::dimension / bounding_box / voxels / volume + numpy
               conversion (copy-in / copy-out for the skeleton)
    ops.i    — volume_ops free functions (vol_normalize & friends)
    sdf.i    — minimal cvc::geometry + mesh→SDF (gated on CVC_ENABLE_SDF)

  Deliberately NOT wrapped in v1: GUI/Qt, cvc::state / state_exec /
  distributed state, energy & docking modules, directors/callbacks.
  Do not %include raw libcvc headers here — they are boost/CGAL-heavy and
  SWIG chokes on the template depth.  All fragments are hand-curated.
*/
%module pycvc

%{
#define SWIG_FILE_WITH_INIT

// NumPy C API (compiled against the numpy found at configure time)
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>

#include <cstring>
#include <stdexcept>

#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/types.h>
#include <cvc/geometry/geometry.h>
#include <cvc/utility/algorithm.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_ops.h>
#include <cvc/volume/voxels.h>
%}

%include "std_string.i"
%include "exception.i"

%feature("autodoc", "1");

// Translate C++ exceptions (cvc::exception derives from boost::exception,
// not std::exception, so it needs its own catch clause).
%exception {
  try {
    $action
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "unknown C++ exception");
  }
}

%init %{
  import_array();
%}

%include "app.i"
%include "volume.i"
%include "ops.i"
%include "sdf.i"
