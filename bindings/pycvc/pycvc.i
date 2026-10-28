// pycvc.i — SWIG Python bindings for libcvc (Phase-1 rearchitecture).
//
// DIRECT WRAP: SWIG wraps the REAL cvc value types — cvc::voxels,
// cvc::volume (: public voxels), and cvc::geometry — by VALUE (no
// %shared_ptr). Python `volume` is-a `voxels`, exactly like the C++
// hierarchy; there is no parallel facade layer. The heavy libcvc headers
// are %include'd here (curated with %ignore for the boost-typed / raw /
// templated members that don't marshal), and a thin set of %extend methods
// adds the numpy-friendly convenience surface (builders, lowercase
// dimension accessors, zero-copy views, CUDA adapter) that the tests use.
//
// Every constructor/op takes the cvc::app EXPLICITLY (pycvc.volume(app),
// state_set(app,...), sdf(app,...), observer.watch(app)). There is no
// module-global "current app" and no attach/detach — a host passes the
// shared_ptr<cvc::app> it owns; standalone code uses make_app()/pycvc.App().
// directors="1": enable cross-language polymorphism so Python can subclass
// C++ types and have C++ call the Python overrides. Used (per-class, via
// %feature) only where a real callback interface exists — currently
// pycvc::state_observer (Phase 3 state push callbacks).
%module(directors="1") pycvc

// On Windows, register the extension module's sibling DLL directory
// (<prefix>/bin holds cvc.dll and its whole dependency closure) BEFORE the
// `import _pycvc` in the generated proxy. Python 3.8+ no longer searches PATH
// for an extension module's dependencies, so a self-contained binding must add
// the directory itself — the same pattern torch/vtk follow. pycvc.py installs
// to <prefix>/Lib/site-packages, so <prefix>/bin is ../../bin from here. No-op
// off Windows (POSIX resolves via RPATH / LD_LIBRARY_PATH).
%pythonbegin %{
import os as _os, sys as _sys
if _sys.platform == "win32":
    try:
        _cvc_bin = _os.path.normpath(
            _os.path.join(_os.path.dirname(__file__), "..", "..", "bin"))
        if _os.path.isdir(_cvc_bin):
            _os.add_dll_directory(_cvc_bin)
    except (OSError, AttributeError, NameError):
        pass
%}

%{
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/core/clock_follower.h>
#include <cvc/core/world_clock.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/core/world_units.h>
#include <cvc/volume/dimension.h>
#include <cvc/volume/bounding_box.h>
#include <cvc/volume/voxels.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_file_io.h>
#include <cvc/geometry/geometry.h>
#include <cvc/geometry/geometry_file_io.h>
#include "pycvc_context.h"
#include "pycvc_buffer.h"
#include "pycvc_algorithm.h"
#include "pycvc_state.h"
#include "pycvc_exec.h"
#include <stdexcept>
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
%include <std_shared_ptr.i>
%include <exception.i>

// Surface C++ exceptions (bad array lengths, unreadable files, out-of-range
// voxel access) as Python exceptions instead of aborting the interpreter.
// cvc::exception derives from boost::exception (NOT std::exception), so its
// arm MUST come FIRST — without it a thrown cvc error would slip past a
// std::exception catch and only the catch(...) fallback (or std::terminate)
// would see it. cvc::exception::what() yields the real message.
//
// A macro so a sub-interface that installs its own mapping for a stretch of
// declarations (pycvc_lod.i maps to ValueError/IndexError/OSError and releases
// the GIL) can restore this one afterwards.
%define PYCVC_DEFAULT_EXCEPTION
%exception {
  try {
    $action
  } catch (const cvc::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "pycvc: C++ exception (see libcvc)");
  }
}
%enddef
PYCVC_DEFAULT_EXCEPTION

namespace std {
  %template(DoubleVector) vector<double>;
  %template(IndexVector) vector<unsigned long>;
  %template(StringVector) vector<string>;
}

// boost's fixed-width integer typedefs (uint64_t/int64_t, used by geometry's
// num_*()/index_t and other 64-bit returns) are OPAQUE to SWIG unless mapped:
// without this, a `boost::uint64_t` return is wrapped as a leaked pointer
// ("memory leak of type 'boost::uint64_t *'") instead of marshaling to a
// Python int. Bind them to the native 64-bit integer typemaps (both are 8-byte
// on every target; the value round-trips regardless of the platform's
// long/long-long choice).
%apply unsigned long long { boost::uint64_t };
%apply long long { boost::int64_t };
// Same treatment for the std:: spellings — world_clock's tick / dropped counts
// are std::uint64_t and would otherwise leak as pointers rather than marshal.
%apply unsigned long long { std::uint64_t };
%apply long long { std::int64_t };

// ── The app handle (explicit, no module-global) ─────────────────────
// cvc::app crosses to Python as an OPAQUE std::shared_ptr<cvc::app> and is
// threaded EXPLICITLY into every ctor/op (pycvc.volume(app), state_set(app,…),
// sdf(app,…), observer.watch(app)). There is no attach()/detach() and no
// "current app": make_app() (a.k.a. pycvc.App) mints a standalone one; a host
// passes the shared_ptr it already owns. cvc::app is never dereferenced from
// Python. %shared_ptr uses the std flavor (app manages these handles with std).
%shared_ptr(cvc::app)
// An OPAQUE wrapped class (empty body) — not just a forward declaration — so
// SWIG generates a proxy for cvc::app and %shared_ptr can hang the destructor
// on it (a bare `class app;` leaks the shared_ptr<app>, "no destructor found").
// The real (heavy, non-copyable) cvc::app comes from <cvc/core/app.h> in the
// %{ %} block; this body is only for SWIG's parser. %nodefaultctor: Python
// never constructs an app directly — it comes from make_app()/the host.
%nodefaultctor cvc::app;
namespace cvc { class app {}; }

namespace pycvc {
  std::shared_ptr<cvc::app> make_app();
}

// Adopt a host-owned cvc::app delivered as a PyCapsule named "cvc.app" (which
// holds a std::shared_ptr<cvc::app>*). This is how an EMBEDDING HOST (e.g.
// volrover3) hands pycvc scripts its LIVE app WITHOUT the host needing pycvc.i or
// a SWIG module of its own: the host builds the capsule; pycvc wraps it into
// pycvc's OWN app proxy right here, so the handle is type-compatible with
// state_set / volume / geometry BY CONSTRUCTION — no cross-module SWIG type
// sharing, no SWIG-runtime-version coupling. This is the first entry of pycvc's
// intended cross-extension C-API (a NumPy-style capi header would formalize it).
%inline %{
namespace pycvc {
std::shared_ptr<cvc::app> app_from_capsule(PyObject *cap) {
  if (!cap || !PyCapsule_CheckExact(cap))
    throw std::invalid_argument("pycvc.app_from_capsule: argument is not a PyCapsule");
  void *p = PyCapsule_GetPointer(cap, "cvc.app");
  if (!p)
    throw std::invalid_argument(
        "pycvc.app_from_capsule: capsule is not named \"cvc.app\" or is empty");
  return *static_cast<std::shared_ptr<cvc::app> *>(p);
}
} // namespace pycvc
%}

// Ergonomic alias: pycvc.App() reads as a constructor for a fresh app.
%pythoncode %{
App = make_app


def _keepalive(obj):
    """What an object that borrows `obj` (holds its C++ object by reference) stashes to keep it
    alive: `obj.this`, the SwigPyObject that owns the C++ object, not the proxy itself.

    The proxy is GC-tracked: when it and a borrower are garbage in the same collection (always
    the case at interpreter exit for module globals and anything left in a reference cycle), the
    collector may clear the proxy first, which drops its `this` and frees the C++ object while the
    borrower, whose destructor still uses it, has yet to be destroyed (an access violation in
    ~scene_reader at exit, for an app). A SwigPyObject is not GC-tracked, so only the borrower's
    own reference keeps it alive, and the borrower's `this` (set first) goes before it.

    A proxy that does not own its C++ object (a reference into another one, such as
    Viewport.camera()) is returned as is: its own stash of that owner is what keeps it valid.
    Not for a director class (a node type Python can subclass): there the C++ object calls back
    into its Python half, which `this` alone does not keep alive. Stash the proxy for those.
    """
    this = getattr(obj, "this", None)
    return this if this is not None and this.own() else obj
%}
// Keep the app alive for as long as a volume/geometry built from it lives: the
// C++ voxels/geometry hold the app by RAW reference (app& _ctx), so without this
// the app could be freed while a Python object still points at it. Stash it on
// the proxy (the ctor arg is named `app`) through _keepalive, like every
// _pycvc_app below.
%pythonappend cvc::volume::volume(std::shared_ptr<cvc::app>) %{ self._pycvc_app = _keepalive(app) %}
%pythonappend cvc::geometry::geometry(std::shared_ptr<cvc::app>) %{ self._pycvc_app = _keepalive(app) %}
// state_set/get/has/children/remove + state_observer come from pycvc_state.h
// (%include'd below), all taking the app explicitly.

// ── Zero-copy numpy views ──────────────────────────────────────────
// A wrapped method returning a pycvc::ArrayView becomes a numpy array that
// VIEWS the C++ buffer (no data copy). The array's base is a capsule that
// owns a shared_ptr to the C++ storage, so the memory outlives the wrapped
// object for exactly as long as any view of it does — safe zero-copy.
namespace pycvc { struct ArrayView; }

%typemap(out) pycvc::ArrayView {
  const pycvc::ArrayView& _v = $1;
  int _nd = static_cast<int>(_v.shape.size());
  std::vector<npy_intp> _dims(_v.shape.begin(), _v.shape.end());
  int _npt = (_v.dtype == pycvc::DType::Float64)   ? NPY_DOUBLE
             : (_v.dtype == pycvc::DType::Float32) ? NPY_FLOAT
             : (_v.dtype == pycvc::DType::UInt8)   ? NPY_UINT8
             : (_v.dtype == pycvc::DType::UInt16)  ? NPY_UINT16
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

// volume.h declares an exception via CVC_DEF_EXCEPTION(sub_volume_out_of_bounds)
// at namespace scope. SWIG does not follow the #include of exception.h that
// defines that macro, so without help it mis-parses the invocation as a
// variable. Neutralize the macro for SWIG's PARSER only — the emitted wrapper
// still #include's the real exception.h (in the %{ %} block above) with the
// real definition, so nothing changes at compile time.
#define CVC_DEF_EXCEPTION(name)

// ── Leaf types the class headers reference ──────────────────────────
// The value types below appear only in constructor / operation signatures
// that we %ignore (see below), or are built inside the C++ %extend bodies
// (which compile against the real, complete headers pulled in %{ %} above).
// SWIG therefore only needs their NAMES: keep them opaque to sidestep
// dimension's active anonymous union and bounding_box's template, neither of
// which wraps cleanly and neither of which any exposed method needs as a
// Python type. The data_type enum IS declared so voxelType() marshals as a
// plain int; its values resolve to the real cvc:: symbols at compile time.
namespace cvc {
  typedef unsigned long uint64;
  typedef long int64;
  enum data_type { UChar = 0, UShort, UInt, Float, Double, UInt64, Char, Int, Int64, Undefined };
  class dimension;
  class composite_function;
  template <class T> class generic_bounding_box;
  typedef generic_bounding_box<double> bounding_box;
}

// ── cvc::world_clock — simulation time, decoupled from wall time ────
// The authoritative simulation clock (cvc/core/world_clock.h): time advances in
// whole fixed_dt quanta, so a scenario reproduces identically on a fast machine
// and a slow one and anything with per-second units stays correct no matter how
// often we render. Scripts driving an animation want exactly this instead of
// integrating whatever variable wall-clock delta a host tick handed them.
//
// Two pieces of the C++ surface are reshaped for Python. `step_result` is
// wrapped as itself, so advance() hands back an object with .steps / .alpha /
// .dropped_steps — the names the C++ callers already use. But the nested
// `config` struct and the `mode` enum class are %ignore'd in favour of:
//   * a scalar ctor, world_clock(fixed_dt, scale, max_steps_per_advance);
//   * a string mode — "live" / "replay" / "paused" / "stepping".
// Both keep callers from having to construct a nested proxy type for what is
// really just three numbers and a four-valued choice. Note that a name-wide
// %ignore also suppresses a same-named %extend, which is why the replacements
// below are named differently (mode_name) rather than shadowing the original.
// An invalid quantum/scale still throws from the ctor
// (cvc::unsupported_exception, surfaced by the %exception block above).
// SWIG does not wrap nested structs by default (Warning 325) — advance() would
// then hand back an opaque, leaked step_result. flatnested hoists it to a
// top-level proxy so the three fields marshal properly.
// flatnested hoists it to module scope, where a bare `step_result` would be far
// too generic a name to sit next to volume/geometry — qualify it.
%feature("flatnested") cvc::world_clock::step_result;
%rename(world_clock_step_result) cvc::world_clock::step_result;
%ignore cvc::world_clock::config;
%ignore cvc::world_clock::mode;
%ignore cvc::world_clock::world_clock(cvc::world_clock::config);
%ignore cvc::world_clock::current_mode;
%ignore cvc::world_clock::set_mode;
%ignore cvc::world_clock::configure;
%ignore cvc::world_clock::current_config;
%include "cvc/core/world_clock.h"
%extend cvc::world_clock {
  // Build with an explicit quantum instead of the nested config struct.
  world_clock(double fixed_dt, double scale = 1.0, int max_steps_per_advance = 8,
              int lockstep_steps = 1, bool cap_scales_with_rate = false) {
    cvc::world_clock::config c;
    c.fixed_dt = fixed_dt;
    c.scale = scale;
    c.max_steps_per_advance = max_steps_per_advance;
    c.lockstep_steps = lockstep_steps;
    c.cap_scales_with_rate = cap_scales_with_rate;
    return new cvc::world_clock(c);
  }
  // configure() with scalars: replace the quantum (and the rest) and restart at
  // tick 0, the way a host picks a quantum every simulation divides. Named
  // differently from the %ignore'd configure(config), which would suppress it.
  void set_config(double fixed_dt, double scale = 1.0, int max_steps_per_advance = 8,
                  int lockstep_steps = 1, bool cap_scales_with_rate = false) {
    cvc::world_clock::config c;
    c.fixed_dt = fixed_dt;
    c.scale = scale;
    c.max_steps_per_advance = max_steps_per_advance;
    c.lockstep_steps = lockstep_steps;
    c.cap_scales_with_rate = cap_scales_with_rate;
    $self->configure(c);
  }
  std::string mode_name() const {
    return cvc::world_clock_binding::mode_name($self->current_mode());
  }
  void set_mode_name(const std::string& m) {
    cvc::world_clock::mode mode;
    if (!cvc::world_clock_binding::parse_mode(m, mode))
      throw std::invalid_argument("world_clock.set_mode_name: expected 'live', 'replay', "
                                  "'paused', 'stepping' or 'lockstep'");
    $self->set_mode(mode);
  }
}

// ── Following one clock at several rates (cvc/core/clock_follower.h) ──
// ticks_per_step / steps_due / clock_follower: a Python simulation whose step is
// k quanta follows the shared clock the same way a C++ one does.
%include "cvc/core/clock_follower.h"

// ── cvc::world_clock_binding — the clock on the state tree ──────────
// Speed / pause / mode / single-step come from <prefix>.* keys and the shared
// time is published there (see the header for the key table). Built from the
// app handle and a clock (usually app.world_clock()); keeps the app alive.
// The real ctor takes app& (not the shared_ptr<app> Python holds): ignore it
// and re-expose ONLY the shared_ptr factory below, the volume ctor's pattern (a
// blanket %ignore of the ctor would also suppress the same-named %extend).
// The factory ctors take no default argument: SWIG would emit the defaulted
// form as a second declaration that the blanket %ignore then suppresses, so the
// two arities are spelled out instead.
%ignore cvc::world_clock_binding::world_clock_binding;
%rename("%s") cvc::world_clock_binding::world_clock_binding(std::shared_ptr<cvc::app>,
                                                           cvc::world_clock &);
%rename("%s") cvc::world_clock_binding::world_clock_binding(std::shared_ptr<cvc::app>,
                                                           cvc::world_clock &,
                                                           const std::string &);
%rename("%s") cvc::world_clock_binding::world_clock_binding(std::shared_ptr<cvc::app>,
                                                           cvc::world_clock &,
                                                           const cvc::world_clock_keys &);
%ignore cvc::world_clock_binding::mode_name;
%ignore cvc::world_clock_binding::parse_mode;
// Keep the app and the clock alive while the binding refers to them. With two
// overloads SWIG emits one __init__(self, *args), so the arguments are
// positional here, and the append is attached by name so the merged __init__
// carries it (a signature-specific append is dropped when overloads merge).
%pythonappend cvc::world_clock_binding::world_clock_binding %{
  self._pycvc_app = _keepalive(args[0])
  self._pycvc_clock = args[1]
%}
%include "cvc/core/world_clock_binding.h"
%extend cvc::world_clock_binding {
  world_clock_binding(std::shared_ptr<cvc::app> app, cvc::world_clock &clock) {
    if (!app)
      throw std::invalid_argument("world_clock_binding: app is None");
    return new cvc::world_clock_binding(*app, clock, "sim");
  }
  world_clock_binding(std::shared_ptr<cvc::app> app, cvc::world_clock &clock,
                      const std::string &prefix) {
    if (!app)
      throw std::invalid_argument("world_clock_binding: app is None");
    return new cvc::world_clock_binding(*app, clock, prefix);
  }
  world_clock_binding(std::shared_ptr<cvc::app> app, cvc::world_clock &clock,
                      const cvc::world_clock_keys &keys) {
    if (!app)
      throw std::invalid_argument("world_clock_binding: app is None");
    return new cvc::world_clock_binding(*app, clock, keys);
  }
}

// ── cvc::unit_registry — named units and the conversions between them ──
// The table world_units converts through (cvc/core/unit_registry.h): the
// built-in SI + imperial units, plus whatever an application drops in. Wrapped
// directly; the two nested info structs are flatnested like world_units'. The
// unit_kind enum class is replaced by unit_info.kind_name() ("linear" |
// "affine" | "function"). add_function takes two Python callables
// (float -> float) in place of std::function: each call re-acquires the GIL, so
// a function unit works from any thread that converts through the registry, and
// an exception raised by the callable surfaces as a RuntimeError naming it.
%{
#include <cvc/core/unit_registry.h>

// A Python callable as a unit_registry::conversion. Holds a strong reference,
// taken and dropped under the GIL; after interpreter shutdown the reference is
// leaked rather than touched.
struct pycvc_py_conversion {
  PyObject *fn;
  explicit pycvc_py_conversion(PyObject *f) : fn(f) {
    PyGILState_STATE g = PyGILState_Ensure();
    Py_XINCREF(fn);
    PyGILState_Release(g);
  }
  pycvc_py_conversion(const pycvc_py_conversion &o) : pycvc_py_conversion(o.fn) {}
  pycvc_py_conversion &operator=(const pycvc_py_conversion &) = delete;
  ~pycvc_py_conversion() {
    if (!Py_IsInitialized())
      return;
    PyGILState_STATE g = PyGILState_Ensure();
    Py_XDECREF(fn);
    PyGILState_Release(g);
  }
  double operator()(double v) const {
    PyGILState_STATE g = PyGILState_Ensure();
    PyObject *r = PyObject_CallFunction(fn, "d", v);
    double out = 0.0;
    std::string err;
    if (r) {
      out = PyFloat_AsDouble(r);
      Py_DECREF(r);
    }
    if (PyErr_Occurred()) {
      PyObject *type = nullptr, *value = nullptr, *tb = nullptr;
      PyErr_Fetch(&type, &value, &tb);
      PyObject *s = value ? PyObject_Str(value) : nullptr;
      const char *msg = s ? PyUnicode_AsUTF8(s) : nullptr;
      err = std::string("unit_registry: a Python conversion raised ") +
            (type ? reinterpret_cast<PyTypeObject *>(type)->tp_name : "an error") + ": " +
            (msg ? msg : "");
      Py_XDECREF(s);
      Py_XDECREF(type);
      Py_XDECREF(value);
      Py_XDECREF(tb);
      PyErr_Clear();
    }
    PyGILState_Release(g);
    if (!err.empty())
      throw std::runtime_error(err);
    return out;
  }
};
%}
%feature("flatnested") cvc::unit_registry::dimension_info;
%feature("flatnested") cvc::unit_registry::unit_info;
%rename(unit_registry_dimension_info) cvc::unit_registry::dimension_info;
%rename(unit_registry_unit_info) cvc::unit_registry::unit_info;
%ignore cvc::unit_registry::unit_kind;
%ignore cvc::unit_registry::unit_info::kind;
%ignore cvc::unit_registry::conversion;
%ignore cvc::unit_registry::operator=;
%ignore cvc::unit_registry::add_function;
%rename(add_function) cvc::unit_registry::pycvc_add_function;
%include "cvc/core/unit_registry.h"
%extend cvc::unit_registry::unit_info {
  std::string kind_name() const {
    switch ($self->kind) {
    case cvc::unit_registry::unit_kind::affine:
      return "affine";
    case cvc::unit_registry::unit_kind::function:
      return "function";
    default:
      return "linear";
    }
  }
}
%extend cvc::unit_registry {
  // si = to_si(value), value = from_si(si), both Python callables.
  void pycvc_add_function(const std::string &symbol, const std::string &dimension,
                          PyObject *to_si, PyObject *from_si,
                          const std::string &name = std::string()) {
    if (!PyCallable_Check(to_si) || !PyCallable_Check(from_si))
      throw std::invalid_argument("unit_registry.add_function: to_si and from_si must be callable");
    $self->add_function(symbol, dimension, pycvc_py_conversion(to_si),
                        pycvc_py_conversion(from_si), name);
  }
}

// ── cvc::world_units — the SI unit base + display regime ────────────
// The spatial counterpart to world_clock (cvc/core/world_units.h): a canonical
// SI store with a display regime (SI/imperial) and metres_per_world_unit pinning
// world-space to metres. Reshaped for Python exactly like world_clock: the
// nested value structs (measurement{value,unit}, coordinate{x,y,z,unit},
// normalized_frame{scale,cx,cy,cz}, xyz{x,y,z}) are flatnested to top-level
// proxies so their fields marshal (SWIG Warning 325 otherwise leaks them
// opaque); the two enum classes (system, dimension) and the nested config are
// %ignore'd in favour of a scalar ctor and string-keyed helpers. As with
// world_clock, a name-wide %ignore also suppresses a same-named %extend, so the
// dimension-keyed replacements take a `_d` suffix and the regime ones a `_name`
// suffix rather than shadowing the originals. A dimension is its registry name
// ("length", "velocity", "energy_per_length", or one the application added), so
// the `_d` helpers reach custom dimensions too. The plain-double and
// unit-symbol methods (metres_per_world_unit, world_to_metres, to_si, convert,
// world_point_to_real, si_point_to_normalized, ...) wrap as-is; registry()
// returns the instance's own unit_registry and keeps the world_units alive.
%feature("flatnested") cvc::world_units::measurement;
%feature("flatnested") cvc::world_units::coordinate;
%feature("flatnested") cvc::world_units::normalized_frame;
%feature("flatnested") cvc::world_units::xyz;
%rename(world_units_measurement) cvc::world_units::measurement;
%rename(world_units_coordinate) cvc::world_units::coordinate;
%rename(world_units_normalized_frame) cvc::world_units::normalized_frame;
%rename(world_units_xyz) cvc::world_units::xyz;
%ignore cvc::world_units::system;
%ignore cvc::world_units::dimension;
%ignore cvc::world_units::config;
%ignore cvc::world_units::world_units(cvc::world_units::config);
%ignore cvc::world_units::dimension_name;
%ignore cvc::world_units::regime;
%ignore cvc::world_units::set_regime;
%ignore cvc::world_units::to_display;
%ignore cvc::world_units::from_display;
%ignore cvc::world_units::unit_symbol;
%ignore cvc::world_units::format;
%ignore cvc::world_units::set_display_unit;
%ignore cvc::world_units::clear_display_unit;
%ignore cvc::world_units::display_unit;
%ignore cvc::world_units::to_normalized;
%ignore cvc::world_units::from_normalized;
%ignore cvc::world_units::registry() const;
%pythonappend cvc::world_units::registry %{
  val._pycvc_owner = self
%}
%include "cvc/core/world_units.h"
%extend cvc::world_units {
  // Build with an explicit scale + string regime instead of the nested config.
  world_units(double metres_per_world_unit, const std::string &regime = "si") {
    cvc::world_units::config c;
    c.metres_per_world_unit = metres_per_world_unit;
    c.regime = (regime == "imperial") ? cvc::world_units::system::imperial
                                       : cvc::world_units::system::si;
    return new cvc::world_units(c);
  }
  std::string regime_name() const {
    return $self->regime() == cvc::world_units::system::imperial ? "imperial" : "si";
  }
  void set_regime_name(const std::string &s) {
    if (s == "si") $self->set_regime(cvc::world_units::system::si);
    else if (s == "imperial") $self->set_regime(cvc::world_units::system::imperial);
    else throw std::invalid_argument("world_units.set_regime_name: expected 'si' or 'imperial'");
  }
  double to_display_d(double v, const std::string &dim) const {
    return $self->to_display(v, dim);
  }
  double from_display_d(double v, const std::string &dim) const {
    return $self->from_display(v, dim);
  }
  std::string unit_symbol_d(const std::string &dim) const {
    return $self->unit_symbol(dim);
  }
  cvc::world_units::measurement format_d(double v, const std::string &dim) const {
    return $self->format(v, dim);
  }
  void set_display_unit_d(const std::string &dim, const std::string &symbol) {
    $self->set_display_unit(dim, symbol);
  }
  void clear_display_unit_d(const std::string &dim) {
    $self->clear_display_unit(dim);
  }
  std::string display_unit_d(const std::string &dim) const {
    return $self->display_unit(dim);
  }
  double to_normalized_d(double v, const std::string &dim,
                         const cvc::world_units::normalized_frame &f) const {
    return $self->to_normalized(v, dim, f);
  }
  double from_normalized_d(double v, const std::string &dim,
                           const cvc::world_units::normalized_frame &f) const {
    return $self->from_normalized(v, dim, f);
  }
}

// ── cvc::app: per-app world_clock() / world_units() accessors ────────
// app is the opaque proxy above; now that both bases are wrapped, expose the
// per-app handles (owned by the app; SWIG returns non-owning proxies). This is
// how Python reaches the application-wide clock/units without a singleton.
%extend cvc::app {
  cvc::world_clock &world_clock() { return $self->world_clock(); }
  cvc::world_units &world_units() { return $self->world_units(); }
}

// ── cvc::voxels — curate the surface ────────────────────────────────
// Ignore: all app&-taking ctors (Python builds via the volume factory
// %extend), the app& accessor, boost-typed / raw / templated members
// (histogram -> boost::tuple, data_as_shared_array -> boost::shared_array,
// data_ptr / active_storage -> raw pointer / shared_ptr<void>, get_gpu_info
// -> vector<gpu_device_info>), the dimension-typed min/max/voxel_dimensions
// overloads, and the Phase-2 in-place algorithms. operator()/operator*/
// operator=/== are auto-skipped by SWIG. (Ignored members stay fully
// callable from the C++ %extend bodies below — %ignore only trims the
// Python surface.)
%ignore cvc::voxels::voxels;
%ignore cvc::voxels::ctx;
%ignore cvc::voxels::voxel_dimensions;
%ignore cvc::voxels::min;
%ignore cvc::voxels::max;
%ignore cvc::voxels::histogram;
%ignore cvc::voxels::copy;
%ignore cvc::voxels::sub;
%ignore cvc::voxels::fill;
%ignore cvc::voxels::fillsub;
%ignore cvc::voxels::map;
%ignore cvc::voxels::resize;
%ignore cvc::voxels::bilateralFilter;
%ignore cvc::voxels::composite;
%ignore cvc::voxels::contrastEnhancement;
%ignore cvc::voxels::anisotropicDiffusion;
%ignore cvc::voxels::gdtvFilter;
%ignore cvc::voxels::get_gpu_info;
%ignore cvc::voxels::data_ptr;
%ignore cvc::voxels::data_as_shared_array;
%ignore cvc::voxels::active_storage;
// Raw CUDA controls are replaced by the volume %extend adapter (enable_cuda/
// disable_cuda/cuda_available/on_gpu/cuda_ptr). using_cuda() is kept (it is
// inert & returns false on this host-only build).
%ignore cvc::voxels::enableCUDA;
%ignore cvc::voxels::disableCUDA;
%ignore cvc::voxels::switchGPU;
// cuda_available() is kept exposed: it's a clean `static bool` defined
// unconditionally (returns false on a CUDA-off build), so volume inherits it
// and the CUDA tests self-skip. (A %extend of the same name would be swallowed
// by an %ignore here — same collision as the ctors.)
%ignore cvc::voxels::cuda_device_count;
%ignore cvc::voxels::cuda_device;
%ignore cvc::voxels::get_current_gpu;
%ignore cvc::voxels::set_current_gpu;

%include "cvc/volume/voxels.h"

// ── cvc::volume — curate the surface ────────────────────────────────
// Ignore the app&/dimension/bounding_box ctors and ops; the kept members are
// the double-valued spatial accessors (XMin..ZMax, XSpan..ZSpan), desc(),
// interpolate(). read/write are kept only for the C++ %extend load/save.
%ignore cvc::volume::volume;
// ...but KEEP our %extend factory ctor volume(app) below. A blanket
// `%ignore Class::Class` also suppresses same-named %extend ctors; the real
// volume ctors take app& (not shared_ptr<app>), so re-exposing the
// shared_ptr<app> signature un-ignores ONLY our factory.
%rename("%s") cvc::volume::volume(std::shared_ptr<cvc::app>);
%ignore cvc::volume::copy;
%ignore cvc::volume::sub;
%ignore cvc::volume::resize;
%ignore cvc::volume::combineWith;
%ignore cvc::volume::read;
%ignore cvc::volume::write;
%ignore cvc::volume::boundingBox;

%include "cvc/volume/volume.h"

// ── cvc::geometry — curate the surface ──────────────────────────────
// Ignore: all ctors (Python builds via the factory %extend), the boost::array
// container accessors (points/tris/colors/... and their const_ forms) and the
// boost::shared_ptr owners (points_ptr/colors_ptr) — all reached instead from
// the C++ %extend builders/views — plus the by-value boost::array returns
// (min_point/max_point/extents), the num_* accessors (re-added as
// num_vertices/num_triangles/num_lines), and the Phase-2 algorithms.
// Hide ALL real geometry ctors (the app-less default sets _ctx=nullptr; we
// require an explicit app for consistency with volume). Our %extend factory
// geometry(app) below is re-exposed by its shared_ptr<app> signature — the real
// ctors take app&/string/geometry&, none take shared_ptr<app>, so no collision.
%ignore cvc::geometry::geometry;
%rename("%s") cvc::geometry::geometry(std::shared_ptr<cvc::app>);
%ignore cvc::geometry::copy;
%ignore cvc::geometry::ctx;
%ignore cvc::geometry::points;
%ignore cvc::geometry::boundary;
%ignore cvc::geometry::normals;
%ignore cvc::geometry::colors;
%ignore cvc::geometry::curvatures;
%ignore cvc::geometry::functions;
%ignore cvc::geometry::lines;
%ignore cvc::geometry::tris;
%ignore cvc::geometry::quads;
%ignore cvc::geometry::tets;
%ignore cvc::geometry::hexs;
%ignore cvc::geometry::const_points;
%ignore cvc::geometry::const_boundary;
%ignore cvc::geometry::const_normals;
%ignore cvc::geometry::const_colors;
%ignore cvc::geometry::const_curvatures;
%ignore cvc::geometry::const_functions;
%ignore cvc::geometry::const_lines;
%ignore cvc::geometry::const_tris;
%ignore cvc::geometry::const_quads;
%ignore cvc::geometry::const_tets;
%ignore cvc::geometry::const_hexs;
%ignore cvc::geometry::points_ptr;
%ignore cvc::geometry::colors_ptr;
// Phase-2 UVs / tangents: hide the raw boost::array-vector accessors + shared
// owners (like points/colors above). The Python surface is the flat set_uvs/
// set_tangents builders + the zero-copy uvs()/tangents() views in the %extend
// below. Those views are DECLARED as uvs_view()/tangents_view() (distinct names,
// so the %ignore here does not swallow them) and %rename'd back to the natural
// uvs()/tangents() — a same-named %extend would be eaten by the %ignore.
%ignore cvc::geometry::uvs;
%ignore cvc::geometry::const_uvs;
%ignore cvc::geometry::tangents;
%ignore cvc::geometry::const_tangents;
%ignore cvc::geometry::uvs_ptr;
%ignore cvc::geometry::tangents_ptr;
%rename(uvs) cvc::geometry::uvs_view;
%rename(tangents) cvc::geometry::tangents_view;
%ignore cvc::geometry::min_point;
%ignore cvc::geometry::max_point;
%ignore cvc::geometry::extents;
%ignore cvc::geometry::num_points;
// num_lines() is kept exposed (real `uint64_t num_lines() const`): the tests
// use it by that exact name, and a same-named %extend would be swallowed by an
// %ignore. num_vertices()/num_triangles() are %extend aliases (their real
// counterparts num_points()/num_tris() have DIFFERENT names, so no collision).
%ignore cvc::geometry::num_tris;
%ignore cvc::geometry::num_quads;
%ignore cvc::geometry::num_tets;
%ignore cvc::geometry::num_hexs;
// merge / tri_surface / invert_normals / reorient are now WRAPPED (removed from
// the ignore list): merge(const geometry&) and invert_normals()/reorient() are
// self-returning geometry& methods (same shape as the already-bound
// compute_normals(), so they marshal identically — the return proxy aliases
// self); tri_surface() returns a new geometry by value.
%ignore cvc::geometry::calculate_surf_normals;
%ignore cvc::geometry::generate_wire_interior;
%ignore cvc::geometry::project;
%ignore cvc::geometry::smoothing;
%ignore cvc::geometry::quality_improve;
// ...but re-expose our Pythonic %extend overloads (Phase-2 filters), whose
// signatures differ from the real app&/enum ones, so only these reach Python.
%rename("%s") cvc::geometry::smoothing(std::shared_ptr<cvc::app>, double, bool, bool);
%rename("%s") cvc::geometry::quality_improve(int, int);
%ignore cvc::geometry::read;
%ignore cvc::geometry::write;

%include "cvc/geometry/geometry.h"

// ── voxels: numpy-friendly scalar accessors ─────────────────────────
%extend cvc::voxels {
  // Voxel value at grid index (i, j, k) — the read half of operator().
  double value(unsigned long i, unsigned long j, unsigned long k) const {
    return (*$self)(i, j, k);
  }
  unsigned long xdim() const { return $self->XDim(); }
  unsigned long ydim() const { return $self->YDim(); }
  unsigned long zdim() const { return $self->ZDim(); }
  double min_value() const { return $self->min(); }
  double max_value() const { return $self->max(); }
}

// ── volume: factory ctor, builders, views, CUDA adapter ─────────────
%extend cvc::volume {
  // Default-construct against the module app (Phase 0 injected context).
  volume(std::shared_ptr<cvc::app> app) {
    if (!app)
      throw std::invalid_argument("pycvc.volume: null app handle");
    return new cvc::volume(*app);
  }

  // ── In-library data-prep (the canonical ML/training ops) ────────────────
  // The underlying voxels::map/fill/sub/resize are %ignore'd on the base scope;
  // these re-expose them on the volume proxy under DISTINCT names (so the base
  // %ignore cannot swallow a same-named %extend), turning "prep a volume before
  // handing it to a network" into one call each.
  //
  // normalize(lo, hi): affine-remap every voxel so the data range maps to
  // [lo, hi] (e.g. [0,1] or [-1,1] network input).
  void normalize(double lo, double hi) { $self->map(lo, hi); }
  // fill_value(val): set every voxel to a constant (clear before stamping
  // obstacles). Named fill_value, not fill — the base %ignore cvc::voxels::fill
  // DOES reach this derived %extend scope and would swallow a same-named helper.
  void fill_value(double val) { $self->fill(val); }
  // resample(nx, ny, nz): resize this volume IN PLACE to an nx*ny*nz grid (the
  // uniform network-input grid). Builds the opaque `dimension` from ints, as
  // set_float_grid does.
  void resample(unsigned long nx, unsigned long ny, unsigned long nz) {
    $self->resize(cvc::dimension(nx, ny, nz));
  }
  // crop(ox,oy,oz, nx,ny,nz): a NEW volume holding the nx*ny*nz sub-grid at
  // voxel offset (ox,oy,oz) — copy-then-sub, so the source is left intact
  // (patch augmentation). sub() extracts a subvolume in place, so it runs on the
  // copy. The 4-arg overload is called; volume::sub's MSVC-only `brain_damage`
  // trailing arg keeps its default and is never exposed.
  cvc::volume crop(unsigned long ox, unsigned long oy, unsigned long oz, unsigned long nx,
                   unsigned long ny, unsigned long nz) const {
    cvc::volume out(*$self);
    out.sub(ox, oy, oz, cvc::dimension(nx, ny, nz));
    return out;
  }

  // Build a Float volume from a flat, row-major (x fastest, then y, then z)
  // scalar grid of nx*ny*nz values, over the object-space box
  // [minx,maxx] x [miny,maxy] x [minz,maxz].
  void set_float_grid(const std::vector<double>& values, unsigned long nx, unsigned long ny,
                      unsigned long nz, double minx, double miny, double minz, double maxx,
                      double maxy, double maxz) {
    const std::size_t n =
        static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny) * static_cast<std::size_t>(nz);
    if (nx == 0 || ny == 0 || nz == 0)
      throw std::invalid_argument("set_float_grid: dimensions must be positive");
    if (values.size() != n)
      throw std::invalid_argument("set_float_grid: len(values) must equal nx*ny*nz");
    std::vector<float> fbuf(values.begin(), values.end());
    cvc::dimension dim(nx, ny, nz);
    cvc::bounding_box box(minx, miny, minz, maxx, maxy, maxz);
    // Rebuild in place using THIS volume's own app (set when it was
    // constructed via pycvc.volume(app) — voxels stores it as _ctx).
    *$self = cvc::volume($self->ctx(), reinterpret_cast<const unsigned char*>(fbuf.data()), dim,
                         cvc::Float, box);
  }

  double xmin() const { return $self->XMin(); }
  double xmax() const { return $self->XMax(); }
  double ymin() const { return $self->YMin(); }
  double ymax() const { return $self->YMax(); }
  double zmin() const { return $self->ZMin(); }
  double zmax() const { return $self->ZMax(); }

  // File I/O (.rawiv and friends). write() createVolumeFile()s the target
  // first, so a fresh path succeeds (unlike the free writeVolumeFile overload
  // that only overwrites an existing file).
  void load(const std::string& filename) { $self->read(filename); }
  void save(const std::string& filename) { $self->write(filename); }

  // Zero-copy numpy view of the voxel grid. Shape (nz, ny, nx), float32,
  // writable. The owner pins the EXACT block data_ptr() aliases (via
  // active_storage()) — NOT the whole volume — so after an enable_cuda()/
  // disable_cuda() migration or a set_float_grid() rebuild (each of which
  // reallocates and frees the old block) the numpy view remains a valid,
  // decoupled STALE SNAPSHOT rather than a use-after-free.
  pycvc::ArrayView grid() {
    if ($self->voxelType() != cvc::Float)
      throw std::invalid_argument("grid(): zero-copy view requires a Float volume");
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::Float32;
    v.writable = true;
    v.shape = {static_cast<long>($self->ZDim()), static_cast<long>($self->YDim()),
               static_cast<long>($self->XDim())};
    v.data = $self->data_ptr();
    v.owner = $self->active_storage();
    return v;
  }

  // GPU/CUDA support. All host-safe: on a CUDA-disabled libcvc build (this
  // build) on_gpu() is False, cuda_ptr() is 0, and enable_cuda() raises. The
  // CUDA tests self-skip on !cuda_available().
  bool on_gpu() const {
#ifdef CVC_USING_CUDA
    return $self->cuda_data_ptr() != nullptr;
#else
    return false;
#endif
  }
  unsigned long long cuda_ptr() const {
#ifdef CVC_USING_CUDA
    return reinterpret_cast<unsigned long long>($self->cuda_data_ptr());
#else
    return 0;
#endif
  }
  // (cuda_available() is the real inherited voxels static — see %ignore notes.)
  void enable_cuda(int device = -1) {
#ifdef CVC_USING_CUDA
    $self->enableCUDA(device);
#else
    (void)device;
    throw std::runtime_error("enable_cuda: this libcvc build has CUDA disabled");
#endif
  }
  void disable_cuda() {
#ifdef CVC_USING_CUDA
    $self->disableCUDA();
#endif
  }

  // ── Denoise / enhance filters (in-place cvc::voxels ops; the real
  // camelCase methods are %ignore'd — these lower_snake wrappers have
  // distinct names so no ignore-collision; each auto-dispatches to the CUDA
  // kernel when the volume is GPU-resident). ──
  void bilateral_filter(double radiometric_sigma = 200.0, double spatial_sigma = 1.5,
                        unsigned int filter_radius = 2) {
    $self->bilateralFilter(radiometric_sigma, spatial_sigma, filter_radius);
  }
  void contrast_enhancement(double resistor = 0.95) { $self->contrastEnhancement(resistor); }
  void anisotropic_diffusion(unsigned int iterations = 20) {
    $self->anisotropicDiffusion(iterations);
  }
  void gdtv_filter(double q, double lambda, unsigned int iterations, unsigned int neighbours = 0) {
    $self->gdtvFilter(q, lambda, iterations, neighbours);
  }

// ── GPU adapter: expose a GPU-resident volume to cupy/torch/numba ───
// When the voxels live in CUDA unified memory (on_gpu()), the same buffer
// grid() views on the host is also device-accessible. __cuda_array_interface__
// (CAI v3) lets GPU array libraries wrap it zero-copy on the device — so a
// single unified allocation serves numpy (host) AND cupy (device) with no
// copies. Raises AttributeError on host-only / CUDA-disabled builds, which is
// the correct signal for those libraries.
%pythoncode %{
    @property
    def __cuda_array_interface__(self):
        if not self.on_gpu():
            raise AttributeError(
                "volume is not GPU-resident (CUDA-disabled build or host data); "
                "use grid() for a host numpy view")
        nz, ny, nx = self.zdim(), self.ydim(), self.xdim()
        return {
            "shape": (nz, ny, nx),
            "typestr": "<f4",
            "data": (self.cuda_ptr(), False),  # (ptr, read_only=False)
            "version": 3,
            "strides": None,                    # C-contiguous
        }
%}
}

// ── geometry: factory ctor, builders, zero-copy views, I/O ──────────
%extend cvc::geometry {
  // Construct against an explicit app (the real explicit geometry(app&) ctor).
  geometry(std::shared_ptr<cvc::app> app) {
    if (!app)
      throw std::invalid_argument("pycvc.geometry: null app handle");
    return new cvc::geometry(*app);
  }

  // Incremental builders. Indices use size_t (== uint64 on LP64).
  std::size_t add_vertex(double x, double y, double z) {
    cvc::geometry::point_t p;
    p[0] = x;
    p[1] = y;
    p[2] = z;
    $self->points().push_back(p);
    return $self->points().size() - 1;
  }
  void add_triangle(std::size_t a, std::size_t b, std::size_t c) {
    cvc::geometry::tri_t t;
    t[0] = a;
    t[1] = b;
    t[2] = c;
    $self->tris().push_back(t);
  }
  void add_line(std::size_t a, std::size_t b) {
    cvc::geometry::line_t l;
    l[0] = a;
    l[1] = b;
    $self->lines().push_back(l);
  }

  // Bulk builders (fast path; flat row-major arrays).
  void add_vertices(const std::vector<double>& xyz) {
    if (xyz.size() % 3 != 0)
      throw std::invalid_argument("add_vertices: length must be a multiple of 3");
    auto& pts = $self->points();
    pts.reserve(pts.size() + xyz.size() / 3);
    for (std::size_t i = 0; i + 2 < xyz.size(); i += 3) {
      cvc::geometry::point_t p;
      p[0] = xyz[i];
      p[1] = xyz[i + 1];
      p[2] = xyz[i + 2];
      pts.push_back(p);
    }
  }
  void add_triangles(const std::vector<unsigned long>& ijk) {
    if (ijk.size() % 3 != 0)
      throw std::invalid_argument("add_triangles: length must be a multiple of 3");
    auto& tris = $self->tris();
    tris.reserve(tris.size() + ijk.size() / 3);
    for (std::size_t i = 0; i + 2 < ijk.size(); i += 3) {
      cvc::geometry::tri_t t;
      t[0] = ijk[i];
      t[1] = ijk[i + 1];
      t[2] = ijk[i + 2];
      tris.push_back(t);
    }
  }
  void add_lines(const std::vector<unsigned long>& ab) {
    if (ab.size() % 2 != 0)
      throw std::invalid_argument("add_lines: length must be a multiple of 2");
    auto& lines = $self->lines();
    lines.reserve(lines.size() + ab.size() / 2);
    for (std::size_t i = 0; i + 1 < ab.size(); i += 2) {
      cvc::geometry::line_t l;
      l[0] = ab[i];
      l[1] = ab[i + 1];
      lines.push_back(l);
    }
  }
  void set_colors(const std::vector<double>& rgb) {
    if (rgb.size() != $self->const_points().size() * 3)
      throw std::invalid_argument("set_colors: length must equal 3 * num_vertices()");
    auto& colors = $self->colors();
    colors.clear();
    colors.reserve(rgb.size() / 3);
    for (std::size_t i = 0; i + 2 < rgb.size(); i += 3) {
      cvc::geometry::color_t c;
      c[0] = rgb[i];
      c[1] = rgb[i + 1];
      c[2] = rgb[i + 2];
      colors.push_back(c);
    }
  }
  // Per-vertex normals as a flat [nx,ny,nz,...] list (len == 3*num_vertices()).
  // Read-out completes the compute_normals()/calculate_surf_normals() story
  // (there was no way to get the result back to Python); set_normals stamps
  // authored normals in. Copy-based: geometry exposes no normals_ptr(), so the
  // zero-copy view that vertices() uses cannot be replicated here.
  std::vector<double> get_normals() const {
    const auto& norms = $self->const_normals();
    std::vector<double> out;
    out.reserve(norms.size() * 3);
    for (const auto& n : norms) {
      out.push_back(n[0]);
      out.push_back(n[1]);
      out.push_back(n[2]);
    }
    return out;
  }
  void set_normals(const std::vector<double>& xyz) {
    if (xyz.size() != $self->const_points().size() * 3)
      throw std::invalid_argument("set_normals: length must equal 3 * num_vertices()");
    auto& norms = $self->normals();
    norms.clear();
    norms.reserve(xyz.size() / 3);
    for (std::size_t i = 0; i + 2 < xyz.size(); i += 3) {
      cvc::geometry::vector_t n;
      n[0] = xyz[i];
      n[1] = xyz[i + 1];
      n[2] = xyz[i + 2];
      norms.push_back(n);
    }
  }
  // The mesh AABB as a (minx,miny,minz,maxx,maxy,maxz) 6-tuple (the bounding_box
  // return is opaque — same treatment as model.extents). Aliased to `extents`
  // in the %pythoncode below.
  std::vector<double> extents_bbox() const {
    cvc::bounding_box b = $self->extents();
    return {b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz};
  }
  %pythoncode %{
    extents = extents_bbox  # geom.extents() -> (minx..maxz), like model.extents()
  %}

  // Per-vertex texture coordinates (Phase 2). Flat row-major u,v pairs; one uv
  // per vertex (len == 2 * num_vertices()). Mirrors set_colors. These reach the
  // cvcGL SetTCoords slot so a textured mesh samples node.set_texture().
  void set_uvs(const std::vector<double>& uv) {
    if (uv.size() != $self->const_points().size() * 2)
      throw std::invalid_argument("set_uvs: length must equal 2 * num_vertices()");
    auto& uvs = $self->uvs();
    uvs.clear();
    uvs.reserve(uv.size() / 2);
    for (std::size_t i = 0; i + 1 < uv.size(); i += 2) {
      cvc::geometry::uv_t t;
      t[0] = uv[i];
      t[1] = uv[i + 1];
      uvs.push_back(t);
    }
  }
  // Per-vertex tangent basis (Phase 2). Flat x,y,z,w quads (w = handedness ±1);
  // one tangent per vertex (len == 4 * num_vertices()).
  void set_tangents(const std::vector<double>& t) {
    if (t.size() != $self->const_points().size() * 4)
      throw std::invalid_argument("set_tangents: length must equal 4 * num_vertices()");
    auto& tangents = $self->tangents();
    tangents.clear();
    tangents.reserve(t.size() / 4);
    for (std::size_t i = 0; i + 3 < t.size(); i += 4) {
      cvc::geometry::tangent_t tg;
      tg[0] = t[i];
      tg[1] = t[i + 1];
      tg[2] = t[i + 2];
      tg[3] = t[i + 3];
      tangents.push_back(tg);
    }
  }

  std::size_t num_vertices() const { return $self->const_points().size(); }
  std::size_t num_triangles() const { return $self->const_tris().size(); }
  // num_lines(): use the real inherited accessor (kept un-ignored above).

  // Zero-copy numpy views. Each pins the SPECIFIC container's boost::shared_ptr
  // (points_ptr()/colors_ptr(), non-detaching) rather than the whole geometry:
  // cvc::geometry copy-on-writes each container, so a later append / set_colors
  // / clear COW-detaches to a fresh block and RETIRES this one to the view — a
  // valid, decoupled snapshot, never a dangling read. The boost::shared_ptr is
  // bridged into ArrayView.owner (a std::shared_ptr<void>) by a keep-alive
  // deleter that captures a copy of the boost pointer, so the block outlives
  // the view across COW forks. Shape (N,3) float64, writable.
  pycvc::ArrayView vertices() {
    cvc::geometry::points_ptr_t container = $self->points_ptr();
    auto& vec = *container;
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::Float64;
    v.writable = true;
    v.shape = {static_cast<long>(vec.size()), 3};
    v.data = vec.empty() ? nullptr : &vec[0][0];
    v.owner = std::shared_ptr<void>(container.get(), [container](void*) { /* keep-alive */ });
    return v;
  }
  pycvc::ArrayView vertex_colors() {
    cvc::geometry::colors_ptr_t container = $self->colors_ptr();
    auto& vec = *container;
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::Float64;
    v.writable = true;
    v.shape = {static_cast<long>(vec.size()), 3};
    v.data = vec.empty() ? nullptr : &vec[0][0];
    v.owner = std::shared_ptr<void>(container.get(), [container](void*) { /* keep-alive */ });
    return v;
  }
  // Zero-copy (N,2) float64 view of the UVs, pinned via uvs_ptr() (same COW-safe
  // keep-alive as vertices()/vertex_colors()). %rename'd to uvs(). Writable — a
  // numpy edit writes the geometry's uv container in place.
  pycvc::ArrayView uvs_view() {
    cvc::geometry::uvs_ptr_t container = $self->uvs_ptr();
    auto& vec = *container;
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::Float64;
    v.writable = true;
    v.shape = {static_cast<long>(vec.size()), 2};
    v.data = vec.empty() ? nullptr : &vec[0][0];
    v.owner = std::shared_ptr<void>(container.get(), [container](void*) { /* keep-alive */ });
    return v;
  }
  // Zero-copy (N,4) float64 view of the tangent basis, pinned via tangents_ptr().
  // %rename'd to tangents().
  pycvc::ArrayView tangents_view() {
    cvc::geometry::tangents_ptr_t container = $self->tangents_ptr();
    auto& vec = *container;
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::Float64;
    v.writable = true;
    v.shape = {static_cast<long>(vec.size()), 4};
    v.data = vec.empty() ? nullptr : &vec[0][0];
    v.owner = std::shared_ptr<void>(container.get(), [container](void*) { /* keep-alive */ });
    return v;
  }

  // ── Mesh filters. smoothing() takes the injected app explicitly;
  // quality_improve() is mesher-gated (LBIE) — raises when the build has the
  // mesher off. The real same-named methods are %ignore'd and these %extend
  // overloads are re-exposed by signature (see the %rename notes above). ──
  void smoothing(std::shared_ptr<cvc::app> app, double delta = 0.1, bool fix_boundary = false,
                 bool geometric_flow = true) {
    if (!app)
      throw std::invalid_argument("pycvc geometry.smoothing: null app handle");
    $self->smoothing(*app, static_cast<float>(delta), fix_boundary, /*perturb_1=*/false,
                     geometric_flow, /*smoothing_enabled=*/true, /*perturb_2=*/false);
  }
  void quality_improve(int iterations = 1, int method = 1) {
#ifdef CVC_ENABLE_MESHER
    $self->quality_improve(iterations, static_cast<cvc::improvement_method>(method));
#else
    (void)iterations;
    (void)method;
    throw std::runtime_error("quality_improve: this libcvc build has the mesher disabled");
#endif
  }

  // File I/O (.off/.raw/.rawc/…) via cvc::geometry_file_io free functions.
  void save(const std::string& filename) const { cvc::write_geometry(*$self, filename); }
  void load(const std::string& filename) { *$self = cvc::read_geometry(filename); }
}

// ── Phase 1: cvc::image value type + zero-copy numpy() view ──────────────
// Wraps the VTK-free raster image (load/save, resize/convert/flip) and a
// zero-copy (H,W,C) numpy view over its buffer. Kept in a sibling interface
// file %include'd here so `image` lands in the `pycvc` module (pycvc.image.*)
// and cvcGL's node.set_texture(image) can %import it. Uses the ArrayView
// machinery + capsule dtor defined above.
%include "pycvc_image.i"

// ── cvc::ariadne::stream: real-time frame transport (video) ───────────────────────
// Phase-2 streaming bindings. %included AFTER pycvc_image.i so it reuses the
// ArrayView out-typemap + the pycvc_owner capsule dtor for StreamFrame.numpy().
%include "pycvc_stream.i"

// ── cvc::volren: software raycaster value types + headless raycaster ─────
// The volren settings structs + raycaster (renders a cvc::volume to a cvc::image,
// no VTK). Placed here, after volume + image + bounding_box + world_units are all
// wrapped above, because volume_settings/frame/raycaster reference them.
%include "pycvc_volren.i"

// ── cvc::volslice: view-aligned slice renderer value types ──────────────
// Placed AFTER pycvc_volren.i on purpose: volslice reuses volren.i's std::array
// typemaps and aliases volren::mat4/vec3d/transfer_function (`using volren::...`),
// all wrapped by pycvc_volren.i; also after volume/image/bounding_box/world_units,
// which box3d/slice_params/render_settings reference.
%include "pycvc_volslice.i"

// ── Phase 3 (Phase-6 binding): cvc::model + pycvc.load_model ─────────────
// The multi-mesh scene value type (meshes + materials + textures) and the native
// loader pycvc.load_model(path) → pycvc.model. %include'd HERE, after geometry
// AND image, because model::mesh holds a cvc::geometry and material holds a
// cvc::image — both must already be wrapped for model's accessors to marshal.
%include "pycvc_model.i"

// ── Phase 2: compute layer (SDF / meshing / quality / generators) ───────
// Module-level free functions + enum constants + QualityStats, taking/returning
// the real wrapped cvc::geometry/cvc::volume (declared above). Comes last so
// those value types are already known to SWIG.
%include "pycvc_algorithm.h"

// ── Phase 2b: belief-space grid navigation (cvc::nav) ───────────────────
// EDT / SDF / A* / line-of-sight / inflate / string-pull kernels for GRL-SNAM,
// numpy in and out via the ArrayView typemap above. Free functions + the
// NavSdfField view holder; all marshaling is inline in the .i.
%include "pycvc_nav.i"

// ── cvc::lod + cvc::simplify: LOD pyramids, selection, tiles, scene.cvch5 ─
// QEM simplify, per-asset pyramids, the selection math, tile partitioning +
// pooled tiled builds (with a Python on_tile callback), and the HDF5 store
// (scene_writer -> bytes, scene_reader from a path / bytes / open_verified).
// After geometry/volume/image/model (the pyramids and tiles hold them). It
// installs its own exception mapping + GIL release for its calls and restores
// PYCVC_DEFAULT_EXCEPTION at its end.
%include "pycvc_lod.i"

// ── The world's one representation: height_field, world_geometry, city bundles ─
// The same terrain, buildings and surface table the renderer, the physics world
// and the navigator read in C++ (WORLD-GEOMETRY-ROADMAP.md, G4b). After geometry
// (height_field derives from it) and pycvc_nav.i (land cover is cvc::nav's).
%include "pycvc_world.i"

// ── cvc::physics and the navigator's plant (P6) ────────────────────────────────
// The physics world, its vehicles and the nav_plant that drives a sim_world's agents
// as those vehicles (NAV-VEHICLE-PHYSICS-ROADMAP.md, P6). After pycvc_nav.i (the
// sim_world handles) and pycvc_world.i (height_field, for terrain bodies).
%include "pycvc_physics.i"

// ── Direct wrap of the REAL cvc::state class (full surface, no facade) ───────
// Like voxels/volume/geometry, the real libcvc header is %include'd here so
// Python gets the actual cvc::state tree node — not a parallel facade. This
// exposes the value channel (value()/value(str)), the boost::any DATA channel
// (data()/data(obj)/resolvedData() via the typemap below), tree navigation
// (child(), children(), name/fullName), the comment/hidden/read-only flags, and
// the whole Phase 8 LINK surface: linkTo (opaque/transparent), writable-link
// write-through, and resolvedValue()/resolvedData() read-through. state& is
// app-owned (handed out by instance()/child()); the protected ctor means Python
// never constructs or deletes one, so returned references are non-owning.
//
// The free functions in pycvc_state.h (state_set/get/…) and the state_observer
// director stay: they are the string-keyed convenience layer and the change-push
// path (boost::signals2 signals do not marshal, so Python change-notify rides
// the director, not the wrapped signal members).
%{
#include <cvc/state/state.h>
%}

// boost::any <-> Python for the data channel. Marshals the common scalars both
// ways (bool/int/float/str; empty any == None). bool BEFORE int (a Python bool
// is a PyLong subclass). Anything else surfaces as None rather than a leaked
// opaque pointer — the value channel or a registered codec covers richer types.
%typemap(out) boost::any {
  const boost::any &_a = $1;
  if (_a.empty()) { $result = Py_None; Py_INCREF(Py_None); }
  else if (_a.type() == typeid(bool)) { $result = PyBool_FromLong(boost::any_cast<bool>(_a) ? 1 : 0); }
  else if (_a.type() == typeid(int)) { $result = PyLong_FromLong(boost::any_cast<int>(_a)); }
  else if (_a.type() == typeid(long)) { $result = PyLong_FromLong(boost::any_cast<long>(_a)); }
  else if (_a.type() == typeid(long long)) { $result = PyLong_FromLongLong(boost::any_cast<long long>(_a)); }
  else if (_a.type() == typeid(unsigned int)) { $result = PyLong_FromUnsignedLong(boost::any_cast<unsigned int>(_a)); }
  else if (_a.type() == typeid(unsigned long)) { $result = PyLong_FromUnsignedLong(boost::any_cast<unsigned long>(_a)); }
  else if (_a.type() == typeid(float)) { $result = PyFloat_FromDouble(boost::any_cast<float>(_a)); }
  else if (_a.type() == typeid(double)) { $result = PyFloat_FromDouble(boost::any_cast<double>(_a)); }
  else if (_a.type() == typeid(std::string)) { $result = PyUnicode_FromString(boost::any_cast<std::string>(_a).c_str()); }
  else { $result = Py_None; Py_INCREF(Py_None); }
}
%typemap(in) const boost::any & (boost::any _tmp) {
  if ($input == Py_None) { _tmp = boost::any(); }
  else if (PyBool_Check($input)) { _tmp = boost::any($input == Py_True); }
  else if (PyLong_Check($input)) { _tmp = boost::any((int)PyLong_AsLong($input)); }
  else if (PyFloat_Check($input)) { _tmp = boost::any(PyFloat_AsDouble($input)); }
  else if (PyUnicode_Check($input)) { const char *_s = PyUnicode_AsUTF8($input); _tmp = boost::any(std::string(_s ? _s : "")); }
  else { PyErr_SetString(PyExc_TypeError, "state.data: expected bool/int/float/str/None"); SWIG_fail; }
  $1 = &_tmp;
}
// Overload dispatch needs a typecheck: without it SWIG cannot tell that a Python
// scalar matches data(const boost::any&) vs the no-arg data(). Accept exactly the
// scalars the in-typemap marshals.
%typemap(typecheck, precedence=SWIG_TYPECHECK_STRING) const boost::any & {
  $1 = ($input == Py_None || PyBool_Check($input) || PyLong_Check($input)
        || PyFloat_Check($input) || PyUnicode_Check($input)) ? 1 : 0;
}

// --- members that do not marshal or are not wanted on the Python surface ---
%ignore cvc::state_future;               // futures template (boost::chrono/signals2)
%ignore cvc::state::startup_connection;  // process-global startup registry handle
%ignore cvc::state::_startup;            // static registry vectors (boost::function)
%ignore cvc::state::_appStartup;
%ignore cvc::state::on_startup;          // boost::function registration
%ignore cvc::state::lastMod;             // boost::posix_time::ptime
%ignore cvc::state::ptree;               // boost::property_tree
%ignore cvc::state::traverse;            // boost::function callback param
%ignore cvc::state::updateValue;         // std::function callback param (compareAndSetValue and
                                         // exchangeValue are wrapped)
%ignore cvc::state::valueData;           // template
%ignore cvc::state::wait_for_value;      // template + boost::chrono
%ignore cvc::state::wait_for_data;       // template + boost::chrono
%ignore cvc::state::value_future;        // template
%ignore cvc::state::isData;              // template
%ignore cvc::state::expireAt;            // boost::posix_time::ptime
%ignore cvc::state::expireAfter;         // boost::posix_time::time_duration
%ignore cvc::state::expiryTime;          // boost::posix_time::ptime
%ignore cvc::state::resolveLink;         // returns nested link_resolution struct
%ignore cvc::state::link_resolution;     // (Python read-through uses resolvedValue/Data)
%ignore cvc::state::link_resolution_kind;
%ignore cvc::state::resolveRemote;       // distributed shard / delegation layer
%ignore cvc::state::remote_link_resolution;
%ignore cvc::state::remote_resolution_kind;
%ignore cvc::state::sendMessage;         // out-of-band messaging layer
%ignore cvc::state::send_message_result;
// boost::signals2 signals — Python change-notify rides the state_observer director
%ignore cvc::state::valueChanged;
%ignore cvc::state::dataChanged;
%ignore cvc::state::childChanged;
%ignore cvc::state::destroyed;
%ignore cvc::state::commentChanged;
%ignore cvc::state::hiddenChanged;
%ignore cvc::state::readOnlyChanged;
%ignore cvc::state::propertyChanged;
%ignore cvc::state::properties; // property_keys() + property(key) cover it
%ignore cvc::state::linkChanged;
%ignore cvc::state::traverseEnter;
%ignore cvc::state::traverseExit;
%ignore cvc::state::expiring;
// conversion operators (implicit std::string / ptree)
%ignore cvc::state::operator std::string;
%ignore cvc::state::operator boost::property_tree::ptree;

// operator()(childname) -> child(): the tree navigation accessor.
%rename(child) cvc::state::operator();

// The Python class is `State` (capitalized). This is not cosmetic: the legacy
// free-function layer in pycvc_state.h uses names like state_save/state_children,
// and SWIG's flat member wrappers for cvc::state::save/children would be the SAME
// symbols (state_save/…) — a hard "multiply defined" collision. Naming the class
// State makes the member wrappers State_save/State_children, distinct from the
// free functions, so both coexist while callers migrate off the free layer.
%rename(State) cvc::state;

// Nested enum class link_mode {opaque, transparent} lifts to the module so
// linkTo(path, mode) / setLinkMode(mode) / linkMode() are callable from Python.
%feature("flatnested") cvc::state::link_mode;

%include "cvc/state/state.h"

// ── Phase 3: state access + push callbacks ──────────────────────────────
// state_has/children/remove act on the shared root; state_observer is a
// director base — a Python subclass overriding on_changed() is called by C++
// on every state mutation. Only this class gets a director.
%feature("director") pycvc::state_observer;
%include "pycvc_state.h"

// ── UTF-8 display-width utilities (cvc::text, roadmap §17) ───────────────
// text_display_width / _pad_to_width / _truncate_to_width / … so a pycvc UI does
// column math in display columns, not bytes or codepoints. std::string round-trips
// UTF-8 both ways here (§17.6, audited).
%include "pycvc_text.i"

// ── Async state handlers on a bounded coroutine pool ────────────────────
// AsyncStateObserver rides on the state_observer director: C++ delivers
// on_changed(path) SYNCHRONOUSLY on the state writer thread (GIL held), and we
// marshal it onto an asyncio loop with call_soon_threadsafe (thread-safe, never
// blocks the writer). A bounded queue feeds a fixed pool of N worker coroutines,
// so a burst of state changes can't spawn unbounded work; on overflow the queue
// drops (oldest by default) and counts it. Because handlers re-read current
// state, dropping intermediate changes is safe — you converge to the latest.
%pythoncode %{
class AsyncStateObserver(state_observer):
    """Dispatch state-change callbacks to async handlers on a bounded pool.

    Override `async def handle(self, path)` in a subclass, or pass
    handler=<async fn>. Then call start(app, loop) to begin watching. Do NOT
    override on_changed() — that is the C++ director callback (runs on the writer
    thread) and is used internally to marshal onto the loop.

        obs = pycvc.AsyncStateObserver(handler, concurrency=8, maxsize=1000)
        obs.start(app)                 # from inside a running loop
        ...
        await obs.drain(); await obs.stop()
    """

    def __init__(self, handler=None, *, concurrency=4, maxsize=1024,
                 overflow="drop_oldest"):
        super().__init__()
        if overflow not in ("drop_oldest", "drop_newest"):
            raise ValueError("overflow must be 'drop_oldest' or 'drop_newest'")
        self._handler = handler
        self._concurrency = int(concurrency)
        self._maxsize = int(maxsize)
        self._overflow = overflow
        self._loop = None
        self._queue = None
        self._workers = []
        self.dropped = 0

    async def handle(self, path):
        """Override this, or pass handler= to __init__."""
        if self._handler is None:
            raise NotImplementedError("override handle() or pass handler=")
        await self._handler(path)

    def start(self, app, loop=None):
        """Spawn the worker pool and begin watching `app`'s state tree."""
        import asyncio
        self._loop = loop or asyncio.get_running_loop()
        self._queue = asyncio.Queue(maxsize=self._maxsize)
        self._workers = [self._loop.create_task(self._worker())
                         for _ in range(self._concurrency)]
        self.watch(app)
        return self

    # C++ director callback — runs on the state WRITER thread (GIL held).
    def on_changed(self, path):
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(self._offer, path)

    def _offer(self, path):
        # Runs on the loop thread (asyncio.Queue is not thread-safe).
        import asyncio
        try:
            self._queue.put_nowait(path)
        except asyncio.QueueFull:
            self.dropped += 1
            if self._overflow == "drop_oldest":
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                    self._queue.put_nowait(path)
                except (asyncio.QueueEmpty, asyncio.QueueFull):
                    pass
            # drop_newest: drop `path` (already counted)

    async def _worker(self):
        import asyncio
        try:
            while True:
                path = await self._queue.get()
                try:
                    await self.handle(path)
                except Exception:
                    pass  # a bad handler must not kill the pool
                finally:
                    self._queue.task_done()
        except asyncio.CancelledError:
            pass

    async def drain(self):
        """Wait until every queued change has been processed."""
        if self._queue is not None:
            await self._queue.join()

    async def stop(self):
        """Stop watching and shut the pool down."""
        import asyncio
        self.unwatch()
        for w in self._workers:
            w.cancel()
        await asyncio.gather(*self._workers, return_exceptions=True)
        self._workers = []
%}

// ── state_exec (docs/PYCVC_STATE_EXEC_PARITY.md) ─────────────────────────
// DIRECT WRAP of cvc::state_exec under the C++ names. Where C++ takes a
// cvc::state&, Python passes a State (State.instance(app) is the root). Only
// the declarations below are visible to SWIG; the parity doc lists what is
// wrapped, what comes next, and what is deliberately not wrapped (the
// coroutine API, the string_view-based parser class, ...). The deep-data
// settings (data_options, data_options_from_state, "Deep data on the
// executor") are wrapped the same way as the parser settings.
%{
#include <cvc/state/state_exec/async_host.h>
#include <cvc/state/state_exec/async_scheduler.h>
#include <cvc/state/state_exec/builtins.h>
#include <cvc/state/state_exec/data_job.h>
#include <cvc/state/state_exec/detail/lock_home.h>
#include <cvc/state/state_exec/evaluator.h>
#include <cvc/state/state_exec/heap.h>
#include <cvc/state/state_exec/intrinsics.h>
#include <cvc/state/state_exec/parser.h>
#include <cvc/state/state_exec/process.h>
#include <cvc/state/state_exec/resource_policy.h>
#include <cvc/state/state_exec/scheduler.h>
#include <cvc/state/state_exec/scheduler_base.h>
#include <cvc/state/state_exec/stackless_evaluator.h>
#include <cvc/state/state_exec/state_value_codec.h>
#include <cvc/state/state_exec/stdlib.h>
#include <cvc/state/state_exec/types.h>
#include <cvc/core/thread_class.h>
#include "pycvc_value.h"
#include <algorithm>
#include <functional>
#include <initializer_list>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <variant>
#include <vector>

// SWIG passes by-value arguments with SWIG_STD_MOVE, which is std::move only when __cplusplus says
// C++11; MSVC reports 199711L without /Zc:__cplusplus, so there it copies, and a std::unique_ptr
// (a data job given to run_data_job) can't be copied. Move, as on GCC and Clang, for the wrapper
// functions, which come after this.
#undef SWIG_STD_MOVE
#define SWIG_STD_MOVE(OBJ) std::move(OBJ)

namespace pycvc {
// The evaluators' on_complete parameter type, under a name SWIG can match a typemap on.
using value_callback = std::function<void(const cvc::state_exec::value_t &)>;
} // namespace pycvc
%}

// ── values: the one value_t <-> Python conversion (pycvc_value.h) ──────
// Defined here because it creates and recognises the pycvc.symbol and
// pycvc.value proxies, which need the SWIG runtime; pycvc_exec.cpp calls it
// through pycvc_value.h.
%{
namespace pycvc {
namespace {
using cvc::state_exec::value_t;

swig_type_info *symbol_swig_type() {
  static swig_type_info *t = SWIG_TypeQuery("cvc::state_exec::symbol *");
  return t;
}
swig_type_info *value_swig_type() {
  static swig_type_info *t = SWIG_TypeQuery("cvc::state_exec::value_tag *");
  return t;
}

PyObject *text_to_py(const std::string &s) {
  return PyUnicode_DecodeUTF8(s.data(), static_cast<Py_ssize_t>(s.size()), "strict");
}

// The two directions walk a value with an explicit stack rather than recursing, so a value nested
// any depth converts (a parsed script may nest 100000 deep). A list or dict shared within a value
// converts once, to one object, so sharing survives the trip and a shared structure converts in
// time proportional to its size: a DSL list that holds itself becomes a Python list that holds
// itself. The other way, a Python value that holds itself raises ValueError, because only the
// evaluator's own containers are tracked for cycle collection (heap.h).

// Everything but a list or dict: a new reference, or nullptr with an error set.
PyObject *leaf_to_py(const value_t &v) {
  using namespace cvc::state_exec;
  if (v.is_nil())
    Py_RETURN_NONE;
  if (const auto *b = std::get_if<bool>(&v.v))
    return PyBool_FromLong(*b);
  if (const auto *i = std::get_if<int64_t>(&v.v))
    return PyLong_FromLongLong(static_cast<long long>(*i));
  if (const auto *d = std::get_if<double>(&v.v))
    return PyFloat_FromDouble(*d);
  if (const auto *s = std::get_if<std::string>(&v.v))
    return text_to_py(*s);
  if (const auto *by = std::get_if<bytes_value>(&v.v))
    return PyBytes_FromStringAndSize(by->data.data(), static_cast<Py_ssize_t>(by->data.size()));
  if (const auto *sym = std::get_if<symbol>(&v.v))
    return SWIG_NewPointerObj(new symbol(*sym), symbol_swig_type(), SWIG_POINTER_OWN);
  // closure / native_fn / data_object / generator: an opaque handle that converts back to the
  // same value, so Python can hold it and pass it into the DSL again.
  return SWIG_NewPointerObj(new value_t(v), value_swig_type(), SWIG_POINTER_OWN);
}

PyObject *to_py(const value_t &root) {
  using namespace cvc::state_exec;
  struct frame {
    const value_t *v; // a list or dict
    PyObject *out;    // its Python object, being filled
    size_t next;      // the next element to convert
    size_t size;      // the Python list's length
  };
  std::vector<frame> stack;
  std::unordered_map<const void *, PyObject *> made; // container storage -> its Python object
  // A new reference to `v` in Python. A list or dict not made yet is created empty and pushed to
  // be filled; one already made (shared, or an enclosing one) is the same object.
  auto convert = [&](const value_t &v) -> PyObject * {
    const auto *l = std::get_if<list_ptr>(&v.v);
    const auto *d = std::get_if<dict_ptr>(&v.v);
    if (l == nullptr && d == nullptr)
      return leaf_to_py(v);
    const void *storage = l ? static_cast<const void *>(l->get()) : d->get();
    if (storage == nullptr)
      return l ? PyList_New(0) : PyDict_New();
    if (const auto it = made.find(storage); it != made.end()) {
      Py_INCREF(it->second);
      return it->second;
    }
    const size_t size = l ? (*l)->size() : 0;
    PyObject *out = l ? PyList_New(static_cast<Py_ssize_t>(size)) : PyDict_New();
    if (out == nullptr)
      return nullptr;
    made.emplace(storage, out);
    stack.push_back({&v, out, 0, size});
    return out;
  };
  PyObject *result = convert(root);
  if (result == nullptr)
    return nullptr;
  while (!stack.empty()) {
    const size_t top = stack.size() - 1; // an index: convert() may grow the stack
    const value_t &v = *stack[top].v;
    PyObject *out = stack[top].out;
    const size_t i = stack[top].next++;
    if (const auto *l = std::get_if<list_ptr>(&v.v)) {
      if (i == stack[top].size) {
        stack.pop_back();
        continue;
      }
      // The list can't change while the GIL is held, but a slot is filled with None rather than
      // left empty should it be shorter than the Python list made for it.
      PyObject *item = i < (*l)->size() ? convert((**l)[i]) : (Py_INCREF(Py_None), Py_None);
      if (item == nullptr) {
        Py_DECREF(result);
        return nullptr;
      }
      PyList_SET_ITEM(out, static_cast<Py_ssize_t>(i), item); // steals the reference
    } else {
      const auto &entries = **std::get_if<dict_ptr>(&v.v);
      if (i >= entries.size()) {
        stack.pop_back();
        continue;
      }
      PyObject *key = text_to_py(entries[i].first);
      PyObject *val = key ? convert(entries[i].second) : nullptr;
      const int rc = val ? PyDict_SetItem(out, key, val) : -1;
      Py_XDECREF(key);
      Py_XDECREF(val);
      if (rc < 0) {
        Py_DECREF(result);
        return nullptr;
      }
    }
  }
  return result;
}

// 1: `o` converted into `out`; 0: `o` is a list, tuple or dict, which from_py walks; -1: an error
// is set.
int leaf_from_py(PyObject *o, value_t &out) {
  using namespace cvc::state_exec;
  if (o == nullptr || o == Py_None) {
    out = value_t();
    return 1;
  }
  if (PyBool_Check(o)) { // before int: bool is an int subclass
    out = value_t(o == Py_True);
    return 1;
  }
  if (PyLong_Check(o)) {
    int overflow = 0;
    const long long x = PyLong_AsLongLongAndOverflow(o, &overflow);
    if (overflow != 0) {
      PyErr_SetString(PyExc_OverflowError, "pycvc: int does not fit in 64 bits");
      return -1;
    }
    if (x == -1 && PyErr_Occurred())
      return -1;
    out = value_t(static_cast<int64_t>(x));
    return 1;
  }
  if (PyFloat_Check(o)) {
    const double d = PyFloat_AsDouble(o);
    if (d == -1.0 && PyErr_Occurred())
      return -1;
    out = value_t(d);
    return 1;
  }
  if (PyUnicode_Check(o)) {
    Py_ssize_t n = 0;
    const char *s = PyUnicode_AsUTF8AndSize(o, &n); // fails (and raises) on lone surrogates
    if (s == nullptr)
      return -1;
    out = value_t(std::string(s, static_cast<size_t>(n)));
    return 1;
  }
  if (PyBytes_Check(o)) {
    out = make_bytes(std::string(PyBytes_AS_STRING(o), static_cast<size_t>(PyBytes_GET_SIZE(o))));
    return 1;
  }
  if (PyByteArray_Check(o)) {
    out = make_bytes(
        std::string(PyByteArray_AS_STRING(o), static_cast<size_t>(PyByteArray_GET_SIZE(o))));
    return 1;
  }
  if (PyList_Check(o) || PyTuple_Check(o) || PyDict_Check(o))
    return 0;
  void *p = nullptr;
  if (SWIG_IsOK(SWIG_ConvertPtr(o, &p, symbol_swig_type(), 0)) && p != nullptr) {
    out = value_t(*static_cast<symbol *>(p));
    return 1;
  }
  if (SWIG_IsOK(SWIG_ConvertPtr(o, &p, value_swig_type(), 0)) && p != nullptr) {
    out = *static_cast<value_t *>(p);
    return 1;
  }
  PyErr_Clear(); // a failed proxy lookup above may leave an attribute error behind
  PyErr_Format(PyExc_TypeError, "pycvc: cannot convert %.200s to a state_exec value",
               Py_TYPE(o)->tp_name);
  return -1;
}

bool from_py(PyObject *root, value_t &out) {
  using namespace cvc::state_exec;
  const int leaf = leaf_from_py(root, out);
  if (leaf != 0)
    return leaf > 0;
  struct frame {
    PyObject *o; // a list, tuple or dict
    value_t v;   // its value, being filled (shares its storage with the value handed out)
    Py_ssize_t pos;
  };
  std::vector<frame> stack;
  std::unordered_map<PyObject *, value_t> made; // container -> its value
  std::unordered_set<PyObject *> filling;       // containers on the stack: met again, a cycle
  // References on every container made, so none goes while its value is in `made` (a lookup of a
  // proxy can run Python code). Dropped when this returns.
  struct holds {
    std::vector<PyObject *> objects;
    ~holds() {
      for (PyObject *o : objects)
        Py_DECREF(o);
    }
  } held;
  // `o`'s value: one already made is shared; otherwise an empty list or dict, pushed to be filled.
  auto container = [&](PyObject *o, value_t &v) -> bool {
    if (filling.count(o) != 0) {
      PyErr_SetString(PyExc_ValueError, "pycvc: value contains itself");
      return false;
    }
    if (const auto it = made.find(o); it != made.end()) {
      v = it->second;
      return true;
    }
    if (PyDict_Check(o)) {
      v = make_dict();
    } else {
      v = make_list();
      std::get<list_ptr>(v.v)->reserve(
          static_cast<size_t>(PyTuple_Check(o) ? PyTuple_GET_SIZE(o) : PyList_GET_SIZE(o)));
    }
    Py_INCREF(o);
    held.objects.push_back(o);
    made.emplace(o, v);
    filling.insert(o);
    stack.push_back({o, v, 0});
    return true;
  };
  // An element's value, into `v`.
  auto element = [&](PyObject *item, value_t &v) -> bool {
    Py_INCREF(item); // the container could drop it while a proxy lookup runs Python code
    const int r = leaf_from_py(item, v);
    const bool ok = r > 0 || (r == 0 && container(item, v));
    Py_DECREF(item);
    return ok;
  };
  if (!container(root, out))
    return false;
  while (!stack.empty()) {
    const size_t top = stack.size() - 1; // an index: container() may grow the stack
    PyObject *o = stack[top].o;
    if (PyDict_Check(o)) {
      PyObject *k = nullptr;
      PyObject *item = nullptr;
      if (!PyDict_Next(o, &stack[top].pos, &k, &item)) {
        filling.erase(o);
        stack.pop_back();
        continue;
      }
      if (!PyUnicode_Check(k)) {
        PyErr_Format(PyExc_TypeError, "pycvc: dict keys must be str, not %.200s",
                     Py_TYPE(k)->tp_name);
        return false;
      }
      Py_ssize_t n = 0;
      const char *ks = PyUnicode_AsUTF8AndSize(k, &n);
      if (ks == nullptr)
        return false;
      std::string key(ks, static_cast<size_t>(n));
      value_t e;
      if (!element(item, e))
        return false;
      std::get<dict_ptr>(stack[top].v.v)->emplace_back(std::move(key), std::move(e));
    } else {
      const bool tup = PyTuple_Check(o);
      const Py_ssize_t i = stack[top].pos;
      if (i >= (tup ? PyTuple_GET_SIZE(o) : PyList_GET_SIZE(o))) {
        filling.erase(o);
        stack.pop_back();
        continue;
      }
      stack[top].pos = i + 1;
      value_t e;
      if (!element(tup ? PyTuple_GET_ITEM(o, i) : PyList_GET_ITEM(o, i), e))
        return false;
      std::get<list_ptr>(stack[top].v.v)->push_back(std::move(e));
    }
  }
  return true;
}
} // namespace

PyObject *value_to_py(const cvc::state_exec::value_t &v) { return to_py(v); }

PyObject *values_to_py_tuple(std::span<const cvc::state_exec::value_t> values) {
  PyObject *t = PyTuple_New(static_cast<Py_ssize_t>(values.size()));
  if (t == nullptr)
    return nullptr;
  for (size_t i = 0; i < values.size(); ++i) {
    PyObject *item = to_py(values[i]);
    if (item == nullptr) {
      Py_DECREF(t);
      return nullptr;
    }
    PyTuple_SET_ITEM(t, static_cast<Py_ssize_t>(i), item);
  }
  return t;
}

bool py_to_value(PyObject *o, cvc::state_exec::value_t &out) { return from_py(o, out); }
} // namespace pycvc
%}

%typemap(in) cvc::state_exec::value_tag (cvc::state_exec::value_tag temp) %{
  if (!pycvc::py_to_value($input, temp))
    SWIG_fail;
  $1 = temp;
%}
%typemap(in) const cvc::state_exec::value_tag & (cvc::state_exec::value_tag temp) %{
  if (!pycvc::py_to_value($input, temp))
    SWIG_fail;
  $1 = &temp;
%}
// Any Python object may be a value, so the value overload is tried last (SWIG tries the lowest
// precedence first): a str goes to execute(script) and a process_status to
// to_string(process_status), not to their value_t overloads.
%typemap(typecheck, precedence=10000) cvc::state_exec::value_tag,
    const cvc::state_exec::value_tag & "$1 = 1;"
%typemap(out) cvc::state_exec::value_tag %{
  $result = pycvc::value_to_py(static_cast<const cvc::state_exec::value_tag &>($1));
  if (!$result)
    SWIG_fail;
%}
%typemap(out) std::vector<cvc::state_exec::value_tag> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over this initialization
    const std::vector<cvc::state_exec::value_tag> &pycvc_vals = $1;
    $result = PyList_New(static_cast<Py_ssize_t>(pycvc_vals.size()));
    if (!$result)
      SWIG_fail;
    for (size_t i = 0; i < pycvc_vals.size(); ++i) {
      PyObject *item = pycvc::value_to_py(pycvc_vals[i]);
      if (!item) {
        Py_DECREF($result);
        $result = nullptr;
        SWIG_fail;
      }
      PyList_SET_ITEM($result, static_cast<Py_ssize_t>(i), item);
    }
  }
%}

// ── maps and optionals (the scheduler surface) ──────────────────────────
// A name -> value map (environment::bindings, execute_options::signal_handlers) is a dict with str
// keys, in key order. A pid -> value map (run()'s results) is a dict with int keys.
%{
namespace pycvc {
namespace {
using value_map = std::unordered_map<std::string, cvc::state_exec::value_t>;

PyObject *value_map_to_py(const value_map &m) {
  std::vector<const value_map::value_type *> entries;
  entries.reserve(m.size());
  for (const auto &kv : m)
    entries.push_back(&kv);
  std::sort(entries.begin(), entries.end(),
            [](const auto *a, const auto *b) { return a->first < b->first; });
  PyObject *dict = PyDict_New();
  if (dict == nullptr)
    return nullptr;
  for (const auto *kv : entries) {
    PyObject *key = text_to_py(kv->first);
    PyObject *val = key != nullptr ? value_to_py(kv->second) : nullptr;
    const int rc = val != nullptr ? PyDict_SetItem(dict, key, val) : -1;
    Py_XDECREF(key);
    Py_XDECREF(val);
    if (rc != 0) {
      Py_DECREF(dict);
      return nullptr;
    }
  }
  return dict;
}

bool py_to_value_map(PyObject *o, value_map &out) {
  if (!PyDict_Check(o)) {
    PyErr_Format(PyExc_TypeError, "pycvc: expected a dict of str to values, got %.200s",
                 Py_TYPE(o)->tp_name);
    return false;
  }
  value_map m;
  PyObject *key = nullptr;
  PyObject *val = nullptr;
  Py_ssize_t pos = 0;
  while (PyDict_Next(o, &pos, &key, &val)) {
    if (!PyUnicode_Check(key)) {
      PyErr_Format(PyExc_TypeError, "pycvc: dict keys must be str, got %.200s",
                   Py_TYPE(key)->tp_name);
      return false;
    }
    Py_ssize_t n = 0;
    const char *s = PyUnicode_AsUTF8AndSize(key, &n);
    cvc::state_exec::value_t v;
    if (s == nullptr || !py_to_value(val, v))
      return false;
    m[std::string(s, static_cast<size_t>(n))] = std::move(v);
  }
  out = std::move(m);
  return true;
}
} // namespace
} // namespace pycvc
%}

%typemap(in) const std::unordered_map<std::string, cvc::state_exec::value_tag> &
    (std::unordered_map<std::string, cvc::state_exec::value_tag> temp) %{
  if (!pycvc::py_to_value_map($input, temp))
    SWIG_fail;
  $1 = &temp;
%}
%typemap(out) const std::unordered_map<std::string, cvc::state_exec::value_tag> & %{
  $result = pycvc::value_map_to_py(*$1);
  if (!$result)
    SWIG_fail;
%}
%typemap(out) std::unordered_map<int, cvc::state_exec::value_tag> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over these initializations
    const std::unordered_map<int, cvc::state_exec::value_tag> &pycvc_map = $1;
    $result = PyDict_New();
    if (!$result)
      SWIG_fail;
    for (const auto &kv : pycvc_map) {
      PyObject *k = PyLong_FromLong(kv.first);
      PyObject *v = k ? pycvc::value_to_py(kv.second) : nullptr;
      const int rc = v ? PyDict_SetItem($result, k, v) : -1;
      Py_XDECREF(k);
      Py_XDECREF(v);
      if (rc != 0) {
        Py_DECREF($result);
        $result = nullptr;
        SWIG_fail;
      }
    }
  }
%}

// Limits passed as std::optional: None is std::nullopt.
%typemap(in) std::optional<unsigned long long> %{
  if ($input != Py_None) {
    const unsigned long long pycvc_v = PyLong_AsUnsignedLongLong($input);
    if (pycvc_v == static_cast<unsigned long long>(-1) && PyErr_Occurred())
      SWIG_fail;
    $1 = pycvc_v;
  }
%}
%typemap(typecheck, precedence=SWIG_TYPECHECK_UINT64) std::optional<unsigned long long>
    "$1 = ($input == Py_None || PyLong_Check($input)) ? 1 : 0;"
%typemap(in) std::optional<double> %{
  if ($input != Py_None) {
    const double pycvc_v = PyFloat_AsDouble($input);
    if (pycvc_v == -1.0 && PyErr_Occurred())
      SWIG_fail;
    $1 = pycvc_v;
  }
%}
%typemap(typecheck, precedence=SWIG_TYPECHECK_DOUBLE) std::optional<double>
    "$1 = ($input == Py_None || PyFloat_Check($input) || PyLong_Check($input)) ? 1 : 0;"

// Optional results other than values are None when empty (a value can be nil, so get_result and
// pop_pending_message raise KeyError instead; see their %extend below).
%typemap(out) std::optional<std::string> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over this initialization
    const std::optional<std::string> &pycvc_opt = $1;
    if (pycvc_opt) {
      $result = PyUnicode_DecodeUTF8(pycvc_opt->data(), static_cast<Py_ssize_t>(pycvc_opt->size()),
                                     "surrogateescape");
      if (!$result)
        SWIG_fail;
    } else {
      $result = Py_None;
      Py_INCREF(Py_None);
    }
  }
%}
%typemap(out) std::optional<cvc::state_exec::process_info> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over this initialization
    const std::optional<cvc::state_exec::process_info> &pycvc_opt = $1;
    if (pycvc_opt) {
      $result = SWIG_NewPointerObj(new cvc::state_exec::process_info(*pycvc_opt),
                                   $descriptor(cvc::state_exec::process_info *), SWIG_POINTER_OWN);
      if (!$result)
        SWIG_fail;
    } else {
      $result = Py_None;
      Py_INCREF(Py_None);
    }
  }
%}
%typemap(out) std::vector<cvc::state_exec::process_info> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over this initialization
    const std::vector<cvc::state_exec::process_info> &pycvc_infos = $1;
    $result = PyList_New(static_cast<Py_ssize_t>(pycvc_infos.size()));
    if (!$result)
      SWIG_fail;
    for (size_t i = 0; i < pycvc_infos.size(); ++i) {
      PyObject *item = SWIG_NewPointerObj(new cvc::state_exec::process_info(pycvc_infos[i]),
                                          $descriptor(cvc::state_exec::process_info *),
                                          SWIG_POINTER_OWN);
      if (!item) {
        Py_DECREF($result);
        $result = nullptr;
        SWIG_fail;
      }
      PyList_SET_ITEM($result, static_cast<Py_ssize_t>(i), item);
    }
  }
%}

// ── members and results of the evaluator and process types ──
// A value_t member reads as a Python value.
%typemap(out) const cvc::state_exec::value_tag & %{
  $result = pycvc::value_to_py(*$1);
  if (!$result)
    SWIG_fail;
%}

%{
namespace pycvc {
namespace {
using cvc::state_exec::value_t;

PyObject *value_list_to_py(const std::vector<value_t> &vals) {
  PyObject *list = PyList_New(static_cast<Py_ssize_t>(vals.size()));
  if (list == nullptr)
    return nullptr;
  for (size_t i = 0; i < vals.size(); ++i) {
    PyObject *item = value_to_py(vals[i]);
    if (item == nullptr) {
      Py_DECREF(list);
      return nullptr;
    }
    PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), item);
  }
  return list;
}

// Each item of the sequence `o` converted with `convert`, or false with a Python error set.
template <class T, class F> bool py_to_list(PyObject *o, std::vector<T> &out, F convert) {
  PyObject *seq = PySequence_Fast(o, "pycvc: expected a list");
  if (seq == nullptr)
    return false;
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  std::vector<T> items;
  items.reserve(static_cast<size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    T item;
    if (!convert(PySequence_Fast_GET_ITEM(seq, i), item)) {
      Py_DECREF(seq);
      return false;
    }
    items.push_back(std::move(item));
  }
  Py_DECREF(seq);
  out = std::move(items);
  return true;
}

bool py_to_value_list(PyObject *o, std::vector<value_t> &out) {
  return py_to_list(o, out, [](PyObject *item, value_t &v) { return py_to_value(item, v); });
}

// A struct proxy of `type` copied out (a list item), or false with a TypeError set.
template <class T> bool py_to_struct(PyObject *o, swig_type_info *type, T &out) {
  void *p = nullptr;
  if (!SWIG_IsOK(SWIG_ConvertPtr(o, &p, type, 0)) || p == nullptr) {
    PyErr_Format(PyExc_TypeError, "pycvc: expected a pycvc.%s, got %.200s",
                 type ? type->str : "?", Py_TYPE(o)->tp_name);
    return false;
  }
  out = *static_cast<T *>(p);
  return true;
}

// A new owned proxy of a copy of `v`, or nullptr with a Python error set.
template <class T> PyObject *struct_to_py(const T &v, swig_type_info *type) {
  return SWIG_NewPointerObj(new T(v), type, SWIG_POINTER_OWN);
}
} // namespace
} // namespace pycvc
%}

// A list of values, and a list of lists of values (eval_frame members).
%typemap(in) const std::vector<cvc::state_exec::value_tag> &
    (std::vector<cvc::state_exec::value_tag> temp) %{
  if (!pycvc::py_to_value_list($input, temp))
    SWIG_fail;
  $1 = &temp;
%}
%typemap(out) const std::vector<cvc::state_exec::value_tag> & %{
  $result = pycvc::value_list_to_py(*$1);
  if (!$result)
    SWIG_fail;
%}
%typemap(in) const std::vector<std::vector<cvc::state_exec::value_tag> > &
    (std::vector<std::vector<cvc::state_exec::value_tag> > temp) %{
  if (!pycvc::py_to_list($input, temp, [](PyObject *item, std::vector<cvc::state_exec::value_t> &v) {
        return pycvc::py_to_value_list(item, v);
      }))
    SWIG_fail;
  $1 = &temp;
%}
%typemap(out) const std::vector<std::vector<cvc::state_exec::value_tag> > & %{
  $result = PyList_New(static_cast<Py_ssize_t>($1->size()));
  if (!$result)
    SWIG_fail;
  for (size_t i = 0; i < $1->size(); ++i) {
    PyObject *item = pycvc::value_list_to_py((*$1)[i]);
    if (!item) {
      Py_DECREF($result);
      $result = nullptr;
      SWIG_fail;
    }
    PyList_SET_ITEM($result, static_cast<Py_ssize_t>(i), item);
  }
%}

// A list of structs (eval_frame, process::watch_event) reads as a list of copies and is set from a
// list of them.
%define PYCVC_STATE_EXEC_STRUCT_LIST(TYPE)
%typemap(in) const std::vector<TYPE> & (std::vector<TYPE> temp) {
  if (!pycvc::py_to_list($input, temp, [](PyObject *item, TYPE &v) {
        return pycvc::py_to_struct(item, $descriptor(TYPE *), v);
      }))
    SWIG_fail;
  $1 = &temp;
}
%typemap(out) const std::vector<TYPE> & {
  $result = PyList_New(static_cast<Py_ssize_t>($1->size()));
  if (!$result)
    SWIG_fail;
  for (size_t i = 0; i < $1->size(); ++i) {
    PyObject *item = pycvc::struct_to_py((*$1)[i], $descriptor(TYPE *));
    if (!item) {
      Py_DECREF($result);
      $result = nullptr;
      SWIG_fail;
    }
    PyList_SET_ITEM($result, static_cast<Py_ssize_t>(i), item);
  }
}
%enddef
PYCVC_STATE_EXEC_STRUCT_LIST(cvc::state_exec::eval_frame)
PYCVC_STATE_EXEC_STRUCT_LIST(cvc::state_exec::process::watch_event)

// process::watch_handlers: a dict of watch id -> watch_handler_entry (copies).
%typemap(in) const std::unordered_map<int, cvc::state_exec::process::watch_handler_entry> &
    (std::unordered_map<int, cvc::state_exec::process::watch_handler_entry> temp) {
  if (!PyDict_Check($input)) {
    PyErr_SetString(PyExc_TypeError, "pycvc: expected a dict of int to watch_handler_entry");
    SWIG_fail;
  }
  PyObject *k = nullptr;
  PyObject *v = nullptr;
  Py_ssize_t pos = 0;
  while (PyDict_Next($input, &pos, &k, &v)) {
    const long id = PyLong_AsLong(k);
    if (id == -1 && PyErr_Occurred())
      SWIG_fail;
    cvc::state_exec::process::watch_handler_entry entry;
    if (!pycvc::py_to_struct(v, $descriptor(cvc::state_exec::process::watch_handler_entry *),
                             entry))
      SWIG_fail;
    temp[static_cast<int>(id)] = std::move(entry);
  }
  $1 = &temp;
}
%typemap(out) const std::unordered_map<int, cvc::state_exec::process::watch_handler_entry> & {
  $result = PyDict_New();
  if (!$result)
    SWIG_fail;
  for (const auto &kv : *$1) {
    PyObject *k = PyLong_FromLong(kv.first);
    PyObject *v = k ? pycvc::struct_to_py(
                          kv.second, $descriptor(cvc::state_exec::process::watch_handler_entry *))
                    : nullptr;
    const int rc = v ? PyDict_SetItem($result, k, v) : -1;
    Py_XDECREF(k);
    Py_XDECREF(v);
    if (rc != 0) {
      Py_DECREF($result);
      $result = nullptr;
      SWIG_fail;
    }
  }
}

// evaluator::macro_map: a dict of macro name -> (params, body), params a list of pycvc.symbol.
%{
namespace pycvc {
namespace {
using macro_map = cvc::state_exec::evaluator::macro_map;

PyObject *macro_map_to_py(const macro_map &m) {
  PyObject *dict = PyDict_New();
  if (dict == nullptr)
    return nullptr;
  for (const auto &kv : m) {
    std::vector<value_t> params(kv.second.first.begin(), kv.second.first.end());
    PyObject *key = text_to_py(kv.first);
    PyObject *ps = key ? value_list_to_py(params) : nullptr;
    PyObject *body = ps ? value_to_py(kv.second.second) : nullptr;
    PyObject *pair = body ? PyTuple_Pack(2, ps, body) : nullptr;
    const int rc = pair ? PyDict_SetItem(dict, key, pair) : -1;
    Py_XDECREF(key);
    Py_XDECREF(ps);
    Py_XDECREF(body);
    Py_XDECREF(pair);
    if (rc != 0) {
      Py_DECREF(dict);
      return nullptr;
    }
  }
  return dict;
}

bool py_to_macro_map(PyObject *o, macro_map &out) {
  if (!PyDict_Check(o)) {
    PyErr_SetString(PyExc_TypeError, "pycvc: expected a dict of name to (params, body)");
    return false;
  }
  macro_map m;
  PyObject *key = nullptr;
  PyObject *val = nullptr;
  Py_ssize_t pos = 0;
  while (PyDict_Next(o, &pos, &key, &val)) {
    if (!PyUnicode_Check(key) || !PyTuple_Check(val) || PyTuple_GET_SIZE(val) != 2) {
      PyErr_SetString(PyExc_TypeError,
                      "pycvc: a macro is name (str) -> (params, body), params a list of symbols");
      return false;
    }
    std::vector<value_t> params;
    value_t body;
    if (!py_to_value_list(PyTuple_GET_ITEM(val, 0), params) ||
        !py_to_value(PyTuple_GET_ITEM(val, 1), body))
      return false;
    std::vector<cvc::state_exec::symbol> syms;
    for (const value_t &p : params) {
      const auto *s = std::get_if<cvc::state_exec::symbol>(&p.v);
      if (s == nullptr) {
        PyErr_SetString(PyExc_TypeError, "pycvc: macro params must be pycvc.symbol");
        return false;
      }
      syms.push_back(*s);
    }
    const char *name = PyUnicode_AsUTF8(key);
    if (name == nullptr)
      return false;
    m[name] = {std::move(syms), std::move(body)};
  }
  out = std::move(m);
  return true;
}
} // namespace
} // namespace pycvc
%}
%typemap(in) const cvc::state_exec::evaluator::macro_map & (cvc::state_exec::evaluator::macro_map temp) %{
  if (!pycvc::py_to_macro_map($input, temp))
    SWIG_fail;
  $1 = &temp;
%}
%typemap(out) const cvc::state_exec::evaluator::macro_map & %{
  $result = pycvc::macro_map_to_py(*$1);
  if (!$result)
    SWIG_fail;
%}
%typemap(out) cvc::state_exec::evaluator::macro_map %{
  $result = pycvc::macro_map_to_py(static_cast<const cvc::state_exec::evaluator::macro_map &>($1));
  if (!$result)
    SWIG_fail;
%}

// Optional strings as members (process::recv_path, exit_error): None or str.
%typemap(in) const std::optional<std::string> & (std::optional<std::string> temp) %{
  if ($input != Py_None) {
    Py_ssize_t pycvc_n = 0;
    const char *pycvc_s =
        PyUnicode_Check($input) ? PyUnicode_AsUTF8AndSize($input, &pycvc_n) : nullptr;
    if (pycvc_s == nullptr) {
      if (!PyErr_Occurred())
        PyErr_SetString(PyExc_TypeError, "pycvc: expected str or None");
      SWIG_fail;
    }
    temp = std::string(pycvc_s, static_cast<size_t>(pycvc_n));
  }
  $1 = &temp;
%}
%typemap(out) const std::optional<std::string> & %{
  if (*$1) {
    $result = PyUnicode_DecodeUTF8((*$1)->data(), static_cast<Py_ssize_t>((*$1)->size()),
                                   "surrogateescape");
    if (!$result)
      SWIG_fail;
  } else {
    $result = Py_None;
    Py_INCREF(Py_None);
  }
%}
// process::time_left() and alloc_limit(): None when unlimited.
%typemap(out) std::optional<double> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over this initialization
    const std::optional<double> &pycvc_opt = $1;
    $result = pycvc_opt ? PyFloat_FromDouble(*pycvc_opt) : (Py_INCREF(Py_None), Py_None);
  }
%}
%typemap(out) std::optional<size_t> %{
  { // a block, so the wrapper's later `goto fail` doesn't jump over this initialization
    const std::optional<size_t> &pycvc_opt = $1;
    $result = pycvc_opt ? PyLong_FromSize_t(*pycvc_opt) : (Py_INCREF(Py_None), Py_None);
  }
%}

// restrict_special_forms takes any iterable of str, or None for no restriction;
// all_special_forms() returns a set.
// $1 is always assigned: SWIG holds this argument in a SwigValueWrapper, which is empty until then.
%typemap(in) std::shared_ptr<const std::set<std::string> > %{
  $1 = std::shared_ptr<const std::set<std::string>>();
  if ($input != Py_None) {
    auto pycvc_forms = std::make_shared<std::set<std::string>>();
    PyObject *pycvc_it = PyObject_GetIter($input);
    if (pycvc_it == nullptr)
      SWIG_fail;
    while (PyObject *pycvc_item = PyIter_Next(pycvc_it)) {
      const char *pycvc_s = PyUnicode_Check(pycvc_item) ? PyUnicode_AsUTF8(pycvc_item) : nullptr;
      if (pycvc_s != nullptr)
        pycvc_forms->insert(pycvc_s);
      Py_DECREF(pycvc_item);
      if (pycvc_s == nullptr) {
        Py_DECREF(pycvc_it);
        if (!PyErr_Occurred())
          PyErr_SetString(PyExc_TypeError, "pycvc: special forms must be str");
        SWIG_fail;
      }
    }
    Py_DECREF(pycvc_it);
    if (PyErr_Occurred())
      SWIG_fail;
    $1 = std::move(pycvc_forms);
  }
%}
%typemap(out) const std::set<std::string> & %{
  $result = PySet_New(nullptr);
  if (!$result)
    SWIG_fail;
  for (const std::string &pycvc_form : *$1) {
    PyObject *pycvc_s = PyUnicode_FromStringAndSize(pycvc_form.data(),
                                                    static_cast<Py_ssize_t>(pycvc_form.size()));
    if (!pycvc_s || PySet_Add($result, pycvc_s) != 0) {
      Py_XDECREF(pycvc_s);
      Py_DECREF($result);
      $result = nullptr;
      SWIG_fail;
    }
    Py_DECREF(pycvc_s);
  }
%}

// The evaluators' on_complete: a Python callable or None, called with the result.
%typemap(in) pycvc::value_callback %{
  $1 = pycvc::value_callback(); // always assigned, should SWIG hold it in a SwigValueWrapper
  if ($input != Py_None) {
    if (!PyCallable_Check($input)) { // checked here: a typemap runs outside the try block
      PyErr_SetString(PyExc_TypeError, "pycvc: on_complete must be callable or None");
      SWIG_fail;
    }
    $1 = pycvc::make_python_on_complete($input);
  }
%}
%typemap(typecheck, precedence=SWIG_TYPECHECK_POINTER) pycvc::value_callback
    "$1 = ($input == Py_None || PyCallable_Check($input)) ? 1 : 0;"

// ── native functions from Python, and the names C++ keeps ──
%{
#include <mutex>
namespace pycvc {
namespace {
// A native_fn from `fn`: the native function a pycvc.value holds, or a Python callable wrapped by
// python_native_fn (which throws std::invalid_argument, prefixed with `who`, for anything else).
cvc::state_exec::native_fn native_from_py(const std::string &who, const std::string &name,
                                          PyObject *fn) {
  void *p = nullptr;
  if (SWIG_IsOK(SWIG_ConvertPtr(fn, &p, value_swig_type(), 0)) && p != nullptr) {
    const auto &held = *static_cast<cvc::state_exec::value_t *>(p);
    if (const auto *native = std::get_if<cvc::state_exec::native_fn>(&held.v))
      return *native;
  }
  return python_native_fn(who, name, fn);
}

// A `who` that lives for good. C++ keeps the const char* a job or call is given (for its errors,
// which can come after the call returns), where C++ passes a string literal; Python's string
// would go. Bounded by the distinct names used.
const char *stable_who(const std::string &who) {
  static std::mutex mu;
  static std::set<std::string> names; // node-based: c_str() stays put
  std::lock_guard<std::mutex> lock(mu);
  return names.insert(who).first->c_str();
}
} // namespace
} // namespace pycvc
%}

// ── enums: pycvc.process_status and pycvc.scheduling_policy are enum.IntEnum classes ──
// pycvc.process_status.killed is process_status::killed. An enum argument must be a member of its
// IntEnum (strict, like values), which is also what lets to_string(process_status) and
// to_string(value) share their C++ name.
%{
static PyObject *pycvc_enum_process_status = nullptr;
static PyObject *pycvc_enum_scheduling_policy = nullptr;

// enum.IntEnum(name, members) in module pycvc, or nullptr with a Python error set.
static PyObject *pycvc_make_int_enum(const char *name,
                                     std::initializer_list<std::pair<const char *, int>> members) {
  PyObject *enum_mod = PyImport_ImportModule("enum");
  PyObject *int_enum = enum_mod ? PyObject_GetAttrString(enum_mod, "IntEnum") : nullptr;
  Py_XDECREF(enum_mod);
  PyObject *pairs = int_enum ? PyList_New(0) : nullptr;
  if (pairs == nullptr) {
    Py_XDECREF(int_enum);
    return nullptr;
  }
  for (const auto &m : members) {
    PyObject *pair = Py_BuildValue("(si)", m.first, m.second);
    if (pair == nullptr || PyList_Append(pairs, pair) != 0) {
      Py_XDECREF(pair);
      Py_DECREF(pairs);
      Py_DECREF(int_enum);
      return nullptr;
    }
    Py_DECREF(pair);
  }
  PyObject *args = Py_BuildValue("(sO)", name, pairs);
  PyObject *kwargs = Py_BuildValue("{s:s}", "module", "pycvc");
  PyObject *cls = args && kwargs ? PyObject_Call(int_enum, args, kwargs) : nullptr;
  Py_XDECREF(args);
  Py_XDECREF(kwargs);
  Py_DECREF(pairs);
  Py_DECREF(int_enum);
  return cls;
}

static bool pycvc_is_enum(PyObject *o, PyObject *type) {
  const int is = type != nullptr ? PyObject_IsInstance(o, type) : 0;
  if (is < 0)
    PyErr_Clear();
  return is == 1;
}

// An enum argument's value: `o` must be a member of `type`. False, with a TypeError set, otherwise.
static bool pycvc_enum_from_py(PyObject *o, PyObject *type, const char *name, int &out) {
  if (!pycvc_is_enum(o, type)) {
    PyErr_Format(PyExc_TypeError, "pycvc: expected a pycvc.%s member, got %.200s", name,
                 Py_TYPE(o)->tp_name);
    return false;
  }
  const long v = PyLong_AsLong(o);
  if (v == -1 && PyErr_Occurred())
    return false;
  out = static_cast<int>(v);
  return true;
}
%}

%define PYCVC_STATE_EXEC_ENUM(TYPE, PYTYPE, NAME)
%typemap(in) TYPE (int pycvc_ev) {
  if (!pycvc_enum_from_py($input, PYTYPE, NAME, pycvc_ev))
    SWIG_fail;
  $1 = static_cast<TYPE>(pycvc_ev);
}
%typemap(out) TYPE {
  $result = PyObject_CallFunction(PYTYPE, "i", static_cast<int>($1));
  if (!$result)
    SWIG_fail;
}
%typemap(typecheck, precedence=SWIG_TYPECHECK_INT32) TYPE {
  $1 = pycvc_is_enum($input, PYTYPE) ? 1 : 0;
}
// By const reference too, for a %naturalvar member of an enum type SWIG wasn't shown
// (resource_policy::enforce).
%typemap(in) const TYPE & (TYPE pycvc_ev_val, int pycvc_ev) {
  if (!pycvc_enum_from_py($input, PYTYPE, NAME, pycvc_ev))
    SWIG_fail;
  pycvc_ev_val = static_cast<TYPE>(pycvc_ev);
  $1 = &pycvc_ev_val;
}
%typemap(out) const TYPE & {
  $result = PyObject_CallFunction(PYTYPE, "i", static_cast<int>(*$1));
  if (!$result)
    SWIG_fail;
}
%enddef
PYCVC_STATE_EXEC_ENUM(cvc::state_exec::process_status, pycvc_enum_process_status, "process_status")
PYCVC_STATE_EXEC_ENUM(cvc::state_exec::scheduling_policy, pycvc_enum_scheduling_policy,
                      "scheduling_policy")

%init %{
  pycvc_enum_process_status = pycvc_make_int_enum(
      "process_status",
      {{"ready", static_cast<int>(cvc::state_exec::process_status::ready)},
       {"running", static_cast<int>(cvc::state_exec::process_status::running)},
       {"paused", static_cast<int>(cvc::state_exec::process_status::paused)},
       {"waiting", static_cast<int>(cvc::state_exec::process_status::waiting)},
       {"terminated", static_cast<int>(cvc::state_exec::process_status::terminated)},
       {"killed", static_cast<int>(cvc::state_exec::process_status::killed)},
       {"failed", static_cast<int>(cvc::state_exec::process_status::failed)}});
  if (pycvc_enum_process_status != NULL)
    PyDict_SetItemString(d, "process_status", pycvc_enum_process_status);
  pycvc_enum_scheduling_policy = pycvc_make_int_enum(
      "scheduling_policy",
      {{"round_robin", static_cast<int>(cvc::state_exec::scheduling_policy::round_robin)},
       {"priority", static_cast<int>(cvc::state_exec::scheduling_policy::priority)},
       {"priority_rr", static_cast<int>(cvc::state_exec::scheduling_policy::priority_rr)}});
  if (pycvc_enum_scheduling_policy != NULL)
    PyDict_SetItemString(d, "scheduling_policy", pycvc_enum_scheduling_policy);
%}
%pythoncode %{
process_status = _pycvc.process_status
scheduling_policy = _pycvc.scheduling_policy
%}

// pycvc.thread_class (pycvc.set_thread_class, docs/STATE_EXEC_LOCKS.md 7.6) is an IntEnum too.
%{
static PyObject *pycvc_enum_thread_class = nullptr;
%}
PYCVC_STATE_EXEC_ENUM(cvc::thread_class, pycvc_enum_thread_class, "thread_class")
%init %{
  pycvc_enum_thread_class = pycvc_make_int_enum(
      "thread_class", {{"background", static_cast<int>(cvc::thread_class::background)},
                       {"utility", static_cast<int>(cvc::thread_class::utility)},
                       {"normal", static_cast<int>(cvc::thread_class::normal)}});
  if (pycvc_enum_thread_class != NULL)
    PyDict_SetItemString(d, "thread_class", pycvc_enum_thread_class);
%}
%pythoncode %{
thread_class = _pycvc.thread_class
%}

// eval_phase, step_kind, and resource_policy::mode (pycvc.resource_policy.mode, set on the class
// below).
%{
static PyObject *pycvc_enum_eval_phase = nullptr;
static PyObject *pycvc_enum_step_kind = nullptr;
static PyObject *pycvc_enum_resource_policy_mode = nullptr;
%}
PYCVC_STATE_EXEC_ENUM(cvc::state_exec::eval_phase, pycvc_enum_eval_phase, "eval_phase")
PYCVC_STATE_EXEC_ENUM(cvc::state_exec::step_kind, pycvc_enum_step_kind, "step_kind")
PYCVC_STATE_EXEC_ENUM(cvc::state_exec::resource_policy::mode, pycvc_enum_resource_policy_mode,
                      "resource_policy.mode")
%init %{
  {
    using cvc::state_exec::eval_phase;
    pycvc_enum_eval_phase = pycvc_make_int_enum(
        "eval_phase",
        {{"init", static_cast<int>(eval_phase::init)},
         {"eval_args", static_cast<int>(eval_phase::eval_args)},
         {"apply", static_cast<int>(eval_phase::apply)},
         {"if_cond", static_cast<int>(eval_phase::if_cond)},
         {"if_branch", static_cast<int>(eval_phase::if_branch)},
         {"begin_next", static_cast<int>(eval_phase::begin_next)},
         {"while_cond", static_cast<int>(eval_phase::while_cond)},
         {"while_body", static_cast<int>(eval_phase::while_body)},
         {"for_body", static_cast<int>(eval_phase::for_body)},
         {"set_value", static_cast<int>(eval_phase::set_value)},
         {"let_bindings", static_cast<int>(eval_phase::let_bindings)},
         {"let_body", static_cast<int>(eval_phase::let_body)},
         {"return_value", static_cast<int>(eval_phase::return_value)},
         {"eval_inner", static_cast<int>(eval_phase::eval_inner)},
         {"super_args", static_cast<int>(eval_phase::super_args)},
         {"defclass_methods", static_cast<int>(eval_phase::defclass_methods)},
         {"yield_value", static_cast<int>(eval_phase::yield_value)},
         {"break_unwind", static_cast<int>(eval_phase::break_unwind)},
         {"try_body", static_cast<int>(eval_phase::try_body)},
         {"try_handler", static_cast<int>(eval_phase::try_handler)},
         {"try_finally", static_cast<int>(eval_phase::try_finally)},
         {"raise_value", static_cast<int>(eval_phase::raise_value)},
         {"await_call", static_cast<int>(eval_phase::await_call)},
         {"lock_enter", static_cast<int>(eval_phase::lock_enter)},
         {"lock_body", static_cast<int>(eval_phase::lock_body)}});
    if (pycvc_enum_eval_phase != NULL)
      PyDict_SetItemString(d, "eval_phase", pycvc_enum_eval_phase);
    using cvc::state_exec::step_kind;
    pycvc_enum_step_kind =
        pycvc_make_int_enum("step_kind", {{"eval", static_cast<int>(step_kind::eval)},
                                          {"read", static_cast<int>(step_kind::read)},
                                          {"data", static_cast<int>(step_kind::data)},
                                          {"async", static_cast<int>(step_kind::async)}});
    if (pycvc_enum_step_kind != NULL)
      PyDict_SetItemString(d, "step_kind", pycvc_enum_step_kind);
    using rp_mode = cvc::state_exec::resource_policy::mode;
    pycvc_enum_resource_policy_mode =
        pycvc_make_int_enum("mode", {{"strict", static_cast<int>(rp_mode::strict)},
                                     {"clamp", static_cast<int>(rp_mode::clamp)},
                                     {"warn", static_cast<int>(rp_mode::warn)}});
    if (pycvc_enum_resource_policy_mode != NULL)
      PyDict_SetItemString(d, "_resource_policy_mode", pycvc_enum_resource_policy_mode);
  }
%}
%pythoncode %{
eval_phase = _pycvc.eval_phase
step_kind = _pycvc.step_kind
%}

// ── parse_error: pycvc.parse_error, a RuntimeError with .line and .col ──
%{
static PyObject *pycvc_parse_error_type = nullptr;

static void pycvc_raise_parse_error(const cvc::state_exec::parse_error &e) {
  PyObject *inst = PyObject_CallFunction(pycvc_parse_error_type, "s", e.what());
  if (inst == nullptr)
    return;
  PyObject *line = PyLong_FromSize_t(e.line());
  PyObject *col = PyLong_FromSize_t(e.col());
  if (line != nullptr)
    PyObject_SetAttrString(inst, "line", line);
  if (col != nullptr)
    PyObject_SetAttrString(inst, "col", col);
  Py_XDECREF(line);
  Py_XDECREF(col);
  PyErr_SetObject(pycvc_parse_error_type, inst);
  Py_DECREF(inst);
}
%}
%init %{
  pycvc_parse_error_type = PyErr_NewException("pycvc.parse_error", PyExc_RuntimeError, NULL);
  if (pycvc_parse_error_type != NULL)
    PyDict_SetItemString(d, "parse_error", pycvc_parse_error_type);
%}
%pythoncode %{
parse_error = _pycvc.parse_error
%}

// ── evaluation_interrupted / evaluation_timeout: RuntimeError subclasses, as in C++ ──
%{
static PyObject *pycvc_evaluation_interrupted_type = nullptr;
static PyObject *pycvc_evaluation_timeout_type = nullptr;
%}
%init %{
  pycvc_evaluation_interrupted_type =
      PyErr_NewException("pycvc.evaluation_interrupted", PyExc_RuntimeError, NULL);
  if (pycvc_evaluation_interrupted_type != NULL)
    PyDict_SetItemString(d, "evaluation_interrupted", pycvc_evaluation_interrupted_type);
  pycvc_evaluation_timeout_type =
      PyErr_NewException("pycvc.evaluation_timeout", PyExc_RuntimeError, NULL);
  if (pycvc_evaluation_timeout_type != NULL)
    PyDict_SetItemString(d, "evaluation_timeout", pycvc_evaluation_timeout_type);
%}
%pythoncode %{
evaluation_interrupted = _pycvc.evaluation_interrupted
evaluation_timeout = _pycvc.evaluation_timeout
%}

// ── pycvc.dsl_error and pycvc.dsl_class (docs/STATE_EXEC_EXCEPTIONS.md 2.6) ──
%{
static PyObject *pycvc_dsl_error_factory = nullptr; // pycvc._make_dsl_error, set at import
%}
%inline %{
// Set by pycvc at import: the function that builds a raised pycvc.dsl_error.
void _set_dsl_error_factory(PyObject *factory) {
  Py_XINCREF(factory);
  Py_XDECREF(pycvc_dsl_error_factory);
  pycvc_dsl_error_factory = factory;
}
// The class chain of a built-in exception class, most derived first; [] for any other name.
PyObject *_builtin_exception_chain(const std::string &name) {
  const std::vector<std::string> chain =
      cvc::state_exec::class_chain(cvc::state_exec::exception_class(name));
  PyObject *list = PyList_New(static_cast<Py_ssize_t>(chain.size()));
  for (std::size_t i = 0; list != nullptr && i < chain.size(); ++i) {
    PyObject *s = PyUnicode_FromString(chain[i].c_str());
    if (s == nullptr) {
      Py_CLEAR(list);
      break;
    }
    PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), s);
  }
  return list;
}
%}
%pythoncode %{
class dsl_error(RuntimeError):
    """A DSL exception that reached Python (docs/STATE_EXEC_EXCEPTIONS.md 2.6).

    Raised by Exec.run, the evaluators' evaluate/run/step and a scheduler's raise_exit_exception.
    str() is "Class: message". Its type also derives from dsl_class(name) for every class in the
    exception's chain, so ``except pycvc.dsl_class("ValueError")`` catches a DSL ValueError and
    any class derived from it.

    Attributes: ``value`` (the exception, a dict), ``dsl_class`` (its class name), ``dsl_classes``
    (the chain, most derived first), ``origin`` and ``traceback`` (a list of strings, innermost
    last).
    """

    value = None
    dsl_classes = ()
    origin = ""
    traceback = ()

    @property
    def dsl_class(self):
        return self.dsl_classes[0] if self.dsl_classes else ""


_dsl_classes = {}
_dsl_chain_types = {}


def dsl_class(name):
    """The Python exception class for the DSL class ``name``, a dsl_error subclass.

    A built-in class derives from its parent's (ValueError from Exception); any other derives from
    dsl_error, and a raised exception's type derives from every class in its chain.
    """
    cls = _dsl_classes.get(name)
    if cls is None:
        chain = _pycvc._builtin_exception_chain(name)
        base = dsl_class(chain[1]) if len(chain) > 1 else dsl_error
        cls = type(name, (base,), {"__module__": "pycvc.dsl"})
        _dsl_classes[name] = cls
    return cls


def _dsl_error_type(chain):
    key = tuple(chain)
    t = _dsl_chain_types.get(key)
    if t is None:
        bases = tuple(dsl_class(n) for n in key)
        # A base another one already derives from would make the MRO inconsistent.
        bases = tuple(b for b in bases if not any(o is not b and issubclass(o, b) for o in bases))
        if not bases:
            t = dsl_error
        elif len(bases) == 1:
            t = bases[0]
        else:
            t = type(key[0], bases, {"__module__": "pycvc.dsl"})
        _dsl_chain_types[key] = t
    return t


def _make_dsl_error(message, value, chain, origin, traceback):
    e = _dsl_error_type(chain)(message)
    e.value = value
    e.dsl_classes = list(chain)
    e.origin = origin
    e.traceback = list(traceback or ())
    return e


_pycvc._set_dsl_error_factory(_make_dsl_error)
%}

// ── exceptions.h: classes and exception values (docs/STATE_EXEC_EXCEPTIONS.md 2.2, 2.3) ──
// Class dicts and exceptions cross as Python dicts, which are copies. A class dict that came back
// from Python still matches the one it was copied from: is_instance matches a class by its chain
// of names when the dicts differ. Where C++ takes a class (a dict_ptr), Python passes a class dict,
// a class constructor (a pycvc.value), or the name of a built-in class.
%{
namespace pycvc {
// The class `o` names, or null with a Python error set.
static cvc::state_exec::dict_ptr class_arg(PyObject *o, const char *who) {
  namespace se = cvc::state_exec;
  se::value_t v;
  if (!py_to_value(o, v))
    return nullptr;
  if (const auto *s = std::get_if<std::string>(&v.v)) {
    if (se::dict_ptr cls = se::exception_class(*s))
      return cls;
    PyErr_Format(PyExc_KeyError, "%s: no built-in exception class %s", who, s->c_str());
    return nullptr;
  }
  if (const auto *d = std::get_if<se::dict_ptr>(&v.v); d && *d)
    return *d;
  if (se::dict_ptr cls = se::class_of_constructor(v))
    return cls;
  PyErr_Format(PyExc_TypeError, "%s: expected a class dict, a class constructor or a class name",
               who);
  return nullptr;
}

// A new list of `items` as str, or null with a Python error set.
static PyObject *string_list(const std::vector<std::string> &items) {
  PyObject *list = PyList_New(static_cast<Py_ssize_t>(items.size()));
  for (std::size_t i = 0; list != nullptr && i < items.size(); ++i) {
    PyObject *s = PyUnicode_FromString(items[i].c_str());
    if (s == nullptr) {
      Py_CLEAR(list);
      break;
    }
    PyList_SET_ITEM(list, static_cast<Py_ssize_t>(i), s);
  }
  return list;
}

// `cls` as a Python dict, or None.
static PyObject *class_or_none(const cvc::state_exec::dict_ptr &cls) {
  if (!cls)
    Py_RETURN_NONE;
  return value_to_py(cvc::state_exec::value_t(cls));
}
} // namespace pycvc
%}
%rename(exception_class) pycvc_exception_class;
%rename(exception_class_names) pycvc_exception_class_names;
%rename(make_exception) pycvc_make_exception;
%rename(class_of_instance) pycvc_class_of_instance;
%rename(class_of_constructor) pycvc_class_of_constructor;
%rename(is_instance) pycvc_is_instance;
%rename(class_name) pycvc_class_name;
%rename(class_chain) pycvc_class_chain;
%inline %{
// The built-in exception class `name` (a dict), or None.
PyObject *pycvc_exception_class(const std::string &name) {
  return pycvc::class_or_none(cvc::state_exec::exception_class(name));
}
// The built-in exception class names, parents before children.
PyObject *pycvc_exception_class_names() {
  return pycvc::string_list(cvc::state_exec::exception_class_names());
}
// A new exception of class `cls`, with `message`, `data` (None: nil) and `origin`.
PyObject *pycvc_make_exception(PyObject *cls, const std::string &message, PyObject *data = NULL,
                               const std::string &origin = "dsl") {
  namespace se = cvc::state_exec;
  const se::dict_ptr c = pycvc::class_arg(cls, "make_exception");
  if (!c)
    return nullptr;
  se::value_t d;
  if (data != nullptr && data != Py_None && !pycvc::py_to_value(data, d))
    return nullptr;
  return pycvc::value_to_py(se::make_exception(c, message, d, origin));
}
// The class dict of the instance `v`, or None.
PyObject *pycvc_class_of_instance(PyObject *v) {
  cvc::state_exec::value_t value;
  if (!pycvc::py_to_value(v, value))
    return nullptr;
  return pycvc::class_or_none(cvc::state_exec::class_of_instance(value));
}
// The class dict a class constructor makes instances of, or None.
PyObject *pycvc_class_of_constructor(PyObject *ctor) {
  cvc::state_exec::value_t value;
  if (!pycvc::py_to_value(ctor, value))
    return nullptr;
  return pycvc::class_or_none(cvc::state_exec::class_of_constructor(value));
}
// Whether `v` is an instance of `cls` or of a class derived from it.
PyObject *pycvc_is_instance(PyObject *v, PyObject *cls) {
  cvc::state_exec::value_t value;
  if (!pycvc::py_to_value(v, value))
    return nullptr;
  const cvc::state_exec::dict_ptr c = pycvc::class_arg(cls, "is_instance");
  if (!c)
    return nullptr;
  return PyBool_FromLong(cvc::state_exec::is_instance(value, c) ? 1 : 0);
}
// A class's name ("" for a class dict without one).
PyObject *pycvc_class_name(PyObject *cls) {
  const cvc::state_exec::dict_ptr c = pycvc::class_arg(cls, "class_name");
  if (!c)
    return nullptr;
  const std::string name = cvc::state_exec::class_name(c);
  return PyUnicode_FromStringAndSize(name.data(), static_cast<Py_ssize_t>(name.size()));
}
// The class names from `cls` up its __super__ chain, most derived first.
PyObject *pycvc_class_chain(PyObject *cls) {
  const cvc::state_exec::dict_ptr c = pycvc::class_arg(cls, "class_chain");
  if (!c)
    return nullptr;
  return pycvc::string_list(cvc::state_exec::class_chain(c));
}
%}

// ── no value: KeyError ──
// Where C++ returns an empty std::optional<value_t> or a null value_t* (an unbound name, a pid with
// no result, an empty message queue), the wrapper throws pycvc::no_value and Python gets
// KeyError(key). It can't return None: None is the nil value.
%{
namespace pycvc {
struct no_value {
  std::variant<int, std::string> key;
};
} // namespace pycvc

static void pycvc_raise_no_value(const pycvc::no_value &e) {
  PyObject *key = nullptr;
  if (const int *pid = std::get_if<int>(&e.key)) {
    key = PyLong_FromLong(*pid);
  } else {
    const std::string &s = std::get<std::string>(e.key);
    key = PyUnicode_DecodeUTF8(s.data(), static_cast<Py_ssize_t>(s.size()), "surrogateescape");
  }
  if (key != nullptr) {
    PyErr_SetObject(PyExc_KeyError, key);
    Py_DECREF(key);
  }
}
%}

// ── DSL exceptions reaching Python (docs/STATE_EXEC_EXCEPTIONS.md 2.6) ──
// A dsl_exception (an evaluator's uncaught exception, a failed process's) raises pycvc.dsl_error,
// whose type derives from pycvc.dsl_class(name) for each name in the exception's class chain. One
// made from a registered Python function's exception re-raises that exception instead, with the
// dsl_error as its __cause__. The Python side (_make_dsl_error, below) builds the instance.
%{
// Re-raise a Python function's exception that crossed the DSL; `cause` (a new reference, or
// null) becomes its __cause__.
static void pycvc_reraise_python(const pycvc::python_exception &e, PyObject *cause) {
  PyObject *original = e.exception();
  if (original == nullptr) {
    Py_XDECREF(cause);
    PyErr_SetString(PyExc_RuntimeError, e.what());
    return;
  }
  if (cause != nullptr)
    PyException_SetCause(original, cause); // steals cause
  PyErr_SetObject(reinterpret_cast<PyObject *>(Py_TYPE(original)), original);
}

// The pycvc.dsl_error for `exc`: a new reference, or null with a Python error set.
static PyObject *pycvc_make_dsl_error(const cvc::state_exec::value_t &exc) {
  namespace se = cvc::state_exec;
  const std::string summary = se::exception_summary(exc);
  if (pycvc_dsl_error_factory == nullptr) {
    PyErr_SetString(PyExc_RuntimeError, summary.c_str());
    return nullptr;
  }
  PyObject *value = pycvc::value_to_py(exc);
  if (value == nullptr)
    return nullptr;
  const std::vector<std::string> chain = se::class_chain(se::class_of_instance(exc));
  PyObject *names = PyList_New(static_cast<Py_ssize_t>(chain.size()));
  for (std::size_t i = 0; names != nullptr && i < chain.size(); ++i) {
    PyObject *name = PyUnicode_FromString(chain[i].c_str());
    if (name == nullptr) {
      Py_CLEAR(names);
      break;
    }
    PyList_SET_ITEM(names, static_cast<Py_ssize_t>(i), name);
  }
  const se::value_t *tb = se::field(exc, "traceback");
  PyObject *traceback = tb != nullptr ? pycvc::value_to_py(*tb) : (Py_INCREF(Py_None), Py_None);
  const se::value_t *origin = se::field(exc, "origin");
  const std::string *origin_s = origin ? std::get_if<std::string>(&origin->v) : nullptr;
  PyObject *inst = nullptr;
  if (names != nullptr && traceback != nullptr)
    inst = PyObject_CallFunction(pycvc_dsl_error_factory, "sOOsO", summary.c_str(), value, names,
                                 origin_s ? origin_s->c_str() : "", traceback);
  Py_DECREF(value);
  Py_XDECREF(names);
  Py_XDECREF(traceback);
  return inst;
}

// Raise a DSL exception that reached Python.
static void pycvc_raise_dsl_exception(const cvc::state_exec::value_t &exc,
                                      std::exception_ptr original) {
  PyObject *inst = pycvc_make_dsl_error(exc);
  if (inst == nullptr)
    return;
  if (original) {
    try {
      std::rethrow_exception(original);
    } catch (const pycvc::python_exception &e) {
      pycvc_reraise_python(e, inst);
      return;
    } catch (...) {
    }
  }
  PyErr_SetObject(reinterpret_cast<PyObject *>(Py_TYPE(inst)), inst);
  Py_DECREF(inst);
}
%}

// For the state_exec declarations (and Exec, below): the default mapping, with
// parse_error first so it raises pycvc.parse_error. Scoped to this section, so
// the modules that %import pycvc.i keep PYCVC_DEFAULT_EXCEPTION.
%define PYCVC_STATE_EXEC_CATCH
  catch (const pycvc::no_value &e) {
    pycvc_raise_no_value(e);
    SWIG_fail;
  } catch (const pycvc::python_exception &e) {
    pycvc_reraise_python(e, nullptr);
    SWIG_fail;
  } catch (const cvc::state_exec::dsl_exception &e) {
    pycvc_raise_dsl_exception(e.value(), e.original());
    SWIG_fail;
  } catch (const cvc::state_exec::dsl_error &e) {
    pycvc_raise_dsl_exception(cvc::state_exec::exception_from(e), nullptr);
    SWIG_fail;
  } catch (const cvc::state_exec::parse_error &e) {
    pycvc_raise_parse_error(e);
    SWIG_fail;
  } catch (const cvc::state_exec::evaluation_interrupted &e) {
    PyErr_SetString(pycvc_evaluation_interrupted_type, e.what());
    SWIG_fail;
  } catch (const cvc::state_exec::evaluation_timeout &e) {
    PyErr_SetString(pycvc_evaluation_timeout_type, e.what());
    SWIG_fail;
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "pycvc: C++ exception (see libcvc)");
  }
%enddef
%exception {
  try {
    $action
  }
  PYCVC_STATE_EXEC_CATCH
}

// Stepping and running a scheduler release the GIL around the C++ call (pycvc::gil_release, from
// pycvc_lod.i; a throw restores it before the catch arms run), so other Python threads keep going
// while DSL programs run. A Python function a program calls takes the GIL back for its call.
%define PYCVC_STATE_EXEC_NOGIL(NAME)
%exception NAME {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  }
  PYCVC_STATE_EXEC_CATCH
}
%enddef
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::scheduler::step)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::scheduler::run)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::async_scheduler::sync_step)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::async_scheduler::sync_run)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::async_scheduler::wait_for_work)
// The evaluators, likewise.
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::stackless_evaluator::step)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::stackless_evaluator::run)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::stackless_evaluator::evaluate)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::stackless_evaluator::evaluate_script)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::evaluator::evaluate)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::evaluator::evaluate_script)
// And the long walks: a job run to completion, a heap measurement, a cycle collection.
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::run_to_completion)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::measure_heap_share)
PYCVC_STATE_EXEC_NOGIL(cvc::state_exec::collect_cycles)

// Shared as in C++ (environment_ptr, process_ptr, ...). Struct members that are maps, lists or
// values read and write as Python values (copies).
%shared_ptr(cvc::state_exec::environment)
%shared_ptr(cvc::state_exec::script_reader)
%shared_ptr(cvc::state_exec::process)
%shared_ptr(cvc::state_exec::intrinsics_context)
// A data job is owned (std::unique_ptr): Python owns one it is handed, and gives it up to
// run_data_job.
%include <std_unique_ptr.i>
%unique_ptr(cvc::state_exec::data_job)
%naturalvar cvc::state_exec::value_tag;
%naturalvar cvc::state_exec::environment::bindings;
%naturalvar cvc::state_exec::execute_options::signal_handlers;
%naturalvar cvc::state_exec::eval_frame::results;
%naturalvar cvc::state_exec::eval_frame::extra_vals;
%naturalvar cvc::state_exec::eval_frame::extra_strs;
%naturalvar cvc::state_exec::eval_frame::extra_lists;
%naturalvar cvc::state_exec::evaluator_state::stack;
%naturalvar cvc::state_exec::evaluator_state::user_macros;
%naturalvar cvc::state_exec::process::recv_path;
%naturalvar cvc::state_exec::process::exit_error;
%naturalvar cvc::state_exec::process_info::exit_error;
%naturalvar cvc::state_exec::process::signal_handlers;
%naturalvar cvc::state_exec::process::pending_signals;
%naturalvar cvc::state_exec::process::saved_stack;
%naturalvar cvc::state_exec::process::pending_watch_events;
%naturalvar cvc::state_exec::process::watch_handlers;
%naturalvar cvc::state_exec::resource_policy::enforce;
%naturalvar cvc::state_exec::channel_policy::declared;
%naturalvar cvc::state_exec::channel_policy::global;
// ── async_lane (cvc/core/async_lane.h): a persistent worker thread, as an async host's executor
// (async_host.add_lane). stop() and the destructor wait for the lane's jobs: without the GIL, which
// a Python kernel on the lane needs to finish. ──
%{
#include <cvc/core/async_lane.h>
%}
PYCVC_STATE_EXEC_NOGIL(cvc::async_lane::stop)
PYCVC_STATE_EXEC_NOGIL(cvc::async_lane::~async_lane)
namespace cvc {
class async_lane {
public:
  explicit async_lane(std::string name);
  ~async_lane();
  void stop();
  const std::string &name() const;
  bool threaded() const;
  bool stopped() const;
  size_t pending() const;
  size_t in_flight() const;
  unsigned long long completed() const;
  unsigned long long failed() const;
  std::string last_error() const;
  double last_job_ms() const;
};
} // namespace cvc

// ── thread classes (cvc/core/thread_class.h, docs/STATE_EXEC_LOCKS.md 7.6) ──
// pycvc.set_thread_class(cls) puts the calling thread in a pycvc.thread_class (background, utility,
// normal), as far as the OS allows, and returns whether it did. On Linux an unprivileged thread
// can't raise itself back once lowered: that returns False.
%rename(set_thread_class) cvc::set_current_thread_class;
namespace cvc {
enum class thread_class;
bool set_current_thread_class(thread_class cls);
} // namespace cvc

// ── async_call: a Python kernel's handle (register_async_fn(..., with_call=True)) ──
// Shared: the handle keeps the call's record alive. cancelled() says whether to stop (and records
// the CPU time used so far, for the process's budget); post(value) sends progress to
// "<name>.progress"; add_wait(seconds) is time spent waiting on the host, not charged.
%shared_ptr(cvc::state_exec::async_call)
%nodefaultctor cvc::state_exec::async_call;
%{
namespace pycvc {
PyObject *async_call_to_py(std::shared_ptr<cvc::state_exec::async_call> call) {
  static swig_type_info *type = SWIG_TypeQuery("std::shared_ptr< cvc::state_exec::async_call > *");
  if (type == nullptr) {
    PyErr_SetString(PyExc_RuntimeError, "pycvc: the async_call type isn't registered");
    return nullptr;
  }
  return SWIG_NewPointerObj(new std::shared_ptr<cvc::state_exec::async_call>(std::move(call)),
                            type, SWIG_POINTER_OWN);
}
} // namespace pycvc
%}

%rename(global_) cvc::state_exec::channel_policy::global; // `global` is a Python keyword
%rename(raise_) cvc::state_exec::stackless_evaluator::raise; // `raise` is one too
// process::watch_event and process::watch_handler_entry are pycvc.watch_event and
// pycvc.watch_handler_entry (Python has no nested classes here); process.watch_event names them too.
%feature("flatnested") cvc::state_exec::process::watch_event;
%feature("flatnested") cvc::state_exec::process::watch_handler_entry;

namespace cvc {
namespace state_exec {

// symbol: the real struct (name), so a symbol stays a symbol in Python.
struct symbol {
  std::string name;
};
%extend symbol {
  symbol(const std::string &name) { return new cvc::state_exec::symbol{name}; }
  %pythoncode %{
    def __eq__(self, other):
        return isinstance(other, symbol) and self.name == other.name
    def __ne__(self, other):
        return not self.__eq__(other)
    def __hash__(self):
        return hash(("pycvc.symbol", self.name))
    def __repr__(self):
        return "symbol(%r)" % (self.name,)
  %}
}

// value: the real value_t, as an opaque handle. Plain data converts to native
// Python types; a closure, native fn, data object or generator arrives as one of
// these and converts back to the same value. Not constructible from Python.
%rename(value) value_tag;
%nodefaultctor value_tag;
struct value_tag {
  bool is_nil() const;
  bool is_truthy() const;
  std::string type_name() const;
};
%extend value_tag {
  %pythoncode %{
    def __repr__(self):
        return "<pycvc.value %s: %s>" % (self.type_name(), to_string(self))
    def __eq__(self, other):
        try:
            return values_equal(self, other)
        except (TypeError, ValueError, OverflowError):
            return NotImplemented
    def __ne__(self, other):
        r = self.__eq__(other)
        return r if r is NotImplemented else not r
    __hash__ = None
  %}
}
typedef value_tag value_t;

std::string to_string(const value_t &val);
bool values_equal(const value_t &a, const value_t &b);

// Parser (docs/STATE_EXEC_GRAMMAR.md).
struct parse_options {
  size_t max_depth;   // lists and quotes that may nest (default 100000)
  size_t step_tokens; // 0: parse at submit; N: read N tokens per process step
};
struct data_options {
  size_t step_nodes; // 0: deep-data work in the step that needs it; N: N nodes per step
};
parse_options parse_options_from_state(cvc::state &root,
                                       const std::string &scheduler_id = std::string(),
                                       const parse_options &base = parse_options());
data_options data_options_from_state(cvc::state &root,
                                      const std::string &scheduler_id = std::string(),
                                      const data_options &base = data_options());
value_t parse(const std::string &input, const parse_options &opts = parse_options());
std::vector<value_t> parse_all(const std::string &input,
                               const parse_options &opts = parse_options());

// script_reader owns its source, so it is the Python way to read a script a
// slice at a time (parser views a string_view and isn't wrapped). forms(),
// source() and options() return copies here; C++ returns references.
class script_reader {
public:
  script_reader(std::string source, const parse_options &opts);
  bool step(size_t budget);
  std::vector<value_t> forms();
  std::string source() const;
  parse_options options() const;
};

// Snapshot codec for one value: write it under a State node, read it back.
void encode_value(cvc::state &node, const value_t &val);
value_t decode_value(cvc::state &node);

// ── environment and builtins ──
// lookup() returns the bound value, or raises KeyError (C++ returns nullptr). bindings is the
// scope's own map, read and written as a dict (a copy: change bindings with set()). set() on a
// frozen scope raises RuntimeError, as C++ throws std::logic_error.
struct environment {
  std::unordered_map<std::string, value_t> bindings;
  std::shared_ptr<environment> outer;
  bool frozen;
  void set(const std::string &name, value_t val);
  void set_existing(const std::string &name, value_t val);
  static std::shared_ptr<environment> extend(std::shared_ptr<environment> parent);
};
%extend environment {
  value_t lookup(const std::string &name) {
    if (const cvc::state_exec::value_t *v = $self->lookup(name))
      return *v;
    throw pycvc::no_value{name};
  }
}
typedef std::shared_ptr<environment> environment_ptr;

// register_fn takes a Python callable (or a pycvc.value holding a native fn) where C++ takes a
// native_fn. The callable runs when a program calls it, on the thread stepping the scheduler; it
// gets the arguments as Python values and its result converts back, as for Exec.register_fn.
%nodefaultctor builtins;
%nodefaultdtor builtins;
class builtins {
public:
  static environment_ptr make_default_environment();
};
%extend builtins {
  static void register_fn(environment_ptr env, const std::string &name, PyObject *fn) {
    if (!env)
      throw std::invalid_argument("pycvc.builtins.register_fn: env is None");
    cvc::state_exec::builtins::register_fn(
        env, name, pycvc::native_from_py("pycvc.builtins.register_fn", name, fn));
  }
}

std::string percent_encode(const std::string &s);
std::string percent_decode(const std::string &s);

// ── evaluators ──
// evaluation_stats is the C++ class; it can't be copied (it holds a mutex), so the evaluators and
// states hand out references, which keep their owner alive in Python. evaluation_interrupted and
// evaluation_timeout are pycvc exceptions (RuntimeError subclasses). The evaluators release the
// GIL while they evaluate, so another thread can interrupt(), pause() or resume() them.
// step_kind (#658) is an IntEnum: the steps and time are counted by kind.
enum class step_kind;
class evaluation_stats {
public:
  evaluation_stats();
  void start();
  void mark_complete();
  void increment_step();
  void increment_step(step_kind kind);
  void add_steps(step_kind kind, unsigned long long n);
  unsigned long long get_step_count() const;
  unsigned long long steps(step_kind kind) const;
  step_kind last_step_kind() const;
  void add_data_nodes(unsigned long long nodes);
  unsigned long long data_nodes() const;
  void add_step_time(double seconds);
  double step_time(step_kind kind) const;
  bool is_complete() const;
  double elapsed_seconds() const;
};

// eval_phase is the IntEnum above. A frame reads and writes as copies of its parts.
enum class eval_phase;
struct eval_frame {
  value_t expr;
  environment_ptr env;
  eval_phase phase;
  long long index;
  std::vector<value_t> results;
  std::vector<value_t> extra_vals;
  std::vector<std::string> extra_strs;
  std::vector<std::vector<value_t> > extra_lists;
  environment_ptr extra_env;
  bool function_body;
};

// The recursive evaluator. user_macros() returns a copy of the macro map: a dict of name ->
// (params, body).
%pythonappend cvc::state_exec::evaluator::stats %{
  val._pycvc_owner = self
%}
class evaluator {
public:
  typedef std::unordered_map<std::string, std::pair<std::vector<symbol>, value_t> > macro_map;
  evaluator(environment_ptr global_env);
  value_t evaluate(const value_t &expr, environment_ptr env = environment_ptr(),
                   std::optional<double> timeout_sec = std::nullopt,
                   pycvc::value_callback on_complete = pycvc::value_callback());
  value_t evaluate_script(const std::string &script, environment_ptr env = environment_ptr(),
                          std::optional<double> timeout_sec = std::nullopt,
                          pycvc::value_callback on_complete = pycvc::value_callback());
  void interrupt();
  void reset_interrupt();
  void pause();
  void resume();
  bool is_paused() const;
  evaluation_stats &stats();
  macro_map user_macros();
};

// A stackless evaluator's whole state. The members read and write as copies; stats is a reference
// into the state.
%pythonappend cvc::state_exec::evaluator_state::_stats %{
  val._pycvc_owner = self
%}
struct evaluator_state {
  std::vector<eval_frame> stack;
  value_t result;
  bool done;
  value_t root_expr;
  environment_ptr global_env;
  evaluator::macro_map user_macros;
  bool yielded;
  std::shared_ptr<script_reader> reading;
  size_t data_step_nodes;
};
%extend evaluator_state {
  evaluation_stats *_stats() { return &$self->stats; }
  // A heap measurement's result while evaluation waits to resume (#667), or None. Read-only: it
  // belongs to the measurement in flight.
  PyObject *_heap_measured() const {
    if (!$self->heap_measured) {
      Py_INCREF(Py_None);
      return Py_None;
    }
    return PyLong_FromUnsignedLongLong(static_cast<unsigned long long>(*$self->heap_measured));
  }
  // The exception the program didn't catch, once the state is done with one (exceptions P1), or
  // None. Read-only: stepping the state raises it.
  PyObject *_uncaught() const {
    if (!$self->uncaught) {
      Py_INCREF(Py_None);
      return Py_None;
    }
    return pycvc::value_to_py(*$self->uncaught);
  }
  %pythoncode %{
    stats = property(_stats)
    heap_measured = property(_heap_measured)
    uncaught = property(_uncaught)
  %}
}

// evaluator_state can't be copied (its stats hold a mutex), and SWIG returns objects by copy:
// move a returned state into its Python object instead.
%typemap(out) cvc::state_exec::evaluator_state %{
  $result = SWIG_NewPointerObj(new cvc::state_exec::evaluator_state(std::move($1)),
                               $descriptor(cvc::state_exec::evaluator_state *), SWIG_POINTER_OWN);
%}

// The stackless evaluator. on_complete is a Python callable or None. A str passed to
// create_state is a script (the std::string overload), as in C++. raise is raise_ (a Python
// keyword).
class stackless_evaluator {
public:
  stackless_evaluator(environment_ptr global_env);
  evaluator_state create_state(const value_t &expr, environment_ptr env = environment_ptr());
  evaluator_state create_state(const std::string &script, environment_ptr env = environment_ptr(),
                               const parse_options &popts = parse_options());
  bool step(evaluator_state &state);
  value_t run(evaluator_state &state, std::optional<unsigned long long> max_steps = std::nullopt,
              std::optional<double> timeout_sec = std::nullopt,
              pycvc::value_callback on_complete = pycvc::value_callback());
  value_t evaluate(const value_t &expr, environment_ptr env = environment_ptr(),
                   std::optional<double> timeout_sec = std::nullopt,
                   pycvc::value_callback on_complete = pycvc::value_callback());
  value_t evaluate_script(const std::string &script, environment_ptr env = environment_ptr(),
                          std::optional<double> timeout_sec = std::nullopt,
                          pycvc::value_callback on_complete = pycvc::value_callback());
  void interrupt();
  void reset_interrupt();
  void pause();
  void resume();
  bool is_paused() const;
  void restrict_special_forms(std::shared_ptr<const std::set<std::string> > allowed);
  static const std::set<std::string> &all_special_forms();
  environment_ptr global_env() const; // (#667)
  void raise(evaluator_state &state, const value_t &exc); // (exceptions P1)
};

// Snapshot codec for an environment chain and for a whole evaluator state.
void encode_environment(cvc::state &node, const environment_ptr &env);
environment_ptr decode_environment(cvc::state &node);
void encode_evaluator_state(cvc::state &node, const evaluator_state &es);
evaluator_state decode_evaluator_state(cvc::state &node);

// ── processes and schedulers ──
// process_status and scheduling_policy are the IntEnums above; declared opaque here so SWIG adds
// no flat constants of its own.
enum class process_status;
enum class scheduling_policy;
const char *to_string(process_status s);
process_status parse_process_status(const std::string &s);
bool is_finished(process_status s); // terminated, killed or failed (exceptions P2)

// exceptions.h, the declarations that take values as they are (the class helpers are above).
void register_exception_classes(const environment_ptr &env);
bool is_exception(const value_t &v);
std::string exception_message(const value_t &exc);
std::string exception_summary(const value_t &exc);
std::string traceback_line(const value_t &expr);
const size_t kTracebackFrames = 64;

// on_complete is a Python callable (or None) called with the process's result when it terminates,
// on the thread stepping the scheduler. Its return value is ignored; an exception it raises is
// raised from that step()/run() call. The uint64_t limits are Python ints.
struct execute_options {
  std::string name;
  int priority;
  std::string uid;
  std::string gid;
  std::string root_path;
  std::string owner;
  unsigned long long max_steps;
  double max_time;
  double max_step_time;
  unsigned long long max_memory;
  unsigned long long max_messages;
  unsigned long long max_message_bytes;
  std::unordered_map<std::string, value_t> signal_handlers;
  environment_ptr env;
};
%define PYCVC_STATE_EXEC_ON_COMPLETE(CLASS)
%extend CLASS {
  PyObject *_on_complete_get() const {
    if (!$self->on_complete) {
      Py_INCREF(Py_None);
      return Py_None;
    }
    if (const auto *f = $self->on_complete.target<pycvc::python_on_complete>()) {
      Py_INCREF(f->callable.get());
      return f->callable.get();
    }
    throw std::runtime_error("pycvc: on_complete holds a C++ callback");
  }
  void _on_complete_set(PyObject *fn) {
    if (fn == Py_None)
      $self->on_complete = nullptr;
    else
      $self->on_complete = pycvc::make_python_on_complete(fn);
  }
  %pythoncode %{
    on_complete = property(_on_complete_get, _on_complete_set)
  %}
}
%enddef
PYCVC_STATE_EXEC_ON_COMPLETE(execute_options)
// async_steps_per_cpu_ms is a float, or None (unset: the tree's limits.async_steps_per_cpu_ms).
%extend execute_options {
  PyObject *_async_steps_get() const {
    if (!$self->async_steps_per_cpu_ms)
      Py_RETURN_NONE;
    return PyFloat_FromDouble(*$self->async_steps_per_cpu_ms);
  }
  void _async_steps_set(PyObject *rate) {
    if (rate == Py_None) {
      $self->async_steps_per_cpu_ms.reset();
      return;
    }
    const double v = PyFloat_AsDouble(rate);
    if (v == -1.0 && PyErr_Occurred()) {
      PyErr_Clear();
      throw std::invalid_argument("execute_options.async_steps_per_cpu_ms: a number or None");
    }
    $self->async_steps_per_cpu_ms = v;
  }
  %pythoncode %{
    async_steps_per_cpu_ms = property(_async_steps_get, _async_steps_set)
  %}
}

struct process_info {
  int pid;
  std::string name;
  process_status status;
  int priority;
  std::string uid;
  std::string gid;
  unsigned long long step_count;
  double elapsed_time;
  unsigned long long current_memory;
  unsigned long long peak_memory;
  unsigned long long max_memory;
  double max_time;
  unsigned long long message_count;
  unsigned long long max_messages;
  unsigned long long message_bytes;
  unsigned long long max_message_bytes;
  int parent_pid;
  // The budget by kind of step (#658): reading the script, evaluating, deep-data work.
  unsigned long long read_steps;
  unsigned long long eval_steps;
  unsigned long long data_steps;
  unsigned long long data_nodes;
  double read_time;
  double eval_time;
  double data_time;
  // The measured heap share (#667, heap.h), which max_memory covers too.
  unsigned long long heap_memory;
  unsigned long long heap_measurements;
  unsigned long long heap_alloc_bytes;
  // What its async calls were charged (docs/STATE_EXEC_EXCEPTIONS.md 3.8).
  unsigned long long async_calls;
  double async_time;
  double async_cpu_time;
  unsigned long long async_steps;
  // Why it ended, if not normally: the kill reason, or a failed process's "Class: message".
  std::optional<std::string> exit_error;
  // Its holds, and what it waits for: "lock /<key>", "msg <channel>", "sleep" or "frame"
  // (docs/STATE_EXEC_LOCKS.md 5.10).
  unsigned long long locks_held;
  std::optional<std::string> waiting_on;
};
%extend process_info {
  // A failed process's exception (docs/STATE_EXEC_EXCEPTIONS.md 2.6), or None.
  PyObject *_exit_exception() const {
    if (!$self->exit_exception)
      Py_RETURN_NONE;
    return pycvc::value_to_py(*$self->exit_exception);
  }
  %pythoncode %{
    exit_exception = property(_exit_exception)
    def __repr__(self):
        return "<pycvc.process_info pid=%d name=%r status=%s>" % (
            self.pid, self.name, self.status.name)
  %}
}

struct scheduler_stats {
  int total_processes;
  int running;
  int ready;
  int paused;
  int terminated;
  int killed;
  int failed;
  unsigned long long total_steps;
};
%extend scheduler_stats {
  %pythoncode %{
    def __repr__(self):
        return ("<pycvc.scheduler_stats total_processes=%d running=%d ready=%d paused=%d "
                "terminated=%d killed=%d failed=%d total_steps=%d>" % (
                    self.total_processes, self.running, self.ready, self.paused,
                    self.terminated, self.killed, self.failed, self.total_steps))
  %}
}

// ── process: a scheduler's record of one program (shared, as process_ptr) ──
// Its members read and write as copies, except state, a reference into the process. The
// steady_clock time points (create_time, last_run_start, wake_time) aren't wrapped; elapsed_time()
// and accumulated_time give the same information. inbox reads as a list.
const size_t kDefaultMaxAllocBytes = 16777216;
// The per-call allocation limit (#660). A Python function a program calls runs on the thread
// stepping the scheduler, where the step's limit is armed, so it can honour the limit the way a
// builtin does: eval_alloc_check() before building a large result. eval_alloc_guard (RAII, which
// must nest last-in first-out) isn't wrapped.
std::optional<size_t> eval_alloc_limit();
void eval_alloc_check(size_t bytes, const char *who);
size_t alloc_bytes(size_t count, size_t each);
void eval_alloc_note(size_t bytes); // (#667) report bytes allocated, for the heap's re-measures
%pythonappend cvc::state_exec::process::_state %{
  val._pycvc_owner = self
%}
struct process {
  struct watch_event {
    int watch_id;
  };
  struct watch_handler_entry {
    value_t handler;
    std::string path;
    std::string last_value;
  };
  int pid;
  std::string name;
  process_status status;
  int priority;
  std::string uid;
  std::string gid;
  std::string root_path;
  std::string owner;
  bool awaiting_frame;
  unsigned long long max_steps;
  double max_time;
  unsigned long long max_memory;
  unsigned long long max_messages;
  unsigned long long max_message_bytes;
  double max_step_time;
  unsigned long long max_alloc_bytes;
  double accumulated_time;
  double cpu_time_charged;
  std::optional<std::string> recv_path;
  value_t exit_code;
  std::optional<std::string> exit_error;
  std::unordered_map<std::string, value_t> signal_handlers;
  std::vector<std::string> pending_signals;
  bool in_signal_handler;
  bool in_watch_handler;
  std::vector<eval_frame> saved_stack;
  value_t saved_result;
  std::vector<watch_event> pending_watch_events;
  int next_watch_id;
  std::unordered_map<int, watch_handler_entry> watch_handlers;
  unsigned long long message_count;
  unsigned long long message_bytes;
  int parent_pid;
  // The heap share (#667, heap.h).
  unsigned long long heap_bytes;
  unsigned long long heap_alloc_bytes;
  unsigned long long heap_steps;
  unsigned long long heap_measurements;
  size_t heap_check_steps;
  bool heap_measuring;
  // What its async calls were charged (docs/STATE_EXEC_EXCEPTIONS.md 3.8).
  unsigned long long async_calls;
  double async_time;
  double async_cpu_time;
  unsigned long long async_steps;
  bool heap_measure_due(unsigned long long state_bytes) const;
  void release_state();
  double elapsed_time() const;
  bool charges_cpu_time() const;
  void charge_cpu_time(double seconds);
  bool over_time_limit() const;
  std::optional<double> time_left() const;
  std::optional<size_t> alloc_limit(size_t used_bytes) const;
  unsigned long long step_count() const;
};
%extend process {
  evaluator_state *_state() { return &$self->state; }
  // A failed process's exception (docs/STATE_EXEC_EXCEPTIONS.md 2.6), or None. Its exit_original
  // (the C++ exception it was made from) isn't wrapped: raise_exit_exception raises with it.
  PyObject *_exit_exception() const {
    if (!$self->exit_exception)
      Py_RETURN_NONE;
    return pycvc::value_to_py(*$self->exit_exception);
  }
  PyObject *_inbox_get() const {
    std::queue<cvc::state_exec::value_t> q = $self->inbox;
    std::vector<cvc::state_exec::value_t> items;
    for (; !q.empty(); q.pop())
      items.push_back(q.front());
    return pycvc::value_list_to_py(items);
  }
  %pythoncode %{
    state = property(_state)
    inbox = property(_inbox_get)
    exit_exception = property(_exit_exception)
  %}
}
PYCVC_STATE_EXEC_ON_COMPLETE(process)
%pythoncode %{
process.watch_event = watch_event
process.watch_handler_entry = watch_handler_entry
%}
typedef std::shared_ptr<process> process_ptr;
process_ptr make_process();

// The common base of the two schedulers, which intrinsics_context.sched points to. Its members are
// on the two concrete classes.
%nodefaultctor scheduler_base;
%nodefaultdtor scheduler_base;
class scheduler_base {};

// The members scheduler and async_scheduler share (scheduler_base plus their common extras).
// Returns of a C++ reference come back as copies (current_process() shares the process, as
// process_ptr does). Not wrapped: mem_tracker().
%define PYCVC_STATE_EXEC_SCHEDULER_MEMBERS
  process_ptr current_process() const;
  void queue_watch_event(int pid, process::watch_event evt);
  int execute(const std::string &script, const execute_options &opts = execute_options());
  int execute(const value_t &expr, const execute_options &opts = execute_options());
  void stop();
  bool is_running() const;
  bool pause(int pid);
  bool resume(int pid);
  bool kill(int pid);
  int fork(int pid);
  bool sleep(int pid, double seconds);
  bool yield_frame(int pid);
  bool receive_message(int pid, const std::string &path);
  bool receive_message_until(int pid, const std::string &path, double seconds);
  int deliver_to_receivers(const std::string &path, const value_t &msg);
  size_t pending_message_count(const std::string &path) const;
  size_t total_pending_messages() const;
  size_t max_pending_messages;
  bool set_priority(int pid, int priority);
  bool set_max_steps(int pid, unsigned long long max_steps);
  bool set_max_time(int pid, double seconds);
  bool set_max_memory(int pid, unsigned long long bytes);
  bool set_max_messages(int pid, unsigned long long count);
  bool set_max_message_bytes(int pid, unsigned long long bytes);
  bool send_signal(int pid, const std::string &signal);
  std::vector<process_info> list_processes() const;
  std::optional<process_info> get_process_info(int pid) const;
  std::unordered_map<int, value_t> get_results() const;
  std::optional<std::string> get_exit_error(int pid) const;
  void raise_exit_exception(int pid) const;
  scheduler_stats get_stats() const;
  scheduling_policy policy() const;
  int process_count() const;
  bool has_runnable() const;
  int current_pid() const;
  void register_watch_handler(int pid, int watch_id, value_t handler, const std::string &path);
  void unregister_watch_handler(int pid, int watch_id);
  void set_watch_root(cvc::state *root);
  double async_steps_per_cpu_ms(const process &proc);
  int restore(process_ptr proc);
  bool snapshot(int pid, cvc::state &node) const;
  int restore(cvc::state &node, environment_ptr natives = nullptr);
%enddef

// get_result and pop_pending_message raise KeyError(pid / path) where C++ returns std::nullopt.
%define PYCVC_STATE_EXEC_SCHEDULER_EXTEND(CLASS)
%extend CLASS {
  // A failed process's exception (docs/STATE_EXEC_EXCEPTIONS.md 2.6), or None.
  PyObject *get_exit_exception(int pid) const {
    if (std::optional<cvc::state_exec::value_t> e = $self->get_exit_exception(pid))
      return pycvc::value_to_py(*e);
    Py_RETURN_NONE;
  }
  value_t get_result(int pid) const {
    if (std::optional<cvc::state_exec::value_t> r = $self->get_result(pid))
      return *r;
    throw pycvc::no_value{pid};
  }
  value_t pop_pending_message(const std::string &path) {
    if (std::optional<cvc::state_exec::value_t> m = $self->pop_pending_message(path))
      return *m;
    throw pycvc::no_value{path};
  }
}
%enddef

// The synchronous scheduler. run() and step() release the GIL while programs run.
class scheduler : public scheduler_base {
public:
  scheduler();
  scheduler(scheduling_policy policy);
  PYCVC_STATE_EXEC_SCHEDULER_MEMBERS
  int step();
  std::unordered_map<int, value_t> run(std::optional<unsigned long long> max_steps = std::nullopt,
                                       std::optional<double> max_time = std::nullopt);
  void set_id(const std::string &id);
  std::string id() const;
  void load_settings();
  bool reap(int pid);
  parse_options parser_settings() const;
  data_options data_settings() const;
  unsigned long long max_alloc_bytes() const;
  static constexpr const char *settings_root = "state_exec";
  static constexpr const char *defaults_prefix = "state_exec.defaults";
  static constexpr const char *sched_prefix = "state_exec.schedulers";
};
PYCVC_STATE_EXEC_SCHEDULER_EXTEND(scheduler)
unsigned long long max_alloc_bytes_from_state(cvc::state &root, const std::string &scheduler_id,
                                              unsigned long long base = kDefaultMaxAllocBytes);

// The scheduler libcvc's hosts use (app::exec_scheduler(), Exec). The coroutine step()/run() aren't
// wrapped; Python drives it with sync_step()/sync_run(), which release the GIL while programs run.
// post_message() is the one thread-safe call.
class async_scheduler : public scheduler_base {
public:
  async_scheduler();
  async_scheduler(scheduling_policy policy);
  PYCVC_STATE_EXEC_SCHEDULER_MEMBERS
  int sync_step();
  std::unordered_map<int, value_t> sync_run(
      std::optional<unsigned long long> max_steps = std::nullopt,
      std::optional<double> max_time = std::nullopt);
  int kill_owner(const std::string &owner);
  int wake_awaiting();
  void post_message(const std::string &path, const value_t &msg);
  void drain_ingress();
  bool wait_for_work(double seconds);
  size_t awaiting_frame_count() const;
  size_t async_calls_in_flight() const;
  void set_id(const std::string &id);
  std::string id() const;
  bool reap(int pid);
  void set_parser_defaults(const parse_options &defaults);
  parse_options parser_defaults() const;
  static parse_options host_parser_defaults();
  void set_data_defaults(const data_options &defaults);
  data_options data_defaults() const;
  data_options data_settings() const;
  static data_options host_data_defaults();
};
PYCVC_STATE_EXEC_SCHEDULER_EXTEND(async_scheduler)
const double kDefaultAsyncStepsPerCpuMs = 2000.0;
double async_steps_per_cpu_ms_from_state(cvc::state &root, const std::string &scheduler_id);
void publish_async_steps_per_cpu_ms(cvc::state &root, const std::string &scheduler_id,
                                    double rate);

// A process snapshot with its budget so far (docs/STATE_EXEC_EXCEPTIONS.md 3.8), for a
// scheduler's restore().
void encode_process(cvc::state &node, const process &proc);
process_ptr decode_process(cvc::state &node);
void rebind_natives(evaluator_state &st, const environment_ptr &natives);

// ── async functions (docs/STATE_EXEC_EXCEPTIONS.md 3) ──
// async_host() runs kernels inline with no scheduler (a bare evaluator's: its steps only);
// async_host(scheduler) runs kernels inline, charged to the scheduler's processes;
// async_host(async_scheduler, app) parks the calling process while a kernel runs on the app's
// background task pool (without an app, inline). It keeps the scheduler (and app) alive; destroy
// it (or shutdown()) before them. Exec.async_host() is an Exec's. shutdown() and wait_idle()
// release the GIL while they wait. (These directives come before the declaration they apply to.)
// Inside this namespace block, a feature names its declaration relative to it.
%pythonappend async_host::async_host %{
  self._pycvc_refs = args
%}
PYCVC_STATE_EXEC_NOGIL(async_host::shutdown)
PYCVC_STATE_EXEC_NOGIL(async_host::wait_idle)
// Destroying a host shuts it down, which waits for its kernels: without the GIL, which a Python
// kernel needs to finish.
PYCVC_STATE_EXEC_NOGIL(async_host::~async_host)
%nodefaultctor async_host;
// add_lane keeps the lane alive as long as the host: the host's calls need it.
%pythonappend async_host::add_lane %{
  self.__dict__.setdefault("_pycvc_lanes", []).append(lane)
%}
class async_host {
public:
  void shutdown();
  void wait_idle();
  size_t in_flight() const;
  void add_lane(const std::string &name, cvc::async_lane &lane);
};
class async_call {
public:
  const std::string &function_name() const;
  bool cancelled();
  void post(const value_t &progress);
  void add_wait(double seconds);
  bool finished() const;
};
// Whether `v` is a future that held an async call ((name-async ...)'s value).
bool is_async_future(const value_t &v);
%extend async_host {
  async_host() { return new cvc::state_exec::async_host(); }
  async_host(scheduler *sched) { return new cvc::state_exec::async_host(sched); }
  async_host(async_scheduler *sched, std::shared_ptr<cvc::app> app = nullptr) {
    if (!sched)
      throw std::invalid_argument("pycvc.async_host: scheduler is None");
    return new cvc::state_exec::async_host(*sched, app.get());
  }
}

// register_async_fn(env, host, name, fn, executor="background", timeout=None, future_variant=True,
// with_call=False): fn, a Python callable, is the kernel: it runs on an executor thread with copies
// of the arguments, taking the GIL for its call, and its value or exception is the call's
// (docs/STATE_EXEC_EXCEPTIONS.md 3.2). executor is "background" (the app's background task pool;
// "compute" means the same), "inline", or "lane:<name>" (a lane given to host.add_lane); timeout
// is seconds. With with_call, fn gets the call's pycvc.async_call first, before the arguments.
%extend builtins {
  static void register_async_fn(environment_ptr env, cvc::state_exec::async_host &host,
                                const std::string &name, PyObject *fn,
                                const std::string &executor = "background",
                                std::optional<double> timeout = std::nullopt,
                                bool future_variant = true, bool with_call = false) {
    namespace se = cvc::state_exec;
    if (!env)
      throw std::invalid_argument("pycvc.builtins.register_async_fn: env is None");
    se::async_options opts;
    if (executor == "background" || executor == "compute")
      opts.where = se::executor::background();
    else if (executor == "inline")
      opts.where = se::executor::inline_call();
    else if (executor.rfind("lane:", 0) == 0 && executor.size() > 5)
      opts.where = se::executor::on_lane(executor.substr(5));
    else
      throw std::invalid_argument("pycvc.builtins.register_async_fn: executor must be "
                                  "\"background\", \"inline\" or \"lane:<name>\", not \"" +
                                  executor + "\"");
    opts.timeout = timeout;
    opts.future_variant = future_variant;
    se::register_async_fn(
        env, host, name,
        pycvc::python_async_marshal("pycvc.builtins.register_async_fn", name, fn, with_call),
        std::move(opts));
  }
}

// ── resource_policy ──
// mode is pycvc.resource_policy.mode, an IntEnum. validate_limits raises RuntimeError for
// out-of-range limits in strict mode, as C++ throws.
struct resource_policy {
  double max_time_min;
  double max_time_max;
  double max_time_default;
  unsigned long long max_memory_min;
  unsigned long long max_memory_max;
  unsigned long long max_memory_default;
  unsigned long long max_messages_min;
  unsigned long long max_messages_max;
  unsigned long long max_messages_default;
  unsigned long long max_message_bytes_min;
  unsigned long long max_message_bytes_max;
  unsigned long long max_message_bytes_default;
  unsigned long long max_steps_min;
  unsigned long long max_steps_max;
  unsigned long long max_steps_default;
  int max_processes;
  unsigned long long max_total_memory;
  cvc::state_exec::resource_policy::mode enforce;
};
%pythoncode %{
resource_policy.mode = _pycvc._resource_policy_mode
%}
struct process_limits {
  unsigned long long max_steps;
  double max_time;
  unsigned long long max_memory;
  unsigned long long max_messages;
  unsigned long long max_message_bytes;
};
process_limits validate_limits(const resource_policy &policy, const process_limits &requested);

// ── intrinsics ──
// channel_policy.global is global_ (a Python keyword). intrinsics_context is shared; its sched and
// channels are the Python objects assigned to them, which the context keeps alive. Its root is a
// tree node, which lives as long as its app (keep the app alive, as in C++). Not wrapped: tracker
// (memory_tracker is internal), document (document_scope's typed slots are C++ templates), and
// watches (the intrinsics' own registry).
struct channel_policy {
  bool enforce;
  bool quiet;
  std::vector<std::string> declared;
  std::vector<std::string> global;
};
%rename(_sched) intrinsics_context::sched;
%rename(_channels) intrinsics_context::channels;
struct intrinsics_context {
  scheduler_base *sched;
  cvc::state *root;
  std::shared_ptr<process> proc;
  int pid;
  std::string uid;
  std::string cluster_id;
  std::string node_id;
  std::string root_path;
  const channel_policy *channels;
  // with-lock (docs/STATE_EXEC_LOCKS.md 5.5): False makes every busy request as :timeout 0.
  bool lock_wait_allowed;
};
// lock_default_timeout: the timeout of a with-lock that names none, a float, or None (wait as
// long as it takes).
%extend intrinsics_context {
  PyObject *_lock_default_timeout_get() const {
    if (!$self->lock_default_timeout)
      Py_RETURN_NONE;
    return PyFloat_FromDouble(*$self->lock_default_timeout);
  }
  void _lock_default_timeout_set(PyObject *seconds) {
    if (seconds == Py_None) {
      $self->lock_default_timeout.reset();
      return;
    }
    const double v = PyFloat_AsDouble(seconds);
    if (v == -1.0 && PyErr_Occurred()) {
      PyErr_Clear();
      throw std::invalid_argument("intrinsics_context.lock_default_timeout: a number or None");
    }
    $self->lock_default_timeout = v;
  }
  %pythoncode %{
    lock_default_timeout = property(_lock_default_timeout_get, _lock_default_timeout_set)
  %}
}
%extend intrinsics_context {
  %pythoncode %{
    def _get_sched(self):
        return self.__dict__.get("_pycvc_sched") if self._sched is not None else None
    def _set_sched(self, sched):
        self._sched = sched
        self.__dict__["_pycvc_sched"] = sched
    sched = property(_get_sched, _set_sched)
    def _get_channels(self):
        return self.__dict__.get("_pycvc_channels") if self._channels is not None else None
    def _set_channels(self, channels):
        self._channels = channels
        self.__dict__["_pycvc_channels"] = channels
    channels = property(_get_channels, _set_channels)
  %}
}
void apply_chroot(intrinsics_context &ctx, cvc::state &tree_root, const std::string &root_path);
std::string resolve_channel_key(const std::string &root_path, const std::string &channel);
value_t make_future(const std::string &channel);
std::optional<std::string> future_channel_of(const value_t &v);

// ── the heap (#667, heap.h) ──
// A process's heap share, and the per-thread reference-cycle collector, which a Python driver can
// run between scheduler runs on the thread that ran them. Not wrapped: heap_after_step (the
// schedulers' own per-step accounting) and track_captured_env / track_mutated (a value from
// Python is a fresh copy, so tracking it would mean nothing; the evaluator tracks its own).
struct heap_options {
  size_t check_steps;
};
heap_options heap_options_from_state(cvc::state &root,
                                     const std::string &scheduler_id = std::string(),
                                     const heap_options &base = heap_options());
size_t measure_heap_share(const process &proc);
size_t heap_remeasure_bytes(size_t last_measured);
struct cycle_stats {
  size_t candidates;
  size_t examined;
  size_t freed;
};
cycle_stats collect_cycles();
bool cycle_collection_due();
void note_process_exit();
size_t tracked_objects();

// ── deep-data jobs (#658, #665, data_job.h) ──
// A job runs a slice at a time: step(nodes) until it says it's done, then result(). The factories
// hand Python an owned job. From a Python function a program calls (it runs on the stepping
// thread), run_data_job and call_function work as for a C++ native: return what they return at
// once, and the evaluator finishes the call. Not wrapped: the call protocol for C++ job
// implementations (call_request, take_call, pending_call, call_returned, own_inputs) and
// detail::. make_concat_job, run_data_job and call_function are below, in pycvc's own wrappers.
%nodefaultctor data_job;
class data_job {
public:
  virtual ~data_job();
  virtual bool step(size_t nodes) = 0;
  virtual value_t result() = 0;
  size_t nodes() const;
};
const size_t kDataNodeBytes = 256;
// The native-call nesting limits (#682): a level is refused past kMaxNativeNesting, or when less
// than kNativeStackReserve bytes of the thread's stack are left (128 KiB on Windows, 32 KiB
// elsewhere). detail::native_nesting, the RAII level, isn't wrapped.
const size_t kMaxNativeNesting = 256;
const size_t kNativeStackReserve = 131072;
bool is_compound(const value_t &v);
std::unique_ptr<data_job> make_equal_job(value_t a, value_t b);
std::unique_ptr<data_job> make_print_job(value_t v);
std::unique_ptr<data_job> make_copy_job(value_t v);
std::unique_ptr<data_job> make_substitute_job(value_t tmpl,
                                              const std::unordered_map<std::string, value_t> &subst);
std::unique_ptr<data_job> make_heap_share_job(const process &proc);
value_t run_to_completion(data_job &job);
void count_data_nodes(size_t nodes);

// ── stdlib_registry (stdlib.h) ──
// standard() returns a copy (C++ returns a const reference to the shared registry), so Python
// can't add to the one every environment shares. register_function takes a Python callable or a
// pycvc.value holding a native function, as builtins.register_fn does. lookup_qualified returns
// the function as a pycvc.value, or raises KeyError where C++ returns nullptr.
class stdlib_registry {
public:
  stdlib_registry();
  static environment_ptr shared_scope();
  std::vector<std::string> list_functions(const std::string &module) const;
  std::vector<std::string> list_modules() const;
};
%extend stdlib_registry {
  static cvc::state_exec::stdlib_registry standard() {
    return cvc::state_exec::stdlib_registry::standard();
  }
  void register_function(const std::string &module, const std::string &func_name, PyObject *fn) {
    $self->register_function(
        module, func_name,
        pycvc::native_from_py("pycvc.stdlib_registry.register_function", func_name, fn));
  }
  void import_module(const std::string &module, environment_ptr env,
                     const std::vector<std::string> &specific_fns = std::vector<std::string>()) {
    if (!env)
      throw std::invalid_argument("pycvc.stdlib_registry.import_module: env is None");
    $self->import_module(module, env, specific_fns);
  }
  value_t lookup_qualified(const std::string &qualified_name) const {
    if (const cvc::state_exec::native_fn *fn = $self->lookup_qualified(qualified_name))
      return cvc::state_exec::value_t(*fn);
    throw pycvc::no_value{qualified_name};
  }
}
}
}

// make_concat_job, run_data_job and call_function: the C++ calls, except that `who` is kept for
// good (C++ holds the pointer for its errors), and run_data_job's `then` is a Python callable or
// None.
%{
namespace pycvc {
std::unique_ptr<cvc::state_exec::data_job>
make_concat_job_kept(const std::vector<cvc::state_exec::value_t> &parts, const std::string &sep,
                     const std::string &who) {
  return cvc::state_exec::make_concat_job(parts, sep, stable_who(who));
}

cvc::state_exec::value_t run_data_job_then(std::unique_ptr<cvc::state_exec::data_job> job,
                                           PyObject *then = nullptr) {
  if (!job)
    throw std::invalid_argument("pycvc.run_data_job: job is None");
  std::function<cvc::state_exec::value_t(cvc::state_exec::value_t)> after;
  if (then != nullptr && then != Py_None) {
    cvc::state_exec::native_fn fn = python_native_fn("pycvc.run_data_job", "then", then);
    after = [fn](cvc::state_exec::value_t v) {
      return fn(std::span<const cvc::state_exec::value_t>(&v, 1));
    };
  }
  return cvc::state_exec::run_data_job(std::move(job), std::move(after));
}

cvc::state_exec::value_t call_function_kept(const cvc::state_exec::value_t &fn,
                                            const std::vector<cvc::state_exec::value_t> &args,
                                            const std::string &who, PyObject *then = nullptr) {
  const std::span<const cvc::state_exec::value_t> argv(args);
  if (then == nullptr || then == Py_None)
    return cvc::state_exec::call_function(fn, argv, stable_who(who));
  // The overload with a continuation (#683): `then` runs on the call's value once the function
  // has finished, however the evaluator ran it.
  cvc::state_exec::native_fn after = python_native_fn("pycvc.call_function", "then", then);
  return cvc::state_exec::call_function(
      fn, argv, stable_who(who), [after](cvc::state_exec::value_t v) {
        return after(std::span<const cvc::state_exec::value_t>(&v, 1));
      });
}

// make_generator_job takes the generator as a pycvc.value (C++ takes its generator_ptr).
std::unique_ptr<cvc::state_exec::data_job> make_generator_job_from(const cvc::state_exec::value_t &gen,
                                                                   bool collect = false) {
  const auto *g = std::get_if<cvc::state_exec::generator_ptr>(&gen.v);
  if (g == nullptr || !*g)
    throw std::invalid_argument("pycvc.make_generator_job: not a generator");
  return cvc::state_exec::make_generator_job(*g, collect);
}
} // namespace pycvc
%}
%rename(make_concat_job) pycvc::make_concat_job_kept;
%rename(run_data_job) pycvc::run_data_job_then;
%rename(call_function) pycvc::call_function_kept;
%rename(make_generator_job) pycvc::make_generator_job_from;
// One wrapper each, defaults filled in, rather than an overload per default (a value list has no
// overload typecheck).
%feature("compactdefaultargs") pycvc::call_function_kept;
%feature("compactdefaultargs") pycvc::make_generator_job_from;
namespace pycvc {
std::unique_ptr<cvc::state_exec::data_job>
make_concat_job_kept(const std::vector<cvc::state_exec::value_t> &parts, const std::string &sep,
                     const std::string &who);
cvc::state_exec::value_t run_data_job_then(std::unique_ptr<cvc::state_exec::data_job> job,
                                           PyObject *then = nullptr);
cvc::state_exec::value_t call_function_kept(const cvc::state_exec::value_t &fn,
                                            const std::vector<cvc::state_exec::value_t> &args,
                                            const std::string &who, PyObject *then = nullptr);
std::unique_ptr<cvc::state_exec::data_job> make_generator_job_from(const cvc::state_exec::value_t &gen,
                                                                   bool collect = false);
}

// register_intrinsics(env, ctx): the C++ call, except that every function it adds keeps ctx (and
// so the scheduler and channel policy ctx holds) alive. C++ requires the caller to keep the context
// valid instead; in Python an environment may outlive any object.
%{
namespace pycvc {
void register_intrinsics_keeping(cvc::state_exec::environment_ptr env,
                                 std::shared_ptr<cvc::state_exec::intrinsics_context> ctx,
                                 PyObject *owner) {
  using namespace cvc::state_exec;
  if (!env || !ctx)
    throw std::invalid_argument("pycvc.register_intrinsics: env and ctx must not be None");
  // register_intrinsics only adds functions, so add them to a scratch scope and copy each one in,
  // wrapped to hold the context.
  auto scratch = std::make_shared<environment>();
  register_intrinsics(scratch, ctx.get());
  std::shared_ptr<PyObject> keep = python_ref(owner);
  for (auto &[name, v] : scratch->bindings) {
    const auto *fn = std::get_if<native_fn>(&v.v);
    if (fn == nullptr) {
      // The lock form: a copy that keeps the context and its Python owner too (LK-23). It stays
      // a lock_ops object, so the evaluator and a restored wait still find it (gap G-41).
      if (detail::lock_ops_of(v)) {
        auto held = std::make_shared<
            std::pair<std::shared_ptr<intrinsics_context>, std::shared_ptr<PyObject>>>(ctx, keep);
        env->set(name, detail::make_lock_ops(ctx.get(), std::move(held)));
        continue;
      }
      env->set(name, v);
      continue;
    }
    env->set(name, value_t(native_fn([inner = *fn, ctx, keep](std::span<const value_t> args) {
               return inner(args);
             })));
  }
}
} // namespace pycvc
%}
%rename(_register_intrinsics) pycvc::register_intrinsics_keeping;
namespace pycvc {
void register_intrinsics_keeping(cvc::state_exec::environment_ptr env,
                                 std::shared_ptr<cvc::state_exec::intrinsics_context> ctx,
                                 PyObject *owner);
}
%pythoncode %{
def register_intrinsics(env, ctx):
    """cvc::state_exec::register_intrinsics: add the DSL intrinsics over ctx to env.

    Each function added keeps ctx alive, and with it the scheduler and channel policy it holds."""
    _register_intrinsics(env, ctx, ctx)
%}

// Exec.scheduler() returns a reference into the Exec, so the scheduler keeps the Exec alive.
%pythonappend pycvc::Exec::scheduler %{
  val._pycvc_exec = self
%}
// So does Exec.async_host().
%pythonappend pycvc::Exec::async_host %{
  val._pycvc_exec = self
%}

// ── Phase 4: DSL execution + Python-authored DSL functions ──────────────
// pycvc.Exec(app) runs state_exec DSL programs over app's state root and lets a
// Python callable be registered as a DSL function (register_fn). PyObject* is
// SWIG's raw-passthrough type (the callable crosses uninspected; the .cpp
// INCREFs and manages its lifetime).
%include "pycvc_exec.h"

PYCVC_DEFAULT_EXCEPTION
