// pycvc_lod.i — SWIG surface for the cvc::lod family + cvc::simplify.
//
// NOT a standalone module: %include'd by pycvc.i (after geometry, volume, image
// and model are wrapped, since the pyramids, tiles and store hold and return
// them), so everything lands in the core `pycvc` module and pycvc_gl can %import
// mesh_pyramid / view_params for LodGraphicsNode.
//
// DIRECT WRAP, like the rest of pycvc: the real headers are %include'd --
// simplify.h, lod/pyramid.h, lod/select.h, lod/tiles.h, lod/store.h and
// core/thread_pool.h -- curated with %ignore where a member does not marshal,
// and given a thin %extend / typemap surface for the Python side:
//
//  * VALUE-STRUCT MEMBERS are %naturalvar'd, so a std::vector / std::string /
//    cvc::geometry member reads as a Python value (a tuple of copies, a str) and
//    assigns from any sequence. cvc::geometry and cvc::image copy-on-write, so a
//    pyramid's `rungs` tuple shares the rung buffers rather than duplicating
//    them -- but it IS a copy: mutate a rung and assign it back. A tile's
//    `geom` and `cell` are the same kind of copy, through properties (a
//    %naturalvar class member would read as a pointer INTO the tile, dangling
//    once the tile proxy is collected).
//  * OUT-PARAMETERS become extra return values: simplify(mesh) returns
//    (geometry, simplify_result); simplify_progressive returns (rungs, results).
//  * BYTES cross as bytes: scene_writer.to_blob() returns `bytes`, and
//    scene_reader(app, blob) / scene_reader.open_verified(app, blob, sha256)
//    take any bytes-like object (bytes, bytearray, memoryview, a numpy uint8
//    array) without a Python-side copy (the reader copies once, as its contract
//    says, so the buffer need not outlive the call).
//  * CALLBACKS take Python callables: build_tiled_pyramids(..., on_tile=f) calls
//    f(tile_index, tile, pyramid) as each tile completes; partition_*(...,
//    group_key=g) takes g(part_name) -> key, or a list of suffixes (the
//    suffix_group_key rule: ["_walls", "_roof"] groups "b7_walls" with
//    "b7_roof").
//
// THREADING. The long calls -- simplify, the three pyramid builders, partition,
// content hashing, build_tiled_pyramids and every scene.cvch5 read/write/bake --
// RELEASE THE GIL for their duration (an exception-safe RAII guard, not
// Py_BEGIN_ALLOW_THREADS, so a C++ throw restores the GIL on the way out), so
// other Python threads keep running: a loader thread can bake a city while the
// main thread keeps drawing. A Python callback (on_tile, group_key) reacquires
// the GIL for its own call, on whichever thread C++ runs it -- a pool worker or
// the caller. If the callback raises, the C++ call is unwound (after its pool
// fan-out joins, as thread_pool documents) and the ORIGINAL Python exception is
// re-raised from the call that started it, type and traceback intact.
//
// POOLS. Builders take an optional `pool`. Pass pycvc.thread_pool(n) -- a pool
// the caller owns, which is what tiles.h asks for when building behind a
// running render/sim loop -- or app.compute_pool(), the app's shared one. A
// thread_pool runs ONE fan-out at a time and a pooled build holds it for the
// whole call, so never share a pool between a background build whose callback
// needs the GIL and a foreground thread that would block on that same pool
// while HOLDING the GIL (every call in this file releases it first, so mixing
// these calls is safe; a third-party extension that does not release could
// deadlock).
//
// EXCEPTIONS. Within this file C++ failures map to the natural Python types:
// std::invalid_argument -> ValueError, std::out_of_range -> IndexError,
// std::bad_alloc -> MemoryError, an HDF5/store failure (cvc::hdf5_exception:
// a missing, corrupt or truncated file or blob, an absent asset, a container
// the hardened reader refuses) -> OSError, and anything else -> RuntimeError
// (including open_verified's SHA-256 mismatch, which is a std::runtime_error).
//
// LOD is a render proxy only: nothing here may feed a nav/material/RF path.

%{
#include <cvc/core/config.h> // CVC_HDF5_DISABLED: whether lod/store.h was built
#include <cvc/core/thread_pool.h>
#include <cvc/geometry/simplify.h>
#include <cvc/lod/pyramid.h>
#include <cvc/lod/select.h>
#include <cvc/lod/tiles.h>
#include <cstdio>
#include <cstring>
#ifndef CVC_HDF5_DISABLED
#include <cvc/lod/store.h>
#endif
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <vector>

namespace pycvc {

// Release the GIL for the lifetime of the guard. Exception-safe: a C++ throw
// unwinds through the destructor, which restores the thread state, so a throwing
// kernel cannot leave the interpreter without its GIL (the hazard pycvc_nav.i
// documents for Py_BEGIN_ALLOW_THREADS).
class gil_release {
public:
  gil_release() : _save(PyEval_SaveThread()) {}
  ~gil_release() { PyEval_RestoreThread(_save); }
  gil_release(const gil_release &) = delete;
  gil_release &operator=(const gil_release &) = delete;

private:
  PyThreadState *_save;
};

// Hold the GIL for the lifetime of the guard, from ANY thread (a pool worker
// that never touched Python, or a caller that released it above).
class gil_acquire {
public:
  gil_acquire() : _state(PyGILState_Ensure()) {}
  ~gil_acquire() { PyGILState_Release(_state); }
  gil_acquire(const gil_acquire &) = delete;
  gil_acquire &operator=(const gil_acquire &) = delete;

private:
  PyGILState_STATE _state;
};

// The Python exception a callback raised, carried through C++ as a C++
// exception (thread_pool re-throws the first one on the orchestrating thread)
// and re-raised unchanged once the wrapper holds the GIL again. Copies share
// one holder; whatever is never re-raised is released under the GIL.
struct py_error_state {
  PyObject *type = nullptr;
  PyObject *value = nullptr;
  PyObject *traceback = nullptr;
  ~py_error_state() {
    if (type || value || traceback) {
      gil_acquire g;
      Py_XDECREF(type);
      Py_XDECREF(value);
      Py_XDECREF(traceback);
    }
  }
};

class py_callback_error : public std::runtime_error {
public:
  // Take the CURRENT Python error (GIL held). Always yields an error, even if a
  // misbehaving callback returned NULL without setting one.
  static py_callback_error fetch() {
    auto st = std::make_shared<py_error_state>();
    PyErr_Fetch(&st->type, &st->value, &st->traceback);
    if (!st->type) {
      st->type = PyExc_RuntimeError;
      Py_INCREF(st->type);
      st->value = PyUnicode_FromString("pycvc: a Python callback failed without an exception");
    }
    return py_callback_error(st);
  }
  // Re-raise in the interpreter (GIL held), handing over the references.
  void restore() const {
    PyErr_Restore(_st->type, _st->value, _st->traceback);
    _st->type = _st->value = _st->traceback = nullptr;
  }

private:
  explicit py_callback_error(std::shared_ptr<py_error_state> st)
      : std::runtime_error("pycvc: a Python callback raised"), _st(std::move(st)) {}
  std::shared_ptr<py_error_state> _st;
};

// A strong reference to a Python object that may be dropped from any thread.
inline std::shared_ptr<PyObject> hold_pyobject(PyObject *o) {
  Py_INCREF(o);
  return std::shared_ptr<PyObject>(o, [](PyObject *p) {
    gil_acquire g;
    Py_DECREF(p);
  });
}

// A sequence of numbers -> doubles. False (Python error set) on failure.
inline bool seq_to_doubles(PyObject *o, std::vector<double> &out, const char *what) {
  PyObject *seq = PySequence_Fast(o, what);
  if (!seq)
    return false;
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  out.resize(static_cast<std::size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    out[static_cast<std::size_t>(i)] = PyFloat_AsDouble(PySequence_Fast_GET_ITEM(seq, i));
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      return false;
    }
  }
  Py_DECREF(seq);
  return true;
}

inline PyObject *bbox_tuple(const cvc::bounding_box &b) {
  return Py_BuildValue("(dddddd)", b.minx, b.miny, b.minz, b.maxx, b.maxy, b.maxz);
}

} // namespace pycvc
%}

// ── Exception mapping for this file ─────────────────────────────────────────
// The catch arms shared by every LOD call. A Python error from a callback comes
// FIRST (it derives from std::runtime_error); cvc::exception (here: the store's
// hdf5_exception) before std::exception, which it also derives from.
%define PYCVC_LOD_CATCH(CVC_ERROR)
  catch (const pycvc::py_callback_error &e) {
    e.restore();
    SWIG_fail;
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::out_of_range &e) {
    SWIG_exception(SWIG_IndexError, e.what());
  } catch (const std::bad_alloc &e) {
    SWIG_exception(SWIG_MemoryError, e.what());
  } catch (const cvc::exception &e) {
    SWIG_exception(CVC_ERROR, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (...) {
    SWIG_exception(SWIG_RuntimeError, "pycvc: C++ exception (see libcvc)");
  }
%enddef

// Default for this file: map, but keep the GIL (many %extend bodies below build
// Python objects).
%exception {
  try {
    $action
  }
  PYCVC_LOD_CATCH(SWIG_RuntimeError)
}

// A long call: release the GIL around the C++ call alone (argument conversion
// and result marshaling stay under the GIL). Only for calls whose $action never
// touches the Python API -- callbacks reacquire for themselves.
%define PYCVC_LOD_NOGIL(NAME)
%exception NAME {
  try {
    pycvc::gil_release _pycvc_nogil;
    $action
  }
  PYCVC_LOD_CATCH(SWIG_RuntimeError)
}
%enddef
// The same, for the HDF5 store: its failures (cvc::hdf5_exception) are I/O.
%define PYCVC_LOD_STORE_NOGIL(NAME)
%exception NAME {
  try {
    pycvc::gil_release _pycvc_nogil;
    $action
  }
  PYCVC_LOD_CATCH(SWIG_IOError)
}
%enddef

// Python keyword arguments + one wrapper per function (instead of one overload
// per defaulted argument), so `build_mesh_pyramid(g, params, pool=p)` works.
%define PYCVC_LOD_KWARGS(NAME)
%feature("kwargs") NAME;
%feature("compactdefaultargs") NAME;
%enddef

// ── Typemaps ────────────────────────────────────────────────────────────────

// A rung ladder from any sequence of numbers (list, tuple, DoubleVector, numpy).
%typemap(in) (const double *world_error_m, int nrungs) (std::vector<double> tmp) {
  if (!pycvc::seq_to_doubles($input, tmp, "world_error_m must be a sequence of numbers"))
    SWIG_fail;
  $1 = tmp.data();
  $2 = static_cast<int>(tmp.size());
}
%typemap(typecheck, precedence = SWIG_TYPECHECK_DOUBLE_ARRAY) (const double *world_error_m, int nrungs) {
  $1 = (PySequence_Check($input) && !PyUnicode_Check($input)) ? 1 : 0;
}

// A bounding-sphere centre from any 3-sequence.
%typemap(in) const double centre[3] (std::vector<double> tmp) {
  if (!pycvc::seq_to_doubles($input, tmp, "centre must be a 3-sequence"))
    SWIG_fail;
  if (tmp.size() != 3)
    SWIG_exception_fail(SWIG_ValueError, "centre must have exactly 3 elements");
  $1 = tmp.data();
}

// Triangle-count targets from any sequence of non-negative ints, including
// numpy integers (through __index__, which PyLong_AsUnsignedLongLong skips).
%typemap(in) const std::vector<std::uint64_t> &targets (std::vector<std::uint64_t> tmp) {
  PyObject *seq = PySequence_Fast($input, "targets must be a sequence of ints");
  if (!seq)
    SWIG_fail;
  const Py_ssize_t n = PySequence_Fast_GET_SIZE(seq);
  tmp.reserve(static_cast<std::size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    PyObject *as_int = PyNumber_Index(PySequence_Fast_GET_ITEM(seq, i));
    const unsigned long long v = as_int ? PyLong_AsUnsignedLongLong(as_int) : 0;
    Py_XDECREF(as_int);
    if (PyErr_Occurred()) {
      Py_DECREF(seq);
      SWIG_fail;
    }
    tmp.push_back(static_cast<std::uint64_t>(v));
  }
  Py_DECREF(seq);
  $1 = &tmp;
}

// simplify(..., simplify_result *out) -> an extra return value.
%typemap(in, numinputs = 0) cvc::simplify_result *out (cvc::simplify_result tmp) {
  $1 = &tmp;
}
%typemap(argout) cvc::simplify_result *out {
  %append_output(SWIG_NewPointerObj(new cvc::simplify_result(*$1),
                                    $descriptor(cvc::simplify_result *), SWIG_POINTER_OWN));
}
// simplify_progressive(..., std::vector<simplify_result> *out) -> a list of them.
%typemap(in, numinputs = 0) std::vector<cvc::simplify_result> *out
    (std::vector<cvc::simplify_result> tmp) {
  $1 = &tmp;
}
%typemap(argout) std::vector<cvc::simplify_result> *out {
  PyObject *lst = PyList_New(static_cast<Py_ssize_t>($1->size()));
  if (!lst)
    SWIG_fail;
  for (std::size_t i = 0; i < $1->size(); ++i)
    PyList_SET_ITEM(lst, static_cast<Py_ssize_t>(i),
                    SWIG_NewPointerObj(new cvc::simplify_result((*$1)[i]),
                                       $descriptor(cvc::simplify_result *), SWIG_POINTER_OWN));
  %append_output(lst);
}

// Parts for partition_parts: named_part objects or (name, geometry) pairs. The
// sequence is SNAPSHOT into a tuple held until the call returns, so the
// geometries stay alive while the GIL is released, even if another thread
// edits the caller's list meanwhile.
%typemap(in) const std::vector<cvc::lod::named_part> &parts
    (std::vector<cvc::lod::named_part> tmp, PyObject *hold = nullptr) {
  if (!PySequence_Check($input) || PyUnicode_Check($input))
    SWIG_exception_fail(SWIG_TypeError,
                        "parts must be a sequence of named_part or (name, geometry) pairs");
  hold = PySequence_Tuple($input);
  if (!hold)
    SWIG_fail;
  const Py_ssize_t n = PyTuple_GET_SIZE(hold);
  tmp.reserve(static_cast<std::size_t>(n));
  for (Py_ssize_t i = 0; i < n; ++i) {
    PyObject *item = PyTuple_GET_ITEM(hold, i);
    void *p = nullptr;
    if (SWIG_IsOK(SWIG_ConvertPtr(item, &p, $descriptor(cvc::lod::named_part *), 0)) && p) {
      tmp.push_back(*static_cast<cvc::lod::named_part *>(p));
      continue;
    }
    if (PyTuple_Check(item) && PyTuple_GET_SIZE(item) == 2 &&
        PyUnicode_Check(PyTuple_GET_ITEM(item, 0))) {
      void *g = nullptr;
      if (SWIG_IsOK(SWIG_ConvertPtr(PyTuple_GET_ITEM(item, 1), &g, $descriptor(cvc::geometry *), 0)) &&
          g) {
        Py_ssize_t len = 0;
        const char *s = PyUnicode_AsUTF8AndSize(PyTuple_GET_ITEM(item, 0), &len);
        if (!s)
          SWIG_fail;
        cvc::lod::named_part np;
        np.name.assign(s, static_cast<std::size_t>(len));
        np.geom = static_cast<const cvc::geometry *>(g);
        tmp.push_back(np);
        continue;
      }
    }
    PyErr_Format(PyExc_TypeError,
                 "parts[%zd] is neither a named_part nor a (str, geometry) pair", i);
    SWIG_fail;
  }
  $1 = &tmp;
}
%typemap(freearg) const std::vector<cvc::lod::named_part> &parts {
  Py_XDECREF(hold$argnum);
}

// group_key: None (each part its own group), a callable name -> key, or a list
// of suffixes (cvc::lod::suffix_group_key).
%typemap(in) const cvc::lod::group_key_fn & (cvc::lod::group_key_fn tmp) {
  if ($input == Py_None) {
    // identity
  } else if (PyCallable_Check($input)) {
    std::shared_ptr<PyObject> cb = pycvc::hold_pyobject($input);
    tmp = [cb](const std::string &name) -> std::string {
      pycvc::gil_acquire g;
      PyObject *arg = PyUnicode_FromStringAndSize(name.data(), static_cast<Py_ssize_t>(name.size()));
      if (!arg)
        throw pycvc::py_callback_error::fetch();
      PyObject *r = PyObject_CallFunctionObjArgs(cb.get(), arg, nullptr);
      Py_DECREF(arg);
      if (!r)
        throw pycvc::py_callback_error::fetch();
      Py_ssize_t len = 0;
      const char *s = PyUnicode_Check(r) ? PyUnicode_AsUTF8AndSize(r, &len) : nullptr;
      if (!s) {
        Py_DECREF(r);
        if (!PyErr_Occurred())
          PyErr_SetString(PyExc_TypeError, "group_key must return a str");
        throw pycvc::py_callback_error::fetch();
      }
      std::string key(s, static_cast<std::size_t>(len));
      Py_DECREF(r);
      return key;
    };
  } else if (PySequence_Check($input) && !PyUnicode_Check($input)) {
    PyObject *seq = PySequence_Fast($input, "group_key suffixes must be a sequence of str");
    if (!seq)
      SWIG_fail;
    std::vector<std::string> suffixes;
    for (Py_ssize_t i = 0; i < PySequence_Fast_GET_SIZE(seq); ++i) {
      Py_ssize_t len = 0;
      PyObject *it = PySequence_Fast_GET_ITEM(seq, i);
      const char *s = PyUnicode_Check(it) ? PyUnicode_AsUTF8AndSize(it, &len) : nullptr;
      if (!s) {
        Py_DECREF(seq);
        SWIG_exception_fail(SWIG_TypeError, "group_key suffixes must all be str");
      }
      suffixes.emplace_back(s, static_cast<std::size_t>(len));
    }
    Py_DECREF(seq);
    tmp = cvc::lod::suffix_group_key(suffixes);
  } else {
    SWIG_exception_fail(SWIG_TypeError,
                        "group_key must be None, a callable name -> key, or a list of suffixes");
  }
  $1 = &tmp;
}
%typemap(typecheck, precedence = SWIG_TYPECHECK_POINTER) const cvc::lod::group_key_fn & {
  $1 = ($input == Py_None || PyCallable_Check($input) ||
        (PySequence_Check($input) && !PyUnicode_Check($input))) ? 1 : 0;
}

// on_tile: None or a callable f(tile_index, tile, pyramid). The tile and the
// pyramid are COPIES (their geometry shares the built buffers), so the callback
// may keep them.
%typemap(in) const cvc::lod::tile_done_fn & (cvc::lod::tile_done_fn tmp) {
  if ($input != Py_None) {
    if (!PyCallable_Check($input))
      SWIG_exception_fail(SWIG_TypeError, "on_tile must be a callable f(index, tile, pyramid) or None");
    std::shared_ptr<PyObject> cb = pycvc::hold_pyobject($input);
    swig_type_info *tile_t = $descriptor(cvc::lod::tile *);
    swig_type_info *pyr_t = $descriptor(cvc::lod::mesh_pyramid *);
    tmp = [cb, tile_t, pyr_t](std::size_t index, const cvc::lod::tile &t,
                              const cvc::lod::mesh_pyramid &pyr) {
      pycvc::gil_acquire g;
      PyObject *idx = PyLong_FromSize_t(index);
      PyObject *pt = SWIG_NewPointerObj(new cvc::lod::tile(t), tile_t, SWIG_POINTER_OWN);
      PyObject *pp = SWIG_NewPointerObj(new cvc::lod::mesh_pyramid(pyr), pyr_t, SWIG_POINTER_OWN);
      PyObject *r = (idx && pt && pp) ? PyObject_CallFunctionObjArgs(cb.get(), idx, pt, pp, nullptr)
                                      : nullptr;
      Py_XDECREF(idx);
      Py_XDECREF(pt);
      Py_XDECREF(pp);
      if (!r)
        throw pycvc::py_callback_error::fetch();
      Py_DECREF(r);
    };
  }
  $1 = &tmp;
}
%typemap(typecheck, precedence = SWIG_TYPECHECK_POINTER) const cvc::lod::tile_done_fn & {
  $1 = ($input == Py_None || PyCallable_Check($input)) ? 1 : 0;
}

// ── cvc::thread_pool ─────────────────────────────────────────────────────────
// A pool the CALLER owns -- pycvc.thread_pool(n_workers) -- for the `pool`
// argument of the builders below. parallel_for (a std::function fan-out) is
// not exposed: Python drives the pool only through those kernels.
%ignore cvc::thread_pool::parallel_for;
%include "cvc/core/thread_pool.h"

// The app's own shared compute pool (cvc::app::computePool). Non-owning: the
// app owns it, so the returned handle keeps the app alive.
%pythonappend cvc::app::compute_pool %{
    val._pycvc_app = self
%}
%extend cvc::app {
  cvc::thread_pool &compute_pool() { return $self->computePool(); }
}

// ── cvc::simplify (QEM) ──────────────────────────────────────────────────────
PYCVC_LOD_NOGIL(cvc::simplify)
PYCVC_LOD_NOGIL(cvc::simplify_progressive)
PYCVC_LOD_NOGIL(cvc::sampled_hausdorff)
PYCVC_LOD_KWARGS(cvc::simplify)
PYCVC_LOD_KWARGS(cvc::simplify_progressive)
PYCVC_LOD_KWARGS(cvc::sampled_hausdorff)
%include "cvc/geometry/simplify.h"

%extend cvc::simplify_result {
  std::string __repr__() const {
    return "simplify_result(in_tris=" + std::to_string($self->in_tris) +
           ", out_tris=" + std::to_string($self->out_tris) +
           ", world_error=" + std::to_string($self->world_error) + ")";
  }
}

// ── cvc::lod pyramids ────────────────────────────────────────────────────────
// Sequence proxies for the rung vectors. cvc::volume has no default ctor, so a
// volume ladder's rungs are reached with rung(k) instead (below).
%template(GeometryVector) std::vector<cvc::geometry>;
%template(ImageVector) std::vector<cvc::image>;

// %naturalvar on a MEMBER: read it as a Python value, assign from a sequence.
%naturalvar cvc::lod::mesh_pyramid::rungs;
%naturalvar cvc::lod::mesh_pyramid::world_error_m;
%naturalvar cvc::lod::image_pyramid::rungs;
%naturalvar cvc::lod::image_pyramid::world_error_m;
%naturalvar cvc::lod::volume_pyramid::world_error_m;
%ignore cvc::lod::volume_pyramid::rungs;
PYCVC_LOD_NOGIL(cvc::lod::build_mesh_pyramid)
PYCVC_LOD_NOGIL(cvc::lod::build_volume_pyramid)
PYCVC_LOD_NOGIL(cvc::lod::build_image_pyramid)
PYCVC_LOD_KWARGS(cvc::lod::build_mesh_pyramid)
PYCVC_LOD_KWARGS(cvc::lod::build_volume_pyramid)
PYCVC_LOD_KWARGS(cvc::lod::build_image_pyramid)
%include "cvc/lod/pyramid.h"

%template(MeshPyramidVector) std::vector<cvc::lod::mesh_pyramid>;

%extend cvc::lod::mesh_pyramid {
  std::size_t rung_count() const { return $self->rungs.size(); }
  std::size_t __len__() const { return $self->rungs.size(); }
  // Triangles per rung, finest first.
  PyObject *rung_triangles() const {
    PyObject *lst = PyList_New(static_cast<Py_ssize_t>($self->rungs.size()));
    if (!lst)
      return nullptr;
    for (std::size_t k = 0; k < $self->rungs.size(); ++k)
      PyList_SET_ITEM(lst, static_cast<Py_ssize_t>(k),
                      PyLong_FromUnsignedLongLong($self->rungs[k].num_tris()));
    return lst;
  }
  // Rung k (a copy sharing the rung's buffers); IndexError out of range.
  cvc::geometry rung(int k) const {
    if (k < 0 || static_cast<std::size_t>(k) >= $self->rungs.size())
      throw std::out_of_range("mesh_pyramid.rung: index out of range");
    return $self->rungs[static_cast<std::size_t>(k)];
  }
}
%extend cvc::lod::image_pyramid {
  std::size_t rung_count() const { return $self->rungs.size(); }
  std::size_t __len__() const { return $self->rungs.size(); }
  cvc::image rung(int k) const {
    if (k < 0 || static_cast<std::size_t>(k) >= $self->rungs.size())
      throw std::out_of_range("image_pyramid.rung: index out of range");
    return $self->rungs[static_cast<std::size_t>(k)];
  }
}
%extend cvc::lod::volume_pyramid {
  std::size_t rung_count() const { return $self->rungs.size(); }
  std::size_t __len__() const { return $self->rungs.size(); }
  cvc::volume rung(int k) const {
    if (k < 0 || static_cast<std::size_t>(k) >= $self->rungs.size())
      throw std::out_of_range("volume_pyramid.rung: index out of range");
    return $self->rungs[static_cast<std::size_t>(k)];
  }
}

// ── cvc::lod selection math ──────────────────────────────────────────────────
// view_params, the quality presets, and the per-rung crossovers. The budget
// solver (candidate / solve / plan) is not wrapped: candidate points at
// caller-owned per-rung arrays, which a Python list cannot back safely.
%ignore cvc::lod::view_params::eye; // double[3] -> the `eye` property below
%ignore cvc::lod::budget;
%ignore cvc::lod::budget_profile;
%ignore cvc::lod::preset_budget;
%ignore cvc::lod::candidate;
%ignore cvc::lod::bound;
%ignore cvc::lod::plan;
%ignore cvc::lod::priority;
%ignore cvc::lod::draws;
%ignore cvc::lod::solve;
%ignore cvc::lod::solver;
%include "cvc/lod/select.h"

%extend cvc::lod::view_params {
  PyObject *get_eye() const {
    return Py_BuildValue("(ddd)", $self->eye[0], $self->eye[1], $self->eye[2]);
  }
  void set_eye(const std::vector<double> &e) {
    if (e.size() != 3)
      throw std::invalid_argument("view_params.eye: need [x, y, z]");
    $self->eye[0] = e[0];
    $self->eye[1] = e[1];
    $self->eye[2] = e[2];
  }
  std::string __repr__() const {
    char buf[256];
    std::snprintf(buf, sizeof buf,
                  "view_params(eye=(%g, %g, %g), viewport_h_px=%g, tan_half_fov=%g, "
                  "desired_pixel_error=%g, hysteresis=%g, ortho_px_per_m=%g)",
                  $self->eye[0], $self->eye[1], $self->eye[2], $self->viewport_h_px,
                  $self->tan_half_fov, $self->desired_pixel_error, $self->hysteresis,
                  $self->ortho_px_per_m);
    return buf;
  }
%pythoncode %{
    eye = property(get_eye, set_eye)
%}
}

// ── cvc::lod tiles ───────────────────────────────────────────────────────────
// named_part holds a NON-owning geometry pointer: keep the geometry alive on the
// proxy for as long as the part is.
%pythonappend cvc::lod::named_part::named_part %{
    if len(args) > 1: self._pycvc_geom = args[1]
%}
%naturalvar cvc::lod::named_part::name;
%naturalvar cvc::lod::tile::parts;
%ignore cvc::lod::named_part::geom;          // raw pointer; the ctor sets it
%ignore cvc::lod::tile::cell;                // -> the `cell` / `geom` copy properties below
%ignore cvc::lod::tile::geom;
%ignore cvc::lod::partition_params::origin;  // boost::array -> the `origin` property below
%ignore cvc::lod::tile::bounds;              // opaque bounding_box -> the `bounds` property below
%ignore cvc::lod::suffix_group_key;          // pass the suffix list as group_key instead
%ignore cvc::lod::cell_index::operator!=;
PYCVC_LOD_NOGIL(cvc::lod::partition_parts)
PYCVC_LOD_NOGIL(cvc::lod::partition_model)
PYCVC_LOD_NOGIL(cvc::lod::partition_components)
PYCVC_LOD_NOGIL(cvc::lod::content_hash)
PYCVC_LOD_NOGIL(cvc::lod::build_tiled_pyramids)
PYCVC_LOD_KWARGS(cvc::lod::partition_parts)
PYCVC_LOD_KWARGS(cvc::lod::partition_model)
PYCVC_LOD_KWARGS(cvc::lod::partition_components)
PYCVC_LOD_KWARGS(cvc::lod::content_hash)
PYCVC_LOD_KWARGS(cvc::lod::build_tiled_pyramids)
%include "cvc/lod/tiles.h"

%template(TileVector) std::vector<cvc::lod::tile>;

%extend cvc::lod::cell_index {
  long __hash__() const { return static_cast<long>($self->j * 1000003 + $self->i); }
  std::string __repr__() const {
    return "cell_index(i=" + std::to_string($self->i) + ", j=" + std::to_string($self->j) + ")";
  }
}
%extend cvc::lod::named_part {
  // The part's geometry (a copy sharing its buffers), or None for an empty part.
  PyObject *geometry_ref() const {
    if (!$self->geom)
      Py_RETURN_NONE;
    return SWIG_NewPointerObj(new cvc::geometry(*$self->geom), SWIGTYPE_p_cvc__geometry,
                              SWIG_POINTER_OWN);
  }
%pythoncode %{
    geometry = property(geometry_ref)
%}
}
%extend cvc::lod::partition_params {
  PyObject *get_origin() const {
    return Py_BuildValue("(ddd)", $self->origin[0], $self->origin[1], $self->origin[2]);
  }
  void set_origin(const std::vector<double> &o) {
    if (o.size() != 3)
      throw std::invalid_argument("partition_params.origin: need [x, y, z]");
    $self->origin[0] = o[0];
    $self->origin[1] = o[1];
    $self->origin[2] = o[2];
  }
%pythoncode %{
    origin = property(get_origin, set_origin)
%}
}
%extend cvc::lod::tile {
  // By value, so the result owns itself and outlives the tile (geometry is
  // copy-on-write: the copy shares the tile's buffers).
  cvc::geometry get_geom() const { return $self->geom; }
  void set_geom(const cvc::geometry &g) { $self->geom = g; }
  cvc::lod::cell_index get_cell() const { return $self->cell; }
  void set_cell(const cvc::lod::cell_index &c) { $self->cell = c; }
  // World AABB of the tile's points as (minx, miny, minz, maxx, maxy, maxz).
  PyObject *bounds_box() const { return pycvc::bbox_tuple($self->bounds); }
  std::string __repr__() const {
    return "tile(cell=(" + std::to_string($self->cell.i) + ", " + std::to_string($self->cell.j) +
           "), tris=" + std::to_string($self->geom.num_tris()) +
           ", parts=" + std::to_string($self->parts.size()) + ")";
  }
%pythoncode %{
    geom = property(get_geom, set_geom)
    cell = property(get_cell, set_cell)
    bounds = property(bounds_box)
%}
}

// ── cvc::lod store (scene.cvch5; needs libcvc built with HDF5) ───────────────
// CVC_HDF5_DISABLED comes from the generated <cvc/core/config.h>, %import'd so
// SWIG's preprocessor sees the same answer the C++ build does.
%import "cvc/core/config.h"
#ifndef CVC_HDF5_DISABLED

%naturalvar cvc::lod::lod_index_entry::name;
%naturalvar cvc::lod::lod_index_entry::world_error_m;
%naturalvar cvc::lod::lod_index_entry::source_hash;
%template(LodIndexEntryVector) std::vector<cvc::lod::lod_index_entry>;

// Bytes in: any bytes-like object, exported for the duration of the call (the
// reader copies it once, per its contract).
%typemap(in) (const unsigned char *bytes, std::size_t n) (Py_buffer view, bool have_view = false) {
  if (PyObject_GetBuffer($input, &view, PyBUF_SIMPLE) != 0)
    SWIG_exception_fail(SWIG_TypeError,
                        "expected a bytes-like object (bytes, bytearray, memoryview, uint8 array)");
  have_view = true;
  $1 = static_cast<$1_ltype>(view.buf);
  $2 = static_cast<std::size_t>(view.len);
}
%typemap(freearg) (const unsigned char *bytes, std::size_t n) {
  if (have_view$argnum)
    PyBuffer_Release(&view$argnum);
}
%typemap(typecheck, precedence = SWIG_TYPECHECK_CHAR_PTR) (const unsigned char *bytes, std::size_t n) {
  $1 = (PyObject_CheckBuffer($input) && !PyUnicode_Check($input)) ? 1 : 0;
}
// Bytes out: the container image as one `bytes` object. Only the allocation
// holds the GIL: the copy, which page-faults a fresh buffer as large as the
// container (as slow as to_blob itself), runs without it -- the object is not
// visible to any other thread until this returns.
%typemap(out) std::vector<unsigned char> cvc::lod::scene_writer::to_blob {
  $result = PyBytes_FromStringAndSize(nullptr, static_cast<Py_ssize_t>($1.size()));
  if (!$result)
    SWIG_fail;
  if (!$1.empty()) {
    pycvc::gil_release _pycvc_nogil;
    std::memcpy(PyBytes_AS_STRING($result), $1.data(), $1.size());
  }
}

// scene_reader is MOVE-ONLY (it owns an HDF5 handle), so the static by-value
// factory open_verified is not wrapped as declared: how a by-value return gets
// onto the heap is up to SWIG, and SWIG < 4.1 copy-constructs it
// (`new scene_reader(static_cast<const scene_reader &>(result))`), a hard
// compile error. The %extend below exposes the same Python call,
// scene_reader.open_verified(ctx, blob, sha256) -- same name, arguments,
// bytes-like typemap, GIL release and exception mapping -- as a factory that
// constructs the reader on the heap from the prvalue (no copy, no move) and
// hands Python ownership of it (%newobject).
%ignore cvc::lod::scene_reader::open_verified;
%rename(open_verified) cvc::lod::scene_reader::_pycvc_open_verified;
%newobject cvc::lod::scene_reader::_pycvc_open_verified;

// Readers and writers hold the app by reference: keep it alive on the proxy.
%pythonappend cvc::lod::scene_writer::scene_writer %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::lod::scene_reader::scene_reader %{
    if args: self._pycvc_app = args[0]
%}
%pythonappend cvc::lod::scene_reader::_pycvc_open_verified %{
    val._pycvc_app = ctx
%}
%ignore cvc::lod::scene_writer::scene_writer(scene_writer &&);
%ignore cvc::lod::scene_writer::operator=;
%ignore cvc::lod::scene_reader::scene_reader(scene_reader &&);
%ignore cvc::lod::scene_reader::operator=;

PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_writer::scene_writer)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_writer::write_mesh_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_writer::write_image_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_writer::to_blob)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_reader::scene_reader)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_reader::_pycvc_open_verified)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_reader::read_mesh_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_reader::read_image_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_reader::index)
PYCVC_LOD_STORE_NOGIL(cvc::lod::scene_reader::has)
PYCVC_LOD_STORE_NOGIL(cvc::lod::write_mesh_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::read_mesh_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::write_image_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::read_image_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::read_lod_index)
PYCVC_LOD_STORE_NOGIL(cvc::lod::has_pyramid)
PYCVC_LOD_STORE_NOGIL(cvc::lod::bake_mesh_asset)
PYCVC_LOD_NOGIL(cvc::lod::mesh_content_hash)
PYCVC_LOD_NOGIL(cvc::lod::image_content_hash)
PYCVC_LOD_KWARGS(cvc::lod::bake_mesh_asset)
%include "cvc/lod/store.h"

%extend cvc::lod::scene_reader {
  // Python's scene_reader.open_verified (renamed above). The parameter names
  // bind the bytes-like (bytes, n) typemap and the %pythonappend's `ctx`.
  static cvc::lod::scene_reader *_pycvc_open_verified(cvc::app &ctx, const unsigned char *bytes,
                                                      std::size_t n,
                                                      const std::string &expected_sha256_hex) {
    return new cvc::lod::scene_reader(
        cvc::lod::scene_reader::open_verified(ctx, bytes, n, expected_sha256_hex));
  }
}

%extend cvc::lod::lod_index_entry {
  std::string __repr__() const {
    return std::string("lod_index_entry(name='") + $self->name + "', kind='" + $self->kind +
           "', nrungs=" + std::to_string($self->nrungs) + ")";
  }
}

%pythoncode %{
HAVE_LOD_STORE = True
%}
#else
%pythoncode %{
HAVE_LOD_STORE = False
%}
#endif // CVC_HDF5_DISABLED

// Back to pycvc.i's default mapping for everything after this file.
PYCVC_DEFAULT_EXCEPTION

// ── Python conveniences ─────────────────────────────────────────────────────
%pythoncode %{
QUALITY_PRESETS = {
    "pristine": quality_preset_pristine,
    "balanced": quality_preset_balanced,
    "aggressive": quality_preset_aggressive,
}

_preset_view_enum = preset_view


def preset_view(preset="balanced"):
    """preset_view('pristine'|'balanced'|'aggressive') -> view_params.

    Also accepts the quality_preset_* constants, as the C++ call does."""
    if isinstance(preset, str):
        try:
            preset = QUALITY_PRESETS[preset]
        except KeyError:
            raise ValueError("preset_view: expected one of %s, got %r"
                             % (sorted(QUALITY_PRESETS), preset)) from None
    return _preset_view_enum(preset)


def view_at(x, y, z, preset="balanced"):
    """A view_params for an eye at (x, y, z) with a quality preset's budget."""
    v = preset_view(preset)
    v.eye = (x, y, z)
    return v
%}
