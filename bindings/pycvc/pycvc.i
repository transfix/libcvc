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

%{
#include <cvc/core/app.h>
#include <cvc/core/exception.h>
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
%}
// Keep the app alive for as long as a volume/geometry built from it lives: the
// C++ voxels/geometry hold the app by RAW reference (app& _ctx), so without this
// the app could be freed while a Python object still points at it. Stash it on
// the proxy (the ctor arg is named `app`).
%pythonappend cvc::volume::volume(std::shared_ptr<cvc::app>) %{ self._pycvc_app = app %}
%pythonappend cvc::geometry::geometry(std::shared_ptr<cvc::app>) %{ self._pycvc_app = app %}
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
%ignore cvc::geometry::merge;
%ignore cvc::geometry::tri_surface;
%ignore cvc::geometry::calculate_surf_normals;
%ignore cvc::geometry::generate_wire_interior;
%ignore cvc::geometry::invert_normals;
%ignore cvc::geometry::reorient;
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

// ── Phase 3: state access + push callbacks ──────────────────────────────
// state_has/children/remove act on the shared root; state_observer is a
// director base — a Python subclass overriding on_changed() is called by C++
// on every state mutation. Only this class gets a director.
%feature("director") pycvc::state_observer;
%include "pycvc_state.h"

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

// ── Phase 4: DSL execution + Python-authored DSL functions ──────────────
// pycvc.Exec(app) runs state_exec DSL programs over app's state root and lets a
// Python callable be registered as a DSL function (register_fn). PyObject* is
// SWIG's raw-passthrough type (the callable crosses uninspected; the .cpp
// INCREFs and manages its lifetime).
%include "pycvc_exec.h"
