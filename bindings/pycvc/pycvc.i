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
// Every constructor/op is bound to the ONE module app (pycvc::ctx(), from
// pycvc_context.{h,cpp}, Phase 0): the factory %extend ctors build against
// it so a host-injected app and Python share one state tree.
%module pycvc

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

// ── Injected-app substrate (Phase 0) ────────────────────────────────
// cvc::app is shared into Python as an OPAQUE std::shared_ptr<cvc::app>: a
// host makes one (make_app()) and attach()es it; the module then binds all
// wrapped ctors/ops to that app's state tree. cvc::app is never dereferenced
// from Python — the handle is just passed back into the *_on helpers. Uses
// the std flavor of %shared_ptr (app manages these handles with std).
%shared_ptr(cvc::app)
namespace cvc { class app; }

namespace pycvc {
  std::shared_ptr<cvc::app> make_app();
  void attach(std::shared_ptr<cvc::app> handle);
  void detach();
  void state_set(const std::string& path, const std::string& value);
  std::string state_get(const std::string& path);
  void state_set_on(std::shared_ptr<cvc::app> handle, const std::string& path,
                    const std::string& value);
  std::string state_get_on(std::shared_ptr<cvc::app> handle, const std::string& path);
}

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
// ...but KEEP the zero-arg %extend factory ctor below. A blanket
// `%ignore Class::Class` also suppresses %extend-added constructors of the
// same name; volume has no real zero-arg ctor, so re-exposing volume() by
// signature un-ignores ONLY our factory (which binds to pycvc::ctx()).
%rename("%s") cvc::volume::volume();
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
// cvc::geometry HAS a real app-less default ctor `geometry()` (and a copy
// ctor); keep both exposed. Its container ops (points/tris/colors, COW) and
// the free-function file IO (read_geometry/write_geometry, no app&) never
// deref _ctx — only the member read() and Phase-2 algorithms do — so an
// app-less geometry is safe for everything Phase 1 exposes. Ignore ONLY the
// app&/string ctors (blanket-ignoring geometry() would also kill the default,
// leaving no constructor). Do NOT %extend a geometry() ctor: it would collide
// with the real zero-arg default.
%ignore cvc::geometry::geometry(cvc::app &);
%ignore cvc::geometry::geometry(cvc::app &, const std::string &);
%ignore cvc::geometry::geometry(const std::string &);
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
%rename("%s") cvc::geometry::smoothing(double, bool, bool);
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
  volume() { return new cvc::volume(pycvc::ctx()); }

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
    *$self = cvc::volume(pycvc::ctx(), reinterpret_cast<const unsigned char*>(fbuf.data()), dim,
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
  // (No factory ctor here — the real app-less geometry() default ctor is kept
  // exposed above; it is safe for all Phase-1 ops. See the %ignore notes.)

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

  // ── Mesh filters. smoothing() takes the injected app explicitly;
  // quality_improve() is mesher-gated (LBIE) — raises when the build has the
  // mesher off. The real same-named methods are %ignore'd and these %extend
  // overloads are re-exposed by signature (see the %rename notes above). ──
  void smoothing(double delta = 0.1, bool fix_boundary = false, bool geometric_flow = true) {
    $self->smoothing(pycvc::ctx(), static_cast<float>(delta), fix_boundary, /*perturb_1=*/false,
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

// ── Phase 2: compute layer (SDF / meshing / quality / generators) ───────
// Module-level free functions + enum constants + QualityStats, taking/returning
// the real wrapped cvc::geometry/cvc::volume (declared above). Comes last so
// those value types are already known to SWIG.
%include "pycvc_algorithm.h"
