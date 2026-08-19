// pycvc_nav.i — Python surface for cvc::nav (belief-space grid navigation).
//
// The compute lives in libcvc (inc/cvc/nav/grid_nav.h); this file only
// marshals numpy <-> the raw-pointer kernels. All numpy C-API use is kept
// INSIDE this interface file so it compiles into the SWIG wrapper translation
// unit — the one that called import_array() (%init in pycvc.i) — exactly like
// image.from_numpy. Array results are returned through the existing
// pycvc::ArrayView out-typemap (one owned copy, viewed zero-copy by numpy);
// occupancy/cost/path inputs come in as numpy arrays (PyObject*), coerced to a
// contiguous dtype for the duration of the call and released before returning,
// so no input array is pinned by a result.

%{
#include <cvc/nav/grid_nav.h>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {

// Coerce `o` to a C-contiguous 2-D uint8 array (accepts bool / int / uint8).
// Returns a NEW reference the caller must Py_DECREF; throws on failure.
PyArrayObject *pycvc_nav_as_u8(PyObject *o, int &rows, int &cols)
{
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(o, NPY_UINT8, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
  if (!a)
    throw std::invalid_argument("pycvc.nav: expected a 2-D uint8/bool array");
  rows = static_cast<int>(PyArray_DIM(a, 0));
  cols = static_cast<int>(PyArray_DIM(a, 1));
  return a;
}

// Move `src` into a heap buffer and return an ArrayView the out-typemap turns
// into a numpy array owning that buffer. The buffer is owned SOLELY by the
// numpy view (through the capsule), so the array is writable: mutating it — as
// GRL-SNAM does with the SDF/EDT fields (in-place np.clip, scaling) — touches
// only memory numpy exclusively holds, and it matches the fresh-writable-array
// contract of the pure-Python functions these replace.
template <class T>
pycvc::ArrayView pycvc_nav_view(std::vector<T> &&src, std::vector<long> shape,
                                pycvc::DType dt)
{
  auto buf = std::make_shared<std::vector<T>>(std::move(src));
  pycvc::ArrayView v;
  v.dtype = dt;
  v.shape = std::move(shape);
  v.writable = true;
  v.data = buf->empty() ? nullptr : buf->data();
  v.owner = buf;
  return v;
}

} // namespace
%}

%inline %{
namespace pycvc {

// (H,W) uint8/bool occupancy -> (H,W) float64 squared Euclidean distance
// transform to the nearest set (nonzero) cell. See cvc::nav::edt2_squared.
ArrayView nav_edt2_squared(PyObject *occ)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  std::vector<double> d = cvc::nav::edt2_squared(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols);
  Py_DECREF(a);
  return pycvc_nav_view<double>(std::move(d), {(long)rows, (long)cols},
                                DType::Float64);
}

// Footprint occupancy -> normalized SDF + unit normals (cvc::nav::build_sdf),
// returned as a single (3, H, W) float32 array: plane 0 = phi, plane 1 =
// normal_x (d/dcol), plane 2 = normal_y (d/drow). That is exactly the layout
// GRL-SNAM's SDFField stacks internally, so the adapter can hand the three
// planes straight through.
ArrayView nav_build_sdf(PyObject *occ, double min_x, double min_y, double max_x,
                        double max_y, double scale)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  cvc::nav::sdf_field f = cvc::nav::build_sdf(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, min_x,
      min_y, max_x, max_y, scale);
  Py_DECREF(a);
  const long n = static_cast<long>(rows) * cols;
  std::vector<float> out(3 * n);
  std::copy(f.phi.begin(), f.phi.end(), out.begin());
  std::copy(f.normal_x.begin(), f.normal_x.end(), out.begin() + n);
  std::copy(f.normal_y.begin(), f.normal_y.end(), out.begin() + 2 * n);
  return pycvc_nav_view<float>(std::move(out), {3, (long)rows, (long)cols},
                               DType::Float32);
}

// (H,W) uint8/bool -> (H,W) uint8 dilated by `cells` 4-connected steps.
ArrayView nav_inflate(PyObject *occ, int cells)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  std::vector<std::uint8_t> o = cvc::nav::inflate(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, cells);
  Py_DECREF(a);
  return pycvc_nav_view<std::uint8_t>(std::move(o), {(long)rows, (long)cols},
                                      DType::UInt8);
}

// True iff no blocked cell lies on the inclusive Bresenham segment.
bool nav_line_of_sight(PyObject *occ, int ar, int ac, int br, int bc)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  bool r = cvc::nav::line_of_sight(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, ar, ac,
      br, bc);
  Py_DECREF(a);
  return r;
}

// Snap (r,c) to the nearest free cell. Returns a (2,) uint64 {row,col}, or an
// empty (0,) array meaning None.
ArrayView nav_nearest_free(PyObject *occ, int r, int c, int max_radius = 12)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  std::pair<int, int> p = cvc::nav::nearest_free(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, r, c,
      max_radius);
  Py_DECREF(a);
  if (p.first < 0)
    return pycvc_nav_view<std::uint64_t>({}, {0}, DType::UInt64);
  std::vector<std::uint64_t> out = {(std::uint64_t)p.first,
                                    (std::uint64_t)p.second};
  return pycvc_nav_view<std::uint64_t>(std::move(out), {2}, DType::UInt64);
}

// 8-connected A* with corner-cut prevention. `cost` is a (H,W) float64 numpy
// array (per-cell surcharge) or None. Returns an (N,2) uint64 path, or an
// empty (0,2) array when unreachable.
ArrayView nav_astar(PyObject *occ, int start_r, int start_c, int goal_r,
                    int goal_c, PyObject *cost)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  const double *cptr = nullptr;
  PyArrayObject *ca = nullptr;
  if (cost && cost != Py_None)
  {
    ca = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(cost, NPY_DOUBLE, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
    if (!ca)
    {
      Py_DECREF(a);
      throw std::invalid_argument(
          "pycvc.nav_astar: cost must be a 2-D float64 array");
    }
    if (PyArray_DIM(ca, 0) != rows || PyArray_DIM(ca, 1) != cols)
    {
      Py_DECREF(a);
      Py_DECREF(ca);
      throw std::invalid_argument(
          "pycvc.nav_astar: cost shape must match occupancy");
    }
    cptr = static_cast<const double *>(PyArray_DATA(ca));
  }
  std::vector<int> path = cvc::nav::astar(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, start_r,
      start_c, goal_r, goal_c, cptr);
  Py_DECREF(a);
  if (ca)
    Py_DECREF(ca);
  const long n = static_cast<long>(path.size() / 2);
  std::vector<std::uint64_t> out(path.begin(), path.end());
  return pycvc_nav_view<std::uint64_t>(std::move(out), {n, 2}, DType::UInt64);
}

// String-pull an (N,2) int path. Returns an (M,2) uint64 path.
ArrayView nav_simplify(PyObject *occ, PyObject *path)
{
  int rows, cols;
  PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
  PyArrayObject *pa = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(path, NPY_INT32, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
  if (!pa)
  {
    Py_DECREF(a);
    throw std::invalid_argument(
        "pycvc.nav_simplify: path must be an (N,2) int array");
  }
  if (PyArray_DIM(pa, 1) != 2)
  {
    Py_DECREF(a);
    Py_DECREF(pa);
    throw std::invalid_argument(
        "pycvc.nav_simplify: path must have shape (N,2)");
  }
  const int n = static_cast<int>(PyArray_DIM(pa, 0));
  std::vector<int> s = cvc::nav::simplify(
      static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols,
      static_cast<const int *>(PyArray_DATA(pa)), n);
  Py_DECREF(a);
  Py_DECREF(pa);
  const long m = static_cast<long>(s.size() / 2);
  std::vector<std::uint64_t> out(s.begin(), s.end());
  return pycvc_nav_view<std::uint64_t>(std::move(out), {m, 2}, DType::UInt64);
}

} // namespace pycvc
%}
