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

// A flattened [r0,c0,...] path -> a fresh (K,2) uint64 numpy array (new ref).
PyObject *pycvc_nav_path_array(const std::vector<int> &p)
{
  npy_intp dims[2] = {static_cast<npy_intp>(p.size() / 2), 2};
  PyObject *arr = PyArray_SimpleNew(2, dims, NPY_UINT64);
  if (!arr)
    return nullptr;
  auto *d = static_cast<std::uint64_t *>(
      PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
  for (std::size_t j = 0; j < p.size(); ++j)
    d[j] = static_cast<std::uint64_t>(p[j]);
  return arr;
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

// ── Batched, threaded kernels (PERFORMANCE.md stage 4) ──────────────────────

// Batched A*. `occ` is (N,H,W) uint8/bool — plane i is agent i's belief;
// `starts`/`goals` are (N,2) int; `cost` is (N,H,W) float64 or None;
// `num_threads` <= 0 uses hardware concurrency. Returns a length-N Python list
// of (Ki,2) uint64 paths ((0,2) == unreachable). The GIL is released across
// the parallel compute, so the N agents run concurrently.
PyObject *nav_astar_batch(PyObject *occ, PyObject *starts, PyObject *goals,
                          PyObject *cost, int num_threads = 0)
{
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(occ, NPY_UINT8, 3, 3, NPY_ARRAY_C_CONTIGUOUS));
  if (!a)
    throw std::invalid_argument(
        "pycvc.nav_astar_batch: occ must be an (N,H,W) uint8/bool array");
  const int N = static_cast<int>(PyArray_DIM(a, 0));
  const int rows = static_cast<int>(PyArray_DIM(a, 1));
  const int cols = static_cast<int>(PyArray_DIM(a, 2));
  PyArrayObject *sa = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(starts, NPY_INT32, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
  PyArrayObject *ga = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(goals, NPY_INT32, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
  if (!sa || !ga || PyArray_DIM(sa, 0) != N || PyArray_DIM(ga, 0) != N ||
      PyArray_DIM(sa, 1) != 2 || PyArray_DIM(ga, 1) != 2)
  {
    Py_DECREF(a);
    Py_XDECREF(sa);
    Py_XDECREF(ga);
    throw std::invalid_argument(
        "pycvc.nav_astar_batch: starts/goals must be (N,2) matching occ");
  }
  PyArrayObject *ca = nullptr;
  const double *cost_base = nullptr;
  if (cost && cost != Py_None)
  {
    ca = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(cost, NPY_DOUBLE, 3, 3, NPY_ARRAY_C_CONTIGUOUS));
    if (!ca || PyArray_DIM(ca, 0) != N || PyArray_DIM(ca, 1) != rows ||
        PyArray_DIM(ca, 2) != cols)
    {
      Py_DECREF(a);
      Py_DECREF(sa);
      Py_DECREF(ga);
      Py_XDECREF(ca);
      throw std::invalid_argument(
          "pycvc.nav_astar_batch: cost must be (N,H,W) matching occ");
    }
    cost_base = static_cast<const double *>(PyArray_DATA(ca));
  }
  const std::uint8_t *occ_base =
      static_cast<const std::uint8_t *>(PyArray_DATA(a));
  const std::int32_t *sd = static_cast<const std::int32_t *>(PyArray_DATA(sa));
  const std::int32_t *gd = static_cast<const std::int32_t *>(PyArray_DATA(ga));
  const long plane = static_cast<long>(rows) * cols;
  std::vector<cvc::nav::astar_query> qs(N);
  for (int i = 0; i < N; ++i)
  {
    qs[i].occ = occ_base + static_cast<long>(i) * plane;
    qs[i].start_r = sd[2 * i];
    qs[i].start_c = sd[2 * i + 1];
    qs[i].goal_r = gd[2 * i];
    qs[i].goal_c = gd[2 * i + 1];
    qs[i].cost = cost_base ? cost_base + static_cast<long>(i) * plane : nullptr;
  }
  std::vector<std::vector<int>> results;
  Py_BEGIN_ALLOW_THREADS
  results = cvc::nav::astar_batch(qs, rows, cols, num_threads);
  Py_END_ALLOW_THREADS
  Py_DECREF(a);
  Py_DECREF(sa);
  Py_DECREF(ga);
  Py_XDECREF(ca);
  PyObject *lst = PyList_New(static_cast<Py_ssize_t>(results.size()));
  if (!lst)
    throw std::runtime_error("pycvc.nav_astar_batch: result list alloc failed");
  for (std::size_t i = 0; i < results.size(); ++i)
  {
    PyObject *arr = pycvc_nav_path_array(results[i]);
    if (!arr)
    {
      Py_DECREF(lst);
      throw std::runtime_error("pycvc.nav_astar_batch: path array alloc failed");
    }
    PyList_SET_ITEM(lst, static_cast<Py_ssize_t>(i), arr); // steals the ref
  }
  return lst;
}

// Batched SDF build. `occ` is (N,H,W) uint8/bool. Returns (N,3,H,W) float32:
// plane [i,0]=phi, [i,1]=normal_x, [i,2]=normal_y. GIL released across compute.
ArrayView nav_build_sdf_batch(PyObject *occ, double min_x, double min_y,
                              double max_x, double max_y, double scale,
                              int num_threads = 0)
{
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(occ, NPY_UINT8, 3, 3, NPY_ARRAY_C_CONTIGUOUS));
  if (!a)
    throw std::invalid_argument(
        "pycvc.nav_build_sdf_batch: occ must be an (N,H,W) uint8/bool array");
  const int N = static_cast<int>(PyArray_DIM(a, 0));
  const int rows = static_cast<int>(PyArray_DIM(a, 1));
  const int cols = static_cast<int>(PyArray_DIM(a, 2));
  const std::uint8_t *base = static_cast<const std::uint8_t *>(PyArray_DATA(a));
  const long plane = static_cast<long>(rows) * cols;
  std::vector<const std::uint8_t *> occs(N);
  for (int i = 0; i < N; ++i)
    occs[i] = base + static_cast<long>(i) * plane;
  std::vector<cvc::nav::sdf_field> fields;
  Py_BEGIN_ALLOW_THREADS
  fields = cvc::nav::build_sdf_batch(occs, rows, cols, min_x, min_y, max_x,
                                     max_y, scale, num_threads);
  Py_END_ALLOW_THREADS
  Py_DECREF(a);
  std::vector<float> out(static_cast<std::size_t>(N) * 3 * plane);
  for (int i = 0; i < N; ++i)
  {
    float *dst = out.data() + static_cast<std::size_t>(i) * 3 * plane;
    std::copy(fields[i].phi.begin(), fields[i].phi.end(), dst);
    std::copy(fields[i].normal_x.begin(), fields[i].normal_x.end(), dst + plane);
    std::copy(fields[i].normal_y.begin(), fields[i].normal_y.end(),
              dst + 2 * plane);
  }
  return pycvc_nav_view<float>(std::move(out),
                               {(long)N, 3, (long)rows, (long)cols},
                               DType::Float32);
}

// Batched inflate. `occ` is (N,H,W) uint8/bool. Returns (N,H,W) uint8 dilated by
// `cells` 4-connected steps. GIL released across the parallel compute.
ArrayView nav_inflate_batch(PyObject *occ, int cells, int num_threads = 0)
{
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(occ, NPY_UINT8, 3, 3, NPY_ARRAY_C_CONTIGUOUS));
  if (!a)
    throw std::invalid_argument(
        "pycvc.nav_inflate_batch: occ must be an (N,H,W) uint8/bool array");
  const int N = static_cast<int>(PyArray_DIM(a, 0));
  const int rows = static_cast<int>(PyArray_DIM(a, 1));
  const int cols = static_cast<int>(PyArray_DIM(a, 2));
  const std::uint8_t *base = static_cast<const std::uint8_t *>(PyArray_DATA(a));
  const long plane = static_cast<long>(rows) * cols;
  std::vector<const std::uint8_t *> occs(N);
  for (int i = 0; i < N; ++i)
    occs[i] = base + static_cast<long>(i) * plane;
  std::vector<std::vector<std::uint8_t>> res;
  Py_BEGIN_ALLOW_THREADS
  res = cvc::nav::inflate_batch(occs, rows, cols, cells, num_threads);
  Py_END_ALLOW_THREADS
  Py_DECREF(a);
  std::vector<std::uint8_t> out(static_cast<std::size_t>(N) * plane);
  for (int i = 0; i < N; ++i)
    std::copy(res[i].begin(), res[i].end(), out.data() + static_cast<std::size_t>(i) * plane);
  return pycvc_nav_view<std::uint8_t>(std::move(out), {(long)N, (long)rows, (long)cols},
                                      DType::UInt8);
}

// Fixed-radius neighbour query (CGAL Kd_tree). `positions` is an (N,2) float64
// array of (x,y). Returns a length-N Python list; entry i is a uint64 array of
// the indices of every OTHER point within `radius` of point i.
PyObject *nav_neighbors(PyObject *positions, double radius)
{
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(positions, NPY_DOUBLE, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
  if (!a)
    throw std::invalid_argument(
        "pycvc.nav_neighbors: positions must be an (N,2) float64 array");
  if (PyArray_DIM(a, 1) != 2)
  {
    Py_DECREF(a);
    throw std::invalid_argument("pycvc.nav_neighbors: positions must have shape (N,2)");
  }
  const int n = static_cast<int>(PyArray_DIM(a, 0));
  const double *pos = static_cast<const double *>(PyArray_DATA(a));
  cvc::nav::neighbor_csr csr;
  Py_BEGIN_ALLOW_THREADS
  csr = cvc::nav::neighbors_within_radius(pos, n, radius);
  Py_END_ALLOW_THREADS
  Py_DECREF(a);
  PyObject *lst = PyList_New(n);
  if (!lst)
    throw std::runtime_error("pycvc.nav_neighbors: result list alloc failed");
  for (int i = 0; i < n; ++i)
  {
    npy_intp dim = csr.offsets[i + 1] - csr.offsets[i];
    PyObject *arr = PyArray_SimpleNew(1, &dim, NPY_UINT64);
    if (!arr)
    {
      Py_DECREF(lst);
      throw std::runtime_error("pycvc.nav_neighbors: array alloc failed");
    }
    auto *d = static_cast<std::uint64_t *>(
        PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
    for (int j = csr.offsets[i]; j < csr.offsets[i + 1]; ++j)
      d[j - csr.offsets[i]] = static_cast<std::uint64_t>(csr.indices[j]);
    PyList_SET_ITEM(lst, i, arr);
  }
  return lst;
}

} // namespace pycvc
%}
