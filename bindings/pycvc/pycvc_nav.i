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
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/grid_nav.h>
#include <algorithm>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
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

// Borrow a writable, C-contiguous ndarray of an EXACT dtype+ndim, WITHOUT a
// copy (never PyArray_FROMANY here — a coerce-copy would silently swallow every
// in-place write while the returned flips still looked plausible). Validate and
// reject: throws unless the caller's array is already exactly right. Returns the
// raw data pointer into caller memory (so the kernel writes through) and fills
// `shp` with the dims. The array stays alive via the caller's live reference for
// the whole call, so no INCREF is needed.
void *pycvc_nav_writable(PyObject *o, int typenum, int ndim, const char *name,
                         std::vector<npy_intp> &shp)
{
  const std::string who = std::string("pycvc.nav_sense_batch: ") + name;
  if (!o || !PyArray_Check(o))
    throw std::invalid_argument(who + " must be a numpy ndarray");
  PyArrayObject *a = reinterpret_cast<PyArrayObject *>(o);
  if (PyArray_TYPE(a) != typenum)
    throw std::invalid_argument(who + " has the wrong dtype (mutated in place, never coerced)");
  if (PyArray_NDIM(a) != ndim)
    throw std::invalid_argument(who + " has the wrong ndim");
  if (!PyArray_ISWRITEABLE(a))
    throw std::invalid_argument(who + " must be writable (it is mutated in place)");
  if (!PyArray_ISCARRAY(a))
    throw std::invalid_argument(who + " must be C-contiguous and aligned");
  shp.assign(PyArray_DIMS(a), PyArray_DIMS(a) + ndim);
  return PyArray_DATA(a);
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

// Batched belief sense. `logodds` (M,H,W f32), `last_visible`/`ever_seen`
// (M,H,W numpy-bool), `version` (M,) i32 are MUTATED IN PLACE (borrowed +
// validated, never copied — see pycvc_nav_writable). N agents (`positions`
// (N,2)f64, `headings`/`range_m`/`fov_rad` (N,)f64, `n_rays` (N,)i32) ray-cast
// `truth` (H,W u8) into plane `agent_map[i]` ((N,)i32). `peer_boxes` (N,Kmax,4)
// i32 and `mover_boxes` (Mv,4) i32 are optional half-open cell blockers (None to
// omit). Returns a fresh `flips` (N,) i32. GIL released across the compute.
PyObject *nav_sense_batch(PyObject *truth, PyObject *positions, PyObject *headings,
                          PyObject *range_m, PyObject *n_rays, PyObject *fov_rad,
                          PyObject *logodds, PyObject *last_visible, PyObject *ever_seen,
                          PyObject *version, PyObject *agent_map, PyObject *peer_boxes,
                          PyObject *mover_boxes, double min_x, double min_y, double max_x,
                          double max_y, double l_occ, double l_free, double l_clamp,
                          int num_threads = 0)
{
  // (a) In-place planes: borrow + validate (never copy). All three (M,H,W)
  // blocks must agree; version is (M,).
  std::vector<npy_intp> s_lo, s_lv, s_es, s_ver;
  float *lo = static_cast<float *>(pycvc_nav_writable(logodds, NPY_FLOAT, 3, "logodds", s_lo));
  std::uint8_t *lv =
      static_cast<std::uint8_t *>(pycvc_nav_writable(last_visible, NPY_BOOL, 3, "last_visible", s_lv));
  std::uint8_t *es =
      static_cast<std::uint8_t *>(pycvc_nav_writable(ever_seen, NPY_BOOL, 3, "ever_seen", s_es));
  std::int32_t *ver =
      static_cast<std::int32_t *>(pycvc_nav_writable(version, NPY_INT32, 1, "version", s_ver));
  const int M = static_cast<int>(s_lo[0]), H = static_cast<int>(s_lo[1]), W = static_cast<int>(s_lo[2]);
  if (s_lv != s_lo || s_es != s_lo || s_ver[0] != M)
    throw std::invalid_argument(
        "pycvc.nav_sense_batch: logodds/last_visible/ever_seen must share (M,H,W) and version be (M,)");

  // (b) Read-only inputs: FROMANY contiguous copies, tracked for cleanup.
  std::vector<PyArrayObject *> hold;
  auto fail = [&](const char *msg) {
    for (PyArrayObject *h : hold)
      Py_DECREF(h);
    throw std::invalid_argument(msg);
  };
  auto take = [&](PyObject *o, int typ, int nd) -> PyArrayObject * {
    PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(o, typ, nd, nd, NPY_ARRAY_C_CONTIGUOUS));
    if (!a)
      fail("pycvc.nav_sense_batch: an input array had the wrong dtype/rank");
    hold.push_back(a);
    return a;
  };
  PyArrayObject *tr = take(truth, NPY_UINT8, 2);
  if (PyArray_DIM(tr, 0) != H || PyArray_DIM(tr, 1) != W)
    fail("pycvc.nav_sense_batch: truth must be (H,W) matching the belief planes");
  PyArrayObject *po = take(positions, NPY_DOUBLE, 2);
  const int N = static_cast<int>(PyArray_DIM(po, 0));
  if (PyArray_DIM(po, 1) != 2)
    fail("pycvc.nav_sense_batch: positions must be (N,2)");
  PyArrayObject *he = take(headings, NPY_DOUBLE, 1);
  PyArrayObject *rg = take(range_m, NPY_DOUBLE, 1);
  PyArrayObject *nr = take(n_rays, NPY_INT32, 1);
  PyArrayObject *fv = take(fov_rad, NPY_DOUBLE, 1);
  PyArrayObject *am = take(agent_map, NPY_INT32, 1);
  if (PyArray_DIM(he, 0) != N || PyArray_DIM(rg, 0) != N || PyArray_DIM(nr, 0) != N ||
      PyArray_DIM(fv, 0) != N || PyArray_DIM(am, 0) != N)
    fail("pycvc.nav_sense_batch: headings/range_m/n_rays/fov_rad/agent_map must all be length N");

  const std::int32_t *peer = nullptr;
  int kmax = 0;
  if (peer_boxes && peer_boxes != Py_None) {
    PyArrayObject *pb = take(peer_boxes, NPY_INT32, 3);
    if (PyArray_DIM(pb, 0) != N || PyArray_DIM(pb, 2) != 4)
      fail("pycvc.nav_sense_batch: peer_boxes must be (N,Kmax,4)");
    kmax = static_cast<int>(PyArray_DIM(pb, 1));
    peer = static_cast<const std::int32_t *>(PyArray_DATA(pb));
  }
  const std::int32_t *mov = nullptr;
  int n_movers = 0;
  if (mover_boxes && mover_boxes != Py_None) {
    PyArrayObject *mb = take(mover_boxes, NPY_INT32, 2);
    if (PyArray_DIM(mb, 1) != 4)
      fail("pycvc.nav_sense_batch: mover_boxes must be (Mv,4)");
    n_movers = static_cast<int>(PyArray_DIM(mb, 0));
    mov = static_cast<const std::int32_t *>(PyArray_DATA(mb));
  }
  // Validate agent_map range before releasing the GIL (a bad index is OOB write).
  const std::int32_t *amd = static_cast<const std::int32_t *>(PyArray_DATA(am));
  for (int i = 0; i < N; ++i)
    if (amd[i] < 0 || amd[i] >= M)
      fail("pycvc.nav_sense_batch: agent_map has an out-of-range plane index");

  cvc::nav::sense_agents ag;
  ag.pos = static_cast<const double *>(PyArray_DATA(po));
  ag.heading = static_cast<const double *>(PyArray_DATA(he));
  ag.range_m = static_cast<const double *>(PyArray_DATA(rg));
  ag.fov_rad = static_cast<const double *>(PyArray_DATA(fv));
  ag.n_rays = static_cast<const std::int32_t *>(PyArray_DATA(nr));
  ag.agent_map = amd;
  ag.n = N;
  cvc::nav::belief_planes pl{lo, lv, es, ver, M};

  npy_intp fd = N;
  PyObject *flips = PyArray_SimpleNew(1, &fd, NPY_INT32);
  if (!flips)
    fail("pycvc.nav_sense_batch: flips alloc failed");
  std::int32_t *fo = static_cast<std::int32_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(flips)));

  const std::uint8_t *trd = static_cast<const std::uint8_t *>(PyArray_DATA(tr));
  Py_BEGIN_ALLOW_THREADS
  cvc::nav::sense_batch(trd, H, W, min_x, min_y, max_x, max_y, ag, peer, kmax, mov, n_movers, pl,
                        l_occ, l_free, l_clamp, fo, num_threads);
  Py_END_ALLOW_THREADS

  for (PyArrayObject *h : hold)
    Py_DECREF(h); // planes/version are borrowed (NOT decref'd)
  return flips;
}

// Torch-free bilinear SDF sample (the drive's field read). `field` is (M,3,H,W)
// float32, `on` is (N,2) float32 normalized positions, `map_id` is (N,) int32 or
// None (=> plane 0 for all). The world<->grid constants match SDFField. Returns
// a Python tuple (phi (N,) f32, normal (N,2) f32). Float-equivalent to
// SDFField.sample / BatchedSDFField.sample; GIL released across the compute.
PyObject *nav_sdf_sample(PyObject *field, PyObject *on, PyObject *map_id, double min_x, double min_y,
                         double max_x, double max_y, double cx, double cy, double scale,
                         int num_threads = 0)
{
  std::vector<PyArrayObject *> hold;
  auto fail = [&](const char *msg) {
    for (PyArrayObject *h : hold)
      Py_DECREF(h);
    throw std::invalid_argument(msg);
  };
  auto take = [&](PyObject *o, int typ, int nd) -> PyArrayObject * {
    PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(o, typ, nd, nd, NPY_ARRAY_C_CONTIGUOUS));
    if (!a)
      fail("pycvc.nav_sdf_sample: an input array had the wrong dtype/rank");
    hold.push_back(a);
    return a;
  };
  PyArrayObject *fa = take(field, NPY_FLOAT, 4);
  if (PyArray_DIM(fa, 1) != 3)
    fail("pycvc.nav_sdf_sample: field must be (M,3,H,W) float32");
  PyArrayObject *oa = take(on, NPY_FLOAT, 2);
  if (PyArray_DIM(oa, 1) != 2)
    fail("pycvc.nav_sdf_sample: on must be (N,2) float32");
  const int M = static_cast<int>(PyArray_DIM(fa, 0));
  const int H = static_cast<int>(PyArray_DIM(fa, 2));
  const int W = static_cast<int>(PyArray_DIM(fa, 3));
  const int N = static_cast<int>(PyArray_DIM(oa, 0));
  const int *mid = nullptr;
  if (map_id && map_id != Py_None) {
    PyArrayObject *ma = take(map_id, NPY_INT32, 1);
    if (PyArray_DIM(ma, 0) != N)
      fail("pycvc.nav_sdf_sample: map_id must be (N,)");
    const std::int32_t *m = static_cast<const std::int32_t *>(PyArray_DATA(ma));
    for (int i = 0; i < N; ++i)
      if (m[i] < 0 || m[i] >= M)
        fail("pycvc.nav_sdf_sample: map_id has an out-of-range plane index");
    mid = m;
  }

  npy_intp pd = N;
  PyObject *phi = PyArray_SimpleNew(1, &pd, NPY_FLOAT);
  npy_intp nd2[2] = {N, 2};
  PyObject *nrm = PyArray_SimpleNew(2, nd2, NPY_FLOAT);
  if (!phi || !nrm) {
    Py_XDECREF(phi);
    Py_XDECREF(nrm);
    fail("pycvc.nav_sdf_sample: output alloc failed");
  }
  float *phid = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(phi)));
  float *nrmd = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(nrm)));

  cvc::nav::field_stack fs;
  fs.data = static_cast<const float *>(PyArray_DATA(fa));
  fs.M = M;
  fs.H = H;
  fs.W = W;
  fs.mnx = min_x;
  fs.mny = min_y;
  fs.mxx = max_x;
  fs.mxy = max_y;
  fs.cx = cx;
  fs.cy = cy;
  fs.S = scale;
  const float *ond = static_cast<const float *>(PyArray_DATA(oa));
  Py_BEGIN_ALLOW_THREADS
  cvc::nav::sdf_sample(fs, ond, N, mid, phid, nrmd, num_threads);
  Py_END_ALLOW_THREADS

  for (PyArrayObject *h : hold)
    Py_DECREF(h);
  PyObject *tup = PyTuple_Pack(2, phi, nrm);
  Py_DECREF(phi);
  Py_DECREF(nrm);
  return tup;
}

// Load a `.cvcnav` policy from `path` and forward `feats` (N,in) float32 ->
// coeffs (N,out) float32. Reloads per call (a utility / the parity test uses it;
// a long-lived C++ host holds the coef_mlp object). Float-equivalent to
// CoefMLP.forward; GIL released across the compute.
PyObject *nav_coef_mlp_forward(const char *path, PyObject *feats, int num_threads = 0)
{
  PyArrayObject *fa = reinterpret_cast<PyArrayObject *>(
      PyArray_FROMANY(feats, NPY_FLOAT, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
  if (!fa)
    throw std::invalid_argument("pycvc.nav_coef_mlp_forward: feats must be (N,in) float32");
  cvc::nav::coef_mlp model = cvc::nav::coef_mlp::load(path); // throws on a bad/stale file
  const int N = static_cast<int>(PyArray_DIM(fa, 0));
  const int in = static_cast<int>(PyArray_DIM(fa, 1));
  if (in != model.in_features()) {
    Py_DECREF(fa);
    throw std::invalid_argument("pycvc.nav_coef_mlp_forward: feats width != model in_features");
  }
  npy_intp dims[2] = {N, model.out_features()};
  PyObject *out = PyArray_SimpleNew(2, dims, NPY_FLOAT);
  if (!out) {
    Py_DECREF(fa);
    throw std::runtime_error("pycvc.nav_coef_mlp_forward: output alloc failed");
  }
  const float *fd = static_cast<const float *>(PyArray_DATA(fa));
  float *od = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(out)));
  Py_BEGIN_ALLOW_THREADS
  model.forward(fd, N, od, num_threads);
  Py_END_ALLOW_THREADS
  Py_DECREF(fa);
  return out;
}

} // namespace pycvc
%}
