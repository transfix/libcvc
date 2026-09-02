// NOTE: do NOT run `clang-format -i` on this .i file. clang-format reads
// the SWIG directives %{ / %} / %inline %{ as C operators and mangles them
// (%{ -> % {), which breaks the SWIG parse. The CI clang-format leg
// deliberately excludes *.i for this reason; keep it that way.
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
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <cvc/nav/belief_occupancy.h>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/coef_train.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/grid_nav.h>
#include <cvc/nav/coef_energy_net.h>
#include <cvc/nav/geom_rollout.h>
#include <cvc/nav/material.h>
#include <cvc/nav/material_train.h>
#include <cvc/nav/sim_thread.h>
#include <cvc/nav/sim_world.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

      namespace {

          // Coerce `o` to a C-contiguous 2-D uint8 array (accepts bool / int / uint8).
          // Returns a NEW reference the caller must Py_DECREF; throws on failure.
          PyArrayObject * pycvc_nav_as_u8(PyObject * o, int &rows, int &cols){
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
pycvc::ArrayView pycvc_nav_view(std::vector<T> &&src, std::vector<long> shape, pycvc::DType dt) {
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
                         std::vector<npy_intp> &shp) {
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
PyObject *pycvc_nav_path_array(const std::vector<int> &p) {
  npy_intp dims[2] = {static_cast<npy_intp>(p.size() / 2), 2};
  PyObject *arr = PyArray_SimpleNew(2, dims, NPY_UINT64);
  if (!arr)
    return nullptr;
  auto *d = static_cast<std::uint64_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
  for (std::size_t j = 0; j < p.size(); ++j)
    d[j] = static_cast<std::uint64_t>(p[j]);
  return arr;
}

} // namespace
%}

// MaterialTrainer is an internal state holder (not default-constructible — it
// owns a material_adam). It is reached only through the nav_material_trainer_*
// handle functions, so SWIG must NOT try to wrap it as a Python class.
%ignore pycvc::MaterialTrainer;
// Internal numpy->material_batch marshaller (non-copyable, and its bind() takes 18
// raw PyObject*). Reached only through the nav_material_trainer_* functions.
%ignore pycvc::MaterialBatchArgs;

%inline %{
  namespace pycvc {

  // (H,W) uint8/bool occupancy -> (H,W) float64 squared Euclidean distance
  // transform to the nearest set (nonzero) cell. See cvc::nav::edt2_squared.
  ArrayView nav_edt2_squared(PyObject *occ) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    std::vector<double> d =
        cvc::nav::edt2_squared(static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols);
    Py_DECREF(a);
    return pycvc_nav_view<double>(std::move(d), {(long)rows, (long)cols}, DType::Float64);
  }

  // Footprint occupancy -> normalized SDF + unit normals (cvc::nav::build_sdf),
  // returned as a single (3, H, W) float32 array: plane 0 = phi, plane 1 =
  // normal_x (d/dcol), plane 2 = normal_y (d/drow). That is exactly the layout
  // GRL-SNAM's SDFField stacks internally, so the adapter can hand the three
  // planes straight through.
  ArrayView nav_build_sdf(PyObject *occ, double min_x, double min_y, double max_x, double max_y,
                          double scale) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    cvc::nav::sdf_field f = cvc::nav::build_sdf(static_cast<const std::uint8_t *>(PyArray_DATA(a)),
                                                rows, cols, min_x, min_y, max_x, max_y, scale);
    Py_DECREF(a);
    const long n = static_cast<long>(rows) * cols;
    std::vector<float> out(3 * n);
    std::copy(f.phi.begin(), f.phi.end(), out.begin());
    std::copy(f.normal_x.begin(), f.normal_x.end(), out.begin() + n);
    std::copy(f.normal_y.begin(), f.normal_y.end(), out.begin() + 2 * n);
    return pycvc_nav_view<float>(std::move(out), {3, (long)rows, (long)cols}, DType::Float32);
  }

  // (H,W) uint8/bool -> (H,W) uint8 dilated by `cells` 4-connected steps.
  ArrayView nav_inflate(PyObject *occ, int cells) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    std::vector<std::uint8_t> o =
        cvc::nav::inflate(static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, cells);
    Py_DECREF(a);
    return pycvc_nav_view<std::uint8_t>(std::move(o), {(long)rows, (long)cols}, DType::UInt8);
  }

  // True iff no blocked cell lies on the inclusive Bresenham segment.
  bool nav_line_of_sight(PyObject *occ, int ar, int ac, int br, int bc) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    bool r = cvc::nav::line_of_sight(static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols,
                                     ar, ac, br, bc);
    Py_DECREF(a);
    return r;
  }

  // Snap (r,c) to the nearest free cell. Returns a (2,) uint64 {row,col}, or an
  // empty (0,) array meaning None.
  ArrayView nav_nearest_free(PyObject *occ, int r, int c, int max_radius = 12) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    std::pair<int, int> p = cvc::nav::nearest_free(
        static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, r, c, max_radius);
    Py_DECREF(a);
    if (p.first < 0)
      return pycvc_nav_view<std::uint64_t>({}, {0}, DType::UInt64);
    std::vector<std::uint64_t> out = {(std::uint64_t)p.first, (std::uint64_t)p.second};
    return pycvc_nav_view<std::uint64_t>(std::move(out), {2}, DType::UInt64);
  }

  // 8-connected A* with corner-cut prevention. `cost` is a (H,W) float64 numpy
  // array (per-cell surcharge) or None. Returns an (N,2) uint64 path, or an
  // empty (0,2) array when unreachable.
  ArrayView nav_astar(PyObject *occ, int start_r, int start_c, int goal_r, int goal_c,
                      PyObject *cost) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    const double *cptr = nullptr;
    PyArrayObject *ca = nullptr;
    if (cost && cost != Py_None) {
      ca = reinterpret_cast<PyArrayObject *>(
          PyArray_FROMANY(cost, NPY_DOUBLE, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
      if (!ca) {
        Py_DECREF(a);
        throw std::invalid_argument("pycvc.nav_astar: cost must be a 2-D float64 array");
      }
      if (PyArray_DIM(ca, 0) != rows || PyArray_DIM(ca, 1) != cols) {
        Py_DECREF(a);
        Py_DECREF(ca);
        throw std::invalid_argument("pycvc.nav_astar: cost shape must match occupancy");
      }
      cptr = static_cast<const double *>(PyArray_DATA(ca));
    }
    std::vector<int> path = cvc::nav::astar(static_cast<const std::uint8_t *>(PyArray_DATA(a)),
                                            rows, cols, start_r, start_c, goal_r, goal_c, cptr);
    Py_DECREF(a);
    if (ca)
      Py_DECREF(ca);
    const long n = static_cast<long>(path.size() / 2);
    std::vector<std::uint64_t> out(path.begin(), path.end());
    return pycvc_nav_view<std::uint64_t>(std::move(out), {n, 2}, DType::UInt64);
  }

  // String-pull an (N,2) int path. Returns an (M,2) uint64 path.
  ArrayView nav_simplify(PyObject *occ, PyObject *path) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    PyArrayObject *pa = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(path, NPY_INT32, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
    if (!pa) {
      Py_DECREF(a);
      throw std::invalid_argument("pycvc.nav_simplify: path must be an (N,2) int array");
    }
    if (PyArray_DIM(pa, 1) != 2) {
      Py_DECREF(a);
      Py_DECREF(pa);
      throw std::invalid_argument("pycvc.nav_simplify: path must have shape (N,2)");
    }
    const int n = static_cast<int>(PyArray_DIM(pa, 0));
    std::vector<int> s =
        cvc::nav::simplify(static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols,
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
  PyObject *nav_astar_batch(PyObject *occ, PyObject *starts, PyObject *goals, PyObject *cost,
                            int num_threads = 0) {
    PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(occ, NPY_UINT8, 3, 3, NPY_ARRAY_C_CONTIGUOUS));
    if (!a)
      throw std::invalid_argument("pycvc.nav_astar_batch: occ must be an (N,H,W) uint8/bool array");
    const int N = static_cast<int>(PyArray_DIM(a, 0));
    const int rows = static_cast<int>(PyArray_DIM(a, 1));
    const int cols = static_cast<int>(PyArray_DIM(a, 2));
    PyArrayObject *sa = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(starts, NPY_INT32, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
    PyArrayObject *ga = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(goals, NPY_INT32, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
    if (!sa || !ga || PyArray_DIM(sa, 0) != N || PyArray_DIM(ga, 0) != N ||
        PyArray_DIM(sa, 1) != 2 || PyArray_DIM(ga, 1) != 2) {
      Py_DECREF(a);
      Py_XDECREF(sa);
      Py_XDECREF(ga);
      throw std::invalid_argument("pycvc.nav_astar_batch: starts/goals must be (N,2) matching occ");
    }
    PyArrayObject *ca = nullptr;
    const double *cost_base = nullptr;
    if (cost && cost != Py_None) {
      ca = reinterpret_cast<PyArrayObject *>(
          PyArray_FROMANY(cost, NPY_DOUBLE, 3, 3, NPY_ARRAY_C_CONTIGUOUS));
      if (!ca || PyArray_DIM(ca, 0) != N || PyArray_DIM(ca, 1) != rows ||
          PyArray_DIM(ca, 2) != cols) {
        Py_DECREF(a);
        Py_DECREF(sa);
        Py_DECREF(ga);
        Py_XDECREF(ca);
        throw std::invalid_argument("pycvc.nav_astar_batch: cost must be (N,H,W) matching occ");
      }
      cost_base = static_cast<const double *>(PyArray_DATA(ca));
    }
    const std::uint8_t *occ_base = static_cast<const std::uint8_t *>(PyArray_DATA(a));
    const std::int32_t *sd = static_cast<const std::int32_t *>(PyArray_DATA(sa));
    const std::int32_t *gd = static_cast<const std::int32_t *>(PyArray_DATA(ga));
    const long plane = static_cast<long>(rows) * cols;
    std::vector<cvc::nav::astar_query> qs(N);
    for (int i = 0; i < N; ++i) {
      qs[i].occ = occ_base + static_cast<long>(i) * plane;
      qs[i].start_r = sd[2 * i];
      qs[i].start_c = sd[2 * i + 1];
      qs[i].goal_r = gd[2 * i];
      qs[i].goal_c = gd[2 * i + 1];
      qs[i].cost = cost_base ? cost_base + static_cast<long>(i) * plane : nullptr;
    }
    std::vector<std::vector<int>> results;
    Py_BEGIN_ALLOW_THREADS results = cvc::nav::astar_batch(qs, rows, cols, num_threads);
    Py_END_ALLOW_THREADS Py_DECREF(a);
    Py_DECREF(sa);
    Py_DECREF(ga);
    Py_XDECREF(ca);
    PyObject *lst = PyList_New(static_cast<Py_ssize_t>(results.size()));
    if (!lst)
      throw std::runtime_error("pycvc.nav_astar_batch: result list alloc failed");
    for (std::size_t i = 0; i < results.size(); ++i) {
      PyObject *arr = pycvc_nav_path_array(results[i]);
      if (!arr) {
        Py_DECREF(lst);
        throw std::runtime_error("pycvc.nav_astar_batch: path array alloc failed");
      }
      PyList_SET_ITEM(lst, static_cast<Py_ssize_t>(i), arr); // steals the ref
    }
    return lst;
  }

  // Batched SDF build. `occ` is (N,H,W) uint8/bool. Returns (N,3,H,W) float32:
  // plane [i,0]=phi, [i,1]=normal_x, [i,2]=normal_y. GIL released across compute.
  ArrayView nav_build_sdf_batch(PyObject *occ, double min_x, double min_y, double max_x,
                                double max_y, double scale, int num_threads = 0) {
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
    Py_BEGIN_ALLOW_THREADS fields =
        cvc::nav::build_sdf_batch(occs, rows, cols, min_x, min_y, max_x, max_y, scale, num_threads);
    Py_END_ALLOW_THREADS Py_DECREF(a);
    std::vector<float> out(static_cast<std::size_t>(N) * 3 * plane);
    for (int i = 0; i < N; ++i) {
      float *dst = out.data() + static_cast<std::size_t>(i) * 3 * plane;
      std::copy(fields[i].phi.begin(), fields[i].phi.end(), dst);
      std::copy(fields[i].normal_x.begin(), fields[i].normal_x.end(), dst + plane);
      std::copy(fields[i].normal_y.begin(), fields[i].normal_y.end(), dst + 2 * plane);
    }
    return pycvc_nav_view<float>(std::move(out), {(long)N, 3, (long)rows, (long)cols},
                                 DType::Float32);
  }

  // Batched inflate. `occ` is (N,H,W) uint8/bool. Returns (N,H,W) uint8 dilated by
  // `cells` 4-connected steps. GIL released across the parallel compute.
  ArrayView nav_inflate_batch(PyObject *occ, int cells, int num_threads = 0) {
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
    Py_BEGIN_ALLOW_THREADS res = cvc::nav::inflate_batch(occs, rows, cols, cells, num_threads);
    Py_END_ALLOW_THREADS Py_DECREF(a);
    std::vector<std::uint8_t> out(static_cast<std::size_t>(N) * plane);
    for (int i = 0; i < N; ++i)
      std::copy(res[i].begin(), res[i].end(), out.data() + static_cast<std::size_t>(i) * plane);
    return pycvc_nav_view<std::uint8_t>(std::move(out), {(long)N, (long)rows, (long)cols},
                                        DType::UInt8);
  }

  // Fixed-radius neighbour query (CGAL Kd_tree). `positions` is an (N,2) float64
  // array of (x,y). Returns a length-N Python list; entry i is a uint64 array of
  // the indices of every OTHER point within `radius` of point i.
  PyObject *nav_neighbors(PyObject *positions, double radius) {
    PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(positions, NPY_DOUBLE, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
    if (!a)
      throw std::invalid_argument("pycvc.nav_neighbors: positions must be an (N,2) float64 array");
    if (PyArray_DIM(a, 1) != 2) {
      Py_DECREF(a);
      throw std::invalid_argument("pycvc.nav_neighbors: positions must have shape (N,2)");
    }
    const int n = static_cast<int>(PyArray_DIM(a, 0));
    const double *pos = static_cast<const double *>(PyArray_DATA(a));
    cvc::nav::neighbor_csr csr;
    Py_BEGIN_ALLOW_THREADS csr = cvc::nav::neighbors_within_radius(pos, n, radius);
    Py_END_ALLOW_THREADS Py_DECREF(a);
    PyObject *lst = PyList_New(n);
    if (!lst)
      throw std::runtime_error("pycvc.nav_neighbors: result list alloc failed");
    for (int i = 0; i < n; ++i) {
      npy_intp dim = csr.offsets[i + 1] - csr.offsets[i];
      PyObject *arr = PyArray_SimpleNew(1, &dim, NPY_UINT64);
      if (!arr) {
        Py_DECREF(lst);
        throw std::runtime_error("pycvc.nav_neighbors: array alloc failed");
      }
      auto *d = static_cast<std::uint64_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
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
                            int num_threads = 0) {
    // (a) In-place planes: borrow + validate (never copy). All three (M,H,W)
    // blocks must agree; version is (M,).
    std::vector<npy_intp> s_lo, s_lv, s_es, s_ver;
    float *lo = static_cast<float *>(pycvc_nav_writable(logodds, NPY_FLOAT, 3, "logodds", s_lo));
    std::uint8_t *lv = static_cast<std::uint8_t *>(
        pycvc_nav_writable(last_visible, NPY_BOOL, 3, "last_visible", s_lv));
    std::uint8_t *es =
        static_cast<std::uint8_t *>(pycvc_nav_writable(ever_seen, NPY_BOOL, 3, "ever_seen", s_es));
    std::int32_t *ver =
        static_cast<std::int32_t *>(pycvc_nav_writable(version, NPY_INT32, 1, "version", s_ver));
    const int M = static_cast<int>(s_lo[0]), H = static_cast<int>(s_lo[1]),
              W = static_cast<int>(s_lo[2]);
    if (s_lv != s_lo || s_es != s_lo || s_ver[0] != M)
      throw std::invalid_argument("pycvc.nav_sense_batch: logodds/last_visible/ever_seen must "
                                  "share (M,H,W) and version be (M,)");

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
    std::int32_t *fo =
        static_cast<std::int32_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(flips)));

    const std::uint8_t *trd = static_cast<const std::uint8_t *>(PyArray_DATA(tr));
    Py_BEGIN_ALLOW_THREADS cvc::nav::sense_batch(trd, H, W, min_x, min_y, max_x, max_y, ag, peer,
                                                 kmax, mov, n_movers, pl, l_occ, l_free, l_clamp,
                                                 fo, num_threads);
    Py_END_ALLOW_THREADS

        for (PyArrayObject *h : hold) Py_DECREF(h); // planes/version are borrowed (NOT decref'd)
    return flips;
  }

  // Torch-free bilinear SDF sample (the drive's field read). `field` is (M,3,H,W)
  // float32, `on` is (N,2) float32 normalized positions, `map_id` is (N,) int32 or
  // None (=> plane 0 for all). The world<->grid constants match SDFField. Returns
  // a Python tuple (phi (N,) f32, normal (N,2) f32). Float-equivalent to
  // SDFField.sample / BatchedSDFField.sample; GIL released across the compute.
  PyObject *nav_sdf_sample(PyObject *field, PyObject *on, PyObject *map_id, double min_x,
                           double min_y, double max_x, double max_y, double cx, double cy,
                           double scale, int num_threads = 0) {
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
    Py_BEGIN_ALLOW_THREADS cvc::nav::sdf_sample(fs, ond, N, mid, phid, nrmd, num_threads);
    Py_END_ALLOW_THREADS

        for (PyArrayObject *h : hold) Py_DECREF(h);
    PyObject *tup = PyTuple_Pack(2, phi, nrm);
    Py_DECREF(phi);
    Py_DECREF(nrm);
    return tup;
  }

  // Belief log-odds -> planning occupancy (belief.to_occupancy / composite).
  // logodds (H,W) f32; optional dyn_stamp (H,W) f64. Returns occ (H,W) uint8 (0/1).
  PyObject *nav_composite_occupancy(PyObject *logodds, const char *policy, double p_thresh,
                                    double band, PyObject *dyn_stamp, double t_now, double ttl_s) {
    PyArrayObject *la = reinterpret_cast<PyArrayObject *>(
        PyArray_FROMANY(logodds, NPY_FLOAT, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
    if (!la)
      throw std::invalid_argument("pycvc.nav_composite_occupancy: logodds must be (H,W) float32");
    const int H = static_cast<int>(PyArray_DIM(la, 0));
    const int W = static_cast<int>(PyArray_DIM(la, 1));
    cvc::nav::unknown_policy pol = cvc::nav::unknown_policy::optimistic;
    if (std::string(policy) == "pessimistic")
      pol = cvc::nav::unknown_policy::pessimistic;
    else if (std::string(policy) != "optimistic") {
      Py_DECREF(la);
      throw std::invalid_argument(
          "pycvc.nav_composite_occupancy: policy must be optimistic/pessimistic");
    }
    PyArrayObject *da = nullptr;
    const double *dyn = nullptr;
    if (dyn_stamp && dyn_stamp != Py_None) {
      da = reinterpret_cast<PyArrayObject *>(
          PyArray_FROMANY(dyn_stamp, NPY_DOUBLE, 2, 2, NPY_ARRAY_C_CONTIGUOUS));
      if (!da) {
        Py_DECREF(la);
        throw std::invalid_argument(
            "pycvc.nav_composite_occupancy: dyn_stamp must be (H,W) float64");
      }
      if (PyArray_DIM(da, 0) != H || PyArray_DIM(da, 1) != W) {
        Py_DECREF(la);
        Py_DECREF(da);
        throw std::invalid_argument("pycvc.nav_composite_occupancy: dyn_stamp shape != logodds");
      }
      dyn = static_cast<const double *>(PyArray_DATA(da));
    }
    npy_intp dims[2] = {H, W};
    PyObject *occ = PyArray_SimpleNew(2, dims, NPY_UINT8);
    if (!occ) {
      Py_DECREF(la);
      Py_XDECREF(da);
      throw std::runtime_error("pycvc.nav_composite_occupancy: output alloc failed");
    }
    const float *lo = static_cast<const float *>(PyArray_DATA(la));
    std::uint8_t *od =
        static_cast<std::uint8_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(occ)));
    Py_BEGIN_ALLOW_THREADS cvc::nav::composite_occupancy(lo, H, W, pol, p_thresh, band, dyn, t_now,
                                                         ttl_s, od);
    Py_END_ALLOW_THREADS Py_DECREF(la);
    Py_XDECREF(da);
    return occ;
  }

  // Load a `.cvcnav` policy from `path` and forward `feats` (N,in) float32 ->
  // coeffs (N,out) float32. Reloads per call (a utility / the parity test uses it;
  // a long-lived C++ host holds the coef_mlp object). Float-equivalent to
  // CoefMLP.forward; GIL released across the compute.
  PyObject *nav_coef_mlp_forward(const char *path, PyObject *feats, int num_threads = 0) {
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
    Py_BEGIN_ALLOW_THREADS model.forward(fd, N, od, num_threads);
    Py_END_ALLOW_THREADS Py_DECREF(fa);
    return out;
  }

  // Coefficient-net features. field (M,3,H,W) f32, on/goal (N,2) f32 normalized,
  // map_id (N,) i32 or None. Returns feat (N,5) f32. Float-equivalent to
  // sdf_nav.coef_feats; GIL released across the compute.
  PyObject *nav_coef_feats(PyObject *field, PyObject *on, PyObject *goal, PyObject *map_id,
                           double min_x, double min_y, double max_x, double max_y, double cx,
                           double cy, double scale, int num_threads = 0) {
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
        fail("pycvc.nav_coef_feats: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ga = take(goal, NPY_FLOAT, 2);
    const int M = static_cast<int>(PyArray_DIM(fa, 0));
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(fa, 1) != 3 || PyArray_DIM(oa, 1) != 2 || PyArray_DIM(ga, 0) != N ||
        PyArray_DIM(ga, 1) != 2)
      fail("pycvc.nav_coef_feats: shapes must be field(M,3,H,W) on(N,2) goal(N,2)");
    const int *mid = nullptr;
    if (map_id && map_id != Py_None) {
      PyArrayObject *ma = take(map_id, NPY_INT32, 1);
      if (PyArray_DIM(ma, 0) != N)
        fail("pycvc.nav_coef_feats: map_id must be (N,)");
      const std::int32_t *m = static_cast<const std::int32_t *>(PyArray_DATA(ma));
      for (int i = 0; i < N; ++i)
        if (m[i] < 0 || m[i] >= M)
          fail("pycvc.nav_coef_feats: map_id has an out-of-range plane index");
      mid = m;
    }
    npy_intp dims[2] = {N, 5};
    PyObject *feat = PyArray_SimpleNew(2, dims, NPY_FLOAT);
    if (!feat)
      fail("pycvc.nav_coef_feats: output alloc failed");
    cvc::nav::field_stack fs;
    fs.data = static_cast<const float *>(PyArray_DATA(fa));
    fs.M = M;
    fs.H = static_cast<int>(PyArray_DIM(fa, 2));
    fs.W = static_cast<int>(PyArray_DIM(fa, 3));
    fs.mnx = min_x;
    fs.mny = min_y;
    fs.mxx = max_x;
    fs.mxy = max_y;
    fs.cx = cx;
    fs.cy = cy;
    fs.S = scale;
    const float *ond = static_cast<const float *>(PyArray_DATA(oa));
    const float *gd = static_cast<const float *>(PyArray_DATA(ga));
    float *fo = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(feat)));
    Py_BEGIN_ALLOW_THREADS cvc::nav::coef_feats(fs, ond, gd, N, mid, fo, num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    return feat;
  }

  // One bicycle drive tick (nsub substeps). Returns fresh (o (N,2), th (N,), sp
  // (N,), minclr (N,)) f32; inputs are not mutated. Float-equivalent to
  // sdf_nav.bicycle_rollout(steps=1). GIL released across the compute.
  PyObject *nav_bicycle_rollout(PyObject *field, PyObject *on, PyObject *th, PyObject *sp,
                                PyObject *goal, PyObject *al, PyObject *be, PyObject *ga,
                                PyObject *map_id, double min_x, double min_y, double max_x,
                                double max_y, double cx, double cy, double scale, double rr,
                                double d_hat, double dt, double vmax, double L, double delta_max,
                                double a_max, double a_lat_max, double k_steer, int nsub,
                                int allow_reverse, int num_threads = 0,
                                PyObject *body_offsets = nullptr, double body_rr = 0.0,
                                double body_gain = 1.0, double track_width = 0.0,
                                PyObject *grip = nullptr) {
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
        fail("pycvc.nav_bicycle_rollout: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ta = take(th, NPY_FLOAT, 1);
    PyArrayObject *sa = take(sp, NPY_FLOAT, 1);
    PyArrayObject *gla = take(goal, NPY_FLOAT, 2);
    PyArrayObject *aa = take(al, NPY_FLOAT, 1);
    PyArrayObject *ba = take(be, NPY_FLOAT, 1);
    PyArrayObject *gaa = take(ga, NPY_FLOAT, 1);
    const int M = static_cast<int>(PyArray_DIM(fa, 0));
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(oa, 1) != 2 || PyArray_DIM(ta, 0) != N || PyArray_DIM(sa, 0) != N ||
        PyArray_DIM(gla, 0) != N || PyArray_DIM(gla, 1) != 2 || PyArray_DIM(aa, 0) != N ||
        PyArray_DIM(ba, 0) != N || PyArray_DIM(gaa, 0) != N)
      fail("pycvc.nav_bicycle_rollout: pose / coeff arrays must all be length N");
    const int *mid = nullptr;
    if (map_id && map_id != Py_None) {
      PyArrayObject *ma = take(map_id, NPY_INT32, 1);
      if (PyArray_DIM(ma, 0) != N)
        fail("pycvc.nav_bicycle_rollout: map_id must be (N,)");
      const std::int32_t *m = static_cast<const std::int32_t *>(PyArray_DATA(ma));
      for (int i = 0; i < N; ++i)
        if (m[i] < 0 || m[i] >= M)
          fail("pycvc.nav_bicycle_rollout: map_id has an out-of-range plane index");
      mid = m;
    }
    // Fresh output copies (mutated in place); inputs untouched.
    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *oo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *to = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *so = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *mc = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    if (!oo || !to || !so || !mc) {
      Py_XDECREF(oo);
      Py_XDECREF(to);
      Py_XDECREF(so);
      Py_XDECREF(mc);
      fail("pycvc.nav_bicycle_rollout: output alloc failed");
    }
    float *ood = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(oo)));
    float *tod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(to)));
    float *sod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(so)));
    float *mcd = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(mc)));
    std::memcpy(ood, PyArray_DATA(oa), sizeof(float) * 2 * N);
    std::memcpy(tod, PyArray_DATA(ta), sizeof(float) * N);
    std::memcpy(sod, PyArray_DATA(sa), sizeof(float) * N);

    cvc::nav::field_stack fs;
    fs.data = static_cast<const float *>(PyArray_DATA(fa));
    fs.M = M;
    fs.H = static_cast<int>(PyArray_DIM(fa, 2));
    fs.W = static_cast<int>(PyArray_DIM(fa, 3));
    fs.mnx = min_x;
    fs.mny = min_y;
    fs.mxx = max_x;
    fs.mxy = max_y;
    fs.cx = cx;
    fs.cy = cy;
    fs.S = scale;
    cvc::nav::veh_params v;
    v.rr = static_cast<float>(rr);
    v.d_hat = static_cast<float>(d_hat);
    v.dt = static_cast<float>(dt);
    v.vmax = static_cast<float>(vmax);
    v.L = static_cast<float>(L);
    v.delta_max = static_cast<float>(delta_max);
    v.a_max = static_cast<float>(a_max);
    v.a_lat_max = static_cast<float>(a_lat_max);
    v.k_steer = static_cast<float>(k_steer);
    v.nsub = nsub;
    v.allow_reverse = allow_reverse != 0;
    // Optional vehicle refinements; absent (None / 0) leaves the legacy drive
    // bit-for-bit. The arrays are borrowed for the call, kept alive by `hold`.
    v.body_rr = static_cast<float>(body_rr);
    v.body_gain = static_cast<float>(body_gain);
    v.track_width = static_cast<float>(track_width);
    if (body_offsets && body_offsets != Py_None) {
      PyArrayObject *boa = take(body_offsets, NPY_FLOAT, 1);
      v.body_offsets = static_cast<const float *>(PyArray_DATA(boa));
      v.n_body = static_cast<int>(PyArray_DIM(boa, 0));
      if (v.n_body > 0 && !(v.body_rr > 0.0f))
        fail("nav_bicycle_rollout: body_offsets needs a positive body_rr");
    }
    cvc::nav::friction_field gf;
    if (grip && grip != Py_None) {
      // The grip raster shares the FIELD's world frame — same bounds, center and
      // scale. Anything else would need its own seven constants on an already
      // long signature, and every caller builds both from one scenario anyway.
      PyArrayObject *gpa = take(grip, NPY_FLOAT, 2);
      gf.data = static_cast<const float *>(PyArray_DATA(gpa));
      gf.M = 1;
      gf.H = static_cast<int>(PyArray_DIM(gpa, 0));
      gf.W = static_cast<int>(PyArray_DIM(gpa, 1));
      gf.mnx = min_x;
      gf.mny = min_y;
      gf.mxx = max_x;
      gf.mxy = max_y;
      gf.cx = cx;
      gf.cy = cy;
      gf.S = scale;
      v.grip = &gf;
    }
    const float *gld = static_cast<const float *>(PyArray_DATA(gla));
    const float *ald = static_cast<const float *>(PyArray_DATA(aa));
    const float *bed = static_cast<const float *>(PyArray_DATA(ba));
    const float *gad = static_cast<const float *>(PyArray_DATA(gaa));
    Py_BEGIN_ALLOW_THREADS cvc::nav::bicycle_rollout(fs, ood, tod, sod, gld, ald, bed, gad, N, mid,
                                                     v, mcd, num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    PyObject *tup = PyTuple_Pack(4, oo, to, so, mc);
    Py_DECREF(oo);
    Py_DECREF(to);
    Py_DECREF(so);
    Py_DECREF(mc);
    return tup;
  }

  // Carrot FSM (swarm.py._plan_carrot). The read-only pose/sample come in as
  // (N,2)/(N,) f32; the FSM state columns are borrowed writable and mutated IN
  // PLACE (stall/mode/hist_count i32, turn/dhit/best/wall_entry/pos_hist f32,
  // we_valid bool, sp f32); tracking/parked/active are read-only bool. Returns
  // carrot (N,2) f32.
  // ── sim_world: the pure-C++ reactive swarm runtime ──────────────────────────
  // Wrapped as an opaque PyCapsule handle (the object owns beliefs/field/policy/
  // columns); create -> step -> snapshot, with live retarget.

  static const char *kSimWorldCapsule = "cvc.nav.sim_world";

  static void sim_world_capsule_dtor(PyObject *cap) {
    auto *sw = static_cast<cvc::nav::sim_world *>(PyCapsule_GetPointer(cap, kSimWorldCapsule));
    delete sw;
  }

  static cvc::nav::sim_world *sim_world_from(PyObject *cap) {
    if (!cap || !PyCapsule_CheckExact(cap))
      throw std::invalid_argument("pycvc.nav_sim_world_*: not a sim_world handle");
    auto *sw = static_cast<cvc::nav::sim_world *>(PyCapsule_GetPointer(cap, kSimWorldCapsule));
    if (!sw)
      throw std::invalid_argument("pycvc.nav_sim_world_*: null / already-destroyed handle");
    return sw;
  }

  PyObject *nav_sim_world_create(PyObject *truth, PyObject *prior_occ, const char *weights_path,
                                 PyObject *on, PyObject *goal, PyObject *color, int rows, int cols,
                                 double min_x, double min_y, double max_x, double max_y, double cx,
                                 double cy, double scale, double range_m, int n_rays,
                                 double fov_rad, double rr, double d_hat, double dt, double vmax,
                                 double L, double delta_max, double a_max, double a_lat_max,
                                 double k_steer, int nsub, int allow_reverse, double reach_tol,
                                 int sense_every, int freeze_sense, double l_occ, double l_free,
                                 double l_clamp, int optimistic, double p_thresh, double band,
                                 double ttl_s) {
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
        fail("pycvc.nav_sim_world_create: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *tr = take(truth, NPY_UINT8, 2);
    PyArrayObject *pr = take(prior_occ, NPY_UINT8, 2);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ga = take(goal, NPY_FLOAT, 2);
    PyArrayObject *ca = take(color, NPY_FLOAT, 2);
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(tr, 0) != rows || PyArray_DIM(tr, 1) != cols || PyArray_DIM(pr, 0) != rows ||
        PyArray_DIM(pr, 1) != cols || PyArray_DIM(ga, 0) != N || PyArray_DIM(ca, 0) != N)
      fail("pycvc.nav_sim_world_create: truth/prior (rows,cols), on/goal/color (N,.) mismatch");

    cvc::nav::coef_mlp model = cvc::nav::coef_mlp::load(weights_path); // throws on bad/stale

    cvc::nav::sim_world::config cfg;
    cfg.rows = rows;
    cfg.cols = cols;
    cfg.min_x = min_x;
    cfg.min_y = min_y;
    cfg.max_x = max_x;
    cfg.max_y = max_y;
    cfg.cx = cx;
    cfg.cy = cy;
    cfg.scale = scale;
    cfg.range_m = range_m;
    cfg.n_rays = n_rays;
    cfg.fov_rad = fov_rad;
    cfg.veh.rr = static_cast<float>(rr);
    cfg.veh.d_hat = static_cast<float>(d_hat);
    cfg.veh.dt = static_cast<float>(dt);
    cfg.veh.vmax = static_cast<float>(vmax);
    cfg.veh.L = static_cast<float>(L);
    cfg.veh.delta_max = static_cast<float>(delta_max);
    cfg.veh.a_max = static_cast<float>(a_max);
    cfg.veh.a_lat_max = static_cast<float>(a_lat_max);
    cfg.veh.k_steer = static_cast<float>(k_steer);
    cfg.veh.nsub = nsub;
    cfg.veh.allow_reverse = allow_reverse != 0;
    cfg.reach_tol = static_cast<float>(reach_tol);
    cfg.sense_every = sense_every;
    cfg.freeze_sense = freeze_sense != 0;
    cfg.l_occ = l_occ;
    cfg.l_free = l_free;
    cfg.l_clamp = l_clamp;
    cfg.optimistic = optimistic != 0;
    cfg.p_thresh = p_thresh;
    cfg.band = band;
    cfg.ttl_s = ttl_s;

    cvc::nav::sim_world *sw = nullptr;
    sw = new cvc::nav::sim_world(cfg, static_cast<const std::uint8_t *>(PyArray_DATA(tr)),
                                 static_cast<const std::uint8_t *>(PyArray_DATA(pr)),
                                 std::move(model), static_cast<const float *>(PyArray_DATA(oa)),
                                 static_cast<const float *>(PyArray_DATA(ga)),
                                 static_cast<const float *>(PyArray_DATA(ca)), N);
    for (PyArrayObject *h : hold)
      Py_DECREF(h);
    return PyCapsule_New(sw, kSimWorldCapsule, sim_world_capsule_dtor);
  }

  PyObject *nav_sim_world_step(PyObject *handle, int num_threads = 0) {
    cvc::nav::sim_world *sw = sim_world_from(handle);
    Py_BEGIN_ALLOW_THREADS sw->step(num_threads);
    Py_END_ALLOW_THREADS Py_RETURN_NONE;
  }

  // Returns (pos (N,2) f32 world, heading (N,) f32, speed (N,) f32, mode (N,) i32,
  // reached (N,) bool).
  PyObject *nav_sim_world_snapshot(PyObject *handle) {
    cvc::nav::sim_world *sw = sim_world_from(handle);
    const int N = sw->size();
    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *pos = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *hd = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *sp = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *md = PyArray_SimpleNew(1, &d1, NPY_INT32);
    PyObject *rc = PyArray_SimpleNew(1, &d1, NPY_BOOL);
    if (!pos || !hd || !sp || !md || !rc) {
      Py_XDECREF(pos);
      Py_XDECREF(hd);
      Py_XDECREF(sp);
      Py_XDECREF(md);
      Py_XDECREF(rc);
      throw std::runtime_error("pycvc.nav_sim_world_snapshot: alloc failed");
    }
    sw->snapshot(static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(pos))),
                 static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(hd))),
                 static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(sp))),
                 static_cast<int *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(md))),
                 static_cast<std::uint8_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(rc))));
    PyObject *tup = PyTuple_Pack(5, pos, hd, sp, md, rc);
    Py_DECREF(pos);
    Py_DECREF(hd);
    Py_DECREF(sp);
    Py_DECREF(md);
    Py_DECREF(rc);
    return tup;
  }

  PyObject *nav_sim_world_retarget(PyObject *handle, int i, double gx_n, double gy_n) {
    sim_world_from(handle)->retarget(i, static_cast<float>(gx_n), static_cast<float>(gy_n));
    Py_RETURN_NONE;
  }

  // ── material on the sim_world (P2b) ─────────────────────────────────────────
  // risk/hard are [planes,H,W] (planes==1 shared; >1 one plane per belief group,
  // keyed per agent by the SAME map_id as belief). The material_config is passed
  // as scalars, matching the sim_world_create style.
  PyObject *nav_sim_world_set_material(PyObject *handle, PyObject *risk, PyObject *hard, int planes,
                                       double lam_soft, double lam_hard, double k_sharp,
                                       double d_hat_m, double sigma, int gate_enabled,
                                       int primitive_count, int horizon_cells, double hard_margin_m,
                                       double improvement_margin, double material_trigger,
                                       double progress_slack_cells) {
    cvc::nav::sim_world *sw = sim_world_from(handle);
    if (planes < 1)
      throw std::invalid_argument("pycvc.nav_sim_world_set_material: planes must be >= 1");
    std::vector<PyArrayObject *> hold;
    auto fail = [&](const char *msg) {
      for (PyArrayObject *h : hold)
        Py_DECREF(h);
      throw std::invalid_argument(msg);
    };
    auto take = [&](PyObject *o, int typ) -> PyArrayObject * {
      PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
          PyArray_FROMANY(o, typ, 1, 3, NPY_ARRAY_C_CONTIGUOUS));
      if (!a)
        fail("pycvc.nav_sim_world_set_material: risk/hard had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *r = take(risk, NPY_FLOAT);
    PyArrayObject *h = take(hard, NPY_UINT8);
    const npy_intp want =
        static_cast<npy_intp>(planes) * sw->rows() * sw->cols();
    if (PyArray_SIZE(r) != want || PyArray_SIZE(h) != want)
      fail("pycvc.nav_sim_world_set_material: risk/hard must hold planes*rows*cols elements");
    cvc::nav::material_config mc;
    mc.lam_soft = static_cast<float>(lam_soft);
    mc.lam_hard = static_cast<float>(lam_hard);
    mc.k_sharp = static_cast<float>(k_sharp);
    mc.d_hat_m = static_cast<float>(d_hat_m);
    mc.sigma = sigma;
    mc.gate_enabled = gate_enabled != 0;
    mc.gate.primitive_count = primitive_count;
    mc.gate.horizon_cells = horizon_cells;
    mc.gate.hard_margin_m = hard_margin_m;
    mc.gate.improvement_margin = improvement_margin;
    mc.gate.material_trigger = material_trigger;
    mc.gate.progress_slack_cells = progress_slack_cells;
    // sim_world::set_material throws when any agent's map_id >= planes (each agent
    // indexes its material plane by its BELIEF plane id). A C++ throw crossing
    // Py_BEGIN_ALLOW_THREADS leaves the GIL unrestored and takes the interpreter
    // down, so validate the precondition HERE, before the release — the house rule
    // for every throwing kernel in this file.
    if (planes > 1) {
      const int *mid = sw->agent_planes();
      for (int i = 0, n = sw->size(); i < n; ++i)
        if (mid[i] >= planes)
          fail("pycvc.nav_sim_world_set_material: an agent's map_id is >= planes (each agent "
               "indexes its material plane by its belief plane id)");
    }
    const float *rd = static_cast<const float *>(PyArray_DATA(r));
    const std::uint8_t *hd = static_cast<const std::uint8_t *>(PyArray_DATA(h));
    Py_BEGIN_ALLOW_THREADS sw->set_material(rd, hd, mc, planes);
    Py_END_ALLOW_THREADS for (PyArrayObject *a : hold) Py_DECREF(a);
    Py_RETURN_NONE;
  }

  PyObject *nav_sim_world_clear_material(PyObject *handle) {
    sim_world_from(handle)->clear_material();
    Py_RETURN_NONE;
  }

  // Last tick's per-agent gate decision [n] bool (valid only while material set).
  PyObject *nav_sim_world_material_gate_active(PyObject *handle) {
    cvc::nav::sim_world *sw = sim_world_from(handle);
    // material_gate_active() hands back mat_gate_active_.data(), and that vector is
    // only sized inside set_material — on a world that never had material attached
    // it is EMPTY, so the copy below would read N elements off a null/!dereferenceable
    // pointer. Refuse instead of corrupting the interpreter.
    if (!sw->has_material())
      throw std::invalid_argument("pycvc.nav_sim_world_material_gate_active: no material attached "
                                  "(call nav_sim_world_set_material first)");
    const int N = sw->size();
    npy_intp d1 = N;
    PyObject *out = PyArray_SimpleNew(1, &d1, NPY_BOOL);
    if (!out)
      throw std::runtime_error("pycvc.nav_sim_world_material_gate_active: alloc failed");
    const std::uint8_t *g = sw->material_gate_active();
    std::uint8_t *dst = static_cast<std::uint8_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(out)));
    for (int i = 0; i < N; ++i)
      dst[i] = g[i];
    return out;
  }

  // ── sim_thread: run a sim_world off the render thread ───────────────────────
  // The C++ worker is a real thread that never touches the GIL (it runs only
  // cvc::nav kernels), so it advances genuinely concurrently with Python. The
  // sim_thread capsule holds a reference to the sim_world capsule so the world
  // outlives the thread; ~sim_thread joins before the world is released.

  static const char *kSimThreadCapsule = "cvc.nav.sim_thread";

  static void sim_thread_capsule_dtor(PyObject *cap) {
    auto *st = static_cast<cvc::nav::sim_thread *>(PyCapsule_GetPointer(cap, kSimThreadCapsule));
    delete st; // ~sim_thread stops+joins the worker before the world can be freed
    PyObject *swcap = static_cast<PyObject *>(PyCapsule_GetContext(cap));
    Py_XDECREF(swcap);
  }

  static cvc::nav::sim_thread *sim_thread_from(PyObject *cap) {
    if (!cap || !PyCapsule_CheckExact(cap))
      throw std::invalid_argument("pycvc.nav_sim_thread_*: not a sim_thread handle");
    auto *st = static_cast<cvc::nav::sim_thread *>(PyCapsule_GetPointer(cap, kSimThreadCapsule));
    if (!st)
      throw std::invalid_argument("pycvc.nav_sim_thread_*: null handle");
    return st;
  }

  PyObject *nav_sim_thread_create(PyObject *world_handle, double hz) {
    cvc::nav::sim_world *sw = sim_world_from(world_handle); // validates the handle
    auto *st = new cvc::nav::sim_thread(*sw, hz);
    PyObject *cap = PyCapsule_New(st, kSimThreadCapsule, sim_thread_capsule_dtor);
    if (!cap) {
      delete st;
      throw std::runtime_error("pycvc.nav_sim_thread_create: capsule alloc failed");
    }
    PyCapsule_SetContext(cap, world_handle); // keep the world alive under us
    Py_INCREF(world_handle);
    return cap;
  }

  PyObject *nav_sim_thread_start(PyObject *cap) {
    sim_thread_from(cap)->start();
    Py_RETURN_NONE;
  }

  PyObject *nav_sim_thread_stop(PyObject *cap) {
    cvc::nav::sim_thread *st = sim_thread_from(cap);
    Py_BEGIN_ALLOW_THREADS st->stop();
    Py_END_ALLOW_THREADS Py_RETURN_NONE;
  }

  PyObject *nav_sim_thread_retarget(PyObject *cap, int i, double gx_n, double gy_n) {
    sim_thread_from(cap)->retarget(i, static_cast<float>(gx_n), static_cast<float>(gy_n));
    Py_RETURN_NONE;
  }

  PyObject *nav_sim_thread_set_paused(PyObject *cap, int paused) {
    sim_thread_from(cap)->set_paused(paused != 0);
    Py_RETURN_NONE;
  }

  PyObject *nav_sim_thread_set_rate(PyObject *cap, double hz) {
    sim_thread_from(cap)->set_rate(hz);
    Py_RETURN_NONE;
  }

  PyObject *nav_sim_thread_ticks(PyObject *cap) {
    return PyLong_FromLong(sim_thread_from(cap)->ticks());
  }

  PyObject *nav_sim_thread_behind(PyObject *cap) {
    return PyLong_FromLong(sim_thread_from(cap)->behind());
  }

  // Lock-free read of the latest published frame -> (pos (N,2) world f32, heading
  // f32, speed f32, mode i32, reached bool, tick int), or None before the first.
  PyObject *nav_sim_thread_read(PyObject *cap) {
    std::shared_ptr<const cvc::nav::sim_thread::snapshot> s = sim_thread_from(cap)->read();
    if (!s)
      Py_RETURN_NONE;
    const int N = s->n;
    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *pos = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *hd = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *sp = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *md = PyArray_SimpleNew(1, &d1, NPY_INT32);
    PyObject *rc = PyArray_SimpleNew(1, &d1, NPY_BOOL);
    if (!pos || !hd || !sp || !md || !rc) {
      Py_XDECREF(pos);
      Py_XDECREF(hd);
      Py_XDECREF(sp);
      Py_XDECREF(md);
      Py_XDECREF(rc);
      throw std::runtime_error("pycvc.nav_sim_thread_read: alloc failed");
    }
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(pos)), s->pos.data(),
                sizeof(float) * 2 * N);
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(hd)), s->heading.data(),
                sizeof(float) * N);
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(sp)), s->speed.data(),
                sizeof(float) * N);
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(md)), s->mode.data(),
                sizeof(int) * N);
    std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(rc)), s->reached.data(),
                sizeof(std::uint8_t) * N);
    PyObject *tup = PyTuple_Pack(6, pos, hd, sp, md, rc, PyLong_FromLong(s->tick));
    Py_DECREF(pos);
    Py_DECREF(hd);
    Py_DECREF(sp);
    Py_DECREF(md);
    Py_DECREF(rc);
    return tup;
  }

  PyObject *nav_carrot_step(PyObject *on, PyObject *goal, PyObject *th, PyObject *sp, PyObject *phi,
                            PyObject *nrm, PyObject *stall, PyObject *mode, PyObject *turn,
                            PyObject *dhit, PyObject *best, PyObject *wall_entry,
                            PyObject *we_valid, PyObject *tracking, PyObject *pos_hist,
                            PyObject *hist_count, PyObject *parked, PyObject *active,
                            double reach_tol, double a_max, double dt, int num_threads = 0) {
    std::vector<PyArrayObject *> hold;
    auto fail = [&](const char *msg) {
      for (PyArrayObject *h : hold)
        Py_DECREF(h);
      throw std::invalid_argument(msg);
    };
    auto ro = [&](PyObject *o, int typ, int nd) -> PyArrayObject * {
      PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
          PyArray_FROMANY(o, typ, nd, nd, NPY_ARRAY_C_CONTIGUOUS));
      if (!a)
        fail("pycvc.nav_carrot_step: a read-only input had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *oa = ro(on, NPY_FLOAT, 2);
    PyArrayObject *ga = ro(goal, NPY_FLOAT, 2);
    PyArrayObject *tha = ro(th, NPY_FLOAT, 1);
    PyArrayObject *pha = ro(phi, NPY_FLOAT, 1);
    PyArrayObject *na = ro(nrm, NPY_FLOAT, 2);
    PyArrayObject *tka = ro(tracking, NPY_BOOL, 1);
    PyArrayObject *pka = ro(parked, NPY_BOOL, 1);
    PyArrayObject *aca = ro(active, NPY_BOOL, 1);
    const int N = static_cast<int>(PyArray_DIM(oa, 0));

    std::vector<npy_intp> sh;
    cvc::nav::fsm_state s;
    s.stall = static_cast<int *>(pycvc_nav_writable(stall, NPY_INT32, 1, "stall", sh));
    s.mode = static_cast<int *>(pycvc_nav_writable(mode, NPY_INT32, 1, "mode", sh));
    s.turn = static_cast<float *>(pycvc_nav_writable(turn, NPY_FLOAT, 1, "turn", sh));
    s.dhit = static_cast<float *>(pycvc_nav_writable(dhit, NPY_FLOAT, 1, "dhit", sh));
    s.best = static_cast<float *>(pycvc_nav_writable(best, NPY_FLOAT, 1, "best", sh));
    s.wall_entry =
        static_cast<float *>(pycvc_nav_writable(wall_entry, NPY_FLOAT, 2, "wall_entry", sh));
    s.we_valid =
        static_cast<std::uint8_t *>(pycvc_nav_writable(we_valid, NPY_BOOL, 1, "we_valid", sh));
    s.pos_hist = static_cast<float *>(pycvc_nav_writable(pos_hist, NPY_FLOAT, 3, "pos_hist", sh));
    s.hist_count =
        static_cast<int *>(pycvc_nav_writable(hist_count, NPY_INT32, 1, "hist_count", sh));
    float *spd = static_cast<float *>(pycvc_nav_writable(sp, NPY_FLOAT, 1, "sp", sh));
    s.tracking = static_cast<const std::uint8_t *>(PyArray_DATA(tka));
    s.parked = static_cast<const std::uint8_t *>(PyArray_DATA(pka));
    s.active = static_cast<const std::uint8_t *>(PyArray_DATA(aca));

    npy_intp d2[2] = {N, 2};
    PyObject *carrot = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    if (!carrot)
      fail("pycvc.nav_carrot_step: output alloc failed");
    cvc::nav::carrot_params cp;
    cp.reach_tol = static_cast<float>(reach_tol);
    cp.a_max = static_cast<float>(a_max);
    cp.dt = static_cast<float>(dt);
    const float *ond = static_cast<const float *>(PyArray_DATA(oa));
    const float *gd = static_cast<const float *>(PyArray_DATA(ga));
    const float *thd = static_cast<const float *>(PyArray_DATA(tha));
    const float *phd = static_cast<const float *>(PyArray_DATA(pha));
    const float *nd = static_cast<const float *>(PyArray_DATA(na));
    float *cod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(carrot)));
    Py_BEGIN_ALLOW_THREADS cvc::nav::carrot_step(ond, gd, thd, spd, phd, nd, s, N, cp, cod,
                                                 num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    return carrot;
  }

  // The fused per-agent drive tick: sample -> coef_feats -> coef_mlp(weights_path)
  // -> bicycle_rollout, given each agent's carrot. Returns fresh (o (N,2), th (N,),
  // sp (N,), minclr (N,)) f32. Float-equivalent to the torch Swarm drive.
  PyObject *nav_drive_step(PyObject *field, PyObject *on, PyObject *th, PyObject *sp,
                           PyObject *carrot, const char *weights_path, PyObject *map_id,
                           double min_x, double min_y, double max_x, double max_y, double cx,
                           double cy, double scale, double rr, double d_hat, double dt, double vmax,
                           double L, double delta_max, double a_max, double a_lat_max,
                           double k_steer, int nsub, int allow_reverse, int num_threads = 0) {
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
        fail("pycvc.nav_drive_step: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ta = take(th, NPY_FLOAT, 1);
    PyArrayObject *sa = take(sp, NPY_FLOAT, 1);
    PyArrayObject *ca = take(carrot, NPY_FLOAT, 2);
    const int M = static_cast<int>(PyArray_DIM(fa, 0));
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(oa, 1) != 2 || PyArray_DIM(ta, 0) != N || PyArray_DIM(sa, 0) != N ||
        PyArray_DIM(ca, 0) != N || PyArray_DIM(ca, 1) != 2)
      fail("pycvc.nav_drive_step: pose / carrot arrays must all be length N");
    const int *mid = nullptr;
    if (map_id && map_id != Py_None) {
      PyArrayObject *ma = take(map_id, NPY_INT32, 1);
      if (PyArray_DIM(ma, 0) != N)
        fail("pycvc.nav_drive_step: map_id must be (N,)");
      const std::int32_t *m = static_cast<const std::int32_t *>(PyArray_DATA(ma));
      for (int i = 0; i < N; ++i)
        if (m[i] < 0 || m[i] >= M)
          fail("pycvc.nav_drive_step: map_id has an out-of-range plane index");
      mid = m;
    }
    cvc::nav::coef_mlp model = cvc::nav::coef_mlp::load(weights_path); // throws on bad/stale file

    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *oo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *to = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *so = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *mc = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    if (!oo || !to || !so || !mc) {
      Py_XDECREF(oo);
      Py_XDECREF(to);
      Py_XDECREF(so);
      Py_XDECREF(mc);
      fail("pycvc.nav_drive_step: output alloc failed");
    }
    float *ood = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(oo)));
    float *tod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(to)));
    float *sod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(so)));
    float *mcd = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(mc)));
    std::memcpy(ood, PyArray_DATA(oa), sizeof(float) * 2 * N);
    std::memcpy(tod, PyArray_DATA(ta), sizeof(float) * N);
    std::memcpy(sod, PyArray_DATA(sa), sizeof(float) * N);

    cvc::nav::field_stack fs;
    fs.data = static_cast<const float *>(PyArray_DATA(fa));
    fs.M = M;
    fs.H = static_cast<int>(PyArray_DIM(fa, 2));
    fs.W = static_cast<int>(PyArray_DIM(fa, 3));
    fs.mnx = min_x;
    fs.mny = min_y;
    fs.mxx = max_x;
    fs.mxy = max_y;
    fs.cx = cx;
    fs.cy = cy;
    fs.S = scale;
    cvc::nav::veh_params v;
    v.rr = static_cast<float>(rr);
    v.d_hat = static_cast<float>(d_hat);
    v.dt = static_cast<float>(dt);
    v.vmax = static_cast<float>(vmax);
    v.L = static_cast<float>(L);
    v.delta_max = static_cast<float>(delta_max);
    v.a_max = static_cast<float>(a_max);
    v.a_lat_max = static_cast<float>(a_lat_max);
    v.k_steer = static_cast<float>(k_steer);
    v.nsub = nsub;
    v.allow_reverse = allow_reverse != 0;
    const float *cad = static_cast<const float *>(PyArray_DATA(ca));
    Py_BEGIN_ALLOW_THREADS cvc::nav::drive_step(fs, ood, tod, sod, cad, model, N, mid, v, mcd,
                                                num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    PyObject *tup = PyTuple_Pack(4, oo, to, so, mc);
    Py_DECREF(oo);
    Py_DECREF(to);
    Py_DECREF(so);
    Py_DECREF(mc);
    return tup;
  }

  // GPU fused drive tick (nav/drive.cu). Same surface as nav_drive_step but runs
  // on the default CUDA device; shared field (plane 0). Returns fresh
  // (o (N,2), th (N,), sp (N,), minclr (N,)) f32. Present always; raises if this
  // pycvc was built without CUDA.
  PyObject *nav_drive_step_cuda(PyObject *field, PyObject *on, PyObject *th, PyObject *sp,
                                PyObject *carrot, const char *weights_path, double min_x,
                                double min_y, double max_x, double max_y, double cx, double cy,
                                double scale, double rr, double d_hat, double dt, double vmax,
                                double L, double delta_max, double a_max, double a_lat_max,
                                double k_steer, int nsub, int allow_reverse) {
#ifdef CVC_USING_CUDA
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
        fail("pycvc.nav_drive_step_cuda: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ta = take(th, NPY_FLOAT, 1);
    PyArrayObject *sa = take(sp, NPY_FLOAT, 1);
    PyArrayObject *ca = take(carrot, NPY_FLOAT, 2);
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    cvc::nav::coef_mlp model = cvc::nav::coef_mlp::load(weights_path);
    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *oo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *to = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *so = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *mc = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    if (!oo || !to || !so || !mc) {
      Py_XDECREF(oo);
      Py_XDECREF(to);
      Py_XDECREF(so);
      Py_XDECREF(mc);
      fail("pycvc.nav_drive_step_cuda: output alloc failed");
    }
    float *ood = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(oo)));
    float *tod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(to)));
    float *sod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(so)));
    float *mcd = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(mc)));
    std::memcpy(ood, PyArray_DATA(oa), sizeof(float) * 2 * N);
    std::memcpy(tod, PyArray_DATA(ta), sizeof(float) * N);
    std::memcpy(sod, PyArray_DATA(sa), sizeof(float) * N);
    cvc::nav::field_stack fs;
    fs.data = static_cast<const float *>(PyArray_DATA(fa));
    fs.M = static_cast<int>(PyArray_DIM(fa, 0));
    fs.H = static_cast<int>(PyArray_DIM(fa, 2));
    fs.W = static_cast<int>(PyArray_DIM(fa, 3));
    fs.mnx = min_x;
    fs.mny = min_y;
    fs.mxx = max_x;
    fs.mxy = max_y;
    fs.cx = cx;
    fs.cy = cy;
    fs.S = scale;
    cvc::nav::veh_params v;
    v.rr = (float)rr;
    v.d_hat = (float)d_hat;
    v.dt = (float)dt;
    v.vmax = (float)vmax;
    v.L = (float)L;
    v.delta_max = (float)delta_max;
    v.a_max = (float)a_max;
    v.a_lat_max = (float)a_lat_max;
    v.k_steer = (float)k_steer;
    v.nsub = nsub;
    v.allow_reverse = allow_reverse != 0;
    const float *cad = static_cast<const float *>(PyArray_DATA(ca));
    Py_BEGIN_ALLOW_THREADS cvc::nav::drive_step_cuda(fs, ood, tod, sod, cad, model, N, v, mcd);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    PyObject *tup = PyTuple_Pack(4, oo, to, so, mc);
    Py_DECREF(oo);
    Py_DECREF(to);
    Py_DECREF(so);
    Py_DECREF(mc);
    return tup;
#else
    (void)field;
    (void)on;
    (void)th;
    (void)sp;
    (void)carrot;
    (void)weights_path;
    throw std::runtime_error("pycvc.nav_drive_step_cuda: this pycvc was built without CUDA");
#endif
  }

  // Self-supervised training of the CoefMLP navigation policy from a scene's
  // occupancy (cvc::nav::coef_train) — NO torch, NO Python numerics. Trains and
  // writes the versioned .cvcnav to `out_path`; returns the final window loss.
  // `occ` is a (H,W) uint8/bool free/obstacle grid; `rollout` is 0 (surrogate) or
  // 1 (bicycle); `use_cuda` picks the device-resident GPU trainer when this pycvc
  // was built with CUDA and a device is present. The CPU path releases the GIL
  // during training. This is the OPT-IN native trainer behind GRL-SNAM's
  // GRL_SNAM_TRAIN_BACKEND flag; the torch coef_train.py stays canonical.
  double nav_train_coef_mlp(PyObject *occ, double min_x, double min_y, double max_x, double max_y,
                            double scale, double rr, double d_hat, double dt, double vmax,
                            int steps, int horizon, int n, int window, int hidden, double lr,
                            double w_coll, double grad_clip, unsigned seed, int rollout,
                            int use_cuda, const char *out_path) {
    int rows, cols;
    PyArrayObject *a = pycvc_nav_as_u8(occ, rows, cols);
    cvc::nav::training_scene sc = cvc::nav::occupancy_scene(
        static_cast<const std::uint8_t *>(PyArray_DATA(a)), rows, cols, min_x, min_y, max_x, max_y,
        scale, (float)rr, (float)d_hat, (float)dt, (float)vmax);
    Py_DECREF(a);

    cvc::nav::train_config cfg;
    cfg.steps = steps;
    cfg.horizon = horizon;
    cfg.n = n;
    cfg.window = window;
    cfg.hidden = hidden;
    cfg.lr = (float)lr;
    cfg.w_coll = (float)w_coll;
    cfg.grad_clip = (float)grad_clip;
    cfg.seed = seed;
    cfg.rollout =
        rollout == 1 ? cvc::nav::rollout_kind::bicycle : cvc::nav::rollout_kind::surrogate;

    bool used_cuda = false;
#ifdef CVC_USING_CUDA
    if (use_cuda && cvc::nav::train_cuda_available()) {
      cvc::nav::coef_mlp policy = cvc::nav::train_coef_mlp_cuda(sc, cfg);
      policy.save(out_path, "trained by pycvc.nav_train_coef_mlp (cuda)");
      used_cuda = true;
    }
#else
    (void)use_cuda;
#endif
    if (!used_cuda) {
      cvc::nav::coef_trainer tr(cfg, 1);
      Py_BEGIN_ALLOW_THREADS tr.train(sc);
      Py_END_ALLOW_THREADS tr.to_coef_mlp().save(out_path,
                                                 "trained by pycvc.nav_train_coef_mlp (cpu)");
    }
    return used_cuda ? 1.0 : 0.0; // which backend ran (weights are on disk at out_path)
  }

  // ── material-aware navigation (cvc/nav/material.h) ──────────────────────────
  // NEW symbols (never a re-signatured existing one): GRL-SNAM keys its
  // HAS_MATERIAL capability off nav_witness_gate and HAS_MATERIAL_DRIVE off
  // nav_drive_step_material, so version skew degrades to the Python path.

  // (H,W) f32 raw risk + (H,W) u8 hard -> (6,H,W) f32 derived material planes
  // [r~, phi_m, grad_rx, grad_ry, grad_px, grad_py] — BIT-identical to
  // grl_snam.material.MaterialGrid._derive.
  ArrayView nav_material_build(PyObject *risk_raw, PyObject *hard, double cell_w, double scale,
                               double sigma) {
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
        fail("pycvc.nav_material_build: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *ra = take(risk_raw, NPY_FLOAT, 2);
    PyArrayObject *ha = take(hard, NPY_UINT8, 2);
    const int rows = static_cast<int>(PyArray_DIM(ra, 0));
    const int cols = static_cast<int>(PyArray_DIM(ra, 1));
    if (PyArray_DIM(ha, 0) != rows || PyArray_DIM(ha, 1) != cols)
      fail("pycvc.nav_material_build: risk and hard shapes differ");
    // Pre-validate the kernel's own throwing precondition HERE: a C++ exception
    // unwinding through Py_BEGIN_ALLOW_THREADS leaves the GIL unrestored.
    const int radius = static_cast<int>(4.0 * sigma + 0.5);
    if (sigma > 0.0 && (rows < radius + 1 || cols < radius + 1))
      fail("pycvc.nav_material_build: grid smaller than blur radius + 1");
    cvc::nav::material_planes mp;
    const float *rd = static_cast<const float *>(PyArray_DATA(ra));
    const std::uint8_t *hd = static_cast<const std::uint8_t *>(PyArray_DATA(ha));
    Py_BEGIN_ALLOW_THREADS mp = cvc::nav::material_build(rd, hd, rows, cols, cell_w, scale, sigma);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    std::vector<float> out = mp.stacked();
    return pycvc_nav_view<float>(std::move(out), {6, (long)rows, (long)cols}, DType::Float32);
  }

  // Frame-wise feasibility witness (BIT-identical to grl_snam.material.witness_gate).
  // Positions are CONTINUOUS CELL coords (row, col f64). gate_hard must already
  // include occupancy; clear_m is its metres clearance plane. Returns (10,) f64:
  // [active, nominal, best, feasible_count, dir_r, dir_c, end_r, end_c,
  //  min_clearance_m, 0].
  ArrayView nav_witness_gate(PyObject *risk, PyObject *gate_hard, PyObject *clear_m, double pos_r,
                             double pos_c, double goal_r, double goal_c, int horizon_cells,
                             double hard_margin_m, int primitive_count, double improvement_margin,
                             double material_trigger, double progress_slack_cells) {
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
        fail("pycvc.nav_witness_gate: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *ra = take(risk, NPY_FLOAT, 2);
    PyArrayObject *ha = take(gate_hard, NPY_UINT8, 2);
    PyArrayObject *ca = take(clear_m, NPY_FLOAT, 2);
    const int rows = static_cast<int>(PyArray_DIM(ra, 0));
    const int cols = static_cast<int>(PyArray_DIM(ra, 1));
    if (PyArray_DIM(ha, 0) != rows || PyArray_DIM(ha, 1) != cols || PyArray_DIM(ca, 0) != rows ||
        PyArray_DIM(ca, 1) != cols)
      fail("pycvc.nav_witness_gate: risk/gate_hard/clear_m shapes differ");
    cvc::nav::gate_params gp;
    gp.primitive_count = primitive_count;
    gp.horizon_cells = horizon_cells;
    gp.hard_margin_m = hard_margin_m;
    gp.improvement_margin = improvement_margin;
    gp.material_trigger = material_trigger;
    gp.progress_slack_cells = progress_slack_cells;
    const cvc::nav::gate_decision g = cvc::nav::witness_gate(
        static_cast<const float *>(PyArray_DATA(ra)),
        static_cast<const std::uint8_t *>(PyArray_DATA(ha)),
        static_cast<const float *>(PyArray_DATA(ca)), rows, cols, pos_r, pos_c, goal_r, goal_c, gp);
    for (PyArrayObject *h : hold)
      Py_DECREF(h);
    std::vector<double> out = {g.active ? 1.0 : 0.0,
                               g.nominal_risk,
                               g.best_risk,
                               static_cast<double>(g.feasible_count),
                               g.dir_r,
                               g.dir_c,
                               g.end_r,
                               g.end_c,
                               g.min_clearance_m,
                               0.0};
    return pycvc_nav_view<double>(std::move(out), {10}, DType::Float64);
  }

  // Batched witness gate over (N,2) f64 cell-coord positions/goals. Returns
  // (N,4) f64 [active, nominal, best, feasible_count]; byte-identical to N
  // serial gates. GIL released; threads across agents.
  ArrayView nav_witness_gate_batch(PyObject *risk, PyObject *gate_hard, PyObject *clear_m,
                                   PyObject *pos_rc, PyObject *goal_rc, int horizon_cells,
                                   double hard_margin_m, int primitive_count,
                                   double improvement_margin, double material_trigger,
                                   double progress_slack_cells, int num_threads = 0) {
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
        fail("pycvc.nav_witness_gate_batch: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *ra = take(risk, NPY_FLOAT, 2);
    PyArrayObject *ha = take(gate_hard, NPY_UINT8, 2);
    PyArrayObject *ca = take(clear_m, NPY_FLOAT, 2);
    PyArrayObject *pa = take(pos_rc, NPY_DOUBLE, 2);
    PyArrayObject *ga = take(goal_rc, NPY_DOUBLE, 2);
    const int rows = static_cast<int>(PyArray_DIM(ra, 0));
    const int cols = static_cast<int>(PyArray_DIM(ra, 1));
    const int N = static_cast<int>(PyArray_DIM(pa, 0));
    if (PyArray_DIM(ha, 0) != rows || PyArray_DIM(ha, 1) != cols || PyArray_DIM(ca, 0) != rows ||
        PyArray_DIM(ca, 1) != cols)
      fail("pycvc.nav_witness_gate_batch: risk/gate_hard/clear_m shapes differ");
    if (PyArray_DIM(pa, 1) != 2 || PyArray_DIM(ga, 0) != N || PyArray_DIM(ga, 1) != 2)
      fail("pycvc.nav_witness_gate_batch: pos_rc/goal_rc must be (N,2)");
    cvc::nav::gate_params gp;
    gp.primitive_count = primitive_count;
    gp.horizon_cells = horizon_cells;
    gp.hard_margin_m = hard_margin_m;
    gp.improvement_margin = improvement_margin;
    gp.material_trigger = material_trigger;
    gp.progress_slack_cells = progress_slack_cells;
    std::vector<std::uint8_t> act(N);
    std::vector<double> nom(N), best(N);
    std::vector<std::int32_t> cnt(N);
    const float *rd = static_cast<const float *>(PyArray_DATA(ra));
    const std::uint8_t *hd = static_cast<const std::uint8_t *>(PyArray_DATA(ha));
    const float *cd = static_cast<const float *>(PyArray_DATA(ca));
    const double *pd = static_cast<const double *>(PyArray_DATA(pa));
    const double *gd = static_cast<const double *>(PyArray_DATA(ga));
    Py_BEGIN_ALLOW_THREADS cvc::nav::witness_gate_batch(rd, hd, cd, rows, cols, pd, gd, N, gp,
                                                        act.data(), nom.data(), best.data(),
                                                        cnt.data(), num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    std::vector<double> out(static_cast<std::size_t>(N) * 4);
    for (int i = 0; i < N; ++i) {
      out[4 * i + 0] = act[i] ? 1.0 : 0.0;
      out[4 * i + 1] = nom[i];
      out[4 * i + 2] = best[i];
      out[4 * i + 3] = static_cast<double>(cnt[i]);
    }
    return pycvc_nav_view<double>(std::move(out), {(long)N, 4}, DType::Float64);
  }

  // (M,6,H,W) f32 material stack sampled at (N,2) f32 normalized positions ->
  // (N,6) f32 [risk, phi_m, grad_rx, grad_ry, grad_px, grad_py]. FLOAT tier
  // (the sdf_sample op chain). GIL released.
  ArrayView nav_material_sample(PyObject *field, double min_x, double min_y, double max_x,
                                double max_y, double cx, double cy, double scale, PyObject *on,
                                int num_threads = 0) {
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
        fail("pycvc.nav_material_sample: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    if (PyArray_DIM(fa, 1) != 6)
      fail("pycvc.nav_material_sample: field must be (M,6,H,W)");
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(oa, 1) != 2)
      fail("pycvc.nav_material_sample: on must be (N,2)");
    cvc::nav::material_stack ms;
    ms.data = static_cast<const float *>(PyArray_DATA(fa));
    ms.M = static_cast<int>(PyArray_DIM(fa, 0));
    ms.H = static_cast<int>(PyArray_DIM(fa, 2));
    ms.W = static_cast<int>(PyArray_DIM(fa, 3));
    ms.mnx = min_x;
    ms.mny = min_y;
    ms.mxx = max_x;
    ms.mxy = max_y;
    ms.cx = cx;
    ms.cy = cy;
    ms.S = scale;
    std::vector<float> risk(N), phi(N), gr(static_cast<std::size_t>(2) * N),
        gp(static_cast<std::size_t>(2) * N);
    const float *ond = static_cast<const float *>(PyArray_DATA(oa));
    Py_BEGIN_ALLOW_THREADS cvc::nav::material_sample(ms, ond, N, nullptr, risk.data(), phi.data(),
                                                     gr.data(), gp.data(), num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    std::vector<float> out(static_cast<std::size_t>(N) * 6);
    for (int i = 0; i < N; ++i) {
      out[6 * i + 0] = risk[i];
      out[6 * i + 1] = phi[i];
      out[6 * i + 2] = gr[2 * i];
      out[6 * i + 3] = gr[2 * i + 1];
      out[6 * i + 4] = gp[2 * i];
      out[6 * i + 5] = gp[2 * i + 1];
    }
    return pycvc_nav_view<float>(std::move(out), {(long)N, 6}, DType::Float32);
  }

  // nav_bicycle_rollout with the material coupling: extra inputs are the
  // (Mm,6,H,W) material stack (same world transform as the field), the [N]
  // EFFECTIVE lam_soft (gate already multiplied in) and [N] lam_hard columns,
  // and the barrier constants. Returns the same (o, th, sp, minclr) tuple.
  PyObject *nav_bicycle_rollout_material(
      PyObject *field, PyObject *on, PyObject *th, PyObject *sp, PyObject *goal, PyObject *al,
      PyObject *be, PyObject *ga, PyObject *mat_field, PyObject *lam_soft, PyObject *lam_hard,
      double mat_k_sharp, double mat_d_hat_m, PyObject *map_id, double min_x, double min_y,
      double max_x, double max_y, double cx, double cy, double scale, double rr, double d_hat,
      double dt, double vmax, double L, double delta_max, double a_max, double a_lat_max,
      double k_steer, int nsub, int allow_reverse, int num_threads = 0,
      PyObject *body_offsets = nullptr, double body_rr = 0.0, double body_gain = 1.0,
      double track_width = 0.0, PyObject *grip = nullptr) {
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
        fail("pycvc.nav_bicycle_rollout_material: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ta = take(th, NPY_FLOAT, 1);
    PyArrayObject *sa = take(sp, NPY_FLOAT, 1);
    PyArrayObject *gla = take(goal, NPY_FLOAT, 2);
    PyArrayObject *aa = take(al, NPY_FLOAT, 1);
    PyArrayObject *ba = take(be, NPY_FLOAT, 1);
    PyArrayObject *gaa = take(ga, NPY_FLOAT, 1);
    PyArrayObject *ma = take(mat_field, NPY_FLOAT, 4);
    PyArrayObject *lsa = take(lam_soft, NPY_FLOAT, 1);
    PyArrayObject *lha = take(lam_hard, NPY_FLOAT, 1);
    const int M = static_cast<int>(PyArray_DIM(fa, 0));
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(fa, 1) != 3)
      fail("pycvc.nav_bicycle_rollout_material: field must be (M,3,H,W)");
    if (PyArray_DIM(ma, 1) != 6)
      fail("pycvc.nav_bicycle_rollout_material: mat_field must be (Mm,6,H,W)");
    if (PyArray_DIM(oa, 1) != 2 || PyArray_DIM(ta, 0) != N || PyArray_DIM(sa, 0) != N ||
        PyArray_DIM(gla, 0) != N || PyArray_DIM(gla, 1) != 2 || PyArray_DIM(aa, 0) != N ||
        PyArray_DIM(ba, 0) != N || PyArray_DIM(gaa, 0) != N || PyArray_DIM(lsa, 0) != N ||
        PyArray_DIM(lha, 0) != N)
      fail("pycvc.nav_bicycle_rollout_material: pose / coeff / lambda arrays must all be length N");
    const int Mm = static_cast<int>(PyArray_DIM(ma, 0));
    const int *mid = nullptr;
    if (map_id && map_id != Py_None) {
      PyArrayObject *mia = take(map_id, NPY_INT32, 1);
      if (PyArray_DIM(mia, 0) != N)
        fail("pycvc.nav_bicycle_rollout_material: map_id must be (N,)");
      const std::int32_t *m = static_cast<const std::int32_t *>(PyArray_DATA(mia));
      for (int i = 0; i < N; ++i)
        if (m[i] < 0 || m[i] >= M || (Mm > 1 && m[i] >= Mm))
          fail("pycvc.nav_bicycle_rollout_material: map_id has an out-of-range plane index");
      mid = m;
    }
    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *oo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *to = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *so = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *mc = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    if (!oo || !to || !so || !mc) {
      Py_XDECREF(oo);
      Py_XDECREF(to);
      Py_XDECREF(so);
      Py_XDECREF(mc);
      fail("pycvc.nav_bicycle_rollout_material: output alloc failed");
    }
    float *ood = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(oo)));
    float *tod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(to)));
    float *sod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(so)));
    float *mcd = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(mc)));
    std::memcpy(ood, PyArray_DATA(oa), sizeof(float) * 2 * N);
    std::memcpy(tod, PyArray_DATA(ta), sizeof(float) * N);
    std::memcpy(sod, PyArray_DATA(sa), sizeof(float) * N);
    cvc::nav::field_stack fs;
    fs.data = static_cast<const float *>(PyArray_DATA(fa));
    fs.M = M;
    fs.H = static_cast<int>(PyArray_DIM(fa, 2));
    fs.W = static_cast<int>(PyArray_DIM(fa, 3));
    fs.mnx = min_x;
    fs.mny = min_y;
    fs.mxx = max_x;
    fs.mxy = max_y;
    fs.cx = cx;
    fs.cy = cy;
    fs.S = scale;
    cvc::nav::material_stack ms;
    ms.data = static_cast<const float *>(PyArray_DATA(ma));
    ms.M = Mm;
    ms.H = static_cast<int>(PyArray_DIM(ma, 2));
    ms.W = static_cast<int>(PyArray_DIM(ma, 3));
    ms.mnx = min_x;
    ms.mny = min_y;
    ms.mxx = max_x;
    ms.mxy = max_y;
    ms.cx = cx;
    ms.cy = cy;
    ms.S = scale;
    cvc::nav::material_drive md;
    md.stack = &ms;
    md.lam_soft = static_cast<const float *>(PyArray_DATA(lsa));
    md.lam_hard = static_cast<const float *>(PyArray_DATA(lha));
    md.k_sharp = static_cast<float>(mat_k_sharp);
    md.d_hat_m = static_cast<float>(mat_d_hat_m);
    cvc::nav::veh_params v;
    v.rr = static_cast<float>(rr);
    v.d_hat = static_cast<float>(d_hat);
    v.dt = static_cast<float>(dt);
    v.vmax = static_cast<float>(vmax);
    v.L = static_cast<float>(L);
    v.delta_max = static_cast<float>(delta_max);
    v.a_max = static_cast<float>(a_max);
    v.a_lat_max = static_cast<float>(a_lat_max);
    v.k_steer = static_cast<float>(k_steer);
    v.nsub = nsub;
    v.allow_reverse = allow_reverse != 0;
    // Optional vehicle refinements; absent (None / 0) leaves the legacy drive
    // bit-for-bit. The arrays are borrowed for the call, kept alive by `hold`.
    v.body_rr = static_cast<float>(body_rr);
    v.body_gain = static_cast<float>(body_gain);
    v.track_width = static_cast<float>(track_width);
    if (body_offsets && body_offsets != Py_None) {
      PyArrayObject *boa = take(body_offsets, NPY_FLOAT, 1);
      v.body_offsets = static_cast<const float *>(PyArray_DATA(boa));
      v.n_body = static_cast<int>(PyArray_DIM(boa, 0));
      if (v.n_body > 0 && !(v.body_rr > 0.0f))
        fail("nav_bicycle_rollout: body_offsets needs a positive body_rr");
    }
    cvc::nav::friction_field gf;
    if (grip && grip != Py_None) {
      // The grip raster shares the FIELD's world frame — same bounds, center and
      // scale. Anything else would need its own seven constants on an already
      // long signature, and every caller builds both from one scenario anyway.
      PyArrayObject *gpa = take(grip, NPY_FLOAT, 2);
      gf.data = static_cast<const float *>(PyArray_DATA(gpa));
      gf.M = 1;
      gf.H = static_cast<int>(PyArray_DIM(gpa, 0));
      gf.W = static_cast<int>(PyArray_DIM(gpa, 1));
      gf.mnx = min_x;
      gf.mny = min_y;
      gf.mxx = max_x;
      gf.mxy = max_y;
      gf.cx = cx;
      gf.cy = cy;
      gf.S = scale;
      v.grip = &gf;
    }
    const float *gld = static_cast<const float *>(PyArray_DATA(gla));
    const float *ald = static_cast<const float *>(PyArray_DATA(aa));
    const float *bed = static_cast<const float *>(PyArray_DATA(ba));
    const float *gad = static_cast<const float *>(PyArray_DATA(gaa));
    Py_BEGIN_ALLOW_THREADS cvc::nav::bicycle_rollout_material(fs, ood, tod, sod, gld, ald, bed, gad,
                                                              N, mid, v, md, mcd, num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    PyObject *tuple = PyTuple_Pack(4, oo, to, so, mc);
    Py_DECREF(oo);
    Py_DECREF(to);
    Py_DECREF(so);
    Py_DECREF(mc);
    return tuple;
  }

  // The FUSED torch-free material drive: nav_drive_step (loads the coef_mlp from
  // weights_path, samples -> coef_feats -> forward -> bicycle) plus the material
  // coupling of nav_bicycle_rollout_material. This is what a torch-free host (or
  // GRL-SNAM's GRL_SNAM_NAV_DRIVE=native) calls to drive material-aware agents
  // without libtorch. Returns fresh (o (N,2), th (N,), sp (N,), minclr (N,)) f32.
  PyObject *nav_drive_step_material(PyObject *field, PyObject *on, PyObject *th, PyObject *sp,
                                    PyObject *carrot, const char *weights_path, PyObject *mat_field,
                                    PyObject *lam_soft, PyObject *lam_hard, double mat_k_sharp,
                                    double mat_d_hat_m, PyObject *map_id, double min_x,
                                    double min_y, double max_x, double max_y, double cx, double cy,
                                    double scale, double rr, double d_hat, double dt, double vmax,
                                    double L, double delta_max, double a_max, double a_lat_max,
                                    double k_steer, int nsub, int allow_reverse,
                                    int num_threads = 0) {
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
        fail("pycvc.nav_drive_step_material: an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *fa = take(field, NPY_FLOAT, 4);
    PyArrayObject *oa = take(on, NPY_FLOAT, 2);
    PyArrayObject *ta = take(th, NPY_FLOAT, 1);
    PyArrayObject *sa = take(sp, NPY_FLOAT, 1);
    PyArrayObject *ca = take(carrot, NPY_FLOAT, 2);
    PyArrayObject *ma = take(mat_field, NPY_FLOAT, 4);
    PyArrayObject *lsa = take(lam_soft, NPY_FLOAT, 1);
    PyArrayObject *lha = take(lam_hard, NPY_FLOAT, 1);
    const int M = static_cast<int>(PyArray_DIM(fa, 0));
    const int N = static_cast<int>(PyArray_DIM(oa, 0));
    if (PyArray_DIM(fa, 1) != 3)
      fail("pycvc.nav_drive_step_material: field must be (M,3,H,W)");
    if (PyArray_DIM(ma, 1) != 6)
      fail("pycvc.nav_drive_step_material: mat_field must be (Mm,6,H,W)");
    if (PyArray_DIM(oa, 1) != 2 || PyArray_DIM(ta, 0) != N || PyArray_DIM(sa, 0) != N ||
        PyArray_DIM(ca, 0) != N || PyArray_DIM(ca, 1) != 2 || PyArray_DIM(lsa, 0) != N ||
        PyArray_DIM(lha, 0) != N)
      fail("pycvc.nav_drive_step_material: pose / carrot / lambda arrays must all be length N");
    const int Mm = static_cast<int>(PyArray_DIM(ma, 0));
    const int *mid = nullptr;
    if (map_id && map_id != Py_None) {
      PyArrayObject *mia = take(map_id, NPY_INT32, 1);
      if (PyArray_DIM(mia, 0) != N)
        fail("pycvc.nav_drive_step_material: map_id must be (N,)");
      const std::int32_t *m = static_cast<const std::int32_t *>(PyArray_DATA(mia));
      for (int i = 0; i < N; ++i)
        if (m[i] < 0 || m[i] >= M || (Mm > 1 && m[i] >= Mm))
          fail("pycvc.nav_drive_step_material: map_id has an out-of-range plane index");
      mid = m;
    }
    cvc::nav::coef_mlp model = cvc::nav::coef_mlp::load(weights_path); // throws on bad/stale file

    npy_intp d2[2] = {N, 2}, d1 = N;
    PyObject *oo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
    PyObject *to = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *so = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    PyObject *mc = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
    if (!oo || !to || !so || !mc) {
      Py_XDECREF(oo);
      Py_XDECREF(to);
      Py_XDECREF(so);
      Py_XDECREF(mc);
      fail("pycvc.nav_drive_step_material: output alloc failed");
    }
    float *ood = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(oo)));
    float *tod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(to)));
    float *sod = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(so)));
    float *mcd = static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(mc)));
    std::memcpy(ood, PyArray_DATA(oa), sizeof(float) * 2 * N);
    std::memcpy(tod, PyArray_DATA(ta), sizeof(float) * N);
    std::memcpy(sod, PyArray_DATA(sa), sizeof(float) * N);

    cvc::nav::field_stack fs;
    fs.data = static_cast<const float *>(PyArray_DATA(fa));
    fs.M = M;
    fs.H = static_cast<int>(PyArray_DIM(fa, 2));
    fs.W = static_cast<int>(PyArray_DIM(fa, 3));
    fs.mnx = min_x;
    fs.mny = min_y;
    fs.mxx = max_x;
    fs.mxy = max_y;
    fs.cx = cx;
    fs.cy = cy;
    fs.S = scale;
    cvc::nav::material_stack ms;
    ms.data = static_cast<const float *>(PyArray_DATA(ma));
    ms.M = Mm;
    ms.H = static_cast<int>(PyArray_DIM(ma, 2));
    ms.W = static_cast<int>(PyArray_DIM(ma, 3));
    ms.mnx = min_x;
    ms.mny = min_y;
    ms.mxx = max_x;
    ms.mxy = max_y;
    ms.cx = cx;
    ms.cy = cy;
    ms.S = scale;
    cvc::nav::material_drive md;
    md.stack = &ms;
    md.lam_soft = static_cast<const float *>(PyArray_DATA(lsa));
    md.lam_hard = static_cast<const float *>(PyArray_DATA(lha));
    md.k_sharp = static_cast<float>(mat_k_sharp);
    md.d_hat_m = static_cast<float>(mat_d_hat_m);
    cvc::nav::veh_params v;
    v.rr = static_cast<float>(rr);
    v.d_hat = static_cast<float>(d_hat);
    v.dt = static_cast<float>(dt);
    v.vmax = static_cast<float>(vmax);
    v.L = static_cast<float>(L);
    v.delta_max = static_cast<float>(delta_max);
    v.a_max = static_cast<float>(a_max);
    v.a_lat_max = static_cast<float>(a_lat_max);
    v.k_steer = static_cast<float>(k_steer);
    v.nsub = nsub;
    v.allow_reverse = allow_reverse != 0;
    const float *cad = static_cast<const float *>(PyArray_DATA(ca));
    Py_BEGIN_ALLOW_THREADS cvc::nav::drive_step_material(fs, ood, tod, sod, cad, model, N, mid, v,
                                                         md, mcd, num_threads);
    Py_END_ALLOW_THREADS for (PyArrayObject *h : hold) Py_DECREF(h);
    PyObject *tuple = PyTuple_Pack(4, oo, to, so, mc);
    Py_DECREF(oo);
    Py_DECREF(to);
    Py_DECREF(so);
    Py_DECREF(mc);
    return tuple;
  }

// Learned material coefficient network forward (cvc::nav::coef_energy_net).
// Batched over n agents with ragged obstacle lists. Inputs: weights_path
// (.cvcnm), obs_feats (total,6) f32, obs_mask (total,) u8, obs_offsets (n+1,)
// i32, goal_feats (n,4) f32, risk_patch (n,2,P,P) f32. Returns tuple
// (alphas (total,), beta (n,), gamma (n,), lam_soft (n,), lam_hard (n,),
// mu_lat (n,)) f32. Float-equivalent to the torch CoefEnergyNetMaterial (math
// attention path). GIL released across the compute.
PyObject *nav_matnet_forward(const char *weights_path, PyObject *obs_feats, PyObject *obs_mask,
                             PyObject *obs_offsets, PyObject *goal_feats, PyObject *risk_patch,
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
      fail("pycvc.nav_matnet_forward: an input array had the wrong dtype/rank");
    hold.push_back(a);
    return a;
  };
  PyArrayObject *ofa = take(obs_feats, NPY_FLOAT, 2);
  PyArrayObject *oma = take(obs_mask, NPY_UINT8, 1);
  PyArrayObject *oo = take(obs_offsets, NPY_INT32, 1);
  PyArrayObject *gfa = take(goal_feats, NPY_FLOAT, 2);
  PyArrayObject *rpa = take(risk_patch, NPY_FLOAT, 4);
  const int total = static_cast<int>(PyArray_DIM(ofa, 0));
  const int n = static_cast<int>(PyArray_DIM(gfa, 0));
  const int P = static_cast<int>(PyArray_DIM(rpa, 2));
  if (PyArray_DIM(ofa, 1) != 6 || PyArray_DIM(oma, 0) != total || PyArray_DIM(oo, 0) != n + 1 ||
      PyArray_DIM(gfa, 1) != 4 || PyArray_DIM(rpa, 0) != n || PyArray_DIM(rpa, 1) != 2 ||
      PyArray_DIM(rpa, 3) != P)
    fail("pycvc.nav_matnet_forward: shape mismatch (obs (T,6) mask (T,) offs (n+1,) goal (n,4) "
         "patch (n,2,P,P))");
  const int *offs = static_cast<const int *>(PyArray_DATA(oo));
  if (offs[0] != 0 || offs[n] != total)
    fail("pycvc.nav_matnet_forward: obs_offsets must start at 0 and end at total");

  cvc::nav::coef_energy_net model = cvc::nav::coef_energy_net::load(weights_path);
  if (model.patch_size() != P)
    fail("pycvc.nav_matnet_forward: risk_patch P does not match the model's patch_size");

  npy_intp dt = total, dn = n;
  PyObject *al = PyArray_SimpleNew(1, &dt, NPY_FLOAT);
  PyObject *be = PyArray_SimpleNew(1, &dn, NPY_FLOAT);
  PyObject *ga = PyArray_SimpleNew(1, &dn, NPY_FLOAT);
  PyObject *ls = PyArray_SimpleNew(1, &dn, NPY_FLOAT);
  PyObject *lh = PyArray_SimpleNew(1, &dn, NPY_FLOAT);
  PyObject *ml = PyArray_SimpleNew(1, &dn, NPY_FLOAT);
  if (!al || !be || !ga || !ls || !lh || !ml) {
    Py_XDECREF(al);
    Py_XDECREF(be);
    Py_XDECREF(ga);
    Py_XDECREF(ls);
    Py_XDECREF(lh);
    Py_XDECREF(ml);
    fail("pycvc.nav_matnet_forward: output alloc failed");
  }
  auto D = [](PyObject *o) {
    return static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(o)));
  };
  const float *ofd = static_cast<const float *>(PyArray_DATA(ofa));
  const std::uint8_t *omd = static_cast<const std::uint8_t *>(PyArray_DATA(oma));
  const float *gfd = static_cast<const float *>(PyArray_DATA(gfa));
  const float *rpd = static_cast<const float *>(PyArray_DATA(rpa));
  Py_BEGIN_ALLOW_THREADS
  model.forward_batch(ofd, omd, offs, n, gfd, rpd, P, D(al), D(be), D(ga), D(ls), D(lh), D(ml),
                      num_threads);
  Py_END_ALLOW_THREADS
  for (PyArrayObject *h : hold)
    Py_DECREF(h);
  PyObject *tup = PyTuple_Pack(6, al, be, ga, ls, lh, ml);
  Py_DECREF(al);
  Py_DECREF(be);
  Py_DECREF(ga);
  Py_DECREF(ls);
  Py_DECREF(lh);
  Py_DECREF(ml);
  return tup;
}

// Obstacle-list material surrogate rollout (cvc::nav::integrate_surrogate_material).
// The faithful torch-free integrator of the source material method (per-obstacle
// IPC barriers + soft/hard material forces + semi-implicit Euler). B agents, N
// padded obstacles (masked), 6-channel patch (B,6,Hp,Wp). Returns fresh
// (oT (B,2), vT (B,2), min_clear (B,), cum_risk (B,), hard_count (B,),
// arc_length (B,)) f32. GIL released.
PyObject *nav_integrate_surrogate_material(
    PyObject *o0, PyObject *v0, PyObject *goal, PyObject *C, PyObject *R, PyObject *mask,
    PyObject *alphas, PyObject *beta, PyObject *gamma, PyObject *lam_soft, PyObject *lam_hard,
    PyObject *rollout_patch, PyObject *rr, PyObject *d_hat, PyObject *dt, PyObject *H,
    double margin_factor, double mass, double d_hat_sdf, double k_sharp, int num_threads = 0)
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
      fail("pycvc.nav_integrate_surrogate_material: an input array had the wrong dtype/rank");
    hold.push_back(a);
    return a;
  };
  PyArrayObject *o0a = take(o0, NPY_FLOAT, 2);
  PyArrayObject *v0a = take(v0, NPY_FLOAT, 2);
  PyArrayObject *ga = take(goal, NPY_FLOAT, 2);
  PyArrayObject *Ca = take(C, NPY_FLOAT, 3);
  PyArrayObject *Ra = take(R, NPY_FLOAT, 2);
  PyArrayObject *ma = take(mask, NPY_UINT8, 2);
  PyArrayObject *aa = take(alphas, NPY_FLOAT, 2);
  PyArrayObject *ba = take(beta, NPY_FLOAT, 1);
  PyArrayObject *gma = take(gamma, NPY_FLOAT, 1);
  PyArrayObject *lsa = take(lam_soft, NPY_FLOAT, 1);
  PyArrayObject *lha = take(lam_hard, NPY_FLOAT, 1);
  PyArrayObject *pa = take(rollout_patch, NPY_FLOAT, 4);
  PyArrayObject *rra = take(rr, NPY_FLOAT, 1);
  PyArrayObject *dha = take(d_hat, NPY_FLOAT, 1);
  PyArrayObject *dta = take(dt, NPY_FLOAT, 1);
  PyArrayObject *Ha = take(H, NPY_INT32, 1);
  const int B = static_cast<int>(PyArray_DIM(o0a, 0));
  const int N = static_cast<int>(PyArray_DIM(Ca, 1));
  const int Hp = static_cast<int>(PyArray_DIM(pa, 2));
  const int Wp = static_cast<int>(PyArray_DIM(pa, 3));
  if (PyArray_DIM(o0a, 1) != 2 || PyArray_DIM(v0a, 0) != B || PyArray_DIM(ga, 0) != B ||
      PyArray_DIM(Ca, 0) != B || PyArray_DIM(Ca, 2) != 2 || PyArray_DIM(Ra, 0) != B ||
      PyArray_DIM(Ra, 1) != N || PyArray_DIM(ma, 0) != B || PyArray_DIM(ma, 1) != N ||
      PyArray_DIM(aa, 0) != B || PyArray_DIM(aa, 1) != N || PyArray_DIM(ba, 0) != B ||
      PyArray_DIM(gma, 0) != B || PyArray_DIM(lsa, 0) != B || PyArray_DIM(lha, 0) != B ||
      PyArray_DIM(pa, 0) != B || PyArray_DIM(pa, 1) != 6 || PyArray_DIM(rra, 0) != B ||
      PyArray_DIM(dha, 0) != B || PyArray_DIM(dta, 0) != B || PyArray_DIM(Ha, 0) != B)
    fail("pycvc.nav_integrate_surrogate_material: shape mismatch");

  npy_intp d2[2] = {B, 2}, d1 = B;
  PyObject *oo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
  PyObject *vo = PyArray_SimpleNew(2, d2, NPY_FLOAT);
  PyObject *mco = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
  PyObject *cro = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
  PyObject *hco = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
  PyObject *alo = PyArray_SimpleNew(1, &d1, NPY_FLOAT);
  if (!oo || !vo || !mco || !cro || !hco || !alo) {
    Py_XDECREF(oo);
    Py_XDECREF(vo);
    Py_XDECREF(mco);
    Py_XDECREF(cro);
    Py_XDECREF(hco);
    Py_XDECREF(alo);
    fail("pycvc.nav_integrate_surrogate_material: output alloc failed");
  }
  auto D = [](PyObject *o) {
    return static_cast<float *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(o)));
  };
  std::memcpy(D(oo), PyArray_DATA(o0a), sizeof(float) * 2 * B);
  std::memcpy(D(vo), PyArray_DATA(v0a), sizeof(float) * 2 * B);
  cvc::nav::surrogate_material_params p;
  p.margin_factor = static_cast<float>(margin_factor);
  p.mass = static_cast<float>(mass);
  p.d_hat_sdf = static_cast<float>(d_hat_sdf);
  p.k_sharp = static_cast<float>(k_sharp);
  Py_BEGIN_ALLOW_THREADS
  cvc::nav::integrate_surrogate_material(
      D(oo), D(vo), static_cast<const float *>(PyArray_DATA(ga)),
      static_cast<const float *>(PyArray_DATA(Ca)), static_cast<const float *>(PyArray_DATA(Ra)),
      static_cast<const std::uint8_t *>(PyArray_DATA(ma)),
      static_cast<const float *>(PyArray_DATA(aa)), static_cast<const float *>(PyArray_DATA(ba)),
      static_cast<const float *>(PyArray_DATA(gma)), static_cast<const float *>(PyArray_DATA(lsa)),
      static_cast<const float *>(PyArray_DATA(lha)), static_cast<const float *>(PyArray_DATA(pa)),
      static_cast<const float *>(PyArray_DATA(rra)), static_cast<const float *>(PyArray_DATA(dha)),
      static_cast<const float *>(PyArray_DATA(dta)),
      static_cast<const int *>(PyArray_DATA(Ha)), B, N, Hp, Wp, p, D(mco), D(cro), D(hco), D(alo),
      num_threads);
  Py_END_ALLOW_THREADS
  for (PyArrayObject *h : hold)
    Py_DECREF(h);
  PyObject *tup = PyTuple_Pack(6, oo, vo, mco, cro, hco, alo);
  Py_DECREF(oo);
  Py_DECREF(vo);
  Py_DECREF(mco);
  Py_DECREF(cro);
  Py_DECREF(hco);
  Py_DECREF(alo);
  return tup;
}

// ── torch-free material trainer (coef_energy_net + Adam, held server-side) ────
// A stateful handle: create from a .cvcnm checkpoint (the initial/warm-start
// weights matnet_export.py writes), drive it with numpy batches, save the
// trained weights. The dataset stays in Python; only the material_batch arrays
// cross into C++, where the whole loss+backward+Adam step runs GIL-free.
// `use_cuda` routes the four heavy ops (model forward/backward, rollout
// forward/VJP) through their device twins inside material_loss_and_grad. It is a
// REQUEST, not a guarantee: each op falls back to its host twin when the build has
// no CUDA, no device is present, or the batch exceeds a kernel's limits — so
// nav_material_trainer_cuda_active() reports what the handle will actually use.
// The Adam step stays on the host optimizer (material_adam), which owns the
// weights the .cvcnm save writes.
struct MaterialTrainer {
  cvc::nav::coef_energy_net model;
  cvc::nav::material_adam opt;
  cvc::nav::material_loss_config cfg;
  bool use_cuda = false;
  MaterialTrainer(cvc::nav::coef_energy_net m, float grad_clip)
      : model(std::move(m)), opt(model, grad_clip) {}
};
static const char *kMaterialTrainerCapsule = "cvc.nav.material_trainer";
static void material_trainer_capsule_dtor(PyObject *cap) {
  delete static_cast<MaterialTrainer *>(PyCapsule_GetPointer(cap, kMaterialTrainerCapsule));
}
static MaterialTrainer *material_trainer_from(PyObject *cap) {
  auto *t = static_cast<MaterialTrainer *>(PyCapsule_GetPointer(cap, kMaterialTrainerCapsule));
  if (!t)
    throw std::invalid_argument("pycvc: not a material-trainer handle");
  return t;
}

// Marshals the 17 numpy arrays of a cvc::nav::material_batch, holding a reference
// to each for the duration of the call (released by the destructor, so a throw
// mid-validation cannot leak). Shared by the trainer's step (loss+backward) and
// loss (forward-only) entry points so their shape contracts cannot drift apart.
// Every check runs BEFORE the caller releases the GIL — a throw crossing
// Py_BEGIN_ALLOW_THREADS would leave the GIL unrestored.
struct MaterialBatchArgs {
  std::vector<PyArrayObject *> hold;
  cvc::nav::material_batch b;

  MaterialBatchArgs() = default;
  MaterialBatchArgs(const MaterialBatchArgs &) = delete;
  MaterialBatchArgs &operator=(const MaterialBatchArgs &) = delete;
  ~MaterialBatchArgs() {
    for (PyArrayObject *h : hold)
      Py_DECREF(h);
  }

  void bind(const char *who, int model_patch_size, PyObject *obs_feats, PyObject *obs_mask,
            PyObject *goal_feats, PyObject *risk_patch, PyObject *o0, PyObject *v0, PyObject *goal,
            PyObject *C, PyObject *R, PyObject *rollout_patch, PyObject *rr, PyObject *d_hat,
            PyObject *dt, PyObject *H, PyObject *o_tgt, PyObject *v_tgt, PyObject *gamma_o) {
    auto oops = [&](const std::string &msg) {
      throw std::invalid_argument(std::string("pycvc.") + who + ": " + msg);
    };
    auto take = [&](PyObject *o, int typ, int nd) -> PyArrayObject * {
      PyArrayObject *a = reinterpret_cast<PyArrayObject *>(
          PyArray_FROMANY(o, typ, nd, nd, NPY_ARRAY_C_CONTIGUOUS));
      if (!a)
        oops("an input array had the wrong dtype/rank");
      hold.push_back(a);
      return a;
    };
    PyArrayObject *ofa = take(obs_feats, NPY_FLOAT, 3);
    PyArrayObject *oma = take(obs_mask, NPY_UINT8, 2);
    PyArrayObject *gfa = take(goal_feats, NPY_FLOAT, 2);
    PyArrayObject *rpa = take(risk_patch, NPY_FLOAT, 4);
    PyArrayObject *o0a = take(o0, NPY_FLOAT, 2);
    PyArrayObject *v0a = take(v0, NPY_FLOAT, 2);
    PyArrayObject *ga = take(goal, NPY_FLOAT, 2);
    PyArrayObject *Ca = take(C, NPY_FLOAT, 3);
    PyArrayObject *Ra = take(R, NPY_FLOAT, 2);
    PyArrayObject *rlpa = take(rollout_patch, NPY_FLOAT, 4);
    PyArrayObject *rra = take(rr, NPY_FLOAT, 1);
    PyArrayObject *dha = take(d_hat, NPY_FLOAT, 1);
    PyArrayObject *dta = take(dt, NPY_FLOAT, 1);
    PyArrayObject *Ha = take(H, NPY_INT32, 1);
    PyArrayObject *ota = take(o_tgt, NPY_FLOAT, 2);
    PyArrayObject *vta = take(v_tgt, NPY_FLOAT, 2);
    PyArrayObject *goa = take(gamma_o, NPY_FLOAT, 1);

    const int B = static_cast<int>(PyArray_DIM(o0a, 0));
    const int N = static_cast<int>(PyArray_DIM(Ca, 1));
    const int P = static_cast<int>(PyArray_DIM(rpa, 2));
    const int Hp = static_cast<int>(PyArray_DIM(rlpa, 2)),
              Wp = static_cast<int>(PyArray_DIM(rlpa, 3));
    if (PyArray_DIM(ofa, 0) != B || PyArray_DIM(ofa, 1) != N || PyArray_DIM(ofa, 2) != 6 ||
        PyArray_DIM(oma, 0) != B || PyArray_DIM(oma, 1) != N || PyArray_DIM(gfa, 0) != B ||
        PyArray_DIM(gfa, 1) != 4 || PyArray_DIM(rpa, 0) != B || PyArray_DIM(rpa, 1) != 2 ||
        PyArray_DIM(rpa, 3) != P || PyArray_DIM(o0a, 1) != 2 || PyArray_DIM(v0a, 0) != B ||
        PyArray_DIM(v0a, 1) != 2 || PyArray_DIM(ga, 0) != B || PyArray_DIM(ga, 1) != 2 ||
        PyArray_DIM(Ca, 0) != B || PyArray_DIM(Ca, 2) != 2 || PyArray_DIM(Ra, 0) != B ||
        PyArray_DIM(Ra, 1) != N || PyArray_DIM(rlpa, 0) != B || PyArray_DIM(rlpa, 1) != 6 ||
        PyArray_DIM(rra, 0) != B || PyArray_DIM(dha, 0) != B || PyArray_DIM(dta, 0) != B ||
        PyArray_DIM(Ha, 0) != B || PyArray_DIM(ota, 0) != B || PyArray_DIM(ota, 1) != 2 ||
        PyArray_DIM(vta, 0) != B || PyArray_DIM(vta, 1) != 2 || PyArray_DIM(goa, 0) != B)
      oops("batch shape mismatch");
    if (model_patch_size != P)
      oops("risk_patch P != model patch_size");

    auto FD = [](PyArrayObject *a) { return static_cast<const float *>(PyArray_DATA(a)); };
    b.B = B;
    b.N = N;
    b.P = P;
    b.Hp = Hp;
    b.Wp = Wp;
    b.obs_feats = FD(ofa);
    b.obs_mask = static_cast<const std::uint8_t *>(PyArray_DATA(oma));
    b.goal_feats = FD(gfa);
    b.risk_patch = FD(rpa);
    b.o0 = FD(o0a);
    b.v0 = FD(v0a);
    b.goal = FD(ga);
    b.C = FD(Ca);
    b.R = FD(Ra);
    b.rollout_patch = FD(rlpa);
    b.rr = FD(rra);
    b.d_hat = FD(dha);
    b.dt = FD(dta);
    b.H = static_cast<const int *>(PyArray_DATA(Ha));
    b.o_tgt = FD(ota);
    b.v_tgt = FD(vta);
    b.gamma_o = FD(goa);
  }
};

PyObject *nav_material_trainer_create(const char *cvcnm_path, double grad_clip = 5.0,
                                      double w_traj = 1.0, double w_vel = 0.5, double w_fric = 0.1,
                                      double w_clear = 5e-3, double w_lreg = 0.01,
                                      double w_goal = 2.0, double w_len = 0.01, double w_risk = 1.0,
                                      double w_hard = 5.0, double cvar_alpha = 0.95,
                                      double w_multi = 0.5, double lam_soft_max = 5.0,
                                      double lam_hard_max = 10.0, double margin_factor = 0.5,
                                      double mass = 1.0, double d_hat_sdf = 3.0,
                                      double k_sharp = 5.0, double tau = 0.05, int ms_h = 3,
                                      double ms_dt_mult = 4.0, int use_cuda = 0) {
  cvc::nav::coef_energy_net model = cvc::nav::coef_energy_net::load(cvcnm_path); // throws on bad
  auto *t = new MaterialTrainer(std::move(model), static_cast<float>(grad_clip));
  t->use_cuda = use_cuda != 0;
  auto &c = t->cfg;
  c.w_traj = (float)w_traj;
  c.w_vel = (float)w_vel;
  c.w_fric = (float)w_fric;
  c.w_clear = (float)w_clear;
  c.w_lreg = (float)w_lreg;
  c.w_goal = (float)w_goal;
  c.w_len = (float)w_len;
  c.w_risk = (float)w_risk;
  c.w_hard = (float)w_hard;
  c.cvar_alpha = (float)cvar_alpha;
  c.w_multi = (float)w_multi;
  c.lam_soft_max = (float)lam_soft_max;
  c.lam_hard_max = (float)lam_hard_max;
  c.rollout.margin_factor = (float)margin_factor;
  c.rollout.mass = (float)mass;
  c.rollout.d_hat_sdf = (float)d_hat_sdf;
  c.rollout.k_sharp = (float)k_sharp;
  c.multi.tau = (float)tau;
  c.multi.ms_h = ms_h;
  c.multi.ms_dt_mult = (float)ms_dt_mult;
  return PyCapsule_New(t, kMaterialTrainerCapsule, material_trainer_capsule_dtor);
}

// One training step over a padded batch. Arrays (C-contiguous): obs_feats
// (B,N,6) f32, obs_mask (B,N) u8, goal_feats (B,4) f32, risk_patch (B,2,P,P) f32,
// o0/v0/goal/o_tgt/v_tgt (B,2) f32, C (B,N,2) f32, R (B,N) f32, rollout_patch
// (B,6,Hp,Wp) f32, rr/d_hat/dt/gamma_o (B,) f32, H (B,) i32. Applies Adam at lr;
// returns the scalar loss.
PyObject *nav_material_trainer_step(PyObject *handle, PyObject *obs_feats, PyObject *obs_mask,
                                    PyObject *goal_feats, PyObject *risk_patch, PyObject *o0,
                                    PyObject *v0, PyObject *goal, PyObject *C, PyObject *R,
                                    PyObject *rollout_patch, PyObject *rr, PyObject *d_hat,
                                    PyObject *dt, PyObject *H, PyObject *o_tgt, PyObject *v_tgt,
                                    PyObject *gamma_o, double lr, int num_threads = 0) {
  MaterialTrainer *t = material_trainer_from(handle);
  MaterialBatchArgs ba;
  ba.bind("nav_material_trainer_step", t->model.patch_size(), obs_feats, obs_mask, goal_feats,
          risk_patch, o0, v0, goal, C, R, rollout_patch, rr, d_hat, dt, H, o_tgt, v_tgt, gamma_o);
  t->cfg.num_threads = num_threads;

  double L = 0.0;
  Py_BEGIN_ALLOW_THREADS
  cvc::nav::coef_energy_net::param_grads grads = t->model.zero_grads();
  L = cvc::nav::material_loss_and_grad(t->model, ba.b, t->cfg, grads, nullptr, t->use_cuda);
  t->opt.step(t->model, grads, static_cast<float>(lr));
  Py_END_ALLOW_THREADS
  return PyFloat_FromDouble(L);
}

// Forward-only loss over a batch — the VALIDATION entry point. Scores a held-out
// split without running the backward or touching the optimizer, so the weights and
// the Adam moments are unchanged. `frozen_eta` pins the detached CVaR quantile
// (pass NaN, the default, to compute it from this batch's own costs).
PyObject *nav_material_trainer_loss(PyObject *handle, PyObject *obs_feats, PyObject *obs_mask,
                                    PyObject *goal_feats, PyObject *risk_patch, PyObject *o0,
                                    PyObject *v0, PyObject *goal, PyObject *C, PyObject *R,
                                    PyObject *rollout_patch, PyObject *rr, PyObject *d_hat,
                                    PyObject *dt, PyObject *H, PyObject *o_tgt, PyObject *v_tgt,
                                    PyObject *gamma_o, double frozen_eta, int num_threads = 0) {
  MaterialTrainer *t = material_trainer_from(handle);
  MaterialBatchArgs ba;
  ba.bind("nav_material_trainer_loss", t->model.patch_size(), obs_feats, obs_mask, goal_feats,
          risk_patch, o0, v0, goal, C, R, rollout_patch, rr, d_hat, dt, H, o_tgt, v_tgt, gamma_o);
  cvc::nav::material_loss_config cfg = t->cfg; // don't publish num_threads onto the handle
  cfg.num_threads = num_threads;

  double L = 0.0;
  Py_BEGIN_ALLOW_THREADS
  L = cvc::nav::material_loss(t->model, ba.b, cfg, static_cast<float>(frozen_eta), t->use_cuda);
  Py_END_ALLOW_THREADS
  return PyFloat_FromDouble(L);
}

// What the handle will ACTUALLY use: true only when it was created with use_cuda
// and the build+device can serve it. Lets a caller log the real path rather than
// the request (material_loss_and_grad falls back silently per-op).
PyObject *nav_material_trainer_cuda_active(PyObject *handle) {
  MaterialTrainer *t = material_trainer_from(handle);
  const bool on = t->use_cuda && cvc::nav::coef_energy_cuda_available() &&
                  cvc::nav::material_rollout_cuda_available();
  return PyBool_FromLong(on ? 1 : 0);
}

// Device probes, independent of any handle (so a caller can decide before it
// builds one). Always defined -- CPU-only builds link the `false` stubs.
PyObject *nav_material_cuda_available() {
  const bool on = cvc::nav::coef_energy_cuda_available() &&
                  cvc::nav::material_rollout_cuda_available() &&
                  cvc::nav::material_train_cuda_available();
  return PyBool_FromLong(on ? 1 : 0);
}

// Largest max(H) the device rollout VJP accepts; above it that one op falls back
// to the host adjoint. 0 on a CPU-only build.
PyObject *nav_material_cuda_max_horizon() {
  return PyLong_FromLong(cvc::nav::material_rollout_cuda_max_horizon());
}

PyObject *nav_material_trainer_save(PyObject *handle, const char *path) {
  MaterialTrainer *t = material_trainer_from(handle);
  t->model.save(path);
  Py_RETURN_NONE;
}

} // namespace pycvc

%}
