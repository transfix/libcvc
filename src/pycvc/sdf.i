/*
  sdf.i — minimal cvc::geometry construction + mesh→SDF→volume.

  Enough surface for a mesh → signed-distance-field → numpy round trip:
    - cvc::geometry with numpy mesh in/out (points Nx3 float64, tris Mx3 ints)
    - generate_sphere / generate_cube procedural test meshes (ungated)
    - sdf(geom, xdim, ydim, zdim[, bbox][, flip_normals]) → volume
      (gated on CVC_ENABLE_SDF, matching the cvc library build; when the
      library is built without SDF support the function simply doesn't
      exist in the module)
*/

%{
namespace {

// numpy mesh → cvc::geometry (copy-in): points (N,3) float, tris (M,3) int.
void pycvc_geometry_set_mesh(cvc::geometry &geom, PyObject *points, PyObject *tris) {
  PyArrayObject *p = reinterpret_cast<PyArrayObject *>(
      PyArray_FROM_OTF(points, NPY_FLOAT64,
                       NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_ALIGNED | NPY_ARRAY_FORCECAST));
  if (!p) {
    PyErr_Clear();
    throw std::runtime_error("pycvc: points must be convertible to an (N, 3) float array");
  }
  if (PyArray_NDIM(p) != 2 || PyArray_DIM(p, 1) != 3) {
    Py_DECREF(p);
    throw std::runtime_error("pycvc: points must have shape (N, 3)");
  }
  PyArrayObject *t = reinterpret_cast<PyArrayObject *>(
      PyArray_FROM_OTF(tris, NPY_UINT64,
                       NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_ALIGNED | NPY_ARRAY_FORCECAST));
  if (!t) {
    PyErr_Clear();
    Py_DECREF(p);
    throw std::runtime_error("pycvc: tris must be convertible to an (M, 3) integer array");
  }
  if (PyArray_NDIM(t) != 2 || PyArray_DIM(t, 1) != 3) {
    Py_DECREF(p);
    Py_DECREF(t);
    throw std::runtime_error("pycvc: tris must have shape (M, 3)");
  }

  const npy_intp npts = PyArray_DIM(p, 0);
  const npy_intp ntris = PyArray_DIM(t, 0);
  const double *pd = static_cast<const double *>(PyArray_DATA(p));
  const npy_uint64 *td = static_cast<const npy_uint64 *>(PyArray_DATA(t));

  cvc::geometry::points_t &P = geom.points();
  P.clear();
  P.reserve(static_cast<size_t>(npts));
  for (npy_intp i = 0; i < npts; ++i) {
    cvc::geometry::point_t pt = {{pd[3 * i], pd[3 * i + 1], pd[3 * i + 2]}};
    P.push_back(pt);
  }
  cvc::geometry::tris_t &T = geom.tris();
  T.clear();
  T.reserve(static_cast<size_t>(ntris));
  for (npy_intp i = 0; i < ntris; ++i) {
    cvc::geometry::tri_t tr = {{static_cast<cvc::geometry::index_t>(td[3 * i]),
                                static_cast<cvc::geometry::index_t>(td[3 * i + 1]),
                                static_cast<cvc::geometry::index_t>(td[3 * i + 2])}};
    T.push_back(tr);
  }
  geom.set_geometry_type(cvc::geometry::SURFACE_TRI);

  Py_DECREF(p);
  Py_DECREF(t);
}

// cvc::geometry points → (N, 3) float64 numpy array (copy-out).
PyObject *pycvc_geometry_points_to_numpy(const cvc::geometry &geom) {
  const cvc::geometry::points_t &P = geom.const_points();
  npy_intp dims[2] = {static_cast<npy_intp>(P.size()), 3};
  PyObject *out = PyArray_SimpleNew(2, dims, NPY_FLOAT64);
  if (!out) {
    PyErr_Clear();
    throw std::runtime_error("pycvc: could not allocate points array");
  }
  double *od = static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(out)));
  for (size_t i = 0; i < P.size(); ++i)
    for (int j = 0; j < 3; ++j)
      od[3 * i + j] = P[i][j];
  return out;
}

// cvc::geometry tris → (M, 3) uint64 numpy array (copy-out).
PyObject *pycvc_geometry_tris_to_numpy(const cvc::geometry &geom) {
  const cvc::geometry::tris_t &T = geom.const_tris();
  npy_intp dims[2] = {static_cast<npy_intp>(T.size()), 3};
  PyObject *out = PyArray_SimpleNew(2, dims, NPY_UINT64);
  if (!out) {
    PyErr_Clear();
    throw std::runtime_error("pycvc: could not allocate tris array");
  }
  npy_uint64 *od =
      static_cast<npy_uint64 *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(out)));
  for (size_t i = 0; i < T.size(); ++i)
    for (int j = 0; j < 3; ++j)
      od[3 * i + j] = static_cast<npy_uint64>(T[i][j]);
  return out;
}

} // namespace
%}

// ---------------------------------------------------------------------------
// Hand-curated declarations (subset of inc/cvc/geometry/geometry.h and
// inc/cvc/utility/algorithm.h — keep signatures in sync).
// ---------------------------------------------------------------------------
namespace cvc {

class geometry {
public:
  geometry();
  ~geometry();
  unsigned long long num_points() const;
  unsigned long long num_tris() const;
  bool empty() const;
  bounding_box extents() const;
};

%extend geometry {
  // Load a triangle mesh from numpy: points (N, 3) float, tris (M, 3) int.
  void set_mesh(PyObject *points, PyObject *tris) {
    pycvc_geometry_set_mesh(*$self, points, tris);
  }
  PyObject *points_to_numpy() const {
    return pycvc_geometry_points_to_numpy(*$self);
  }
  PyObject *tris_to_numpy() const {
    return pycvc_geometry_tris_to_numpy(*$self);
  }
}

// Procedural test meshes (inc/cvc/utility/algorithm.h, ungated).
geometry generate_sphere(double cx, double cy, double cz, double radius, int thetaRes = 32,
                         int phiRes = 16);
geometry generate_cube(double cx, double cy, double cz, double sizeX, double sizeY,
                       double sizeZ);

} // namespace cvc

#ifdef CVC_ENABLE_SDF
%inline %{
// Mesh → signed distance field on the module app context.  If bbox is
// omitted / null, the extents of the geometry are used (per cvc::sdf).
cvc::volume sdf(const cvc::geometry &geom, unsigned long long xdim, unsigned long long ydim,
                unsigned long long zdim, const cvc::bounding_box &bbox = cvc::bounding_box(),
                bool flip_normals = false) {
  return cvc::sdf(pycvc_ctx(), geom, cvc::dimension(xdim, ydim, zdim), bbox, cvc::SDF_V1,
                  flip_normals);
}
%}
#endif
