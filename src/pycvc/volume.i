/*
  volume.i — cvc::dimension, cvc::bounding_box, cvc::voxels, cvc::volume
  with numpy conversion.

  NUMPY STRATEGY (skeleton): copy-in / copy-out.
    - pycvc.volume(arr) converts the array to a supported contiguous dtype
      and the cvc::volume raw-pointer constructor memcpy's it into its own
      boost::shared_array buffer (one copy in).
    - vol.to_numpy() / np.asarray(vol) allocates a fresh ndarray and
      memcpy's the voxel buffer out (one copy out).
  Zero-copy out (PyArray_SimpleNewFromData + PyArray_BASE keeping the owning
  voxels alive) is the planned follow-up; copies are cheap relative to the
  heavy whole-field filters this surface exists for, and copy semantics
  side-step the copy-on-write / shared_array lifetime hazards flagged in the
  design doc.

  ARRAY CONVENTION: a C-ordered numpy array of shape (Z, Y, X) maps directly
  onto libcvc's voxel layout (x fastest: idx = x + y*XDim + z*XDim*YDim), so
  conversion in either direction is a single flat memcpy.  A 2D raster is
  the degenerate case: shape (1, H, W) gives a ZDim()==1 volume.

  Supported dtypes: uint8, uint16, uint32, float32, float64 (mapped to
  UChar/UShort/UInt/Float/Double).  Anything else is force-cast to float32.
*/

%{
namespace {

cvc::data_type pycvc_npy_to_cvc(int npy_type) {
  switch (npy_type) {
  case NPY_UINT8:
    return cvc::UChar;
  case NPY_UINT16:
    return cvc::UShort;
  case NPY_UINT32:
    return cvc::UInt;
  case NPY_FLOAT32:
    return cvc::Float;
  case NPY_FLOAT64:
    return cvc::Double;
  default:
    return cvc::Undefined;
  }
}

int pycvc_cvc_to_npy(cvc::data_type vt) {
  switch (vt) {
  case cvc::UChar:
    return NPY_UINT8;
  case cvc::UShort:
    return NPY_UINT16;
  case cvc::UInt:
    return NPY_UINT32;
  case cvc::Float:
    return NPY_FLOAT32;
  case cvc::Double:
    return NPY_FLOAT64;
  default:
    return -1;
  }
}

// Convert an arbitrary array-like into a 3D C-contiguous PyArrayObject of a
// cvc-supported dtype (unsupported dtypes force-cast to float32).  Returns a
// new reference; throws on failure (translated by the module %exception).
PyArrayObject *pycvc_as_3d_carray(PyObject *array, cvc::data_type &vt_out) {
  PyArrayObject *probe = reinterpret_cast<PyArrayObject *>(PyArray_FROM_O(array));
  if (!probe) {
    PyErr_Clear();
    throw std::runtime_error("pycvc: input is not convertible to a numpy array");
  }
  int target = PyArray_TYPE(probe);
  cvc::data_type vt = pycvc_npy_to_cvc(target);
  if (vt == cvc::Undefined) {
    vt = cvc::Float;
    target = NPY_FLOAT32;
  }
  PyArrayObject *carr = reinterpret_cast<PyArrayObject *>(
      PyArray_FROM_OTF(reinterpret_cast<PyObject *>(probe), target,
                       NPY_ARRAY_C_CONTIGUOUS | NPY_ARRAY_ALIGNED | NPY_ARRAY_FORCECAST));
  Py_DECREF(probe);
  if (!carr) {
    PyErr_Clear();
    throw std::runtime_error("pycvc: could not convert array to a contiguous supported dtype");
  }
  if (PyArray_NDIM(carr) != 3) {
    Py_DECREF(carr);
    throw std::runtime_error("pycvc: expected a 3D array of shape (Z, Y, X); "
                             "wrap a 2D raster as arr[None, :, :]");
  }
  vt_out = vt;
  return carr;
}

// numpy (Z, Y, X) → new cvc::volume (copy-in).  If box is null, the bounding
// box defaults to index space: (0,0,0)-(XDim-1, YDim-1, ZDim-1).
cvc::volume *pycvc_volume_from_numpy(PyObject *array, const cvc::bounding_box *box) {
  cvc::data_type vt = cvc::Undefined;
  PyArrayObject *carr = pycvc_as_3d_carray(array, vt);
  const npy_intp *sh = PyArray_DIMS(carr);
  cvc::dimension dim(static_cast<cvc::uint64>(sh[2]), static_cast<cvc::uint64>(sh[1]),
                     static_cast<cvc::uint64>(sh[0]));
  cvc::volume *vol = 0;
  try {
    cvc::bounding_box bbox = box ? *box : cvc::bounding_box(dim);
    vol = new cvc::volume(pycvc_ctx(), static_cast<const unsigned char *>(PyArray_DATA(carr)),
                          dim, vt, bbox);
  } catch (...) {
    Py_DECREF(carr);
    throw;
  }
  Py_DECREF(carr);
  return vol;
}

// cvc::voxels → new numpy array of shape (Z, Y, X) (copy-out).
PyObject *pycvc_voxels_to_numpy(const cvc::voxels &v) {
  const int npy_type = pycvc_cvc_to_npy(v.voxelType());
  if (npy_type < 0)
    throw std::runtime_error("pycvc: unsupported voxel type for numpy export");
  npy_intp dims[3] = {static_cast<npy_intp>(v.ZDim()), static_cast<npy_intp>(v.YDim()),
                      static_cast<npy_intp>(v.XDim())};
  PyObject *out = PyArray_SimpleNew(3, dims, npy_type);
  if (!out) {
    PyErr_Clear();
    throw std::runtime_error("pycvc: could not allocate output numpy array");
  }
  std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(out)), v.data_ptr(),
              static_cast<size_t>(v.XDim() * v.YDim() * v.ZDim() * v.voxelSize()));
  return out;
}

} // namespace
%}

// ---------------------------------------------------------------------------
// Hand-curated declarations (subset of inc/cvc/volume/{dimension,bounding_box,
// voxels,volume}.h — keep signatures in sync with the real headers).
// ---------------------------------------------------------------------------
// voxels declares no wrapped constructor (the real ones need an app&), so
// stop SWIG from synthesizing a default-constructor wrapper that doesn't
// exist in C++.  volume gets its constructors via %extend below.
%nodefaultctor cvc::voxels;

namespace cvc {

enum data_type { UChar = 0, UShort, UInt, Float, Double, UInt64, Char, Int, Int64, Undefined };

class dimension {
public:
  dimension();
  dimension(unsigned long long x, unsigned long long y, unsigned long long z);
  unsigned long long XDim() const;
  unsigned long long YDim() const;
  unsigned long long ZDim() const;
  unsigned long long size() const;
  bool isNull() const;
  std::string str() const;
};

class bounding_box {
public:
  bounding_box();
  bounding_box(double minx, double miny, double minz, double maxx, double maxy, double maxz);
  double XMin() const;
  double XMax() const;
  double YMin() const;
  double YMax() const;
  double ZMin() const;
  double ZMax() const;
  bool isNull() const;
  bool contains(double x, double y, double z) const;
  std::string str() const;
};

class voxels {
public:
  // No constructor wrapped on purpose (needs an app& context) — construct
  // through pycvc.volume(numpy_array).
  unsigned long long XDim() const;
  unsigned long long YDim() const;
  unsigned long long ZDim() const;
  data_type voxelType() const;
  const char *voxelTypeStr() const;
  double min() const;
  double max() const;

  voxels &fill(double val);
  voxels &map(double min_, double max_);
  void unsetMinMax();
};

%extend voxels {
  // Copy-out to a numpy array of shape (ZDim, YDim, XDim).
  PyObject *to_numpy() const {
    return pycvc_voxels_to_numpy(*$self);
  }

  // Edge-preserving smoothing family (src/cvc/processing/) — the r̃
  // manufacturers.  All mutate in place and return self.
  //
  // Wrapped via %extend (not declared directly) so we can invalidate the
  // cached min/max after the data mutates: the C++ filters read min()/max()
  // before filtering and never call unsetMinMax(), so a following
  // vol_normalize() would otherwise remap against stale pre-filter bounds.
  voxels &bilateralFilter(double radiometricSigma = 200.0, double spatialSigma = 1.5,
                          unsigned int filterRadius = 2) {
    cvc::voxels &r = $self->bilateralFilter(radiometricSigma, spatialSigma, filterRadius);
    $self->unsetMinMax();
    return r;
  }
  voxels &anisotropicDiffusion(unsigned int iterations = 20) {
    cvc::voxels &r = $self->anisotropicDiffusion(iterations);
    $self->unsetMinMax();
    return r;
  }
  voxels &gdtvFilter(double parameterq, double lambda_, unsigned int iteration,
                     unsigned int neighbour) {
    cvc::voxels &r = $self->gdtvFilter(parameterq, lambda_, iteration, neighbour);
    $self->unsetMinMax();
    return r;
  }
}

class volume : public voxels {
public:
  // Bounding box in object space
  double XMin() const;
  double XMax() const;
  double YMin() const;
  double YMax() const;
  double ZMin() const;
  double ZMax() const;
  double XSpan() const;
  double YSpan() const;
  double ZSpan() const;

  const bounding_box &boundingBox() const;
  void boundingBox(const bounding_box &box);

  // Trilinear interpolation at object-space coordinates (must be inside the
  // bounding box or a RuntimeError is raised).
  double interpolate(double obj_x, double obj_y, double obj_z) const;
};

%extend volume {
  // volume(arr): copy a (Z, Y, X) numpy array in; bounding box defaults to
  // index space (0,0,0)-(XDim-1, YDim-1, ZDim-1).
  volume(PyObject *array) {
    return pycvc_volume_from_numpy(array, 0);
  }
  // volume(arr, bbox): same, with an explicit object-space bounding box.
  volume(PyObject *array, const cvc::bounding_box &box) {
    return pycvc_volume_from_numpy(array, &box);
  }
}

} // namespace cvc

// Python-side sugar: numpy protocol + shape property.
%pythoncode %{
def _pycvc_voxels_array(self, dtype=None, copy=None):
    a = self.to_numpy()
    if dtype is not None:
        a = a.astype(dtype)
    return a

voxels.__array__ = _pycvc_voxels_array
voxels.shape = property(
    lambda self: (int(self.ZDim()), int(self.YDim()), int(self.XDim())),
    doc="Volume shape as a numpy-style (Z, Y, X) tuple.")
del _pycvc_voxels_array
%}
