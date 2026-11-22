// pycvc_volume_slice.i — axis-aligned slices of a volume (inc/cvc/volume/volume_slice.h), the data
// side of cvcGL's volume_slice node (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §6, P7):
//
//   s = pycvc.extract_slice(vol, pycvc.slice_axis_z, 40)   # the plane k = 40
//   s.values()                    # a (height, width) float64 numpy array (a copy); row 0 the bottom
//   px = pycvc.color_slice(s, model, lo, hi, True)  # (height, width, 4) uint8, the top row first
//   src = pycvc.slice_source(app, "big.rawiv")      # a file read one plane at a time
//   src.slice(pycvc.slice_axis_y, 12), src.min(), src.max(), src.value(i, j, k)
//   pycvc.slice_axis_from("xz") == pycvc.slice_axis_y
//
// `model` is a transfer-function model as its state string (cvc::tf::to_string's JSON; "" is the
// default model). The plane's axes, right then up: down z, x and y; down y, x and z; down x, z and
// y. A slice outside the volume raises IndexError, an empty volume or a bad axis name ValueError,
// a file that can't be read RuntimeError. Reads and colouring let other Python threads run.
// volume_slice_params has its parameter table (pycvc_params.i).

%{
#include <cvc/tf/model.h>
#include <cvc/volume/volume_slice.h>
%}

// The enums' descriptions are for the parameter tables (C++ only).
#define BOOST_DESCRIBE_ENUM(E, ...)

%ignore cvc::parse_slice_axis;
%ignore cvc::slice_plane_axes;
%ignore cvc::grid_of;
%ignore cvc::slice_grid;
%ignore cvc::volume_slice_data::values;
%ignore cvc::volume_slice_data::voxel;
%ignore cvc::color_slice;
%ignore cvc::slice_source;
%ignore cvc::volume_payload;         // C++ nodes following a state key (node_data.h)
%ignore cvc::volume_payload_pointee;
%ignore cvc::param_table<cvc::volume_slice_params>;

%exception cvc::extract_slice {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::out_of_range &e) {
    SWIG_exception(SWIG_IndexError, e.what());
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%include "cvc/volume/volume_slice.h"

%extend cvc::volume_slice_data {
  // The values as a fresh (height, width) float64 numpy array: [row, column], row 0 the bottom
  // (values(), below: the member's own name is ignored, and with it a method of that name here).
  PyObject *values_array() const {
    npy_intp dims[2] = {static_cast<npy_intp>($self->height), static_cast<npy_intp>($self->width)};
    PyObject *arr = PyArray_SimpleNew(2, dims, NPY_FLOAT64);
    if (!arr)
      return nullptr;
    std::copy($self->values.begin(), $self->values.end(),
              static_cast<double *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr))));
    return arr;
  }
  %pythoncode %{
    def values(self):
        """The values as a fresh (height, width) float64 numpy array; row 0 the bottom."""
        return self.values_array()

    def __repr__(self):
        return "<volume_slice_data %s %d: %d x %d>" % (
            slice_axis_name(self.axis), self.index, self.width, self.height)
  %}
}

%define PYCVC_SLICE_EXCEPTION(NAME)
%exception NAME {
  try {
    $action
  } catch (const std::out_of_range &e) {
    SWIG_exception(SWIG_IndexError, e.what());
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%enddef
PYCVC_SLICE_EXCEPTION(pycvc::slice_axis_from)
PYCVC_SLICE_EXCEPTION(pycvc::slice_source::slice_source)
PYCVC_SLICE_EXCEPTION(pycvc::slice_source::slice)
PYCVC_SLICE_EXCEPTION(pycvc::slice_source::value)

%inline %{
namespace pycvc {
// An axis by name (x, y, z, 0, 1, 2, or a view's name: xy, xz, zy); ValueError for anything else.
cvc::slice_axis slice_axis_from(const std::string &text) {
  cvc::slice_axis a = cvc::slice_axis::z;
  if (!cvc::parse_slice_axis(text, a))
    throw std::invalid_argument("'" + text + "' is not an axis: x, y or z (or xy, xz, zy)");
  return a;
}
// The plane's axes looking down `a` (0 x, 1 y, 2 z): (right, up).
PyObject *slice_plane_axes(cvc::slice_axis a) {
  int r = 0, u = 1;
  cvc::slice_plane_axes(a, r, u);
  return Py_BuildValue("(ii)", r, u);
}
// The pixels of `s` through the model `model` (its state string) over [lo, hi]: a fresh
// (height, width, 4) uint8 numpy array, the top row first. ValueError when the model does not
// parse.
PyObject *color_slice(const cvc::volume_slice_data &s, const std::string &model, double lo,
                      double hi, bool opaque) {
  cvc::tf::model m;
  std::string err;
  if (!cvc::tf::from_string(model, m, &err)) {
    PyErr_SetString(PyExc_ValueError, err.c_str());
    return nullptr;
  }
  std::vector<std::uint8_t> px;
  {
    pycvc::gil_release pycvc_nogil;
    px = cvc::color_slice(s, m, lo, hi, opaque);
  }
  npy_intp dims[3] = {static_cast<npy_intp>(s.height), static_cast<npy_intp>(s.width), 4};
  PyObject *arr = PyArray_SimpleNew(3, dims, NPY_UINT8);
  if (!arr)
    return nullptr;
  std::copy(px.begin(), px.end(),
            static_cast<std::uint8_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr))));
  return arr;
}

// cvc::slice_source: a volume in memory, or a volume file read one plane at a time.
class slice_source {
public:
  explicit slice_source(const cvc::volume &vol, std::size_t cache = 8)
      : _src(std::make_shared<cvc::slice_source>(vol, cache)) {}
  slice_source(std::shared_ptr<cvc::app> app, const std::string &file, unsigned int var = 0,
               unsigned int time = 0, std::size_t cache = 8) {
    if (!app)
      throw std::invalid_argument("pycvc.slice_source: null app handle");
    _app = app;
    pycvc::gil_release pycvc_nogil;
    _src = std::make_shared<cvc::slice_source>(*app, file, var, time, cache);
  }
  cvc::volume_slice_data slice(cvc::slice_axis a, std::uint64_t index) {
    pycvc::gil_release pycvc_nogil;
    return *_src->slice(a, index);
  }
  double value(std::uint64_t i, std::uint64_t j, std::uint64_t k,
               cvc::slice_axis a = cvc::slice_axis::z) {
    const std::uint64_t ijk[3] = {i, j, k};
    pycvc::gil_release pycvc_nogil;
    return _src->value(ijk, a);
  }
  double min() const { return _src->min(); }
  double max() const { return _src->max(); }
  bool in_memory() const { return _src->in_memory(); }
  std::string file() const { return _src->file(); }
  std::size_t reads() const { return _src->reads(); }
  // How many planes look down `a`, and where plane `index` lies along it.
  std::uint64_t count(cvc::slice_axis a) const { return _src->grid().count(a); }
  double plane(cvc::slice_axis a, std::uint64_t index) const {
    return _src->grid().plane(a, index);
  }
  double spacing(int axis) const { return _src->grid().spacing(axis); }

private:
  std::shared_ptr<cvc::app> _app; // a file source reads through its app
  std::shared_ptr<cvc::slice_source> _src;
};
} // namespace pycvc
%}

PYCVC_PARAM_TABLE(volume_slice_params, cvc::volume_slice_params)
