// pycvc_point_classes.i — cvc::point_classes (inc/cvc/volume/point_classes.h), the seeds GenSeg
// grows from and the points VolumeGridRover marks (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md
// §1.4, P0d):
//
//   pc = pycvc.point_classes()
//   pc.add("membrane", 12, 40, 7)                  # a voxel by its i j k
//   pycvc.write_point_classes(app, pc, "seeds.vgr")
//   back = pycvc.read_point_classes(app, "seeds.vgr")
//   grid = pycvc.point_class_read_params(dims=[64, 64, 32], bbox=[0, 0, 0, 1, 1, 0.5])
//   pts = pycvc.read_point_classes(app, "points.pts", grid)
//   pc.points("membrane")          # [[12, 40, 7]]
//   pc.points_array("membrane")    # a (n, 3) int32 numpy array (a copy)
//
// read_point_classes / write_point_classes take the app whose handlers serve the format, and let
// other Python threads run meanwhile. Errors raise ValueError (bad parameters, classes with
// problems) or RuntimeError (a file that can't be read or written). pc.classes is a list of
// copies; set one with pc.set_class(c). point_class and point_class_read_params have their
// parameter tables (pycvc_params.i).

%{
#include <cvc/volume/point_classes.h>
%}

// Points and dims are std::vector<int>: lists of ints from Python, tuples back.
%template(IntVector) std::vector<int>;
%template(point_class_vector) std::vector<cvc::point_class>;
// Lists reach Python as tuples, copies (the members' own getters hand out views).
%naturalvar cvc::point_class::color;
%naturalvar cvc::point_class::points;
%naturalvar cvc::point_class_read_params::dims;
%naturalvar cvc::point_class_read_params::bbox;

%ignore cvc::point_classes::classes;
%ignore cvc::point_class_file_io;
%ignore cvc::point_class_handlers;
%ignore cvc::read_point_classes;
%ignore cvc::write_point_classes;
%ignore cvc::point_class_read_extensions;
%ignore cvc::point_class_write_extensions;
%ignore cvc::point_class_file_error;
%ignore cvc::point_class::operator!=;
%ignore cvc::point_classes::operator!=;
%rename(__eq__) cvc::point_class::operator==;
%rename(__eq__) cvc::point_classes::operator==;
%exception cvc::point_classes::add {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%exception cvc::point_classes::set_class {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%include "cvc/volume/point_classes.h"

%extend cvc::point_classes {
  std::vector<cvc::point_class> classes_copy() const { return $self->classes; }
  // The class's points as a list of [i, j, k] lists (a copy). KeyError-like ValueError when there
  // is no such class.
  PyObject *point_lists(const std::string &name, int variable = 0, int timestep = 0) const {
    const int k = $self->class_index(name, variable, timestep);
    if (k < 0) {
      PyErr_SetString(PyExc_ValueError, ("no class '" + name + "'").c_str());
      return nullptr;
    }
    const cvc::point_class &c = $self->classes[static_cast<std::size_t>(k)];
    PyObject *out = PyList_New(static_cast<Py_ssize_t>(c.size()));
    for (std::size_t n = 0; n < c.size(); ++n)
      PyList_SET_ITEM(out, static_cast<Py_ssize_t>(n),
                      Py_BuildValue("[iii]", c.points[3 * n], c.points[3 * n + 1],
                                    c.points[3 * n + 2]));
    return out;
  }
  // The class's points as a fresh (n, 3) int32 numpy array.
  PyObject *points_array(const std::string &name, int variable = 0, int timestep = 0) const {
    const int k = $self->class_index(name, variable, timestep);
    if (k < 0) {
      PyErr_SetString(PyExc_ValueError, ("no class '" + name + "'").c_str());
      return nullptr;
    }
    const cvc::point_class &c = $self->classes[static_cast<std::size_t>(k)];
    npy_intp dims[2] = {static_cast<npy_intp>(c.size()), 3};
    PyObject *arr = PyArray_SimpleNew(2, dims, NPY_INT32);
    if (!arr)
      return nullptr;
    auto *d = static_cast<std::int32_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
    for (std::size_t j = 0; j < c.points.size(); ++j)
      d[j] = static_cast<std::int32_t>(c.points[j]);
    return arr;
  }
  %pythoncode %{
    classes = property(classes_copy, doc="The classes, in order: copies Python owns.")

    def points(self, name, variable=0, timestep=0):
        """The class's points: [i, j, k] lists (copies)."""
        return self.point_lists(name, variable, timestep)

    def __repr__(self):
        return "<point_classes %d classes, %d points>" % (len(self.classes), self.point_count())
  %}
}

PYCVC_PARAM_TABLE(point_class, cvc::point_class)
PYCVC_PARAM_TABLE(point_class_read_params, cvc::point_class_read_params)

%exception read_point_classes {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception write_point_classes {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
// pycvc.read_point_classes(app, path[, params]): cvc::read_point_classes with the app's handlers.
cvc::point_classes read_point_classes(std::shared_ptr<cvc::app> app, const std::string &path,
                                      const cvc::point_class_read_params &params =
                                          cvc::point_class_read_params()) {
  if (!app)
    throw std::invalid_argument("read_point_classes: app is None");
  return cvc::read_point_classes(*app, path, params);
}
// pycvc.write_point_classes(app, pc, path).
void write_point_classes(std::shared_ptr<cvc::app> app, const cvc::point_classes &pc,
                         const std::string &path) {
  if (!app)
    throw std::invalid_argument("write_point_classes: app is None");
  cvc::write_point_classes(*app, pc, path);
}
std::vector<std::string> point_class_read_extensions(std::shared_ptr<cvc::app> app) {
  return app ? cvc::point_class_read_extensions(*app) : std::vector<std::string>();
}
std::vector<std::string> point_class_write_extensions(std::shared_ptr<cvc::app> app) {
  return app ? cvc::point_class_write_extensions(*app) : std::vector<std::string>();
}
%}
