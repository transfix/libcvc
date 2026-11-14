// pycvc_contours.i — cvc::contour_stack (inc/cvc/geometry/contours.h), the contours the tiler and
// VolumeGridRover share (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, P0d):
//
//   s = pycvc.contour_stack()
//   s.z_spacing = 0.05
//   s.add(3, "axon", [0, 0, 1, 0, 1, 1])           # slice 3: x0 y0 x1 y1 ...
//   pycvc.write_contours(app, s, "series.ser")     # the format by its extension
//   t = pycvc.read_contours(app, "series.ser", pycvc.contour_read_params(include=["ax.*"]))
//   t.indices(), t.component_names(), t.polygons(3, "axon"), t.components[0].color
//
// read_contours / write_contours take the app whose handlers serve the format (each app's own),
// and let other Python threads run while they read or write. Errors raise ValueError (bad
// parameters, a stack with problems) or RuntimeError (a file that can't be read or written).
//
// Python gets copies: s.components is a list of copies (set one with s.set_component(c)), and
// s.polygons(index, name) a list of coordinate lists. The slices map is reached through
// indices(), polygons(), slice_z() and set_slice_z(). contour_read_params and contour_component
// have their parameter tables (pycvc_params.i): keywords, validate(), to_state / from_state.

%{
#include <cmath>
#include <cvc/geometry/contours.h>
%}

%template(contour_component_vector) std::vector<cvc::contour_component>;
// Their lists reach Python as tuples, copies (the members' own getters hand out views).
%naturalvar cvc::contour_component::color;
%naturalvar cvc::contour_read_params::include;
%naturalvar cvc::contour_read_params::exclude;

// Containers reach Python as copies (a reference into the stack would dangle once a temporary
// stack is collected); the handler interface stays C++'s.
%ignore cvc::contour_stack::slices;
%ignore cvc::contour_stack::components;
%ignore cvc::contour_stack::polygons;
%ignore cvc::contour_stack::indices;
%ignore cvc::contour_slice;
%ignore cvc::contour_polygon;
%ignore cvc::contour_file_io;
%ignore cvc::contour_handlers;
%ignore cvc::read_contours;
%ignore cvc::write_contours;
%ignore cvc::contour_read_extensions;
%ignore cvc::contour_write_extensions;
%ignore cvc::contour_file_error;
%ignore cvc::contour_stack::operator!=;
%ignore cvc::contour_component::operator!=;
%rename(__eq__) cvc::contour_stack::operator==;
%rename(__eq__) cvc::contour_component::operator==;
%exception cvc::contour_stack::add {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%exception cvc::contour_stack::set_component {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%include "cvc/geometry/contours.h"

%extend cvc::contour_stack {
  std::vector<cvc::contour_component> components_copy() const { return $self->components; }
  // Slice `index`'s polygons of `component`, each a list x0, y0, x1, y1, ... (a copy).
  PyObject *polygon_lists(int index, const std::string &component) const {
    const std::vector<cvc::contour_polygon> polys = $self->polygons(index, component);
    PyObject *out = PyList_New(static_cast<Py_ssize_t>(polys.size()));
    for (std::size_t i = 0; i < polys.size(); ++i) {
      PyObject *xy = PyList_New(static_cast<Py_ssize_t>(polys[i].xy.size()));
      for (std::size_t k = 0; k < polys[i].xy.size(); ++k)
        PyList_SET_ITEM(xy, static_cast<Py_ssize_t>(k), PyFloat_FromDouble(polys[i].xy[k]));
      PyList_SET_ITEM(out, static_cast<Py_ssize_t>(i), xy);
    }
    return out;
  }
  // The z indices, ascending.
  PyObject *index_list() const {
    const std::vector<int> v = $self->indices();
    PyObject *out = PyList_New(static_cast<Py_ssize_t>(v.size()));
    for (std::size_t i = 0; i < v.size(); ++i)
      PyList_SET_ITEM(out, static_cast<Py_ssize_t>(i), PyLong_FromLong(v[i]));
    return out;
  }
  // Give slice `index` its own plane (creating the slice); NaN goes back to the spacing's.
  void set_slice_z(int index, double z) { $self->slices[index].z = z; }
  // Whether slice `index` exists (it may hold no polygons: an empty slice).
  bool has_slice(int index) const { return $self->slices.count(index) != 0; }
  %pythoncode %{
    components = property(components_copy, doc="The components, in order: copies Python owns.")

    def polygons(self, index, component):
        """Slice `index`'s polygons of `component`: lists x0, y0, x1, y1, ... (copies)."""
        return self.polygon_lists(index, component)

    def indices(self):
        """The z indices, ascending."""
        return self.index_list()

    def __repr__(self):
        return "<contour_stack %d slices, %d polygons, components %s>" % (
            len(self.indices()), self.polygon_count(), list(self.component_names()))
  %}
}

PYCVC_PARAM_TABLE(contour_component, cvc::contour_component)
PYCVC_PARAM_TABLE(contour_read_params, cvc::contour_read_params)

%exception read_contours {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception write_contours {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception contour_problems {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
// pycvc.read_contours(app, path[, params]): cvc::read_contours with the app's handlers.
cvc::contour_stack read_contours(std::shared_ptr<cvc::app> app, const std::string &path,
                                 const cvc::contour_read_params &params =
                                     cvc::contour_read_params()) {
  if (!app)
    throw std::invalid_argument("read_contours: app is None");
  return cvc::read_contours(*app, path, params);
}
// pycvc.write_contours(app, stack, path).
void write_contours(std::shared_ptr<cvc::app> app, const cvc::contour_stack &stack,
                    const std::string &path) {
  if (!app)
    throw std::invalid_argument("write_contours: app is None");
  cvc::write_contours(*app, stack, path);
}
// The extensions the app reads, and writes.
std::vector<std::string> contour_read_extensions(std::shared_ptr<cvc::app> app) {
  return app ? cvc::contour_read_extensions(*app) : std::vector<std::string>();
}
std::vector<std::string> contour_write_extensions(std::shared_ptr<cvc::app> app) {
  return app ? cvc::contour_write_extensions(*app) : std::vector<std::string>();
}
%}
