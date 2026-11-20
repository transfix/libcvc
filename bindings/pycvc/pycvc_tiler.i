// pycvc_tiler.i — surfaces between a contour stack's slices (inc/cvc/geometry/tiler.h), the
// VolumeRover ContourTiler port (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §3): tile_contours
// on a contour_stack (pycvc_contours.i), its result, and tile_params with its parameter table
// (inc/cvc/geometry/tiler_params.h), so it takes keywords:
//
//   s = pycvc.contour_stack()
//   s.add(0, "axon", [0, 0, 4, 0, 4, 3, 0, 3]); s.add(1, "axon", [...])
//   r = pycvc.tile_contours(s, curation_delta=0.01)   # the GIL released meanwhile
//   r.names(), r.surface("axon").num_triangles(), r.surfaces, r.warnings
//
// pycvc.tile_contours_job(s, ...) is the same as a resumable job (pycvc_jobs.i). Bad input (bad
// parameters, a stack with problems, a curation_delta not below the slice spacing) raises
// ValueError. The stepper is not wrapped: the job is how Python steps it. Files are the CLI's
// (cvc tile): write a surface with its geometry's own writers.
%{
#include <cvc/geometry/tiler.h>
#include <cvc/geometry/tiler_params.h>
#include <stdexcept>
%}

// Their lists reach Python as tuples, copies (the members' own getters hand out views).
%naturalvar cvc::tile_params::include;
%naturalvar cvc::tile_params::exclude;
%naturalvar cvc::tile_params::color;

// The result's parts are copies Python owns: the member variables' own getters would hand out
// views into the struct, which dangle once it is collected. (Geometry copies share their data.)
// surface() raises KeyError for a component with none (declared before the %extend).
%exception cvc::tile_result::surface {
  try {
    $action
  } catch (const std::out_of_range &e) {
    PyErr_SetString(PyExc_KeyError, e.what());
    SWIG_fail;
  }
}
%ignore cvc::tile_result::components;
%ignore cvc::tile_result::warnings;
%extend cvc::tile_result {
  std::vector<std::string> names() const {
    std::vector<std::string> out;
    for (const auto &c : $self->components)
      out.push_back(c.first);
    return out;
  }
  // Component `name`'s surface; KeyError for a component with none.
  cvc::geometry surface(const std::string &name) const {
    const auto it = $self->components.find(name);
    if (it == $self->components.end())
      throw std::out_of_range("tile_result: no surface for component '" + name + "'");
    return it->second;
  }
  std::vector<std::string> warnings_copy() const { return $self->warnings; }
  %pythoncode %{
    surfaces = property(lambda self: {n: self.surface(n) for n in self.names()},
                        doc="A dict from each component's name to its surface (a geometry).")
    warnings = property(lambda self: list(self.warnings_copy()),
                        doc="What the tiler skipped or could not do, each naming where.")
    def __repr__(self):
        return "<tile_result: %d surfaces (%s), %d warnings>" % (
            len(self.names()), ", ".join(self.names()), len(self.warnings_copy()))
  %}
}

%ignore cvc::tiler_error;
%ignore cvc::tile_stepper;
%ignore cvc::tile_contours_stepper;
%ignore cvc::tile_contours_job; // pycvc_jobs.i wraps it
%ignore cvc::tile_params_problems; // tile_params.validate() gives them

// Tiling takes a while: other Python threads run meanwhile. Bad input raises ValueError.
%exception cvc::tile_contours {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::tiler_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_tile_contours) cvc::tile_contours;
%include "cvc/geometry/tiler.h"
PYCVC_PARAM_TABLE(tile_params, cvc::tile_params)

%pythoncode %{
def _tile_params(params, keywords, who):
    if params is None:
        return tile_params(**keywords)
    if keywords:
        raise TypeError(who + ": pass a tile_params or keywords, not both")
    return params


def tile_contours(stack, params=None, **keywords):
    """The surfaces between a contour_stack's slices, one per component:
    tile_contours(s, tile_params(...)) or tile_contours(s, curation_delta=0.01, ...) with
    tile_params' fields as keywords."""
    return _tile_contours(stack, _tile_params(params, keywords, "tile_contours"))
%}
