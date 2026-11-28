// pycvc_tight_cocone.i — a watertight surface through a point cloud (inc/cvc/geometry/tight_cocone.h),
// the VolumeRover TightCocone port (docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md §2.4, V1):
// tight_cocone on a geometry's points, its result, and tight_cocone_params with its parameter
// table (inc/cvc/geometry/tight_cocone_params.h), so it takes keywords:
//
//   g = pycvc.geometry(app); g.add_vertices(xyz)      # a flat [x, y, z, ...] list
//   r = pycvc.tight_cocone(g, robust=True)            # the GIL released meanwhile
//   r.surface.num_triangles(), r.point_index, r.inside_cells, r.warnings
//
// pycvc.tight_cocone_job(g, ...) is the same as a resumable job (pycvc_jobs.i). The stepper and the
// points-vector overloads are not wrapped: a geometry is how Python passes points.
%{
#include <cvc/geometry/tight_cocone.h>
#include <cvc/geometry/tight_cocone_params.h>
%}

// The result's parts are copies Python owns: the member variables' own getters would hand out
// views into the struct, which dangle once it is collected.
%ignore cvc::tight_cocone_result::surface;
%ignore cvc::tight_cocone_result::point_index;
%ignore cvc::tight_cocone_result::warnings;
%extend cvc::tight_cocone_result {
  cvc::geometry surface_copy() const { return $self->surface; }
  std::vector<unsigned long> point_index_copy() const {
    return std::vector<unsigned long>($self->point_index.begin(), $self->point_index.end());
  }
  std::vector<std::string> warnings_copy() const { return $self->warnings; }
  %pythoncode %{
    surface = property(surface_copy,
                       doc="Triangles, facing out, over the input points the surface uses.")
    point_index = property(point_index_copy,
                           doc="Per surface point: its index in the input points.")
    warnings = property(warnings_copy, doc="What the legacy code printed or did silently.")
    def __repr__(self):
        return "<tight_cocone_result: %d triangles over %d of %d points, %d warnings>" % (
            self.surface.num_triangles(), len(self.point_index), self.points,
            len(self.warnings))
  %}
}

%ignore cvc::tight_cocone_error;
%ignore cvc::tight_cocone_stepper;
%rename("$ignore", fullname=1) cvc::tight_cocone(const std::vector<std::array<double, 3>> &,
                                                 const tight_cocone_params &);
%rename("$ignore", fullname=1) cvc::tight_cocone(const std::vector<std::array<double, 3>> &);
%ignore cvc::tight_cocone_job;  // pycvc_jobs.i wraps the geometry overload

// A surface takes a while: other Python threads run meanwhile. Bad input raises ValueError.
%exception cvc::tight_cocone {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::tight_cocone_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_tight_cocone) cvc::tight_cocone;
%include "cvc/geometry/tight_cocone.h"
PYCVC_PARAM_TABLE(tight_cocone_params, cvc::tight_cocone_params)

%pythoncode %{
def tight_cocone(geom, params=None, **keywords):
    """A watertight surface through a geometry's points: tight_cocone(g, tight_cocone_params(...))
    or tight_cocone(g, robust=True, ...) with tight_cocone_params' fields as keywords."""
    if params is None:
        params = tight_cocone_params(**keywords)
    elif keywords:
        raise TypeError("tight_cocone: pass a tight_cocone_params or keywords, not both")
    return _tight_cocone(geom, params)
%}
