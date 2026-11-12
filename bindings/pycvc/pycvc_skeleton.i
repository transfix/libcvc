// pycvc_skeleton.i — the skeleton of a point cloud (inc/cvc/geometry/skeleton.h), the VolumeRover
// Skeletonization port (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §2): skeletonize on a
// geometry's points, its result, and skeleton_params with its parameter table
// (inc/cvc/geometry/skeleton_params.h), so it takes keywords:
//
//   g = pycvc.geometry(app); g.add_vertices(xyz)      # a flat [x, y, z, ...] list
//   s = pycvc.skeletonize(g, robust=True, sheet_count=3)   # the GIL released meanwhile
//   s.strands.num_lines(), s.sheets.num_triangles(), s.polylines, s.warnings
//
// pycvc.skeletonize_job(g, ...) is the same as a resumable job (pycvc_jobs.i). The stepper and the
// points-vector overloads are not wrapped: a geometry is how Python passes points.
%{
#include <cvc/geometry/skeleton.h>
#include <cvc/geometry/skeleton_params.h>
%}

namespace std {
  %template(IntVector) vector<int>;
}

// The result's parts are copies Python owns: the member variables' own getters would hand out
// views into the struct, which dangle once it is collected.
%ignore cvc::skeleton::strands;
%ignore cvc::skeleton::sheets;
%ignore cvc::skeleton::helix;
%ignore cvc::skeleton::strand_of_line;
%ignore cvc::skeleton::sheet_of_tri;
%ignore cvc::skeleton::warnings;
%extend cvc::skeleton {
  cvc::geometry strands_copy() const { return $self->strands; }
  cvc::geometry sheets_copy() const { return $self->sheets; }
  cvc::geometry helix_copy() const { return $self->helix; }
  std::vector<int> strand_of_line_copy() const { return $self->strand_of_line; }
  std::vector<int> sheet_of_tri_copy() const { return $self->sheet_of_tri; }
  std::vector<std::string> warnings_copy() const { return $self->warnings; }
  %pythoncode %{
    strands = property(strands_copy, doc="Lines: the polylines, then the small clusters' stars.")
    sheets = property(sheets_copy, doc="Triangles: the sheets' Voronoi facets, fanned.")
    helix = property(helix_copy, doc="Lines: the helix strips (with helix_window).")
    strand_of_line = property(strand_of_line_copy,
                              doc="Per strands line: its polyline, or -1 - c for cluster c's star.")
    sheet_of_tri = property(sheet_of_tri_copy, doc="Per sheets triangle: its sheet, 0 the largest.")
    warnings = property(warnings_copy, doc="What the legacy code wrote to files or the console.")
    def __repr__(self):
        return "<skeleton: %d strands, %d sheets of %d clusters, %d warnings>" % (
            self.polylines, self.sheet_clusters, self.clusters, len(self.warnings))
  %}
}

%ignore cvc::skeleton_error;
%ignore cvc::skeleton_stepper;
%ignore cvc::skeletonize_stepper;
%rename("$ignore", fullname=1) cvc::skeletonize(const std::vector<std::array<double, 3>> &,
                                                const skeleton_params &);
%rename("$ignore", fullname=1) cvc::skeletonize(const std::vector<std::array<double, 3>> &);
%ignore cvc::skeletonize_job;  // pycvc_jobs.i wraps the geometry overload

// A skeleton takes a while: other Python threads run meanwhile. Bad input raises ValueError.
%exception cvc::skeletonize {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::skeleton_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_skeletonize) cvc::skeletonize;
%include "cvc/geometry/skeleton.h"
PYCVC_PARAM_TABLE(skeleton_params, cvc::skeleton_params)

%pythoncode %{
def skeletonize(geom, params=None, **keywords):
    """The skeleton of a geometry's points: skeletonize(g, skeleton_params(...)) or
    skeletonize(g, robust=True, ...) with skeleton_params' fields as keywords."""
    if params is None:
        params = skeleton_params(**keywords)
    elif keywords:
        raise TypeError("skeletonize: pass a skeleton_params or keywords, not both")
    return _skeletonize(geom, params)
%}
