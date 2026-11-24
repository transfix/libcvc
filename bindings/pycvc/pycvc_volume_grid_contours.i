// pycvc_volume_grid_contours.i — VolumeGridRover's contours (inc/cvc/volume/volume_grid_contours.h,
// docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, P10): curves drawn per object on a volume
// grid's planes, linear or spline, their editor and gestures, the object rows, stacks, JSON, the
// isocontour tracer and the tiler:
//
//   c = pycvc.grid_contours()
//   c.frame = pycvc.grid_frame_of_volume(vol)          # or frame.dims / .origin / .spacing tuples
//   ed = pycvc.grid_contour_editor(); ed.set_frame(c.frame)
//   ed.add_object(pycvc.grid_contour_object(name="axon", color=[0.2, 0.8, 0.3]))
//   ed.add_point(0, pycvc.slice_axis_z, 12, 3.0, 4.0)  # a grid_point_ref (object, curve, point)
//   ed.add_point(0, pycvc.slice_axis_z, 12, 9.0, 4.5); ed.add_point(0, pycvc.slice_axis_z, 12, 6, 9)
//   ed.close_active()
//   s = pycvc.grid_contours_stack(ed.contours())       # a contour_stack (pycvc_contours.i)
//   r = pycvc.tile_grid_contours(ed.contours(), curation_delta=0.01)   # the GIL released
//   r.surfaces["axon"].num_triangles()
//   for ru in pycvc.trace_slice(vol_slice, c.frame, 0.0): ...       # isocontours of a slice
//
// pycvc.tile_grid_contours_job(c, ...) is the same as a resumable job (pycvc_jobs.i). A frame's
// dims, origin and spacing, and an outcome's band_rect, are tuples. c.objects, an object's curves
// and ed.contours() are copies: change one and set it back whole (c.objects = [o, ...]). ed.nearest_point() is None when
// nothing is near. Bad input (contours with problems, an object that does not exist) raises
// ValueError or IndexError; a tile that can't be made raises ValueError. volume_grid_contour_params
// and grid_contour_object take keywords from their parameter tables.

%{
#include <cvc/geometry/tiler_params.h>
#include <cvc/volume/volume.h>
#include <cvc/volume/volume_grid_contours.h>
#include <cvc/volume/volume_slice.h>
#include <stdexcept>
%}

%template(grid_curve_vector) std::vector<cvc::grid_curve>;
%template(grid_contour_object_vector) std::vector<cvc::grid_contour_object>;
%template(grid_curve_ref_vector) std::vector<cvc::grid_curve_ref>;
%template(grid_point_ref_vector) std::vector<cvc::grid_point_ref>;
%template(DoubleVectorVector) std::vector<std::vector<double>>;

%naturalvar cvc::grid_curve::ru;
%naturalvar cvc::grid_contour_object::color;
%naturalvar cvc::grid_contour_object::curves;
%naturalvar cvc::grid_contours::frame;
%naturalvar cvc::grid_contours::objects;
%naturalvar cvc::volume_grid_session::contours;
%naturalvar cvc::contour_pick_outcome::note;
%naturalvar cvc::volume_grid_contour_params::source;

// The frame's arrays and an outcome's rectangle as tuples, both ways.
%exception cvc::grid_frame::_set_dims {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%exception cvc::grid_frame::_set_origin {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%exception cvc::grid_frame::_set_spacing {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%ignore cvc::grid_frame::dims;
%ignore cvc::grid_frame::origin;
%ignore cvc::grid_frame::spacing;
%extend cvc::grid_frame {
  std::vector<int> _dims() const { return {$self->dims[0], $self->dims[1], $self->dims[2]}; }
  std::vector<double> _origin() const {
    return {$self->origin[0], $self->origin[1], $self->origin[2]};
  }
  std::vector<double> _spacing() const {
    return {$self->spacing[0], $self->spacing[1], $self->spacing[2]};
  }
  void _set_dims(const std::vector<int> &v) {
    if (v.size() != 3)
      throw std::invalid_argument("grid_frame.dims: three values");
    for (int a = 0; a < 3; ++a)
      $self->dims[a] = v[a];
  }
  void _set_origin(const std::vector<double> &v) {
    if (v.size() != 3)
      throw std::invalid_argument("grid_frame.origin: three values");
    for (int a = 0; a < 3; ++a)
      $self->origin[a] = v[a];
  }
  void _set_spacing(const std::vector<double> &v) {
    if (v.size() != 3)
      throw std::invalid_argument("grid_frame.spacing: three values");
    for (int a = 0; a < 3; ++a)
      $self->spacing[a] = v[a];
  }
  %pythoncode %{
    dims = property(lambda self: tuple(self._dims()), lambda self, v: self._set_dims(list(v)),
                    doc="Voxels along x, y and z.")
    origin = property(lambda self: tuple(self._origin()),
                      lambda self, v: self._set_origin(list(v)), doc="Voxel 0's centre.")
    spacing = property(lambda self: tuple(self._spacing()),
                       lambda self, v: self._set_spacing(list(v)),
                       doc="The distance between neighbouring voxel centres along x, y and z.")
    def __repr__(self):
        return "<grid_frame dims %s origin %s spacing %s>" % (self.dims, self.origin,
                                                               self.spacing)
  %}
}
%ignore cvc::contour_pick_outcome::band_rect;
%extend cvc::contour_pick_outcome {
  std::vector<double> _band_rect() const {
    return {$self->band_rect[0], $self->band_rect[1], $self->band_rect[2], $self->band_rect[3]};
  }
  %pythoncode %{
    band_rect = property(lambda self: tuple(self._band_rect()),
                         doc="A rubber band's corners, r0 u0 r1 u1.")
  %}
}

// The editor: std::optional as None, the active curve as three calls.
%ignore cvc::grid_contour_editor::active;
%ignore cvc::grid_contour_editor::set_active;
%ignore cvc::grid_contour_editor::active_curve;
%ignore cvc::grid_contour_editor::nearest_point;
%extend cvc::grid_contour_editor {
  bool has_active() const { return static_cast<bool>($self->active()); }
  cvc::grid_curve_ref active_curve_ref() const {
    if (!$self->active())
      throw std::out_of_range("grid_contour_editor: no active curve");
    return $self->active()->curve;
  }
  bool active_back() const { return $self->active() && $self->active()->back; }
  void set_active_curve(std::size_t object, std::size_t curve, bool back = true) {
    $self->set_active(cvc::grid_contour_editor::active_curve{{object, curve}, back});
  }
  void clear_active() { $self->set_active(std::nullopt); }
  std::vector<cvc::grid_point_ref> _nearest(cvc::slice_axis a, std::int64_t index, double r,
                                            double u, double radius, long object = -1) const {
    std::vector<cvc::grid_point_ref> out;
    if (const auto p = $self->nearest_point(a, index, r, u, radius, object))
      out.push_back(*p);
    return out;
  }
  %pythoncode %{
    def nearest_point(self, a, index, r, u, radius, object=-1):
        """The control point nearest (r, u) within radius on that plane, or None."""
        found = self._nearest(a, index, r, u, radius, object)
        return found[0] if len(found) else None
  %}
}
%exception cvc::grid_contour_editor::active_curve_ref {
  try {
    $action
  } catch (const std::out_of_range &e) {
    PyErr_SetString(PyExc_LookupError, e.what());
    SWIG_fail;
  }
}

// Exceptions: a missing object, curve or point is IndexError; contours with problems ValueError.
%define PYCVC_GRID_CONTOUR_THROWS(NAME)
%exception NAME {
  try {
    $action
  } catch (const std::out_of_range &e) {
    PyErr_SetString(PyExc_IndexError, e.what());
    SWIG_fail;
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::runtime_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%enddef
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::set_contours)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::add_object)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::set_objects)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::add_point)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::add_curves)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::move_point)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::curve)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_editor::point)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contour_gestures::apply)
PYCVC_GRID_CONTOUR_THROWS(cvc::sample_grid_curve)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contours_stack)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contours_from_stack)
PYCVC_GRID_CONTOUR_THROWS(cvc::trace_slice)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contours_json)
PYCVC_GRID_CONTOUR_THROWS(cvc::grid_contours_from_json)

// The frame of a slice_grid comes through a volume here (slice_grid is not wrapped).
%ignore cvc::grid_frame_of;
%inline %{
cvc::grid_frame grid_frame_of_volume(const cvc::volume &v) {
  return cvc::grid_frame_of(cvc::grid_of(v));
}
%}

// The result's parts are copies Python owns, as tile_result's (pycvc_tiler.i).
%exception cvc::grid_tile_result::surface {
  try {
    $action
  } catch (const std::out_of_range &e) {
    PyErr_SetString(PyExc_KeyError, e.what());
    SWIG_fail;
  }
}
%ignore cvc::grid_tile_result::surfaces;
%ignore cvc::grid_tile_result::warnings;
%extend cvc::grid_tile_result {
  std::vector<std::string> names() const {
    std::vector<std::string> out;
    for (const auto &c : $self->surfaces)
      out.push_back(c.first);
    return out;
  }
  cvc::geometry surface(const std::string &name) const {
    const auto it = $self->surfaces.find(name);
    if (it == $self->surfaces.end())
      throw std::out_of_range("grid_tile_result: no surface for object '" + name + "'");
    return it->second;
  }
  std::vector<std::string> warnings_copy() const { return $self->warnings; }
  %pythoncode %{
    surfaces = property(lambda self: {n: self.surface(n) for n in self.names()},
                        doc="A dict from each object's name to its surface (a geometry).")
    warnings = property(lambda self: list(self.warnings_copy()),
                        doc="What the tiler skipped or could not do, each naming where.")
    def __repr__(self):
        return "<grid_tile_result: %d surfaces (%s), %d warnings>" % (
            len(self.names()), ", ".join(self.names()), len(self.warnings_copy()))
  %}
}
%ignore cvc::grid_tile_stepper;
%ignore cvc::tile_grid_contours_job; // pycvc_jobs.i wraps it
%exception cvc::tile_grid_contours {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::tiler_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_tile_grid_contours) cvc::tile_grid_contours;

// The rows take a pycvc.State node; read_contour_object_rows' problems come back as a list.
%rename(_read_contour_object_rows) cvc::read_contour_object_rows;

%include "cvc/volume/volume_grid_contours.h"
PYCVC_PARAM_TABLE(volume_grid_contour_params, cvc::volume_grid_contour_params)
PYCVC_PARAM_TABLE(grid_contour_object, cvc::grid_contour_object)

%pythoncode %{
def read_contour_object_rows(rows, problems=None):
    """The objects a list: widget's rows describe under the State node `rows`; each problem
    found is appended to `problems` (a list) when given."""
    found = StringVector()
    out = list(_read_contour_object_rows(rows, found))
    if problems is not None:
        problems.extend(list(found))
    return out


def tile_grid_contours(contours, params=None, **keywords):
    """A surface per object from a volume grid's closed curves: tile_grid_contours(c,
    tile_params(...)) or tile_grid_contours(c, curation_delta=0.01, ...) with tile_params' fields
    as keywords (lengths in object units)."""
    return _tile_grid_contours(contours, _tile_params(params, keywords, "tile_grid_contours"))
%}
