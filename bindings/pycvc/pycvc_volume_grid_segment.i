// pycvc_volume_grid_segment.i — VolumeGridRover's Segmentation tab, its data side
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §4, §5, P9): each class's surface of a label
// volume (inc/cvc/volume/class_surfaces.h), and what cvcGL's volume_grid node shows of a
// segmentation (inc/cvc/volume/volume_grid_segment.h): the result's classes as rows, a slice of the
// labels as the see-through layer, the tab's view parameters:
//
//   r = pycvc.segment_seeded(vol, seeds, t_low=0.2)            # pycvc_segmentation.i
//   s = pycvc.class_surfaces(r.labels, smoothing=6)            # the GIL released meanwhile
//   s.labels                     # [0, 1]: the labels with a surface
//   s.surface(0).num_triangles() # label 0's surface, a geometry (KeyError for none)
//   s.surfaces, s.voxels, s.warnings                           # dicts by label, a list
//   rows = pycvc.segment_class_rows(seeds, 0, 0, r, s)         # name, color, visible, voxels...
//   rows[0].name, rows[0].color, rows[0].visible
//   pycvc.write_segment_rows(pycvc.State.instance(app).child("doc.grid.segment.rows"), rows)
//   px = pycvc.color_labels(pycvc.extract_slice(r.labels, pycvc.slice_axis_z, 12), rows, 0.5)
//   px.shape                     # (height, width, 4) uint8, the top row first
//   job = pycvc.class_surfaces_job(r.labels, smoothing=6)     # pycvc_jobs.i
//   job = pycvc.volume_grid_segment_job(vol, seeds, t_low=0.2, mesh={"smoothing": 6})
//
// class_surface_params and volume_grid_segment_view_params have their parameter tables
// (pycvc_params.i), so they take keywords. A voxel that is no label and bad parameters raise
// ValueError. The steppers are not wrapped: the jobs are how Python steps them.

%{
#include <cvc/volume/class_surfaces.h>
#include <cvc/volume/volume_grid_segment.h>
#include <stdexcept>
%}

%naturalvar cvc::class_surface_params::labels;

// The result's parts are copies Python owns (the members' own getters hand out views into the
// struct, which dangle once it is collected; geometry copies share their data).
%exception cvc::class_surface_result::surface {
  try {
    $action
  } catch (const std::out_of_range &e) {
    PyErr_SetString(PyExc_KeyError, e.what());
    SWIG_fail;
  }
}
%ignore cvc::class_surface_result::labels;
%ignore cvc::class_surface_result::surfaces;
%ignore cvc::class_surface_result::voxels;
%ignore cvc::class_surface_result::warnings;
%ignore cvc::class_surface_result::surface_of;
%extend cvc::class_surface_result {
  std::vector<int> labels_copy() const { return $self->labels; }
  // Label `label`'s surface; KeyError for a label with none.
  cvc::geometry surface(int label) const {
    const cvc::geometry *g = $self->surface_of(label);
    if (!g)
      throw std::out_of_range("class_surface_result: no surface for label " +
                              std::to_string(label));
    return *g;
  }
  PyObject *voxels_list() const {
    PyObject *out = PyList_New(static_cast<Py_ssize_t>($self->voxels.size()));
    for (std::size_t c = 0; c < $self->voxels.size(); ++c)
      PyList_SET_ITEM(out, static_cast<Py_ssize_t>(c), PyLong_FromSize_t($self->voxels[c]));
    return out;
  }
  std::vector<std::string> warnings_copy() const { return $self->warnings; }
  %pythoncode %{
    labels = property(lambda self: list(self.labels_copy()),
                      doc="The labels that have a surface, ascending.")
    surfaces = property(lambda self: {l: self.surface(l) for l in self.labels_copy()},
                        doc="A dict from each label to its surface (a geometry).")
    voxels = property(lambda self: dict(zip(self.labels_copy(), self.voxels_list())),
                      doc="A dict from each label to how many voxels it has.")
    warnings = property(lambda self: list(self.warnings_copy()),
                        doc="A label asked for that the volume lacks, a label with no surface.")
    def __repr__(self):
        return "<class_surface_result: %d surfaces, %d triangles>" % (
            len(self.labels_copy()), self.triangle_count())
  %}
}

%ignore cvc::class_surfaces_error;
%ignore cvc::class_surfaces_stepper;
%ignore cvc::class_surfaces_job; // pycvc_jobs.i wraps it

// Surfacing takes a while: other Python threads run meanwhile. Bad input raises ValueError.
%exception cvc::class_surfaces {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::class_surfaces_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_class_surfaces) cvc::class_surfaces;
%include "cvc/volume/class_surfaces.h"
PYCVC_PARAM_TABLE(class_surface_params, cvc::class_surface_params)

// ---- the tab's data side ---------------------------------------------------------------------

%ignore cvc::volume_grid_segment_stepper;
%ignore cvc::volume_grid_segment_stepper_of;
%ignore cvc::volume_grid_segment_result::segmentation;
%ignore cvc::volume_grid_segment_result::surfaces;
%ignore cvc::volume_grid_segment_result::volume_grid_segment_result;
%extend cvc::volume_grid_segment_result {
  cvc::seeded_segmentation segmentation_copy() const { return $self->segmentation; }
  cvc::class_surface_result surfaces_copy() const { return $self->surfaces; }
  %pythoncode %{
    segmentation = property(segmentation_copy, doc="The segmentation (a seeded_segmentation).")
    surfaces = property(surfaces_copy, doc="Each class's surface (a class_surface_result).")
  %}
}

%ignore cvc::segment_class_row::color;
%extend cvc::segment_class_row {
  PyObject *color_tuple() const {
    return Py_BuildValue("(ddd)", $self->color[0], $self->color[1], $self->color[2]);
  }
  void set_color(double r, double g, double b) { $self->color = {r, g, b}; }
  %pythoncode %{
    color = property(color_tuple, lambda self, c: self.set_color(*c),
                     doc="(r, g, b) in [0, 1].")
    def __repr__(self):
        return "<segment_class_row %r: color %s, %s, %d voxels, %d triangles>" % (
            self.name, self.color, "shown" if self.visible else "hidden", self.voxels,
            self.triangles)
  %}
}
%template(segment_class_row_vector) std::vector<cvc::segment_class_row>;

// read_segment_rows(rows, rows_list, problems=None) is Python, below; color_labels too (a numpy
// array, not a byte vector).
%rename(_read_segment_rows) cvc::read_segment_rows;
%rename(_segment_class_rows_impl) cvc::segment_class_rows;
%ignore cvc::color_labels;

%include "cvc/volume/volume_grid_segment.h"
PYCVC_PARAM_TABLE(volume_grid_segment_view_params, cvc::volume_grid_segment_view_params)

%inline %{
// The see-through layer of a slice of the labels as a fresh (height, width, 4) uint8 numpy array,
// the top row first (cvc::color_labels).
PyObject *_color_labels(const cvc::volume_slice_data &labels,
                        const std::vector<cvc::segment_class_row> &rows, double opacity) {
  const std::vector<std::uint8_t> px = cvc::color_labels(labels, rows, opacity);
  npy_intp dims[3] = {static_cast<npy_intp>(labels.height), static_cast<npy_intp>(labels.width),
                      4};
  PyObject *arr = PyArray_SimpleNew(3, dims, NPY_UINT8);
  if (!arr)
    return nullptr;
  std::memcpy(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)), px.data(), px.size());
  return arr;
}
%}

%pythoncode %{
def _class_surface_params(params, keywords, who):
    if params is not None:
        if keywords:
            raise TypeError(who + ": pass a class_surface_params or keywords, not both")
        return params
    valid = class_surface_params.fields()
    for k in keywords:
        if k not in valid:
            raise TypeError("%s: unknown keyword '%s'; valid: %s" % (who, k, ", ".join(valid)))
    return class_surface_params(**keywords)


def class_surfaces(labels, params=None, **keywords):
    """Each class's surface of a label volume (a segmentation's labels): class_surfaces(labels,
    class_surface_params(...)) or class_surfaces(labels, smoothing=..., close=..., ...) with the
    params' fields as keywords."""
    return _class_surfaces(labels, _class_surface_params(params, keywords, "class_surfaces"))


def segment_class_rows(seeds, variable, timestep, segmentation, surfaces=None):
    """The rows of a segmentation's classes, coloured as the point classes of `variable` and
    `timestep` that seeded them: a list of segment_class_row (name, color, visible, voxels,
    triangles), triangles from `surfaces` (a class_surface_result) when given."""
    if surfaces is None:
        surfaces = class_surface_result()
    return list(_segment_class_rows_impl(seeds, variable, timestep, segmentation, surfaces))


def read_segment_rows(rows, rows_list, problems=None):
    """The colours and shown flags the rows under `rows` (a State) hold now, into `rows_list` (a
    list of segment_class_row, changed in place); what was wrong is appended to `problems` (a
    list) when it is given. True when any changed."""
    v = segment_class_row_vector(rows_list)
    found = StringVector()
    changed = _read_segment_rows(rows, v, found)
    for i, r in enumerate(v):
        rows_list[i] = r
    if problems is not None:
        problems.extend(found)
    return changed


def color_labels(labels_slice, rows, opacity):
    """A slice of the labels (extract_slice of a segmentation's labels) as the see-through layer
    the grid draws over its plane: a (height, width, 4) uint8 numpy array, the top row first; a
    voxel of class n in rows[n]'s colour with alpha `opacity` when that row is shown, others clear."""
    return _color_labels(labels_slice, segment_class_row_vector(rows), opacity)
%}
