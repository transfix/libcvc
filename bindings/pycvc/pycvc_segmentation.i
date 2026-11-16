// pycvc_segmentation.i — seeded segmentation of a volume (inc/cvc/volume/segmentation.h), the
// VolumeRover GenSeg port (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §5): segment_seeded on a
// volume and point classes, its result, and seeded_segmentation_params with its parameter table
// (inc/cvc/volume/segmentation_params.h), so it takes keywords:
//
//   seeds = pycvc.point_classes(); seeds.add("virus", 40, 40, 40); seeds.add("solvent", 2, 2, 2)
//   r = pycvc.segment_seeded(vol, seeds, t_low=0.1, t_high=0.8)  # the GIL released meanwhile
//   r.labels_array()               # the labels, a zero-copy uint8 numpy view (255: no class)
//   r.labels, r.classes[0]         # the labels' and each class's volume (copies sharing voxels)
//   r.class_names, r.class_voxels, r.border_array(), r.warnings
//   pycvc.segment_seeded(vol, seeds, automatic="none", normalized_thresholds=True, t_low=40)
//
// An enum field (automatic) takes its name as text, or pycvc.seed_mode_none / _local_maxima.
// pycvc.segment_seeded_job(vol, seeds, ...) is the same as a resumable job (pycvc_jobs.i). Bad
// input (no seeds, a seed outside the volume, a constant volume, bad parameters) raises ValueError.
// The stepper is not wrapped: the job is how Python steps it.
%{
#include <cvc/volume/segmentation.h>
#include <cvc/volume/segmentation_params.h>
%}

// The result's parts are copies Python owns: the member variables' own getters would hand out
// views into the struct, which dangle once it is collected. (Volume copies share their voxels.)
%ignore cvc::seeded_segmentation::seeded_segmentation;
%ignore cvc::seeded_segmentation::labels;
%ignore cvc::seeded_segmentation::classes;
%ignore cvc::seeded_segmentation::border;
%ignore cvc::seeded_segmentation::class_names;
%ignore cvc::seeded_segmentation::class_voxels;
%ignore cvc::seeded_segmentation::warnings;
%ignore cvc::seeded_segmentation::unlabelled;
%extend cvc::seeded_segmentation {
  cvc::volume labels_copy() const { return $self->labels; }
  // Zero-copy numpy view of the labels: shape (nz, ny, nx), uint8, 255 where no class reached.
  // The view pins the labels' voxels, so it outlives the result.
  pycvc::ArrayView labels_array() {
    pycvc::ArrayView v;
    v.dtype = pycvc::DType::UInt8;
    v.writable = false;
    v.shape = {static_cast<long>($self->labels.ZDim()), static_cast<long>($self->labels.YDim()),
               static_cast<long>($self->labels.XDim())};
    v.data = $self->labels.data_ptr();
    v.owner = $self->labels.active_storage();
    return v;
  }
  int num_classes() const { return int($self->class_names.size()); }
  // Class c's volume (its voxels, zero elsewhere); IndexError past the classes there are.
  cvc::volume class_volume(int c) const {
    if (c < 0 || std::size_t(c) >= $self->classes.size())
      throw std::out_of_range("seeded_segmentation: no class volume " + std::to_string(c));
    return $self->classes[std::size_t(c)];
  }
  std::vector<std::string> class_names_copy() const { return $self->class_names; }
  std::vector<std::string> warnings_copy() const { return $self->warnings; }
  PyObject *class_voxels_list() const {
    PyObject *out = PyList_New(static_cast<Py_ssize_t>($self->class_voxels.size()));
    for (std::size_t c = 0; c < $self->class_voxels.size(); ++c)
      PyList_SET_ITEM(out, static_cast<Py_ssize_t>(c),
                      PyLong_FromSize_t($self->class_voxels[c]));
    return out;
  }
  // The border voxels as a fresh (n, 3) int32 numpy array of i, j, k.
  PyObject *border_array() const {
    npy_intp dims[2] = {static_cast<npy_intp>($self->border.size()), 3};
    PyObject *arr = PyArray_SimpleNew(2, dims, NPY_INT32);
    if (!arr)
      return nullptr;
    auto *d = static_cast<std::int32_t *>(PyArray_DATA(reinterpret_cast<PyArrayObject *>(arr)));
    for (std::size_t n = 0; n < $self->border.size(); ++n)
      for (int a = 0; a < 3; ++a)
        d[3 * n + a] = static_cast<std::int32_t>($self->border[n][a]);
    return arr;
  }
  %pythoncode %{
    labels = property(labels_copy, doc="The labels: a UChar volume, 255 where no class reached.")
    classes = property(lambda self: [self.class_volume(c)
                                     for c in range(self._class_volume_count())],
                       doc="Per class, its voxels (zero elsewhere); [] without class_volumes.")
    class_names = property(lambda self: list(self.class_names_copy()), doc="The classes' names.")
    class_voxels = property(class_voxels_list, doc="Per class, how many voxels it labels.")
    warnings = property(lambda self: list(self.warnings_copy()),
                        doc="Odd seeds: below t_low, in two classes, a class with none.")
    def __repr__(self):
        return "<seeded_segmentation: %d classes, %d voxels labelled, %d border voxels>" % (
            self.num_classes(), sum(self.class_voxels), len(self.border_array()))
  %}
}
%extend cvc::seeded_segmentation {
  int _class_volume_count() const { return int($self->classes.size()); }
}
%exception cvc::seeded_segmentation::class_volume {
  try {
    $action
  } catch (const std::out_of_range &e) {
    SWIG_exception(SWIG_IndexError, e.what());
  }
}

%ignore cvc::segmentation_error;
%ignore cvc::segment_stepper;
%ignore cvc::segment_seeded_stepper;
%ignore cvc::segment_seeded_job; // pycvc_jobs.i wraps it

// A segmentation takes a while: other Python threads run meanwhile. Bad input raises ValueError.
%exception cvc::segment_seeded {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const cvc::segmentation_error &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_segment_seeded) cvc::segment_seeded;
%include "cvc/volume/segmentation.h"
PYCVC_PARAM_TABLE(seeded_segmentation_params, cvc::seeded_segmentation_params)

%pythoncode %{
def _seeded_segmentation_params(params, keywords, fn):
    """The params from a seeded_segmentation_params or keywords (an enum's name as text)."""
    if params is not None:
        if keywords:
            raise TypeError(fn + ": pass a seeded_segmentation_params or keywords, not both")
        return params
    valid = seeded_segmentation_params.fields()
    for k in keywords:
        if k not in valid:
            raise TypeError("%s: unknown keyword '%s'; valid: %s" % (fn, k, ", ".join(valid)))
    p = seeded_segmentation_params(**{k: v for k, v in keywords.items()
                                      if not isinstance(v, str)})
    for k, v in keywords.items():
        if isinstance(v, str):
            p.set_text(k, v)
    return p


def segment_seeded(vol, seeds, params=None, **keywords):
    """The seeded segmentation of a volume: segment_seeded(vol, seeds, seeded_segmentation_params(
    ...)) or segment_seeded(vol, seeds, t_low=..., t_high=..., ...) with the params' fields as
    keywords. `seeds` is a point_classes; the classes of `variable` and `timestep` seed the run."""
    return _segment_seeded(vol, seeds,
                           _seeded_segmentation_params(params, keywords, "segment_seeded"))
%}
