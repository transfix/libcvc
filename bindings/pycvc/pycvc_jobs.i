// pycvc_jobs.i — cvc::result_job<R> (inc/cvc/core/result_job.h), wrapped once, generically
// (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.4, §1.9). A port's job function returns
// std::unique_ptr<cvc::result_job<R>>; PYCVC_RESULT_JOB(pyname, R) gives that job to Python:
//
//   job = pycvc.contour_tree_job(vol, pycvc.contour_tree_options(max_dim=64))
//   while not job.step(4096):          # the GIL is released inside each step
//       print(job.phase(), job.progress())
//   tree = job.result()                # or job.take_result()
//
// The members: step(budget) (true once done; budget 0 runs to the end), done(), cancel() (from any
// thread), cancelled(), phase(), progress() (in [0, 1], never falling), status(), failed(),
// error(), has_result(), result(), take_result(), work_ms(), max_step_ms() and steps(). result()
// raises ValueError or RuntimeError with the stepper's own message after a failure, and
// RuntimeError before the job is done, after a cancel, and once the result was taken. job.run()
// steps it to the end, calling progress(phase, fraction) after each step when given, and returns
// the result.
//
// result() hands Python a copy, which it owns: a reference into the job would dangle once the job
// is gone (pycvc.contour_tree_job(vol).run() drops its job at once) or its result taken.
// take_result() moves the result out, with no copy.
//
// SWIG cannot read result_job.h's C++20 concepts, so the class's members are declared here as the
// header has them: this is the real cvc::result_job<R>, not a facade. The one difference: result(),
// a const R & in C++, is declared by value, so that SWIG copies it.

%{
#include <cvc/core/result_job.h>
#include <cvc/volume/volume_jobs.h>
#include <stdexcept>
%}

namespace cvc {
template <class R> class result_job {
public:
  virtual ~result_job();
  bool step(std::size_t budget);
  bool done() const;
  void cancel();
  bool cancelled() const;
  const char *phase() const;
  double progress() const;
  std::string status() const;
  bool failed() const;
  std::string error() const;
  bool has_result() const;
  R result() const; // a const R & in C++: by value here, so Python owns a copy
  R take_result();
  double work_ms() const;
  double max_step_ms() const;
  std::size_t steps() const;

protected:
  result_job();
};
} // namespace cvc

%define PYCVC_RESULT_JOB(PYNAME, R)
%unique_ptr(cvc::result_job< R >)
// A step runs the routine: other Python threads go on meanwhile.
%exception cvc::result_job< R >::step {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception cvc::result_job< R >::result {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception cvc::result_job< R >::take_result {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%template(PYNAME) cvc::result_job< R >;
%extend cvc::result_job< R > {
  %pythoncode %{
    def run(self, budget=4096, progress=None):
        """Step to the end (`budget` units a step), calling progress(phase, fraction) after each
        step when given; return the result."""
        while not self.step(budget):
            if progress is not None:
                progress(self.phase(), self.progress())
        return self.result()

    def __repr__(self):
        return "<%s %s>" % (type(self).__name__, self.status())
  %}
}
%enddef

// The volume jobs (cvc/volume/volume_jobs.h): the contour tree (pycvc_contour_tree.i wraps the
// tree and its options) and the resize.
PYCVC_RESULT_JOB(result_job_contour_tree, cvc::contour_tree)
PYCVC_RESULT_JOB(result_job_volume, cvc::volume)
%exception cvc::contour_tree_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
// cvc::dimension is opaque to Python (pycvc.i), so the resize takes nx, ny, nz, as
// volume.resample does.
%ignore cvc::resize_job;
%include "cvc/volume/volume_jobs.h"
%exception resize_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  } catch (const cvc::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
// pycvc.resize_job(vol, nx, ny, nz): cvc::resize_job(vol, cvc::dimension(nx, ny, nz)).
std::unique_ptr<cvc::result_job<cvc::volume>> resize_job(const cvc::volume &src, unsigned long nx,
                                                         unsigned long ny, unsigned long nz) {
  return cvc::resize_job(src, cvc::dimension(nx, ny, nz));
}
%}
// The seeded segmentation job (cvc/volume/segmentation.h; pycvc_segmentation.i wraps the result
// and its parameters): pycvc.segment_seeded_job(vol, seeds, t_low=0.1, ...) or
// (vol, seeds, pycvc.seeded_segmentation_params(...)).
PYCVC_RESULT_JOB(result_job_seeded_segmentation, cvc::seeded_segmentation)
%exception _segment_seeded_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::seeded_segmentation>>
_segment_seeded_job(const cvc::volume &vol, const cvc::point_classes &seeds,
                    const cvc::seeded_segmentation_params &p) {
  return cvc::segment_seeded_job(vol, seeds, p);
}
%}
%pythoncode %{
def segment_seeded_job(vol, seeds, params=None, **keywords):
    """segment_seeded as a resumable job: step(budget), progress(), phase(), cancel(), result()."""
    return _segment_seeded_job(vol, seeds,
                               _seeded_segmentation_params(params, keywords, "segment_seeded_job"))
%}
// The paper's seeds as a job: pycvc.critical_point_seeds_job(vol, cp_feature_threshold=0.3, ...).
PYCVC_RESULT_JOB(result_job_point_classes, cvc::point_classes)
%exception _critical_point_seeds_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::point_classes>>
_critical_point_seeds_job(const cvc::volume &vol, const cvc::seeded_segmentation_params &p) {
  return cvc::critical_point_seeds_job(vol, p);
}
%}
%pythoncode %{
def critical_point_seeds_job(vol, params=None, **keywords):
    """critical_point_seeds as a resumable job: step(budget), progress(), phase(), cancel(),
    result()."""
    return _critical_point_seeds_job(vol, _seeded_segmentation_params(params, keywords,
                                                                     "critical_point_seeds_job"))
%}
// The skeleton job (cvc/geometry/skeleton.h; pycvc_skeleton.i wraps the result and its
// parameters): pycvc.skeletonize_job(g, robust=True, ...) or (g, pycvc.skeleton_params(...)).
PYCVC_RESULT_JOB(result_job_skeleton, cvc::skeleton)
%exception _skeletonize_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::skeleton>> _skeletonize_job(const cvc::geometry &g,
                                                                 const cvc::skeleton_params &p) {
  return cvc::skeletonize_job(g, p);
}
%}
%pythoncode %{
def skeletonize_job(geom, params=None, **keywords):
    """skeletonize as a resumable job: step(budget), progress(), phase(), cancel(), result()."""
    if params is None:
        params = skeleton_params(**keywords)
    elif keywords:
        raise TypeError("skeletonize_job: pass a skeleton_params or keywords, not both")
    return _skeletonize_job(geom, params)
%}
// The tiler job (cvc/geometry/tiler.h; pycvc_tiler.i wraps the result and its parameters):
// pycvc.tile_contours_job(stack, curation_delta=0.01, ...) or (stack, pycvc.tile_params(...)).
PYCVC_RESULT_JOB(result_job_tile_result, cvc::tile_result)
%exception _tile_contours_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::tile_result>>
_tile_contours_job(const cvc::contour_stack &stack, const cvc::tile_params &p) {
  return cvc::tile_contours_job(stack, p);
}
%}
%pythoncode %{
def tile_contours_job(stack, params=None, **keywords):
    """tile_contours as a resumable job: step(budget), progress(), phase(), cancel(), result()."""
    return _tile_contours_job(stack, _tile_params(params, keywords, "tile_contours_job"))
%}
// Each class's surface of a label volume (cvc/volume/class_surfaces.h; pycvc_volume_grid_segment.i
// wraps the result and its parameters): pycvc.class_surfaces_job(labels, smoothing=6, ...).
PYCVC_RESULT_JOB(result_job_class_surface_result, cvc::class_surface_result)
%exception _class_surfaces_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::class_surface_result>>
_class_surfaces_job(const cvc::volume &labels, const cvc::class_surface_params &p) {
  return cvc::class_surfaces_job(labels, p);
}
%}
%pythoncode %{
def class_surfaces_job(labels, params=None, **keywords):
    """class_surfaces as a resumable job: step(budget), progress(), phase(), cancel(), result()."""
    return _class_surfaces_job(labels, _class_surface_params(params, keywords,
                                                             "class_surfaces_job"))
%}
// The volume_grid Segmentation tab's run (cvc/volume/volume_grid_segment.h): segment_seeded,
// then class_surfaces on its labels, as one job with one progress:
// pycvc.volume_grid_segment_job(vol, seeds, t_low=0.2, mesh={"smoothing": 6}, surfaces=True).
PYCVC_RESULT_JOB(result_job_volume_grid_segment_result, cvc::volume_grid_segment_result)
%exception _volume_grid_segment_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::volume_grid_segment_result>>
_volume_grid_segment_job(const cvc::volume &vol, const cvc::point_classes &seeds,
                         const cvc::seeded_segmentation_params &p,
                         const cvc::class_surface_params &mesh, bool surfaces) {
  return cvc::make_result_job(
      std::make_unique<cvc::volume_grid_segment_stepper>(vol, seeds, p, mesh, surfaces));
}
%}
%pythoncode %{
def volume_grid_segment_job(vol, seeds, params=None, mesh=None, surfaces=True, **keywords):
    """The volume_grid's segment run as a resumable job: segment_seeded (a
    seeded_segmentation_params, or its fields as keywords), then (with surfaces) class_surfaces on
    the labels (mesh: a class_surface_params or a dict of its fields). result() is a
    volume_grid_segment_result: .segmentation and .surfaces."""
    p = _seeded_segmentation_params(params, keywords, "volume_grid_segment_job")
    if mesh is None:
        m = class_surface_params()
    elif isinstance(mesh, dict):
        m = _class_surface_params(None, mesh, "volume_grid_segment_job")
    else:
        m = mesh
    return _volume_grid_segment_job(vol, seeds, p, m, bool(surfaces))
%}
// A volume grid's contours tiled (cvc/volume/volume_grid_contours.h; pycvc_volume_grid_contours.i
// wraps the result): pycvc.tile_grid_contours_job(contours, curation_delta=0.01, ...).
PYCVC_RESULT_JOB(result_job_grid_tile_result, cvc::grid_tile_result)
%exception _tile_grid_contours_job {
  try {
    $action
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::grid_tile_result>>
_tile_grid_contours_job(const cvc::grid_contours &c, const cvc::tile_params &p) {
  return cvc::tile_grid_contours_job(c, p);
}
%}
%pythoncode %{
def tile_grid_contours_job(contours, params=None, **keywords):
    """tile_grid_contours as a resumable job: step(budget), progress(), phase(), cancel(),
    result()."""
    return _tile_grid_contours_job(contours,
                                   _tile_params(params, keywords, "tile_grid_contours_job"))
%}
// The molecule jobs (cvc/mol/; pycvc_mol.i wraps the results and their parameters):
// pycvc.build_view_job(app, m, style="cartoon"), blur_job(app, m, resolution=2.0),
// torsions_job(m) and group_tree_job(m). Each copies the molecule; build_view_job and blur_job keep
// the app alive for as long as the job lives.
%{
namespace pycvc {
// A stepper that holds an app & beside the app it holds: declared first, the app outlives it.
template <class S> struct app_stepper {
  std::shared_ptr<cvc::app> app;
  S s;
  template <class... A>
  explicit app_stepper(std::shared_ptr<cvc::app> a, A &&...args)
      : app(std::move(a)), s(*app, std::forward<A>(args)...) {}
  bool step(std::size_t budget) { return s.step(budget); }
  bool done() const { return s.done(); }
  const char *phase() const { return s.phase(); }
  double progress() const { return s.progress(); }
  std::string status() const { return s.status(); }
  void cancel() { s.cancel(); }
  auto take() { return s.take(); }
};
} // namespace pycvc
%}
PYCVC_RESULT_JOB(result_job_molecule_view, cvc::mol::molecule_view)
PYCVC_RESULT_JOB(result_job_residue_torsions, std::vector<cvc::mol::residue_torsions>)
PYCVC_RESULT_JOB(result_job_group_tree, cvc::mol::group_tree)
%exception _build_view_job {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%exception _blur_job {
  try {
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%inline %{
std::unique_ptr<cvc::result_job<cvc::mol::molecule_view>>
_build_view_job(std::shared_ptr<cvc::app> app, const cvc::mol::molecule &m,
                const cvc::mol::view_params &p) {
  if (!app)
    throw std::invalid_argument("build_view_job: app is None");
  return cvc::make_result_job(std::make_unique<pycvc::app_stepper<cvc::mol::build_view_stepper>>(
      std::move(app), std::make_shared<const cvc::mol::molecule>(m), p));
}
std::unique_ptr<cvc::result_job<cvc::volume>> _blur_job(std::shared_ptr<cvc::app> app,
                                                        const cvc::mol::molecule &m,
                                                        const cvc::mol::blur_params &p) {
  if (!app)
    throw std::invalid_argument("blur_job: app is None");
  return cvc::make_result_job(
      std::make_unique<pycvc::app_stepper<cvc::mol::blur_stepper>>(std::move(app), m, p));
}
// pycvc.torsions_job(m): cvc::mol::torsions as a resumable job.
std::unique_ptr<cvc::result_job<std::vector<cvc::mol::residue_torsions>>>
torsions_job(const cvc::mol::molecule &m) {
  return cvc::mol::torsions_job(std::make_shared<const cvc::mol::molecule>(m));
}
// pycvc.group_tree_job(m): cvc::mol::build_group_tree as a resumable job.
std::unique_ptr<cvc::result_job<cvc::mol::group_tree>> group_tree_job(const cvc::mol::molecule &m) {
  return cvc::mol::group_tree_job(std::make_shared<const cvc::mol::molecule>(m));
}
%}
%pythoncode %{
def build_view_job(app, molecule, params=None, **keywords):
    """build_view as a resumable job: step(budget), progress(), phase(), cancel(), result()."""
    return _build_view_job(app, molecule, _params_from(molecule_view_params, params, keywords,
                                                       "build_view_job"))


def blur_job(app, molecule, params=None, **keywords):
    """blur as a resumable job: step(budget), progress(), phase(), cancel(), result()."""
    return _blur_job(app, molecule, _params_from(blur_params, params, keywords, "blur_job"))
%}
