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
