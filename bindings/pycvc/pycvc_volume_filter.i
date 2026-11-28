// pycvc_volume_filter.i — VolumeRover2's volume filters on a volume (inc/cvc/volume/volume_filter.h;
// docs/roadmap/VOLROVER-TEXMOL-APPS-ROADMAP.md V0): bilateral, contrast enhancement, anisotropic
// diffusion and GDTV, each a new volume (the input is never written), with volume_filter_params
// and its parameter table (pycvc_params.i), so they take keywords:
//
//   out = pycvc.filter_volume(vol)                                    # bilateral, the defaults
//   out = pycvc.filter_volume(vol, filter="contrast", contrast_resistor=0.9)
//   out = pycvc.filter_volume(vol, filter="gdtv", gdtv_exponent=1.0, gdtv_neighbours="twenty_six")
//   p = pycvc.volume_filter_params(anisotropic_iterations=5); p.set_text("filter", "anisotropic")
//   out = pycvc.filter_volume(vol, p)
//   job = pycvc.volume_filter_job(vol, filter="anisotropic")          # resumable (pycvc_jobs.i)
//   while not job.step(65536): print(job.phase(), job.progress())
//   out = job.result()
//
// An enum field (filter, gdtv_neighbours) takes its name as text, or pycvc.volume_filter_kind_gdtv
// and pycvc.volume_gdtv_neighbours_twenty_six. Out-of-range parameters, an unknown filter and an
// empty volume raise ValueError; an unknown keyword raises TypeError listing the valid ones. The
// filter runs with the GIL released, so other Python threads go on.

%{
#include <cvc/volume/volume_filter.h>
%}

// The enums' descriptions are for the parameter tables (C++ only).
#ifndef BOOST_DESCRIBE_ENUM
#define BOOST_DESCRIBE_ENUM(E, ...)
#endif

%ignore cvc::param_table<cvc::volume_filter_params>;
%ignore cvc::volume_filter_stepper; // Python steps the job (pycvc_jobs.i)
%ignore cvc::volume_filter_job;     // pycvc_jobs.i wraps it with a cvc::volume result

%exception cvc::filter_volume {
  try {
    pycvc::gil_release pycvc_nogil;
    $action
  } catch (const std::invalid_argument &e) {
    SWIG_exception(SWIG_ValueError, e.what());
  } catch (const std::exception &e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}
%rename(_filter_volume) cvc::filter_volume;
%include "cvc/volume/volume_filter.h"
PYCVC_PARAM_TABLE(volume_filter_params, cvc::volume_filter_params)

%pythoncode %{
def filter_volume(vol, params=None, **keywords):
    """A new volume: `vol` filtered by a volume_filter_params or its fields as keywords (filter
    names the filter: "bilateral", "contrast", "anisotropic" or "gdtv"). `vol` is not written."""
    return _filter_volume(vol, _params_from(volume_filter_params, params, keywords,
                                            "filter_volume"))
%}
