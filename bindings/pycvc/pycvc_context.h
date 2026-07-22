// pycvc_context.h — shared process-wide cvc::app for the State/Exec facades.
//
// This header is DELIBERATELY NOT %included by pycvc.i: SWIG never parses it.
// The two facade translation units (pycvc_state.cpp, pycvc_exec.cpp) include
// it so that pycvc.State and pycvc.Exec bind to the SAME app root
// (cvc::state::instance(process_ctx())). Sharing one app is what makes the
// Python -> state -> DSL round trip work: a value written through pycvc.State
// is visible to a DSL program run through pycvc.Exec, and vice-versa.
//
// (pycvc_volume.cpp / pycvc_geometry.cpp keep their own private ctx() — they
// carry no shared state, so they don't need this.)
#pragma once

namespace cvc {
class app;
}

namespace pycvc {

// The one process-wide cvc::app shared by the State and Exec facades. Defined
// once in pycvc_state.cpp using the same `static cvc::app app;` idiom as
// pycvc_volume.cpp's ctx().
cvc::app &process_ctx();

} // namespace pycvc
