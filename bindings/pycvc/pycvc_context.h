// pycvc_context.h — app creation. There is NO module-global "current app" and
// no attach()/detach(): pycvc never holds a singleton. The cvc::app is threaded
// EXPLICITLY into every constructor and operation (pycvc.volume(app),
// pycvc.state_set(app, ...), pycvc.sdf(app, ...), observer.watch(app)). A host
// (VolRover's embedded interpreter) passes the shared_ptr<cvc::app> it already
// runs under; standalone code makes its own with make_app() (a.k.a. pycvc.App).
//
// cvc::app is non-copyable / non-movable and heavyweight, so it only ever
// crosses the boundary as a std::shared_ptr<cvc::app> (std, not boost). It is
// forward-declared here; the .cpp includes <cvc/core/app.h>.
#pragma once

#include <memory>

namespace cvc {
class app;
}

namespace pycvc {

// Create a fresh, independently-owned cvc::app. Two apps are two independent
// state trees; sharing one app (this handle, or the host's) shares everything.
std::shared_ptr<cvc::app> make_app();

} // namespace pycvc
