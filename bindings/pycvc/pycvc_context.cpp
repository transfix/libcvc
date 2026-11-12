// pycvc_context.cpp — app creation. No module-global state whatsoever.
#include "pycvc_context.h"

#include <cvc/core/app.h>
#include <cvc/state/state_exec/lock_manager.h>

namespace pycvc {

std::shared_ptr<cvc::app> make_app() {
  auto app = std::make_shared<cvc::app>();
  // pycvc pumps the app's schedulers on threads of its own (an Exec's run() on any Python
  // thread): its lock manager's auto mode picks shared (gap G-24).
  cvc::state_exec::detail::note_threaded_host(*app);
  return app;
}

} // namespace pycvc
