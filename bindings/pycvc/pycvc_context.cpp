// pycvc_context.cpp — app creation. No module-global state whatsoever.
#include "pycvc_context.h"

#include <cvc/core/app.h>

namespace pycvc {

std::shared_ptr<cvc::app> make_app() { return std::make_shared<cvc::app>(); }

} // namespace pycvc
