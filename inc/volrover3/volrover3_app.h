#ifndef VOLROVER3_APP_H
#define VOLROVER3_APP_H

#include <cvc/core/app.h>

// --------------------------------------------------------------------
// volrover3_app()
// --------------------------------------------------------------------
// Purpose:
//   Returns a reference to the per-process cvc::app instance used by
//   volrover3.  Uses a function-local static so callers never touch
//   cvc::app::instance() (the singleton being phased out).
// --------------------------------------------------------------------
namespace volrover3 {
cvc::app &app();
}

#endif
