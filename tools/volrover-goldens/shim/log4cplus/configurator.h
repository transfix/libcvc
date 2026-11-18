// log4cplus's configurator.h, for the legacy ContourTiler's includes (logger.h says why).
#ifndef VOLROVER_GOLDENS_SHIM_LOG4CPLUS_CONFIGURATOR_H
#define VOLROVER_GOLDENS_SHIM_LOG4CPLUS_CONFIGURATOR_H

#include <log4cplus/logger.h>

namespace log4cplus {
struct BasicConfigurator {
  static void doConfigure() {}
};
struct PropertyConfigurator {
  static void doConfigure(const char *) {}
};
} // namespace log4cplus

#endif
