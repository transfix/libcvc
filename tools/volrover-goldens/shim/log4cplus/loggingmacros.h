// The log4cplus logging macros, logging nothing (logger.h says why). The arguments are not
// evaluated.
#ifndef VOLROVER_GOLDENS_SHIM_LOG4CPLUS_LOGGINGMACROS_H
#define VOLROVER_GOLDENS_SHIM_LOG4CPLUS_LOGGINGMACROS_H

#include <log4cplus/logger.h>

#define LOG4CPLUS_TRACE(logger, msg)                                                               \
  do {                                                                                             \
  } while (0)
#define LOG4CPLUS_DEBUG(logger, msg)                                                               \
  do {                                                                                             \
  } while (0)
#define LOG4CPLUS_INFO(logger, msg)                                                                \
  do {                                                                                             \
  } while (0)
#define LOG4CPLUS_WARN(logger, msg)                                                                \
  do {                                                                                             \
  } while (0)
#define LOG4CPLUS_ERROR(logger, msg)                                                               \
  do {                                                                                             \
  } while (0)
#define LOG4CPLUS_FATAL(logger, msg)                                                               \
  do {                                                                                             \
  } while (0)

#endif
