// What the legacy ContourTiler needs of log4cplus, logging nothing, so the goldens build needs no
// log4cplus (tools/volrover-goldens/README.md). Never built into libcvc.
#ifndef VOLROVER_GOLDENS_SHIM_LOG4CPLUS_LOGGER_H
#define VOLROVER_GOLDENS_SHIM_LOG4CPLUS_LOGGER_H

#include <string>

namespace log4cplus {

typedef int LogLevel;
const LogLevel TRACE_LOG_LEVEL = 0;
const LogLevel DEBUG_LOG_LEVEL = 10000;
const LogLevel INFO_LOG_LEVEL = 20000;
const LogLevel WARN_LOG_LEVEL = 30000;
const LogLevel ERROR_LOG_LEVEL = 40000;

class Logger {
public:
  static Logger getInstance(const std::string &) { return Logger(); }
  static Logger getRoot() { return Logger(); }
  void setLogLevel(LogLevel) {}
};

} // namespace log4cplus

#endif
