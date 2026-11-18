/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// What replaces the legacy library's log4cplus (VOLROVER-LIBRARY-PORTS-ROADMAP.md §1.3): 156
// Logger::getInstance calls and 104 function-static loggers, every one of them process-wide. Here
// the log is a member of the call's tiler_context, and a Logger is a handle on it: a function that
// logs takes the context and asks it for a logger by the legacy name.
//
//   void f(tiler_context &ctx) {
//     Logger logger = ctx.logger("tiler.tile");
//     TILER_WARN(logger, "skipped " << n << " contours");
//   }
//
// Trace and debug output (403 of the legacy calls) is compiled out: the macros drop their
// arguments unevaluated. Info goes to the log; warnings and errors also become the result's
// warnings (tile_result::warnings). Nothing here is process-wide.

#ifndef CVC_GEOMETRY_TILER_CONTEXT_H
#define CVC_GEOMETRY_TILER_CONTEXT_H

#include <sstream>
#include <string>
#include <vector>

namespace cvc::tiler {

enum class log_level { trace, debug, info, warn, error };

// One call's messages, in order.
class tiler_log {
public:
  struct entry {
    log_level level;
    std::string logger;
    std::string message;
  };
  void write(log_level level, const char *logger, std::string message) {
    _entries.push_back({level, logger ? logger : "", std::move(message)});
  }
  const std::vector<entry> &entries() const { return _entries; }
  // Warnings and errors since entry `from`, as "logger: message".
  std::vector<std::string> warnings(std::size_t from = 0) const {
    std::vector<std::string> out;
    for (std::size_t i = from; i < _entries.size(); ++i)
      if (_entries[i].level >= log_level::warn)
        out.push_back(_entries[i].logger + ": " + _entries[i].message);
    return out;
  }
  void clear() { _entries.clear(); }

private:
  std::vector<entry> _entries;
};

// A named handle on a log (log4cplus::Logger's place). Cheap to copy; logs nothing without a log.
class Logger {
public:
  Logger() = default;
  Logger(tiler_log *log, const char *name) : _log(log), _name(name) {}
  bool enabled() const { return _log != nullptr; }
  void write(log_level level, std::string message) const {
    if (_log)
      _log->write(level, _name, std::move(message));
  }

private:
  tiler_log *_log = nullptr;
  const char *_name = "";
};

// The per-call state the legacy library kept in process-wide objects. One per tile_contours()
// call (or stepper); never shared between calls.
struct tiler_context {
  tiler_log log;
  Logger logger(const char *name) { return Logger(&log, name); }

  // Where the work is, for tile_stepper's phase() and progress(): the phase's name, its share
  // [lo, hi] of the whole, and what it is on ("slices 3 - 4"). progress never goes back.
  const char *phase = "snapping";
  double progress = 0;
  std::string label;
  void stage(const char *name, double lo, double hi) {
    phase = name;
    _lo = lo;
    _hi = hi;
    advance(0);
  }
  // `within` of the current phase is done.
  void advance(double within) {
    const double w = within < 0 ? 0 : (within > 1 ? 1 : within);
    const double p = _lo + (_hi - _lo) * w;
    if (p > progress)
      progress = p;
  }

private:
  double _lo = 0, _hi = 0;
};

} // namespace cvc::tiler

#define TILER_LOG_AT_(logger, level, msg)                                                          \
  do {                                                                                             \
    if ((logger).enabled()) {                                                                      \
      std::ostringstream tiler_log_os_;                                                            \
      tiler_log_os_ << msg;                                                                        \
      (logger).write(level, tiler_log_os_.str());                                                  \
    }                                                                                              \
  } while (0)

#define TILER_TRACE(logger, msg)                                                                   \
  do {                                                                                             \
  } while (0)
#define TILER_DEBUG(logger, msg)                                                                   \
  do {                                                                                             \
  } while (0)
#define TILER_INFO(logger, msg) TILER_LOG_AT_(logger, ::cvc::tiler::log_level::info, msg)
#define TILER_WARN(logger, msg) TILER_LOG_AT_(logger, ::cvc::tiler::log_level::warn, msg)
#define TILER_ERROR(logger, msg) TILER_LOG_AT_(logger, ::cvc::tiler::log_level::error, msg)

#endif // CVC_GEOMETRY_TILER_CONTEXT_H
