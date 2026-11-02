// A runtime host's live warnings — see inc/cvc/ariadne/host_warnings.h.

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cvc/ariadne/host_warnings.h>
#include <cvc/ariadne/web_document.h>
#include <utility>

#ifdef _WIN32
#include <io.h>
#define ARIADNE_ISATTY _isatty
#define ARIADNE_FILENO _fileno
#else
#include <unistd.h>
#define ARIADNE_ISATTY isatty
#define ARIADNE_FILENO fileno
#endif

namespace cvc {
namespace ariadne {

namespace {
// The runtime's warnings mostly begin with its own name ("ari: on_tick: ..."); the host's prefix
// already says it.
std::string trimmed(const std::string &runtime, const std::string &w) {
  const std::string own = runtime + ": ";
  return w.rfind(own, 0) == 0 ? w.substr(own.size()) : w;
}
} // namespace

LiveWarnings::LiveWarnings(std::string runtime, std::string log_path,
                           bool stderr_when_not_a_terminal)
    : runtime_(std::move(runtime)), logPath_(std::move(log_path)) {
  if (logPath_.empty())
    if (const char *env = std::getenv("ARIADNE_LOG"))
      logPath_ = env;
  if (!logPath_.empty()) {
    log_ = std::fopen(logPath_.c_str(), "a");
    if (!log_)
      error_ = "--log " + logPath_ + ": " + std::strerror(errno);
  }
  toStderr_ = stderr_when_not_a_terminal && !ARIADNE_ISATTY(ARIADNE_FILENO(stderr));
}

LiveWarnings::~LiveWarnings() {
  if (log_)
    std::fclose(log_);
}

void LiveWarnings::write_log(const std::string &warning) {
  if (!log_)
    return;
  char when[32] = "";
  const std::time_t now = std::time(nullptr);
  if (const std::tm *t = std::localtime(&now))
    std::strftime(when, sizeof when, "%Y-%m-%d %H:%M:%S", t);
  std::fprintf(log_, "%s %s: warning: %s\n", when, runtime_.c_str(),
               trimmed(runtime_, warning).c_str());
  std::fflush(log_);
}

void LiveWarnings::log(const std::string &warning) { write_log(warning); }

void LiveWarnings::operator()(const std::string &warning) {
  ++count_;
  latest_ = trimmed(runtime_, warning);
  latestAt_ = std::chrono::steady_clock::now();
  write_log(warning);
  if (toStderr_) {
    std::fprintf(stderr, "%s: warning: %s\n", runtime_.c_str(), latest_.c_str());
    std::fflush(stderr);
  }
  report_warning(warning);
}

std::string LiveWarnings::notice(double seconds) const {
  if (!count_ ||
      std::chrono::steady_clock::now() - latestAt_ > std::chrono::duration<double>(seconds))
    return {};
  std::string n = runtime_ + ": warning: " + latest_;
  if (n.size() > 300) // a few lines at most: the log and the summary have all of it
    n = n.substr(0, 297) + "...";
  // How many so far, and where to read them all: the log, else the list at exit (on stderr each
  // came as it happened, and the exit only counts them).
  std::string more = count_ > 1 ? std::to_string(count_) + " warnings" : std::string();
  const std::string where = log_ ? "in " + logPath_ : toStderr_ ? "" : "listed at exit";
  if (!more.empty() && !where.empty())
    more += "; ";
  more += where;
  if (!more.empty())
    n += "  (" + more + ")";
  return n;
}

void LiveWarnings::summarize(const std::vector<std::string> &rest) const {
  if (!toStderr_) {
    for (const std::string &w : rest)
      std::fprintf(stderr, "%s: warning: %s\n", runtime_.c_str(), trimmed(runtime_, w).c_str());
    return;
  }
  if (count_)
    std::fprintf(stderr, "%s: %zu warning%s while the document ran (above)\n", runtime_.c_str(),
                 count_, count_ == 1 ? "" : "s");
}

} // namespace ariadne
} // namespace cvc
