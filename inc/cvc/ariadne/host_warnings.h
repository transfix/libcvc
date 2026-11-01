// Runtime warnings for a runtime host (ari, arigl), reported while a document runs rather than
// only after it exits: a resident (on_tick:, on_key: ...) that stopped, was restarted or given up,
// a program that failed, a broken expression.
//
// A host makes one LiveWarnings, hands it to AppRuntime::set_warning_listener, and each warning
// then goes, as it is produced, to:
//   - the log file, when --log FILE or $ARIADNE_LOG names one: a line each, with the time,
//     appended and flushed;
//   - stderr, when the host asks for it and stderr is not a terminal (ari run redirected with
//     2>file, a pipe, the browser page's console): a full-screen UI owns the terminal, so there
//     they wait for the summary at exit;
//   - the browser page (web_document.h: Module.ariadneWarning), a no-op natively;
//   - notice(): the line a full-screen UI shows for a few seconds (FtxuiBackend's RunOptions).
// At exit, summarize() prints every warning of the run to stderr, or, when they already went there,
// how many there were.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

class LiveWarnings {
public:
  // `runtime` prefixes each line ("ari"). `log_path` is --log's value; empty takes $ARIADNE_LOG,
  // and neither means no log file. `stderr_when_not_a_terminal`: also write each one to stderr
  // when stderr is not a terminal.
  LiveWarnings(std::string runtime, std::string log_path, bool stderr_when_not_a_terminal);
  ~LiveWarnings();
  LiveWarnings(const LiveWarnings &) = delete;
  LiveWarnings &operator=(const LiveWarnings &) = delete;

  // "" when the log file (if any) is open, else why it could not be opened.
  const std::string &error() const { return error_; }
  // The log file's path, "" for none.
  const std::string &log_path() const { return logPath_; }

  // A warning while the document runs: to every place above.
  void operator()(const std::string &warning);
  // A warning the host has already printed (the load's): to the log file only.
  void log(const std::string &warning);

  // The latest warning while it is recent (`seconds`), with how many others came before it; ""
  // otherwise. For a status line.
  std::string notice(double seconds = 8.0) const;
  // How many warnings operator() has seen.
  std::size_t count() const { return count_; }

  // At exit, `rest` being AppRuntime::take_warnings(): each one to stderr as
  // "<runtime>: warning: ...", or, when they went to stderr as they came, one line counting them.
  void summarize(const std::vector<std::string> &rest) const;

private:
  void write_log(const std::string &warning);

  std::string runtime_;
  std::string logPath_;
  std::string error_;
  std::FILE *log_ = nullptr;
  bool toStderr_ = false;
  std::size_t count_ = 0;
  std::string latest_;
  std::chrono::steady_clock::time_point latestAt_{};
};

} // namespace ariadne
} // namespace cvc
