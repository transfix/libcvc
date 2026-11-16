// OS signals for a host program (ari, arigl, arisrv, cvc serve), sent to one cvc::app.
//
// Each app keeps its own signal state, its host_signals (signals_of(app)): a stop flag, and the
// signals trapped for a document's on_signal:. Nothing about it is process-wide but the route: a
// signal handler can reach only static storage, so the process keeps one pointer, to the
// host_signals its signals go to. A signal_route sets that pointer and the handlers for as long
// as it lives, and puts the previous ones back when it goes.
//
// - signal_route::mode::stop: SIGINT and SIGTERM set the app's stop flag; a server polls
//   stop_requested() in its loop (arisrv, cvc serve). On Windows too (std::signal).
// - signal_route::mode::trap: SIGTERM, SIGHUP, SIGINT, SIGQUIT, SIGUSR1 and SIGUSR2 are recorded
//   for the app. The host polls take_trapped() once per frame and passes each name to
//   AppRuntime::post_signal (or gives it to FtxuiBackend::RunOptions::take_signal, which does the
//   same); a signal the document declares no handler for is the host's to act on,
//   conventionally by exiting with 128 + signal_number(name). On Windows nothing is trapped.
//
// Signal handlers are process-wide, so nothing in libcvc routes signals by itself: only a host
// program does, around its own loop. Routes nest, the newest going first; the app must outlive
// its route.
#pragma once

#include <atomic>
#include <memory>
#include <string>

namespace cvc {
class app;

namespace ariadne {

// One app's signal state: made with the app, gone with it (app::slot), so two apps never see each
// other's stop or signals. Its flags are lock-free atomics, so a signal handler may set them and
// any thread may read them.
class host_signals {
public:
  static constexpr const char *app_slot = "cvc.ariadne.host_signals";

  // Whether this app has been asked to stop: by request_stop(), or by a SIGINT or SIGTERM a stop
  // route sent it. It stays set.
  bool stop_requested() const noexcept { return stop_.load(); }
  void request_stop() noexcept { stop_.store(true); }

  // The name of one signal a trap route recorded for this app since the last call ("SIGTERM",
  // ...), or "" when none did. Each is taken once.
  std::string take_trapped();

private:
  friend struct signal_handlers; // host_signals.cpp's handlers set these
  std::atomic<bool> stop_{false};
  std::atomic<unsigned> trapped_{0}; // a bit per trapped signal, in host_signals.cpp's order
};

// The app's signal state: a.slot<host_signals>().
host_signals &signals_of(cvc::app &a);

// While it lives, the process's signals go to one app's host_signals, as `m` says.
class signal_route {
public:
  enum class mode { stop, trap };

  signal_route(cvc::app &a, mode m);
  ~signal_route(); // the previous handlers, and the previous route, come back
  signal_route(const signal_route &) = delete;
  signal_route &operator=(const signal_route &) = delete;

private:
  struct saved; // the handlers this route replaced, and the route before it
  std::unique_ptr<saved> saved_;
};

// The number for a trapped signal's name; 0 for any other name.
int signal_number(const std::string &name);

} // namespace ariadne
} // namespace cvc
