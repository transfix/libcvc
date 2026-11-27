// OS signals for a host program, sent to one app — see inc/cvc/ariadne/host_signals.h.

#if defined(__wasi__)
// wasip1 delivers no signals to a program: wasi-libc's <signal.h> is an #error without its
// emulation, and the emulation's handlers only ever run from raise(). A route therefore has
// nothing to take; a stop comes from request_stop() alone, and nothing is ever trapped.
#include <cvc/ariadne/host_signals.h>
#include <cvc/core/app.h>

namespace cvc {
namespace ariadne {

std::string host_signals::take_trapped() { return {}; }

host_signals &signals_of(cvc::app &a) { return a.slot<host_signals>(); }

struct signal_route::saved {};

signal_route::signal_route(cvc::app &, mode) : saved_(std::make_unique<saved>()) {}

signal_route::~signal_route() = default;

int signal_number(const std::string &) { return 0; }

} // namespace ariadne
} // namespace cvc

#else // !__wasi__

#include <csignal>
#include <cvc/ariadne/host_signals.h>
#include <cvc/core/app.h>

namespace cvc {
namespace ariadne {

namespace {

static_assert(std::atomic<host_signals *>::is_always_lock_free &&
                  std::atomic<bool>::is_always_lock_free &&
                  std::atomic<unsigned>::is_always_lock_free,
              "a signal handler may only touch lock-free atomics");

// Where the process's signals go: the routed app's host_signals, or null when no route is up. A
// signal handler can reach only static storage, so this one pointer is the process's, as the
// handlers are; signal_route sets it. The flags it points at are that app's own.
std::atomic<host_signals *> g_raised{nullptr};

// The signals a stop route takes.
constexpr int kStops[] = {SIGINT, SIGTERM};
constexpr int kStopCount = static_cast<int>(sizeof(kStops) / sizeof(kStops[0]));

#ifndef _WIN32
// The signals a trap route records, in host_signals::trapped_'s bit order.
struct Trap {
  int signo;
  const char *name;
};
constexpr Trap kTraps[] = {{SIGTERM, "SIGTERM"}, {SIGHUP, "SIGHUP"},   {SIGINT, "SIGINT"},
                           {SIGQUIT, "SIGQUIT"}, {SIGUSR1, "SIGUSR1"}, {SIGUSR2, "SIGUSR2"}};
constexpr int kTrapCount = static_cast<int>(sizeof(kTraps) / sizeof(kTraps[0]));
constexpr int kMaxRouted = kTrapCount;
static_assert(kStopCount <= kMaxRouted, "a stop route saves its handlers in the same arrays");
#else
constexpr int kMaxRouted = kStopCount;
#endif

} // namespace

// The handlers' way in to an app's flags. Only lock-free atomics: all a handler may touch.
struct signal_handlers {
  static void stop() noexcept {
    if (host_signals *s = g_raised.load())
      s->stop_.store(true);
  }
#ifndef _WIN32
  static void trap(int signo) noexcept {
    host_signals *s = g_raised.load();
    if (!s)
      return;
    for (int i = 0; i < kTrapCount; ++i)
      if (kTraps[i].signo == signo)
        s->trapped_.fetch_or(1u << i);
  }
  static std::string take(host_signals &s) {
    for (int i = 0; i < kTrapCount; ++i) {
      const unsigned bit = 1u << i;
      if (s.trapped_.fetch_and(~bit) & bit)
        return kTraps[i].name;
    }
    return {};
  }
#endif
};

namespace {
extern "C" void on_stop_signal(int) { signal_handlers::stop(); }
#ifndef _WIN32
extern "C" void on_trap_signal(int signo) { signal_handlers::trap(signo); }
#endif
} // namespace

std::string host_signals::take_trapped() {
#ifndef _WIN32
  return signal_handlers::take(*this);
#else
  return {}; // nothing is trapped on Windows
#endif
}

host_signals &signals_of(cvc::app &a) { return a.slot<host_signals>(); }

struct signal_route::saved {
  host_signals *previous = nullptr; // the route this one replaced
  int count = 0;
  int signo[kMaxRouted] = {};
#ifndef _WIN32
  struct sigaction action[kMaxRouted];
#else
  void (*action[kMaxRouted])(int) = {};
#endif
};

signal_route::signal_route(cvc::app &a, mode m) : saved_(std::make_unique<saved>()) {
  saved &s = *saved_;
  // The route first, then the handlers, so a signal that comes between finds this app.
  s.previous = g_raised.exchange(&signals_of(a));
#ifndef _WIN32
  struct sigaction sa {};
  sigemptyset(&sa.sa_mask);
  const auto take = [&](int signo) {
    if (sigaction(signo, &sa, &s.action[s.count]) == 0)
      s.signo[s.count++] = signo;
  };
  if (m == mode::stop) {
    sa.sa_handler = on_stop_signal;
    sa.sa_flags = SA_RESTART; // as std::signal has it on glibc: a blocking call carries on
    for (int signo : kStops)
      take(signo);
  } else {
    sa.sa_handler = on_trap_signal;
    for (const Trap &t : kTraps)
      take(t.signo);
  }
#else
  // Windows has SIGINT (Ctrl+C) and SIGTERM (raise) to stop on, and no signals to trap. The CRT
  // puts a handler back to SIG_DFL as it calls it, so a second Ctrl+C ends the process.
  if (m == mode::stop)
    for (int signo : kStops) {
      void (*previous)(int) = std::signal(signo, on_stop_signal);
      if (previous != SIG_ERR) {
        s.action[s.count] = previous;
        s.signo[s.count++] = signo;
      }
    }
#endif
}

signal_route::~signal_route() {
  saved &s = *saved_;
  // The handlers first, then the route, so a signal that comes between still finds an app: this
  // route's, which outlives it.
  for (int i = s.count; i-- > 0;) {
#ifndef _WIN32
    sigaction(s.signo[i], &s.action[i], nullptr);
#else
    std::signal(s.signo[i], s.action[i]);
#endif
  }
  g_raised.store(s.previous);
}

int signal_number(const std::string &name) {
#ifndef _WIN32
  for (const Trap &t : kTraps)
    if (name == t.name)
      return t.signo;
#else
  (void)name;
#endif
  return 0;
}

} // namespace ariadne
} // namespace cvc

#endif // !__wasi__
