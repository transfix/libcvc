// OS-signal traps for runtime hosts — see inc/cvc/ariadne/host_signals.h.

#include <cvc/ariadne/host_signals.h>

#ifndef _WIN32
#include <csignal>
#endif

namespace cvc {
namespace ariadne {

#ifndef _WIN32
namespace {

// A signal handler may only touch lock-free, signal-safe state, so these flags are file-scope by
// necessity: they are the process's signal dispositions, not application state.
struct Trap {
  int signo;
  const char *name;
};
constexpr Trap kTraps[] = {{SIGTERM, "SIGTERM"}, {SIGHUP, "SIGHUP"},   {SIGINT, "SIGINT"},
                           {SIGQUIT, "SIGQUIT"}, {SIGUSR1, "SIGUSR1"}, {SIGUSR2, "SIGUSR2"}};
constexpr int kTrapCount = static_cast<int>(sizeof(kTraps) / sizeof(kTraps[0]));
volatile std::sig_atomic_t g_raised[kTrapCount] = {};
struct sigaction g_previous[kTrapCount];

extern "C" void on_os_signal(int signo) {
  for (int i = 0; i < kTrapCount; ++i)
    if (kTraps[i].signo == signo)
      g_raised[i] = 1;
}

} // namespace

void install_signal_traps() {
  struct sigaction sa {};
  sa.sa_handler = on_os_signal;
  sigemptyset(&sa.sa_mask);
  for (int i = 0; i < kTrapCount; ++i)
    sigaction(kTraps[i].signo, &sa, &g_previous[i]);
}

void restore_signal_traps() {
  for (int i = 0; i < kTrapCount; ++i)
    sigaction(kTraps[i].signo, &g_previous[i], nullptr);
}

std::string take_trapped_signal() {
  for (int i = 0; i < kTrapCount; ++i)
    if (g_raised[i]) {
      g_raised[i] = 0;
      return kTraps[i].name;
    }
  return {};
}

int signal_number(const std::string &name) {
  for (const Trap &t : kTraps)
    if (name == t.name)
      return t.signo;
  return 0;
}

#else // _WIN32: no POSIX signals to trap

void install_signal_traps() {}
void restore_signal_traps() {}
std::string take_trapped_signal() { return {}; }
int signal_number(const std::string &) { return 0; }

#endif

} // namespace ariadne
} // namespace cvc
