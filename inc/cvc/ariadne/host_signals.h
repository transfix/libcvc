// OS signals for a runtime host (ari, arigl, arisrv), to hand to a document's on_signal:.
//
// install_signal_traps() replaces the handlers for SIGTERM, SIGHUP, SIGINT, SIGQUIT, SIGUSR1 and
// SIGUSR2 with one that only sets a flag (all a handler may safely do). The host then polls
// take_trapped_signal() once per frame and passes each name to AppRuntime::post_signal (or gives
// it to FtxuiBackend::RunOptions::take_signal, which does the same); a signal the document
// declares no handler for is the host's to act on, conventionally by exiting with 128 +
// signal_number(name). restore_signal_traps() puts the previous handlers back.
//
// Signal dispositions are process-wide, so nothing in libcvc installs these by itself: only a
// host program opts in, around its own loop. On Windows the traps are no-ops and nothing is
// ever trapped.
#pragma once

#include <string>

namespace cvc {
namespace ariadne {

void install_signal_traps();
void restore_signal_traps();

// The name of one signal that arrived since the last call ("SIGTERM", ...), or "" when none did.
std::string take_trapped_signal();

// The number for a trapped signal's name; 0 for any other name.
int signal_number(const std::string &name);

} // namespace ariadne
} // namespace cvc
