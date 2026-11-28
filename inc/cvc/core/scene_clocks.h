#ifndef CVC_CORE_SCENE_CLOCKS_H
#define CVC_CORE_SCENE_CLOCKS_H

// The clocks of the scenes that run on clocks of their own (docs/roadmap/ARIADNE_SHELL.md 4.4,
// SH3). A document mounted in the Ariadne shell steps its world at its own rate, on a world clock
// of its own, kept with the app under the state prefix the document is mounted at. A scene's
// nodes find their clock by the bind prefix they were realized under: scene_clock(app, prefix) is
// the clock of the scene at `prefix` or at the nearest prefix above it (a.b is under a), and
// app.world_clock() when there is none. So code that reads its scene's clock through
// scene_clock(app, its prefix) reads the mounted scene's in the shell and the app's anywhere else.
//
// The clocks are the app's (cvc::app::slot), never a process global: two apps never share one.

#include <string>

namespace cvc {
class app;
class world_clock;

// Make the clock at `prefix` with the quantum `fixed_dt` seconds (or return the one already
// there, unchanged). A clock stays valid until remove_scene_clock drops it or the app goes. An
// empty prefix names the app's own clock and is refused (std::invalid_argument).
world_clock &add_scene_clock(app &a, const std::string &prefix, double fixed_dt);

// Drop the clock at `prefix` (not one under it). Nothing when there is none.
void remove_scene_clock(app &a, const std::string &prefix);

// The clock of the scene at `prefix` or the nearest prefix above it, else a.world_clock().
world_clock &scene_clock(app &a, const std::string &prefix);

// Whether a clock was made at exactly `prefix`.
bool has_scene_clock(app &a, const std::string &prefix);

} // namespace cvc

#endif // CVC_CORE_SCENE_CLOCKS_H
