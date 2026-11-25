/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// shown_clock.h — the time a scene is drawn at (ARIADNE_WORLDS.md §18.5, OW5q).
//
// What a drawn scene shows that moves with time (the water's waves, the storms, the clouds, the
// trees' and the foliage's sway) is drawn at the app's world clock's time, unless a host showing
// another host's world says otherwise: a client of a relay draws the relay's world at the relay's
// time (as its holon counts the server's ticks), so the waves it draws are the ones its boat's
// model floats on and the storms the ones the relay's air blows from. That host writes the time
// (seconds, 17 digits) to `shown_clock` under the scene's state prefix each frame
// (show_relay_seas); every such node reads it here, and seeds it empty so a host sees the scene
// draws by it. Empty or unset: the app's.

#ifndef __CVC_ARIADNE_SHOWN_CLOCK_H__
#define __CVC_ARIADNE_SHOWN_CLOCK_H__

#include <cmath>
#include <cstdlib>
#include <cvc/ariadne/bind.h>
#include <string>

namespace cvc {
class app;
namespace ariadne {

// The key, relative to a scene's state prefix.
inline constexpr const char *kShownClockKey = "shown_clock";

// The time to draw at: `shown_clock` under PREFIX when a host set it, else APP_T (the app's world
// clock's time).
inline double shown_time(cvc::app &app, const std::string &prefix, double app_t) {
  const std::string text =
      read_or<std::string>(app, resolve_bind(prefix, kShownClockKey), std::string());
  if (text.empty())
    return app_t;
  char *end = nullptr;
  const double t = std::strtod(text.c_str(), &end);
  return end != text.c_str() && std::isfinite(t) ? t : app_t;
}

// Seed it empty under PREFIX: the scene draws by it (its own time until a host writes one).
inline void seed_shown_clock(cvc::app &app, const std::string &prefix) {
  read_or_seed<std::string>(app, resolve_bind(prefix, kShownClockKey), std::string());
}

} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_SHOWN_CLOCK_H__
