/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <algorithm>
#include <boost/lexical_cast.hpp>
#include <cctype>
#include <climits>
#include <cvc/core/app.h>
#include <cvc/core/world_clock_binding.h>
#include <cvc/state/state.h>

namespace cvc {

namespace {

std::string trim_lower(const std::string &s) {
  std::string t;
  for (char c : s)
    if (!std::isspace(static_cast<unsigned char>(c)))
      t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return t;
}

// Read through a transparent link to its target, the way a bound widget reads
// (cvc/ariadne/bind.h read_effective): a mounted scope's hole is a transparent
// link, and a control written through it lands on the target.
std::string read_raw(cvc::app &ctx, const std::string &path) {
  try {
    cvc::state &s = cvc::state::instance(ctx)(path);
    if (s.isLink() && s.linkMode() == cvc::state::link_mode::transparent) {
      const cvc::state::link_resolution lr = s.resolveLink();
      if (lr.kind == cvc::state::link_resolution_kind::resolved && lr.target)
        return lr.target->value();
    }
    return s.value();
  } catch (const std::exception &) {
    return std::string();
  }
}

// Never throws into a frame: a read-only or unwritable key keeps its value.
template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

void write_str(cvc::app &ctx, const std::string &path, const std::string &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

template <class T> bool parse_num(const std::string &s, T &out) {
  const std::string t = trim_lower(s);
  if (t.empty())
    return false;
  try {
    out = boost::lexical_cast<T>(t);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

// cvc::state stores a bool as "1"/"0", and state_exec renders one as
// "true"/"false"; accept both, as Ariadne's bool widgets do.
bool parse_bool(const std::string &s, bool &out) {
  const std::string t = trim_lower(s);
  if (t == "1" || t == "true" || t == "yes" || t == "on") {
    out = true;
    return true;
  }
  if (t == "0" || t == "false" || t == "no" || t == "off") {
    out = false;
    return true;
  }
  double d = 0.0;
  if (parse_num(t, d)) {
    out = d != 0.0;
    return true;
  }
  return false;
}

} // namespace

world_clock_binding::world_clock_binding(cvc::app &ctx, cvc::world_clock &clock,
                                         std::string prefix)
    : _ctx(ctx), _clock(clock), _prefix(std::move(prefix)) {}

std::string world_clock_binding::key(const std::string &name) const {
  return _prefix.empty() ? name : _prefix + "." + name;
}

const char *world_clock_binding::mode_name(cvc::world_clock::mode m) {
  switch (m) {
  case cvc::world_clock::mode::live:
    return "live";
  case cvc::world_clock::mode::replay:
    return "replay";
  case cvc::world_clock::mode::paused:
    return "paused";
  case cvc::world_clock::mode::stepping:
    return "stepping";
  case cvc::world_clock::mode::lockstep:
    return "lockstep";
  }
  return "live";
}

bool world_clock_binding::parse_mode(const std::string &name, cvc::world_clock::mode &out) {
  const std::string t = trim_lower(name);
  if (t == "live")
    out = cvc::world_clock::mode::live;
  else if (t == "replay")
    out = cvc::world_clock::mode::replay;
  else if (t == "paused")
    out = cvc::world_clock::mode::paused;
  else if (t == "stepping")
    out = cvc::world_clock::mode::stepping;
  else if (t == "lockstep")
    out = cvc::world_clock::mode::lockstep;
  else
    return false;
  return true;
}

void world_clock_binding::seed() {
  _seeded = true;
  const cvc::world_clock::mode m = _clock.current_mode();
  // The binding keeps pause apart from mode, so a clock that is paused seeds
  // paused = 1 and the mode it would resume in (live).
  _paused = (m == cvc::world_clock::mode::paused);
  _requested = _paused ? cvc::world_clock::mode::live : m;

  if (read_raw(_ctx, key("speed")).empty())
    write_val(_ctx, key("speed"), _clock.scale());
  if (read_raw(_ctx, key("paused")).empty())
    write_val(_ctx, key("paused"), _paused ? 1 : 0);
  if (read_raw(_ctx, key("mode")).empty())
    write_str(_ctx, key("mode"), mode_name(_requested));
  if (read_raw(_ctx, key("lockstep_steps")).empty())
    write_val(_ctx, key("lockstep_steps"), _clock.lockstep_steps());
  if (read_raw(_ctx, key("step")).empty())
    write_val(_ctx, key("step"), 0);

  // Requests made before the binding existed are not replayed as a burst.
  long long seen = 0;
  if (parse_num(read_raw(_ctx, key("step")), seen))
    _step_seen = seen;
}

void world_clock_binding::pull() {
  if (!_seeded)
    seed();

  double speed = 0.0;
  if (parse_num(read_raw(_ctx, key("speed")), speed))
    _clock.set_scale(speed); // the clock clamps a negative or non-finite speed to 0

  bool paused = false;
  if (parse_bool(read_raw(_ctx, key("paused")), paused))
    _paused = paused;

  cvc::world_clock::mode requested;
  if (parse_mode(read_raw(_ctx, key("mode")), requested))
    _requested = requested;

  int lockstep = 0;
  if (parse_num(read_raw(_ctx, key("lockstep_steps")), lockstep))
    _clock.set_lockstep_steps(lockstep);

  long long step = 0;
  if (parse_num(read_raw(_ctx, key("step")), step)) {
    if (step > _step_seen)
      _pending_steps += static_cast<std::uint64_t>(step - _step_seen);
    _step_seen = step; // a counter reset (step went down) requests nothing
  }

  cvc::world_clock::mode eff = _requested;
  if (_paused && (eff == cvc::world_clock::mode::live || eff == cvc::world_clock::mode::lockstep))
    eff = cvc::world_clock::mode::paused;
  if (_clock.current_mode() != eff)
    _clock.set_mode(eff);
}

void world_clock_binding::push() {
  write_val(_ctx, key("time"), _clock.t());
  write_val(_ctx, key("tick"), _clock.tick());
  write_val(_ctx, key("fixed_dt"), _clock.fixed_dt());
  write_val(_ctx, key("realized_speed"), _clock.realized_scale());
  write_val(_ctx, key("dropped"), _clock.total_dropped());
}

cvc::world_clock::step_result world_clock_binding::drive(double wall_dt) {
  pull();
  cvc::world_clock::step_result r = _clock.advance(wall_dt);
  if (_pending_steps > 0) {
    const int n = static_cast<int>(std::min<std::uint64_t>(_pending_steps, INT_MAX));
    const cvc::world_clock::step_result s = _clock.advance_ticks(n);
    _pending_steps -= static_cast<std::uint64_t>(n);
    // One driver, so the requested steps directly follow the advanced ones.
    if (r.steps == 0)
      r.first_tick = s.first_tick;
    r.steps += s.steps;
    r.alpha = s.alpha;
  }
  push();
  return r;
}

} // namespace cvc
