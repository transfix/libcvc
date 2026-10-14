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
#include <utility>

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
  if (path.empty()) // a lane that is off reads as unset
    return std::string();
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

// Never throws into a frame: a read-only or unwritable key keeps its value, and
// a lane that is off (empty path) is not written.
template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  if (path.empty())
    return;
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

void write_str(cvc::app &ctx, const std::string &path, const std::string &v) {
  if (path.empty())
    return;
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

world_clock_keys world_clock_keys::under(const std::string &prefix) {
  const auto at = [&prefix](const char *name) {
    return prefix.empty() ? std::string(name) : prefix + "." + name;
  };
  world_clock_keys k;
  k.speed = at("speed");
  k.paused = at("paused");
  k.mode = at("mode");
  k.lockstep_steps = at("lockstep_steps");
  k.step = at("step");
  k.time = at("time");
  k.tick = at("tick");
  k.fixed_dt = at("fixed_dt");
  k.realized_speed = at("realized_speed");
  k.dropped = at("dropped");
  return k;
}

world_clock_binding::world_clock_binding(cvc::app &ctx, cvc::world_clock &clock, std::string prefix)
    : _ctx(ctx), _clock(clock), _prefix(std::move(prefix)),
      _keys(world_clock_keys::under(_prefix)) {}

world_clock_binding::world_clock_binding(cvc::app &ctx, cvc::world_clock &clock,
                                         world_clock_keys keys)
    : _ctx(ctx), _clock(clock), _keys(std::move(keys)) {}

std::string world_clock_binding::key(const std::string &name) const {
  const std::pair<const char *, const std::string *> table[] = {
      {"speed", &_keys.speed},
      {"paused", &_keys.paused},
      {"mode", &_keys.mode},
      {"lockstep_steps", &_keys.lockstep_steps},
      {"step", &_keys.step},
      {"time", &_keys.time},
      {"tick", &_keys.tick},
      {"fixed_dt", &_keys.fixed_dt},
      {"realized_speed", &_keys.realized_speed},
      {"dropped", &_keys.dropped},
  };
  for (const auto &e : table)
    if (name == e.first)
      return *e.second;
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

  if (read_raw(_ctx, _keys.speed).empty())
    write_val(_ctx, _keys.speed, _clock.scale());
  if (read_raw(_ctx, _keys.paused).empty())
    write_val(_ctx, _keys.paused, _paused ? 1 : 0);
  if (read_raw(_ctx, _keys.mode).empty())
    write_str(_ctx, _keys.mode, mode_name(_requested));
  if (read_raw(_ctx, _keys.lockstep_steps).empty())
    write_val(_ctx, _keys.lockstep_steps, _clock.lockstep_steps());
  if (read_raw(_ctx, _keys.step).empty())
    write_val(_ctx, _keys.step, 0);

  // Requests made before the binding existed are not replayed as a burst.
  long long seen = 0;
  if (parse_num(read_raw(_ctx, _keys.step), seen))
    _step_seen = seen;
}

void world_clock_binding::pull() {
  if (!_seeded)
    seed();

  double speed = 0.0;
  if (parse_num(read_raw(_ctx, _keys.speed), speed))
    _clock.set_scale(speed); // the clock clamps a negative or non-finite speed to 0

  bool paused = false;
  if (parse_bool(read_raw(_ctx, _keys.paused), paused))
    _paused = paused;

  cvc::world_clock::mode requested = _requested;
  if (parse_mode(read_raw(_ctx, _keys.mode), requested))
    _requested = requested;

  int lockstep = 0;
  if (parse_num(read_raw(_ctx, _keys.lockstep_steps), lockstep))
    _clock.set_lockstep_steps(lockstep);

  long long step = 0;
  if (parse_num(read_raw(_ctx, _keys.step), step)) {
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
  write_val(_ctx, _keys.time, _clock.t());
  write_val(_ctx, _keys.tick, _clock.tick());
  write_val(_ctx, _keys.fixed_dt, _clock.fixed_dt());
  write_val(_ctx, _keys.realized_speed, _clock.realized_scale());
  write_val(_ctx, _keys.dropped, _clock.total_dropped());
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
