/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
*/

#include <algorithm>
#include <cvc/ariadne/controls.h>

namespace cvc {
namespace ariadne {

namespace {
constexpr double kInteractHold = 0.4; // s
constexpr double kSteerRamp = 0.25;   // s from straight to the lock

control_action act(const char *id, const char *label, std::vector<std::string> keys,
                   std::vector<std::string> pad = {}) {
  control_action a;
  a.id = id;
  a.label = label;
  a.binding.keys = std::move(keys);
  a.binding.pad = std::move(pad);
  return a;
}

control_scheme drive_scheme(const char *id, bool brakes) {
  control_scheme s;
  s.id = id;
  s.kind = control_kind::drive;
  s.actions.push_back(act("throttle", "Throttle", {"W", "Up"}));
  s.actions.push_back(act("reverse", "Reverse", {"S", "Down"}));
  if (brakes)
    s.actions.push_back(act("brake", "Brake", {"Space"}));
  control_action left = act("steer_left", "Steer left", {"A", "Left"});
  control_action right = act("steer_right", "Steer right", {"D", "Right"});
  left.binding.ramp = right.binding.ramp = kSteerRamp;
  s.actions.push_back(left);
  s.actions.push_back(right);
  control_action out = act("interact", "Get out (hold)", {"E"});
  out.binding.hold = kInteractHold;
  s.actions.push_back(out);
  return s;
}
} // namespace

const control_action *control_scheme::find(const std::string &action) const {
  for (const control_action &a : actions)
    if (a.id == action)
      return &a;
  return nullptr;
}

std::vector<control_scheme> default_control_schemes() {
  control_scheme walk;
  walk.id = "walk";
  walk.kind = control_kind::walk;
  walk.actions.push_back(act("forward", "Walk forward", {"W", "Up"}));
  walk.actions.push_back(act("backward", "Walk back", {"S", "Down"}));
  walk.actions.push_back(act("strafe_left", "Step left", {"A", "Left"}));
  walk.actions.push_back(act("strafe_right", "Step right", {"D", "Right"}));
  walk.actions.push_back(act("jump", "Jump", {"Space"}));
  walk.actions.push_back(act("mouse_look", "Mouse look on / off", {"V"}));
  control_action in = act("interact", "Get in (hold)", {"E"});
  in.binding.hold = kInteractHold;
  walk.actions.push_back(in);
  return {walk, drive_scheme("car", true), drive_scheme("boat", false)};
}

control_scheme default_control_scheme(const std::string &id) {
  for (control_scheme &s : default_control_schemes())
    if (s.id == id)
      return std::move(s);
  return control_scheme{"", control_kind::walk, {}};
}

void control_resolver::set_scheme(control_scheme scheme) {
  std::map<std::string, live> next;
  for (const control_action &a : scheme.actions) {
    live l;
    // Still held from the scheme before: it must be let go first.
    const auto it = _live.find(a.id);
    if (it != _live.end() && it->second.down) {
      l.down = l.was_down = true;
      l.spent = true;
    }
    next[a.id] = l;
  }
  _live = std::move(next);
  _scheme = std::move(scheme);
}

void control_resolver::set_keys(const std::string &action, std::vector<std::string> keys) {
  for (control_action &a : _scheme.actions)
    if (a.id == action)
      a.binding.keys = std::move(keys);
}

void control_resolver::update(const held_fn &held, double dt) {
  dt = std::max(0.0, dt);
  for (const control_action &a : _scheme.actions) {
    live &l = _live[a.id];
    bool down = false;
    for (const std::string &k : a.binding.keys)
      down = down || (held && held(k));
    l.was_down = l.down;
    l.down = down;
    const double target = down ? 1.0 : 0.0;
    const double ramp = a.binding.ramp;
    if (ramp > 0.0) {
      const double step = dt / ramp;
      l.value = l.value < target ? std::min(target, l.value + step)
                                 : std::max(target, l.value - step);
    } else {
      l.value = target;
    }
    l.fired = false;
    if (!down)
      l.spent = false; // let go: it may fire (or toggle) again
    if (a.binding.hold > 0.0) {
      if (!down) {
        l.held_s = 0.0;
      } else if (!l.spent) {
        l.held_s += dt;
        if (l.held_s >= a.binding.hold) {
          l.fired = true;
          l.spent = true;
          l.held_s = 0.0;
        }
      }
    }
  }
}

double control_resolver::value(const std::string &action) const {
  const auto it = _live.find(action);
  return it == _live.end() ? 0.0 : it->second.value;
}

bool control_resolver::fired(const std::string &action) const {
  const auto it = _live.find(action);
  return it != _live.end() && it->second.fired;
}

double control_resolver::progress(const std::string &action) const {
  const auto it = _live.find(action);
  const control_action *a = _scheme.find(action);
  if (it == _live.end() || !a || !(a->binding.hold > 0.0) || it->second.spent)
    return 0.0;
  return std::clamp(it->second.held_s / a->binding.hold, 0.0, 1.0);
}

bool control_resolver::pressed(const std::string &action) const {
  const auto it = _live.find(action);
  return it != _live.end() && it->second.down && !it->second.was_down && !it->second.spent;
}

void control_resolver::drive_axes(avatar_intent &in) const {
  in.steer = value("steer_left") - value("steer_right");
  in.throttle = value("throttle") - value("reverse");
  in.brake = value("brake");
}

} // namespace ariadne
} // namespace cvc
