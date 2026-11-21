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
#include <cmath>
#include <cvc/ariadne/controls.h>

namespace cvc {
namespace ariadne {

namespace {
constexpr double kInteractHold = 0.4; // s
constexpr double kSteerRamp = 0.25;   // s from straight to the lock
constexpr double kStickDead = 0.15;   // of a stick's travel: a resting stick drifts
constexpr double kTriggerDead = 0.05; // of a trigger's

control_action act(const char *id, const char *label, std::vector<std::string> keys,
                   std::vector<std::string> pad = {}) {
  control_action a;
  a.id = id;
  a.label = label;
  a.binding.keys = std::move(keys);
  a.binding.pad = std::move(pad);
  return a;
}

control_action dead(control_action a, double deadzone) {
  a.binding.deadzone = deadzone;
  return a;
}

// Looking around: the right stick only (the mouse looks on its own).
void look_actions(control_scheme &s) {
  s.actions.push_back(dead(act("look_left", "Look left", {}, {"RightX-"}), kStickDead));
  s.actions.push_back(dead(act("look_right", "Look right", {}, {"RightX+"}), kStickDead));
  s.actions.push_back(dead(act("look_up", "Look up", {}, {"RightY-"}), kStickDead));
  s.actions.push_back(dead(act("look_down", "Look down", {}, {"RightY+"}), kStickDead));
}

control_scheme drive_scheme(const char *id, bool brakes) {
  control_scheme s;
  s.id = id;
  s.kind = control_kind::drive;
  s.actions.push_back(dead(act("throttle", "Throttle", {"W", "Up"}, {"RT"}), kTriggerDead));
  // A boat's reverse is the left trigger, where a car's brake is; a car's is LB.
  s.actions.push_back(
      dead(act("reverse", "Reverse", {"S", "Down"}, {brakes ? "LB" : "LT"}), kTriggerDead));
  if (brakes)
    s.actions.push_back(dead(act("brake", "Brake", {"Space"}, {"LT"}), kTriggerDead));
  control_action left =
      dead(act("steer_left", "Steer left", {"A", "Left"}, {"LeftX-"}), kStickDead);
  control_action right =
      dead(act("steer_right", "Steer right", {"D", "Right"}, {"LeftX+"}), kStickDead);
  left.binding.ramp = right.binding.ramp = kSteerRamp;
  s.actions.push_back(left);
  s.actions.push_back(right);
  control_action out = act("interact", "Get out (hold)", {"E"}, {"X"});
  out.binding.hold = kInteractHold;
  s.actions.push_back(out);
  look_actions(s);
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
  walk.actions.push_back(dead(act("forward", "Walk forward", {"W", "Up"}, {"LeftY-"}), kStickDead));
  walk.actions.push_back(dead(act("backward", "Walk back", {"S", "Down"}, {"LeftY+"}), kStickDead));
  walk.actions.push_back(
      dead(act("strafe_left", "Step left", {"A", "Left"}, {"LeftX-"}), kStickDead));
  walk.actions.push_back(
      dead(act("strafe_right", "Step right", {"D", "Right"}, {"LeftX+"}), kStickDead));
  walk.actions.push_back(act("jump", "Jump", {"Space"}, {"A"}));
  walk.actions.push_back(act("mouse_look", "Mouse look on / off", {"V"}, {"RightStick"}));
  control_action in = act("interact", "Get in (hold)", {"E"}, {"X"});
  in.binding.hold = kInteractHold;
  walk.actions.push_back(in);
  look_actions(walk);
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

void control_resolver::set_pad(const std::string &action, std::vector<std::string> pad) {
  for (control_action &a : _scheme.actions)
    if (a.id == action)
      a.binding.pad = std::move(pad);
}

double pad_reading(const control_binding &binding, const control_resolver::pad_fn &pad) {
  if (!pad)
    return 0.0;
  double best = 0.0;
  for (const std::string &name : binding.pad) {
    if (name.empty())
      continue;
    // A stick's half by its sign; anything else as it reads.
    const char sign = name.back();
    const bool half = sign == '+' || sign == '-';
    double r = pad(half ? name.substr(0, name.size() - 1) : name);
    if (!std::isfinite(r))
      continue;
    if (binding.invert)
      r = -r;
    if (half)
      r = sign == '+' ? r : -r;
    best = std::max(best, std::clamp(r, 0.0, 1.0));
  }
  const double dz = std::clamp(binding.deadzone, 0.0, 0.99);
  if (best <= dz)
    return 0.0;
  return std::clamp((best - dz) / (1.0 - dz) * binding.scale, 0.0, 1.0);
}

void control_resolver::update(const held_fn &held, double dt, const pad_fn &pad) {
  dt = std::max(0.0, dt);
  bool any_key = false, any_pad = false;
  for (const control_action &a : _scheme.actions) {
    live &l = _live[a.id];
    bool keyed = false;
    for (const std::string &k : a.binding.keys)
      keyed = keyed || (held && held(k));
    const double padded = pad_reading(a.binding, pad);
    any_key = any_key || keyed;
    any_pad = any_pad || padded > 0.0;
    const bool down = keyed || padded > 0.5;
    l.was_down = l.down;
    l.down = down;
    // The keys' value ramps; the gamepad's is as it reads.
    const double target = keyed ? 1.0 : 0.0;
    const double ramp = a.binding.ramp;
    if (ramp > 0.0) {
      const double step = dt / ramp;
      l.key_value = l.key_value < target ? std::min(target, l.key_value + step)
                                         : std::max(target, l.key_value - step);
    } else {
      l.key_value = target;
    }
    l.value = std::max(l.key_value, padded);
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
  if (any_key)
    _from_pad = false;
  else if (any_pad)
    _from_pad = true;
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

void control_resolver::walk_axes(double &forward, double &right) const {
  forward = value("forward") - value("backward");
  right = value("strafe_right") - value("strafe_left");
}

void control_resolver::look_axes(double &right, double &up) const {
  right = value("look_right") - value("look_left");
  up = value("look_up") - value("look_down");
}

} // namespace ariadne
} // namespace cvc
