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

#ifndef CVC_ARIADNE_CONTROLS_H
#define CVC_ARIADNE_CONTROLS_H

#include <cvc/ariadne/avatar_sim.h> // avatar_intent
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace cvc {
namespace ariadne {

// ----------------
// Control schemes (ARIADNE_WORLDS.md section 18.6)
// ----------------
// How a participant's inputs drive what it controls. Each kind of thing driven has a scheme of
// its own, and each scheme its own bindings, so a car's keys are not a person's: a person walks
// (WASD forward, back and strafe, facing where the mouse aims), a car or a boat drives (throttle,
// reverse and steering: neither can strafe). Getting in switches the participant to its
// vehicle's scheme, getting out back to walking.
//
//   walk   forward, backward, strafe_left, strafe_right, jump, mouse_look (toggles it), interact
//   car    throttle, reverse, brake, steer_left, steer_right, interact
//   boat   the car's, with no brake (its reverse is the propeller astern)
//
// interact gets in (the nearest drivable vehicle within its reach) and out; it fires once held
// for its `hold` (0.4 s), and letting go before then cancels it.
//
// A binding names keys and mouse buttons as the Controls window stores them (keys.h: SDL's names,
// "Mouse1".."Mouse5" for the buttons) and, for a gamepad, its SDL standard names (pad: A, B, X, Y,
// LB, RB, LT, RT, LeftX, ...; read from OW6b on). A host keeps each scheme's bindings in its state
// so the Controls window can change them (arigl: <viewer>.avatar.keys.<action> for walk, as before,
// and <viewer>.controls.<scheme>.<action> for the others).

enum class control_kind { walk, drive };

struct control_binding {
  std::vector<std::string> keys; // keys and mouse buttons; any held holds it
  std::vector<std::string> pad;  // gamepad buttons and axes (OW6b)
  // An analog input below `deadzone` is none; above it, scaled to [0, 1] and by `scale`, and
  // turned over by `invert`. A digital one is 0 or 1.
  double deadzone = 0.0, scale = 1.0;
  bool invert = false;
  // Seconds from 0 to full while held, and back to 0 when let go (0: at once): a key steers
  // smoothly instead of yanking the wheel to its lock.
  double ramp = 0.0;
  // Seconds held before it fires (interact); 0 is a plain hold.
  double hold = 0.0;
  bool operator==(const control_binding &) const = default;
};

struct control_action {
  std::string id, label;
  control_binding binding;
  bool operator==(const control_action &) const = default;
};

struct control_scheme {
  std::string id;
  control_kind kind = control_kind::walk;
  std::vector<control_action> actions; // in the order the Controls window lists them
  const control_action *find(const std::string &action) const;
  bool operator==(const control_scheme &) const = default;
};

// The built-in schemes: walk, car, boat (above), with their default bindings.
std::vector<control_scheme> default_control_schemes();
// One of them by id; one with an empty id for another.
control_scheme default_control_scheme(const std::string &id);

// A scheme's bindings, as they are being used: every frame, update() with what is held and the
// frame's time; then each action's value, and whether a hold action fired.
class control_resolver {
public:
  // Which inputs are held: a key's or mouse button's stored name (keys.h) in, held out.
  using held_fn = std::function<bool(const std::string &)>;

  control_resolver() = default;
  explicit control_resolver(control_scheme scheme) { set_scheme(std::move(scheme)); }

  // A new scheme (getting in or out): every action starts let go, and an interact still held from
  // the last scheme waits to be let go before it can fire again (the E that got you in does not
  // get you straight back out).
  void set_scheme(control_scheme scheme);
  const control_scheme &scheme() const { return _scheme; }
  // A binding's keys from now on (the Controls window changed them).
  void set_keys(const std::string &action, std::vector<std::string> keys);

  void update(const held_fn &held, double dt);

  // An action's value in [0, 1] after its ramp; 0 for an action the scheme has not.
  double value(const std::string &action) const;
  bool held(const std::string &action) const { return value(action) > 0.5; }
  // A hold action: true on the update its hold elapsed (once per press), and how far along the
  // hold is now, in [0, 1] (what a prompt's ring shows; 0 when let go or after it fired).
  bool fired(const std::string &action) const;
  double progress(const std::string &action) const;
  // A toggle: true on the update an action was first pressed.
  bool pressed(const std::string &action) const;

  // A drive scheme's actions as the intent's drive axes (shares, avatar_intent): steer =
  // steer_left - steer_right, throttle = throttle - reverse, brake.
  void drive_axes(avatar_intent &in) const;

private:
  struct live {
    bool down = false, was_down = false;
    double value = 0.0;
    double held_s = 0.0;
    bool fired = false, spent = false; // spent: fired this press (wait for a let go)
  };
  control_scheme _scheme;
  std::map<std::string, live> _live;
};

} // namespace ariadne
} // namespace cvc

#endif // CVC_ARIADNE_CONTROLS_H
