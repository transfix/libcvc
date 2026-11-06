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

#ifndef __CVC_ARIADNE_CONFERENCE_AVATAR_PANEL_H__
#define __CVC_ARIADNE_CONFERENCE_AVATAR_PANEL_H__

#include <cstddef>
#include <cstdint>
#include <cvc/ariadne/avatar_physics.h>
#include <cvc/ariadne/conference/room_avatar.h> // avatar_catalog, avatar_choice
#include <deque>
#include <string>

namespace cvc {
class app;
namespace ariadne {
class holon;
namespace conference {

// ----------------
// The Avatar window's model (conference_roster.ari; ARIADNE_CONFERENCE_ROADMAP.md §5k)
// ----------------
// What you look like, what you move with and where your energy went, for one participant: demo3's
// ledger screen, for an avatar. update() reads the CLIENT holon (its prediction of our avatar,
// which the relay's published reserve keeps honest) and the room tree; publish() writes text and
// numbers for the document under `<prefix>.self.avatar.`, so a host adds two calls and every host
// -- a window, the terminal, a test -- shows the same thing.
//
//   look.{color, accessory}            the labels everyone draws (room_avatar.h)
//   body, controller                   "0.50 x 0.50 x 1.70 m box, 70 kg", "walker"
//   policy.{budget, push, power, reserve, carrying, empty, sustains}   the relay's rule, each a
//                                      short value ("840 N"), with what it means as
//                                      policy.<line>_tip (for a tooltip)
//   energy.{fraction, text, power, speed, state, line}   the reserve now (fraction 0..1 for a
//                                      bar; line: "spending, 549 W at 4.00 m/s")
//   ledger.{start, refilled, work, carried, overflow, unpaid, corrections, now, residual,
//           distance, time, exhausted, peak, per_km}            the books, as text
//   ledger.{balance, summary}          "balances (residual 0.000 J)"; "412 m in 2 min 05 s, ..."
//   ledger.share.{work, carried, overflow}   each spending line's share of all spent (0..1)
//   history.{reserve, power, window}   the last samples (kJ, W), comma-separated, for plot:
//   ready                              "1" once our avatar has a body to account for
class avatar_panel {
public:
  // History: a sample every PERIOD seconds of wall time, the last CAPACITY kept (60 s by default).
  explicit avatar_panel(double period = 0.25, std::size_t capacity = 240);

  // Once a frame, after client_pump: H is the client holon and PID our avatar in ROOM. The room's
  // catalog is read again only when its revision moves (or ROOM changes); our choice, two leaves,
  // is checked every frame.
  void update(const holon &h, cvc::app &app, const std::string &room, const std::string &pid,
              double wall_dt);

  // Write the fields above under PREFIX (the document's state root, e.g. "ui.docs.conf").
  void publish(cvc::app &app, const std::string &prefix) const;

  // Whether the last update took a history sample (a period has passed): when a host publishes,
  // so the document's fields refresh at the history's rate rather than every frame.
  bool sampled() const { return _sampled; }

  // What the last update saw (tests).
  bool ready() const { return _ready; }
  const avatar_energy &energy() const { return _energy; }
  const std::deque<double> &reserve_history() const { return _reserve; }
  const std::deque<double> &power_history() const { return _power; }
  // How many updates read the room's catalog: the first, then only those after its revision or
  // the room changed (tests).
  std::size_t catalog_reads() const { return _catalog_reads; }

private:
  // Re-resolve the look when the catalog or our choice changed since the last update.
  void refresh_look(cvc::app &app, const std::string &room, const std::string &pid);

  double _period;
  std::size_t _capacity;
  double _since = 0.0; // wall seconds since the last history sample
  bool _ready = false;
  bool _sampled = false;
  avatar_energy _energy;
  avatar_energy_policy _policy;
  avatar_body _body;
  double _speed = 0.0;
  std::string _color, _accessory;
  std::deque<double> _reserve, _power;
  // What the look was resolved from: the catalog as of revision _catalog_rev in _look_room, and
  // _look_pid's choice.
  bool _have_catalog = false;
  std::uint64_t _catalog_rev = 0;
  std::size_t _catalog_reads = 0;
  std::string _look_room, _look_pid;
  avatar_catalog _catalog;
  avatar_choice _choice;
};

} // namespace conference
} // namespace ariadne
} // namespace cvc

#endif // __CVC_ARIADNE_CONFERENCE_AVATAR_PANEL_H__
