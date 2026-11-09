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
#include <cstdio>
#include <cvc/ariadne/conference/avatar_panel.h>
#include <cvc/ariadne/conference/room_avatar.h>
#include <cvc/ariadne/holon.h>
#include <cvc/core/app.h>
#include <cvc/state/state.h>
#include <string>
#include <utility>

namespace cvc {
namespace ariadne {
namespace conference {

namespace {

std::string printf_str(const char *fmt, double a, double b = 0.0, double c = 0.0) {
  char buf[96];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c);
  return std::string(buf);
}

// Joules as "12.3 kJ" (or "850 J" under one kJ), signed when SIGNED.
std::string joules(double j, bool signed_ = false) {
  const double a = std::fabs(j);
  const char *sign = !signed_ ? (j < 0.0 ? "-" : "") : (j < 0.0 ? "-" : "+");
  if (a < 999.5)
    return sign + printf_str("%.0f J", a);
  return sign + printf_str("%.1f kJ", a / 1000.0);
}

// Watts as "549 W" or "1.20 kW".
std::string watts(double w) {
  return std::fabs(w) < 999.5 ? printf_str("%.0f W", w) : printf_str("%.2f kW", w / 1000.0);
}

// Seconds as "42 s", "3 min 05 s" or "1 h 02 min".
std::string duration(double s) {
  const long t = static_cast<long>(s + 0.5);
  if (t < 60)
    return std::to_string(t) + " s";
  if (t < 3600)
    return printf_str("%.0f min %02.0f s", static_cast<double>(t / 60),
                      static_cast<double>(t % 60));
  return printf_str("%.0f h %02.0f min", static_cast<double>(t / 3600),
                    static_cast<double>((t % 3600) / 60));
}

std::string series(const std::deque<double> &v, const char *fmt) {
  std::string s;
  char buf[32];
  for (double x : v) {
    std::snprintf(buf, sizeof(buf), fmt, x);
    if (!s.empty())
      s += ',';
    s += buf;
  }
  return s;
}

} // namespace

avatar_panel::avatar_panel(double period, std::size_t capacity)
    : _period(period > 0.0 ? period : 0.25), _capacity(capacity > 0 ? capacity : 1) {}

void avatar_panel::update(const holon &h, cvc::app &app, const std::string &room,
                          const std::string &pid, double wall_dt) {
  _policy = h.energy_policy();
  _body = h.body();
  _controller = h.controller().id;
  const cvc::physics::vec3 g = h.rules().physics.gravity;
  _gravity = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
  refresh_look(app, room, pid);

  _sampled = false;
  _ready = h.energy(pid, _energy);
  if (!_ready) {
    _since = 0.0, _sampled = true; // nothing to plot yet; the settings still change
    return;
  }
  const cvc::physics::vec3 v = h.state(pid).linear_velocity;
  _speed = std::sqrt(v.x * v.x + v.y * v.y);
  // The history: a sample every period of wall time, the oldest dropped past capacity.
  _since += wall_dt > 0.0 ? wall_dt : 0.0;
  if (_reserve.empty() || _since >= _period) {
    _since = 0.0;
    _sampled = true;
    _reserve.push_back(_energy.reserve / 1000.0);
    _power.push_back(_energy.power);
    while (_reserve.size() > _capacity)
      _reserve.pop_front();
    while (_power.size() > _capacity)
      _power.pop_front();
  }
}

// The look everyone draws: our choice through the room's catalog. Reading the catalog walks every
// option's nodes (a few hundred), which every frame was most of a browser client's room step, so
// it is read again only when its revision moves: write_avatar_catalog bumps that last, so a new
// revision means the options it covers are here. The look is resolved again only when the catalog
// or our choice changed.
void avatar_panel::refresh_look(cvc::app &app, const std::string &room, const std::string &pid) {
  const std::uint64_t rev = avatar_catalog_revision(app, room);
  const bool reread = !_have_catalog || rev != _catalog_rev || room != _look_room;
  if (reread) {
    _catalog = read_avatar_catalog(app, room);
    _have_catalog = true;
    _catalog_rev = rev;
    ++_catalog_reads;
  }
  avatar_choice choice = read_avatar_choice(app, room, pid);
  if (!reread && pid == _look_pid && choice.color == _choice.color &&
      choice.accessory == _choice.accessory)
    return;
  _look_room = room, _look_pid = pid, _choice = std::move(choice);
  const avatar_look look = resolve_avatar_look(_catalog, _choice, pid);
  const avatar_color_option *c = _catalog.find_color(look.color_id);
  const avatar_accessory_option *a = _catalog.find_accessory(look.accessory_id);
  _color = c ? c->label : std::string();
  _accessory = a ? a->label : std::string();
}

void avatar_panel::publish(cvc::app &app, const std::string &prefix) const {
  auto &st = cvc::state::instance(app);
  const std::string base = prefix + ".self.avatar.";
  const auto put = [&](const std::string &key, const std::string &val) {
    st(base + key).value(val);
  };
  put("look.color", _color);
  put("look.accessory", _accessory);
  put("body", printf_str("%.2f x %.2f x %.2f m box, ", _body.width, _body.width, _body.height) +
                  printf_str("%.0f kg", _body.mass));
  put("controller", _controller);

  const avatar_energy_policy &p = _policy;
  const double carry =
      p.transport_cost * _body.mass * _gravity; // N: what carrying the body costs per m/s
  // Each policy line is a short value, and what it means as its tip (`policy.<line>_tip`): the
  // window shows the tip on hover, where a sentence beside the value would be cut off.
  const bool stop = p.empty == avatar_energy_policy::when_empty::stop;
  put("policy.budget", p.enabled ? "on" : "off");
  put("policy.budget_tip", p.enabled ? "The relay's, the same for everyone."
                                     : "The relay sets none: moving costs nothing.");
  put("policy.push", printf_str("%.0f N", p.force_max));
  put("policy.push_tip",
      printf_str("The most you push with: %.1f m/s^2 at this mass.", p.force_max / _body.mass));
  put("policy.power", watts(p.power_max));
  put("policy.power_tip", "The most power you spend. Resting power, " + watts(p.power_rest) +
                              ", refills the reserve all the time.");
  put("policy.reserve", joules(p.reserve_max));
  put("policy.reserve_tip", "What you can spend beyond resting power before you run out.");
  put("policy.carrying", watts(carry * 1.0) + " per m/s");
  put("policy.carrying_tip",
      printf_str("Carrying your weight along costs %.2f x m g |v|.", p.transport_cost));
  put("policy.empty", stop ? "stop" : "slow down");
  put("policy.empty_tip",
      stop ? printf_str("When the reserve runs out you stop until %.0f%% of it refills.",
                        100.0 * p.recover_fraction)
           : "When the reserve runs out you slow to what resting power sustains.");
  put("policy.sustains", carry > 0.0 ? printf_str("%.1f m/s", p.power_rest / carry) : "any speed");
  put("policy.sustains_tip", "The speed resting power keeps up for as long as you like.");

  put("ready", _ready ? "1" : "0");
  if (!_ready)
    return;
  const avatar_energy &e = _energy;
  const avatar_ledger &b = e.ledger;
  const double full = p.reserve_max > 0.0 ? p.reserve_max : 1.0;
  put("energy.fraction", printf_str("%.4f", e.reserve / full));
  put("energy.text", joules(e.reserve) + " of " + joules(p.reserve_max));
  put("energy.power", watts(e.power));
  put("energy.speed", printf_str("%.2f m/s", _speed));
  const std::string state = !p.enabled                ? "no budget"
                            : e.exhausted             ? "exhausted"
                            : e.power > p.power_rest  ? "spending"
                            : e.reserve >= full - 0.5 ? "full"
                                                      : "refilling";
  put("energy.state", state);
  put("energy.line", state + ", " + watts(e.power) + printf_str(" at %.2f m/s", _speed));

  put("ledger.start", joules(b.start));
  put("ledger.refilled", joules(b.refilled, true));
  put("ledger.work", joules(-b.work, true));
  put("ledger.carried", joules(-b.carried, true));
  put("ledger.overflow", joules(-b.overflow, true));
  put("ledger.unpaid", joules(b.unpaid, true));
  put("ledger.corrections", joules(b.corrections, true));
  put("ledger.now", joules(e.reserve));
  const double residual = b.residual(e.reserve);
  put("ledger.residual", printf_str("%.3f J", std::fabs(residual) < 5e-4 ? 0.0 : residual));
  put("ledger.distance", printf_str("%.0f m", b.distance));
  put("ledger.time", duration(b.time));
  put("ledger.exhausted", duration(b.exhausted_time));
  put("ledger.peak", watts(b.peak_power));
  put("ledger.balance",
      printf_str("balances (residual %.3f J)", std::fabs(residual) < 5e-4 ? 0.0 : residual));
  const double spent = b.work + b.carried;
  put("ledger.per_km",
      b.distance >= 10.0 ? printf_str("%.1f kJ/km", spent / 1000.0 / (b.distance / 1000.0)) : "--");
  put("ledger.summary",
      printf_str("%.0f m in ", b.distance) + duration(b.time) +
          (b.distance >= 10.0 ? printf_str(", %.1f kJ/km", spent / 1000.0 / (b.distance / 1000.0))
                              : std::string()) +
          "; peak " + watts(b.peak_power) + "; exhausted " + duration(b.exhausted_time));
  const double out = b.work + b.carried + b.overflow;
  put("ledger.share.work", printf_str("%.4f", out > 0.0 ? b.work / out : 0.0));
  put("ledger.share.carried", printf_str("%.4f", out > 0.0 ? b.carried / out : 0.0));
  put("ledger.share.overflow", printf_str("%.4f", out > 0.0 ? b.overflow / out : 0.0));

  put("history.reserve", series(_reserve, "%.2f"));
  put("history.power", series(_power, "%.0f"));
  put("history.window", "the last " + duration(_period * static_cast<double>(_reserve.size())));
}

} // namespace conference
} // namespace ariadne
} // namespace cvc
