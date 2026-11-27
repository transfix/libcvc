/*
  Copyright 2007-2011 The University of Texas at Austin

        Authors: Joe Rivera <transfix@ices.utexas.edu>
        Advisor: Chandrajit Bajaj <bajaj@cs.utexas.edu>

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// weather.cpp — the weather model (see weather.h).

#include <algorithm>
#include <bit>
#include <cmath>
#include <cvc/lsys/rng.h>
#include <cvc/world/weather.h>
#include <locale>
#include <sstream>

namespace cvc {
namespace world {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kInf = std::numeric_limits<double>::infinity();
// A cell's radius while it grows (a share of its radius at maturity at birth), and how much wider
// it spreads as it decays.
constexpr double R_BIRTH = 0.45, R_SPREAD = 0.15;
// The authored storms' draws sit apart from the slots' on the weather stream.
constexpr std::uint64_t AUTHORED_SALT = 0x5bd1e9955bd1e995ull;

double smooth(double u) {
  u = std::min(1.0, std::max(0.0, u));
  return u * u * (3.0 - 2.0 * u);
}

double lerp(double a, double b, double u) { return a + (b - a) * u; }

void heading(double speed, double az_deg, double &vx, double &vy) {
  const double a = az_deg * kPi / 180.0;
  vx = speed * std::sin(a);
  vy = -speed * std::cos(a);
}

double draw(std::uint64_t seed, std::uint64_t el, std::uint32_t d) {
  return lsys::uni(seed, lsys::stream::weather, el, d);
}

// The event's own weather time, when it carries one.
bool anchored(const weather_event &e) { return std::isfinite(e.tau); }

// An integer key for the digest: v rounded to 1/q.
std::int64_t key(double v, double q) {
  if (!std::isfinite(v))
    return std::numeric_limits<std::int64_t>::min();
  return static_cast<std::int64_t>(std::llround(v * q));
}

std::uint64_t fnv(std::uint64_t h, std::uint64_t v) {
  for (int b = 0; b < 8; ++b) {
    h ^= (v >> (8 * b)) & 0xffu;
    h *= 0x100000001b3ull;
  }
  return h;
}
} // namespace

const char *weather_state_name(weather_state s) {
  switch (s) {
  case weather_state::clear:
    return "clear";
  case weather_state::building:
    return "building";
  case weather_state::storm:
    return "storm";
  case weather_state::clearing:
    return "clearing";
  }
  return "clear";
}

weather::weather(const weather_params &p) : m_p(p) { rebuild(); }

void weather::set_params(const weather_params &p) {
  m_p = p;
  rebuild();
}

void weather::set_events(std::vector<weather_event> ev) {
  std::stable_sort(ev.begin(), ev.end(),
                   [](const weather_event &a, const weather_event &b) { return a.t < b.t; });
  m_ev = std::move(ev);
  rebuild();
}

void weather::add_event(const weather_event &e) {
  const auto at =
      std::upper_bound(m_ev.begin(), m_ev.end(), e,
                       [](const weather_event &a, const weather_event &b) { return a.t < b.t; });
  m_ev.insert(at, e);
  rebuild();
}

void weather::rebuild() {
  // The rate: weather time is the world time times the base rate until the first rate event, and
  // each rate event starts a piece where its world time maps (or at its anchor).
  m_rate.clear();
  m_rate.push_back({0.0, 0.0, std::max(0.0, m_p.rate)});
  for (const weather_event &e : m_ev)
    if (e.type == weather_event::kind::rate) {
      const double tau = anchored(e) ? e.tau : weather_time(e.t);
      m_rate.push_back({e.t, tau, std::max(0.0, e.scale)});
    }
  // The steering and the calms, in weather time.
  m_steer.clear();
  double vx, vy;
  heading(m_p.steer_speed, m_p.steer_az, vx, vy);
  m_steer.push_back({-kInf, vx, vy});
  m_calm.clear();
  for (const weather_event &e : m_ev) {
    const double tau = anchored(e) ? e.tau : weather_time(e.t);
    if (e.type == weather_event::kind::steer) {
      heading(e.speed, e.az, vx, vy);
      m_steer.push_back({tau, vx, vy});
    } else if (e.type == weather_event::kind::calm) {
      m_calm.push_back(tau);
    }
  }
  std::stable_sort(m_steer.begin(), m_steer.end(),
                   [](const steer_piece &a, const steer_piece &b) { return a.tau < b.tau; });
  std::sort(m_calm.begin(), m_calm.end());
}

double weather::weather_time(double t) const {
  // The last piece starting at or before t (the first, the base rate, also runs back before 0).
  const rate_piece *p = &m_rate.front();
  for (std::size_t k = 1; k < m_rate.size() && m_rate[k].t <= t; ++k)
    p = &m_rate[k];
  return p->tau + (t - p->t) * p->scale;
}

double weather::rate_at(double t) const {
  double s = m_rate.front().scale;
  for (std::size_t k = 1; k < m_rate.size() && m_rate[k].t <= t; ++k)
    s = m_rate[k].scale;
  return s;
}

void weather::steering(double tau, double &vx, double &vy) const {
  const steer_piece *p = &m_steer.front();
  for (std::size_t k = 1; k < m_steer.size() && m_steer[k].tau <= tau; ++k)
    p = &m_steer[k];
  vx = p->vx;
  vy = p->vy;
}

void weather::drift(double tau0, double tau1, double &dx, double &dy) const {
  dx = dy = 0.0;
  const double a = std::min(tau0, tau1), b = std::max(tau0, tau1);
  for (std::size_t k = 0; k < m_steer.size(); ++k) {
    const double lo = std::max(a, m_steer[k].tau);
    const double hi = std::min(b, k + 1 < m_steer.size() ? m_steer[k + 1].tau : kInf);
    if (hi > lo) {
      dx += m_steer[k].vx * (hi - lo);
      dy += m_steer[k].vy * (hi - lo);
    }
  }
  if (tau1 < tau0) {
    dx = -dx;
    dy = -dy;
  }
}

void weather::place(storm_cell &c, double miss) const {
  // Born upwind of the area, so that the steering at its birth brings it `miss` area half-widths to
  // the side of the centre in the middle of its maturity.
  double vx, vy;
  steering(c.born, vx, vy);
  const double g = std::min(1.0, std::max(0.0, m_p.growth));
  const double d = std::min(1.0 - g, std::max(0.0, m_p.decay));
  const double mid = c.life * (g + 0.5 * (1.0 - g - d));
  const double v = std::hypot(vx, vy);
  const double px = v > 0.0 ? -vy / v : 0.0, py = v > 0.0 ? vx / v : 1.0;
  c.x0 = m_p.cx + px * miss * m_p.area - vx * mid;
  c.y0 = m_p.cy + py * miss * m_p.area - vy * mid;
}

bool weather::scheduled(long k, storm_cell &c) const {
  const std::uint64_t el = static_cast<std::uint64_t>(k);
  if (!(m_p.interval > 0.0) || draw(m_p.seed, el, 0) >= m_p.chance)
    return false;
  const double top = std::min(m_p.life_max, m_p.interval);
  const double life = lerp(std::min(m_p.life_min, top), top, draw(m_p.seed, el, 1));
  if (!(life > 0.0))
    return false;
  const double born = k * m_p.interval + draw(m_p.seed, el, 2) * (m_p.interval - life);
  if (born < m_p.quiet)
    return false;
  c = storm_cell();
  c.id = el & ~storm_cell::AUTHORED;
  c.born = born;
  c.life = life;
  c.radius = lerp(m_p.radius_min, m_p.radius_max, draw(m_p.seed, el, 3));
  c.peak = lerp(m_p.peak_min, m_p.peak_max, draw(m_p.seed, el, 4));
  place(c, (2.0 * draw(m_p.seed, el, 5) - 1.0) * m_p.miss);
  return true;
}

bool weather::authored(const weather_event &e, storm_cell &c) const {
  if (e.type != weather_event::kind::storm)
    return false;
#if defined(__cpp_lib_bit_cast)
  const std::uint64_t bits = std::bit_cast<std::uint64_t>(e.t);
#else // NetBSD's libstdc++ (gcc 10) has no std::bit_cast; its clang has the builtin
  const std::uint64_t bits = __builtin_bit_cast(std::uint64_t, e.t);
#endif
  const std::uint64_t el = bits ^ AUTHORED_SALT;
  c = storm_cell();
  c.id = storm_cell::AUTHORED | (bits & ~storm_cell::AUTHORED);
  c.authored = true;
  c.born = anchored(e) ? e.tau : weather_time(e.t);
  c.life = e.life > 0.0 ? e.life : lerp(m_p.life_min, m_p.life_max, draw(m_p.seed, el, 1));
  c.radius =
      e.radius > 0.0 ? e.radius : lerp(m_p.radius_min, m_p.radius_max, draw(m_p.seed, el, 3));
  c.peak = e.peak > 0.0 ? std::min(1.0, e.peak)
                        : lerp(m_p.peak_min, m_p.peak_max, draw(m_p.seed, el, 4));
  if (std::isfinite(e.x) && std::isfinite(e.y)) {
    c.x0 = e.x;
    c.y0 = e.y;
  } else {
    place(c, (2.0 * draw(m_p.seed, el, 5) - 1.0) * m_p.miss);
  }
  return c.life > 0.0;
}

void weather::evaluate(storm_cell &c, double tau) const {
  const double g = std::min(1.0, std::max(0.0, m_p.growth));
  const double d = std::min(1.0 - g, std::max(0.0, m_p.decay));
  auto envelope = [&](double age) {
    const double u = age / c.life;
    if (u < g)
      return c.peak * smooth(u / g);
    if (u < 1.0 - d)
      return c.peak;
    return d > 0.0 ? c.peak * (1.0 - smooth((u - (1.0 - d)) / d)) : 0.0;
  };
  c.end = c.born + c.life;
  // The first calm in its life starts it clearing then.
  double calm = kInf;
  for (double tc : m_calm)
    if (tc >= c.born && tc < c.end) {
      calm = tc;
      break;
    }
  const double fade = std::max(1e-6, m_p.calm_s);
  if (calm < kInf)
    c.end = std::min(c.end, calm + fade);
  c.age = tau - c.born;
  const double u = c.age / c.life;
  double in = envelope(c.age);
  if (tau >= calm) {
    in = std::min(in, envelope(calm - c.born) * (1.0 - smooth((tau - calm) / fade)));
    c.phase = weather_state::clearing;
  } else {
    c.phase = u < g ? weather_state::building
                    : (u < 1.0 - d ? weather_state::storm : weather_state::clearing);
  }
  c.intensity = std::max(0.0, in);
  const double grow = g > 0.0 ? smooth(u / g) : 1.0;
  const double spread = d > 0.0 ? smooth((u - (1.0 - d)) / d) : 0.0;
  c.r = c.radius * (R_BIRTH + (1.0 - R_BIRTH) * grow) * (1.0 + R_SPREAD * spread);
  double dx, dy;
  drift(c.born, tau, dx, dy);
  c.x = c.x0 + dx;
  c.y = c.y0 + dy;
  steering(tau, c.vx, c.vy);
}

weather_now weather::at(double t) const {
  weather_now w;
  w.t = t;
  w.tau = weather_time(t);
  w.rate = rate_at(t);
  steering(w.tau, w.steer_x, w.steer_y);
  auto alive = [&](storm_cell &c) {
    evaluate(c, w.tau);
    return c.age >= 0.0 && w.tau < c.end;
  };
  // A scheduled storm lives inside its slot: only the slot holding tau can have one alive.
  storm_cell c;
  if (m_p.interval > 0.0) {
    const long k = static_cast<long>(std::floor(w.tau / m_p.interval));
    if (scheduled(k, c) && alive(c))
      w.cells.push_back(c);
  }
  for (const weather_event &e : m_ev)
    if (authored(e, c) && alive(c))
      w.cells.push_back(c);
  for (std::size_t k = 0; k < w.cells.size(); ++k)
    if (w.dominant < 0 || w.cells[k].intensity > w.storminess) {
      w.dominant = static_cast<int>(k);
      w.storminess = w.cells[k].intensity;
    }
  w.state = w.dominant < 0 ? weather_state::clear : w.cells[w.dominant].phase;
  return w;
}

std::vector<storm_cell> weather::recent(double t, double window) const {
  const double tau = weather_time(t), from = tau - std::max(0.0, window);
  std::vector<storm_cell> out;
  auto keep = [&](storm_cell &c) {
    evaluate(c, tau);
    if (c.age >= 0.0 && c.end > from)
      out.push_back(c);
  };
  storm_cell c;
  if (m_p.interval > 0.0) {
    // A scheduled storm lives inside its slot: the slots from `from`'s to tau's.
    const long k0 = static_cast<long>(std::floor(from / m_p.interval));
    const long k1 = static_cast<long>(std::floor(tau / m_p.interval));
    for (long k = std::max(k0, k1 - 64); k <= k1; ++k)
      if (scheduled(k, c))
        keep(c);
  }
  for (const weather_event &e : m_ev)
    if (authored(e, c))
      keep(c);
  return out;
}

double weather::rain_radius(double r, double intensity) const {
  // footprint(d, r) = 1 - smooth((d - r/2) / (r/2)); the rain needs intensity * footprint > onset.
  const double onset = std::min(0.999, std::max(0.0, m_p.rain_onset));
  if (!(r > 0.0) || intensity <= onset)
    return 0.0;
  const double target = 1.0 - onset / intensity; // smooth(u) = target, u in [0, 1]
  double lo = 0.0, hi = 1.0;
  for (int k = 0; k < 40; ++k) {
    const double u = 0.5 * (lo + hi);
    (smooth(u) < target ? lo : hi) = u;
  }
  return 0.5 * r * (1.0 + 0.5 * (lo + hi));
}

double weather::wetness(double t, double x, double y, double soak, double dry) const {
  soak = std::max(1.0, soak);
  dry = std::max(1.0, dry);
  const double tau = weather_time(t);
  double wet = 0.0;
  for (const storm_cell &c : recent(t, 4.0 * dry)) {
    const double rr = rain_radius(c.radius, c.peak);
    if (!(rr > 0.0))
      continue;
    // Its centre at weather time s from now is (c.x, c.y) + v s: it rained on the point while that
    // lay within rr, from its birth (s = -age) to its end (s = end - tau) or now.
    const double dx = x - c.x, dy = y - c.y, vv = c.vx * c.vx + c.vy * c.vy;
    double sIn, sOut;
    if (vv < 1e-8) {
      if (dx * dx + dy * dy >= rr * rr)
        continue;
      sIn = -c.age;
      sOut = std::min(c.end - tau, 0.0);
    } else {
      const double s0 = (dx * c.vx + dy * c.vy) / vv;
      const double qx = dx - c.vx * s0, qy = dy - c.vy * s0;
      const double h2 = rr * rr - (qx * qx + qy * qy);
      if (h2 <= 0.0)
        continue;
      const double hs = std::sqrt(h2 / vv);
      sIn = std::max(s0 - hs, -c.age);
      sOut = std::min(std::min(s0 + hs, c.end - tau), 0.0);
    }
    if (sOut <= sIn)
      continue;
    wet = std::max(wet, std::min(1.0, (sOut - sIn) / soak) * std::exp(sOut / dry));
  }
  return wet;
}

double lightning_strike::intensity(double age) const {
  if (age < 0.0 || age > life())
    return 0.0;
  double a = 0.0;
  for (int k = 0; k < strokes && k < 4; ++k) {
    const double u = (age - at[k]) / std::max(1e-6, dur[k]);
    if (u >= 0.0 && u <= 1.0)
      a = std::max(a, amp[k] * std::pow(1.0 - u, 1.7) * (0.75 + 0.25 * std::sin(u * 61.0)));
  }
  return std::max(a, 0.05 * std::exp(-7.0 * age)); // the ionised channel's afterglow
}

double lightning_strike::life() const {
  const int last = std::max(0, std::min(strokes, 4) - 1);
  return at[last] + dur[last] + 0.35;
}

std::vector<lightning_strike> weather::strikes(double t, double window) const {
  std::vector<lightning_strike> out;
  if (!(m_p.flash_rate > 0.0) || !(window > 0.0))
    return out;
  const double tau = weather_time(t), from = tau - window;
  for (const storm_cell &c : recent(t, window)) {
    for (long j = static_cast<long>(std::floor(from)); j <= static_cast<long>(std::floor(tau));
         ++j) {
      // The cell at the slot: its intensity there sets the rate.
      storm_cell s = c;
      evaluate(s, double(j) + 0.5);
      if (s.age < 0.0 || double(j) + 0.5 >= s.end)
        continue;
      const double ramp = std::min(1.0, std::max(0.0, (s.intensity - m_p.flash_onset) /
                                                          std::max(1e-6, 1.0 - m_p.flash_onset)));
      const double rate = m_p.flash_rate * ramp * ramp * (3.0 - 2.0 * ramp);
      const std::uint64_t el = (c.id * 0x9E3779B97F4A7C15ull) ^ static_cast<std::uint64_t>(j);
      if (draw(m_p.seed ^ 0x6c696768746e696eull, el, 0) >= rate)
        continue;
      lightning_strike k;
      k.id = el;
      k.tau = double(j) + draw(m_p.seed, el ^ AUTHORED_SALT, 1);
      if (k.tau <= from || k.tau > tau)
        continue;
      evaluate(s, k.tau);
      const double rr = std::max(0.25 * s.r, rain_radius(s.r, s.intensity));
      const double a = 2.0 * kPi * draw(m_p.seed, el ^ AUTHORED_SALT, 2);
      const double d = rr * std::sqrt(draw(m_p.seed, el ^ AUTHORED_SALT, 3));
      k.x = s.x + d * std::cos(a);
      k.y = s.y + d * std::sin(a);
      k.ground = draw(m_p.seed, el ^ AUTHORED_SALT, 4) < m_p.ground_share;
      // Return strokes, as ABYSSAL's: 1 to 4 flashes, each dimmer and shorter than the last.
      k.strokes = 1 + static_cast<int>(draw(m_p.seed, el ^ AUTHORED_SALT, 5) * 3.999);
      double ts = 0.0;
      for (int i = 0; i < k.strokes; ++i) {
        k.at[i] = ts;
        k.dur[i] = 0.035 + 0.09 * draw(m_p.seed, el ^ AUTHORED_SALT, 6 + 2 * i);
        k.amp[i] = std::pow(0.62, i);
        ts += 0.04 + 0.11 * draw(m_p.seed, el ^ AUTHORED_SALT, 7 + 2 * i);
      }
      out.push_back(k);
    }
  }
  std::sort(out.begin(), out.end(), [](const lightning_strike &a, const lightning_strike &b) {
    return a.tau != b.tau ? a.tau < b.tau : a.id < b.id;
  });
  return out;
}

void weather::compact(double t) {
  const double tau = weather_time(t);
  // Anchor every event at the weather time it has now, so the rate events can go.
  std::vector<weather_event> ev = m_ev;
  for (weather_event &e : ev)
    if (!anchored(e))
      e.tau = weather_time(e.t);
  // The rate: the last rate event at or before t, and the later ones.
  int lastRate = -1;
  for (std::size_t k = 0; k < ev.size(); ++k)
    if (ev[k].type == weather_event::kind::rate && ev[k].t <= t)
      lastRate = static_cast<int>(k);
  // The storms still to come or alive at t.
  std::vector<bool> keep(ev.size(), true);
  double oldest = tau; // the oldest birth among the cells that matter from t on
  storm_cell c;
  for (std::size_t k = 0; k < ev.size(); ++k)
    if (authored(ev[k], c)) {
      evaluate(c, tau);
      if (c.end <= tau)
        keep[k] = false;
      else
        oldest = std::min(oldest, c.born);
    }
  storm_cell slot;
  const bool hasSlot =
      m_p.interval > 0.0 && scheduled(static_cast<long>(std::floor(tau / m_p.interval)), slot);
  if (hasSlot) {
    evaluate(slot, tau);
    if (slot.end > tau)
      oldest = std::min(oldest, slot.born);
  }
  // The steering: what reaches back to the oldest birth that matters.
  int lastSteer = -1;
  for (std::size_t k = 0; k < ev.size(); ++k)
    if (ev[k].type == weather_event::kind::steer && ev[k].tau <= oldest)
      lastSteer = static_cast<int>(k);
  for (std::size_t k = 0; k < ev.size(); ++k) {
    const weather_event &e = ev[k];
    if (e.type == weather_event::kind::rate && static_cast<int>(k) < lastRate)
      keep[k] = false;
    if (e.type == weather_event::kind::steer && static_cast<int>(k) < lastSteer)
      keep[k] = false;
    if (e.type == weather_event::kind::calm) {
      // A calm matters while a cell born before it, whose own life runs past t, is still around
      // to be held to it (its kept storm, or the slot's).
      bool needed = false;
      auto bornBefore = [&](const storm_cell &s) {
        return s.born <= e.tau && s.born + s.life > tau;
      };
      for (std::size_t j = 0; j < ev.size() && !needed; ++j)
        if (keep[j] && authored(ev[j], c))
          needed = bornBefore(c);
      if (!needed && hasSlot)
        needed = bornBefore(slot);
      keep[k] = needed;
    }
  }
  std::vector<weather_event> out;
  for (std::size_t k = 0; k < ev.size(); ++k)
    if (keep[k])
      out.push_back(ev[k]);
  // Every kept event is anchored, so the rate events dropped no longer move them, and the last rate
  // event at or before t (anchored too) carries the weather time on from there.
  set_events(std::move(out));
}

double weather::footprint(double d, double r) {
  if (!(r > 0.0))
    return 0.0;
  return 1.0 - smooth((d - 0.5 * r) / (0.5 * r));
}

double weather::intensity(const weather_now &w, double x, double y) {
  double clear = 1.0;
  for (const storm_cell &c : w.cells)
    clear *= 1.0 - c.intensity * footprint(std::hypot(x - c.x, y - c.y), c.r);
  return 1.0 - clear;
}

double weather::rain_rate(const weather_now &w, double x, double y) const {
  const double onset = std::min(0.999, std::max(0.0, m_p.rain_onset));
  const double u = (intensity(w, x, y) - onset) / (1.0 - onset);
  return u > 0.0 ? m_p.rain_max * std::pow(std::min(1.0, u), 1.5) : 0.0;
}

double weather::nearby(const weather_now &w, double x, double y, double reach) {
  double s = 0.0;
  for (const storm_cell &c : w.cells) {
    const double d = std::hypot(x - c.x, y - c.y) - c.r;
    const double near = d <= 0.0 ? 1.0 : (reach > 0.0 ? 1.0 - smooth(d / reach) : 0.0);
    s = std::max(s, c.intensity * near);
  }
  return s;
}

double weather::front_reach(double r) const {
  return std::max(0.0, r) * std::max(m_p.front_ahead, m_p.front_behind) *
         (1.0 + std::max(0.0, m_p.front_width));
}

bool weather::front_cell_of(const storm_cell &c, cvc::algo::front_cell &out) const {
  const double speed = m_p.front_speed;
  if (!(speed > 0.0) || !(c.r > 0.0))
    return false;
  // The downdraft comes with the rain: none below its onset, a full cell's at full intensity.
  const double onset = std::min(0.999, std::max(0.0, m_p.rain_onset));
  const double strength = smooth((c.intensity - onset) / (1.0 - onset));
  if (!(strength > 0.0))
    return false;
  out.x = c.x;
  out.y = c.y;
  out.r = c.r;
  out.vx = c.vx;
  out.vy = c.vy;
  out.out = speed * strength;
  out.ahead = std::max(1e-6, m_p.front_ahead);
  out.behind = std::max(1e-6, m_p.front_behind);
  out.width = std::max(1e-6, m_p.front_width);
  return true;
}

void weather::gust_front(const weather_now &w, double x, double y, double &vx, double &vy) const {
  vx = vy = 0.0;
  // Each cell's outflow (cvc/algo/gust_front.h: the shape the sea and the shaders read too).
  cvc::algo::front_cell f;
  for (const storm_cell &c : w.cells)
    if (front_cell_of(c, f))
      cvc::algo::add_gust_front(f, x, y, vx, vy);
}

std::uint64_t weather::digest(const weather_now &w) {
  std::uint64_t h = 0xcbf29ce484222325ull;
  h = fnv(h, static_cast<std::uint64_t>(key(w.tau, 1e4)));
  h = fnv(h, static_cast<std::uint64_t>(w.state));
  h = fnv(h, static_cast<std::uint64_t>(w.dominant));
  h = fnv(h, w.cells.size());
  for (const storm_cell &c : w.cells) {
    h = fnv(h, c.id);
    h = fnv(h, static_cast<std::uint64_t>(c.phase));
    for (double v : {c.x, c.y, c.r, c.age, c.born, c.end})
      h = fnv(h, static_cast<std::uint64_t>(key(v, 1e4)));
    h = fnv(h, static_cast<std::uint64_t>(key(c.intensity, 1e7)));
  }
  return h;
}

// ─────────────────────── the events as text

namespace {
const char *kind_name(weather_event::kind k) {
  switch (k) {
  case weather_event::kind::storm:
    return "storm";
  case weather_event::kind::steer:
    return "steer";
  case weather_event::kind::rate:
    return "rate";
  case weather_event::kind::calm:
    return "calm";
  }
  return "storm";
}
} // namespace

std::string format_weather_events(const std::vector<weather_event> &ev) {
  std::ostringstream os;
  os.imbue(std::locale::classic());
  os.precision(17);
  bool first = true;
  for (const weather_event &e : ev) {
    if (!first)
      os << "; ";
    first = false;
    os << kind_name(e.type) << " t=" << e.t;
    switch (e.type) {
    case weather_event::kind::storm:
      if (std::isfinite(e.x) && std::isfinite(e.y))
        os << " x=" << e.x << " y=" << e.y;
      if (e.radius > 0.0)
        os << " r=" << e.radius;
      if (e.peak > 0.0)
        os << " peak=" << e.peak;
      if (e.life > 0.0)
        os << " life=" << e.life;
      break;
    case weather_event::kind::steer:
      os << " speed=" << e.speed << " az=" << e.az;
      break;
    case weather_event::kind::rate:
      os << " scale=" << e.scale;
      break;
    case weather_event::kind::calm:
      break;
    }
    if (anchored(e))
      os << " tau=" << e.tau;
  }
  return os.str();
}

bool parse_weather_events(const std::string &text, std::vector<weather_event> &out,
                          std::string *error) {
  auto fail = [error](const std::string &why) {
    if (error)
      *error = why;
    return false;
  };
  std::vector<weather_event> ev;
  std::size_t at = 0;
  while (at <= text.size()) {
    std::size_t end = text.find(';', at);
    if (end == std::string::npos)
      end = text.size();
    std::istringstream is(text.substr(at, end - at));
    is.imbue(std::locale::classic());
    at = end + 1;
    std::string kind;
    if (!(is >> kind))
      continue; // an empty entry
    weather_event e;
    if (kind == "storm")
      e.type = weather_event::kind::storm;
    else if (kind == "steer")
      e.type = weather_event::kind::steer;
    else if (kind == "rate")
      e.type = weather_event::kind::rate;
    else if (kind == "calm")
      e.type = weather_event::kind::calm;
    else
      return fail("unknown weather event '" + kind + "'");
    bool hasT = false;
    std::string kv;
    while (is >> kv) {
      const std::size_t eq = kv.find('=');
      if (eq == std::string::npos || eq == 0 || eq + 1 == kv.size())
        return fail("'" + kv + "' is not key=value");
      const std::string k = kv.substr(0, eq);
      std::istringstream vs(kv.substr(eq + 1));
      vs.imbue(std::locale::classic());
      double v = 0.0;
      if (!(vs >> v) || !(vs >> std::ws).eof())
        return fail("'" + kv + "': not a number");
      if (k == "t") {
        e.t = v;
        hasT = true;
      } else if (k == "x") {
        e.x = v;
      } else if (k == "y") {
        e.y = v;
      } else if (k == "r") {
        e.radius = v;
      } else if (k == "peak") {
        e.peak = v;
      } else if (k == "life") {
        e.life = v;
      } else if (k == "speed") {
        e.speed = v;
      } else if (k == "az") {
        e.az = v;
      } else if (k == "scale") {
        e.scale = v;
      } else if (k == "tau") {
        e.tau = v;
      } else {
        return fail("unknown key '" + k + "' in a " + kind + " event");
      }
    }
    if (!hasT)
      return fail("a " + kind + " event without t=");
    ev.push_back(e);
  }
  out = std::move(ev);
  return true;
}

} // namespace world
} // namespace cvc
