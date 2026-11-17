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

// cvc::world::weather — the weather model (OCEAN-AND-WEATHER-ROADMAP.md §9 F2): a pure function of
// its parameters, the world time and the authored events; storms on a schedule run clear, building,
// storm, clearing and clear; an authored storm starts one; cells ride the steering wind; the events
// survive their text form; compact() keeps the weather from now on as it was.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cvc/world/weather.h>
#include <gtest/gtest.h>
#include <vector>

using cvc::world::format_weather_events;
using cvc::world::parse_weather_events;
using cvc::world::storm_cell;
using cvc::world::weather;
using cvc::world::weather_event;
using cvc::world::weather_now;
using cvc::world::weather_params;
using cvc::world::weather_state;

namespace {
constexpr double kPi = 3.14159265358979323846;

weather_event storm_at(double t, double x, double y, double life = 1800.0, double r = 2000.0,
                       double peak = 1.0) {
  weather_event e;
  e.type = weather_event::kind::storm;
  e.t = t;
  e.x = x;
  e.y = y;
  e.life = life;
  e.radius = r;
  e.peak = peak;
  return e;
}

weather_event steer_at(double t, double speed, double az) {
  weather_event e;
  e.type = weather_event::kind::steer;
  e.t = t;
  e.speed = speed;
  e.az = az;
  return e;
}

weather_event rate_at(double t, double scale) {
  weather_event e;
  e.type = weather_event::kind::rate;
  e.t = t;
  e.scale = scale;
  return e;
}

weather_event calm_at(double t) {
  weather_event e;
  e.type = weather_event::kind::calm;
  e.t = t;
  return e;
}

// Only authored storms.
weather_params quiet() {
  weather_params p;
  p.chance = 0.0;
  return p;
}

// The first scheduled storm of a seed's weather: its slot's cell at its birth.
bool first_storm(const weather &w, storm_cell &out, double &born) {
  const weather_params &p = w.params();
  for (int k = 0; k < 200; ++k) {
    // Scan the slot a minute at a time for its storm's birth.
    for (double tau = k * p.interval; tau < (k + 1) * p.interval; tau += 60.0) {
      const weather_now n = w.at(tau);
      for (const storm_cell &c : n.cells)
        if (!c.authored) {
          out = c;
          born = c.born;
          return true;
        }
    }
  }
  return false;
}
} // namespace

TEST(WorldWeather, IsAPureFunctionOfTheSeedTheTimeAndTheEvents) {
  weather_params p;
  p.seed = 7;
  p.chance = 0.8;
  const std::vector<weather_event> ev = {storm_at(500.0, 100.0, -300.0),
                                         steer_at(900.0, 5.0, 135.0), rate_at(1500.0, 4.0),
                                         calm_at(4000.0)};
  weather a(p), b(p);
  a.set_events(ev);
  b.set_events(ev);
  // Asked in another order, after other times: the same answers (nothing is stepped or kept).
  std::vector<double> times;
  for (double t = -100.0; t < 20000.0; t += 137.0)
    times.push_back(t);
  std::vector<std::uint64_t> da, db(times.size());
  for (double t : times)
    da.push_back(weather::digest(a.at(t)));
  for (std::size_t k = times.size(); k-- > 0;)
    db[k] = weather::digest(b.at(times[k]));
  EXPECT_EQ(da, db);
  // Another seed: another schedule.
  weather_params q = p;
  q.seed = 8;
  weather c(q);
  c.set_events(ev);
  int differ = 0;
  for (std::size_t k = 0; k < times.size(); ++k)
    differ += weather::digest(c.at(times[k])) != da[k] ? 1 : 0;
  EXPECT_GT(differ, 10);
  // The golden hash: the same weather on every platform and in every build. The digest rounds
  // positions to 0.1 mm and intensities to 1e-7, so an ulp of sin or cos does not move it.
  std::uint64_t h = 0xcbf29ce484222325ull;
  for (std::uint64_t d : da)
    h = (h ^ d) * 0x100000001b3ull;
  std::printf("weather golden: %016llx\n", static_cast<unsigned long long>(h));
  EXPECT_EQ(h, 0x19e696666ebdfd0dull);
}

TEST(WorldWeather, AScheduledStormRunsItsLifeOnSchedule) {
  weather_params p;
  p.seed = 3;
  p.chance = 1.0; // every slot holds one
  const weather w(p);
  storm_cell c;
  double born = 0.0;
  ASSERT_TRUE(first_storm(w, c, born));
  EXPECT_GE(c.born, p.quiet);
  EXPECT_GE(c.life, p.life_min);
  EXPECT_LE(c.life, p.life_max);
  EXPECT_GE(c.radius, p.radius_min);
  EXPECT_LE(c.radius, p.radius_max);
  const double L = c.life, g = p.growth, d = p.decay;
  const double t1 = born + g * L, t2 = born + (1.0 - d) * L, t3 = born + L;
  // clear -> building -> storm -> clearing -> clear, at the shares of its life.
  EXPECT_EQ(w.at(born - 1.0).state, weather_state::clear);
  EXPECT_EQ(w.at(born + 1.0).state, weather_state::building);
  EXPECT_EQ(w.at(t1 - 1.0).state, weather_state::building);
  EXPECT_EQ(w.at(t1 + 1.0).state, weather_state::storm);
  EXPECT_EQ(w.at(t2 - 1.0).state, weather_state::storm);
  EXPECT_EQ(w.at(t2 + 1.0).state, weather_state::clearing);
  EXPECT_EQ(w.at(t3 - 1.0).state, weather_state::clearing);
  EXPECT_EQ(w.at(t3 + 1.0).state, weather_state::clear);
  // Its intensity climbs to its peak, holds it and falls back to nothing.
  EXPECT_NEAR(w.at(born).storminess, 0.0, 1e-12);
  EXPECT_NEAR(w.at(0.5 * (t1 + t2)).storminess, c.peak, 1e-12);
  double last = -1.0;
  for (double t = born; t < t1; t += 10.0) {
    const double s = w.at(t).storminess;
    EXPECT_GE(s, last);
    last = s;
  }
  EXPECT_LT(w.at(t3 - 1.0).storminess, 0.01);
  // Its radius widens as it grows.
  EXPECT_LT(w.at(born + 1.0).cells[0].r, w.at(t1 + 1.0).cells[0].r);
  // It never outlives its slot, so the next scheduled storm starts from a clear sky.
  EXPECT_LE(t3, (std::floor(born / p.interval) + 1.0) * p.interval + 1e-6);
}

TEST(WorldWeather, AScheduledStormBuildsUpwindAndCrossesTheArea) {
  weather_params p;
  p.seed = 11;
  p.chance = 1.0;
  p.cx = 200.0;
  p.cy = -100.0;
  p.steer_speed = 4.0;
  p.steer_az = 60.0;
  const weather w(p);
  storm_cell c;
  double born = 0.0;
  ASSERT_TRUE(first_storm(w, c, born));
  const double vx = 4.0 * std::sin(60.0 * kPi / 180.0), vy = -4.0 * std::cos(60.0 * kPi / 180.0);
  // Born upwind of the area's centre...
  EXPECT_LT((c.x0 - p.cx) * vx + (c.y0 - p.cy) * vy, 0.0);
  // ...it passes within `miss` area half-widths of it in the middle of its maturity.
  const double mid = born + c.life * (p.growth + 0.5 * (1.0 - p.growth - p.decay));
  const weather_now n = w.at(mid);
  ASSERT_EQ(n.cells.size(), 1u);
  EXPECT_LE(std::hypot(n.cells[0].x - p.cx, n.cells[0].y - p.cy), p.miss * p.area + 1e-6);
  EXPECT_EQ(n.state, weather_state::storm);
}

TEST(WorldWeather, AnAuthoredStormStartsOneAndACalmClearsIt) {
  weather w(quiet());
  EXPECT_EQ(w.at(1e5).state, weather_state::clear); // no schedule: clear forever
  w.add_event(storm_at(300.0, 0.0, 0.0, 1200.0));
  EXPECT_EQ(w.at(299.0).state, weather_state::clear);
  EXPECT_EQ(w.at(301.0).state, weather_state::building);
  EXPECT_EQ(w.at(300.0 + 600.0).state, weather_state::storm);
  EXPECT_EQ(w.at(300.0 + 1100.0).state, weather_state::clearing);
  EXPECT_EQ(w.at(300.0 + 1201.0).state, weather_state::clear);
  const weather_now n = w.at(900.0);
  ASSERT_EQ(n.cells.size(), 1u);
  EXPECT_TRUE(n.cells[0].authored);
  EXPECT_DOUBLE_EQ(n.storminess, 1.0);
  // A calm at 800 s: from then it clears over calm_s, from the intensity it had.
  w.add_event(calm_at(800.0));
  EXPECT_EQ(w.at(799.0).state, weather_state::storm);
  EXPECT_EQ(w.at(801.0).state, weather_state::clearing);
  EXPECT_NEAR(w.at(800.0).storminess, 1.0, 1e-12);
  EXPECT_GT(w.at(900.0).storminess, 0.0);
  EXPECT_LT(w.at(900.0).storminess, 1.0);
  EXPECT_EQ(w.at(800.0 + w.params().calm_s + 1.0).state, weather_state::clear);
  // A storm started after the calm is not held to it.
  w.add_event(storm_at(2000.0, 0.0, 0.0, 1200.0));
  EXPECT_EQ(w.at(2600.0).state, weather_state::storm);
  // Without a place it is born upwind of the area, as a scheduled one.
  weather u(quiet());
  weather_event e;
  e.t = 100.0;
  u.add_event(e);
  const weather_now m = u.at(101.0);
  ASSERT_EQ(m.cells.size(), 1u);
  EXPECT_LT(m.cells[0].x, 0.0); // the default steering blows toward +x
}

TEST(WorldWeather, CellsRideTheSteeringWind) {
  weather_params p = quiet();
  p.steer_speed = 5.0;
  p.steer_az = 90.0; // toward +x
  weather w(p);
  w.add_event(storm_at(0.0, -1000.0, 50.0, 3000.0));
  storm_cell c = w.at(400.0).cells.at(0);
  EXPECT_NEAR(c.x, -1000.0 + 5.0 * 400.0, 1e-9);
  EXPECT_NEAR(c.y, 50.0, 1e-9);
  EXPECT_NEAR(c.vx, 5.0, 1e-12);
  // The wind turns at 1000 s toward -y (heading 0) at 2 m/s: the track bends there, unbroken.
  w.add_event(steer_at(1000.0, 2.0, 0.0));
  const storm_cell before = w.at(1000.0 - 1e-6).cells.at(0);
  const storm_cell after = w.at(1000.0).cells.at(0);
  EXPECT_NEAR(before.x, after.x, 1e-4);
  EXPECT_NEAR(before.y, after.y, 1e-4);
  c = w.at(1500.0).cells.at(0);
  EXPECT_NEAR(c.x, -1000.0 + 5.0 * 1000.0, 1e-9);
  EXPECT_NEAR(c.y, 50.0 - 2.0 * 500.0, 1e-9);
  // The track before the turn is as it was: an event changes the weather from its time on.
  EXPECT_NEAR(w.at(400.0).cells.at(0).x, -1000.0 + 5.0 * 400.0, 1e-9);
  // A time-lapse: at 10 weather seconds a world second the storm lives a tenth as long and moves
  // ten times as fast in world time, along the same track.
  weather f(p);
  f.add_event(rate_at(0.0, 10.0));
  f.add_event(storm_at(0.0, -1000.0, 50.0, 3000.0));
  EXPECT_NEAR(f.at(40.0).cells.at(0).x, -1000.0 + 5.0 * 400.0, 1e-9);
  EXPECT_EQ(f.at(301.0).state, weather_state::clear);
  // Rate 0 holds the weather.
  f.add_event(rate_at(100.0, 0.0));
  EXPECT_EQ(weather::digest(f.at(100.0)), weather::digest(f.at(5000.0)));
}

TEST(WorldWeather, TheRainFallsUnderAThickCell) {
  weather_params p = quiet();
  p.steer_speed = 0.0; // the cell stays where it was born
  weather w(p);
  w.add_event(storm_at(0.0, 0.0, 0.0, 1800.0, 2000.0, 1.0));
  const weather_now clear = w.at(-10.0), full = w.at(900.0);
  EXPECT_EQ(w.rain_rate(clear, 0.0, 0.0), 0.0);
  EXPECT_NEAR(weather::intensity(full, 0.0, 0.0), 1.0, 1e-12);
  EXPECT_NEAR(w.rain_rate(full, 0.0, 0.0), w.params().rain_max, 1e-9);
  EXPECT_GT(w.rain_rate(full, 1000.0, 0.0), 0.0);    // within its core (half its radius)
  EXPECT_EQ(w.rain_rate(full, 2100.0, 0.0), 0.0);    // past its radius
  EXPECT_EQ(w.rain_rate(w.at(30.0), 0.0, 0.0), 0.0); // a young cell is not yet thick enough
  // nearby(): 1 under the cell, fading `reach` past its edge.
  EXPECT_DOUBLE_EQ(weather::nearby(full, 500.0, 0.0, 1000.0), 1.0);
  EXPECT_GT(weather::nearby(full, 2500.0, 0.0, 1000.0), 0.0);
  EXPECT_EQ(weather::nearby(full, 3500.0, 0.0, 1000.0), 0.0);
}

TEST(WorldWeather, TheEventsSurviveTheirTextForm) {
  std::vector<weather_event> ev = {storm_at(120.25, 1.0 / 3.0, -7.5e3), steer_at(300.0, 4.0, 135.0),
                                   rate_at(0.0, 30.0), calm_at(900.0)};
  weather_event loose;
  loose.t = 50.0; // a storm without place, radius, peak or life
  ev.push_back(loose);
  ev[2].tau = 12.5;
  const std::string text = format_weather_events(ev);
  std::printf("%s\n", text.c_str());
  std::vector<weather_event> back;
  std::string err;
  ASSERT_TRUE(parse_weather_events(text, back, &err)) << err;
  ASSERT_EQ(back.size(), ev.size());
  weather a(quiet()), b(quiet());
  a.set_events(ev);
  b.set_events(back);
  for (double t = 0.0; t < 4000.0; t += 97.0)
    EXPECT_EQ(weather::digest(a.at(t)), weather::digest(b.at(t)));
  EXPECT_EQ(format_weather_events(b.events()), format_weather_events(a.events()));
  // Malformed text leaves the list alone and says why.
  std::vector<weather_event> keep = back;
  EXPECT_FALSE(parse_weather_events("storm x=1", keep, &err));
  EXPECT_NE(err.find("without t="), std::string::npos);
  EXPECT_FALSE(parse_weather_events("hail t=3", keep, &err));
  EXPECT_FALSE(parse_weather_events("storm t=3 x=abc", keep, &err));
  EXPECT_FALSE(parse_weather_events("storm t=3 colour=2", keep, &err));
  EXPECT_EQ(keep.size(), back.size());
  EXPECT_TRUE(parse_weather_events("", keep, &err));
  EXPECT_TRUE(keep.empty());
}

TEST(WorldWeather, CompactKeepsTheWeatherFromNowOn) {
  weather_params p;
  p.seed = 5;
  p.chance = 0.7;
  p.interval = 2400.0;
  weather w(p);
  std::vector<weather_event> ev;
  for (int k = 0; k < 12; ++k) {
    ev.push_back(steer_at(200.0 * k + 10.0, 2.0 + k % 3, 30.0 * k));
    ev.push_back(rate_at(200.0 * k + 50.0, 1.0 + k % 4));
  }
  ev.push_back(storm_at(100.0, 0.0, 0.0, 1200.0));    // gone by 2000
  ev.push_back(storm_at(1500.0, 300.0, 0.0, 3000.0)); // alive at 2000
  ev.push_back(calm_at(1800.0));
  ev.push_back(storm_at(5000.0, 0.0, 0.0, 1500.0)); // still to come
  w.set_events(ev);
  const double now = 2000.0;
  std::vector<std::uint64_t> before;
  for (double t = now; t < now + 30000.0; t += 211.0)
    before.push_back(weather::digest(w.at(t)));
  weather c = w;
  c.compact(now);
  std::printf("compact: %zu events to %zu\n", w.events().size(), c.events().size());
  EXPECT_LT(c.events().size(), w.events().size());
  std::size_t k = 0;
  for (double t = now; t < now + 30000.0; t += 211.0, ++k)
    EXPECT_EQ(weather::digest(c.at(t)), before[k]) << "at t = " << t;
  // Its text reads back to the same weather.
  std::vector<weather_event> back;
  ASSERT_TRUE(parse_weather_events(format_weather_events(c.events()), back));
  weather r(p);
  r.set_events(back);
  k = 0;
  for (double t = now; t < now + 30000.0; t += 211.0, ++k)
    EXPECT_EQ(weather::digest(r.at(t)), before[k]) << "at t = " << t;
}

TEST(WorldWeather, AFrameIsCheap) {
  // A frame asks for the weather once: a slot, the authored storms and the steering's pieces.
  weather_params p;
  p.chance = 1.0;
  weather w(p);
  std::vector<weather_event> ev;
  for (int k = 0; k < 16; ++k) {
    ev.push_back(storm_at(500.0 * k, 0.0, 0.0));
    ev.push_back(steer_at(500.0 * k + 250.0, 3.0, 20.0 * k));
  }
  w.set_events(ev);
  const auto t0 = std::chrono::steady_clock::now();
  double acc = 0.0;
  const int n = 2000;
  for (int k = 0; k < n; ++k)
    acc += w.at(3000.0 + k).storminess;
  const double us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() / n;
  std::printf("weather::at: %.2f us with 32 events (%g)\n", us, acc);
#if defined(__OPTIMIZE__) || (defined(_MSC_VER) && !defined(_DEBUG))
  EXPECT_LT(us, 200.0);
#endif
}

// O19: the storms that rained recently, alive or gone (the ground dries along their tracks), and a
// cell's rain footprint.
TEST(WorldWeather, RecentCellsOutliveTheirRain) {
  weather w(quiet());
  w.add_event(storm_at(0.0, 0.0, 0.0, 1200.0, 2000.0, 1.0));
  EXPECT_TRUE(w.recent(-10.0, 600.0).empty()); // not born yet
  ASSERT_EQ(w.recent(600.0, 600.0).size(), 1u);
  EXPECT_GT(w.recent(600.0, 600.0)[0].intensity, 0.9);
  const std::vector<storm_cell> gone = w.recent(1500.0, 600.0); // ended at 1200, 300 s ago
  ASSERT_EQ(gone.size(), 1u);
  EXPECT_EQ(gone[0].intensity, 0.0);
  EXPECT_DOUBLE_EQ(gone[0].end, 1200.0);
  EXPECT_TRUE(w.at(1500.0).cells.empty());      // at() keeps only the living
  EXPECT_TRUE(w.recent(2000.0, 600.0).empty()); // past the window
  // A scheduled storm's too, from an earlier slot.
  weather_params p;
  p.seed = 3;
  p.chance = 1.0;
  const weather s(p);
  storm_cell c;
  double born = 0.0;
  ASSERT_TRUE(first_storm(s, c, born));
  const double after = born + c.life + 100.0;
  ASSERT_EQ(s.recent(after, 200.0).size(), 1u);
  EXPECT_EQ(s.recent(after, 200.0)[0].id, c.id);
  // The rain footprint: none below the onset, between half the radius and the radius above it.
  EXPECT_EQ(w.rain_radius(2000.0, 0.3), 0.0);
  const double rr = w.rain_radius(2000.0, 1.0);
  EXPECT_GT(rr, 1000.0);
  EXPECT_LT(rr, 2000.0);
  // ...where the field crosses the onset.
  EXPECT_NEAR(1.0 * weather::footprint(rr, 2000.0), w.params().rain_onset, 1e-6);
}

// O19: the ground wets under the rain, soaks in `soak` seconds of it, and dries by 1/e every `dry`
// seconds once the rain has passed (the same as cloud_sky's shader draws).
TEST(WorldWeather, TheGroundWetsAndDries) {
  weather_params p = quiet();
  p.steer_speed = 4.0; // toward +x
  weather w(p);
  // A full storm 1000 m in radius born 3 km upwind of the origin, crossing it.
  w.add_event(storm_at(0.0, -3000.0, 0.0, 2400.0, 1000.0, 1.0));
  const double soak = 300.0, dry = 600.0;
  EXPECT_EQ(w.wetness(-1.0, 0.0, 0.0, soak, dry), 0.0);
  EXPECT_EQ(w.wetness(100.0, 0.0, 0.0, soak, dry), 0.0); // its rain has not reached the origin
  // The rain radius at the peak, and when the origin is under it: the centre passes at 750 s.
  const double rr = w.rain_radius(1000.0, 1.0);
  const double in = (3000.0 - rr) / 4.0, out = (3000.0 + rr) / 4.0;
  std::printf("rain over the origin from %.0f s to %.0f s\n", in, out);
  double last = 0.0;
  for (double t = in + 10.0; t < out; t += 20.0) { // it wets as the rain lies on it
    const double wt = w.wetness(t, 0.0, 0.0, soak, dry);
    EXPECT_GE(wt, last - 1e-12);
    last = wt;
  }
  EXPECT_NEAR(w.wetness(in + 0.5 * soak, 0.0, 0.0, soak, dry), 0.5, 0.02);
  EXPECT_NEAR(w.wetness(out - 1.0, 0.0, 0.0, soak, dry), 1.0, 1e-9); // soaked
  // ...and dries once it has passed: 1/e every `dry` seconds.
  const double w1 = w.wetness(out + 100.0, 0.0, 0.0, soak, dry);
  const double w2 = w.wetness(out + 100.0 + dry, 0.0, 0.0, soak, dry);
  EXPECT_NEAR(w2 / w1, std::exp(-1.0), 1e-6);
  EXPECT_NEAR(w1, std::exp(-100.0 / dry), 1e-6);
  // Beside the track, out of the rain, it stays dry.
  EXPECT_EQ(w.wetness(out, 0.0, 2.0 * rr, soak, dry), 0.0);
  // Long after (past 4 dry times), the storm is forgotten.
  EXPECT_EQ(w.wetness(2400.0 + 5.0 * dry, 0.0, 0.0, soak, dry), 0.0);
}
