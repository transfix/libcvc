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

// weather.h — the weather model: storm cells on a schedule, and the storms people start
// (OCEAN-AND-WEATHER-ROADMAP.md §9 F2, F7; decision D5).
//
// The weather is a pure function of its parameters (the seed among them), the world time and a
// list of authored events. Nothing in it steps or remembers: weather::at(t) computes the cells
// alive at t from scratch, so two clients holding the same parameters and events agree at every t,
// and a client that joins late, seeks or replays sees what the others see. Standard library only.
//
// Storm cells. Each has a birthplace, a radius and a peak intensity, and lives `life` seconds of
// weather time: it grows over the first `growth` share of its life (a smoothstep to its peak),
// holds its peak, and decays over the last `decay` share. Its centre moves with the steering wind
// from where it was born, and its radius widens as it grows (and a little more as it decays, an
// anvil spreading). Where a cell is in that life is its phase: building, storm or clearing. The
// weather's state is the strongest live cell's phase, or clear when no cell is alive.
//
// The schedule. Weather time is cut into slots of `interval` seconds; slot k holds a storm or not
// by a hashed draw on the seed (`chance`), and its life, radius, peak and start within the slot
// are hashed draws too, the life at most the slot's length, so a scheduled storm never outlives
// its slot and scheduled storms never overlap: clear, building, storm, clearing, clear. A
// scheduled storm is born upwind of the area it serves, timed to cross near its centre in the
// middle of its maturity: it builds upwind, walks over the world and clears past it.
//
// Authored events (the `weather.*` state keys carry them as text, format_weather_events):
//   storm  a storm born at t: at (x, y), or upwind of the area like a scheduled one; its radius,
//          peak and life given or drawn from the schedule's ranges;
//   steer  the steering wind from t on (speed, heading). The weather node logs one whenever the
//          scene's wind changes, so the cells' tracks stay a function of the events;
//   rate   weather seconds per world second from t on (a time-lapse of the weather; 0 holds it);
//   calm   every cell alive at t starts to clear then, over calm_s weather seconds.
// Any event may carry `tau`, the weather time it takes effect at (compact() writes it).
//
// Hooks for the work after this (O19 rain, O20 storm wind and lightning): the cells at t with
// their centres, radii, intensities, phases and motion (weather_now::cells), the storm field
// intensity(x, y), and the rain rate rain_rate(x, y) in mm/h. Everything a frame needs is
// O(cells + events): nothing here is per texel.
//
// The design of the states and of authored events follows the Director of ABYSSAL
// (Token-Gremlin/natural-disasters, src/weather/Director.js, MIT), which plays scripted acts and
// eases the weather toward each act's targets; here the targets are storm cells on a hashed
// schedule instead of a table, and the easing is a closed form of time, not a damped state.

#ifndef CVC_WORLD_WEATHER_H
#define CVC_WORLD_WEATHER_H

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace cvc {
namespace world {

struct weather_params {
  std::uint64_t seed = 1;
  // The area the schedule's storms serve: its centre and half-width (metres).
  double cx = 0.0, cy = 0.0, area = 600.0;
  // The schedule: slots of `interval` weather seconds, a share `chance` of them holding a storm (0:
  // only authored storms). No scheduled storm is born before `quiet` (a world opens clear).
  double interval = 3600.0;
  double chance = 0.5;
  double quiet = 600.0;
  // A storm's life (weather seconds), the shares of it spent growing and decaying, its radius at
  // maturity (metres) and its peak intensity (0..1), each drawn from [min, max].
  double life_min = 1200.0, life_max = 3600.0;
  double growth = 0.3, decay = 0.35;
  double radius_min = 1500.0, radius_max = 3000.0;
  double peak_min = 0.75, peak_max = 1.0;
  // How far to the side of the area's centre a scheduled storm may pass, in area half-widths.
  double miss = 0.5;
  // The steering wind before any steer event: metres per weather second, and the heading it blows
  // toward in degrees as cloud_layer's wind_az (90 is +x, 0 is -y).
  double steer_speed = 3.0, steer_az = 90.0;
  // Weather seconds per world second before any rate event.
  double rate = 1.0;
  // The rain under a cell: none below an intensity of rain_onset, rain_max mm/h at full.
  double rain_onset = 0.35, rain_max = 50.0;
  // A calm event's fade, weather seconds.
  double calm_s = 300.0;
  // Lightning (O20): a cell strikes at flash_rate a weather second at full intensity, ramping in
  // from flash_onset, within its rain radius; a share ground_share of the strikes reach the ground
  // (the rest stay in the cloud).
  double flash_rate = 0.05, flash_onset = 0.55, ground_share = 0.6;
};

// A lightning strike: where and when, scheduled by the weather like everything in it, so every
// client sees the same strike at the same time. Its strokes are in world seconds from the strike
// (a flash is a fraction of a second whatever the weather's rate).
struct lightning_strike {
  std::uint64_t id = 0;    // the seed of its bolt's shape
  double tau = 0.0;        // weather time
  double x = 0.0, y = 0.0; // the ground point under it
  bool ground = true;      // to the ground, or only in the cloud
  int strokes = 1;         // return strokes, 1..4
  double at[4] = {0, 0, 0, 0}, dur[4] = {0, 0, 0, 0}, amp[4] = {0, 0, 0, 0};
  // Its brightness `age` world seconds after it: each stroke a fast rise and a decay with a
  // flicker (ABYSSAL's), and a faint afterglow; 0 past its life.
  double intensity(double age) const;
  double life() const; // world seconds it shows for
};

enum class weather_state : int { clear = 0, building = 1, storm = 2, clearing = 3 };
const char *weather_state_name(weather_state s);

struct weather_event {
  enum class kind : int { storm = 0, steer = 1, rate = 2, calm = 3 };
  kind type = kind::storm;
  double t = 0.0; // the world time it takes effect, seconds
  // storm: its birthplace (NaN: upwind of the area, as a scheduled storm), its radius at maturity,
  // its peak and its life (0: drawn from the schedule's ranges).
  double x = std::numeric_limits<double>::quiet_NaN();
  double y = std::numeric_limits<double>::quiet_NaN();
  double radius = 0.0, peak = 0.0, life = 0.0;
  // steer: metres per weather second and the heading (degrees, as weather_params::steer_az).
  double speed = 0.0, az = 0.0;
  // rate: weather seconds per world second from t on.
  double scale = 1.0;
  // When finite, the weather time the event takes effect at, in place of the one its world time
  // maps to: compact() anchors the events it keeps, so the rate events it drops no longer matter.
  double tau = std::numeric_limits<double>::quiet_NaN();
};

struct storm_cell {
  std::uint64_t id = 0; // its slot (scheduled), or AUTHORED | the event's time bits
  bool authored = false;
  double born = 0.0; // weather time
  double life = 0.0; // weather seconds, as drawn
  double end = 0.0;  // when it is gone (a calm can bring it forward)
  double radius = 0.0, peak = 0.0;
  double x0 = 0.0, y0 = 0.0; // its birthplace
  // At the time asked for:
  double age = 0.0;          // weather seconds since its birth
  double x = 0.0, y = 0.0;   // its centre
  double r = 0.0;            // its radius
  double intensity = 0.0;    // 0..peak
  double vx = 0.0, vy = 0.0; // its motion, metres per weather second (the steering wind)
  weather_state phase = weather_state::building;
  static constexpr std::uint64_t AUTHORED = 1ull << 63;
};

// The weather at one world time.
struct weather_now {
  double t = 0.0;    // world time
  double tau = 0.0;  // weather time
  double rate = 1.0; // weather seconds per world second at t
  weather_state state = weather_state::clear;
  int dominant = -1;                   // the strongest cell (an index into cells), or -1
  double storminess = 0.0;             // its intensity
  double steer_x = 0.0, steer_y = 0.0; // the steering wind, metres per weather second
  std::vector<storm_cell> cells;       // the cells alive at t
};

class weather {
public:
  explicit weather(const weather_params &p = weather_params());

  const weather_params &params() const { return m_p; }
  void set_params(const weather_params &p);

  // The authored events, kept in time order (equal times keep their order).
  const std::vector<weather_event> &events() const { return m_ev; }
  void set_events(std::vector<weather_event> ev);
  void add_event(const weather_event &e);
  // Drop the events that can no longer change the weather at t or later, folding what they did
  // into the ones kept (a rate anchored, the steering at the oldest live cell's birth), so a long
  // session's list stays short. at(t') is unchanged for every t' >= t.
  void compact(double t);

  // World time to weather time, and the rate there.
  double weather_time(double t) const;
  double rate_at(double t) const;
  // The steering wind at weather time tau (metres per weather second), and how far it carries a
  // cell from tau0 to tau1.
  void steering(double tau, double &vx, double &vy) const;
  void drift(double tau0, double tau1, double &dx, double &dy) const;

  // The weather at world time t: the live cells and the state. O(cells + events).
  weather_now at(double t) const;
  // The cells born by t whose life reached into the last `window` weather seconds, alive or not, as
  // at(t) evaluates them (a gone cell's intensity is 0, its centre where the steering has carried
  // it): what the ground's drying reads (O19), the tracks of the storms that rained on it.
  std::vector<storm_cell> recent(double t, double window) const;
  // A cell's rain footprint: the radius within which its field passes the rain onset at intensity
  // `intensity` (0 when it does not), for its radius r.
  double rain_radius(double r, double intensity) const;
  // How wet the ground at (x, y) is at world time t, 0..1 (the wet roads O19 draws, the friction
  // O20 will read): for each storm of the last 4 `dry` weather seconds, how long its rain lay over
  // the point over `soak` seconds (at most 1), times exp(-(weather seconds since its rain left) /
  // dry); the wettest. A storm's rain covers its rain radius at its peak, along its track as its
  // motion now runs it back (cloud_sky's shader computes the same, cloudWet).
  double wetness(double t, double x, double y, double soak, double dry) const;
  // The lightning strikes of the weather seconds (tau - window, tau], tau the weather time at t:
  // each cell's alive in it, in 1-second slots, a slot holding one by a hashed draw at the cell's
  // rate there (flash_rate times a smoothstep of its intensity from flash_onset to 1), at a hashed
  // time in it and a hashed point in the cell's rain radius. Oldest first. O(cells x window).
  std::vector<lightning_strike> strikes(double t, double window) const;

  // A cell's footprint at distance d from its centre: 1 over its core, falling to 0 at its radius.
  static double footprint(double d, double r);
  // The storm field at (x, y): 1 - prod(1 - I_k f_k), 0 in clear air, up to 1 under a full cell.
  static double intensity(const weather_now &w, double x, double y);
  // The rain rate at (x, y), mm/h: rain_max * ((I - onset) / (1 - onset))^1.5 where the field I
  // passes the onset (the hook O19's rain reads).
  double rain_rate(const weather_now &w, double x, double y) const;
  // The strongest cell's intensity, each cell weighted by how near it is to (x, y): 1 within its
  // radius, falling to 0 `reach` metres past it. What a deck seen from (x, y) should show.
  static double nearby(const weather_now &w, double x, double y, double reach);
  // A digest of the weather at t (the cells and the state, rounded to a tenth of a millimetre and
  // 1e-7 of intensity, so one ulp of a platform's sin or cos does not move it): the golden hash.
  static std::uint64_t digest(const weather_now &w);

private:
  weather_params m_p;
  std::vector<weather_event> m_ev;
  // The rate in pieces: from world time t, weather time tau + (t' - t) * scale.
  struct rate_piece {
    double t, tau, scale;
  };
  std::vector<rate_piece> m_rate;
  // The steering in pieces: from weather time tau, (vx, vy).
  struct steer_piece {
    double tau, vx, vy;
  };
  std::vector<steer_piece> m_steer;
  std::vector<double> m_calm; // the calms' weather times, rising
  void rebuild();
  bool scheduled(long k, storm_cell &c) const;
  bool authored(const weather_event &e, storm_cell &c) const;
  void place(storm_cell &c, double miss) const;
  void evaluate(storm_cell &c, double tau) const;
};

// The events as text (the `weather.events` key) and back: entries split by ';', each a kind and
// key=value pairs, numbers written so they read back exactly ("storm t=120 r=2000; steer t=300
// speed=4 az=135; rate t=0 scale=30; calm t=900"). parse_weather_events leaves `out` alone and
// says why in `error` when the text is malformed; an empty text is no events.
std::string format_weather_events(const std::vector<weather_event> &ev);
bool parse_weather_events(const std::string &text, std::vector<weather_event> &out,
                          std::string *error = nullptr);

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_WEATHER_H
