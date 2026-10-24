/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// See plant_log.h.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cvc/nav/detail/dyn_bike.h>
#include <cvc/nav/dyn_bike.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/nav_plant.h>
#include <cvc/physics/plant_log.h>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace cvc {
namespace physics {

namespace {

constexpr double kG = 9.80665;
constexpr double kPi = 3.14159265358979323846;

vec3 cross(const vec3 &a, const vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

vec3 rotate(const quat &q, const vec3 &v) {
  const vec3 u{q.x, q.y, q.z};
  const vec3 t = cross(u, v) * 2.0;
  return v + t * q.w + cross(u, t);
}

// The rearmost unsteered axle's x, as nav_plant reads the rear axle; 0 without one.
double rear_axle_x(const vehicle_desc &d) {
  double rear = 0.0;
  bool have = false;
  for (const axle_desc &a : d.axles)
    if (!a.steered && (!have || a.x < rear)) {
      rear = a.x;
      have = true;
    }
  return rear;
}

double wrap(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a < -kPi)
    a += 2.0 * kPi;
  return a;
}

// Accumulates one bin's squared residuals.
struct accum {
  std::uint64_t n = 0;
  double p2 = 0.0, pmax = 0.0, h2 = 0.0, hmax = 0.0, s2 = 0.0, smax = 0.0, vy2 = 0.0, r2 = 0.0;
  void add(double p, double h, double s, double vy, double r) {
    ++n;
    p2 += p * p;
    pmax = std::max(pmax, p);
    h2 += h * h;
    hmax = std::max(hmax, h);
    s2 += s * s;
    smax = std::max(smax, s);
    vy2 += vy * vy;
    r2 += r * r;
  }
  residual_bin bin(double lo, double hi) const {
    residual_bin b;
    b.lo_g = lo;
    b.hi_g = hi;
    b.ticks = n;
    if (n) {
      b.position_rms_m = std::sqrt(p2 / n);
      b.position_max_m = pmax;
      b.heading_rms_rad = std::sqrt(h2 / n);
      b.heading_max_rad = hmax;
      b.speed_rms_mps = std::sqrt(s2 / n);
      b.speed_max_mps = smax;
      b.lateral_rms_mps = std::sqrt(vy2 / n);
      b.yaw_rate_rms = std::sqrt(r2 / n);
    }
    return b;
  }
};

} // namespace

plant_log::plant_log(const cvc::nav::sim_world &sim, nav_plant &plant) : _sim(sim), _plant(plant) {
  const cvc::nav::sim_world::config &c = sim.current_config();
  // The tick is a whole number of physics steps (nav_plant::attach requires it); nav's dt is a
  // float, so take it from the physics step.
  const double fixed = plant.physics_world().config().fixed_dt;
  _dt = std::max(1.0, std::round(c.veh.dt / fixed)) * fixed;
  // The top speed sim_world plans each vehicle with: its own, capped at nav's.
  for (int i = 0; i < plant.size(); ++i) {
    const float lim = plant.limits(i).vmax;
    const double vn = lim > 0.0f ? std::min(lim, c.veh.vmax) : c.veh.vmax;
    _vmax.push_back(vn * plant.options().metres_per_world_unit / c.scale);
  }
  // nav's accelerations are normalized: (m / metres_per_world_unit) * scale per s^2.
  _accel_si = plant.options().metres_per_world_unit / c.scale;
}

planar_sample plant_log::sample(int i) const {
  const vehicle_state vs = _plant.physics_world().vehicle_state(_plant.vehicle(i));
  const body_state &b = vs.body;
  const vehicle_desc &d = _plant.desc(i);
  const vec3 fwd = rotate(b.rotation, {1.0, 0.0, 0.0});
  const vec3 com = b.position + rotate(b.rotation, d.center_of_mass);
  const vec3 rear = b.position + rotate(b.rotation, {rear_axle_x(d), 0.0, 0.0});
  planar_sample s;
  s.psi = std::atan2(fwd.y, fwd.x);
  const double c = std::cos(s.psi), sn = std::sin(s.psi);
  s.x = com.x;
  s.y = com.y;
  s.vx = b.linear_velocity.x * c + b.linear_velocity.y * sn;
  s.vy = -b.linear_velocity.x * sn + b.linear_velocity.y * c;
  s.r = b.angular_velocity.z;
  s.rear_x = rear.x;
  s.rear_y = rear.y;
  s.steer = vs.steer;
  s.grade = std::max(-1.0, std::min(1.0, fwd.z));
  // + left side up: the body's left axis rising (plant_state::roll, plant_sample::roll).
  s.roll = std::max(-1.0, std::min(1.0, rotate(b.rotation, {0.0, 1.0, 0.0}).z));
  s.wheels = static_cast<int>(vs.wheels.size());
  for (const wheel_state &w : vs.wheels)
    s.wheels_down += w.contact ? 1 : 0;
  s.grounded = s.wheels_down + 1 >= s.wheels;
  if (i < static_cast<int>(_vmax.size()))
    s.vmax = _vmax[i];
  return s;
}

void plant_log::start() {
  const int n = _plant.size();
  _runs.assign(static_cast<std::size_t>(n), {});
  for (int i = 0; i < n; ++i)
    _runs[i].push_back(sample(i));
}

void plant_log::record() {
  if (_runs.empty())
    start();
  const float *sc = _sim.plant_steer_command();
  const float *ac = _sim.plant_accel_command();
  for (int i = 0; i < agents(); ++i) {
    std::vector<planar_sample> &run = _runs[i];
    const planar_sample &prev = run.back();
    planar_sample s = sample(i);
    s.t = static_cast<double>(run.size()) * _dt;
    s.psi = prev.psi + wrap(s.psi - prev.psi);
    s.steer_cmd = sc ? sc[i] : 0.0;
    s.accel_cmd = ac ? ac[i] * _accel_si : 0.0;
    const vehicle_desc &d = _plant.desc(i);
    s.drive = std::min(std::max(s.accel_cmd, -d.max_decel), d.max_accel);
    s.contacts = _plant.static_contacts_step(i);
    s.energy = _plant.energy_step(i);
    // The centre of mass's acceleration over the tick, across the mean heading.
    const double c0 = std::cos(prev.psi), s0 = std::sin(prev.psi);
    const double c1 = std::cos(s.psi), s1 = std::sin(s.psi);
    const double vwx0 = prev.vx * c0 - prev.vy * s0, vwy0 = prev.vx * s0 + prev.vy * c0;
    const double vwx1 = s.vx * c1 - s.vy * s1, vwy1 = s.vx * s1 + s.vy * c1;
    const double pm = 0.5 * (prev.psi + s.psi);
    s.a_lat = (-(vwx1 - vwx0) * std::sin(pm) + (vwy1 - vwy0) * std::cos(pm)) / _dt;
    s.a_lon = ((vwx1 - vwx0) * std::cos(pm) + (vwy1 - vwy0) * std::sin(pm)) / _dt;
    run.push_back(s);
  }
}

namespace {

// Why a tick is left out of the residual (0: it is not).
enum skip_why { kept = 0, contact, airborne, impact, rest };

skip_why why_skipped(const planar_sample &s0, const planar_sample &s1) {
  if (s1.contacts > 0)
    return contact;
  if (!s0.grounded || !s1.grounded)
    return airborne;
  if (std::hypot(s1.a_lat, s1.a_lon) > kImpactG * kG)
    return impact;
  if (std::fabs(s0.vx) < 0.2 && std::fabs(s1.vx) < 0.2)
    return rest;
  return kept;
}

// The dynamic model for one tick: from the plant's state at its start (and the model's own drive
// actuator, a_shadow, carried along the run), the held command for nsub substeps, as
// one_tick_residuals takes it. Updates a_shadow.
cvc::nav::diff::dyn_state<double> dyn_tick(const cvc::nav::diff::dyn_bike_params &p,
                                           cvc::nav::diff::bike_veh &lim, const planar_sample &s0,
                                           const planar_sample &s1, int nsub, double &a_shadow) {
  // Both models keep the speed within the plan's, as sim_world's reference does.
  lim.vmax = static_cast<float>(s0.vmax > 0.0 ? s0.vmax : 1e3);
  cvc::nav::diff::dyn_state<double> m{};
  m.x = s0.rear_x;
  m.y = s0.rear_y;
  m.psi = s0.psi;
  m.vx = s0.vx;
  m.vy = s0.vy;
  m.r = s0.r;
  m.delta = s0.steer;
  m.a = a_shadow;
  for (int j = 0; j < nsub; ++j) {
    cvc::nav::diff::dyn_state<double> o;
    cvc::nav::diff::dyn_plant<double>(p, lim, m, s1.steer_cmd, s1.accel_cmd, s0.grade, o);
    m = o;
  }
  a_shadow = m.a;
  return m;
}

cvc::nav::diff::bike_veh dyn_limits(const cvc::nav::diff::dyn_bike_params &p, double max_steer,
                                    double h) {
  cvc::nav::diff::bike_veh lim{};
  lim.L = p.lf + p.lr;
  lim.delta_max = static_cast<float>(max_steer);
  lim.hdt = static_cast<float>(h);
  lim.allow_reverse = 1;
  return lim;
}

cvc::nav::diff::dyn_bike_params si_params(const cvc::nav::dyn_bike_si &si) {
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return cvc::nav::dyn_bike_params_of(si, f);
}

void check_residual_args(double dt, int nsub, const std::vector<double> &bins_g) {
  if (!(dt > 0.0) || nsub < 1)
    throw std::invalid_argument("cvc::physics::one_tick_residuals: needs dt > 0 and nsub >= 1");
  for (std::size_t k = 1; k < bins_g.size(); ++k)
    if (!(bins_g[k] > bins_g[k - 1]))
      throw std::invalid_argument("cvc::physics::one_tick_residuals: bins must ascend");
}

} // namespace

plant_residuals one_tick_residuals(const std::vector<std::vector<planar_sample>> &runs, double dt,
                                   const vehicle_desc &d, double mu, int nsub,
                                   const std::vector<double> &bins_g) {
  return one_tick_residuals(runs, dt, nav_plant::dyn_bike_si_of(d, mu), d.wheelbase(), d.max_steer,
                            nsub, bins_g);
}

plant_residuals one_tick_residuals(const std::vector<std::vector<planar_sample>> &runs, double dt,
                                   const cvc::nav::dyn_bike_si &si, double wheelbase,
                                   double max_steer, int nsub, const std::vector<double> &bins_g) {
  check_residual_args(dt, nsub, bins_g);
  const cvc::nav::diff::dyn_bike_params p = si_params(si);
  const double h = dt / nsub;
  cvc::nav::diff::bike_veh lim = dyn_limits(p, max_steer, h);
  const double L = wheelbase;
  const std::size_t nb = bins_g.size() + 1;
  std::vector<accum> dyn(nb), kin(nb);
  plant_residuals out;
  for (const std::vector<planar_sample> &run : runs) {
    double a_shadow = 0.0; // the model's drive actuator, run on the commands alone
    for (std::size_t k = 0; k + 1 < run.size(); ++k) {
      const planar_sample &s0 = run[k], &s1 = run[k + 1];
      const cvc::nav::diff::dyn_state<double> m = dyn_tick(p, lim, s0, s1, nsub, a_shadow);
      // The kinematic bicycle (D14's reference): the command at once, about the rear axle.
      const double vmax = s0.vmax > 0.0 ? s0.vmax : 1e3;
      double kx = s0.rear_x, ky = s0.rear_y, kth = s0.psi, ksp = s0.vx;
      for (int j = 0; j < nsub; ++j) {
        ksp = std::min(std::max(ksp + h * s1.accel_cmd, -0.25 * vmax), vmax);
        kth = kth + h * (ksp / L) * std::tan(s1.steer_cmd);
        kx = kx + h * ksp * std::cos(kth);
        ky = ky + h * ksp * std::sin(kth);
      }
      switch (why_skipped(s0, s1)) {
      case contact:
        ++out.skipped_contact;
        continue;
      case airborne:
        ++out.skipped_airborne;
        continue;
      case impact:
        ++out.skipped_impact;
        continue;
      case rest:
        ++out.skipped_rest;
        continue;
      case kept:
        break;
      }
      const double g = std::fabs(s1.a_lat) / kG;
      std::size_t b = 0;
      while (b < bins_g.size() && g >= bins_g[b])
        ++b;
      dyn[b].add(std::hypot(m.x - s1.rear_x, m.y - s1.rear_y), std::fabs(wrap(m.psi - s1.psi)),
                 std::fabs(m.vx - s1.vx), std::fabs(m.vy - s1.vy), std::fabs(m.r - s1.r));
      kin[b].add(std::hypot(kx - s1.rear_x, ky - s1.rear_y), std::fabs(wrap(kth - s1.psi)),
                 std::fabs(ksp - s1.vx), 0.0, 0.0);
    }
  }
  for (std::size_t b = 0; b < nb; ++b) {
    const double lo = b == 0 ? 0.0 : bins_g[b - 1];
    const double hi = b < bins_g.size() ? bins_g[b] : std::numeric_limits<double>::infinity();
    out.dynamic.push_back(dyn[b].bin(lo, hi));
    out.kinematic.push_back(kin[b].bin(lo, hi));
  }
  return out;
}

double dyn_bike_cost(const std::vector<std::vector<planar_sample>> &runs, double dt,
                     const cvc::nav::dyn_bike_si &si, double max_steer, int nsub,
                     std::uint64_t *ticks) {
  check_residual_args(dt, nsub, {});
  const cvc::nav::diff::dyn_bike_params p = si_params(si);
  cvc::nav::diff::bike_veh lim = dyn_limits(p, max_steer, dt / nsub);
  const double L = si.cg_to_front + si.cg_to_rear;
  double sum = 0.0;
  std::uint64_t n = 0;
  for (const std::vector<planar_sample> &run : runs) {
    double a_shadow = 0.0;
    for (std::size_t k = 0; k + 1 < run.size(); ++k) {
      const planar_sample &s0 = run[k], &s1 = run[k + 1];
      const cvc::nav::diff::dyn_state<double> m = dyn_tick(p, lim, s0, s1, nsub, a_shadow);
      if (why_skipped(s0, s1) != kept)
        continue;
      const double ex = m.vx - s1.vx, ey = m.vy - s1.vy, er = L * (m.r - s1.r);
      sum += ex * ex + ey * ey + er * er;
      ++n;
    }
  }
  if (ticks)
    *ticks = n;
  return n ? sum / static_cast<double>(n) : 0.0;
}

dyn_bike_fit fit_dyn_bike(const std::vector<std::vector<planar_sample>> &runs, double dt,
                          const vehicle_desc &d, double mu, int nsub, int sweeps) {
  const cvc::nav::dyn_bike_si base = nav_plant::dyn_bike_si_of(d, mu);
  dyn_bike_fit f;
  f.si = base;
  f.cost_default = dyn_bike_cost(runs, dt, base, d.max_steer, nsub, &f.ticks);
  // The candidates, each a value or a factor on the spec-sheet vehicle's.
  struct knob {
    double *value;
    std::vector<double> grid;
  };
  double drive_lag = base.drive_lag, steer_lag = base.steer_lag, bf = 1.0, br = 1.0, iz = 1.0;
  double resist = 1.0;
  const std::vector<double> factors = {0.1, 0.15, 0.2, 0.25, 0.3,  0.4,  0.55,
                                       0.7, 0.85, 1.0, 1.2,  1.45, 1.75, 2.1};
  const std::vector<knob> knobs = {{&drive_lag, {0.0, 0.02, 0.04, 0.07, 0.1, 0.15, 0.2, 0.3}},
                                   {&steer_lag, {0.0, 0.02, 0.04, 0.07, 0.1}},
                                   {&resist, {0.0, 0.25, 0.5, 0.75, 1.0}},
                                   {&bf, factors},
                                   {&br, factors},
                                   {&iz, factors}};
  const auto make = [&]() {
    cvc::nav::dyn_bike_si si = base;
    si.drive_lag = drive_lag;
    si.steer_lag = steer_lag;
    si.tire_B_front = base.tire_B_front * bf;
    si.tire_B_rear = base.tire_B_rear * br;
    si.yaw_inertia = base.yaw_inertia * iz;
    si.drag = base.drag * resist;
    si.rolling_resistance = base.rolling_resistance * resist;
    return si;
  };
  double best = f.cost_default;
  // Coordinate descent: each knob over its grid with the others held, `sweeps` times.
  for (int sw = 0; sw < sweeps; ++sw)
    for (const knob &k : knobs) {
      double keep = *k.value;
      for (const double v : k.grid) {
        *k.value = v;
        const double c = dyn_bike_cost(runs, dt, make(), d.max_steer, nsub, nullptr);
        if (c < best) {
          best = c;
          keep = v;
        }
      }
      *k.value = keep;
    }
  f.si = make();
  f.cost_fit = best;
  f.drive_lag = drive_lag;
  f.steer_lag = steer_lag;
  f.tire_b_front_factor = bf;
  f.tire_b_rear_factor = br;
  f.yaw_inertia_factor = iz;
  f.resistance_factor = resist;
  return f;
}

namespace {

// The power model's six features at each tick of `runs`, in SI, each already times dt, so a
// tick's energy is sum_j theta_j x_j with theta = (k_pos, k_neg, p_idle, c_v, c_v3, c_lat): the
// smooth positive and negative parts of the drive power m a vx, the tick itself, the smooth speed,
// its cube, and cornering's m (vx r)^2 |vx|. `a` is the drive force per unit mass the dynamic
// vehicle `dyn` needs to make the motion the plant made: the measured acceleration less its
// resistances (drag, rolling and grade), as diff::dyn_plant integrates vx' = a + F_res / m.
constexpr int kPowerTerms = 6;

struct power_ticks {
  std::vector<std::array<double, kPowerTerms>> x;
  std::vector<double> y;           // the ledger's energy, J
  std::vector<std::size_t> run_of; // each tick's run
  std::vector<power_run> runs;     // measured_J, distance_m and seconds filled
  double width = 0.0;              // W, the smooth parts' width
};

constexpr double kPowerSpeedEps = 1e-3; // m/s: the smooth speed's width (diff::power_params)

power_ticks power_features(const std::vector<std::vector<planar_sample>> &runs, double dt,
                           const vehicle_desc &d, const cvc::nav::dyn_bike_si *dyn, double width) {
  const cvc::nav::dyn_bike_si si = dyn ? *dyn : nav_plant::dyn_bike_si_of(d);
  power_ticks t;
  double vmax = 1.0;
  for (const auto &run : runs)
    for (const planar_sample &s : run)
      vmax = std::max(vmax, std::fabs(s.vx));
  // 1e-3 of the full positive drive power, as the trainer's default (diff::power_full).
  t.width = width > 0.0 ? width : 1e-3 * si.mass * d.max_accel * vmax;
  t.runs.resize(runs.size());
  for (std::size_t r = 0; r < runs.size(); ++r) {
    const std::vector<planar_sample> &run = runs[r];
    for (std::size_t k = 1; k < run.size(); ++k) {
      const planar_sample &s0 = run[k - 1], &s = run[k];
      const double v = 0.5 * (s0.vx + s.vx);
      const double sg = v > 0.0 ? 1.0 : (v < 0.0 ? -1.0 : 0.0);
      const double f_res = -si.drag * v * std::fabs(v) -
                           si.rolling_resistance * si.mass * si.gravity * sg -
                           si.mass * si.gravity * 0.5 * (s0.grade + s.grade);
      const double a = s.a_lon - f_res / si.mass;
      const double P = si.mass * a * s.vx;
      float dd;
      const double pos =
          cvc::nav::diff::power_softplus(static_cast<float>(P), static_cast<float>(t.width), dd);
      const double neg =
          cvc::nav::diff::power_softplus(static_cast<float>(-P), static_cast<float>(t.width), dd);
      const double speed = std::sqrt(s.vx * s.vx + kPowerSpeedEps * kPowerSpeedEps);
      const double lat = si.mass * s.vx * s.vx * s.r * s.r * speed;
      t.x.push_back({pos * dt, neg * dt, dt, speed * dt, speed * speed * speed * dt, lat * dt});
      t.y.push_back(s.energy);
      t.run_of.push_back(r);
      t.runs[r].measured_J += s.energy;
      t.runs[r].distance_m += std::hypot(s.x - s0.x, s.y - s0.y);
      t.runs[r].seconds += dt;
    }
  }
  return t;
}

void report_power(power_ticks &t, power_fit &f) {
  const auto &si = f.si;
  const double th[kPowerTerms] = {si.k_pos, si.k_neg, si.p_idle, si.c_v, si.c_v3, si.c_lat};
  double ybar = 0.0;
  for (double y : t.y)
    ybar += y;
  ybar = t.y.empty() ? 0.0 : ybar / static_cast<double>(t.y.size());
  double ss_res = 0.0, ss_tot = 0.0;
  for (std::size_t i = 0; i < t.y.size(); ++i) {
    double e = 0.0;
    for (int j = 0; j < kPowerTerms; ++j)
      e += th[j] * t.x[i][j];
    t.runs[t.run_of[i]].model_J += e;
    ss_res += (e - t.y[i]) * (e - t.y[i]);
    ss_tot += (t.y[i] - ybar) * (t.y[i] - ybar);
  }
  f.ticks = t.y.size();
  f.r2 = ss_tot > 0.0 ? 1.0 - ss_res / ss_tot : 0.0;
  double sm = 0.0, sy = 0.0;
  for (const power_run &r : t.runs) {
    sm += r.model_J;
    sy += r.measured_J;
  }
  f.total_error = sy != 0.0 ? (sm - sy) / sy : 0.0;
  const double mean = t.runs.empty() ? 0.0 : sy / static_cast<double>(t.runs.size());
  f.worst_run_error = 0.0;
  for (const power_run &r : t.runs)
    if (std::fabs(r.measured_J) >= 0.01 * std::fabs(mean) && r.measured_J != 0.0)
      f.worst_run_error = std::max(f.worst_run_error,
                                   std::fabs(r.model_J - r.measured_J) / std::fabs(r.measured_J));
  f.runs = t.runs;
}

// Least squares over the coefficients in `free` (the others held at 0), by the normal equations
// on unit-norm columns. Returns false if they are singular.
bool power_lsq(const power_ticks &t, const bool free[kPowerTerms], double th[kPowerTerms]) {
  int idx[kPowerTerms], n = 0;
  for (int j = 0; j < kPowerTerms; ++j)
    if (free[j])
      idx[n++] = j;
  double scale[kPowerTerms] = {};
  for (std::size_t i = 0; i < t.y.size(); ++i)
    for (int a = 0; a < n; ++a)
      scale[a] += t.x[i][idx[a]] * t.x[i][idx[a]];
  for (int a = 0; a < n; ++a) {
    if (!(scale[a] > 0.0))
      return false;
    scale[a] = 1.0 / std::sqrt(scale[a]);
  }
  double A[kPowerTerms][kPowerTerms + 1] = {};
  for (std::size_t i = 0; i < t.y.size(); ++i)
    for (int a = 0; a < n; ++a) {
      const double xa = t.x[i][idx[a]] * scale[a];
      for (int b = 0; b < n; ++b)
        A[a][b] += xa * t.x[i][idx[b]] * scale[b];
      A[a][n] += xa * t.y[i];
    }
  for (int c = 0; c < n; ++c) { // Gauss-Jordan with partial pivoting
    int piv = c;
    for (int r = c + 1; r < n; ++r)
      if (std::fabs(A[r][c]) > std::fabs(A[piv][c]))
        piv = r;
    if (std::fabs(A[piv][c]) < 1e-12)
      return false;
    for (int k = 0; k <= n; ++k)
      std::swap(A[c][k], A[piv][k]);
    for (int r = 0; r < n; ++r) {
      if (r == c)
        continue;
      const double m = A[r][c] / A[c][c];
      for (int k = c; k <= n; ++k)
        A[r][k] -= m * A[c][k];
    }
  }
  for (int j = 0; j < kPowerTerms; ++j)
    th[j] = 0.0;
  for (int a = 0; a < n; ++a)
    th[idx[a]] = A[a][n] / A[a][a] * scale[a];
  return true;
}

} // namespace

power_fit fit_power(const std::vector<std::vector<planar_sample>> &runs, double dt,
                    const vehicle_desc &d, const cvc::nav::dyn_bike_si *dyn) {
  power_ticks t = power_features(runs, dt, d, dyn, 0.0);
  // Non-negative least squares by every subset of the coefficients (k_neg <= 0, the others >= 0):
  // the feasible solution with the least squared error.
  double best[kPowerTerms] = {}, best_sse = std::numeric_limits<double>::infinity();
  for (int mask = 1; mask < (1 << kPowerTerms); ++mask) {
    bool free[kPowerTerms];
    for (int j = 0; j < kPowerTerms; ++j)
      free[j] = (mask >> j) & 1;
    double th[kPowerTerms];
    if (!power_lsq(t, free, th))
      continue;
    if (th[0] < 0.0 || th[1] > 0.0 || th[2] < 0.0 || th[3] < 0.0 || th[4] < 0.0 || th[5] < 0.0)
      continue;
    double sse = 0.0;
    for (std::size_t i = 0; i < t.y.size(); ++i) {
      double e = -t.y[i];
      for (int j = 0; j < kPowerTerms; ++j)
        e += th[j] * t.x[i][j];
      sse += e * e;
    }
    if (sse < best_sse) {
      best_sse = sse;
      std::copy(th, th + kPowerTerms, best);
    }
  }
  power_fit f;
  f.si.k_pos = best[0];
  f.si.k_neg = best[1];
  f.si.p_idle = best[2];
  f.si.c_v = best[3];
  f.si.c_v3 = best[4];
  f.si.c_lat = best[5];
  f.si.width = t.width;
  report_power(t, f);
  return f;
}

power_fit evaluate_power(const std::vector<std::vector<planar_sample>> &runs, double dt,
                         const vehicle_desc &d, const cvc::nav::dyn_power_si &si,
                         const cvc::nav::dyn_bike_si *dyn) {
  power_ticks t = power_features(runs, dt, d, dyn, si.width);
  power_fit f;
  f.si = si;
  report_power(t, f);
  return f;
}

flight_set cut_flights(const std::vector<std::vector<planar_sample>> &runs, std::size_t rows,
                       std::size_t stride, double min_speed, steer_input steer) {
  if (rows < 2 || stride < 1)
    throw std::invalid_argument("cvc::physics::cut_flights: needs rows >= 2 and stride >= 1");
  flight_set f;
  f.rows = rows;
  for (std::size_t a = 0; a < runs.size(); ++a) {
    const std::vector<planar_sample> &run = runs[a];
    for (std::size_t s = 0; s + rows <= run.size(); s += stride) {
      bool contact = false, moving = false;
      double peak = 0.0;
      for (std::size_t k = s; k < s + rows; ++k) {
        contact = contact || !run[k].grounded ||
                  (k > s &&
                   (run[k].contacts > 0 || std::hypot(run[k].a_lat, run[k].a_lon) > kImpactG * kG));
        moving = moving || std::hypot(run[k].vx, run[k].vy) > min_speed;
        if (k > s)
          peak = std::max(peak, std::fabs(run[k].a_lat) / kG);
      }
      if (contact || !moving) {
        ++f.dropped;
        continue;
      }
      f.peak_lateral_g = std::max(f.peak_lateral_g, peak);
      for (std::size_t k = s; k < s + rows; ++k) {
        const planar_sample &q = run[k];
        const double u_steer = steer == steer_input::command
                                   ? q.steer_cmd
                                   : (k > 0 ? 0.5 * (run[k - 1].steer + q.steer) : q.steer);
        const double row[kPlanarWidth] = {q.x, q.y, q.psi, q.vx, q.vy, q.r, q.drive, u_steer};
        f.data.insert(f.data.end(), row, row + kPlanarWidth);
      }
      f.agent.push_back(static_cast<int>(a));
      f.first.push_back(static_cast<int>(s));
      ++f.flights;
    }
  }
  return f;
}

void write_npy(const std::string &path, const std::vector<double> &data,
               const std::vector<std::size_t> &shape) {
  std::size_t n = 1;
  std::string dims;
  for (std::size_t k = 0; k < shape.size(); ++k) {
    n *= shape[k];
    dims += std::to_string(shape[k]) + (shape.size() == 1 || k + 1 < shape.size() ? "," : "");
    if (k + 1 < shape.size())
      dims += " ";
  }
  if (n != data.size())
    throw std::runtime_error("cvc::physics::write_npy: the shape does not match the data");
  std::string header = "{'descr': '<f8', 'fortran_order': False, 'shape': (" + dims + "), }";
  // Pad with spaces to a multiple of 64 bytes with the 10-byte preamble and the newline.
  while ((10 + header.size() + 1) % 64 != 0)
    header += ' ';
  header += '\n';
  std::ofstream out(path, std::ios::binary);
  if (!out)
    throw std::runtime_error("cvc::physics::write_npy: cannot write " + path);
  const unsigned char magic[8] = {0x93, 'N', 'U', 'M', 'P', 'Y', 1, 0};
  out.write(reinterpret_cast<const char *>(magic), 8);
  const std::uint16_t hl = static_cast<std::uint16_t>(header.size());
  const unsigned char len[2] = {static_cast<unsigned char>(hl & 0xff),
                                static_cast<unsigned char>(hl >> 8)};
  out.write(reinterpret_cast<const char *>(len), 2);
  out.write(header.data(), static_cast<std::streamsize>(header.size()));
  // Little-endian doubles: every libcvc target is little-endian.
  out.write(reinterpret_cast<const char *>(data.data()),
            static_cast<std::streamsize>(data.size() * sizeof(double)));
  if (!out)
    throw std::runtime_error("cvc::physics::write_npy: short write to " + path);
}

} // namespace physics
} // namespace cvc
