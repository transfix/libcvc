/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// physics_dyn_contact_test: the dynamic plant's compliant contact (cvc/nav/detail/dyn_contact.h,
// dyn_bike_plant_options::contact) against a field and between vehicles, in port-Hamiltonian
// form:
//   - the barrier's wrench is minus its energy's gradient in the pose, the damper only dissipates,
//     and a pair of vehicles keeps its momentum;
//   - the live plant's books close with the contact lines, to second order per step, for a wall
//     impact and a vehicle-to-vehicle bump (the pair's exchange cancelling);
//   - against the Jolt plant: a Humvee into a wall, head on and at 30 degrees, and into another
//     parked Humvee, with the speed lost, the rebound and the heading change reported;
//   - nav's signed-distance grid as the field (dyn_bike_plant::sdf_field).

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cvc/nav/coef_mlp.h>
#include <cvc/nav/drive.h>
#include <cvc/nav/sim_world.h>
#include <cvc/nav/vehicle_plant.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/dyn_bike_plant.h>
#include <cvc/physics/vehicle.h>
#include <cvc/physics/world.h>
#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace cvc::physics;
namespace diff = cvc::nav::diff;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A wall whose west face is at x = x0: free to its west.
struct wall_field {
  double x0 = 0.0;
  void at(double x, double, double &phi, double &gx, double &gy) const {
    phi = x0 - x;
    gx = -1.0;
    gy = 0.0;
  }
};

// A round pillar of radius rad at (cx, cy).
struct pillar_field {
  double cx = 0.0, cy = 0.0, rad = 1.0;
  void at(double x, double y, double &phi, double &gx, double &gy) const {
    const double dx = x - cx, dy = y - cy, d = std::sqrt(dx * dx + dy * dy);
    phi = d - rad;
    gx = dx / d;
    gy = dy / d;
  }
};

diff::dyn_footprint humvee_footprint() {
  diff::dyn_footprint f;
  f.front = 2.3;
  f.rear = 2.5;
  f.half_width = 1.1;
  return f;
}

// The barrier's energy of body b's points against a field, and of a pair.
template <class F>
double field_energy(const diff::dyn_contact_params &p, const std::vector<diff::dyn_point> &pts,
                    const F &field, const diff::dyn_body &b) {
  diff::dyn_contact c;
  diff::dyn_contact_field(p, pts, field, b, c);
  return c.energy;
}

double pair_energy(const diff::dyn_contact_params &p, const std::vector<diff::dyn_point> &pts,
                   const diff::dyn_footprint &f, const diff::dyn_body &a, const diff::dyn_body &b) {
  diff::dyn_contact ca, cb;
  diff::dyn_contact_pair(p, pts, f, a, pts, f, b, ca, cb);
  return ca.energy + cb.energy;
}

// -dE/d(pose) as a body wrench: the world force -(dE/dx, dE/dy) in body axes, and -dE/dpsi.
template <class E> void minus_gradient(const E &energy, const diff::dyn_body &b, double W[3]) {
  const double h = 1e-6;
  diff::dyn_body p = b, m = b;
  p.x += h;
  m.x -= h;
  const double fx = -(energy(p) - energy(m)) / (2.0 * h);
  p = m = b;
  p.y += h;
  m.y -= h;
  const double fy = -(energy(p) - energy(m)) / (2.0 * h);
  p = m = b;
  p.psi += h;
  m.psi -= h;
  W[2] = -(energy(p) - energy(m)) / (2.0 * h);
  const double c = std::cos(b.psi), s = std::sin(b.psi);
  W[0] = c * fx + s * fy;
  W[1] = -s * fx + c * fy;
}

} // namespace

// The barrier's wrench is minus its energy's gradient in the pose (central differences), against a
// wall and a pillar, at random poses in the skin; the damper's power is minus its dissipation.
TEST(DynContact, TheBarrierIsItsEnergysGradientAndTheDamperDissipates) {
  diff::dyn_contact_params p;
  const diff::dyn_footprint f = humvee_footprint();
  const std::vector<diff::dyn_point> pts = diff::dyn_footprint_points(f, p);
  const wall_field wall{0.0};
  const pillar_field pillar{0.0, 0.0, 1.5};
  std::mt19937 rng(3);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst = 0.0;
  int touching = 0;
  for (int k = 0; k < 200; ++k) {
    diff::dyn_body b;
    b.psi = kPi * u(rng);
    b.vx = 5.0 * u(rng);
    b.vy = u(rng);
    b.r = 0.5 * u(rng);
    const bool at_wall = k % 2 == 0;
    // Within the skin of the obstacle with some point: 0.6 m to 2.6 m off it.
    if (at_wall) {
      b.x = -(1.6 + 1.0 * u(rng));
      b.y = 3.0 * u(rng);
    } else {
      const double ang = kPi * u(rng), d = 1.5 + 1.6 + 0.8 * u(rng);
      b.x = d * std::cos(ang);
      b.y = d * std::sin(ang);
    }
    diff::dyn_contact c;
    if (at_wall)
      diff::dyn_contact_field(p, pts, wall, b, c);
    else
      diff::dyn_contact_field(p, pts, pillar, b, c);
    if (!c.touching)
      continue;
    ++touching;
    double G[3];
    if (at_wall)
      minus_gradient([&](const diff::dyn_body &q) { return field_energy(p, pts, wall, q); }, b, G);
    else
      minus_gradient([&](const diff::dyn_body &q) { return field_energy(p, pts, pillar, q); }, b,
                     G);
    const double scale = 1.0 + std::fabs(c.Wc[0]) + std::fabs(c.Wc[1]) + std::fabs(c.Wc[2]);
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::fabs(c.Wc[i] - G[i]) / scale);
      EXPECT_NEAR(c.Wc[i], G[i], 1e-5 * scale) << (at_wall ? "wall" : "pillar") << ", axis " << i;
    }
    // The damper: power -v^T R_c v.
    const double power = c.Wd[0] * b.vx + c.Wd[1] * b.vy + c.Wd[2] * b.r;
    EXPECT_GE(c.damping, 0.0);
    EXPECT_NEAR(power, -c.damping, 1e-9 * (1.0 + c.damping));
  }
  std::printf("  %d poses in the skin: the barrier's wrench within %.2e of -dPhi/dpose\n", touching,
              worst);
  EXPECT_GT(touching, 50);
}

// Between two vehicles the forces are equal and opposite: the pair's momentum and its angular
// momentum about any point are kept; each vehicle's barrier wrench is minus the pair's energy's
// gradient in its pose; and the damper only dissipates, reading the relative velocity.
TEST(DynContact, APairKeepsItsMomentum) {
  diff::dyn_contact_params p;
  const diff::dyn_footprint f = humvee_footprint();
  const std::vector<diff::dyn_point> pts = diff::dyn_footprint_points(f, p);
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  double worst = 0.0;
  int touching = 0;
  for (int k = 0; k < 2000; ++k) {
    diff::dyn_body a, b;
    a.psi = kPi * u(rng);
    b.psi = kPi * u(rng);
    b.x = 4.5 * u(rng);
    b.y = 4.5 * u(rng);
    a.vx = 4.0 * u(rng);
    a.r = 0.3 * u(rng);
    b.vx = 4.0 * u(rng);
    b.vy = u(rng);
    // Overlaps shallower than the corner radius, where the outline's distance is C1.
    const auto deepest = [&](const diff::dyn_body &x, const diff::dyn_body &y) {
      double least = 1e9;
      const double c = std::cos(x.psi), s = std::sin(x.psi);
      const double cy = std::cos(y.psi), sy = std::sin(y.psi);
      for (const diff::dyn_point &l : pts) {
        const double ox = x.x + c * l.x - s * l.y - y.x, oy = x.y + s * l.x + c * l.y - y.y;
        double gx, gy;
        least = std::min(least, diff::dyn_rounded_box_sdf(f, p.rounding, cy * ox + sy * oy,
                                                          -sy * ox + cy * oy, gx, gy));
      }
      return least;
    };
    if (std::min(deepest(a, b), deepest(b, a)) < -0.8 * p.rounding)
      continue;
    diff::dyn_contact ca, cb;
    diff::dyn_contact_pair(p, pts, f, a, pts, f, b, ca, cb);
    if (!ca.touching)
      continue;
    ++touching;
    // World forces, and moments about the origin.
    const auto world = [](const diff::dyn_body &q, const double W[3], double &Fx, double &Fy,
                          double &M0) {
      const double c = std::cos(q.psi), s = std::sin(q.psi);
      Fx = c * W[0] - s * W[1];
      Fy = s * W[0] + c * W[1];
      M0 = W[2] + q.x * Fy - q.y * Fx;
    };
    double Wa[3], Wb[3], fax, fay, ma, fbx, fby, mb;
    ca.wrench(Wa);
    cb.wrench(Wb);
    world(a, Wa, fax, fay, ma);
    world(b, Wb, fbx, fby, mb);
    const double scale = 1.0 + std::hypot(fax, fay);
    EXPECT_NEAR(fax + fbx, 0.0, 1e-9 * scale);
    EXPECT_NEAR(fay + fby, 0.0, 1e-9 * scale);
    EXPECT_NEAR(ma + mb, 0.0, 1e-9 * scale * (1.0 + std::hypot(b.x, b.y)));
    double G[3];
    minus_gradient([&](const diff::dyn_body &q) { return pair_energy(p, pts, f, q, b); }, a, G);
    const double sc = 1.0 + std::fabs(ca.Wc[0]) + std::fabs(ca.Wc[1]) + std::fabs(ca.Wc[2]);
    for (int i = 0; i < 3; ++i) {
      worst = std::max(worst, std::fabs(ca.Wc[i] - G[i]) / sc);
      EXPECT_NEAR(ca.Wc[i], G[i], 1e-5 * sc) << "axis " << i;
    }
    const double power = ca.Wd[0] * a.vx + ca.Wd[1] * a.vy + ca.Wd[2] * a.r + cb.Wd[0] * b.vx +
                         cb.Wd[1] * b.vy + cb.Wd[2] * b.r;
    EXPECT_NEAR(power, -(ca.damping + cb.damping), 1e-9 * (1.0 + ca.damping + cb.damping));
  }
  std::printf("  %d touching pairs: forces and moments balance; the barrier within %.2e of "
              "-dPhi/dpose\n",
              touching, worst);
  EXPECT_GT(touching, 50);
}

namespace {

#define REQUIRE_JOLT()                                                                             \
  if (!backend_available(backend::jolt))                                                           \
  GTEST_SKIP() << "this libcvc was built without Jolt"

cvc::world_units::normalized_frame metres() {
  cvc::world_units::normalized_frame f;
  f.scale = 1.0;
  return f;
}

// kinetic now - kinetic at attach - the books: zero to rounding, by construction.
double identity_gap(const dyn_bike_plant &plant, int i) {
  const dyn_bike_books &b = plant.books(i);
  return b.kinetic - plant.kinetic_at_attach(i) -
         (b.drive - b.brake - b.dissipated - b.slip + b.floor + b.grade + b.limiter + b.contact -
          b.contact_stored - b.contact_dissipated + b.contact_exchange + b.residual);
}

// One impact on the dynamic plant: what it measured.
struct impact {
  double v_before = 0.0, v_after = 0.0;   // the striking vehicle's speed (world, m/s)
  double vn_before = 0.0, vn_after = 0.0; // along +x (the approach)
  double heading_change = 0.0;            // rad
  double other_after = 0.0;               // the struck vehicle's speed after (a pair)
  double mean_abs_residual = 0.0;         // per substep, J
  double flow = 0.0, residual = 0.0, gap = 0.0, exchange = 0.0, stored = 0.0, dissipated = 0.0;
};

// The Humvee at rest heading `heading`, its rear axle at the origin (and, with `pair`, a second
// one parked 18 m ahead along x), driven raw (no driver model) at 2 m/s^2 until it reaches
// `speed`, then coasting: into a wall 25 m ahead along x, or into the parked one. Measured 1 m
// before the contact and 1 s after that.
impact drive_into(double speed, double heading, bool pair, double substep, double wall_x = 25.0,
                  double friction = -1.0) {
  dyn_bike_plant_options o;
  o.driver = false;
  o.substep = substep;
  o.contact = true;
  if (friction >= 0.0)
    o.contact_params.friction = friction;
  o.contact_vehicles = pair;
  if (!pair) {
    const wall_field w{wall_x};
    o.contact_field = [w](double x, double y, double &phi, double &gx, double &gy) {
      w.at(x, y, phi, gx, gy);
    };
  }
  dyn_bike_plant plant({vehicle_desc::humvee()}, o);
  cvc::nav::plant_state starts[2];
  starts[0].heading = static_cast<float>(heading);
  starts[1].x = 18.0f;
  plant.attach(pair ? 2 : 1, starts, metres(), 0.02);
  impact m;
  bool coasting = false, measured = false;
  double t_before = -1.0, psi_before = 0.0, abs_res = 0.0;
  long substeps = 0;
  for (int t = 0; t < 600; ++t) {
    const diff::dyn_state<double> &s = plant.body(0);
    if (!coasting && s.vx >= speed)
      coasting = true;
    plant.command(0, 0.0f, coasting ? 0.0f : 2.0f);
    if (pair)
      plant.command(1, 0.0f, 0.0f);
    plant.advance();
    abs_res += std::fabs(plant.last_tick(0).residual);
    if (pair)
      abs_res += std::fabs(plant.last_tick(1).residual);
    substeps += plant.substeps();
    const diff::dyn_state<double> &b = plant.body(0);
    const double c = std::cos(b.psi), sn = std::sin(b.psi);
    const double cgx = b.x + plant.params(0).lr * c, cgy = b.y + plant.params(0).lr * sn;
    // The front corner nearest the obstacle, 1 m off it.
    const diff::dyn_footprint &f = plant.footprint(0);
    double ahead = -1e9;
    for (const double side : {-1.0, 1.0})
      ahead = std::max(ahead, cgx + f.front * c - side * f.half_width * sn);
    // The wall, or the parked vehicle's rear (it heads along x).
    const double obstacle =
        pair ? plant.body(1).x + plant.params(1).lr - plant.footprint(1).rear : wall_x;
    const double vwx = c * b.vx - sn * b.vy, vwy = sn * b.vx + c * b.vy;
    const double time = 0.02 * (t + 1);
    if (t_before < 0.0 && ahead > obstacle - 1.0) {
      t_before = time;
      m.v_before = std::hypot(vwx, vwy);
      m.vn_before = vwx;
      psi_before = b.psi;
    }
    if (t_before >= 0.0 && !measured && time >= t_before + 1.0) {
      measured = true;
      m.v_after = std::hypot(vwx, vwy);
      m.vn_after = vwx;
      m.heading_change = b.psi - psi_before;
      if (pair) {
        const diff::dyn_state<double> &q = plant.body(1);
        m.other_after = std::hypot(q.vx, q.vy);
      }
    }
    (void)cgy;
  }
  m.mean_abs_residual = abs_res / substeps;
  for (int i = 0; i < (pair ? 2 : 1); ++i) {
    const dyn_bike_books &b = plant.books(i);
    m.flow += b.flow();
    m.residual += b.residual;
    m.gap += identity_gap(plant, i);
    m.exchange += b.contact_exchange;
    m.stored += b.contact_stored;
    m.dissipated += b.contact_dissipated;
  }
  return m;
}

} // namespace

// The live plant's books with the contact lines close by construction, and the residual, the
// integrator's, is second order per step: a wall impact head on and at 30 degrees, and a bump into
// a parked vehicle, whose exchange line cancels over the pair.
TEST(DynContactPlant, TheBooksCloseToSecondOrder) {
  struct scenario {
    const char *name;
    double heading;
    bool pair;
  };
  for (const scenario &sc :
       {scenario{"wall, head on", 0.0, false}, scenario{"wall, 30 degrees", kPi / 6.0, false},
        scenario{"a parked Humvee", 0.0, true}}) {
    double prev = 0.0, prev_exchange = 0.0;
    for (const double sub : {0.01, 0.005, 0.0025}) {
      const impact m = drive_into(5.0, sc.heading, sc.pair, sub);
      std::printf("  %s, substep %.4f s: stored %.4g J (net), dissipated %.4g J, exchange %.3g J; "
                  "mean |residual| per step %.3e J, |sum| / flow %.2e\n",
                  sc.name, sub, m.stored, m.dissipated, m.exchange, m.mean_abs_residual,
                  std::fabs(m.residual) / m.flow);
      EXPECT_NEAR(m.gap, 0.0, 1e-8 * m.flow);
      EXPECT_GT(m.dissipated, 0.0) << sc.name;
      EXPECT_LT(std::fabs(m.residual) / m.flow, 5e-3) << sc.name;
      if (sc.pair) {
        // What the vehicles passed each other sums, over the pair, to the barrier's integration
        // error: second order per step too.
        EXPECT_LT(std::fabs(m.exchange), 2e-2 * m.flow);
        if (prev_exchange > 0.0)
          EXPECT_GT(prev_exchange / std::fabs(m.exchange), 2.5) << "substep " << sub;
        prev_exchange = std::fabs(m.exchange);
      }
      if (prev > 0.0) {
        const double ratio = prev / m.mean_abs_residual;
        EXPECT_GT(ratio, 3.0) << sc.name << ", substep " << sub;
        EXPECT_LT(ratio, 5.5) << sc.name << ", substep " << sub;
      }
      prev = m.mean_abs_residual;
    }
  }
}

namespace {

// The same impact on the Jolt plant: a Humvee on flat ground in a Jolt world, its rear axle at the
// origin heading `heading`, brought to `speed` by the driver model and then coasting in neutral
// into a wall 25 m ahead along x (or into a second Humvee parked 18 m ahead, in neutral too),
// measured as drive_into measures the dynamic plant: 1 m before the contact and 1 s after that.
impact jolt_into(double speed, double heading, bool pair, double wall_x = 25.0) {
  world w;
  body_desc g;
  g.shape = box{{500.0, 500.0, 0.5}};
  g.motion = motion_type::static_;
  g.position = {0.0, 0.0, -0.5};
  w.add_body(g);
  if (!pair) {
    body_desc wall;
    wall.shape = box{{0.5, 40.0, 3.0}};
    wall.motion = motion_type::static_;
    wall.position = {wall_x + 0.5, 0.0, 3.0};
    w.add_body(wall);
  }
  const vehicle_desc d = vehicle_desc::humvee();
  double rear_x = 0.0;
  for (const axle_desc &a : d.axles)
    rear_x = std::min(rear_x, a.x);
  const auto spawn = [&](double x, double h) {
    const quat q{std::cos(0.5 * h), 0.0, 0.0, std::sin(0.5 * h)};
    return w.add_vehicle(d, {x - rear_x * std::cos(h), -rear_x * std::sin(h), 1.1}, q);
  };
  const vehicle_id a = spawn(0.0, heading);
  vehicle_id b{};
  if (pair)
    b = spawn(18.0, 0.0);
  vehicle_controls idle;
  idle.neutral = true;
  w.set_controls(a, idle);
  if (pair)
    w.set_controls(b, idle);
  w.step(100); // settle on the springs
  const double dt = w.config().fixed_dt;
  // The geometry the dynamic plant reads: the chassis box's front and its half width.
  const double front = d.half_extents.x, half_width = d.half_extents.y;
  impact m;
  bool coasting = false, measured = false;
  double t_before = -1.0, psi_before = 0.0;
  for (int t = 0; t < static_cast<int>(12.0 / dt); ++t) {
    const vehicle_state vs = w.vehicle_state(a);
    const body_state &s = vs.body;
    const double c = 1.0 - 2.0 * (s.rotation.z * s.rotation.z + s.rotation.y * s.rotation.y);
    const double sn = 2.0 * (s.rotation.w * s.rotation.z + s.rotation.x * s.rotation.y);
    const double psi = std::atan2(sn, c);
    const double speed_now = std::hypot(s.linear_velocity.x, s.linear_velocity.y);
    if (!coasting && speed_now >= speed) {
      coasting = true;
      w.set_controls(a, idle);
    }
    if (!coasting) {
      drive_command dc;
      dc.accel = 2.0;
      w.drive(a, dc);
    }
    w.step();
    const double time = dt * (t + 1);
    double ahead = -1e9;
    for (const double side : {-1.0, 1.0})
      ahead =
          std::max(ahead, s.position.x + front * std::cos(psi) - side * half_width * std::sin(psi));
    double obstacle = wall_x;
    if (pair) {
      const body_state &q = w.vehicle_state(b).body;
      obstacle = q.position.x - d.half_extents.x;
    }
    if (t_before < 0.0 && ahead > obstacle - 1.0) {
      t_before = time;
      m.v_before = speed_now;
      m.vn_before = s.linear_velocity.x;
      psi_before = psi;
    }
    if (t_before >= 0.0 && !measured && time >= t_before + 1.0) {
      measured = true;
      m.v_after = speed_now;
      m.vn_after = s.linear_velocity.x;
      m.heading_change = psi - psi_before;
      if (pair) {
        const body_state &q = w.vehicle_state(b).body;
        m.other_after = std::hypot(q.linear_velocity.x, q.linear_velocity.y);
      }
    }
  }
  return m;
}

} // namespace

// The comparison with the Jolt plant: the compliant contact (barrier and damper over the
// footprint) against Jolt's rigid chassis box, for a Humvee into a wall head on and at 30 degrees
// and into a parked Humvee, at 5 m/s. Reported: the speed each loses, the rebound (the normal
// velocity after over before) and the heading change. Gated loosely: both stop the vehicle at the
// wall (no tunnelling), turn it the same way at 30 degrees, and push the parked one forward.
TEST(DynContactPlant, ComparesWithJoltOnImpacts) {
  REQUIRE_JOLT();
  struct scenario {
    const char *name;
    double heading;
    bool pair;
  };
  std::printf("  %-18s %-6s %9s %9s %9s %10s %12s\n", "impact", "plant", "v before", "v after",
              "rebound", "heading", "other after");
  for (const scenario &sc :
       {scenario{"wall, head on", 0.0, false}, scenario{"wall, 30 degrees", kPi / 6.0, false},
        scenario{"a parked Humvee", 0.0, true}}) {
    const impact dyn = drive_into(5.0, sc.heading, sc.pair, 0.01);
    const impact dyn_grippy = drive_into(5.0, sc.heading, sc.pair, 0.01, 25.0, 0.6);
    const impact jolt = jolt_into(5.0, sc.heading, sc.pair);
    for (const auto &row : {std::make_pair("dyn", dyn), std::make_pair("dyn .6", dyn_grippy),
                            std::make_pair("Jolt", jolt)}) {
      const impact &m = row.second;
      std::printf("  %-18s %-6s %8.2f %9.2f %9.2f %8.1f deg %9.2f m/s\n", sc.name, row.first,
                  m.v_before, m.v_after, m.vn_before != 0.0 ? -m.vn_after / m.vn_before : 0.0,
                  m.heading_change * 180.0 / kPi, m.other_after);
    }
    EXPECT_GT(dyn.v_before, 3.0) << sc.name;
    EXPECT_GT(jolt.v_before, 3.0) << sc.name;
    if (!sc.pair) {
      EXPECT_LT(dyn.vn_after, 0.5 * dyn.vn_before) << sc.name << ": the dynamic plant went through";
      EXPECT_LT(jolt.vn_after, 0.5 * jolt.vn_before) << sc.name;
    } else {
      EXPECT_GT(dyn.other_after, 0.5) << "the parked Humvee did not move";
      EXPECT_GT(jolt.other_after, 0.5);
    }
    // At 30 degrees the yaw is a balance of the barrier's push (turning the vehicle along the wall)
    // and the friction's (turning it into the wall): reported, with the friction at 0.6 beside
    // the default's, and held only to a few degrees.
    if (sc.heading != 0.0) {
      EXPECT_LT(std::fabs(dyn.heading_change), 0.2) << sc.name;
      EXPECT_LT(std::fabs(jolt.heading_change), 0.2) << sc.name;
    }
  }
}

// nav's signed-distance grid as the contact field (dyn_bike_plant::sdf_field): a grid laid as
// build_sdf lays a wall's (phi normalized by S) gives the wall's distance in metres and its
// gradient.
TEST(DynContactPlant, ReadsNavsSignedDistanceGrid) {
  // A 100 m square at 0.5 m cells, the wall's half-plane x >= 25 m occupied, phi as build_sdf
  // writes it: normalized (S), clipped, normals beside it.
  const int H = 201, W = 201;
  const double S = 0.05, mn = -50.0, mx = 50.0, cell = (mx - mn) / (W - 1);
  std::vector<float> data(3 * H * W, 0.0f);
  for (int r = 0; r < H; ++r)
    for (int c = 0; c < W; ++c) {
      const double x = mn + c * cell;
      data[r * W + c] = static_cast<float>(S * (25.0 - x));
      data[H * W + r * W + c] = -1.0f;
    }
  cvc::nav::field_stack fs;
  fs.data = data.data();
  fs.M = 1;
  fs.H = H;
  fs.W = W;
  fs.mnx = fs.mny = mn;
  fs.mxx = fs.mxy = mx;
  fs.S = S;
  const auto field = dyn_bike_plant::sdf_field(fs);
  double worst = 0.0;
  for (const double x : {10.0, 20.3, 24.9, 25.4, 30.17})
    for (const double y : {-3.3, 0.0, 12.71}) {
      double phi, gx, gy;
      field(x, y, phi, gx, gy);
      worst = std::max({worst, std::fabs(phi - (25.0 - x)), std::fabs(gx + 1.0), std::fabs(gy)});
    }
  std::printf("  the grid's field against the wall's: worst %.2e\n", worst);
  EXPECT_LT(worst, 1e-5);
}

// The values GRL-SNAM's torch twin of the contact is held to: the footprint's points, the contact
// against a wall and a pillar (analytic fields) and between two vehicles at several poses, and a
// dyn_plant_books step (DynBikePH.Golden's Humvee) under a wrench, with the port's energy.
// CVC_DYN_CONTACT_GOLDEN_WRITE=1 rewrites tests/data/dyn_contact_golden.json; otherwise this build
// must reproduce it to 1e-9.
TEST(DynContact, Golden) {
  namespace fs = std::filesystem;
  const fs::path path = fs::path(__FILE__).parent_path() / "data" / "dyn_contact_golden.json";
  const diff::dyn_contact_params p;
  const diff::dyn_footprint f = humvee_footprint();
  const std::vector<diff::dyn_point> pts = diff::dyn_footprint_points(f, p);
  std::vector<double> now;
  std::ostringstream js;
  const auto num = [](double x) {
    char b[40];
    std::snprintf(b, sizeof b, "%.17g", x);
    return std::string(b);
  };
  const auto arr = [&](const double *v, int n) {
    std::string out = "[";
    for (int k = 0; k < n; ++k)
      out += (k ? ", " : "") + num(v[k]);
    return out + "]";
  };
  const auto contact_json = [&](const diff::dyn_contact &c) {
    double v[8] = {c.Wc[0], c.Wc[1], c.Wc[2], c.Wd[0], c.Wd[1], c.Wd[2], c.energy, c.damping};
    now.insert(now.end(), v, v + 8);
    now.push_back(c.touching);
    return std::string("{\"Wc\": ") + arr(v, 3) + ", \"Wd\": " + arr(v + 3, 3) +
           ", \"energy\": " + num(c.energy) + ", \"damping\": " + num(c.damping) +
           ", \"touching\": " + std::to_string(c.touching) + "}";
  };
  js << "{\n \"format\": \"cvc-dyn-contact-golden-1\",\n \"params\": {\"k\": " << num(p.k)
     << ", \"c\": " << num(p.c) << ", \"friction\": " << num(p.friction)
     << ", \"slide\": " << num(p.slide) << ", \"skin\": " << num(p.skin)
     << ", \"per_side\": " << p.per_side << ", \"per_end\": " << p.per_end
     << ", \"rounding\": " << num(p.rounding) << "},\n \"footprint\": {\"front\": " << num(f.front)
     << ", \"rear\": " << num(f.rear) << ", \"half_width\": " << num(f.half_width)
     << "},\n \"points\": [";
  for (std::size_t k = 0; k < pts.size(); ++k) {
    js << (k ? ", " : "") << "[" << num(pts[k].x) << ", " << num(pts[k].y) << "]";
    now.push_back(pts[k].x);
    now.push_back(pts[k].y);
  }
  // The bodies: (x, y, psi, vx, vy, r) at the centre of gravity, each with points in the skin: the
  // front end 2 cm into the wall, a corner 2 cm in at an angle, a corner 3 cm off it, a side over
  // the pillar, a corner past it.
  const double bodies[][6] = {{22.72, 0.3, 0.0, 4.0, 0.1, 0.05},
                              {22.48, 1.0, 0.52, 3.5, -0.4, 0.3},
                              {22.42, -2.0, -0.4, -1.0, 0.8, -0.6},
                              {0.0, 2.25, 0.0, 2.0, 0.5, 0.2},
                              {2.6, -2.2, 2.4, 1.5, -1.0, 0.7}};
  const wall_field wall{25.0};
  const pillar_field pillar{0.0, 0.0, 1.2};
  js << "],\n \"wall_x0\": " << num(wall.x0) << ", \"pillar\": [" << num(pillar.cx) << ", "
     << num(pillar.cy) << ", " << num(pillar.rad) << "],\n \"field_cases\": [\n";
  for (int k = 0; k < 5; ++k) {
    const diff::dyn_body b{bodies[k][0], bodies[k][1], bodies[k][2],
                           bodies[k][3], bodies[k][4], bodies[k][5]};
    diff::dyn_contact c;
    const bool at_wall = k < 3;
    if (at_wall)
      diff::dyn_contact_field(p, pts, wall, b, c);
    else
      diff::dyn_contact_field(p, pts, pillar, b, c);
    EXPECT_GT(c.touching, 0) << "field case " << k << " pins nothing";
    js << "  {\"field\": \"" << (at_wall ? "wall" : "pillar")
       << "\", \"body\": " << arr(bodies[k], 6) << ", \"contact\": " << contact_json(c) << "}"
       << (k < 4 ? "," : "") << "\n";
  }
  const double pairs[][12] = {{0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 4.75, 0.2, 0.05, 0.0, 0.0, 0.0},
                              {0.0, 0.0, 0.3, 3.0, 0.2, 0.1, 3.1, 1.9, -0.6, 1.0, -0.5, 0.2},
                              {0.0, 0.0, 1.57, 0.5, 2.0, 0.0, 1.0, 2.6, 0.1, 2.5, 0.0, -0.3}};
  js << " ],\n \"pair_cases\": [\n";
  for (int k = 0; k < 3; ++k) {
    const diff::dyn_body a{pairs[k][0], pairs[k][1], pairs[k][2],
                           pairs[k][3], pairs[k][4], pairs[k][5]};
    const diff::dyn_body b{pairs[k][6], pairs[k][7],  pairs[k][8],
                           pairs[k][9], pairs[k][10], pairs[k][11]};
    diff::dyn_contact ca, cb;
    diff::dyn_contact_pair(p, pts, f, a, pts, f, b, ca, cb);
    js << "  {\"a\": " << arr(pairs[k], 6) << ", \"b\": " << arr(pairs[k] + 6, 6)
       << ",\n   \"on_a\": " << contact_json(ca) << ",\n   \"on_b\": " << contact_json(cb) << "}"
       << (k < 2 ? "," : "") << "\n";
  }
  // A plant step under a wrench (the external port).
  diff::dyn_bike_params hp =
      cvc::nav::dyn_bike_params_of(nav_plant::dyn_bike_si_of(vehicle_desc::humvee()), metres());
  hp.tau_steer = 0.05f;
  hp.v_blend = 1.0f;
  hp.v_width = 0.3f;
  diff::bike_veh lim{};
  lim.L = hp.lf + hp.lr;
  lim.vmax = 40.0f;
  lim.delta_max = 0.6f;
  lim.hdt = 0.06f;
  const double state[8] = {1.0, -2.0, 0.3, 4.0, -0.4, 0.2, 0.1, -1.0};
  const double W[3] = {-3.5e4, 1.2e4, -8.0e3};
  diff::dyn_state<double> s, o;
  for (int k = 0; k < 8; ++k)
    s[k] = state[k];
  diff::dyn_ph_books<double> bk;
  double port = 0.0;
  diff::dyn_step_free<double> fr;
  diff::dyn_plant_books<double>(hp, lim, s, 0.12, -2.0, 0.01, o, bk, W, &port, &fr);
  double next[8];
  for (int k = 0; k < 8; ++k)
    next[k] = o[k];
  const double books[] = {bk.dH,    bk.dissipated, bk.supplied, bk.floor,
                          bk.grade, bk.limiter,    bk.residual, port};
  const double twist[] = {fr.vx, fr.vy, fr.r};
  now.insert(now.end(), next, next + 8);
  now.insert(now.end(), books, books + 8);
  now.insert(now.end(), twist, twist + 3);
  // The step's vehicle, so the twin needs no other file.
  js << " ],\n \"step\": {\"params\": {";
  const std::pair<const char *, float> hpar[] = {{"m", hp.m},
                                                 {"Iz", hp.Iz},
                                                 {"lf", hp.lf},
                                                 {"lr", hp.lr},
                                                 {"h", hp.h},
                                                 {"g", hp.g},
                                                 {"mu", hp.mu},
                                                 {"B_f", hp.B_f},
                                                 {"B_r", hp.B_r},
                                                 {"C", hp.C},
                                                 {"front_drive", hp.front_drive},
                                                 {"front_brake", hp.front_brake},
                                                 {"drag", hp.drag},
                                                 {"c_rr", hp.c_rr},
                                                 {"tau_steer", hp.tau_steer},
                                                 {"steer_rate", hp.steer_rate},
                                                 {"tau_drive", hp.tau_drive},
                                                 {"v_blend", hp.v_blend},
                                                 {"v_width", hp.v_width},
                                                 {"eps", hp.eps},
                                                 {"fz_min", hp.fz_min}};
  for (std::size_t k = 0; k < sizeof hpar / sizeof hpar[0]; ++k)
    js << (k ? ", " : "") << "\"" << hpar[k].first << "\": " << num(hpar[k].second);
  js << ", \"iters\": " << hp.iters << "},\n  \"state\": " << arr(state, 8)
     << ", \"delta_cmd\": 0.12, \"a_cmd\": -2, \"grade\": 0.01, \"wrench\": " << arr(W, 3)
     << ",\n  \"limits\": {\"L\": " << num(lim.L) << ", \"vmax\": " << num(lim.vmax)
     << ", \"hdt\": " << num(lim.hdt) << ", \"allow_reverse\": 0}"
     << ",\n  \"next\": " << arr(next, 8) << ",\n  \"books\": {\"dH\": " << num(bk.dH)
     << ", \"dissipated\": " << num(bk.dissipated) << ", \"supplied\": " << num(bk.supplied)
     << ", \"floor\": " << num(bk.floor) << ", \"grade\": " << num(bk.grade)
     << ", \"limiter\": " << num(bk.limiter) << ", \"residual\": " << num(bk.residual)
     << ", \"port\": " << num(port) << "}"
     << ",\n  \"free\": " << arr(twist, 3) << "}\n}\n";
  const char *write = std::getenv("CVC_DYN_CONTACT_GOLDEN_WRITE");
  if (write && std::string(write) == "1") {
    std::ofstream(path, std::ios::binary) << js.str();
    std::printf("  wrote %s\n", path.string().c_str());
    return;
  }
  // Read the file back in the order written.
  boost::property_tree::ptree want;
  ASSERT_NO_THROW(boost::property_tree::read_json(path.string(), want)) << path;
  std::vector<double> golden;
  for (const auto &pt : want.get_child("points"))
    for (const auto &v : pt.second)
      golden.push_back(v.second.get_value<double>());
  const auto read_contact = [&](const boost::property_tree::ptree &c) {
    for (const char *key : {"Wc", "Wd"})
      for (const auto &v : c.get_child(key))
        golden.push_back(v.second.get_value<double>());
    golden.push_back(c.get<double>("energy"));
    golden.push_back(c.get<double>("damping"));
    golden.push_back(c.get<double>("touching"));
  };
  for (const auto &c : want.get_child("field_cases"))
    read_contact(c.second.get_child("contact"));
  for (const auto &c : want.get_child("pair_cases")) {
    read_contact(c.second.get_child("on_a"));
    read_contact(c.second.get_child("on_b"));
  }
  const boost::property_tree::ptree &st = want.get_child("step");
  for (const auto &v : st.get_child("next"))
    golden.push_back(v.second.get_value<double>());
  for (const char *key :
       {"dH", "dissipated", "supplied", "floor", "grade", "limiter", "residual", "port"})
    golden.push_back(st.get<double>(std::string("books.") + key));
  for (const auto &v : st.get_child("free"))
    golden.push_back(v.second.get_value<double>());
  ASSERT_EQ(golden.size(), now.size());
  for (std::size_t k = 0; k < now.size(); ++k)
    EXPECT_LE(std::fabs(now[k] - golden[k]),
              1e-9 * std::max(1.0, std::max(std::fabs(now[k]), std::fabs(golden[k]))))
        << "value " << k << ": golden " << golden[k] << ", now " << now[k];
}

// End to end: sim_world's own field (its occupancy's SDF, sim_world::field) as the live plant's
// contact field. A Humvee driven at 5 m/s at a wall in sim_world's occupancy stops at it.
TEST(DynContactPlant, StopsAtAWallInSimWorldsField) {
  // 200 m square, 1 m cells, the wall x >= 25 m occupied; nav's trained metric.
  cvc::nav::sim_world::config c;
  c.rows = c.cols = 201;
  c.min_x = c.min_y = -100.0;
  c.max_x = c.max_y = 100.0;
  c.scale = 0.05;
  c.veh.rr = 0.15f;
  c.veh.d_hat = 0.35f;
  c.veh.dt = 0.02f;
  c.freeze_sense = true;
  std::vector<std::uint8_t> occ(static_cast<std::size_t>(c.rows) * c.cols, 0);
  for (int r = 0; r < c.rows; ++r)
    for (int k = 0; k < c.cols; ++k)
      if (c.min_x + k * (c.max_x - c.min_x) / (c.cols - 1) >= 25.0)
        occ[static_cast<std::size_t>(r) * c.cols + k] = 1;
  const float o[2] = {static_cast<float>(-60 * c.scale), 0.0f};
  const float g[2] = {static_cast<float>(-40 * c.scale), 0.0f};
  const float col[3] = {1, 1, 1};
  const cvc::nav::sim_world sim(c, occ.data(), occ.data(), cvc::nav::coef_mlp::default_biased(), o,
                                g, col, 1);
  dyn_bike_plant_options opt;
  opt.driver = false;
  opt.contact = true;
  opt.contact_vehicles = false;
  opt.contact_field = dyn_bike_plant::sdf_field(sim.field(), 0, 1.0);
  double phi, gx, gy;
  opt.contact_field(20.0, 3.0, phi, gx, gy);
  std::printf("  sim_world's field 5 m from the wall: phi %.3f m, gradient (%.3f, %.3f)\n", phi, gx,
              gy);
  EXPECT_NEAR(phi, 5.0, 1.0);
  EXPECT_LT(gx, -0.9);
  dyn_bike_plant plant({vehicle_desc::humvee()}, opt);
  cvc::nav::plant_state start;
  plant.attach(1, &start, metres(), 0.02);
  bool coasting = false;
  double furthest = -1e9, min_vx = 1e9;
  for (int t = 0; t < 500; ++t) {
    const diff::dyn_state<double> &s = plant.body(0);
    coasting = coasting || s.vx >= 5.0;
    plant.command(0, 0.0f, coasting ? 0.0f : 2.0f);
    plant.advance();
    const diff::dyn_state<double> &b = plant.body(0);
    furthest = std::max(furthest, b.x + plant.params(0).lr + plant.footprint(0).front);
    min_vx = std::min(min_vx, b.vx);
  }
  std::printf("  the Humvee's front reached x = %.2f m (the wall at 25 m); slowest %.2f m/s\n",
              furthest, min_vx);
  EXPECT_LT(furthest, 25.4) << "it went through the wall";
  EXPECT_GT(furthest, 24.0) << "it never reached the wall";
  EXPECT_LT(std::fabs(plant.body(0).vx), 1.0) << "it did not stop at the wall";
}
