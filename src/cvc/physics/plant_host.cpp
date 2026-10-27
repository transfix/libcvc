/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include "binding_meta.h"

#include <cvc/core/app.h>
#include <cvc/core/exception.h>
#include <cvc/nav/sim_world.h>
#include <cvc/physics/backend.h>
#include <cvc/physics/plant_host.h>
#include <cvc/physics/world_geometry.h>
#include <cvc/state/state.h>
#include <limits>
#include <sstream>

namespace cvc {
namespace physics {

namespace {

constexpr double kNone = std::numeric_limits<double>::quiet_NaN();

// The host's keys, relative to <prefix>.nav_plant.
const detail::key_meta kHostKeys[] = {
    {"vehicle", "knob", "", kNone, kNone,
     "What the agents drive: kinematic (nav's bicycle), a vehicle preset on Jolt (humvee, "
     "roboracer), a comma-separated list of presets cycled over the agents, or dynamic / "
     "dynamic:<presets> for the presets' dynamic single-track model with no physics world. A "
     "change rebuilds the plant."},
    {"driving", "telemetry", "", kNone, kNone,
     "What the agents drive now: the vehicle asked for, or kinematic for Jolt presets in a build "
     "without Jolt."},
    {"available", "build", "", kNone, kNone, "1 when this build has the Jolt physics backend."},
};

std::string trim(const std::string &s) {
  const auto b = s.find_first_not_of(" \t");
  if (b == std::string::npos)
    return std::string();
  const auto e = s.find_last_not_of(" \t");
  return s.substr(b, e - b + 1);
}

bool is_kinematic(const std::string &name) {
  const std::string n = trim(name);
  return n.empty() || n == "kinematic";
}

constexpr const char *kDynamic = "dynamic";

// The presets after "dynamic:", or the Humvee for "dynamic" alone.
std::string dynamic_presets(const std::string &name) {
  const std::string n = trim(name);
  const std::string::size_type colon = n.find(':');
  const std::string rest = colon == std::string::npos ? std::string() : trim(n.substr(colon + 1));
  return rest.empty() ? std::string("humvee") : rest;
}

// The presets a vehicle string names, in order; throws on an unknown one.
std::vector<vehicle_desc> descs_of(const std::string &vehicle) {
  std::vector<vehicle_desc> out;
  std::stringstream ss(vehicle);
  std::string item;
  while (std::getline(ss, item, ','))
    out.push_back(plant_host::preset(trim(item)));
  if (out.empty())
    throw cvc::physics_error("plant_host: no vehicle named");
  return out;
}

template <class T> void write_val(cvc::app &ctx, const std::string &path, const T &v) {
  try {
    cvc::state::instance(ctx)(path).value(v);
  } catch (const std::exception &) {
  }
}

} // namespace

plant_host::plant_host(plant_host_options options) : _o(std::move(options)) {}

// The plant goes before its world: it is the world's vehicles' owner of record.
plant_host::~plant_host() {
  _dyn.reset();
  _plant.reset();
  _world.reset();
}

bool plant_host::available() { return backend_available(backend::jolt); }

std::vector<std::string> plant_host::presets() { return {"humvee", "roboracer"}; }

bool plant_host::is_dynamic(const std::string &vehicle) {
  const std::string n = trim(vehicle);
  return n == kDynamic || n.rfind(std::string(kDynamic) + ":", 0) == 0;
}

cvc::nav::vehicle_plant *plant_host::active() {
  if (_dyn)
    return _dyn.get();
  return _plant.get();
}

vehicle_desc plant_host::preset(const std::string &name) {
  if (name == "humvee")
    return vehicle_desc::humvee();
  if (name == "roboracer")
    return vehicle_desc::roboracer();
  throw cvc::physics_error("plant_host: no vehicle preset named '" + name +
                           "' (humvee, roboracer, kinematic, or dynamic[:presets])");
}

void plant_host::check(const std::string &vehicle) {
  if (is_dynamic(vehicle))
    descs_of(dynamic_presets(vehicle));
  else if (!is_kinematic(vehicle))
    descs_of(vehicle);
}

void plant_host::set_geometry(const cvc::world::world_geometry *g, std::vector<int> nav_materials) {
  _geometry = g;
  _nav_materials = std::move(nav_materials);
}

std::string plant_host::use(cvc::nav::sim_world &sim, const std::string &vehicle) {
  if (is_kinematic(vehicle)) {
    sim.set_plant(nullptr);
    _dyn.reset();
    _plant.reset();
    _world.reset();
    _asked = _driving = "kinematic";
    return _driving;
  }
  if (is_dynamic(vehicle)) {
    std::vector<vehicle_desc> descs = descs_of(dynamic_presets(vehicle)); // throws first
    dyn_bike_plant_options o;
    o.plant = _o.plant;
    o.mu = _o.ground_mu;
    auto d = std::make_unique<dyn_bike_plant>(std::move(descs), o);
    sim.set_plant(nullptr);
    _dyn.reset();
    _plant.reset();
    _world.reset();
    _dyn = std::move(d);
    sim.set_plant(_dyn.get());
    _asked = _driving = trim(vehicle);
    return _driving;
  }
  std::vector<vehicle_desc> descs = descs_of(vehicle); // throws before anything changes
  if (!available()) {
    sim.set_plant(nullptr);
    _dyn.reset();
    _plant.reset();
    _world.reset();
    _asked = trim(vehicle);
    _driving = "kinematic";
    return _driving;
  }
  auto w = std::make_unique<world>(_o.world);
  if (_geometry) {
    add_world_geometry(*w, *_geometry, 0.005, _nav_materials);
  } else {
    const material_id m = w->add_material({"ground", static_cast<float>(_o.ground_mu), 0.0f,
                                           static_cast<float>(_o.ground_rolling_resistance)});
    body_desc g;
    g.shape = box{{_o.ground_half_size, _o.ground_half_size, 0.5}};
    g.motion = motion_type::static_;
    g.position = {0.0, 0.0, _o.ground_z - 0.5};
    g.material = m;
    w->add_body(g);
  }
  nav_plant_options po = _o.plant;
  if (!_geometry)
    po.ground_z = _o.ground_z;
  auto p = std::make_unique<nav_plant>(*w, std::move(descs), po);
  // Detach the old plant before its world goes; then attach the new one.
  sim.set_plant(nullptr);
  _dyn.reset();
  _plant.reset();
  _world = std::move(w);
  _plant = std::move(p);
  sim.set_plant(_plant.get());
  _asked = _driving = trim(vehicle);
  return _driving;
}

void plant_host::pull(cvc::app &ctx, const std::string &prefix, cvc::nav::sim_world &sim) {
  const std::string k = (prefix.empty() ? std::string() : prefix + ".") + "nav_plant.";
  if (_described.insert(k).second) {
    for (const detail::key_meta &m : kHostKeys)
      detail::describe(ctx, k + m.key, m);
    write_val(ctx, k + "available", available() ? 1 : 0);
  }
  std::string want;
  try {
    want = cvc::state::instance(ctx)(k + "vehicle").value();
  } catch (const std::exception &) {
  }
  if (want.empty())
    write_val(ctx, k + "vehicle", _asked);
  else if (trim(want) != _asked) {
    try {
      use(sim, want);
    } catch (const std::exception &) {
      // an unknown name is ignored: the agents keep what they drive
    }
  }
  write_val(ctx, k + "driving", _driving);
  if (_plant)
    _plant->pull(ctx, prefix);
}

} // namespace physics
} // namespace cvc
