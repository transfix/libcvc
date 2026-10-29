/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// plant_ledger.cpp — see inc/cvc/nav/plant_ledger.h.

#include <cvc/nav/plant_ledger.h>

namespace cvc {
namespace nav {

namespace {

using item = plant_ledger_item;
using kind = plant_ledger_item::kind;
using source = plant_ledger_item::source;
constexpr std::uint32_t kAttitude = plant_sample::has_attitude;
constexpr std::uint32_t kLtr = plant_sample::has_ltr;
constexpr std::uint32_t kWheelsDown = plant_sample::has_wheels_down;
constexpr std::uint32_t kContacts = plant_sample::has_contacts;
constexpr std::uint32_t kVehicleContact = plant_sample::has_vehicle_contact;

#define CVC_LEDGER_VEH(field) [](const veh_nav_stats &v) { return static_cast<double>(v.field); }
#define CVC_LEDGER_EP(field) [](const episode_nav_stats &e) { return static_cast<double>(e.field); }

bool applies(const item &it, bool energy, bool plant, std::uint32_t measured) {
  // A building contact needs a plant that collides; energy fed without a ledger counts as before.
  const bool has = (measured & it.needs) == it.needs;
  if (it.from == source::energy)
    return energy && (!plant || has);
  return plant && has;
}

} // namespace

const std::vector<plant_ledger_item> &plant_ledger_vehicle() {
  static const std::vector<item> items = {
      {"energy_J", "J",
       "The energy it drew from its source, net of regeneration (a physics plant).", kind::measure,
       source::energy, 0, CVC_LEDGER_VEH(energy_J), nullptr},
      {"energy_per_km", "J/km", "energy_J per kilometre of its path.", kind::measure,
       source::energy, 0, CVC_LEDGER_VEH(energy_per_km), nullptr},
      {"building_contacts", "", "Contacts its body started with static geometry (a physics plant).",
       kind::count, source::energy, kContacts, CVC_LEDGER_VEH(building_contacts), nullptr},
      {"plant_steps", "", "Ticks its plant measured it (a physics plant).", kind::count,
       source::plant, 0, CVC_LEDGER_VEH(plant_steps), nullptr},
      {"rolled_over", "",
       "1 once it rolled over: tipped past 60 degrees for 1 s (a physics plant).", kind::flag,
       source::plant, kAttitude, CVC_LEDGER_VEH(rolled_over), nullptr},
      {"rolled_over_s", "s", "When it tipped over for good; < 0 = never (a physics plant).",
       kind::measure, source::plant, kAttitude, CVC_LEDGER_VEH(rolled_over_s), nullptr},
      {"tilt_max_deg", "deg", "The most its body tilted from upright (a physics plant).",
       kind::measure, source::plant, kAttitude, CVC_LEDGER_VEH(tilt_max_deg), nullptr},
      {"roll_max_deg", "deg", "The most its body rolled (a physics plant).", kind::measure,
       source::plant, kAttitude, CVC_LEDGER_VEH(roll_max_deg), nullptr},
      {"pitch_max_deg", "deg", "The most its body pitched (a physics plant).", kind::measure,
       source::plant, kAttitude, CVC_LEDGER_VEH(pitch_max_deg), nullptr},
      {"tilt_s", "s", "Time it was tilted past 20 degrees (a physics plant).", kind::measure,
       source::plant, kAttitude, CVC_LEDGER_VEH(tilt_s), nullptr},
      {"ltr_max", "1",
       "Its largest |lateral load transfer ratio|; 1 = a side lifts (a physics plant).",
       kind::measure, source::plant, kLtr, CVC_LEDGER_VEH(ltr_max), nullptr},
      {"lift_s", "s",
       "Time its |load transfer| was past 0.9: a side's wheels about to lift (a physics plant).",
       kind::measure, source::plant, kLtr, CVC_LEDGER_VEH(lift_s), nullptr},
      {"wheels_up_s", "s", "Time with two or more wheels off the ground (a physics plant).",
       kind::measure, source::plant, kWheelsDown, CVC_LEDGER_VEH(wheels_up_s), nullptr},
      {"a_lat_max_g", "g", "Its largest |lateral acceleration| (a physics plant).", kind::measure,
       source::plant, 0, CVC_LEDGER_VEH(a_lat_max_g), nullptr},
      {"steer_cmd_abs_mean", "rad", "Nav's mean |steering command| (a physics plant).",
       kind::measure, source::plant, 0, CVC_LEDGER_VEH(steer_cmd_abs_mean), nullptr},
      {"steer_sat_share", "1",
       "Share of ticks nav steered at 95% of its lock or more (a physics plant).", kind::measure,
       source::plant, 0, CVC_LEDGER_VEH(steer_sat_share), nullptr},
      {"accel_cmd_mean", "m/s^2", "Nav's mean acceleration command (a physics plant).",
       kind::measure, source::plant, 0, CVC_LEDGER_VEH(accel_cmd_mean), nullptr},
      {"brake_share", "1", "Share of ticks nav asked it to slow (a physics plant).", kind::measure,
       source::plant, 0, CVC_LEDGER_VEH(brake_share), nullptr},
      {"vehicle_hits", "",
       "Contacts its body began with another vehicle's (a physics plant's collisions, not "
       "veh_contacts' distance).",
       kind::count, source::plant, kVehicleContact, CVC_LEDGER_VEH(vehicle_hits), nullptr},
      {"vehicle_hit_s", "s", "Time its body touched another vehicle's (a physics plant).",
       kind::measure, source::plant, kVehicleContact, CVC_LEDGER_VEH(vehicle_hit_s), nullptr},
  };
  return items;
}

const std::vector<plant_ledger_item> &plant_ledger_episode() {
  static const std::vector<item> items = {
      {"total_energy_J", "J", "The fleet's energy drawn from its sources (a physics plant).",
       kind::measure, source::energy, 0, nullptr, CVC_LEDGER_EP(total_energy_J)},
      {"total_building_contacts", "",
       "The fleet's contacts with static geometry (a physics plant).", kind::count, source::energy,
       kContacts, nullptr, CVC_LEDGER_EP(total_building_contacts)},
      {"vehicles_rolled", "", "Vehicles that rolled over (a physics plant).", kind::count,
       source::plant, kAttitude, nullptr, CVC_LEDGER_EP(vehicles_rolled)},
      {"first_roll_s", "s", "When the first vehicle rolled over; < 0 = none (a physics plant).",
       kind::measure, source::plant, kAttitude, nullptr, CVC_LEDGER_EP(first_roll_s)},
      {"total_vehicle_hits", "",
       "Contacts the vehicles began with each other, each vehicle's counted (a physics plant).",
       kind::count, source::plant, kVehicleContact, nullptr, CVC_LEDGER_EP(total_vehicle_hits)},
  };
  return items;
}

#undef CVC_LEDGER_VEH
#undef CVC_LEDGER_EP

bool plant_ledger_applies(const plant_ledger_item &it, const veh_nav_stats &v) {
  return it.veh && applies(it, v.energy_tracked, v.plant_tracked, v.plant_measured);
}

bool plant_ledger_applies(const plant_ledger_item &it, const episode_nav_stats &e) {
  return it.ep && applies(it, e.energy_tracked, e.plant_tracked, e.plant_measured);
}

} // namespace nav
} // namespace cvc
