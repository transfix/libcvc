/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::nav's plant ledger, item by item: every field of nav_stats.h's veh_nav_stats and
// episode_nav_stats that a vehicle plant feeds, for whatever writes or shows them (the JSON,
// cvcGL's publish_nav_stats, an application's screen), so each shows all of them.
//
// An item is the plant's energy (energy_tracked: the energy drawn and the building contacts) or
// its measurements (plant_tracked: nav_samplers::plant). It has a value when its plant fed it and
// measures the fields it is reduced from (plant_sample::measured); otherwise it is not
// applicable, and a reader shows it as such, not as its default. On the kinematic plant none
// applies.
//
//   for (const cvc::nav::plant_ledger_item &it : cvc::nav::plant_ledger_vehicle())
//     if (cvc::nav::plant_ledger_applies(it, v))
//       show(it.key, it.veh(v), it.unit);
//     else
//       show(it.key, "n/a");

#ifndef __CVC_NAV_PLANT_LEDGER_H__
#define __CVC_NAV_PLANT_LEDGER_H__

#include <cstdint>
#include <cvc/nav/nav_stats.h>
#include <vector>

namespace cvc {
namespace nav {

struct plant_ledger_item {
  enum class kind : std::uint8_t {
    measure, // a double
    count,   // an integer
    flag,    // 0 or 1
  };
  enum class source : std::uint8_t {
    energy, // energy_tracked: what the plant's vehicles drew, and their building contacts
    plant,  // plant_tracked: the plant's samples of its vehicles
  };
  const char *key;     // the JSON's and the state tree's: .veh.<i>.<key>, .episode.<key>
  const char *unit;    // "" a count or flag, "1" a ratio
  const char *comment; // what it is, in a sentence
  kind type;
  source from;
  std::uint32_t needs; // the plant_sample::measured bits it is reduced from (0: none)
  // Its value: veh on a vehicle's item, ep on an episode's (the other is null).
  double (*veh)(const veh_nav_stats &);
  double (*ep)(const episode_nav_stats &);
};

// Each vehicle's items, then the episode's, in their JSON order.
const std::vector<plant_ledger_item> &plant_ledger_vehicle();
const std::vector<plant_ledger_item> &plant_ledger_episode();

// Whether `it` has a value for this vehicle (episode): its plant fed its source and measures
// what it is reduced from. False: not applicable.
bool plant_ledger_applies(const plant_ledger_item &it, const veh_nav_stats &v);
bool plant_ledger_applies(const plant_ledger_item &it, const episode_nav_stats &e);

} // namespace nav
} // namespace cvc

#endif // __CVC_NAV_PLANT_LEDGER_H__
