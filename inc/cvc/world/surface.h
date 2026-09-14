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

// surface.h — the semantic surface-class registry (roadmap §7.2 / §16.2).
//
// The class map is the AUTHORED TRUTH; `risk_raw` and `hard` are a derived,
// versioned projection of it by table lookup. Two rules from §7.2a are load-
// bearing and asserted in tests:
//
//   * every `hard` class carries rho == 0.0 (the consumer penalises `hard`
//     separately, and a rho=1 wall would bleed through its blur into the
//     corridor). Max risk_raw under merged_default is therefore 0.90.
//   * hard ⊆ occupancy (a hard cell is always occupied).
//
// Each class also carries a generic `rf_material` name from the shared physical
// vocabulary (wood / foliage / soil / rock / reinforced_concrete / brick /
// drywall / glass / metal / water / open_air). This is the seam the DBG side
// keys its RF penetration table off; cvc::world does NOT know the DBG integer
// ids or the n_materials cap — that mapping stays private to cvc::dbg. The
// `penetration_db_per_m[3]` triple is carried as sub-6 GHz PROVENANCE only (the
// authoritative attenuation is the DBG MATERIAL_TABLE, resolved from the name).

#ifndef CVC_WORLD_SURFACE_H
#define CVC_WORLD_SURFACE_H

#include <cstdint>
#include <cvc/lsys/interp.h> // cvc::lsys::role
#include <string>
#include <vector>

namespace cvc {
namespace world {

enum class nav_class : std::uint8_t { free = 0, rough, blocked_wall, blocked_fall, door, portal };
enum class tier : std::uint8_t { low, medium, high_soft, hard_hazard };

struct surface_class {
  std::uint16_t id = 0;
  const char *name = "";
  tier t = tier::medium;
  float rho = 0.0f;  // -> risk_raw ; hard classes carry 0.0 (§7.2a)
  bool hard = false; // -> hard raster (and occupancy)
  nav_class nav = nav_class::free;
  float albedo[3] = {0.5f, 0.5f, 0.5f};
  float veg_density = 0.0f;
  const char *rf_material = "open_air";      // generic 12-name-vocab material label
  float penetration_db_per_m[3] = {0, 0, 0}; // sub6 (low,mid,high) PROVENANCE only
};

// A registry is one ontology: the ordered class table + name lookup. Variants
// (merged_default / soft_vegetation / strict_water_mud) differ only in a few
// soft rho values; the class ids, names and hard set are identical.
class surface_registry {
public:
  // The default ontology. merged_default corresponds to the RELLIS research
  // ontology's `main` mapping (renamed; see risk_ontology.yaml).
  static const surface_registry &builtin();

  // A named ontology variant. Unknown name -> merged_default. Valid names:
  // "merged_default", "soft_vegetation", "strict_water_mud".
  static const surface_registry &variant(const std::string &name);

  const surface_class &operator[](std::uint16_t id) const;
  std::uint16_t by_name(const std::string &name) const; // throws std::out_of_range if unknown
  bool find(const std::string &name, std::uint16_t &out) const;
  std::size_t size() const noexcept { return classes_.size(); }
  const std::string &ontology() const noexcept { return ontology_; }

  // Map a generic cvc::lsys material role to the concrete class id used for the
  // geometry that carries that role (trunk->tree_trunk, wall_concrete->
  // building_concrete, ...). This is the ONLY place role semantics meet the
  // registry; recipes stay decoupled from class ids.
  std::uint16_t class_for_role(cvc::lsys::role r) const;

  // Convenience: the rf_material name for a class id.
  const char *rf_material(std::uint16_t id) const { return (*this)[id].rf_material; }

  std::string to_json() const;
  std::uint64_t ontology_hash() const noexcept { return hash_; }

private:
  surface_registry() = default;
  static surface_registry make(const std::string &ontology);
  void finalize(); // recompute name index + hash

  std::vector<surface_class> classes_;
  std::vector<std::uint16_t> role_map_; // indexed by (int)role
  std::string ontology_ = "merged_default";
  std::uint64_t hash_ = 0;
};

} // namespace world
} // namespace cvc

#endif // CVC_WORLD_SURFACE_H
