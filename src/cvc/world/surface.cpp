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

#include <cstdio>
#include <cstring>
#include <cvc/world/surface.h>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace cvc {
namespace world {

namespace {

const char *tier_name(tier t) {
  switch (t) {
  case tier::low:
    return "low";
  case tier::medium:
    return "medium";
  case tier::high_soft:
    return "high_soft";
  case tier::hard_hazard:
    return "hard_hazard";
  }
  return "medium";
}
const char *nav_name(nav_class n) {
  switch (n) {
  case nav_class::free:
    return "free";
  case nav_class::rough:
    return "rough";
  case nav_class::blocked_wall:
    return "blocked_wall";
  case nav_class::blocked_fall:
    return "blocked_fall";
  case nav_class::door:
    return "door";
  case nav_class::portal:
    return "portal";
  }
  return "free";
}

// Sub-6 GHz penetration (low, mid, high) dB/m, PROVENANCE only; the DBG side is
// authoritative. Sourced from grl_snam_dbg simulator/materials.py MATERIAL_TABLE
// (sub6 triple). `rock` is the new id-12 material (granite-ish, between brick
// and reinforced_concrete).
void penetration_for(const char *rf, float out[3]) {
  struct row {
    const char *name;
    float p[3];
  };
  static const row kRows[] = {
      {"open_air", {0, 0, 0}}, {"reinforced_concrete", {18, 24, 32}},
      {"brick", {10, 14, 20}}, {"glass", {4, 7, 12}},
      {"wood", {3, 5, 8}},     {"foliage", {6, 10, 16}},
      {"drywall", {2, 3, 5}},  {"metal", {30, 40, 55}},
      {"soil", {22, 30, 42}},  {"water", {12, 20, 35}},
      {"rock", {16, 22, 30}},
  };
  for (const row &r : kRows)
    if (std::strcmp(r.name, rf) == 0) {
      out[0] = r.p[0];
      out[1] = r.p[1];
      out[2] = r.p[2];
      return;
    }
  out[0] = out[1] = out[2] = 0.0f;
}

// The 37 shipped classes: roadmap §16.2's 32 (ids 0-31) + 5 outdoor building
// shells (32-36). hard classes carry rho == 0.0 (§7.2a).
std::vector<surface_class> base_classes() {
  auto C = [](std::uint16_t id, const char *name, tier t, float rho, bool hard, nav_class nav,
              float r, float g, float b, float veg, const char *rf) {
    surface_class c;
    c.id = id;
    c.name = name;
    c.t = t;
    c.rho = rho;
    c.hard = hard;
    c.nav = nav;
    c.albedo[0] = r;
    c.albedo[1] = g;
    c.albedo[2] = b;
    c.veg_density = veg;
    c.rf_material = rf;
    return c;
  };
  using T = tier;
  using N = nav_class;
  return {
      // ── Outdoor (18) ──
      C(0, "void_unknown", T::medium, 0.55f, false, N::free, 0.50f, 0.50f, 0.50f, 0.0f, "open_air"),
      C(1, "asphalt", T::low, 0.05f, false, N::free, 0.16f, 0.16f, 0.17f, 0.0f,
        "reinforced_concrete"),
      C(2, "concrete_ext", T::low, 0.08f, false, N::free, 0.55f, 0.54f, 0.52f, 0.0f,
        "reinforced_concrete"),
      C(3, "dirt", T::low, 0.05f, false, N::free, 0.42f, 0.33f, 0.23f, 0.15f, "soil"),
      C(4, "gravel", T::low, 0.12f, false, N::rough, 0.48f, 0.46f, 0.43f, 0.05f, "soil"),
      C(5, "sand", T::medium, 0.18f, false, N::rough, 0.68f, 0.62f, 0.44f, 0.02f, "soil"),
      C(6, "grass", T::medium, 0.25f, false, N::free, 0.27f, 0.44f, 0.19f, 1.00f, "soil"),
      C(7, "tall_grass", T::medium, 0.35f, false, N::rough, 0.30f, 0.42f, 0.18f, 1.40f, "foliage"),
      C(8, "bush_cover", T::medium, 0.45f, false, N::rough, 0.22f, 0.36f, 0.16f, 1.80f, "foliage"),
      C(9, "bare_rock", T::medium, 0.30f, false, N::rough, 0.46f, 0.45f, 0.43f, 0.10f, "rock"),
      C(10, "scree", T::high_soft, 0.55f, false, N::rough, 0.44f, 0.42f, 0.40f, 0.06f, "rock"),
      C(11, "snow", T::medium, 0.40f, false, N::rough, 0.92f, 0.94f, 0.97f, 0.0f, "soil"),
      C(12, "rubble", T::high_soft, 0.75f, false, N::rough, 0.40f, 0.38f, 0.36f, 0.02f, "rock"),
      C(13, "mud", T::high_soft, 0.80f, false, N::rough, 0.30f, 0.24f, 0.17f, 0.30f, "soil"),
      C(14, "puddle", T::high_soft, 0.85f, false, N::rough, 0.24f, 0.28f, 0.30f, 0.0f, "water"),
      C(15, "water_shallow", T::high_soft, 0.90f, false, N::rough, 0.16f, 0.30f, 0.36f, 0.0f,
        "water"),
      C(16, "water_deep", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.06f, 0.14f, 0.22f, 0.0f,
        "water"),
      C(17, "cliff_rock", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.38f, 0.37f, 0.35f, 0.0f,
        "rock"),
      // ── Outdoor obstacles (3) ──
      C(18, "tree_trunk", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.28f, 0.20f, 0.13f, 0.0f,
        "wood"),
      C(19, "boulder", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.42f, 0.41f, 0.39f, 0.0f,
        "rock"),
      C(20, "fence_pole", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.35f, 0.33f, 0.30f, 0.0f,
        "metal"),
      // ── Indoor (12) ──
      C(21, "concrete_floor", T::low, 0.06f, false, N::free, 0.58f, 0.57f, 0.55f, 0.0f,
        "reinforced_concrete"),
      C(22, "tile", T::low, 0.05f, false, N::free, 0.78f, 0.78f, 0.76f, 0.0f,
        "reinforced_concrete"),
      C(23, "linoleum", T::low, 0.05f, false, N::free, 0.62f, 0.60f, 0.52f, 0.0f, "wood"),
      C(24, "wood_floor", T::low, 0.08f, false, N::free, 0.52f, 0.36f, 0.20f, 0.0f, "wood"),
      C(25, "carpet", T::medium, 0.15f, false, N::free, 0.34f, 0.30f, 0.32f, 0.0f, "drywall"),
      C(26, "metal_grating", T::medium, 0.30f, false, N::rough, 0.44f, 0.45f, 0.47f, 0.0f, "metal"),
      C(27, "wet_floor", T::high_soft, 0.65f, false, N::rough, 0.50f, 0.52f, 0.55f, 0.0f,
        "reinforced_concrete"),
      C(28, "debris_indoor", T::high_soft, 0.70f, false, N::rough, 0.40f, 0.37f, 0.33f, 0.0f,
        "rock"),
      C(29, "wall_interior", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.82f, 0.80f, 0.76f,
        0.0f, "drywall"),
      C(30, "glass_pane", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.62f, 0.72f, 0.78f, 0.0f,
        "glass"),
      C(31, "void_fall", T::hard_hazard, 0.00f, true, N::blocked_fall, 0.05f, 0.05f, 0.06f, 0.0f,
        "open_air"),
      // ── Outdoor building shells (5, this task) — hard obstacle walls, tagged
      //    by RF material so a footprint tiles into discs of the right material. ──
      C(32, "building_concrete", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.60f, 0.59f, 0.57f,
        0.0f, "reinforced_concrete"),
      C(33, "building_brick", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.60f, 0.34f, 0.26f,
        0.0f, "brick"),
      C(34, "building_drywall", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.80f, 0.78f, 0.74f,
        0.0f, "drywall"),
      C(35, "building_glass", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.60f, 0.72f, 0.80f,
        0.0f, "glass"),
      C(36, "building_metal", T::hard_hazard, 0.00f, true, N::blocked_wall, 0.55f, 0.56f, 0.58f,
        0.0f, "metal"),
  };
}

void apply_variant(std::vector<surface_class> &c, const std::string &v) {
  auto set = [&](std::uint16_t id, float rho) {
    if (id < c.size())
      c[id].rho = rho;
  };
  if (v == "soft_vegetation") {
    set(6, 0.15f); // grass
    set(7, 0.22f); // tall_grass
    set(8, 0.30f); // bush_cover
  } else if (v == "strict_water_mud") {
    set(13, 0.95f); // mud
    set(14, 0.95f); // puddle
    set(15, 0.97f); // water_shallow
    set(0, 0.60f);  // void_unknown
  }
  // merged_default: no deltas.
}

} // namespace

surface_registry surface_registry::make(const std::string &ontology) {
  surface_registry r;
  r.classes_ = base_classes();
  r.ontology_ = ontology;
  apply_variant(r.classes_, ontology);

  // Role -> class id (the ONLY place lsys roles meet concrete classes).
  r.role_map_.assign(static_cast<std::size_t>(cvc::lsys::role::_count), 0);
  using role = cvc::lsys::role;
  auto R = [&](role rl, std::uint16_t id) { r.role_map_[static_cast<std::size_t>(rl)] = id; };
  R(role::trunk, 18);
  R(role::branch, 18);
  R(role::foliage, 8);
  R(role::rock, 19);
  R(role::ground, 3);
  R(role::water, 16);
  R(role::wall_concrete, 32);
  R(role::wall_brick, 33);
  R(role::wall_drywall, 34);
  R(role::glass, 35);
  R(role::metal, 36);
  R(role::wood_solid, 18);
  R(role::unknown, 0);

  r.finalize();
  return r;
}

void surface_registry::finalize() {
  for (surface_class &c : classes_)
    penetration_for(c.rf_material, c.penetration_db_per_m);
  // Hash over ontology + the load-bearing fields.
  std::uint64_t h = 0xcbf29ce484222325ull;
  auto mix = [&h](std::uint64_t v) {
    h ^= v;
    h *= 0x100000001b3ull;
  };
  for (char ch : ontology_)
    mix((unsigned char)ch);
  for (const surface_class &c : classes_) {
    mix(c.id);
    for (const char *p = c.name; *p; ++p)
      mix((unsigned char)*p);
    std::uint64_t rb = 0;
    std::memcpy(&rb, &c.rho, sizeof(float));
    mix(rb);
    mix(c.hard ? 1u : 0u);
    for (const char *p = c.rf_material; *p; ++p)
      mix((unsigned char)*p);
  }
  hash_ = h;
}

const surface_registry &surface_registry::builtin() {
  static const surface_registry r = make("merged_default");
  return r;
}

const surface_registry &surface_registry::variant(const std::string &name) {
  static const surface_registry def = make("merged_default");
  static const surface_registry soft = make("soft_vegetation");
  static const surface_registry strict = make("strict_water_mud");
  if (name == "soft_vegetation")
    return soft;
  if (name == "strict_water_mud")
    return strict;
  return def;
}

const surface_class &surface_registry::operator[](std::uint16_t id) const {
  if (id >= classes_.size())
    throw std::out_of_range("surface_registry: bad class id");
  return classes_[id];
}

bool surface_registry::find(const std::string &name, std::uint16_t &out) const {
  for (const surface_class &c : classes_)
    if (name == c.name) {
      out = c.id;
      return true;
    }
  return false;
}

std::uint16_t surface_registry::by_name(const std::string &name) const {
  std::uint16_t id;
  if (!find(name, id))
    throw std::out_of_range("surface_registry: unknown class name '" + name + "'");
  return id;
}

std::uint16_t surface_registry::class_for_role(cvc::lsys::role r) const {
  std::size_t i = static_cast<std::size_t>(r);
  return i < role_map_.size() ? role_map_[i] : 0;
}

std::string surface_registry::to_json() const {
  std::ostringstream o;
  o << "{\n  \"ontology\": \"" << ontology_ << "\",\n";
  char hb[32];
  std::snprintf(hb, sizeof(hb), "%016llx", (unsigned long long)hash_);
  o << "  \"ontology_hash\": \"b3:" << hb << "\",\n";
  o << "  \"classes\": [\n";
  for (std::size_t i = 0; i < classes_.size(); ++i) {
    const surface_class &c = classes_[i];
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "    {\"id\": %u, \"name\": \"%s\", \"tier\": \"%s\", \"rho\": %.3f, "
                  "\"hard\": %s, \"nav\": \"%s\", \"albedo\": [%.3f, %.3f, %.3f], "
                  "\"veg_density\": %.3f, \"rf_material\": \"%s\", "
                  "\"penetration_db_per_m\": [%.1f, %.1f, %.1f]}%s\n",
                  c.id, c.name, tier_name(c.t), c.rho, c.hard ? "true" : "false", nav_name(c.nav),
                  c.albedo[0], c.albedo[1], c.albedo[2], c.veg_density, c.rf_material,
                  c.penetration_db_per_m[0], c.penetration_db_per_m[1], c.penetration_db_per_m[2],
                  i + 1 < classes_.size() ? "," : "");
    o << buf;
  }
  o << "  ]\n}\n";
  return o.str();
}

} // namespace world
} // namespace cvc
