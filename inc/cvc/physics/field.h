/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

// cvc::physics fields: regions of the world that push on the bodies inside them,
// such as water a box floats in, snow a wheel sinks into, or wind a ball drifts
// with (docs/roadmap/WORLD-GEOMETRY-ROADMAP.md, G6).
//
//   cvc::physics::field lake;
//   lake.kind = cvc::physics::field_kind::fluid;
//   lake.volume = water;               // a cvc::volume of kg/m^3: what VolumeNode draws
//   const auto id = w.add_field(lake);
//
// A field is cvc::volume data in world metres: the same volume a
// cvc::gl::VolumeNode draws and a cvc::world::world_geometry holds. It acts inside
// its volume's bounding box only, and its values are sampled trilinearly there
// (corner samples, as cvc::volume::interpolate reads them). The world reads the
// values when the field is added or set, so a volume changed afterwards takes
// effect at the next set_field.
//
// How a field pushes. A dynamic body is cut into sample cells when it is added:
// the interior of its sphere, box or convex hull, about 8 cells along its longest
// side. Meshes and terrain are static and never pushed. Before every step, for
// each field and each awake dynamic body, each cell's world box is clipped to the
// field's box; the immersed part, a fraction f of the cell's volume V_c, is sampled
// at its centre:
//
//   fluid   buoyancy  -rho g f V_c       rho the volume's density, g the world's gravity
//           drag      1/2 rho C_d a |u| u  u the fluid's velocity minus the cell's
//   medium  support   k f V_c, upward    k the volume's bearing modulus
//   flow    drag      as for a fluid     no buoyancy: air's is negligible
//   every   damping   c f V_c u          c the field's damping
//
// a is f V_c / V times the body's area across u: pi r^2 for a sphere, the
// projected face area for a box, the area of the sphere of equal volume for a
// hull. A body wholly inside a uniform flow therefore feels the textbook
// 1/2 rho C_d A |u| u. Each cell's force acts at its centre, so torques follow: a
// tilted slab rights itself, and a spinning ball is slowed. Drag and damping are
// integrated implicitly in their own strength (scaled by (1 - e^-x) / x, x the
// body's drag rate times fixed_dt), so a light body in a dense fluid never
// overshoots the fluid's velocity. "Upward" is against gravity (+z in a world
// without gravity).
//
// Damping is the linear resistance quadratic drag lacks at low speed. For a medium
// it is the main resistance to sinking in. For water it stands for the energy a
// floating body loses making waves: without it a body bobbing at a few cm/s feels
// almost no drag and bobs for a long time. A heave damped near critically needs c
// about 2 sqrt(rho g A m) / V_sub (A the waterplane area, m the mass, V_sub the
// volume under water); 1e4 N s/m^4 suits a body of a metre or so.
//
// That damping acts in every direction, so it also holds a body to the water's velocity
// sideways: the body follows the waves' orbital motion and the current, but resists any drift
// through the water as strongly as it resists heaving (a 1 m crate in a 10 m/s wind drifts about
// a millimetre a second). A fluid with `inertial` set is moved sideways as a real one moves a
// small floating body: its damping acts on the vertical alone (the heave it stands for), and
// sideways the body is pushed by the water's acceleration over its immersed volume (the
// Froude-Krylov force, rho V du/dt at the immersed part's centroid, du/dt read from the water's
// velocity a step either side) and drags against the water's velocity (C_d). So it rides the
// waves and drifts with the current, and a wind blows it through the water at the speed where
// the air's drag on it balances the water's.
//
// A medium supports elastically. A flat footprint A sunk d metres into a medium of
// modulus k carries k A d (a Winkler foundation, Bekker's n = 1), and the medium
// pushes the body back out as it rises. A medium that keeps ruts is later work.
//
// A fluid may have a free surface (field::surface, fluid_surface.h): water up to the
// surface's height and air above it, inside the volume's box. A cell's immersed part is
// then also clipped below the surface over its centre, at the world's field time, and its
// drag and damping use the surface's water velocity (the waves' orbital motion, a current)
// in place of the velocity volumes. Make the box tall enough to hold the crests.
//
// A flow may have a wind (field::wind, flow_velocity.h): its velocity, changing in space and
// time, in place of the velocity volumes, read at the world's field time once per body: at its
// centre of mass (the gusts vary over tens of metres; so the wind costs O(bodies) a step), times
// the wind's height profile at each cell, and only above the wind's base (the water or the
// ground: a cell is clipped to it as to a free surface). Each cell's drag is taken against it;
// the air is not counted in field_load::immersed_volume.
// A vehicle's aero drag reads it at the vehicle's centre of mass (vehicle.h), as it reads a
// flow's velocity volumes.
//
// A body with a hull (body_desc::hull, hull.h) is pushed by fluids on its hull's triangles in
// place of its cells: hydrostatic pressure split at the waterline, friction, pressure drag and
// slamming. Media and flows still push on its cells.
//
// A fluid with a free surface may have a wake (field::wake, field_wake below; OCEAN-AND-WEATHER-
// ROADMAP.md C5, D6): the waves its floating bodies make, which push the others. It is off by
// default, and a world without one steps exactly as before. With one, the field keeps a square
// window of cells around the bodies in its water (cvc::ocean::ewave's exact linear step) and one
// wave state in it per body that makes waves: each awake body's immersed columns over the window
// (immersion.h), cut against what it floats in (the surface and the others' waves, so a body
// riding another's wave does not send it on again), are its source,
// dh = -(zeta(t) - zeta(t - dt)). Every body
// reads the sum of the others' waves, never its own: their height adds to the surface's under its
// cells and hull, their velocity (grad phi sideways, dh/dt up) to the water's, and an inertial
// fluid's Froude-Krylov push takes their acceleration (-g grad h), the velocity and the push
// decaying with the depth they are read at (field_wake below). With a shore (wake_shore below)
// the waves reflect off the coast and soak into its beaches. A body's own waves read back
// would cancel its heave's restoring force as it moves (its source lowers the water it presses
// down, at once, and the waves take a while to refill it) and price its wave-making twice (the
// hull's drag stands for it). So boats feel each other's wakes and the debris' rings, and the
// debris the boats'. The window and its states are world state: saved (with every wake parameter,
// so a world with another wake refuses the state) and hashed. restore() reads every wake's
// section whole before any wake takes it, so a failure in it leaves the wakes as they were (the
// world's restore as a whole is not transactional: the engine and the sections before the
// wakes' are taken first). The section is versioned only by its magic (CVCWAKE2, which adds each
// wake's shore digest and the depths its motion is kept at; CVCWAKE1 states, from before, restore
// as wakes without shores). What reads the water at a point (world::water_at) reads the wake's
// waves too, at the point's depth: a boat's propeller and rudder the others' waves, never its own.
//
// Sleeping bodies are not pushed: the forces were in balance when the body fell
// asleep. add_field and set_field wake the bodies inside the field's box, and a wake wakes a body
// its waves reach (more than a millimetre high where it sits).
//
// Standard library only, like the rest of cvc/physics: cvc::volume is declared,
// not included.

#ifndef __CVC_PHYSICS_FIELD_H__
#define __CVC_PHYSICS_FIELD_H__

#include <array>
#include <cstddef>
#include <cstdint>
#include <cvc/physics/flow_velocity.h>
#include <cvc/physics/fluid_surface.h>
#include <cvc/physics/types.h>
#include <memory>
#include <vector>

namespace cvc {

class volume;

namespace physics {

enum class field_kind : std::uint8_t {
  fluid,  // water, oil: buoyancy and drag
  medium, // snow, mud, sand: support as a body sinks in, and damping
  flow,   // moving air: drag only
};

// A wake's coast (field_wake::shore; OCEAN-AND-WEATHER-ROADMAP.md C6): where its window's waves
// meet land. mask(x, y) is 1 in open water and 0 on land, a wall the waves reflect off crest for
// crest (cvc::ocean::ewave's no-flux wall) and never run over; between them is a beach, which
// absorbs at 10 (1 - m) /s. Like a fluid_surface it must be deterministic and the same on every
// platform, and immutable once a field holds it; digest() names everything mask() depends on,
// and a world folds it into the wake's section of save() and state_hash(). shore_mask.h has one
// made from a shore grid (cvc::world::water_bodies), as the drawn wake window masks.
class wake_shore {
public:
  virtual ~wake_shore() = default;
  virtual double mask(double x, double y) const = 0;
  virtual std::uint64_t digest() const = 0;
};

// A fluid's wake (field::wake): the window its floating bodies' waves are stepped on. `cells` 0 is
// no wake (the default). The window (cells x cells of `cell` metres, one water depth) sits on a
// grid of whole cells from the world's origin where it holds the most bodies in the water, a boat
// (a body with a hull) counting as four: a proposal is the bodies within the window's inner
// square (the window less its sponge) round a body, and the window takes the best proposal over
// what it holds only when that holds more. Every boat proposes, and of the other bodies the first
// (by id) in each occupied square of a grid as wide as that inner square, at most 64 proposals in
// all, so placing the window costs at most 64 x the bodies in the water a wave step. It centres on
// what it holds and moves by whole cells once that is cells / 8 off its centre, the waves going
// with it and the strip that enters flat. Bodies in the water outside it make and feel no waves
// (wake_info::outside counts them): two groups farther apart than the window cannot share it. The
// waves step every `period` seconds of world time, in whole physics steps (at least one), and the
// bodies read them at every step. A sponge at the window's edge takes the waves that leave. With
// no `shore` the window is open water: a wake runs on over land, where nothing floats to feel it,
// and does not reflect off the coast. With one, the window is masked from it wherever it goes
// (C6): its waves reflect off steep shores crest for crest, soak into beaches and never run over
// land, and a pond inside the window reflects off all its banks (the window's step is then
// cvc::ocean::ewave's no-flux step, one substep of the waves' step). At most `sources` awake
// bodies make waves, each holding a slot: a body that falls asleep, leaves the window or leaves
// the world gives its slot up and its waves fade on; a body awake in the window without one takes
// a free slot, boats first, then the nearest the window's centre (by id among equals).
//
// The waves' motion decays with depth as in linear theory, each wavenumber k on its own
// (e^{-k d} in deep water; cosh and sinh over the window's depth otherwise), d below the water's
// top where it is read: a body deep down feels less of them than one afloat. Their height is the
// surface's and does not decay.
//
// Memory and cost: each body that makes waves holds a state of 80 cells^2 bytes (320 KiB at 64,
// 1.25 MiB at 128), at most sources + 4 states (the 4 fading), and saves 32 cells^2 bytes of it
// (so a save and a state_hash() grow by that each). A wave step costs about one
// cvc::ocean::ewave step per state with waves in it: 0.16-0.22 ms at 64^2, 0.6-1.0 ms at 128^2
// (ocean_ewave_test), every `period`; the reads are bilinear lookups. Depth adds, per state, its
// motion at 1 to 5 depths below the top (cells / 2, 1, 2, 4 and 8 cells down, as deep as the
// bodies in the window reach): 40 cells^2 bytes and about one and a half inverse FFTs a depth,
// two forward FFTs a state. The limits below keep a window at most 128 cells and 16 sources: 20
// states of 1.25 MiB (and 3.1 MiB of depths at the deepest), wave steps of about 20 ms at worst.
struct field_wake {
  int cells = 0;            // cells a side (0: no wake; else a power of two, 8 .. 128)
  double cell = 1.0;        // m: a cell's side (0.05 .. 100)
  double depth = 0.0;       // m: the window's one water depth (<= 0: deep water; at most 10 km)
  double period = 1.0 / 30; // s between two of the waves' steps (whole physics steps; (0, 10])
  double damping_k2 = 2e-3; // m^2/s: the short waves decay as exp(-damping_k2 |k|^2 t)
  double damping = 0.02;    // 1/s: and every wave as exp(-damping t)
  double sponge_cells = 8;  // the edge band's width (cells, at most cells / 2)
  double sponge_rate = 2.0; // 1/s: its damping at the edge
  int samples = 2;          // sub-columns a side in each cell for a body's immersed depth (1 .. 8)
  int sources = 8;          // the most bodies that make waves (1 .. 16)
  // The coast the window is masked from (wake_shore above; null: open water, the default). Two
  // wakes with shores of one digest are the same wake.
  std::shared_ptr<const wake_shore> shore;
  bool on() const { return cells > 0; }
  // The shore's digest, 0 for none.
  std::uint64_t shore_digest() const { return shore ? shore->digest() : 0; }
  friend bool operator==(const field_wake &a, const field_wake &b) {
    return a.cells == b.cells && a.cell == b.cell && a.depth == b.depth && a.period == b.period &&
           a.damping_k2 == b.damping_k2 && a.damping == b.damping &&
           a.sponge_cells == b.sponge_cells && a.sponge_rate == b.sponge_rate &&
           a.samples == b.samples && a.sources == b.sources && !a.shore == !b.shore &&
           a.shore_digest() == b.shore_digest();
  }
  friend bool operator!=(const field_wake &a, const field_wake &b) { return !(a == b); }
};

struct field {
  field_kind kind = field_kind::fluid;
  // The field's region and strength, in world metres. The field acts inside this
  // volume's bounding box only. For a fluid or a flow the values are mass density
  // (kg/m^3). For a medium they are the bearing modulus k (N/m^3): the support per
  // cubic metre of a body sunk into it.
  std::shared_ptr<const cvc::volume> volume;
  // The stuff's velocity (m/s) along world x, y and z, one volume per component on
  // a grid of its own. Give all three or none; with none, the stuff is still.
  // Outside a component's bounding box that component is 0.
  std::array<std::shared_ptr<const cvc::volume>, 3> velocity;
  double drag_coefficient = 1.0; // fluid, flow: C_d
  double damping = 0.0;          // every kind: c, N s/m^4
  // fluid: damp the vertical only, and move a body sideways by the water's acceleration and its
  // drag (above). Off: the damping acts in every direction, as it always has.
  bool inertial = false;
  // medium: the friction a vehicle's tire finds in it (snow ~0.3, mud ~0.4), in place of the
  // ground's; 0 keeps the ground's. A vehicle's wheels also sink into a medium and ride on it
  // (vehicle.h).
  double traction = 0.0;
  // medium: whether it keeps the ruts wheels press into it (plastic snow, mud). A wheel then
  // leaves its sinkage behind it, a rut map at rut_cell (m) resolution over the field's box; a
  // wheel in a rut sits at least as deep, and the compaction resistance counts only snow
  // compacted further (b k (z^2 - rut^2) / 2), so a second pass in the tracks sinks no deeper and
  // rolls with less drag. The ruts are world state: saved, restored and hashed. set_field
  // starts the medium fresh.
  bool ruts = false;
  double rut_cell = 0.05;
  // fluid: its free surface (fluid_surface.h): the water's top and velocity, in place of the
  // box's top and the velocity volumes. Null keeps the box's flat top. The world reads it from
  // the step's thread and folds its digest into state_hash(), so it must not change while a
  // field holds it; set_field with a new surface to change the sea.
  std::shared_ptr<const fluid_surface> surface;
  // flow: its wind (flow_velocity.h): the air's velocity in space and time, in place of the
  // velocity volumes. Null keeps them. Folded into state_hash() and held immutable, as `surface`.
  // A flow with a wind may have no `volume`: it is then air of 1.225 kg/m^3 everywhere (10^7 m
  // each way), the scene-wide wind.
  std::shared_ptr<const flow_velocity> wind;
  // fluid with a free surface: the waves its bodies make, pushing the others (field_wake above;
  // off by default). world::set_field with the same wake keeps its waves; another starts afresh.
  field_wake wake;
};

// What a fluid's wake is now (world::field_wake_info).
struct wake_info {
  bool placed = false;          // the window is down (once some body is in the water)
  double x0 = 0.0, y0 = 0.0;    // its cell (0, 0)'s point, world metres
  double cell = 0.0;            // m
  int cells = 0;                // a side
  int windows = 0;              // the bodies' wave states in it, those fading out included
  std::vector<body_id> sources; // the bodies making waves now, in id order
  int outside = 0;              // bodies in the water outside the window (at its last wave step)
};

// A medium's ruts (world::field_ruts): the depth (m) the surface has been pressed down, per cell
// of rut_cell over the field's box in x and y, row-major from (x0, y0), row 0 at min y.
struct rut_map {
  double x0 = 0.0, y0 = 0.0, cell = 0.0;
  int cols = 0, rows = 0;
  std::vector<float> depth;
  // The depth at (x, y), or 0 outside the map.
  double at(double x, double y) const {
    if (cols <= 0 || rows <= 0 || !(cell > 0.0))
      return 0.0;
    const double fc = (x - x0) / cell, fr = (y - y0) / cell;
    if (!(fc >= 0.0 && fr >= 0.0 && fc < cols && fr < rows))
      return 0.0;
    return depth[static_cast<std::size_t>(static_cast<int>(fr)) * cols + static_cast<int>(fc)];
  }
};

// A field handle. Never reused: a removed field's id stays invalid.
struct field_id {
  std::uint32_t value = 0xffffffffu;
  bool valid() const { return value != 0xffffffffu; }
  friend bool operator==(field_id a, field_id b) { return a.value == b.value; }
  friend bool operator!=(field_id a, field_id b) { return a.value != b.value; }
  friend bool operator<(field_id a, field_id b) { return a.value < b.value; }
};

// What the fields did to one body in the last step.
struct field_load {
  vec3 force;                   // N, the sum over the body's cells
  vec3 torque;                  // N m, about the body's centre of mass
  double immersed_volume = 0.0; // m^3, summed over the fields
};

// The water at a point (world::water_at): the first fluid field, in id order, whose box holds the
// point in x and y and reaches below it. A boat's propeller and rudder read it too. With a wake
// (field::wake) its waves' height is in the top and their motion, at the point's depth, in the
// velocity: all the bodies' waves, or for a body that reads it (a boat), the others'.
struct water_sample {
  bool wet = false;     // under the fluid's top and above its bottom, at a density > 0
  double depth = 0.0;   // m below the top: the free surface's height, or the box's top
  double density = 0.0; // kg/m^3
  vec3 velocity;        // m/s, the water's: the surface's, or the velocity volumes'
};

} // namespace physics
} // namespace cvc

#endif // __CVC_PHYSICS_FIELD_H__
