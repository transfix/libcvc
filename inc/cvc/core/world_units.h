/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#ifndef __CVC_WORLD_UNITS_H__
#define __CVC_WORLD_UNITS_H__

#include <cvc/core/namespace.h>
#include <mutex>
#include <string>

namespace cvc {

// ---------------
// cvc::world_units
// ---------------
// The authoritative notion of *how long a world-space unit is* and *which
// measurement regime the user reads results in*, separated from the geometry
// and from the physics. It is to space what world_clock is to time.
//
// Four frames are easy to conflate and must not be:
//
//   local / authoring - a graphic's coordinates in its own units. A CAD part
//     authored in millimetres, a terrain tile in feet, a mesh in arbitrary
//     numbers. Brought into world space by that node's transform (and, at
//     import time, by a per-model authoring scale -- see WORLD_UNITS_API.md).
//   world  - the dimensionless doubles the scene graph, geometry, volumes and
//     the physics integrator all share. One world unit is exactly
//     `metres_per_world_unit` canonical metres. Historically this was 1.0 by
//     unspoken convention; this class makes it explicit.
//   canonical SI - metres, kilograms, seconds and their derivatives. The ONE
//     representation every stored statistic and every physics quantity lives
//     in. A digital twin whose numbers are "provably physically accurate" is
//     one that keeps a single SI source of truth and never a second copy.
//   display regime - SI (metric) or imperial. A presentation and interop
//     policy layered on top of the SI store; it changes how a number is shown
//     and labelled, never how it is stored.
//
// The discipline that buys correctness is the same one world_clock uses for
// time: store the canonical form and DERIVE the display form on read. World
// state is kept in SI and converted to km / miles / pounds only at the moment
// it is shown or handed across an interop boundary. Storing imperial would
// reintroduce rounding drift and make a value read back from a distributed
// state tree ambiguous about which regime it was written in -- exactly the
// class of silent error this type exists to prevent.
//
// Why this feeds physics and the digital twin specifically:
//
//   * a physics engine that works in SI/MKS (Jolt, and the SI convention the
//     nav stack already assumes) exchanges positions, velocities and forces
//     across its boundary in canonical metres / (m/s) / newtons with NO
//     scaling -- only `metres_per_world_unit` is applied when importing the
//     scene's world-space geometry into the engine.
//   * every readout of the world -- a clicked terrain coordinate, a measured
//     span, a reported speed or force -- converts from the same SI store
//     through the same table, so a metre is a metre everywhere and a change of
//     regime re-labels every number at once.
//
// Like world_clock, this class is deliberately NOT wired to cvc::state or
// cvc::app: it has no dependency on either, so it is usable from a bare
// physics loop, from a test, or from a renderer with no cluster attached.
// Making the regime application-wide is a matter of the application publishing
// it into its per-app state tree (e.g. under world.units.*) and mirroring it
// back into an instance of this class -- the same layer-on-top pattern the
// examples use to drive world_clock. That binding lives in the application, not
// here.
//
// Thread-safe: every accessor and mutator takes an internal lock, then does the
// pure arithmetic outside it.
//
class world_units {
public:
  // The measurement system a value is presented in. The canonical store is
  // ALWAYS SI, whatever this is; imperial is display and interop only.
  enum class system {
    si,       // metric: metre, kilogram, second, m/s, N, J, ...
    imperial, // customary: foot/mile, pound, second, mph, lbf, ft-lbf, ...
  };

  // The physical dimension of a scalar, used to pick the conversion factor and
  // the unit label. Everything derives from length / mass / time; the derived
  // members are named for the quantities a digital twin actually reports.
  enum class dimension {
    length,       // metre        <-> foot / mile
    mass,         // kilogram     <-> pound
    time,         // second       <-> second (identity; both systems use seconds)
    velocity,     // metre/second <-> mile/hour
    acceleration, // metre/second^2 <-> foot/second^2
    force,        // newton       <-> pound-force
    energy,       // joule        <-> foot-pound-force
    angle,        // radian       <-> degree (degrees in BOTH regimes: the
                  //   near-universal display convention; radian is the
                  //   canonical/compute unit)
  };

  struct config {
    // Presentation regime. Canonical storage is always SI regardless.
    system regime = system::si;

    // The length in canonical SI metres of ONE world-space unit -- the factor
    // that pins the scene's dimensionless doubles to the metre base. 1.0 means
    // "one world unit is one metre"; 1000.0 means the world is authored in
    // kilometres; 0.3048 means it is authored in feet. Must be finite and > 0:
    // a zero or negative scale does not "pause" anything the way a zero clock
    // scale does, it silently corrupts every coordinate, so it is rejected
    // rather than clamped (see set_metres_per_world_unit).
    double metres_per_world_unit = 1.0;
  };

  world_units();
  explicit world_units(config cfg);

  // --- regime -------------------------------------------------------------

  system regime() const;
  void set_regime(system s);

  // --- world <-> canonical SI --------------------------------------------

  // The length in metres of one world-space unit.
  double metres_per_world_unit() const;

  // Rejects a non-finite or non-positive scale by throwing
  // cvc::unsupported_exception. This is a deliberate departure from
  // world_clock::set_scale, which clamps a bad rate to 0 (a paused clock is a
  // sane state). A bad length scale has no sane fallback -- it would make every
  // downstream metre wrong while looking finite -- and for training and
  // presentation a quietly-wrong metre is worse than a thrown exception.
  void set_metres_per_world_unit(double metres);

  // A scalar length expressed in world units <-> canonical SI metres.
  double world_to_metres(double world_length) const;
  double metres_to_world(double metres) const;

  // --- canonical SI <-> the active display regime ------------------------

  // Convert a canonical-SI quantity of the given dimension to the number a user
  // of the active regime expects, in that regime's BASE unit (metre/foot,
  // kilogram/pound, second, m/s / mph, ...), and back. No magnitude-based
  // promotion happens here; use format() for that.
  double to_display(double si_value, dimension d) const;
  double from_display(double display_value, dimension d) const;

  // The base unit label for a dimension in the active regime, e.g. "m"/"ft",
  // "kg"/"lb", "m/s"/"mph". ASCII only, so it is safe in logs and tests.
  std::string unit_symbol(dimension d) const;

  // A canonical-SI value rendered for display: the value in the active regime
  // and the unit it is in. Lengths are promoted by magnitude -- metres to km at
  // 1000 m, feet to miles at 5280 ft -- so a terrain span or a clicked distance
  // reads naturally; every other dimension uses its base unit. A non-finite
  // input is passed through unpromoted rather than misclassified.
  struct measurement {
    double value = 0.0;
    std::string unit;
  };
  measurement format(double si_value, dimension d) const;

  // --- coordinate mapping ------------------------------------------------

  // A point in WORLD-space coordinates converted to its real-world position in
  // the active regime. This is the tail of the "click on terrain, tell me
  // where in kilometres or miles" path: the GL layer resolves a pick to a
  // world-space (x,y,z) -- through the picked node's transform, so it is in the
  // graphic's own coordinate frame -- and this maps it to canonical metres via
  // metres_per_world_unit and then to the display regime. All three components
  // share ONE unit, chosen from the largest magnitude, so a coordinate never
  // mixes km on one axis with metres on another.
  struct coordinate {
    double x = 0.0, y = 0.0, z = 0.0;
    std::string unit;
  };
  coordinate world_point_to_real(double wx, double wy, double wz) const;

private:
  mutable std::mutex _mutex;
  config _cfg;
};

// ---------------------------------------------------------------------------
// cvc::units -- the one authoritative table of exact SI definitions
// ---------------------------------------------------------------------------
// The international-foot family and derived customary units, defined exactly in
// SI. Kept in one place so world_units and every consumer convert identically:
// the single source of truth the "provably accurate" claim rests on. All are
// exact by definition of the international system of customary units.
namespace units {

inline constexpr double metres_per_foot = 0.3048;         // exact
inline constexpr double metres_per_mile = 1609.344;       // exact (5280 ft)
inline constexpr double feet_per_mile = 5280.0;           // exact
inline constexpr double kilograms_per_pound = 0.45359237; // exact (avoirdupois)
inline constexpr double standard_gravity = 9.80665;       // exact, m/s^2 (defines lbf)
inline constexpr double newtons_per_pound_force =
    kilograms_per_pound * standard_gravity; // exact, = 4.4482216152605 N
inline constexpr double joules_per_foot_pound =
    newtons_per_pound_force * metres_per_foot; // exact, foot-pound-force
inline constexpr double metres_per_second_per_mph =
    metres_per_mile / 3600.0; // exact, = 0.44704 m/s
inline constexpr double pi = 3.14159265358979323846;
inline constexpr double degrees_per_radian = 180.0 / pi;
inline constexpr double radians_per_degree = pi / 180.0;

// Length promotion thresholds (in metres), fixed conventions.
inline constexpr double metres_per_kilometre = 1000.0; // exact

} // namespace units

} // namespace cvc

#endif // __CVC_WORLD_UNITS_H__
