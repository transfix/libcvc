# World Units API Reference (`cvc::world_units`)

> **Scope: this document covers the core `cvc::world_units` class — the
> canonical unit base and display regime.** It is to space what
> [`cvc::world_clock`](../inc/cvc/core/world_clock.h) is to time: a small,
> thread-safe, self-contained class with no dependency on `cvc::state` or
> `cvc::app`, usable from a bare physics loop, a test, or a renderer. Like
> `world_clock`, it holds an internal mutex and is therefore **non-copyable and
> non-movable** — update it in place through the setters, do not reassign it. Making the
> regime application-wide (publishing it into the per-app state tree) and the
> scene-graph consumers that turn a mouse click into a real-world coordinate are
> described here as integration patterns, but the class itself owns only the
> unit math. That math goes through a **unit registry**
> ([`cvc::unit_registry`](../inc/cvc/core/unit_registry.h)): the built-in SI and
> imperial units plus any unit an application drops in, linear, affine or an
> arbitrary function. The registry is copyable; each `world_units` owns one.

## Table of Contents

- [Overview](#overview)
- [The frames](#the-frames)
- [Why SI is the canonical base](#why-si-is-the-canonical-base)
- [API reference](#api-reference)
  - [`config`](#config)
  - [`system` and `dimension`](#system-and-dimension)
  - [World ↔ canonical metres](#world--canonical-metres)
  - [Canonical SI ↔ the display regime](#canonical-si--the-display-regime)
  - [Formatting and coordinates](#formatting-and-coordinates)
  - [Display units per dimension](#display-units-per-dimension)
  - [The unit registry: built-in and custom units](#the-unit-registry-built-in-and-custom-units)
  - [SI ↔ the normalized frame (`cvc::nav`)](#si--the-normalized-frame-cvcnav)
  - [`cvc::units` — the exact conversion table](#cvcunits--the-exact-conversion-table)
- [Making the regime application-wide](#making-the-regime-application-wide)
- [Feeding the physics engine](#feeding-the-physics-engine)
- [Clicking terrain: world point → real-world coordinate](#clicking-terrain-world-point--real-world-coordinate)
- [Integration roadmap (follow-ups)](#integration-roadmap-follow-ups)
- [Worked example](#worked-example)
- [Python (pycvc)](#python-pycvc)

## Overview

Historically every coordinate in libcvc was a dimensionless `double`: geometry
vertices, bounding boxes ("object space"), scene-graph transforms and the nav
integrator all shared raw numbers with no attached physical meaning, and "one
world unit is one metre" was an unspoken convention that lived only in comments
and example constants. `cvc::world_units` makes that convention **explicit and
authoritative**:

- **`metres_per_world_unit`** — the physical length, in canonical SI metres, of
  one world-space unit. This pins the scene's doubles to the metre base.
- **`regime`** — whether results are *presented* in SI (metric) or imperial.
  The canonical store is always SI; imperial is a display and interop policy.

From those two settings it derives every conversion a digital twin needs —
lengths, masses, velocities, accelerations, forces, energies, angles, power,
pressure, temperature, curvature, charge, energy per distance and more —
through **one** unit registry, so a metre is a metre everywhere and a change
of regime re-labels every number at once. Three further pieces make the API
complete for the physics tier and for nav:

- **per-dimension display units** — show velocity in knots or charge in mAh
  while the rest of the regime stays as it is;
- **custom units** — register a unit (linear, affine such as °F, or any pair
  of functions such as dBm) or a whole new dimension at run time;
- **the normalized frame** — convert SI quantities and points to and from the
  frame `cvc::nav`'s kernels and trained coefficients work in.

## The frames

Five spatial frames are easy to conflate and must not be:

| Frame | What it is | Unit |
|-------|-----------|------|
| **local / authoring** | a graphic's coordinates in its own coordinate system | the model's own (mm, ft, arbitrary) |
| **world** | the doubles the scene graph, geometry, volumes and physics share | 1 unit = `metres_per_world_unit` metres |
| **canonical SI** | the single source of truth for stored statistics and physics | metre, kilogram, second, … |
| **display regime** | how a number is shown and labelled | SI or imperial, with per-dimension overrides |
| **normalized (nav)** | what `cvc::nav` kernels, learned coefficients and `.cvcnav` weights use | `(world − centre) · S` |

A graphic is brought from its **local** frame into the **world** frame by its
node transform (and, at import time, by a per-model authoring scale — see the
[roadmap](#integration-roadmap-follow-ups)). `world_units` owns the last two
hops: **world → canonical SI → display regime**.

The discipline that buys correctness is the same one `world_clock` uses for
time — **store the canonical form and derive the display form on read**. World
state is kept in SI and converted to km / miles / pounds only at the moment it
is shown or handed across an interop boundary. Storing imperial would
reintroduce rounding drift and make a value read back from a distributed state
tree ambiguous about which regime wrote it — exactly the class of silent error
this type exists to prevent.

## Why SI is the canonical base

- The nav stack already assumes metres and seconds by convention
  (`sim_world` snapshots document "pose in WORLD metres", "speed world m/s"),
  and the material layer already hard-codes metres and `1/m`.
- A physics engine that works in SI/MKS (Jolt, and the SI convention above)
  exchanges positions, velocities and forces across its boundary in canonical
  metres / (m/s) / newtons with **no scaling** — only `metres_per_world_unit`
  is applied when importing world-space geometry into the engine.
- "Provably physically accurate statistics" requires a single authoritative
  representation. SI is it; imperial never stores, only displays.

## API reference

Header: [`inc/cvc/core/world_units.h`](../inc/cvc/core/world_units.h).
Namespace: `cvc`. Thread-safe: every accessor locks an internal mutex, then does
the arithmetic outside the lock.

### `config`

```cpp
struct config {
  system regime = system::si;         // presentation only; storage is always SI
  double metres_per_world_unit = 1.0; // SI metres per one world-space unit; finite > 0
};

world_units();                        // SI, 1.0 m per world unit
explicit world_units(config cfg);     // throws cvc::unsupported_exception on bad scale
```

A **zero, negative or non-finite** `metres_per_world_unit` is rejected by
throwing `cvc::unsupported_exception`, both at construction and from
`set_metres_per_world_unit`. This is a deliberate departure from
`world_clock::set_scale`, which *clamps* a bad rate to zero: a paused clock is a
sane state, but a bad length scale has no sane fallback — it would make every
downstream metre wrong while still looking finite. For training and
presentation, a quietly-wrong metre is worse than a thrown exception, so it
fails loudly.

### `system` and `dimension`

```cpp
enum class system { si, imperial };

enum class dimension {
  length,       // metre        <-> foot / mile
  mass,         // kilogram     <-> pound
  time,         // second       <-> second   (identity)
  velocity,     // metre/second <-> mile/hour
  acceleration, // metre/second^2 <-> foot/second^2
  force,        // newton       <-> pound-force
  energy,       // joule        <-> foot-pound-force
  angle,        // radian       <-> degree   (degrees in BOTH regimes)
  // appended with the unit registry; the members above keep their values
  power, pressure, temperature, area, volume, angular_velocity, curvature,
  frequency, electric_current, voltage, electric_charge, density, torque,
  energy_per_length,
};

static const char *dimension_name(dimension d); // the registry name, e.g. "energy_per_length"
```

`time` is seconds in both systems. `angle` is shown in degrees in both regimes
(the near-universal display convention); the radian is the canonical/compute
unit.

Each dimension has a canonical SI unit, a default display unit per regime and a
**length exponent** (the power of length it carries, used by the normalized
frame):

| dimension | SI | SI regime shows | imperial shows | length exp. |
|---|---|---|---|---|
| length | m | m (km ≥ 1000 m) | ft (mi ≥ 5280 ft) | 1 |
| mass | kg | kg | lb | 0 |
| time | s | s | s | 0 |
| velocity | m/s | m/s | mph | 1 |
| acceleration | m/s^2 | m/s^2 | ft/s^2 | 1 |
| force | N | N | lbf | 1 |
| energy | J | J | ft-lbf | 2 |
| angle | rad | deg | deg | 0 |
| power | W | W | hp | 2 |
| pressure | Pa | Pa | psi | −1 |
| temperature | K | degC | degF | 0 |
| area | m^2 | m^2 | ft^2 | 2 |
| volume | m^3 | m^3 | ft^3 | 3 |
| angular_velocity | rad/s | deg/s | deg/s | 0 |
| curvature | 1/m | 1/m | 1/ft | −1 |
| frequency | Hz | Hz | Hz | 0 |
| electric_current | A | A | A | 0 |
| voltage | V | V | V | 0 |
| electric_charge | C | C | C | 0 |
| density | kg/m^3 | kg/m^3 | lb/ft^3 | −3 |
| torque | N*m | N*m | lbf*ft | 2 |
| energy_per_length | J/m | Wh/km | Wh/mi | 1 |

The length exponent counts length only: mass and time are the same in the
world and normalized frames, so force (kg·m/s²) has exponent 1 and energy
(kg·m²/s²) has 2.

### World ↔ canonical metres

```cpp
double metres_per_world_unit() const;
void   set_metres_per_world_unit(double metres);   // finite > 0, else throws

double world_to_metres(double world_length) const; // world_length * scale
double metres_to_world(double metres) const;        // metres / scale (scale > 0, safe)
```

### Canonical SI ↔ the display regime

```cpp
double to_display(double canonical_value, dimension d) const; // SI -> regime base unit
double from_display(double display_value, dimension d) const; // regime base unit -> SI
std::string unit_symbol(dimension d) const;                   // "m"/"ft", "kg"/"lb", ...
```

`to_display`/`from_display` use the regime's **base** unit (metre/foot,
kilogram/pound, m/s / mph, …). They do **not** promote by magnitude — use
`format` for that. SI is the identity for its own base units, so
`to_display(x, length)` in the SI regime returns `x`.

### Formatting and coordinates

```cpp
struct measurement { double value; std::string unit; };
measurement format(double canonical_value, dimension d) const;

struct coordinate { double x, y, z; std::string unit; };
coordinate world_point_to_real(double wx, double wy, double wz) const;
```

`format` renders a canonical-SI value for a human: lengths promote by
magnitude — metres → **km** at 1000 m, feet → **miles** at 5280 ft — while every
other dimension uses its base unit. A non-finite input is passed through
unpromoted rather than misclassified.

`world_point_to_real` is the tail of the click-to-coordinate path: it takes a
point in **world** coordinates, converts to canonical metres via
`metres_per_world_unit`, and returns the point in the active regime. All three
components share **one** unit, chosen from the largest magnitude, so a
coordinate never mixes km on one axis with metres on another.

### Display units per dimension

```cpp
void set_display_unit(dimension d, const std::string &symbol);  // throws unit_error if the
void set_display_unit(const std::string &dimension,            //   unit is not of that dimension
                      const std::string &symbol);
void clear_display_unit(dimension d);                           // back to the regime default
std::string display_unit(dimension d) const;                    // what it is shown in now

// every display call also takes a dimension by name, so custom dimensions work
double to_display(double canonical_value, const std::string &dimension) const;
double from_display(double display_value, const std::string &dimension) const;
std::string unit_symbol(const std::string &dimension) const;
measurement format(double canonical_value, const std::string &dimension) const;
```

An override replaces the regime's default for one dimension: velocity in `kn`
for a boat, volume in `L`, battery charge in `mAh`. A **length** override also
turns off the km / mile promotion in `format` and `world_point_to_real`: a user
who asked for nautical miles gets nautical miles at every magnitude. Overrides
survive a regime change (they are per dimension, not per regime). A custom
dimension with no override is shown in its SI unit.

### The unit registry: built-in and custom units

Header: [`inc/cvc/core/unit_registry.h`](../inc/cvc/core/unit_registry.h).

```cpp
cvc::unit_registry &registry();             // this world_units' own table
double to_si(double value, const std::string &symbol) const;   // pass-throughs
double from_si(double canonical, const std::string &symbol) const;
double convert(double value, const std::string &from, const std::string &to) const;
```

A unit is a symbol, the dimension it belongs to and a conversion to that
dimension's SI unit, of one of three kinds:

| kind | conversion | example |
|---|---|---|
| linear | `si = v · scale` | `ft`, `mph`, `kWh`, `psi` |
| affine | `si = v · scale + offset` | `degC`, `degF` |
| function | `si = to_si(v)`, `v = from_si(si)` | `dBm` over watts |

The built-in table holds SI and imperial units for every `dimension` above,
plus common extras: `km cm mm in yd mi nmi`, `g t oz`, `ms min h`,
`km/h ft/s kn`, `gn`, `kN`, `kJ Wh kWh BTU`, `kW dBm`, `kPa bar`,
`km^2 ha mi^2 acre`, `L gal`, `rpm`, `1/km`, `kHz MHz GHz`, `mA mV Ah mAh`,
`kWh/100km`, and the regime defaults in the table above. Every factor comes
from `cvc::units`, so a built-in is exact wherever the definition is.

Dropping in a unit or a dimension:

```cpp
cvc::unit_registry &r = wu.registry();
r.add_linear("furlong", "length", 201.168, "furlong");          // si per unit
r.add_affine("degRe", "temperature", 1.25, 273.15, "Réaumur");  // si = v*1.25 + 273.15
r.add_function("dBW", "power",                                   // any invertible pair
               [](double v) { return std::pow(10.0, v / 10.0); },
               [](double w) { return 10.0 * std::log10(w); });
r.add_dimension("data_rate", "bit/s");                           // a new dimension
r.add_linear("Mbit/s", "data_rate", 1e6);
wu.set_display_unit("data_rate", "Mbit/s");
wu.format(2.5e6, "data_rate");                                   // {2.5, "Mbit/s"}
```

The rules:

- A symbol is unique within a registry; adding an existing symbol throws
  `cvc::unit_error`. Re-declaring a dimension with the same SI symbol and
  exponent is a no-op; any other clash throws.
- A linear or affine scale must be finite and > 0; an offset must be finite.
  A function unit needs both callables; the registry does not check that they
  are inverses.
- `convert` between dimensions, or any unknown unit or dimension, throws
  `cvc::unit_error`. A conversion through a linear or affine unit never throws
  for a finite input, and inverts exactly (one multiply or divide, one add).
- Each `world_units` owns its own registry (a copy of the built-ins), so a unit
  one app registers never appears in another. `unit_registry::empty()` is a
  registry with nothing in it, for a closed custom table. The registry is
  copyable and thread-safe: registration locks; a conversion copies the unit's
  entry under the lock and converts outside it, so a function unit's callable
  runs unlocked.
- Introspection: `has_unit`, `unit(symbol)` (kind, scale, offset, name),
  `units(dimension)`, `dimension_of(symbol)`, `has_dimension`,
  `dimension(name)` (SI symbol, length exponent), `dimensions()`.

### SI ↔ the normalized frame (`cvc::nav`)

`cvc::nav` works in a normalized frame, `normalized = (world − centre) · S`,
whose scale `S` and centre come from `sim_world::config` (`scale`, `cx`, `cy`).
The kernels, the learned coefficients and the `.cvcnav` weights all live there
and must never see a unit conversion
([`NAV_UNITS_ROADMAP.md`](roadmap/NAV_UNITS_ROADMAP.md)). `world_units` converts
at the boundary:

```cpp
struct normalized_frame {
  double scale = 1.0;             // S: normalized units per WORLD unit, finite > 0
  double cx = 0, cy = 0, cz = 0;  // centre, in world units, finite
};
double to_normalized(double canonical_value, dimension d, const normalized_frame &f) const;
double from_normalized(double normalized_value, dimension d, const normalized_frame &f) const;
xyz si_point_to_normalized(double x_m, double y_m, double z_m, const normalized_frame &f) const;
xyz normalized_point_to_si(double nx, double ny, double nz, const normalized_frame &f) const;

// inc/cvc/nav/nav_units.h
cvc::world_units::normalized_frame cvc::nav::frame_of(const sim_world::config &c);
```

A quantity with length exponent `e` converts as
`normalized = si / metres_per_world_unit^e · S^e`: velocity (`e = 1`)
multiplies by `S / mpwu`, curvature (`e = −1`) by `mpwu / S`, area by
`(S / mpwu)²`; time, angle and mass pass through bit for bit. Note the time
axis is not rescaled: nav's normalized velocities are per second. A point
also subtracts the centre: `n = (m / mpwu − c) · S` and `m = (n / S + c) · mpwu`,
the same order as nav's own snapshot, so at one metre per world unit it
reproduces that arithmetic in double precision. An invalid frame throws
`cvc::unit_error`.

This is the seam a physics plant uses: Jolt reports a body's state in SI, the
adapter converts position, velocity and yaw rate to the normalized frame for
nav's policy, and converts the commanded acceleration back to SI for the engine.

```cpp
const auto f = cvc::nav::frame_of(cfg);
double v_n = wu.to_normalized(v_mps, dim::velocity, f);       // to the policy
double a_si = wu.from_normalized(a_n, dim::acceleration, f);  // to the engine
auto p = wu.normalized_point_to_si(o.x, o.y, 0.0, f);         // a snapshot pose in metres
```

### `cvc::units` — the exact conversion table

One authoritative table of exact SI definitions, so `world_units` and every
consumer convert identically. All are exact by definition of the international
customary system.

```cpp
namespace cvc::units {
  metres_per_foot          = 0.3048              // exact
  metres_per_mile          = 1609.344            // exact (5280 ft)
  feet_per_mile            = 5280.0
  kilograms_per_pound      = 0.45359237          // exact (avoirdupois)
  standard_gravity         = 9.80665             // m/s^2 (defines lbf)
  newtons_per_pound_force  = kg_per_pound * g    // = 4.4482216152605 N
  joules_per_foot_pound    = lbf * foot          // foot-pound-force
  metres_per_second_per_mph = mile / 3600        // = 0.44704 m/s
  degrees_per_radian, radians_per_degree, pi
  metres_per_kilometre     = 1000.0
  metres_per_nautical_mile = 1852.0              // exact
  watts_per_horsepower     = 550 * ft-lbf        // mechanical hp, ~745.7 W
  pascals_per_psi          = lbf / inch^2        // ~6894.76 Pa
  joules_per_btu           = 1055.05585262       // International Table BTU
  square_metres_per_acre   = 4046.8564224        // exact (international acre)
  cubic_metres_per_us_gallon = 0.003785411784    // exact (231 in^3)
}
```

## Making the regime application-wide

`world_units` is deliberately **not** wired to `cvc::state` or `cvc::app` — the
same decoupling `world_clock` keeps. "Application-wide" in modern libcvc means
"scoped to one injected `cvc::app`", and the canonical app-wide, observable,
networkable store is that app's per-app state tree. The application publishes the
regime into the tree and mirrors it back into a `world_units` instance, exactly
the layer-on-top pattern the examples use to drive `world_clock` under
`coast.clock.*`:

```cpp
// once, near app startup — seed defaults only if unset, so a script/scene file
// that preset them earlier wins (the "knob" idiom).
cvc::state &units = cvc::state::instance(app)("world.units");
if (units("system").value().empty())
  units("system").value(std::string("SI"));
if (units("metres_per_world_unit").value().empty())
  units("metres_per_world_unit").value(std::string("1.0"));

// build a world_units from the tree, and keep it in sync
cvc::world_units wu({
  units("system").value() == "imperial" ? cvc::world_units::system::imperial
                                         : cvc::world_units::system::si,
  units("metres_per_world_unit").value<double>(),
});
```

Store the regime as string **values**, not a `data()` blob — the state tree is
string-backed via `lexical_cast` and only value/child-structure changes
replicate across a distributed digital twin (a `data()` payload would silently
not propagate). Writing a numeric scale through `value<double>()` is fine (it
lexical_casts to the string form); the rule is to keep it a value node, not to
avoid numbers. For a multi-node twin,
mount the `world.units` subtree in the distributed session as `authoritative` on
the owner and `read_only` on viewers, and it will journal and replicate like any
other value.

## Feeding the physics engine

`cvc::physics` ([`PHYSICS_API.md`](PHYSICS_API.md)) wraps Jolt behind an
engine-agnostic world, and `world_units` is what its plant adapter reads:

- Jolt is SI/MKS (metres, kilograms, seconds; force in newtons), so positions,
  velocities and forces cross the engine boundary in canonical SI with **no
  scaling**. Only `metres_per_world_unit` is applied when importing world-space
  geometry into the physics world.
- Report engine outputs to the user with `format(canonical_value, dimension::force)`
  etc., so a change of regime converts every reported quantity through the same
  table.
- Keep conversions **out** of the bit-identical nav kernels (they are compiled
  without fast-math to match a Python reference); convert only at the boundary.

## Clicking terrain: world point → real-world coordinate

The stated goal — "click on terrain and get a precise coordinate in kilometres
or miles, in the graphic's own coordinate frame" — is a three-step pipeline, and
all three steps now exist in libcvc:

1. **pick** (GL) — `SceneRenderer::pickWorld(displayX, displayY, out[3])` casts a
   ray through a display pixel (VTK lower-left origin) and returns the first
   world-space hit, or false on a miss. A `vtkCellPicker` against the renderer.
2. **node-local** (optional) — `GraphicsNode::worldToLocal(world, local)` puts
   the hit into the picked graphic's OWN coordinate frame (and `localToWorld`
   goes back). The world→object inverse is cached lazily off the pose hot path.
3. **regime** — `world_units::world_point_to_real(x, y, z)` converts world → SI
   metres → the active regime, returning `{x, y, z, unit}` (e.g. km or mi).

`GraphicsNode::localPointToReal(local, world_units)` fuses steps 2→3, so once a
pick has resolved a hit to a local point, one call reports it in km/miles in the
graphic's own frame. See `src/cvcGL/test/cvcgl_world_units.cpp`.

## Mapping model dimensions to the regime

`cvc::model` carries `metres_per_source_unit` — the SI length of one unit of the
file's own coordinate system (1.0 = already metres, the default). `model::
extents_metres()` returns the model's footprint in canonical metres regardless
of authoring units; feed its corners to `world_units` to display a size or extent
in the active regime.

## Reliable dimensions through the transform chain

A graphic deep in the scene graph inherits a chain of local transforms from its
ancestors. `GraphicsNode` maintains each node's object-to-world matrix
(`getWorldTransform()`) current top-down as ancestors are added, moved or
removed, so you never walk or compose that chain yourself:

- **`getWorldBoundingBox()`** — the node's own box in world space (its local box
  pushed through the whole chain and re-fit to an AABB).
- **`getCombinedWorldBoundingBox()`** — the same for the node together with all
  its descendants.
- **`realDimensions(units, includeChildren = true)`** — the world box's extents
  converted through `world_units` into the active regime, three components
  sharing one unit (m/km or ft/mi). This is the "how big is this graphic,
  really" answer, taken reliably through the chain. (It reports the world
  **AABB** extents, so a rotation inflates them; the un-rotated authoring size is
  `getBoundingBox()` scaled.)

For this to be trustworthy the transform itself must not lose precision. Node
transforms are published to the string-backed state tree and read back, so
`setTransform`/`setPosition`/`setRotation`/`setScale` serialize with full
double precision (`max_digits10`), i.e. a real-world coordinate (a UTM easting,
a kilometre-scale offset) round-trips exactly rather than being truncated to six
significant figures. Dimensions and clicked coordinates taken through the chain
are therefore reliable to the last bit the doubles can hold.

## Integration roadmap (remaining follow-ups)

The picking, world↔local mapping, per-model authoring scale above, and the
Python bindings (see [Python (pycvc)](#python-pycvc) below) have landed. What is
left, each a self-contained follow-up:

1. **importer unit stamping** — teach the assimp importer to set
   `model::metres_per_source_unit` from source-file units (glTF metres, FBX
   `UnitScaleFactor`, OBJ unitless). Today every format imports at raw scale and
   the field defaults to 1.0, so a metres-authored file is already correct but an
   FBX in centimetres needs the caller to set the scale — a latent 100× twin bug
   until the importer stamps it.
2. **coordinate/label plumbing** — route `GridNode` tick labels, `BBoxNode`
   coordinate labels and `CameraController` distance/speed readouts through
   `world_units::format`, so every on-screen number carries the regime's unit.

## Worked example

```cpp
#include <cvc/core/world_units.h>

cvc::world_units wu;                        // SI, 1 m per world unit
using dim = cvc::world_units::dimension;

// A span measured in world units, reported for a human.
auto d = wu.format(wu.world_to_metres(3200.0), dim::length);
// d.value == 3.2, d.unit == "km"

// A physics force (newtons), shown to an imperial user.
wu.set_regime(cvc::world_units::system::imperial);
auto f = wu.format(1000.0, dim::force);
// f.value ~= 224.8, f.unit == "lbf"

// A clicked terrain point (world coordinates) as a real-world coordinate.
auto c = wu.world_point_to_real(2.0 * 1609.344, 0.0, 0.0);
// c.x == 2.0, c.unit == "mi"
```

## Python (pycvc)

The whole surface above is wrapped for Python through SWIG, next to
`world_clock`, so the training/twin layer drives the same unit base the C++
scene does. Because Python has no enums-as-types or out-parameters, the binding
reshapes a few things (all of it in `bindings/pycvc/pycvc.i`,
`pycvc_gl.i`, `pycvc_model.i`, `pycvc_volren.i`):

- the `system` / `dimension` enums and the `config` struct become **strings**:
  a regime is `"si"` | `"imperial"`; a dimension is its registry name
  (`"length"`, `"velocity"`, `"temperature"`, `"energy_per_length"`, … or one
  the application added);
- the scalar constructor is `world_units(metres_per_world_unit, regime="si")`;
- the dimension-keyed calls take the dimension string and carry a `_d`
  suffix: `to_display_d`, `from_display_d`, `unit_symbol_d`, `format_d`,
  `set_display_unit_d`, `clear_display_unit_d`, `display_unit_d`,
  `to_normalized_d`, `from_normalized_d`; the regime accessors are
  `regime_name()` / `set_regime_name()`; `to_si` / `from_si` / `convert` and
  the point helpers keep their names;
- `unit_registry` is wrapped as is, with `unit_registry_unit_info.kind_name()`
  (`"linear"` | `"affine"` | `"function"`) in place of the enum, and
  `add_function(symbol, dimension, to_si, from_si, name="")` taking two Python
  callables. A callable runs with the GIL re-acquired, from whatever thread
  converts; if it raises, the conversion raises `RuntimeError` naming the
  Python exception. `world_units.registry()` returns the instance's own
  registry and keeps the `world_units` alive;
- `normalized_frame` and `xyz` cross as `world_units_normalized_frame`
  (`.scale/.cx/.cy/.cz`) and `world_units_xyz` (`.x/.y/.z`);
- `format_d(...)` returns a `measurement` proxy with `.value` / `.unit`;
  `world_point_to_real(...)` returns a `coordinate` proxy with `.x/.y/.z/.unit`;
- the transform-chain and projection helpers on a node return plain Python
  values: `local_to_world([x,y,z])` / `world_to_local([x,y,z])` give a 3-list,
  `get_world_bounding_box()` / `get_combined_world_bounding_box()` give a
  `(minx..maxz)` 6-list, and `local_point_to_real([x,y,z], units)` /
  `real_dimensions(units)` give an `(x, y, z, unit)` tuple.

### The unit base

```python
import pycvc

wu = pycvc.world_units()               # SI, 1 m per world unit
wu = pycvc.world_units(1000.0, "si")   # 1 world unit == 1 km

# canonical SI -> display regime
wu.set_regime_name("imperial")
f = wu.format_d(1000.0, "force")       # f.value ~= 224.8, f.unit == "lbf"
c = wu.world_point_to_real(2 * 1609.344, 0, 0)  # c.x == 2.0, c.unit == "mi"

# per-dimension display units and custom units
wu.set_display_unit_d("velocity", "kn")
reg = wu.registry()
reg.add_function("dBW", "power", lambda v: 10 ** (v / 10), lambda w: 10 * math.log10(w))
reg.convert(0.0, "dBW", "dBm")                  # 30.0

# SI <-> nav's normalized frame
f = pycvc.world_units_normalized_frame()
f.scale, f.cx, f.cy = 0.05, 37.5, -12.0
wu.to_normalized_d(2.0, "velocity", f)          # 0.1 (normalized units per second)
p = wu.normalized_point_to_si(0.3, -0.7, 0.0, f)  # metres
```

The base is a stable **per-`app`** instance (never a process global), reached
the same way as the clock:

```python
app = pycvc.make_app()
app.world_units().set_regime_name("imperial")  # app-wide display regime
app.world_clock()                              # the app's simulation clock
```

### Model dimensions and the transform chain

```python
m = pycvc.load_model("part.obj")
m.metres_per_source_unit = 0.001               # a millimetre-authored file
em = m.extents_metres()                         # (minx..maxz) in canonical metres

sg = pycvc_gl.SceneGraph(app)
node = sg.add_child_geometry("root", "wing", geom)
node.local_to_world([0, 0, 0])                  # origin through the whole chain
node.local_point_to_real([0, 0, 0], app.world_units())  # -> (x, y, z, "km")
node.real_dimensions(app.world_units())         # "how big is this, really"
```

### Scene entry points

- **`SceneRenderer.pick_world(display_x, display_y)`** — a picked terrain/graphic
  point as a world `(x, y, z)` tuple (or `None` on a miss); feed it to
  `world_units.world_point_to_real` for a km/mile readout.
- **`SceneGraph.getGridNode()` / `getAxisNode()`** — the built-in reference grid
  and world axis (`GridNode` / `AxisNode`): bounds, per-plane colours,
  divisions, tick labels, axis length.
- **`SceneGraph.addLight(name)` / `light_node(name)`** — a `LightNode` with its
  kind as a string (`set_kind`/`kind_str`) and target/colour/world-position as
  tuples (`get_target` / `get_color` / `world_position`).
- **`VolRenNode`** (`add_volren` / `volren_node`) — the software raycaster as a
  scene node; `volume_real_dimensions(i, units)` /
  `volume_point_to_real(i, x, y, z, units)` report a rendered volume's real size
  and coordinates in the regime. The headless `raycaster` value types
  (`camera.eye/focal/up`, `render_settings.background`, …) cross as tuples.
- **`VolSliceNode`** (`add_volslice` / `volslice_node`) — the view-aligned slice
  renderer as a scene node (`setVolume` / `config` / `setConfig` / `tick`); its
  `cvc::volslice` value types are wrapped in `pycvc_volslice.i` (see
  [`docs/VOLSLICE_API.md`](VOLSLICE_API.md#python-pycvc)).

The Python contract is pinned by `bindings/pycvc/test_pycvc_world_units.py`
(the unit base + per-app instances), `test_pycvc_gl_world.py` (the transform
chain, grid/axis/light nodes, `VolRenNode` + raycaster value types, and the
`VolSliceNode` slice renderer) and the `extents_metres` case in
`test_pycvc_model.py`.
