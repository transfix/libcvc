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
> unit math.

## Table of Contents

- [Overview](#overview)
- [The four frames](#the-four-frames)
- [Why SI is the canonical base](#why-si-is-the-canonical-base)
- [API reference](#api-reference)
  - [`config`](#config)
  - [`system` and `dimension`](#system-and-dimension)
  - [World ↔ canonical metres](#world--canonical-metres)
  - [Canonical SI ↔ the display regime](#canonical-si--the-display-regime)
  - [Formatting and coordinates](#formatting-and-coordinates)
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
lengths, masses, velocities, accelerations, forces, energies and angles —
through **one** exact factor table, so a metre is a metre everywhere and a change
of regime re-labels every number at once.

## The four frames

Four spatial frames are easy to conflate and must not be:

| Frame | What it is | Unit |
|-------|-----------|------|
| **local / authoring** | a graphic's coordinates in its own coordinate system | the model's own (mm, ft, arbitrary) |
| **world** | the doubles the scene graph, geometry, volumes and physics share | 1 unit = `metres_per_world_unit` metres |
| **canonical SI** | the single source of truth for stored statistics and physics | metre, kilogram, second, … |
| **display regime** | how a number is shown and labelled | SI or imperial |

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
};
```

`time` is seconds in both systems. `angle` is shown in degrees in both regimes
(the near-universal display convention); the radian is the canonical/compute
unit.

### World ↔ canonical metres

```cpp
double metres_per_world_unit() const;
void   set_metres_per_world_unit(double metres);   // finite > 0, else throws

double world_to_metres(double world_length) const; // world_length * scale
double metres_to_world(double metres) const;        // metres / scale (scale > 0, safe)
```

### Canonical SI ↔ the display regime

```cpp
double to_display(double si_value, dimension d) const;    // SI -> regime base unit
double from_display(double display_value, dimension d) const; // regime base unit -> SI
std::string unit_symbol(dimension d) const;               // "m"/"ft", "kg"/"lb", ...
```

`to_display`/`from_display` use the regime's **base** unit (metre/foot,
kilogram/pound, m/s / mph, …). They do **not** promote by magnitude — use
`format` for that. SI is the identity for its own base units, so
`to_display(x, length)` in the SI regime returns `x`.

### Formatting and coordinates

```cpp
struct measurement { double value; std::string unit; };
measurement format(double si_value, dimension d) const;

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

libcvc has no physics engine wired in today (Jolt lives in a separate cvcpkg
package). `world_units` is what a future Jolt bridge reads:

- Jolt is SI/MKS (metres, kilograms, seconds; force in newtons), so positions,
  velocities and forces cross the engine boundary in canonical SI with **no
  scaling**. Only `metres_per_world_unit` is applied when importing world-space
  geometry into the physics world.
- Report engine outputs to the user with `format(si_value, dimension::force)`
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
  a regime is `"si"` | `"imperial"`; a dimension is `"length"` | `"mass"` |
  `"time"` | `"velocity"` | `"acceleration"` | `"force"` | `"energy"` |
  `"angle"`;
- the scalar constructor is `world_units(metres_per_world_unit, regime="si")`;
- the display/formatting calls take the dimension string and carry a `_d`
  suffix: `to_display_d`, `from_display_d`, `unit_symbol_d`, `format_d`; the
  regime accessors are `regime_name()` / `set_regime_name()`;
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

The Python contract is pinned by `bindings/pycvc/test_pycvc_world_units.py`
(the unit base + per-app instances), `test_pycvc_gl_world.py` (the transform
chain, grid/axis/light nodes, `VolRenNode` + raycaster value types) and the
`extents_metres` case in `test_pycvc_model.py`.
