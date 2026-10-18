# One world representation

The renderer, the physics layer, the navigator and every loader must see one world, built from the
same objects. Today each keeps its own copy, and the copies have drifted apart.

The direction (Joe, 2026-10-17): build on `cvc::geometry`, extend it as needed, and add a
`cvc::height_field` geometry subtype.

## What exists today

An inventory of master after 3.6.0 found these problems.

- **Seven height-field types, none convertible to another.**
  - `physics::heightfield`;
  - `navdemo::Terrain`;
  - pycvc's `terrain_grid`;
  - `gl::HeightFieldTexture`;
  - `world::heightfield` (analytic) with `world::grid_spec` and `raster_out.height`;
  - Ariadne's `make_heightfield`;
  - the demos' own `terrainH` functions.

  `physics::heightfield` has no producer outside tests.
- **`terrain.json` is parsed four times**, by hand, in `nav_common.cpp`, `nav_finale.cpp`,
  `material_raster.cpp` and pycvc's `scenes.py`. The parsers disagree on row order:
  - pycvc flips the rows.
  - The C++ `Terrain` does not, and mirrors the terrain in y.
- **The city demos misplace buildings.** On the Austin bundle, measured over 5636 buildings, each
  building's base sits on the terrain read north-first: RMS 0.46 m, 97.3% within 1 m. Read
  south-first, as `Terrain::sample` reads it, the RMS is 14.3 m. The demos also add the terrain
  height to buildings whose bases are already at terrain height.
- **Four material id spaces with no mapping between them:**
  - `world::surface_class` ids (the ontology);
  - `nav` material ids 0..12, with their own μ and risk;
  - `physics::material_id`;
  - `model` PBR materials.

  Per-face materials exist only in `physics::triangle_mesh`. `cvc::geometry` has no per-face
  attribute, and `model::merged()` drops the per-mesh material.
- **No frame or units data.**
  - The glTF importer applies no up-axis conversion and never sets `metres_per_source_unit`.
  - `lod::partition_params::up_axis` is the only up-axis field anywhere.
- **The same terrain is triangulated five ways**, with different diagonals and UV rules. Render
  heights and nav heights come from separate samplings.

## The representation

### Frame

World geometry is in SI metres, z up, in double-precision world coordinates. Grid rows run along
+y: row 0 is the minimum y. A loader converts into this frame once, at load. No consumer guesses
an up axis or flips rows. The floating origin belongs to the physics world (`world_config::origin`),
not to geometry.

### `cvc::geometry`, extended

- **A virtual destructor**, so subtypes are safe to hold by base pointer.
- **`face_materials()`**: one `uint16` per face, triangles first, then quads. Each entry is an id
  in the world's surface table (below). Empty means every face is material 0. The array is
  copy-on-write like the others, and kept by `copy`, `merge` (a side without materials pads with 0),
  `tri_surface` (a split quad gives both triangles its material) and the codecs.

### `cvc::height_field : cvc::geometry`

- **The grid is authoritative:**
  - `rows`, `cols`;
  - the origin `(x0, y0)`;
  - spacings `dx`, `dy`, with sample `(r, c)` at `(x0 + c·dx, y0 + r·dy)` (corner samples, row 0
    = min y);
  - `float` heights, with NaN as a hole;
  - one surface id per cell, `(rows−1)·(cols−1)` of them.
- **The base arrays hold the matching triangle mesh**, rebuilt from the grid:
  - two triangles per cell with one fixed diagonal;
  - a cell touching a hole is skipped;
  - each triangle gets its cell's material;
  - normals, plus UVs over the extent.

  So every consumer that takes a `cvc::geometry` (`GeometryNode`, the codecs, LOD, picking) takes a
  height field unchanged.
- **Edits go through the grid** (`set_height`, `set_heights`, `set_cell_material`, then `rebuild`).
  Writing the mesh arrays directly (points, faces, face materials) throws, because the grid would
  silently go stale. Copying a height field into a plain `cvc::geometry` gives an ordinary,
  editable mesh that shares the arrays copy-on-write.
- **Queries:** `height(x, y)` (bilinear; NaN outside the grid or in a hole), `normal(x, y)`,
  `cell_of(x, y)` and `material(x, y)`.

### The surface table

`world::surface_registry` becomes the one material table. Each `surface_class` gains `mu`,
`restitution` and `rolling_resistance` next to what it already has: rho, hard, nav class, albedo
and the RF material.

- Physics materials derive from it.
- So do nav's μ and risk planes. Nav's 13-entry palette and its `material_mu` table become a view
  of the registry.
- Render colour comes from the albedo.

Face materials and cell materials are surface ids.

### `cvc::world::world_geometry`

The world as data: the surface registry, plus items. Each item has:
- a name;
- a `std::shared_ptr<const cvc::geometry>`, a height field or a mesh;
- an optional placement;
- roles: render, collide, navigate.

Loaders produce it. Consumers take the same objects:
- **render:** `gl::add_world_geometry(SceneGraph&, const world_geometry&)` makes one
  `GeometryNode` per item, sharing the arrays;
- **physics:** `physics::add_world_geometry(world&, const world_geometry&)` makes the static
  bodies and the materials;
- **nav:** rasterization makes the occupancy, height and material planes for a `grid_spec`.

### Physics shapes reference canonical geometry

`physics::shape` becomes `variant<sphere, box, convex_hull, mesh, terrain>`:
- `mesh` holds a `shared_ptr<const cvc::geometry>`, with its face materials;
- `terrain` holds a `shared_ptr<const cvc::height_field>`.

`physics::triangle_mesh` and `physics::heightfield` go away. This changes the physics API that
shipped in 3.6.0 days ago and has no consumers outside libcvc yet. The physics headers stay
standard-library only by forward-declaring the two types, and the facade flattens them for the
engine, so no backend sees `cvc::geometry`.

### What G3a found

- **`cvc::world` never links `cvc::nav`** (roadmap §14.4 of the L-system roadmap, checked at
  configure time), so a world bundle stays a file format. The land-cover classifier
  (`segment_scene_material`) lives in nav, so the loader cannot call it. Instead it takes a
  `land_cover` provider that returns per-sample surface ids, which it reduces to cell surfaces
  itself. Without one, every terrain cell is `terrain_surface`. The provider that classifies a
  bundle's masks and imagery comes with G4. Until then the physics adapter leaves
  `nav_material` at -1 rather than add a physics→nav edge before P4's plant seam.
- **`cvc::bounding_box` treats a zero-volume box as empty**, and a flat terrain has zero volume, so
  `world_geometry::bounds()` returns its own min/max pair.

### Frames in the scene graph

The renderer keeps a retained-mode hierarchy. Each `GraphicsNode` caches world = parent world ×
local, cascades it top-down when a transform changes, and hands it to its VTK props. Every node
type (geometry, volumes, LOD) respects its local transform and its ancestors'.

The physics world's frame is a node: the scene root by default, or any frame node, such as a
render-origin offset that keeps city-scale coordinates out of float precision.

- `gl::add_world_geometry(sg, g, prefix, parent)` makes the static world's nodes children of the
  frame node.
- `gl::physics_scene_binding::set_frame(parent)` poses each body's node so that its *world* matrix
  is frame × body pose, at any depth (`GraphicsNode::setWorldPoseMatrix`, composed on the owner
  thread).
- Static and dynamic geometry therefore share one frame.

### Fields (G6): volumes as physics media

`cvc::volume` is already the scene's voxel object (`VolumeNode` draws it): a scalar grid with a
bounding box and trilinear sampling. G6 makes the same volumes physics media
(`inc/cvc/physics/field.h`, `src/cvc/physics/field_step.{h,cpp}`):

- **`physics::field`**: a kind, a `std::shared_ptr<const cvc::volume>` placed by its bounding box
  in world metres, optional velocity (three volumes, one per world axis, each on its own grid),
  a drag coefficient and a damping. The field acts inside its volume's box only.
  `world::add_field` / `set_field` / `remove_field`; `world::field_load_on(body)` reports the last
  step's force, torque and immersed volume.
- **Kinds** (the volume's values mean):
  - **fluid** (density, kg/m³): buoyancy −ρ g V and quadratic drag ½ ρ C_d A |u| u;
  - **medium** (bearing modulus k, N/m³): support k V upward, a Winkler foundation, so a flat
    footprint A sunk d carries k A d (Bekker's n = 1, elastic: no ruts yet);
  - **flow** (density): drag only.

  Every kind also takes a linear damping c V u. For a medium it is the resistance to sinking in;
  for water it stands for wave-making, without which a floating body bobs for a long time (the
  first slab trial still bobbed ±3 cm after 16 s on drag alone).
- **`world_geometry` volumes.** `volume_item {name, field, roles}` beside the surface items, one
  name space for both. `physics::add_world_geometry` adds a field per collide volume;
  `gl::add_world_geometry` adds a `VolumeNode` per render volume, sharing the volume's voxels. What a
  viewer sees is what a body feels.
- **The field step.** Jolt knows contacts, not fields, so the facade runs a field step before each
  engine step:
  - **Sample cells.** When a dynamic body is added, its interior is cut into sample cells, about 8
    along its longest side. Spheres and boxes are cut analytically. A convex hull's cells come from
    the engine (`world_backend::shape_interior`, Jolt's `CollidePoint`), with its volume and
    centre of mass.
  - **Clipping and sampling.** Each step, each awake body's cells are clipped to each field's box,
    and the immersed fraction is sampled at its centre. The clip is exact for an upright box, so a
    surface at the volume's top edge gives Archimedes' force exactly.
  - **One load per body.** The forces sum to one force and one torque per body
    (`add_field_load`, Jolt's `AddForceAndTorque` without waking).
  - **Stable drag.** Drag and damping are scaled by (1 − e⁻ˣ)/x, where x is the body's drag rate
    times dt, so a light body never overshoots the stuff's velocity.
  - **Reproducible.** Double precision, ascending field and body ids: `state_hash` stays a function
    of the inputs.
  - **Sleep.** Sleeping bodies are skipped; `add_field` and `set_field` wake the bodies in the box.
- **Values are read at add/set**, into the field's own sampler. A gust or a rising river is a
  `set_field` with new volumes between steps.
- **Next.** The vehicle (P2) reads the medium under each wheel (sinkage, rolling resistance) and
  the flow around the body, and the energy ledger (P3) books the work each field does. A medium that
  keeps ruts needs mutable medium state, which is later work.

## Phases

| Phase | What | Gate |
|---|---|---|
| **G1** | `cvc::geometry`: virtual destructor and `face_materials`. `cvc::height_field` with the grid, the mesh view, queries and the write guard. | Face materials survive copy, merge, tri_surface and the codecs. A height field's mesh matches its grid (vertex count, holes skipped, per-face materials, bilinear `height` equals the mesh's height at any point). Copying gives an editable plain mesh. |
| **G2** | Physics consumes canonical geometry: the `mesh` and `terrain` shapes, with the Jolt backend building from them | P1's gates unchanged; a box on a `height_field` rests at `height(x, y)` ±1 cm; per-face materials reach contacts |
| **G3a** | `world_geometry` and the surface table; the city bundle loader (one `terrain.json` parser, the measured row order, glTF frame options); `gl::add_world_geometry` and `physics::add_world_geometry` | Building bases on the terrain, RMS ≤ 0.5 m on the Austin bundle (measured 0.465 m over 5636 buildings by `CityBundle.RealBundleBuildingsSitOnTheTerrain`); render, physics and nav heights agree at every grid sample |
| **G3b** | The city demos (`nav_city_swarm`, `nav_city_drive`) draw and collide with `load_city_bundle`'s `world_geometry` | The buildings stand on the ground on screen; no demo flips rows or lifts buildings |
| **G4a** | `nav::rasterize` (occupancy, height, surface, rho, hard and mu planes from `world_geometry`) and `nav::mesh_footprint` (the demos' `occupancy_from_model`, moved into the library unchanged); land cover as surfaces (`nav::land_cover_provider` over `segment_scene_material`); `nav::palette_ids` for physics materials' `nav_material`; `world::read_terrain_extent`, so `material_raster` and `nav_finale` drop their own `terrain.json` parsers; `navdemo::Terrain` becomes a view of the shared `height_field`; `gl::add_world_geometry(parent)` | The footprint is the old occupancy bit for bit (`NavWorldPlanes.FootprintIsTheOldOccupancyBitForBit`); land cover reaches the Austin terrain's cells |
| **G4b** | pycvc: `load_city_bundle`, `height_field` and `world_geometry` (`bindings/pycvc/pycvc_world.i`), so `pymod_gl/scenes.py`'s `terrain_grid` retires. Its sampler becomes `height_field.height` (the old one was bilinear), `Lab.add_height_field` draws the field's own mesh, and `Lab.add_terrain` and `terrain_mesh` split cells south-west to north-east (they split the other way, against physics) | Python reads the same terrain as C++: a bundle's north-first grid comes back row 0 at min y, `height(x, y)` is the triangle under the point, and on a real bundle the Python heights equal `terrain.json`'s flipped grid sample for sample (`test_pycvc_world_geometry`, `test_pycvc_gl_world_geometry`) |
| **G5** | Ariadne `physics:` and `body:` specs; Ariadne height fields are `cvc::height_field` (and `TerrainHeights` reads them instead of guessing a grid from a vertex count) | A scene spec puts a crate on a height field and it lands where the renderer draws the ground |
| **G6** | Fields: `cvc::volume`s as physics media (above); `world_geometry` volumes drawn as `VolumeNode`s and pushed as fields | Slabs of 300, 500 and 800 kg/m³ settle at drafts of 0.3, 0.5 and 0.8 of their height within 2 mm (measured 0.1199 and 0.3202 m against 0.12 and 0.32); a 10 kg box sinks into k = 2·10⁴ N/m³ to m g/(k A) within 1 mm; a ball in a 10 m/s wind without gravity follows v = w − 1/(1/w + k t) within 1 %; a field added over a sleeping body wakes it; the same run twice gives the same hash (`physics_field_test`) |

P6a's pose binding (`pose_tracker`, `physics_scene_binding`) moves dynamic bodies. This plan covers
everything static. A dynamic body's visual and collision geometry can be one `cvc::geometry` too,
from G2 on.
