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

## Phases

| Phase | What | Gate |
|---|---|---|
| **G1** | `cvc::geometry`: virtual destructor and `face_materials`. `cvc::height_field` with the grid, the mesh view, queries and the write guard. | Face materials survive copy, merge, tri_surface and the codecs. A height field's mesh matches its grid (vertex count, holes skipped, per-face materials, bilinear `height` equals the mesh's height at any point). Copying gives an editable plain mesh. |
| **G2** | Physics consumes canonical geometry: the `mesh` and `terrain` shapes, with the Jolt backend building from them | P1's gates unchanged; a box on a `height_field` rests at `height(x, y)` ±1 cm; per-face materials reach contacts |
| **G3a** | `world_geometry` and the surface table; the city bundle loader (one `terrain.json` parser, the measured row order, glTF frame options); `gl::add_world_geometry` and `physics::add_world_geometry` | Building bases on the terrain, RMS ≤ 0.5 m on the Austin bundle (measured 0.465 m over 5636 buildings by `CityBundle.RealBundleBuildingsSitOnTheTerrain`); render, physics and nav heights agree at every grid sample |
| **G3b** | The city demos (`nav_city_swarm`, `nav_city_drive`) draw and collide with `load_city_bundle`'s `world_geometry` | The buildings stand on the ground on screen; no demo flips rows or lifts buildings |
| **G4** | Nav planes from `world_geometry`; land cover as surfaces (a `city_bundle_options::land_cover` provider over `segment_scene_material`, from outside `cvc::world`); physics materials get their `nav_material`; `navdemo::Terrain`, `material_raster`'s parser, pycvc's `terrain_grid` and Ariadne's `TerrainHeights` retire | Nav goldens unchanged where the old path was right; the y-mirror and the double lift fixed |
| **G5** | Ariadne `physics:` and `body:` specs; Ariadne height fields are `cvc::height_field` | A scene spec puts a crate on a height field and it lands where the renderer draws the ground |

P6a's pose binding (`pose_tracker`, `physics_scene_binding`) moves dynamic bodies. This plan covers
everything static. A dynamic body's visual and collision geometry can be one `cvc::geometry` too,
from G2 on.
