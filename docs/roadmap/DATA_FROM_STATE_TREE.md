# Scene nodes view data on the state tree

Joe, 2026-11-20 (VolumeRover ports P8): "we shouldn't load data from a file in the component,
that's a job for the outer app to load data. ... the main scene might be showing the same volume in
3D, we dont want a copy, and we want both the main scene and the grid ortho windows to update if the
volume data changes, say, from a filter". Then: "this should be true for all components that
operate on or view volume, geometry or other kinds of data stored in memory on the state tree and
rendered by scene graphs".

The rule, for every scene node type and component:

1. It views data the app holds on the state tree: a document's `data:` entry (docs/ARIADNE.md
   §5.17, loaded off the frame thread into `data.<name>`), a key a program or filter writes, or
   another node's published data. It never reads a file itself, and has no Load request or file
   field. A component's Load button calls `(data-load ...)`, and its Save button calls `(data-save ...)`.
2. It keeps no private copy. It renders the `std::shared_ptr<const T>` the key holds. A
   `cvc::volume` handle shares that volume's voxels, and the node publishes the same pointer on its
   own data channel so other nodes can follow it.
3. It follows the data. A new payload at the key, a reload (`rev`), or a link moved to another
   dataset redraws it on the next tick.

P8 part 3 (#1423) did this for `volume_grid` and added the pieces the rest builds on: `data_save`
and `register_data_encoder`, the `point_classes` and `volume_grid_session` decoders, cvcGL's `sdf`
data generator, `shown_bounds` skipping hidden nodes, and `VolumeGridNode::follow` (resolve a path
through links; a change is a new node, a new `lastMod`, or a new payload type or pointee, because
the Windows clock ticks in milliseconds).

## Audit (master 9a4b253a)

| Node type | Code | Reads itself | Today with `source: { data }` |
|---|---|---|---|
| `volren` (built-in) | `scene_realize.cpp` `load_node_volume` | `source: { file }` (`cvc::volume(app, path)`), or `{ sdf: { mesh } }`: `read_geometry` and the SDF, both on the frame thread at realize | Drawn as a generic data node: a `VolumeNode`, not a `VolRenNode`, and no transfer-function binding |
| `volslice` (built-in) | same | same | same |
| `volume` (built-in) | `scene_realize.cpp` | `source: { file }` | Generic data node (`VolumeNode`, follows `rev`). No `state`/`node` source |
| `geometry` (built-in) | `read_scene_node_geometry` | `source: { file }` (`read_geometry`); `material: { texture }` (`read_image`); `model:` (file, or a file in a `data:` package) | Generic data node (mesh, terrain, world). No `state`/`node` source |
| `assets:` | `asset_geometry` | the asset's `source:` like `geometry` | — |
| `volume_slice` | `volume_slice_nodes.cpp` | `source: { file }`: the whole volume, or a `slice_source` read plane by plane above 64 Mi voxels (`region_reads`); `{ sdf }`. `volume_of: <id>` shares another plane's source | Not `reads_data`: drawn as a generic `VolumeNode` |
| `mesh_lab` | `mesh_nodes.cpp` | `<node>.source.file`, seeded from `source: { file }`; the `load`/`reload` requests read it on a worker (primitives and height fields are generated, not read) | — |
| skeleton (P2) | inside `mesh_lab` (`skeleton` request) | nothing: it works on the mesh `mesh_lab` holds | — |
| `fe_lab` | `mesh_nodes.cpp` | `<node>.fe.source.{type,file,dim,...}`: a volume file or a mesh's SDF; `load`/`reload` | — |
| `contour_lab` | `contour_lab_nodes.cpp` | `<node>.source.file` (a Reconstruct series) and the `load` request; it already follows `{ node, key }` and `{ state }` | — |
| segmentation (P5) | core and intrinsics only; no node yet (P9 puts it on `volume_grid`) | — | — |
| `volume_grid` | done in #1423 | — | — |
| vehicle `look:`, boat `model:`, room view | `vehicle_view.cpp`, `boat_node.cpp`, `room_view.cpp` | `{ file }`, or `{ data, file }` (a file inside a `data:` package) | — |
| the rest (`water`, `weather`, `forest_trees`, `generated_world`, `ribbon`, `nav_swarm`, `stream`, ...) | | generate or read state; no file | — |

| Component | File field / Load |
|---|---|
| `mesh_tools.ari` | `mesh_ops.source.file` field, Load (`mesh_ops.request load`) |
| `fe_controls.ari` | `fe.source.type`, `.file`, `.dim`, Load (`fe.request load`) |
| `contour_tiler.ari` | Load (`tiler.request load`) |
| `volume_grid.ari`, `world_source.ari` | already through the app (`data-load`, `data-save`) |
| `volren_controls`, `volslice_controls`, `transfer_function`, the rest | none |

## The steps, a PR each

**D1, volume (`volren`, `volume_slice`).**
- Move the shared pieces out of `VolumeGridNode`:
  - `follow` becomes a followed-key helper next to `cvc::state`: a path resolved through links, and
    a change is a new node, a new `lastMod`, or a new payload type or pointee.
  - `volume_of` becomes a volume-payload helper in `cvc/volume/volume_slice.h`. It takes a
    `shared_ptr<const volume>`, a `shared_ptr<volume>`, a `volume` or a `shared_ptr<slice_source>`.
  - `VolumeGridNode` uses both helpers.
- `volren` and `volume_slice` take `source: { data: <name>, item }`, `{ state: <key> }` or
  `{ node: <id>, key }`.
  - The source path is seeded into `<node>.source.volume` (relative to the document's scope; a
    program can point it somewhere else). The node follows that key every tick.
  - The node renders the payload itself. `VolRenNode` keeps a shallow handle on the same voxels,
    and `volume_slice` cuts through a `slice_source` on that volume.
  - The node publishes the pointer as `<node>.volume`. The transfer function follows it through
    `tf_binding_options::volume_source`.
  - Without `key:`, `{ node }` takes the first of `volume` and `grid.volume` that holds data, the
    way `contour_lab` picks between `grid.contours` and `tiler.contours`.
  - `volume_slice` is registered `reads_data`.
- `source: { file }` and `{ sdf }` work as they do now until the file decision (D6).
- `volume.filter` (decision 2).
- Demo: `examples/ariadne/documents/volume_shared.ari`. One `data:` entry is shown by the main
  scene's `volren` and by a `volume_grid`'s planes (the volume_grid component). A Filter menu
  replaces the volume with a filtered one, and both views redraw.
- Test (cvcGL, two apps): the volren's `volume`, the grid's `grid.volume` and the dataset's
  `volume` are one pointer. After a new volume is written, both nodes hold the new pointer, and the
  other app's nodes are unchanged.

**D2, the built-ins `volume` and `volslice`:** they use the same follower, publish `<node>.volume`, and
take the `state` and `node` sources.

**D3, `geometry`:** `{ state }` and `{ node }` sources follow a geometry payload, and the node
publishes `<node>.geometry`. `data:` already works through the generic path.

**D4, `mesh_lab`:** its mesh comes from `data`, `state` or `node`, and the node publishes the mesh it
holds. The `load`/`reload` requests and `source.file` go. mesh_tools' Load button calls
`(data-load ...)` and points the node at the new dataset. The skeleton already works on that mesh.

**D5, `fe_lab`:** its volume comes from `data`, `state` or `node`, and a mesh's SDF comes from the
`sdf` generator. fe_controls changes the same way as mesh_tools.

**D6, `contour_lab` and the file decision:** `contour_lab` drops `source.file` and `load`; the
series comes through a decoder (a `reconstruct_series` `as:`), and contour_tiler's Load button calls
`(data-load ...)`. Then the decision on `source: { file }` for the built-ins applies across every
type. A `volume_planes` decoder (a `slice_source` read plane by plane) keeps `region_reads` working
for files too large to hold in memory.

Later, to decide: `material: { texture }` images, vehicle and boat model files, and `assets:`. These
are a node's look rather than data it views, and the rule may not need to cover them.

## Decisions (Joe, 2026-11-21)

1. **`source: { file }` (and `{ sdf }`) on the built-ins stays, as sugar for a `data:` entry.** The
   loader turns it into an entry named after the node (`{ sdf }` into a `generate: sdf` entry). All
   loading then goes through the data store, off the frame thread, and two nodes naming one file
   share one load. Every type moves over at once in D6. Until then, D1 to D5 leave `{ file }` as it
   is.
2. **D1's demo runs a real filter.** `volume.filter` follows the §1.6 convention
   (docs/roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md, cvc/core/job_intrinsics.h), in three forms:
   `(volume.filter vol [params])`, `-async`, and `-start ... target`. It runs as a sliced job, so it
   stays off the frame thread and works on single-threaded wasm. It starts with VolumeRover's
   bilateral filter, done a plane at a time from a deep copy. The demo's Filter menu runs
   `volume.filter-start` on the shared volume. A program then stores the result back at the
   dataset's key with `state-data-set`, and the volren and the grid follow it. Contrast
   enhancement, anisotropic diffusion and GDTV follow later as more `filter:` values.
</content>
</invoke>
