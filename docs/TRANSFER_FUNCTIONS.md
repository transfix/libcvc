# Transfer functions, isocontours and the `tf_editor` widget

*The renderer-agnostic transfer-function model (`cvc::tf`), the Ariadne `tf_editor` widget that
edits it (the port of VolumeRover2's ColorTable2), the histogram / contour spectrum / contour tree
overlays it draws, and how one model drives volren, volslice, the VTK volume mapper and the FEM
renderer.*

Roadmap: [`CVCGL-UI-DSL-ROADMAP.md` §10](roadmap/CVCGL-UI-DSL-ROADMAP.md#10-transfer-functions).
Code: [`inc/cvc/tf/`](../inc/cvc/tf), [`inc/cvc/volume/contour_tree.h`](../inc/cvc/volume/contour_tree.h),
[`inc/cvc/volume/contour_spectrum.h`](../inc/cvc/volume/contour_spectrum.h),
[`src/cvc/ariadne/tf_editor_emit.cpp`](../src/cvc/ariadne/tf_editor_emit.cpp),
[`src/cvcGL/ariadne/tf_editor_imgui.cpp`](../src/cvcGL/ariadne/tf_editor_imgui.cpp),
[`inc/cvc/gl/ariadne/tf_binding.h`](../inc/cvc/gl/ariadne/tf_binding.h).

## Contents

1. [Quick start](#1-quick-start)
2. [The model](#2-the-model)
3. [State layout](#3-state-layout)
4. [The `tf_editor` widget](#4-the-tf_editor-widget)
5. [Overlays: histogram, contour spectrum, contour tree](#5-overlays-histogram-contour-spectrum-contour-tree)
6. [Scene nodes and renderer bindings](#6-scene-nodes-and-renderer-bindings)
7. [The shipped component](#7-the-shipped-component)
8. [Using it from C++ without the scene realizer](#8-using-it-from-c-without-the-scene-realizer)
9. [ColorTable2 compatibility and what changed](#9-colortable2-compatibility-and-what-changed)

---

## 1. Quick start

Any volume-type scene node (`volren`, `volslice`, `volume`) and `fe_lab` publishes its transfer
function at `<node>.transfer_function`, so a document only has to include the shipped component:

```yaml
meta: { min_libcvc: 3.7.0 }
import: cvc://components/transfer_function.ari
scene:
  nodes:
    - node: skull
      type: volren
      source: { file: skull.rawiv }
      volren:
        transfer_function:
          color: [[0, 0,0,0], [0.5, 1,0,0], [1, 1,1,1]]
          alpha: [[0, 0], [0.3, 0.1], [1, 0.8]]
          isocontours: [{ id: bone, at: 0.62 }]
windows:
  - include: transfer_function
    args: { node: skull }
```

The window shows the colour ramp, the opacity curve, the isocontour bars and (by default) a
histogram of the skull's values; dragging a node re-renders the volume, dragging an isocontour
moves the isosurface. Right-click for the menu (add / edit / delete nodes, presets, layers,
`.vinay` import / export).

Run it from the repository root: `arigl --component-path src/cvc/ariadne <doc>.ari`.

## 2. The model

`cvc::tf::model` ([`inc/cvc/tf/model.h`](../inc/cvc/tf/model.h)) is ColorTable2's model — three
independent node lists over a **normalized** axis t ∈ [0, 1]:

| list | node | meaning |
|---|---|---|
| `color` | `{at, r, g, b}` | piecewise-linear colour ramp; endpoints at 0 and 1 |
| `alpha` | `{at, a}` | piecewise-linear opacity; endpoints at 0 and 1; optionally cubed |
| `isocontours` | `{id, at[, rgb][, opacity]}` | isovalues with stable string ids; not part of the colour table |

plus `domain` (`auto`, or an explicit `[min, max]` in data units), `opacity_cubed` (ColorTable2's
v³ low-opacity emphasis) and `resolution` (the default baked-table size).

The domain maps t onto data values: `value = lo + (hi - lo) * t`, where `[lo, hi]` is the explicit
domain or, for `auto`, the consumer's data range (`cvc::tf::effective_domain`). Keeping the nodes
normalized is what lets the same transfer function — or a `.vinay` preset — apply to any volume.

An isocontour's surface colour is its `rgb` override, else the colour ramp at its position (the
ColorTable2 / VolumeRover2 rule); its opacity is its override, else 1.

### The state string

The model rides `cvc::state` as **one** string, so a widget, a document, a script, a renderer
adapter and a conference peer all read and write the same key, and it persists
(`state::save`), replicates and fires `state-watch` like any other value:

```json
{"color":[[0,0,0,0],[0.5,1,0,0],[0.75,1,1,0],[1,0,1,0]],
 "alpha":[[0,0],[0.25,0.75],[0.75,0.25],[1,1]],
 "isocontours":[{"id":"bone","at":0.6},{"id":"skin","at":0.3,"rgb":[1,0.8,0.7],"opacity":0.5}],
 "domain":"auto","opacity_cubed":false,"resolution":256}
```

`cvc::tf::to_string` writes that form; `from_string` also accepts the map forms
(`{"at":t,"rgb":[r,g,b]}`, `{"at":t,"a":a}`), `"dims":1`, and missing keys, and normalizes.

### Presets

`cvc::tf::model::preset(name)` — `default` (ColorTable2's black→red→yellow→green), `grayscale`,
`rainbow`, `hot`, `cool`, `xray`, `isolines`. The widget's Presets menu lists them.

### Baking and lowering

- `bake(size)` / `bake_rgba8(size)`: a table of `size` RGBA entries, entry i at t = i/(size−1)
  (any size; ColorTable2's `getTable` only worked at 256).
- `color_table(m, lo, hi)` / `opacity_table(m, lo, hi)`: the flat `value,r,g,b…` / `value,a…`
  tables in data units that `cvc::volren`, `cvc::volslice` and `cvc::gl::VolumeNode` take. A
  cubed opacity is resampled (a cubed ramp is not piecewise-linear in the node positions). A
  fully transparent model lowers to **empty** tables, so a renderer keeps its "no transfer
  function" path.
- `isovalues(m, lo, hi)`: the data value of each isocontour.

## 3. State layout

Relative to the bound model key `T`:

| key | written by | value |
|---|---|---|
| `T` | widget, document, scripts | the model string |
| `T.range` | the producer (a scene binding or the app) | `"min,max"`, the data range (`auto` domain) |
| `T.histogram` | producer | JSON array of bin counts, uniform over `T.range` |
| `T.contour_spectrum` | producer | `{"area":[…],"min_volume":[…],"max_volume":[…],"gradient":[…]}` |
| `T.contour_tree` | producer | `{"vertices":[[x,value],…],"edges":[[a,b],…]}` |
| `T.overlay_request` | widget | CSV of costly layers a widget shows but nobody computed yet |
| `T.overlay_status` | producer | free text while computing / when unavailable |
| `T.interval` | a mesher (`fe_lab` in `mesh.region: interval`) | `"<id>,<id>"`: the two isocontours bounding the interval it meshes (the first drives `fe.mesh.isovalue`, the second `fe.mesh.isovalue2`), `""` otherwise; editors shade the band between their bars |

Each widget instance also persists its own view under the document scope, like a window's
geometry: `tree.<id>.view.{color,opacity,isocontours,histogram,contour_spectrum,contour_tree,
log_histogram,info}` (0/1) and `tree.<id>.view.zoom` (`"lo,hi"`). A checkbox bound to
`tree.<id>.view.histogram` toggles the layer from outside the editor.

## 4. The `tf_editor` widget

A built-in widget kind (`Kind::TfEditor`), so it works in every Ariadne host without
registration — `arigl`, `ariadne_hello`, pycvc's `AriRuntime`, the conference hosts, the
terminal `ari` — and the short form is valid:

```yaml
- tf_editor: Transfer function
  id: skull_tf                    # names the instance (view state at tree.skull_tf.view.*)
  bind: graphics.root.children.skull.transfer_function
  interactive_updates: true       # commit on every drag frame (default) | false: on release
  height: 160                     # canvas height in px
  layers: [histogram]             # the overlays shown at first (histogram | contour_spectrum | contour_tree)
  preset: rainbow                 # the model an EMPTY key is seeded with (or default: '<model json>')
  on: (print "tf changed")        # an action run after each committed edit
  # optional overlay sources when they live elsewhere (binds, resolved like `bind`):
  # range: …  histogram: …  contour_spectrum: …  contour_tree: …
```

The seed of an empty key is `default:` (a model string, or the same document written as a YAML
map), else `preset:`, else the default model; the scene vocabulary — `color:`, `alpha:`,
`isocontours:`, `domain:`, `opacity_cubed:`, `resolution:` — may sit on the widget too and
replaces that part of the seed. When the widget is bound through a `load:` link hole, it reads
the overlays (and writes `overlay_request`, through a read-write hole only) beside the hole's
target, so an editor inside a module sees the host's histogram.

The explicit form `type: tf_editor` wins over the short-form key scan, so a widget written that
way may carry a `color:` list without becoming a colour picker.

### The ImGui editor

Top to bottom: a 20 px **zoom strip** (the whole axis, two range bars that set the window the
canvas shows), the **canvas**, and optionally the **contour information** table.

| on the canvas | |
|---|---|
| drag a blue square | move an opacity node (endpoints keep t = 0/1; their opacity still drags) |
| drag a bar's red handle / the bar | move a colour node (between its neighbours; endpoints pinned) |
| drag a green isocontour bar | move an isocontour (anywhere; its id is kept) |
| double-click empty canvas | add an opacity node there |
| Delete / Backspace (hovered) | delete the selected node (not an endpoint) |
| right-click | Add ▸ alpha / colour / isocontour · Edit colour… · Use ramp colour · Delete · Display ▸ layers · Presets ▸ · Opacity cubed · Reset · Import / Export .vinay… |
| hover empty canvas | light the histogram bin under the pointer; once the pointer settles, a readout |

Hovering a node shows its position, its data value and its colour / opacity; hovering an
isocontour shows its isovalue, the spectrum values there and the number of connected components.
Hovering empty canvas lights the histogram bin under the pointer at once and, once the pointer
settles, shows a readout that follows it: the value, the bin's range, count and share of all
values (while the histogram shows), the spectrum's values and the number of components there
(while those layers show), and what a double-click and the right-click menu do. The menu's Add
and Display items explain themselves on hover. Display ▸ Histogram / Contour spectrum / Contour
tree turn the overlays on and off (section 5).

In the browser the demo page keeps the browser's own context menu off the canvas
(`examples/web/templates/demo.html.in`), so a right-click reaches the editor.

### Terminals and other backends

A backend without a native editor (FTXUI / tvision, a test mock) gets one composed from ordinary
primitives by the core: a raster preview of the ramp and layers (half-block / kitty / Sixel), a
domain row, a slider row per node (in data units), colour pickers, layer and opacity-cubed
checkboxes, an overlay-status line, and add / delete / reset buttons. Every edit commits at once;
reset restores the default lists but keeps the model's domain and resolution, as the ImGui
editor's does.

## 5. Overlays: histogram, contour spectrum, contour tree

The widget draws whatever is published beside its model; it never computes them itself (roadmap
§10.3: layers are "supplied from outside"). The histogram is cheap and computed at once; the
contour spectrum and tree are computed only after some widget shows them (it appends the layer to
`T.overlay_request`).

- **Histogram** — bin counts over the data range, drawn as bars under the ramp, following the zoom
  window; linear with ColorTable2's clip of the top bar to 1.5× the second-largest, or log scale.
- **Contour spectrum** ([`contour_spectrum.h`](../inc/cvc/volume/contour_spectrum.h)) — Bajaj,
  Pascucci & Schikore's signature functions of the isovalue: isosurface area (red), the volume
  below (green) and above (blue) it, and the integral of |∇f|² over it (yellow), each normalized
  for drawing. `cvc::compute_contour_spectrum` is a self-contained port of libContour's
  regular-grid integrals that builds everywhere, wasm included; the vendored libContour
  (`CVC_ENABLE_MESHER`) remains as a parity oracle (`compute_contour_spectrum_libcontour`).
  The port integrates the **exact** isosurface area: libContour weights one term of its
  per-tetrahedron area by s(1−s) where the exact integral needs 2s(1−s), so its (and
  VolumeRover2's) area and gradient curves run 75–100 % low depending on where the isovalue falls
  between voxel values. `contour_spectrum_options::libcontour_area = true` reproduces them.
- **Contour tree** ([`contour_tree.h`](../inc/cvc/volume/contour_tree.h)) — how isosurface
  components appear, merge, split and vanish across the value range (Carr, Snoeyink & Axen 2003),
  ported from VolumeRover's `contourtree`. The number of arcs crossing an isovalue is the number
  of connected components there; the info table and the isocontour tooltip show it. The port fixes
  the original's merge step (it built wrong trees on 2D and thin data), uses a fraction of the
  memory, and is simplified by persistence before drawing so noisy volumes stay legible.

`cvc::tf::compute_overlays(vol, bits, options)` computes all three (the spectrum and tree on a
thumbnail, `options.max_dim`, default 128 per axis), and `publish_overlays(state, ov)` writes them.

**On a mesh.** The same layers exist for a scalar field on a simplicial mesh
([`contour_mesh.h`](../inc/cvc/volume/contour_mesh.h): one value per point, linear over each
tetrahedron, or each triangle of a surface), through `compute_overlays(mesh, bits, options)`, an
`overlay_job` on a `std::shared_ptr<const contour_mesh>` and the matching `schedule_overlays`:

- the spectrum (`mesh_contour_spectrum_builder`) integrates each tetrahedron with the grid's
  per-tetrahedron integrals and that tetrahedron's exact gradient. On a surface each triangle's
  integrals are exact, and the functions become the contours' length (drawn as "area") and the
  area below and above them (as the two "volumes");
- the tree (`mesh_contour_tree_builder`) runs the same join / split / merge over the graph of the
  elements' edges. On a grid's own tetrahedra it builds exactly the grid builder's tree (the tests
  check this on random data). A mesh in several pieces gets a tree per piece. On a domain that is
  not simply connected (a surface with holes, a torus) the level sets can form loops, which no tree
  holds, so the result is still a tree but its component counts are only approximate there.

`overlay_options::has_range` / `range_lo` / `range_hi` compute the layers over a given range
instead of the data's own, for a producer that publishes its own range and histogram (the field
colours below).

### Where the work runs: the state_exec scheduler, not a thread

A contour tree or spectrum of a 128³ thumbnail takes seconds, and single-threaded wasm cannot start
a thread at all. So nothing here owns a thread: every computation is **resumable** —
`cvc::contour_tree_builder`, `cvc::contour_tree_simplifier`, `cvc::contour_spectrum_builder` and,
over all the layers, `cvc::tf::overlay_job` take `step(budget)` calls that each do about `budget`
voxels' worth of work (every phase is sliced, the vertex sort and the resample included). The
one-shot functions run the same builders to completion, so both forms give identical results.

`cvc::tf::schedule_overlays(app, T, vol, bits, options)`
([`overlay_tasks.h`](../inc/cvc/tf/overlay_tasks.h)) drives a job from an ordinary state_exec
process on the app's scheduler (`app.exec_scheduler()`):

```lisp
(let ((r (tf-overlay-step)))
  (while (not (= r #t))
    (await r)                 ; #f = yield one frame; a future = sleep until it's posted
    (set r (tf-overlay-step))))
(tf-overlay-publish)
```

| mode | where | each `(tf-overlay-step)` |
|---|---|---|
| cooperative | single-threaded wasm (always), or `threading: cooperative` | steps the job for ~`slice_ms` (4 ms) on the scheduler thread and returns `#f`: one frame's yield |
| threaded | native, and wasm-mt served cross-origin isolated (`automatic` picks it wherever threads exist) | runs ~`chunk_ms` (250 ms) of the job on the app's compute pool and returns a future the process parks on |

Either way the results reach state only through `(tf-overlay-publish)`, on the scheduler thread;
between steps the task writes its progress to `T.overlay_status` ("contour tree: sorting 42%").
`cancel_overlays(app, pid)` stops a task (a running pool chunk stops at its next check).

A process on the scheduler advances only while something pumps it. Every Ariadne `Runtime` does,
in its `drain()`, whenever a process is live there, whoever submitted it, so overlay tasks arrive in
any Ariadne host: arigl, `ari`, or one that builds a `Runtime` itself (ariadne_hello, nav_convoy).
Each pump drains the cross-thread ingress, re-readies the `(await …)` frame-yielders and runs a
bounded slice. Several Runtimes on one app each pump every frame but share one frame boundary, so an
`(await #f)` (one cooperative slice) still resumes once a frame, not once per Runtime. While no
process is live and no Runtime has run a program, nothing is pumped, and a scheduler nothing ever
used is not built. A host with no Runtime pumps the scheduler itself, once a frame on the thread
that owns it: `cvc::pump_exec_frame(app.exec_scheduler())`
([`async_lane.h`](../inc/cvc/core/async_lane.h)) is one frame. A test or a headless tool can block
on `wait_overlays(app, pid)` instead.

## 6. Scene nodes and renderer bindings

### The scene `transfer_function:` block

`volren:`, `volslice:` and `volume:` take the roadmap §10.1 form:

```yaml
transfer_function:
  domain: auto            # or [min, max] in data units
  opacity_cubed: false
  preset: rainbow         # a starting point the lists below override
  vinay: presets/skull.vinay   # a ColorTable2 file, relative to the document
  color: [[0, 0,0,0], [0.5, 1,0,0], [1, 0,1,0]]   # [at, r, g, b] or {at, rgb}
  alpha: [[0, 0], [0.25, 0.75], [1, 1]]           # [at, a] or {at, a}
  isocontours: [{ id: bone, at: 0.6 }, 0.3]       # a bare number = {at}
```

The legacy form (`points: [{value, color: [r,g,b,a]}]` in data units, `window:`, `auto_domain:`)
still works and renders exactly as before; its points become the seed model, over the domain the
renderer draws them over, so the first unrelated edit changes nothing:

| node | legacy seed domain |
|---|---|
| `volren` | `auto_domain: true`: `auto` (the data range). `auto_domain: false`: the points' extent, explicit (volren bakes over it and holds the TF past it); points all at one value give volren no extent, so it draws them transparent and the seed is transparent. A `window:` plays no part — volren ignores it, and the loader turns it into `auto_domain: false` |
| `volslice` | `window:` given: the window, explicit (the legacy window spans the LUT, so the TF is held past it). Else `auto` (the data range) |
| `volume` | the node's data range, explicit (the points are absolute values). No transfer function: VolumeNode's default ramp — black → white, opacity 0 → 1, `auto` — even on an empty or constant volume, whose ramp VolumeNode makes invisible: the seed then still re-fits to a real volume a host loads later instead of blanking it |

A volren node's `isosurfaces:` become isocontours (their colours and opacities as overrides, their
shininess kept).

### What a binding does

Every volume-type node and every `fe_lab` gets a `tf_binding`
([`tf_binding.h`](../inc/cvc/gl/ariadne/tf_binding.h)). Once per frame, if the model string
changed, it lowers it into the renderer's own keys — one apply however many writes landed:

| node | model → | isocontours → |
|---|---|---|
| `volren` | `volren.volumes.0.transfer_function.{color,opacity}`, `tf_auto_domain` | `volren.volumes.0.isosurfaces` (ray-traced surfaces) |
| `volslice` | `volslice.transfer_function.{color,opacity}` (+ `window` for an explicit domain, while the window is the binding's) | — |
| `volume` (VTK) | the node's `transfer_function.{color,opacity}`, which it now re-reads live | — |
| `fe_lab` | — | `fe.mesh.isovalue` (one isocontour, followed by id) — and, with `fe.mesh.region: interval`, `fe.mesh.isovalue2` (a second one) — then a re-mesh once the drag settles; `T.interval` names the pair |

It publishes `T.range` (volren / volslice / fe_lab: the volume's range; volume: its data range)
and the overlays, computed by a `schedule_overlays` task from a copy-on-write copy of the volume
(the histogram at once, the spectrum and tree on first request) and re-binned onto `T.range` when
that differs from the range they were computed over.

**Field colours: a colour ramp for a field, not a volume.** `mesh_lab` and `fe_lab` also colour
the values they show (a curvature, a geodesic distance, a FEM solution, element sizes) through a
model at `<node>.field_colors`, which the nodes follow themselves (no `tf_binding`): its colour
ramp is baked into the table every GeometryNode showing the values maps through
(`GeometryNode::setColorTable`), over the model's explicit domain or else the values' own range.
The opacity curve and isocontours are not used, so the components' editors hide them
(`show_opacity: false`, `show_isocontours: false`). The node publishes `field_colors.range` and a
128-bin `field_colors.histogram` of the values beside it. The colormap key (`color.colormap`,
`fe.view.colormap`) seeds the model and resets it to that colormap's ramp when it changes: the
colormap sampled finely and thinned by `cvc::tf::simplify_ramp` to the few colour nodes that
follow it within 0.02 per channel, at most 12 (viridis takes 9, jet 6; turbo, the busiest, keeps 12
and stays within 0.05), so the editor shows a handful of bars to grab and move rather than 33.
An editor that shows the contour spectrum or contour tree (Display ▸ in its menu) gets them for
the shown values on their mesh (fe_lab's tetrahedral mesh for the solution, mesh_lab's surface for
a curvature, a geodesic distance or a vibration mode). An overlay task on the app's state_exec
scheduler computes them over `field_colors.range`, so they line up with the histogram, and
publishes them beside the key. The tree is built on the values clamped to that range and simplified
to 200 nodes. Its components inside the range are the field's own, since a piecewise-linear field's
level-set components depend only on which vertices lie above, and the extrema beyond the range (a
curvature's spikes past its 2-98 % range) no longer crowd it. The spectrum takes the values as they
are. New values (a new solve, another colour mode, a new mesh) stop the
running tasks and clear the two layers, and the editors' standing request starts them again. fe_lab's
elements view colours each element with one value, which has neither, so there
`field_colors.overlay_status` says so. See [ARIADNE.md](ARIADNE.md), "Field colours".

**volslice: the TF is fixed in data units, the window is the LUT's span.** `VolSliceNode` samples
the raw-unit tables over its window (else the data range), so the model's `auto` domain is the
volume's range, not the window: moving the window re-lowers nothing, and the slices show the
same transfer function through a different span, exactly as the legacy renderer does. An
explicit domain is also written as `volslice.window` (so the LUT's 256 entries cover it), but
only while the window is the binding's to write — unset, or still what the binding last wrote (or
the document's `window:` it was seeded with). Once anything else changes the window (a script, a
control, a peer), the binding yields it and never writes it again; a later domain edit then moves
the TF in data units and leaves their window alone. Going back to `auto` clears the window only
while it is still the binding's.

**fe_lab: one isocontour drives the mesh.** The binding follows an isocontour by id — the lowest
when it attaches, and the lowest again if that one is deleted — in both directions: moving it in
an editor writes `fe.mesh.isovalue`, and fe_controls' slider moves it. Another isocontour dragged
past it, or added below it, never takes over. An isovalue within 1e-9 of the domain span of the
mesh's is the same isovalue — the slider's value becomes a normalized position and lowers back
a few ulps off (0.37 over [-1e5, 1e5] returns 4.7e-12 away) — so an unrelated edit after a slider
move keeps the slider's exact value and runs no remesh.

**fe_lab: the bars follow each load.** `fe_lab` loads its source on its lane — an `sdf` source's
field after the scene is realized, another dataset on a `load` — so the binding follows the
volume each load brings (`tf_binding_options::volume_source`). Until the first arrives there is
no `T.range` and no overlay; each new volume is taken as a VolumeNode's `setVolume` is (its
overlays recomputed, `T.range` re-published), and on its range the followed bars move to where
the bounds' values sit. The keys keep their values — a range never rewrites them — and nothing
re-meshes.

**A bound past the volume's range shows as its bar pinned at that end.** A bound outside the
data is legal (fe_controls' sliders span −10..10 whatever the data; `interval: [0.5, 2]` on a 0/1
mask means "0.5 and up"), but the model holds positions in [0, 1] only. Such a bound's bar sits at
that end of the axis, and a bar pinned at an end stands for any bound at or past that end: the key
keeps its true value through attach and every later edit (another bar's drag, an opacity node),
and nothing re-meshes for it. Dragging the bar off the end writes the bar's value, as any drag
does. Attaching never re-meshes at all — `auto_mesh` or the Mesh button meshes what the keys hold. The
debounced re-mesh is owed only until some mesh job starts with the region and bounds the keys
hold: the Mesh button pressed during the debounce (or a script's `fe.request = mesh`) does it, and the
binding does not mesh again after it (fe_lab publishes each job's region and bounds at
`fe.last_mesh`).

**Meshing an interval from two isocontours.** With `fe.mesh.region: interval` (props `region:
interval`, or `interval: [a, b]` for the region and both bounds) `fe_lab` meshes the shell
`min(isovalue, isovalue2) ≤ v ≤ max(isovalue, isovalue2)` — whatever `fe.mesh.inside` says — and
the binding follows a SECOND isocontour, by its own id, for `fe.mesh.isovalue2`: drag either bar
and its isovalue follows (both directions, the same tolerance, one debounced re-mesh), and
fe_controls' Isovalue 2 slider moves that bar. The bars may cross; the mesher sorts the bounds.
The two ids stay distinct: a deleted bound re-adopts the lowest isocontour that is not the other
bound's, and in interval mode the model always holds both — switching to interval with fewer than
two isocontours, or deleting one of only two, adds one at that bound's current isovalue (a
switch never re-meshes by itself; the Mesh button does). A bound written in the same tick as the region
(a script, an `on:` action or a peer setting both) keeps its value and moves its bar; only a bound
nobody wrote takes its bar's value. Switching back to `solid` stops following the
second (it stays in the model). The pair in use is published as `T.interval` (`"<isovalue's
id>,<isovalue2's id>"`, `""` in solid mode), and both editors shade the band between the two
bars — translucent magenta over the ramp and histogram, under the curves, through the zoom
window — and mark the bounds in the isocontour tooltip and the information table. The mesh is
`cvc::tetrahedralize2`'s layer, the solid `min(v − lo, hi − v) ≥ 0` (LBIE's TETRA2 mode), so its
boundary is both isosurfaces: an outer and an inner sheet, each wound out of the shell.
The shell is inclusive: a voxel whose value equals a bound is inside it, so `[1, 3]` on a label
volume keeps labels 1 and 3, and a bar at either end of the axis — exactly the data's minimum or
maximum — keeps the voxels there. LBIE counts a voxel exactly on its isovalue as outside, so a
bound voxels sit on is meshed half way to the nearest value beyond it: on integer data the
half-integer, as a label mesher draws it (`[1, 3]` on labels is the same mesh as `[0.5, 3.5]`). A
bound no voxel lies beyond (on or past an end of the range) bounds nothing and adds no sheet:
`[0.5, 1]` and `[0.5, 2]` on a 0/1 mask both mesh `{v ≥ 0.5}`, crossing at 0.5 like a solid
mesh, a CT's `[200, 255]` keeps its saturated 255s, and with neither bound passed the whole box
is meshed.
`examples/ariadne/documents/mesh_lab.ari` starts its FE bunny as such a skin.

## 7. The shipped component

[`src/cvc/ariadne/components/transfer_function.ari`](../src/cvc/ariadne/components/transfer_function.ari):

| unit | what | args (defaults) |
|---|---|---|
| `transfer_function` | a window with the editor | `node` (`volume`), `title`, `id` |
| `transfer_function_editor` | just the editor, to embed in your own window | `node` (`volume`), `label`, `id` |

```yaml
import: cvc://components/transfer_function.ari
windows:
  - include: transfer_function
    args: { node: skull }
  - window: Slices
    children:
      - include: transfer_function_editor
        args: { node: slices, id: slices_tf }
```

Units declare their parameters with defaults (`args:` in the unit), so an include without `args:`
still binds a sensible node, and two instances given different `id`s keep separate views.

The demos in [`examples/ariadne/documents/`](../examples/ariadne/documents) use it:
`volren_bunny.ari` (the raycast bunny's isosurface as an isocontour bar), `volslice_bunny.ari`
(the legacy windowed transfer function as editable nodes), `transfer_function.ari` (both renderers,
one editor each, model-form `transfer_function:` blocks) and `mesh_lab.ari` (two bars driving
fe_lab's interval mesh).

## 8. Using it from C++ without the scene realizer

Any state path works as a model key; an application that owns its volume publishes the range and
overlays itself and reads the model back:

```cpp
cvc::state &tf = cvc::state::instance(app)("myapp.tf");
// Sliced on app.exec_scheduler() (or the compute pool where threads exist); publishes when done.
cvc::tf::schedule_overlays(app, "myapp.tf", vol, cvc::tf::overlay_all);
// ...or, blocking, where a stall is fine (a batch tool):
// cvc::tf::publish_overlays(tf, cvc::tf::compute_overlays(vol));

cvc::tf::model m;
if (cvc::tf::from_string(tf.value(), m)) {
  double lo, hi;
  cvc::tf::effective_domain(m, true, vol.min(), vol.max(), lo, hi);
  auto lut = m.bake_rgba8(256);                 // your renderer's table
  auto isovalues = cvc::tf::isovalues(m, lo, hi);
}
```

and the document binds `- tf_editor: TF` with `bind: /myapp.tf`. `publish_overlays` writes only
the layers it is given (a later spectrum-only publish keeps an earlier histogram), so when the
volume behind the key is replaced, call `cvc::tf::clear_overlays(tf)` first: the old layers go
blank instead of being drawn over the new range until the new ones arrive. `.vinay` files go
through `cvc::tf::read_vinay_file` / `write_vinay_file` ([`vinay.h`](../inc/cvc/tf/vinay.h)).

## 9. ColorTable2 compatibility and what changed

Kept: the node model and its normalization (including the endpoint rule: a missing 0 endpoint
copies the first node, a missing 1 endpoint is white / opacity 1), the default transfer function,
the canvas layout and layer order, the colours, the context menu, interactive vs delayed updates,
the `.vinay` format (read and write) and `.ctbl` export, the isosurface-colour rule, and the
contour spectrum's functions and the contour tree's connectivity (so values match VolumeRover).

Fixed:
- the baked table works at any size (`getTable` hard-coded 255 and overflowed below 256);
- dragging moves the node you grabbed (ColorTable2 handed the drag to a neighbour it crossed);
- cancelling the colour dialog keeps the colour (it turned the node black);
- isocontours have stable ids, survive `.vinay` round trips by position, and their isovalues are
  shown in data units (ColorTable2 showed 0..255);
- the contour tree is correct on 2D and thin data, its edges are drawn as pairs (every other
  segment ColorTable2 drew was spurious) and the component count is right;
- the histogram follows the zoom window and has a log option;
- the spectrum's surface area (and so its gradient curve) is exact (see §5);
- the spectrum, tree and histogram are computed in slices on the state_exec scheduler (or the
  compute pool), never in a paint call, and never on a thread of their own;
- no inherited viewer shortcuts (Escape closed VolumeRover2).
