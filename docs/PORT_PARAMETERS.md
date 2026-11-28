# Port parameter tables

The VolumeRover ports declare each parameter struct once, as a `cvc::param_table`
([`param_table.h`](../inc/cvc/core/param_table.h); [PORT_SHARED_PIECES.md](PORT_SHARED_PIECES.md),
[VOLROVER-LIBRARY-PORTS-ROADMAP.md](roadmap/VOLROVER-LIBRARY-PORTS-ROADMAP.md) §1.5). From that one
table come the state keys a node seeds and reads, their `doc`, `unit`, `min`, `max` and `choices`
properties, the CLI flags, the pycvc keywords and the DSL dict keys. The tables below are the same
declarations printed by `cvc::params_markdown<P>()`, the generator that ships with
`cvc::param_table`. They are not written by hand.

**Generated** on 2026-11-27 from transfix/libcvc master at fc507d7e0. Each table is the
generator's output exactly; the headings and the "Read by" lines are not generated.
`port_parameters_doc_test` ([`src/cvc/tests/port_parameters_doc_test.cpp`](../src/cvc/tests/port_parameters_doc_test.cpp))
keeps them current: it checks each `### <struct>` table against `cvc::params_markdown<P>()`, and
when a param_table changes it fails and prints the table to paste over the old one. A table added
here goes into the test's list too, and the test fails until both have it. The numbers are as
`std::to_chars` prints them, so the tables are compared only where the standard library has
floating `to_chars`. The wasm-mt build's libc++ has none, and there `format_double` spells some
ranges longer (`1000000000` for `1e+09`).

pycvc prints the same table for the structs it wraps: `pycvc.skeleton_params.markdown()`.

**Columns.** *Range* is the field's `min .. max` (`1e+300` and `1e+09` are the table's stand-ins for
no upper bound). An enum's choices follow its type. A list's default is written as the CLI and the
state key take it. An empty string default prints as two backticks.

| Struct | Header | Read by |
|---|---|---|
| [`skeleton_params`](#skeleton_params) | [`skeleton_params.h`](../inc/cvc/geometry/skeleton_params.h) | `cvc::skeletonize`, `cvc skeleton`, `skeleton.compute`, `pycvc.skeleton_params`, mesh_lab's `skeleton.*` keys ([ARIADNE.md](ARIADNE.md) §5.4) |
| [`tile_params`](#tile_params) | [`tiler_params.h`](../inc/cvc/geometry/tiler_params.h) | `cvc::tile_contours`, `cvc tile`, `tiler.tile`, `grid-contours.tile`, `pycvc.tile_params`, contour_lab's `tiler.*` keys, volume_grid's `grid.tile.*` keys |
| [`seeded_segmentation_params`](#seeded_segmentation_params) | [`segmentation_params.h`](../inc/cvc/volume/segmentation_params.h) | `cvc::segment_seeded`, `cvc::critical_point_seeds`, `cvc segment`, `segment.seeded`, `segment.critical-points`, `pycvc.seeded_segmentation_params`, volume_grid's `grid.segment.*` keys |
| [`gradient_vector_diffusion_params`](#gradient_vector_diffusion_params) | [`gradient_vector_diffusion_params.h`](../inc/cvc/volume/gradient_vector_diffusion_params.h) | `cvc::gradient_vector_diffusion` and its job (C++ only); `critical_point_seeds` runs it with the segmentation's `cp_iterations`, `cp_k` and `cp_weights` |
| [`capsid_seed_params`](#capsid_seed_params) | [`seed_generators_params.h`](../inc/cvc/volume/seed_generators_params.h) | `cvc::capsid_critical_points`, `cvc capsid-seeds`, `pycvc.capsid_seed_params` |
| [`rotation_seed_params`](#rotation_seed_params) | [`seed_generators_params.h`](../inc/cvc/volume/seed_generators_params.h) | `cvc::rotated_seeds`, `cvc rotate-seeds`, `pycvc.rotation_seed_params` |
| [`volume_slice_params`](#volume_slice_params) | [`volume_slice.h`](../inc/cvc/volume/volume_slice.h) | the `volume_slice` node's `slice.*` keys, `pycvc.volume_slice_params` |
| [`volume_grid_params`](#volume_grid_params) | [`volume_grid.h`](../inc/cvc/volume/volume_grid.h) | volume_grid's `grid.{view,tool,slice.*}` keys, `pycvc.volume_grid_params` |
| [`volume_grid_display_params`](#volume_grid_display_params) | [`volume_grid.h`](../inc/cvc/volume/volume_grid.h) | volume_grid's `grid.display.*` keys, `pycvc.volume_grid_display_params` |
| [`volume_grid_source_params`](#volume_grid_source_params) | [`volume_grid.h`](../inc/cvc/volume/volume_grid.h) | volume_grid's `grid.source.*` keys, `pycvc.volume_grid_source_params` |
| [`volume_grid_segment_view_params`](#volume_grid_segment_view_params) | [`volume_grid_segment.h`](../inc/cvc/volume/volume_grid_segment.h) | volume_grid's `grid.segment.*` keys (how the result draws), `pycvc.volume_grid_segment_view_params` |
| [`class_surface_params`](#class_surface_params) | [`class_surfaces.h`](../inc/cvc/volume/class_surfaces.h) | `cvc::class_surfaces`, `segment.surfaces`, `pycvc.class_surface_params`, volume_grid's `grid.segment.mesh.*` keys |
| [`volume_grid_contour_params`](#volume_grid_contour_params) | [`volume_grid_contours.h`](../inc/cvc/volume/volume_grid_contours.h) | volume_grid's `grid.contour.*` keys, `grid-contours.display`, `pycvc.volume_grid_contour_params` |
| [`grid_contour_object`](#grid_contour_object) | [`volume_grid_contours.h`](../inc/cvc/volume/volume_grid_contours.h) | volume_grid's `grid.objects.<i>.*` rows, `grid-contours.object`, `pycvc.grid_contour_object` |
| [`pocket_tunnel_params`](#pocket_tunnel_params) | [`pocket_tunnel_params.h`](../inc/cvc/geometry/pocket_tunnel_params.h) | `cvc::pocket_tunnel` and its job (C++ only) |
| [`secondary_params`](#secondary_params) | [`secondary_structures_params.h`](../inc/cvc/geometry/secondary_structures_params.h) | `cvc::find_secondary_structures` and its job (C++ only) |

The shared types' tables (`contour_read_params`, `contour_component`, `point_class_read_params`,
`point_class`) are in [PORT_SHARED_TYPES.md](PORT_SHARED_TYPES.md).

## Skeletonization

### skeleton_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `robust` | bool | `0` |  |  | filter a noisy sample with Robust Cocone first |
| `big_ball_ratio` | number | `0.0625` |  | 0 .. 1e+300 | Robust Cocone: a Delaunay ball is big when its squared radius exceeds this times a corner's mean squared nearest-neighbour distance |
| `theta_if` | number | `0.08726646259971647` | rad | 0 .. 1.5707963267948966 | Robust Cocone: the angle below which a big ball meets the infinite one deeply |
| `theta_ff` | number | `0.17453292519943295` | rad | 0 .. 3.141592653589793 | Robust Cocone: the angle below which two big balls meet deeply |
| `cocone_phi` | number | `0.39269908169872414` | rad | 0 .. 1.5707963267948966 | Tight Cocone: half the cocone's opening angle around the surface |
| `sharp_phi` | number | `2.0943951023931953` | rad | 0 .. 3.141592653589793 | Tight Cocone: facets meeting at more than this dihedral angle make a sharp edge (hard-coded in VolumeRover2) |
| `flat_phi` | number | `1.0471975511965976` | rad | 0 .. 1.5707963267948966 | Tight Cocone: the normal deviation allowed between cocone neighbours of a flat point (uninitialised in VolumeRover2's dialog) |
| `flatness_ratio` | number | `1.44` |  | 0 .. 1e+300 | Tight Cocone: the squared thinness factor of a flat point |
| `medial_theta` | number | `0.39269908169872414` | rad | 0 .. 1.5707963267948966 | medial axis: a Delaunay edge within this angle of a tangent plane is not chosen by the angle condition |
| `medial_ratio` | number | `64` |  | 0 .. 1e+300 | medial axis: a Delaunay edge is chosen when its squared length exceeds this times both ends' umbrella diameters |
| `discard_small` | bool | `0` |  |  | sheets: keep every cluster of at least threshold times the largest area, instead of the largest sheet_count |
| `threshold` | number | `0.1` |  | 0 .. 1 | sheets: the area fraction discard_small keeps |
| `sheet_count` | int | `2` |  | 0 .. 1e+09 | sheets: how many of the largest planar clusters are sheets (the others are stars) |
| `helix_window` | bool | `0` |  |  | also output each strand's Voronoi vertices of cell radius in (2, 3), the legacy protein-helix filter |

## Contour tiling

### tile_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `collinear_epsilon` | number | `0` | world^2 | 0 ..  | a contour vertex whose triangle with its neighbours has at most this area is dropped as collinear |
| `curation_delta` | number | `1e-08` | world | 0 ..  | the least distance kept between contours of one slice (0: no curation); less than the slice spacing |
| `remove_intersections` | bool | `0` |  |  | remove the intersections between different components' tiles (needs curation_delta > 0) |
| `correspondence_overlap` | number | `-1` | world^2 |  | crossing contours of consecutive slices correspond only above this overlap area (negative: any) |
| `include` | list of string | `[]` |  |  | components tiled, as regular expressions on the name (none: every one) |
| `exclude` | list of string | `[]` |  |  | components not tiled, as regular expressions |
| `z_first` | int | `-1` |  | -1 ..  | the first slice index tiled (-1: the stack's first) |
| `z_last` | int | `-1` |  | -1 ..  | the last slice index tiled (-1: the stack's last) |
| `cap_ends` | bool | `1` |  |  | close every component beyond each end of the slice range |
| `color` | list of number | `[]` |  | 0 .. 1 | one colour (r, g, b in [0, 1]) for every surface (empty: each component's) |

## Seeded segmentation (GenSeg)

### seeded_segmentation_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `t_low` | number | `0` | data |  | voxels below this stop every front (data units, or 0..255 with normalized_thresholds) |
| `t_high` | number | `0` | data |  | automatic seeds are the smoothed volume's local maxima above this (data units, or 0..255 with normalized_thresholds) |
| `normalized_thresholds` | bool | `0` |  |  | t_low and t_high are on the legacy 0..255 scale the volume is rescaled to |
| `gamma` | number | `-0.12` |  | -1 .. 0 | front speed exp(gamma * /grad I/), /grad I/ rescaled to 0..255: fronts slow at edges |
| `automatic` | enum: `none`, `local_maxima`, `critical_points` | `local_maxima` |  |  | automatic seeds added to the point classes' (none, local_maxima, or critical_points: the paper's feature and background classes) |
| `variable` | int | `0` |  | 0 ..  | the point classes of this variable seed the run |
| `timestep` | int | `0` |  | 0 ..  | the point classes of this timestep seed the run |
| `class_volumes` | bool | `1` |  |  | also output a volume per class: its voxels' values, zero elsewhere |
| `cp_iterations` | int | `10` |  | 0 .. 1e+05 | critical_points: iterations of the gradient vector diffusion |
| `cp_k` | number | `2` |  | 0 .. 1000 | critical_points: Eq. 3's k, a neighbour weighing exp(k (cos a - 1)), a the angle between the two vectors |
| `cp_feature_threshold` | number | `-inf` | data |  | critical_points: a maximum below this joins the background (data units, or 0..255 with normalized_thresholds; -inf: none does) |
| `cp_background_threshold` | number | `inf` | data |  | critical_points: a minimum above this joins the feature (data units, or 0..255 with normalized_thresholds; inf: none does) |
| `cp_field` | enum: `image`, `gradient_magnitude` | `image` |  |  | critical_points: the critical points of the gradient of the volume (image, the paper) or of its gradient magnitude (gradient_magnitude, SegMed: the maxima, on edges, are dropped) |
| `cp_weights` | enum: `current`, `initial` | `current` |  |  | critical_points: the field whose angles weigh the diffusion's neighbours, the one being diffused (current, the paper) or the first (initial, SegMed) |
| `cp_sample` | int | `0` |  | 0 ..  | critical_points: thin each connected run of critical voxels to one seed in every cp_sample + 1 (SegMed: 20; 0, the paper's: keep every voxel) |

### gradient_vector_diffusion_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `iterations` | int | `10` |  | 0 .. 1e+05 | iterations of the diffusion (SegMed: 10) |
| `k` | number | `2` |  | 0 .. 1000 | Eq. 3's k: a neighbour weighs exp(k (cos a - 1)), a the angle between the two vectors |
| `weights` | enum: `current`, `initial` | `current` |  |  | the field whose angles weigh the neighbours: current (the field being diffused, the paper) or initial (the field before the first iteration, SegMed) |
| `normalize` | bool | `1` |  |  | divide the result by its longest vector's length |

### capsid_seed_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `t_low` | number | `0` | data |  | only voxels above this count (0..255 with normalized_thresholds, the default; data units without) |
| `normalized_thresholds` | bool | `1` |  |  | t_low is on the 0..255 scale the volume is rescaled to |
| `h` | int | `2` |  | 0 .. 1000 | the capsid's triangulation number h (T = h^2 + hk + k^2) |
| `k` | int | `1` |  | 0 .. 1000 | the capsid's triangulation number k |
| `variable` | int | `0` |  | 0 ..  | the variable the classes mark |
| `timestep` | int | `0` |  | 0 ..  | the timestep the classes mark |

### rotation_seed_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `folds` | int | `2` |  | 1 .. 10000 | copies, at the angles 2 pi n / folds about the axis |
| `axis` | list of number | `[0, 0, 1]` |  |  | the axis's direction (x, y, z), world units |
| `center` | list of number | `[]` |  |  | a point on the axis (x, y, z), world units; [] is the volume's centre |
| `variable` | int | `0` |  | 0 ..  | the classes of this variable rotate |
| `timestep` | int | `0` |  | 0 ..  | the classes of this timestep rotate |

## volume_slice and volume_grid

### volume_slice_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `axis` | enum: `x`, `y`, `z` | `z` |  |  | the axis the slice looks down: x, y or z |
| `index` | int | `-1` | voxel | -1 ..  | which plane along the axis, from 0; -1 is the middle one |
| `filter` | enum: `nearest`, `linear` | `nearest` |  |  | nearest draws each voxel as a square; linear blends between voxel centres |
| `opaque` | bool | `1` |  |  | every voxel in its transfer-function colour at full opacity; off takes the opacity from the transfer function |
| `overlay_offset` | number | `2` |  | 0 ..  | how far the slice is pushed back in depth so overlays drawn in its plane stay on top (polygon-offset units) |

### volume_grid_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `view` | enum: `xy`, `xz`, `zy` | `xy` |  |  | the slice the view switch shows: xy (down z), xz (down y) or zy (down x) |
| `tool` | enum: `navigate`, `points`, `probe`, `contours` | `points` |  |  | what a click in a slice view does: navigate moves the other planes there, points marks and selects point classes, probe only reads the voxel, contours draws the current object's curves |
| `slice.x` | int | `-1` | voxel | -1 ..  | the plane looking down x (the ZY view), from 0; -1 is the middle one |
| `slice.y` | int | `-1` | voxel | -1 ..  | the plane looking down y (the XZ view), from 0; -1 is the middle one |
| `slice.z` | int | `-1` | voxel | -1 ..  | the plane looking down z (the XY view), from 0; -1 is the middle one |

### volume_grid_display_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `point_scale` | number | `1` |  | 0.1 .. 10 | every class's radius times this, as its points draw and as a click finds them |
| `show_points` | bool | `1` |  |  | draw the point classes on the slices |
| `crosshairs` | bool | `1` |  |  | draw where the other two planes cross each slice |
| `grey` | bool | `0` |  |  | draw the slices in grey, not through the transfer function |
| `render` | enum: `none`, `vtk`, `volren`, `volslice` | `none` |  |  | what draws the volume in the 3D view, through the same transfer function: none (the planes alone), vtk, volren or volslice |

### volume_grid_source_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `volume` | string | `` |  |  | the state key whose data channel holds the volume the planes cut (relative to the document; a leading / is the root's), followed as it changes |
| `points` | string | `` |  |  | the state key whose data channel holds point classes: they replace the grid's each time they change |
| `session` | string | `` |  |  | the state key whose data channel holds a session: applied each time it changes |
| `var` | int | `0` |  | 0 ..  | the variable the volume holds (the classes of others are kept, not shown) |
| `time` | int | `0` |  | 0 ..  | the timestep the volume holds (the classes of others are kept, not shown) |

### volume_grid_segment_view_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `show` | bool | `1` |  |  | draw the labels on the slices, in the classes' colours |
| `opacity` | number | `0.5` |  | 0 .. 1 | how much of the slice a label hides (0: none, 1: all) |
| `show_surfaces` | bool | `1` |  |  | draw each class's surface in the 3D view |
| `surface_opacity` | number | `1` |  | 0 .. 1 | the class surfaces' opacity in the 3D view |
| `make_surfaces` | bool | `1` |  |  | a run makes each class's surface after its labels (class_surfaces) |
| `live` | bool | `1` |  |  | draw the labels while the fronts advance (a copy of the labels a few times a second) |

### class_surface_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `smoothing` | int | `10` |  | 0 .. 1000 | passes of Taubin's smoothing (a lambda and a mu step each): the voxel steps go, the classes keep their size; 0 keeps the steps |
| `lambda` | number | `0.5` |  | 0 .. 1 | Taubin's lambda: each vertex moves this far towards its neighbours' mean |
| `mu` | number | `-0.53` |  | -1 .. 0 | Taubin's mu (below -lambda): the step back out that keeps the volume |
| `close` | bool | `1` |  |  | cap each class where it meets the volume's faces, so every surface is closed |
| `background` | int | `255` |  |  | the label of no class, which gets no surface (a segmentation's 255) |
| `labels` | list of int | `[]` |  |  | only these labels get a surface (none: every label but the background) |

### volume_grid_contour_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `show` | bool | `1` |  |  | draw the curves on the slices |
| `control_points` | bool | `1` |  |  | draw the curves' control points |
| `point_size` | number | `0.6` | voxel | 0.1 .. 10 | a control point's radius; a click takes a point within 1.5 times it |
| `line_width` | number | `2` | pixel | 0.5 .. 20 | the curves' width |
| `snap` | bool | `0` |  |  | put new and moved points on voxel centres |
| `show_3d` | bool | `1` |  |  | draw every slice's curves in the 3D view |
| `show_surfaces` | bool | `1` |  |  | draw the tiled surfaces in the 3D view |
| `surface_opacity` | number | `1` |  | 0 .. 1 | the tiled surfaces' opacity |
| `auto_tile` | bool | `0` |  |  | tile each time the contours are replaced (a source, a session, a data write) or traced |
| `source` | string | `` |  |  | the state key whose data channel holds contours (a contour_stack or the grid's curves): they replace the grid's each time they change |

### grid_contour_object

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `name` | string | `` |  |  | the object's name |
| `color` | list of number | `[1, 1, 1]` |  | 0 .. 1 | its colour: r, g, b in [0, 1] |
| `interpolation` | enum: `linear`, `spline` | `linear` |  |  | how its curves draw between their control points: linear or spline |
| `samples` | int | `5` |  | 1 .. 64 | points per span of a spline |
| `visible` | bool | `1` |  |  | whether its curves draw |

## PocketTunnel

### pocket_tunnel_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `pockets` | int | `3` |  | 0 .. 1e+09 | how many of the largest pockets (one mouth) the output surface shows |
| `tunnels` | int | `3` |  | 0 .. 1e+09 | how many of the largest tunnels (two or more mouths) and voids (none) the output surface shows |
| `cocone_phi` | number | `0.39269908169872414` | rad | 0 .. 1.5707963267948966 | Tight Cocone: half the cocone's opening angle around the surface |
| `sharp_phi` | number | `2.0943951023931953` | rad | 0 .. 3.141592653589793 | Tight Cocone: facets meeting at more than this dihedral angle make a sharp edge |
| `flat_phi` | number | `1.0471975511965976` | rad | 0 .. 1.5707963267948966 | Tight Cocone: the normal deviation allowed between cocone neighbours of a flat point |
| `flatness_ratio` | number | `1.44` |  | 0 .. 1e+300 | Tight Cocone: the squared thinness factor of a flat point |

## SecondaryStructures

### secondary_params

| Key | Type | Default | Unit | Range | Description |
|---|---|---|---|---|---|
| `cocone_phi` | number | `0.39269908169872414` | rad | 0 .. 1.5707963267948966 | Tight Cocone: half the cocone's opening angle around the surface |
| `sharp_phi` | number | `2.0943951023931953` | rad | 0 .. 3.141592653589793 | Tight Cocone: facets meeting at more than this dihedral angle make a sharp edge |
| `flat_phi` | number | `1.0471975511965976` | rad | 0 .. 1.5707963267948966 | Tight Cocone: the normal deviation allowed between cocone neighbours of a flat point |
| `flatness_ratio` | number | `1.44` |  | 0 .. 1e+300 | Tight Cocone: the squared thinness factor of a flat point |
| `medial_theta` | number | `0.39269908169872414` | rad | 0 .. 1.5707963267948966 | medial axis: a Delaunay edge within this angle of a tangent plane is not chosen by the angle condition |
| `medial_ratio` | number | `64` |  | 0 .. 1e+300 | medial axis: a Delaunay edge is chosen when its squared length exceeds this times both ends' umbrella diameters |
| `helix_min_width` | number | `2` |  | 0 .. 1e+300 | helices: the narrowest U2 edge kept (the radius of the tube around it) |
| `helix_max_width` | number | `3` |  | 0 .. 1e+300 | helices: the widest U2 edge kept |
| `helix_join_distance` | number | `4` |  | 0 .. 1e+300 | helices: candidate points nearer than this are in one cluster |
| `helix_min_length` | number | `3` |  | 0 .. 1e+300 | helices: a fitted axis shorter than this is not a helix |
| `helix_count` | int | `20` |  | 0 .. 1e+09 | helices: how many of the longest axes are the helices |
| `helix_radius` | number | `1.8` |  | 0 .. 1e+300 | helices: the radius of the cylinders drawn |
| `sheet_min_width` | number | `2` |  | 0 .. 1e+300 | sheets: the thinnest polygon drawn (the length of its dual Delaunay edge) |
| `sheet_max_width` | number | `3` |  | 0 .. 1e+300 | sheets: the thickest polygon drawn |
| `sheet_min_area` | number | `10` |  | 0 .. 1e+300 | sheets: a component of polygons smaller than this is never a sheet |
| `sheet_count` | int | `2` |  | 0 .. 1e+09 | sheets: how many of the largest components are the sheets |
