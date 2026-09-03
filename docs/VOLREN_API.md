# cvc::volren — Software Raycast Volume Renderer

`cvc::volren` is the modern C++ port of volrover's `volren` library (with the
ray/isosurface-intersection subset of `libiso` absorbed as an internal detail).
It is a headless, CPU, software ray-caster over `cvc::volume` data producing a
`cvc::image` RGBA frame plus a float depth map, with a camera model that is
directly interconvertible with cvcGL's (`CameraController::getPose()` +
vtkCamera view-angle conventions), so a raycast volume can later be dropped
into a cvcGL scene and composited with a depth-aware shader.

Design rules (per the CVC modernization roadmap, Phase 10/11a):

- **No globals, no singletons.** All state lives in value types owned by a
  `raycaster` instance. Read-only tables are `inline constexpr`.
- **All tunables are typed constants** (`cvc::volren::defaults`,
  `cvc::volren::limits`) — no preprocessor defines.
- **cvc objects at the boundary**: `cvc::volume` in, `cvc::image` out,
  `cvc::bounding_box` for extents.
- **App state tree for settings**: `state_settings` two-way-binds the full
  settings surface at `<prefix>.volren.*` following the cvcGL
  `state_object` pattern (constructor-injected `cvc::app&`).
- **RAII + shared ownership**: the raycaster pins each volume's buffer via
  `voxels::active_storage()` for the duration of a render; per-ray scratch
  (the spline-gradient cache) is a stack object, not hidden statics.
- **Modern threading**: tiles are rendered via `cvc::thread_pool`
  (`app.computePool()` by default); rendering is deterministic — output is
  byte-identical regardless of thread count.

## Namespace and layout

| Where | What |
|---|---|
| `inc/cvc/volren/*.h` | public API, namespace `cvc::volren` |
| `inc/cvc/volren/detail/*.h` | kernels (sampling, spline gradient, MC cell intersection, shading), namespace `cvc::volren::detail` |
| `src/cvc/volren/*.cpp` | implementation |
| `src/cvc/tests/volren_*_test.cpp` | gtest suites |

Built unconditionally into the core `cvc` target (like `cvc::nav` /
`cvc::vis`); it has zero external dependencies beyond core libcvc.

## Camera (`inc/cvc/volren/camera.h`)

```cpp
struct camera {
  enum class projection_type { perspective, orthographic };

  std::array<double, 3> eye{0.0, -4.0, 0.0}; // south of focal, looking north
  std::array<double, 3> focal{0.0, 0.0, 0.0}; // look-at point
  std::array<double, 3> up{0.0, 0.0, 1.0};    // cvc scenes are Z-up
  projection_type projection = projection_type::perspective;
  double vfov_degrees = 30.0;      // VERTICAL field of view (vtkCamera::ViewAngle)
  double parallel_scale = 1.0;     // ortho: half-height in world units (vtkCamera::SetParallelScale)
  int width = 512, height = 512;   // output raster in pixels; aspect = width/height

  view_basis basis() const;                 // orthonormal {right, true_up, back}
  ray generate_ray(int px, int py) const;   // through the pixel CENTER
  static camera from_pose(const double eye[3], const double focal[3],
                          const double up[3], double vfov_degrees,
                          int width, int height);
};
```

Conventions (all chosen to match cvcGL/VTK so `CameraController::getPose()` +
`vtkCamera::GetViewAngle()` feed straight in):

- Right-handed basis: `back = normalize(eye - focal)` (points toward the
  viewer, the legacy `vpn`), `right = normalize(up × back)`,
  `true_up = back × right`.
- FOV is **vertical, in degrees**; horizontal extent follows from aspect.
- **Pixel (0,0) is the TOP-LEFT** ray (matches `cvc::image`'s top-left
  origin; use `image::flipped_vertical()` when uploading to GL/VTK).
- Rays go through pixel centers (`px + 0.5`).
- Orthographic rays share direction `-back`, origins on the eye plane.

The legacy `Viewing` model (world-unit window size + `win_sp` corner + fov in
radians + bottom-left row 0 + raster forced to multiples of 32) is gone; any
raster size renders exactly.

Depth helper:

```cpp
// eye-space depth (distance along -back, i.e. vtk camera Z) -> OpenGL window z
double depth_to_window_z(double depth, double near_z, double far_z,
                         camera::projection_type projection);
```

## Transfer function (`inc/cvc/volren/transfer_function.h`)

Piecewise-linear RGBA ramp over the **raw value domain** (replaces the legacy
integer-density `coldentbl` and makes Float/Double volumes first-class):

```cpp
struct transfer_point { double value; float r, g, b, a; };  // colors in [0,1]

class transfer_function {
  // points kept sorted by value; sample() linearly interpolates,
  // clamping outside [front.value, back.value]
  void add(transfer_point p);
  rgba_f sample(double value) const;
  baked_transfer_function bake(double lo, double hi,
                               std::size_t size = defaults::lut_size) const;
};
```

NaN robustness: a NaN sample value routes to LUT entry 0 (transparent under
the usual low-end-transparent ramps) instead of an undefined index; NaN
gradient magnitudes modulate to 0.

`baked_transfer_function` is the flat LUT the marcher indexes (uniform in
`[lo, hi]`, clamped) — the moral equivalent of `coldentbl`, but instance-owned
and value-domain.

Gradient-magnitude opacity modulation (the legacy `gradtbl` 2D-TF precursor):

```cpp
struct gradient_opacity_ramp {
  bool   enabled = false;   // disabled => modulation factor 1.0
  double ramp0 = 0.0, ramp1 = 0.0, ramp2 = 0.0;  // |gradient| breakpoints
  double plateau = defaults::gradient_plateau;    // legacy 0.9
  float factor(double gradient_magnitude) const;  // 0 / linear / plateau / 0
};
```

## Settings (`inc/cvc/volren/settings.h`)

```cpp
struct light {            // directional; multiple lights ACCUMULATE (legacy bug fixed)
  std::array<float, 3> color{1.f, 1.f, 1.f};
  std::array<double, 3> direction{0.0, 0.0, 1.0};  // toward the light
};

struct isosurface {
  double value = 0.0;      // raw value domain
  float  opacity = 1.0f;
  std::array<float, 3> color{1.f, 1.f, 1.f};
  float  shininess = defaults::shininess;   // real exponent (legacy ignored it)
};

struct cut_plane {         // half-space clip: samples with dot(p - point, normal) < 0 are CULLED
                           // (the normal points at the kept half-space, VTK-style)
  std::array<double, 3> point{}, normal{0.0, 0.0, 1.0};
};

struct volume_settings {   // per volume
  bool shaded = true;            // legacy RAY_CASTING bit (TF + Blinn-Phong)
  bool unshaded = false;         // legacy COL_DENSITY bit (TF only)
  mat4 model_transform;          // scene-graph placement -- see "Scene-graph transforms"
  transfer_function tf;          // domain defaults to the volume's [min,max]
  bool tf_auto_domain = true;    // bake over volume min/max; else tf point extents
  gradient_opacity_ramp gradient_ramp;
  double window_min/max;         // density window (samples outside are skipped)
  bool window_enabled = false;
  std::vector<isosurface> isosurfaces;  // implies the legacy ISO_SURFACE bit when non-empty
};

struct render_settings {   // scene level
  std::array<float, 3> background{0.f, 0.f, 0.f};
  std::vector<light> lights;             // no lights + ambient 0 => shaded samples are black (legacy semantics)
  std::vector<cut_plane> cut_planes;     // legacy declared these but never implemented; live here
  shadow_settings shadows;               // volumetric shadows -- OFF by default (see below)
  bool two_sided_lighting = false;       // legacy light_both
  float ambient = 0.0f;                  // legacy zeroed ambient; now a real knob
  int steps = defaults::steps;           // samples along the scene-bbox diagonal
  float opacity_cutoff = defaults::opacity_cutoff;        // early ray termination (0.95)
  float depth_alpha_threshold = defaults::depth_alpha_threshold; // where the depth map latches
  unsigned threads = 0;                  // 0 => pool default; 1 => serial
  int supersample = 1;                   // sub-samples per pixel EDGE: n => n x n rays
};
```

## Raycaster (`inc/cvc/volren/raycaster.h`)

```cpp
struct frame {
  cvc::image color;  // RGBA8, top-left origin; alpha = accumulated opacity (0 on miss)
  cvc::image depth;  // GRAY f32; eye-space depth of the first sample that pushes
                     // accumulated alpha past depth_alpha_threshold, or the first
                     // isosurface hit if that comes first; +inf where nothing hit
};

class raycaster {
public:
  explicit raycaster(cvc::app &ctx);

  std::size_t add_volume(const cvc::volume &vol,
                         volume_settings vs = {});  // shallow copy; buffer pinned during render
  void clear_volumes();
  volume_settings &volume_config(std::size_t index);

  void invalidate_device_volume(std::size_t index);  // CUDA cache; see below
  void invalidate_device_volumes();

  camera &view();
  render_settings &settings();

  void set_thread_pool(cvc::thread_pool *pool);  // borrowed; default: a private pool
                                                 // (cvc::thread_pool allows one in-flight
                                                 // parallel_for, so the shared computePool
                                                 // is unsafe to borrow blindly)

  void set_backend(backend b);   // cpu (default) / cuda / automatic
  backend backend_used() const;  // what the LAST render actually ran on

  frame render();          // interruption-point + threadProgress aware
  cvc::bounding_box scene_bounds() const;        // union of volume boxes ("metavolume")
};
```

Algorithm (faithful to volren where it defines the look, fixed where it was
broken — every deviation listed in "Fidelity" below):

1. One ray per pixel (`supersample²` of them when supersampling — see
   "Anti-aliasing"); slab-method AABB entry/exit against the union of the
   volume bounding boxes. Miss ⇒ background RGB, alpha 0, depth +inf.
2. March with `unit_step = diagonal(scene_bounds) / steps`. Contribution is
   **per cell entered, not per step** (the defining volren sampling model):
   each 8-corner cell contributes at most once per ray, per volume.
3. Per new cell, per volume, in fixed order:
   - **Isosurfaces**: if the corner min/max brackets an isovalue, intersect
     the ray with the marching-cubes triangulation of the cell (the ported
     `iso_intersectW`); shade the hit with the B-spline volume gradient +
     Blinn-Phong.
   - **Unshaded TF**: baked-LUT color/alpha at the trilinear sample.
   - **Shaded TF**: trilinear sample + B-spline gradient; opacity =
     TF alpha × gradient-ramp factor; Blinn-Phong shading.
4. Front-to-back associated-color compositing:
   `ratio = a·(1-A); RGB += ratio·rgb; A += ratio`, early termination at
   `opacity_cutoff`, then `RGB += background·(1-A)`.
5. Tiles of `defaults::tile_size²` pixels are distributed over the thread
   pool; every tile writes a disjoint region, so output is thread-count
   invariant.

**Per-ray volume culling.** Before marching, each ray solves every volume's
`[t_enter, t_exit]` against that volume's own box through the model transform
(the same slab test the isosurface DDA runs) and keeps the ones it can reach;
a step then visits only the volumes whose window contains `t`, and a ray that
reaches none exits immediately with background. This is pure work elimination
— the culling box strictly contains the region where `grid_sampler::cell_index`
can succeed (`minb - span` to `minb + span·(dim-1)`), with one extra voxel of
margin per face so the rejection cannot disagree with the marched sample by
rounding, and the `t` window is padded by one step. The output is
**byte-identical** to the uncalled march, on both backends.

The gradient is the legacy quadratic-B-spline (de Boor) gradient over the
4×4×4 neighborhood, with the per-ray cache carried in a stack
`detail::spline_gradient_cache` (the state that used to force `vrCopyEnv`).

## Anti-aliasing (`render_settings::supersample`)

`supersample = n` casts an **n × n regular grid** of rays per pixel at
sub-pixel offsets `((i+0.5)/n, (j+0.5)/n)` and box-filters them into the one
output pixel. The cost is **exactly n² rays** — 4× at n=2, 16× at n=4 — and
`limits::max_supersample` caps n at 4. `n = 1` is one ray through the pixel
center and is **bit-identical** to the renderer before supersampling existed,
on both backends: `0.5 / 1.0`, `x / 1.0f` and `min(x, +inf)` are all exact, so
the single-sample path evaluates the same expressions it always did.

**Regular grid, not jitter, and not a rotated one.** Determinism is a
documented contract here (byte-identical across runs and thread counts), which
rules out stochastic sampling outright. A *fixed* rotated grid or a Halton
prefix would satisfy determinism and would resolve near-axis-aligned edges into
more levels for the same ray count — but the regular grid buys something those
do not: an n-supersampled `W × W` render places its rays on exactly the pixel
centers of an `n·W × n·W` single-sampled one,

```
(px + (i+0.5)/n) / W  ==  (n·px + i + 0.5) / (n·W)
```

so the sub-pixel placement is checkable against a render the renderer can
already produce, rather than against a table of magic offsets
(`SupersampleGridLandsOnAFinerRastersPixelCenters` asserts exactly this — the
color to 1 LSB, since the finer raster quantizes each sub-sample to u8 before
the test can average them, and the **depth exactly**).

**Color resolve: an unweighted mean of the STRAIGHT RGBA.** `frame::color`'s
RGB is the color you display — the volume's associated color already
over-blended with the background — so it is *not* premultiplied by the alpha
stored beside it: a missed ray carries the full background color at alpha 0.
Multiplying it by alpha before averaging (the reflex when a resolve is called
"premultiplied") would erase the background contribution of every partially
covered pixel: half an opaque red surface over a white background would resolve
to half red and half **black**, a dark fringe around every silhouette on any
non-black background. Averaging the resolved straight values is exactly right
rather than merely safe — `rgb + background·(1-a)` is affine in `(rgb, a)`, so
the box filter commutes with it, and resolving each sub-sample then averaging
equals averaging the associated colors and over-blending once.

**Depth resolve: the NEAREST finite sub-sample, never an average.** The depth
map exists so volume pixels depth-test per pixel against opaque scene geometry
(`VolRenNode` converts it to window z). Averaging across a silhouette invents a
surface at a depth *no* sub-sample saw — somewhere between the foreground and
the +inf background — so geometry passing between them would punch through the
edge. The nearest hit is a depth some sub-sample actually measured, and it is
the conservative one: a pixel with any foreground coverage occludes at the
foreground's depth. `min` is also order-independent, so the resolve cannot
become a determinism hazard.

**Supersampling vs. resolution scale — two different knobs.**
`cvc::gl::VolRenNode::setResolutionScale()` raycasts at `viewport × scale` and
rescales it onto the quad; `supersample` raycasts *more rays into the same
pixels*.
Both are priced in rays, and they trade against opposite things:

| | Rays | Output resolution | What it fixes |
|---|---|---|---|
| `resolution_scale = s < 1` | `s²` × viewport | `s` × viewport, upscaled | latency — everything gets blurrier, edges included |
| `resolution_scale = 2` | 4 × viewport | unchanged | edges, by an exact 2×2 box downsample in the texture filter |
| `supersample = n` | `n²` × raster | unchanged | stair-stepped isosurface silhouettes, at full sharpness |

Dropping the scale hides aliasing by blurring the whole image; supersampling
removes it while the interior stays as sharp as the raster allows. They
compose: `scale 0.5, supersample 2` costs the same rays as `scale 1.0,
supersample 1` and spends them on smoother edges over a softer image instead of
a sharper, jaggier one. The `volren_bunny` demo has a control for each, one
above the other, to make that comparison directly.

**The scale's other half.** `VolRenNode::MaxResolutionScale` is **2.0**, so the
raster can also be *larger* than the viewport, and there the scale is an
anti-aliasing knob too: the quad's texture filter is bilinear, a 2× raster puts
exactly four texels under each screen pixel, and a screen pixel's center falls
on the midpoint between texel centers in both axes — weights 0.5/0.5, i.e. an
exact 2×2 box average of all four rays. 2.0 is the cap for that same reason and
not an arbitrary one: at 3× the bilinear tap still reads 4 of the 9 texels under
the pixel, so a third of the rays paid for would be discarded. Past 2×,
`supersample` is the knob that averages every ray it casts.

That only holds because the composite filters **premultiplied** color (see the
node section below). With straight alpha it degenerates into a dark ring around
every silhouette — brighter edges are *not* what a 2× raster would have bought.

Measured on this box (GTX 1650, 640×360 rays, 64³ SDF bunny, `steps = 384`,
min of 11 runs):

| supersample | rays/px | CPU, 1 volume | CUDA, 1 volume | CPU, 9 volumes | CUDA, 9 volumes |
|---|---|---|---|---|---|
| 1 | 1 | 31.1 ms | 11.2 ms | 41.5 ms | 33.0 ms |
| 2 | 4 | 120.1 ms | 40.7 ms | 159.0 ms | 126.2 ms |
| 3 | 9 | 300.3 ms | 90.0 ms | 336.4 ms | 287.9 ms |
| 4 | 16 | 484.0 ms | 160.8 ms | 566.8 ms | 507.6 ms |

i.e. slightly *sub*-quadratic (15.6× and 14.4× at n=4 against a 16× ray count),
because the per-pixel work outside the march — and, on the 9-volume scene, the
per-ray volume cull that rejects most of them — amortizes over the sub-samples.

## Volumetric shadows (`inc/cvc/volren/shadow.h`)

```cpp
struct shadow_settings {
  bool enabled = false;      // OFF by default: every existing scene is byte-identical
  std::vector<int> lights;   // indices into render_settings::lights; EMPTY == all cast
  int   resolution = 512;    // light-view raster EDGE, square, clamped to [64, max_raster_dim]
  float strength = 1.0f;     // 0 = byte-identical no-op, 1 = the light contributes nothing
  float bias_scale = 1.0f;   // constant bias, in latch quanta (see below)
  float slope_scale = 1.0f;  // slope bias, in texel_world * tan(theta)
  float min_occluder_opacity = 0.5f;  // an isosurface casts only at/above this opacity
};

struct shadow_view {          // the light-view frame of one built map; PUBLIC on purpose
  std::array<double,3> eye, right, up, forward;   // forward == -normalize(light.direction)
  double parallel_scale, texel_world;             // half-height == half-width; 2*ps/height
  int width, height, light_index;
  bool project(const std::array<double,3> &p, int &ix, int &iy, double &depth) const;
};
```

`raycaster` exposes the maps the last `render()` built:

```cpp
std::size_t      shadow_map_count() const;
cvc::image       shadow_map_depth(std::size_t i) const;   // GRAY f32, +inf where the light missed
const shadow_view &shadow_map_view(std::size_t i) const;
void             invalidate_shadow_maps();
```

**How it works.** One **orthographic light-view pass per casting light**,
rendered by the *same* marcher on a temporary `raycaster`. Its `frame::depth`
latches at the first isosurface hit or the first sample past
`depth_alpha_threshold` — which is exactly "where the light stops". No new
traversal code exists on either backend: the light pass reuses `ray_generator`,
the slab intersect, the per-ray volume cull, the Amanatides–Woo isosurface DDA
and the tile parallelism.

The comparison needs no projection matrix and no near/far round-trip. For an
orthographic camera every ray shares `direction == forward`, so `z_scale == 1`
and `frame::depth` stores exactly `dot(p - eye, forward)` — the same quantity
`shadow_view::project` computes for an arbitrary world point. Misses are `+inf`,
which fails **open** (lit) for free.

**One map covers self-shadowing and inter-volume shadowing.** The light pass
renders the whole registered volume set with the same model transforms and the
same `scene_bounds`, so "volume A shadows volume B" and "A shadows itself" are
the identical lookup.

**The light pass differs from the main pass in exactly three settings**, each
for a reason: `lights` is cleared (the depth latch is driven by accumulated
*alpha* only, so shading it would change nothing and cost a Blinn–Phong per
contribution), `supersample` is forced to 1 (a depth map has no colour to
filter, and the nearest-of-n resolve would dilate every occluder by half a
pixel of its silhouette), and `shadows` is cleared (no recursion). Everything
else — `steps`, `cut_planes`, `opacity_cutoff`, `depth_alpha_threshold`,
`threads` — is inherited, which is why a cut plane that removes an occluder
removes its shadow too.

### Caching

The maps are **camera-independent**, and they are cached on a fingerprint of
the light pass's own inputs: lights, volumes (buffer identity + content
generation + geometry), model transforms, per-volume settings, cut planes,
`steps`, scene bounds and the shadow settings. Moving the camera does **not**
rebuild. `strength`, `bias_scale` and `slope_scale` are consumed by the main
march, not by the light pass, so they are deliberately absent from the key.

The one hole is an in-place write through the unchecked legacy
`voxels::data_ptr()`: the buffer pointer does not move, so the fingerprint
cannot see it. `invalidate_shadow_maps()` announces it (the same contract as
`invalidate_device_volume()` for the resident device cache).

### Bias

```
bias = bias_scale * latch_quantum + slope_scale * texel_world * tan(theta)
cos(theta) = |dot(N, light_dir)|, floored at 0.1 so tan caps at ~9.95
```

Two independent error sources, one term each.

The **slope** term is the lateral footprint: the map stores the depth along the
texel-centre ray, and a receiver up to half a texel across a surface tilted
`theta` from the light sits `texel_world * tan(theta)` deeper.

The **`latch_quantum`** term is the along-ray quantization of the map, and it is
**not** the march step, which is the obvious-looking wrong answer:

- An **isosurface** latch is an exact ray/MC intersection — not quantized at
  all — so one `unit_step` already covers it. Measured, self-shadowing 64³
  sphere: worst disagreement 0.42 `unit_step` overhead, 0.66 at 30° elevation.
- A **transfer-function** latch fires on the first march *sample* that crossed
  the alpha threshold, and the volren sampling model contributes at most once
  per **cell** — so it walks in cells, and so does the receiver sample along its
  own (different) ray: two independent cells of slack, one per ray. Measured on
  the same sphere with a one-cell-thick shaded-TF shell, at 128 → 1024 steps,
  the worst disagreement is **flat in world units** (0.056 → 0.048) while it
  grows from **4.2 to 28.4 `unit_step`s**. The step is measurably the wrong
  unit.

`render()` therefore sets `latch_quantum = max(unit_step, 2 × the coarsest
world-space cell diagonal among volumes that cast through a transfer function)`,
and leaves a pure-isosurface scene on `unit_step` alone — applying the cell
quantum there would peter-pan every crease shadow on the renderer's flagship
content to pay for a latch mechanism it does not use.

Two more things keep acne away, and both are load-bearing (with all bias zeroed,
**52.8%** of a self-shadowing plate speckles; with either term at 1.0, **0.000%**):

- `dot(N, light_dir) <= 0` under one-sided lighting **skips the lookup
  entirely**. Such a surface already gets neither diffuse nor specular, so
  there is nothing to attenuate — this deletes the whole grazing/back-facing
  acne class at negative cost.
- The `cos` floor of 0.1 rather than an epsilon: a flat-gradient sample
  normalizes to `{0,0,0}` and would divide by zero. Flooring makes it maximally
  biased, i.e. **lit** — and it gets no diffuse and no specular anyway, so the
  value is inert. Failing lit is the right direction: an erroneous bright pixel
  reads as unshadowed, an erroneous dark one reads as acne.

### Deliberate limits

- **Hard shadows only.** The test is binary, scaled by `strength`. No penumbra.
  PCF is ~10 lines in both kernels once the single tap is right; not in this
  pass.
- **One occluder layer per light ray** — the latch is a single scalar, so a
  point behind two thin sheets is exactly as dark as behind one.
- **`volume_settings::unshaded` samples are NOT shadowed.** That mode is defined
  as "TF readout with no lighting model"; there is no light term to attenuate,
  and darkening it would invent a lighting model for the one mode whose
  contract is that it has none. Pinned by
  `UnshadedSamplesAreNotShadowed`.
- **Translucency is ignored on the caster side.** The latch fires on the first
  isosurface hit whatever its opacity, which is what `min_occluder_opacity`
  exists to bound: without it the `volren_bunny --shell` decorative shell
  (opacity 0.16, four world units out) becomes the occluder for the body it
  wraps. Transfer-function volumes still cast through the alpha latch and are
  not filtered.
- **A genuinely thick translucent medium still self-shadows at grazing
  incidence**, and no bias fixes it: the map records the *light* direction's
  50%-transmittance surface, which really is a different surface from the view
  direction's, and the gap grows with thickness and obliquity. `bias_scale` is
  the knob; deep/opacity shadow maps are the real fix (future work).
- **A degenerate light direction casts nothing** rather than throwing —
  consistent with `blinn_phong`, where such a light already contributes nothing.
- At most `limits::max_shadow_maps` (4) casting lights; `render()` throws
  `cvc::volren_error` above that rather than silently dropping a light, for
  **both** backends (`cuda_limits::max_shadow_maps` is `static_assert`ed equal,
  so the CPU path can never accept a scene the kernel could not represent).
- **The volume does not cast onto cvcGL scene GEOMETRY** — see the note in the
  CUDA/cvcGL section below.

Those last two together decide what the feature actually *shows* in a scene,
and `volren_bunny` is the worked example. With nothing but volumes receiving,
a bunny of height `h` casts `h/tan(elevation)` across the ground, and its
neighbour on the 3×3 grid stands `1.35 h` away — so at the stage rig's 52° key
the shadow reaches `0.78 h`, lands on ground the volume cannot darken, and the
only visible change is the self-shadowed creases (measured: 709 pixels of a
1280×720 frame, the deepest crease falling to ambient). Below ~36° the reach
passes the pitch and the bunnies start shadowing **each other**, which is the
inter-volume path (measured at 22°: 5899 pixels, a recognisably bunny-shaped
umbra on four of the nine). That is why the demo's shadow section carries a
**key elevation** slider driving the raycast light and the VTK rig together:
without it the feature is real but nearly invisible in its own showcase.

### Cost

Measured on this box (GTX 1650, CUDA 12.0), `volren_bunny`'s scene: 64³ SDF
bunnies, `steps` 384, 640×360 rays, shadow map 512², median of 9.

| | main pass | rebuild frame | steady state |
|---|---|---|---|
| CUDA, 1 bunny | 12.4 ms | +45 ms | **+0.3 ms (2.2%)** |
| CUDA, 4 bunnies | 19.3 ms | +46 ms | **+0.7 ms (3.5%)** |
| CUDA, 9 bunnies | 34.9 ms | +70 ms | **+1.0 ms (3.1%)** |
| CPU, 1 bunny | 36.1 ms | +94 ms | **+1.3 ms (3.4%)** |
| CPU, 9 bunnies | 43.8 ms | +78 ms | within noise |

The **rebuild** column is a frame where the map is (re)built — the first frame,
and any frame after a light, transform, volume or settings change. The **steady
state** is every other frame, including all camera motion: that is the whole
point of the cache. The per-frame lookup is three dot products, one float load
and two compares per *shaded contribution* per casting light, and contributions
are per **cell entered**; for the demo's opaque isosurface that is 1–3 per ray,
hence the few percent. A shaded-TF scene has 50–200 per ray and pays more.

`resolution` is **quadratic** in rebuild cost and inversely linear in the slope
bias: 256² → 512² → 1024² measures +15 / +45 / +149 ms (CUDA, 1 bunny).

**Backend of the light pass.** `backend::cpu` on the parent gives `cpu`;
`backend::cuda` *or* `automatic` gives **`automatic`** — never strict `cuda`,
because an internal pass can fall outside the device scope for reasons the
caller cannot control, and failing the whole frame over that would be
surprising. This does not weaken the CPU/CUDA parity contract: the map is
*data*, and a CPU-built map consumed by a CUDA main pass is byte-identical to
the same map consumed by a CPU main pass.

Two implementation warts, accepted knowingly: the nested pass constructs its
own `cvc::app::thread_info` and reports progress, so a caller watching the
progress bar sees it sweep twice on a rebuild frame; and
`boost::this_thread::interruption_point()` inside the nested pass propagates
`thread_interrupted` out of the outer `render()`, which is correct and needs no
handling.

## Scene-graph transforms

Each volume carries a `mat4 model_transform` — **row-major, column-vector
points, exactly the cvcGL convention** (`GraphicsNode::setTransform`'s
`double[16]`, the `matrix` state key), so a scene-graph node's composed world
matrix drops in verbatim.  Rays are marched in world space; each volume is
sampled through the transform's affine inverse (object-space raycasting), the
MC isosurface intersection runs on the untransformed-direction local ray so
its `t` stays directly comparable to the world march (and the depth map), and
gradients/normals come back to world space through the inverse-transpose.
`scene_bounds()` is the union of the transformed boxes.  The follow-up cvcGL
node only has to compose its world matrix during traversal and hand it over.

## cvcGL node (`inc/cvc/gl/VolRenNode.h`)

`VolRenNode` raycasts on a worker thread (one frame in flight, latest camera
wins) and composites the frame into the VTK scene as a depth-mapped translucent
quad. Its settings live in an embedded `cvc::volren::state_settings` at
`<node state path>.volren`, which is the source of truth: `setRenderConfig()`
writes the tree, the tree's handler refreshes the snapshot the next raycast
captures, and a script, a UI and a replicated peer therefore all drive one
object. The thin accessors are exactly read-modify-writes of that:

| accessor | reaches | note |
|---|---|---|
| `setSupersample(n)` / `supersample()` | `render_settings::supersample` | `[1, limits::max_supersample]`, rejected (not clamped) at `render()` |
| `setShadowsEnabled(b)` / `shadowsEnabled()` | `render_settings::shadows.enabled` | |
| `setShadowConfig(s)` / `shadowConfig()` | the whole `shadow_settings` | resolution, strength, bias… |
| `setResolutionScale(s)` / `resolutionScale()` | its own `volren.resolution_scale` key | clamped to `[MinResolutionScale, MaxResolutionScale]` = `[0.05, 2.0]` |
| `setContinuous(b)`, `setBackend(b)`, `backendUsed()` | node / raycaster state | |
| `raycastWidth()` / `raycastHeight()` | the raster of the last **applied** frame | the honest ray count for a readout, not `viewport × scale` re-derived |
| `invalidateVolumeData()` | deferred `invalidate_device_volumes()` | see below |

**The composite filters PREMULTIPLIED color.** The node uploads `frame::color`
**verbatim** — the raycaster's background is forced to black, so its RGB is
already the volume's color times its own alpha — and a `//VTK::TCoord::Impl`
fragment replacement divides alpha back out *after* VTK samples and modulates
the texture. That ordering is the whole point: bilinear interpolation is linear
only in premultiplied space. Un-premultiplying on the CPU before the upload (as
the node originally did) hands the filter straight alpha, whose transparent
texels carry RGB=0, and the filter then blends that in as if it were black
paint — a dark halo on every silhouette, smeared over several pixels when the
raster is magnified (`scale < 1`, the default) and a quarter-strength edge when
it is minified (`scale > 1`). Measured on the demo's single bunny, switching to
the premultiplied upload changed 1076 pixels at `scale 0.5` and 277 at
`scale 2.0`, **every one of them brighter**, worst case +62/255, and nothing
else in the frame. `cvcgl_volren_node`'s halo check pins it: a flat-lit body
over a backdrop, asserting that no pixel of the blend region is darker than the
backdrop — 199 with the premultiplied upload (i.e. no fringe at all) against
160 with a straight-alpha one.

The divide is by the *final* fragment alpha, which equals the texture's alpha
because the node draws this quad unlit at opacity 1. That is a real constraint,
not an accident: `setOpacity()` on a `VolRenNode` would have its opacity divided
straight back out.

**`invalidateVolumeData()`** is the node's forwarder for the resident-device-
cache escape hatch. It is *deferred*: the raycaster belongs to the worker
thread, so the call arms a flag the worker consumes after re-registering the
volumes and before marching — after, because `clear_volumes()` resets the
content generations the announcement has to bump, and registration deliberately
does not draw a new one. Bumping them also busts the shadow-map fingerprint,
which reads the same generations. It bumps the node's settings version too, so
the stale frame on screen is re-raycast.

## CUDA backend (`inc/cvc/volren/raycaster_cuda.h`, `src/cvc/volren/raycast.cu`)

`set_backend(backend::cuda | backend::automatic)` runs the march on the GPU:
one thread per pixel, the same ray generation, slab intersect, per-ray volume
cull, Amanatides-Woo isosurface DDA, per-cell TF march, front-to-back
compositing, supersampled resolve and shadow lookup. A supersampled pixel marches its `n²`
rays *serially inside its own thread* rather than spreading them over threads:
the frame buffers, the launch geometry and the per-thread footprint are then
all independent of `supersample` (only the loop trip count moves), and the
resolve's arithmetic order cannot depend on the scheduler. It is a **semantic**
mirror, not a bit-exact one — `raycast.cu` is
compiled with `--use_fast_math` in Release — so parity is asserted at the image
level in `volren_cuda_test.cpp`, never as float equality. (The three shadow
parity scenes in fact come out **bit-identical**: the lookup is double-precision
dot products against a frame the two backends receive verbatim, and the only
float arithmetic it adds is one multiply by a visibility factor.)

The shadow maps themselves are built by an ordinary nested `render()` and reach
the kernel as *data*: `raycast_cuda_request::shadow_maps[]` carries each map's
flattened `shadow_view` plus a HOST pointer to its f32 depth raster, and the
host stages every raster into one device allocation (like `lut`). The frames
ride in the parameter block (~128 bytes each, taking it to 1912 bytes of
`cmem[0]`); a host-resolved `light_map[]` means the kernel never scans the maps
looking for a light. `ptxas` for sm_75 measures **3232 bytes stack frame / 255
registers**, against 3184 / 254 before shadows — no occupancy change.

`backend::automatic` falls back to the CPU silently when the device is missing
or the scene is out of scope; `backend::cuda` throws instead, so an explicit
request never quietly costs a CPU march. Either mode falls back (recording it
in `backend_used()`) on a device-side `cvc::cuda_error`. Scope:

| Cap | Value | Note |
|---|---|---|
| `cuda_limits::max_volumes` | 16 | see below |
| `cuda_limits::max_isosurfaces` | 8 | per volume |
| `cuda_limits::max_lights` / `max_cut_planes` | 8 | per scene |
| `cuda_limits::max_iso_hits_per_ray` | 32 | shared by all volumes on the ray; overflow keeps the NEAREST hits |
| `cuda_limits::max_shadow_maps` | 4 | `static_assert`ed equal to `limits::max_shadow_maps`, and `render()` enforces it for BOTH backends, so the device path never sees an over-cap scene |

**Why 16 volumes.** Sixteen per-volume blocks (transform, LUT descriptor,
isosurface table — ~600 bytes each) no longer fit the 4 KB kernel parameter
limit, so the array lives in device memory and the kernel takes a pointer; only
the scene-level settings still ride in the parameter block. What the cap really
bounds is per-thread state: the march keeps a last-cell tracker and a cull
window per volume, i.e. `16 · (3·8 + 2·8) = 640` bytes of local memory per ray
on top of the spline cache and the hit buffer (~2.2 KB per thread total).
Doubling the cap doubles that tail on every scene, single-volume ones included.
For the same reason a thread carries ONE spline-gradient cache keyed on
*(volume, cell)* rather than the CPU's one-per-volume: it is pure memoization,
so the values are identical, and 16 caches would be 13 KB per thread.

**Resident device volume cache.** Voxel blocks stay on the device between
renders and across `raycaster` instances, so a camera-only re-render — or
cvcGL's `VolRenNode`, which rebuilds its raycaster's volume list every frame —
launches with no host-to-device traffic at all. The invalidation rule:

1. An entry is keyed on *(device, host base pointer, byte length)*, validated
   against a content generation, and **co-owns the host block** through
   `voxels::active_storage()`. So the
   block cannot be freed while cached and its address cannot be recycled by a
   different volume — the free/re-malloc-at-the-same-address staleness is
   structurally impossible, not merely unlikely.
2. Because the cache is a co-owner, every write through the supported
   `cvc::volume` API copy-on-writes into a *different* block (`voxels::preWrite`
   sees a non-unique buffer). A mutated volume is a cache miss by construction;
   nothing has to be announced.
3. The one uncovered path is an in-place write through the unchecked legacy
   escape hatch `voxels::data_ptr()`, which no copy-on-write can see. The owner
   announces those with `raycaster::invalidate_device_volume(index)`, which
   bumps the generation; a generation mismatch *retires* the resident block
   (another render may be mid-kernel on it — it is freed when the last lease
   drops) and uploads a fresh one. Registering a volume does *not* draw a new
   generation — registration is not a content change, and cvcGL's `VolRenNode`
   re-registers every volume on every frame.
4. Eviction is least-recently-used against `raycast_cuda_set_cache_budget()`
   (default 512 MB). An entry an in-flight render is using is never evicted, so
   a budget smaller than the scene is exceeded rather than corrupted, and the
   lease that marks it in use is released even when the render throws.

`raycast_cuda_cache_bytes()` reports what is resident,
`raycast_cuda_cache_upload_bytes()` the voxel bytes ever pushed H2D (the
counter a "no upload on a camera move" assertion reads), and
`raycast_cuda_clear_cache()` frees everything not in flight.

## State-tree binding (`inc/cvc/volren/state_settings.h`)

```cpp
class state_settings : public cvc::state_object<state_settings> {
public:
  state_settings(cvc::app &ctx, const std::string &statePath,
                 std::function<void(const snapshot&)> apply = {});
  static std::string sceneStatePath(const std::string &prefix); // "<prefix>.volren"
  ...
};
```

Follows the `ShadowSettings` pattern exactly: synchronous handlers
(`setInstanceThreading(false)`), `seedState()` writes defaults (ints for
bools), `handleStateChanged()` re-reads everything under try/catch and calls
`apply`. Key map (all under `<prefix>.volren.`):

| Key | Type | Meaning |
|---|---|---|
| `camera.eye` / `camera.focal` / `camera.up` | "x,y,z" | pose (same shape as cvcGL `pose.eye`) |
| `camera.projection` | int | 0 perspective, 1 orthographic |
| `camera.vfov_degrees`, `camera.parallel_scale` | double | |
| `image.width`, `image.height` | int | raster size |
| `background` | "r,g,b" floats | |
| `steps`, `opacity_cutoff`, `depth_alpha_threshold` | int/double | |
| `supersample` | int | sub-samples per pixel EDGE; read RAW, `render()` owns the range check (like `steps`) |
| `two_sided_lighting` | int 0/1 | |
| `ambient` | double | |
| `threads` | int | |
| `lights` | flat CSV | `r,g,b,dx,dy,dz` per light |
| `cut_planes` | flat CSV | `px,py,pz,nx,ny,nz` per plane |
| `shadows.enabled` | int 0/1 | master switch |
| `shadows.lights` | flat CSV ints | casting light indices; `""` = every light casts. A separate index list rather than extra fields on `lights`, because 42 values would be ambiguous between 7 lights of 6 fields and 6 of 7 |
| `shadows.resolution` | int | light-view raster edge; CLAMPED on read (the `threads` convention) |
| `shadows.strength` | double | 0 = no-op, 1 = full |
| `shadows.bias_scale` | double | constant bias in latch quanta |
| `shadows.slope_scale` | double | slope bias in `texel_world * tan(theta)` |
| `shadows.min_occluder_opacity` | double | isosurface opacity floor to cast |
| `volumes.count` | int | number of bound volume-settings blocks |
| `volumes.<n>.shaded`, `.unshaded`, `.tf_auto_domain` | int 0/1 | |
| `volumes.<n>.matrix` | 16 CSV doubles | row-major model matrix (cvcGL `matrix` encoding) |
| `volumes.<n>.transfer_function.color` | flat CSV | `value,r,g,b` per point (VolumeNode encoding) |
| `volumes.<n>.transfer_function.opacity` | flat CSV | `value,a` per point (VolumeNode encoding) |
| `volumes.<n>.window` | "min,max" or "" | density window |
| `volumes.<n>.gradient_ramp` | "r0,r1,r2,plateau" or "" | the seeder always writes the plateau; a legacy 3-value string still parses and keeps the default |
| `volumes.<n>.isosurfaces` | flat CSV | `value,opacity,r,g,b,shininess` per surface |

The transfer-function encoding deliberately matches cvcGL `VolumeNode`'s
`transfer_function.color`/`.opacity` keys (flat comma-separated doubles,
independent color and opacity ramps) so the same editors drive both
renderers; `state_settings` merges the two ramps into the combined
`transfer_function` at the union of their control scalars.

## Fidelity vs. the legacy code

Preserved bit-of-the-look:

- Per-cell (not per-sample) contribution; front-to-back associated-color
  accumulation; 0.95 early termination (now `opacity_cutoff`).
- Quadratic-B-spline gradients (de Boor 4³ cache), used both for shaded TF
  samples and as the isosurface normal.
- MC cell classification with strict `<`, the VTK triangulation table, and
  nearest-positive-t hit selection (`iso_intersectW` contract), so raycast
  isosurfaces stay consistent with `cvc::iso(..., FASTCONTOURING)` meshes.
- Gradient-magnitude opacity modulation with the 0.9 plateau.  (One edge
  deviation: the legacy table index saturated at 255, so with `ramp2 >= 255`
  there was no upper cutoff; the port zeroes above `ramp2` — set `ramp2` to a
  huge value to reproduce the uncapped legacy configuration.)
- Blinn-Phong diffuse+specular with the 0.9 output gain.

Fixed (legacy defects, all catalogued in the port notes):

1. Multiple lights now accumulate (`+=`, was `=` — last light won).
2. Blue diffuse now uses the blue light channel (was green).
3. Multi-volume isosurfaces use *their* volume's origin/span (was env[0]'s).
4. Float volumes render natively (was `assert(0)`); any `cvc::data_type` works.
5. Shininess is a real per-surface exponent (was a baked x^10 table).
6. Cut planes actually clip (were parsed, stored, and ignored).
7. No raster rounding to multiples of 32; no dropped remainder tiles; no
   out-of-bounds tile shuffle; no per-thread env copies or leaks.
8. Saturated rays composite over the background correctly instead of the
   divide-by-alpha normalization hack.
9. `Viewing::raydir` / `Surface::shading` mutation during rendering is gone —
   the render path is `const` over settings and reentrant by construction.
10. A camera inside the scene bounds renders from its position (the legacy
    AABB test rejected any ray whose entry parameter was negative, so an
    inside camera saw only background).
11. Volumes respect a scene-graph model matrix (new capability — see
    "Scene-graph transforms").
12. Volumetric shadows (new capability — see "Volumetric shadows"). Off by
    default, so every image above is unchanged; its own deliberate limits are
    listed in that section, and the sharpest of them is that
    `volume_settings::unshaded` samples are **not** shadowed.

Dropped (dead or superseded — see the modernization plan):

- Materials/opacity trapezoids, `ColorMode` (never consumed by the tracer).
- The `.cnf` config format (replaced by the state tree), `vrLoadVolume`'s
  format zoo (superseded by `cvc::volume_file_io`), PPM output (superseded by
  `cvc::image::save`).
- MPI drivers (unbuilt legacy). Slab-decomposed out-of-core rendering is
  noted as future work.
- RawV multi-variable RGBA volumes — future work as an optional per-volume
  RGB source once a concrete consumer exists.

## Future work

- Per-sample (opacity-corrected) integration mode alongside the per-cell one.
- A spatial acceleration structure over the volumes (the per-ray cull is a
  linear scan, which is right for the tens of volumes the cap allows and wrong
  for thousands).
- Caching the baked LUTs on the device alongside the voxels. They are small
  (16 KB per volume) and change with the settings, so they are re-staged every
  render in one concatenated copy; only a scene with many volumes and a static
  transfer function would notice.
- A rotated-grid or low-discrepancy sub-sample pattern. For a perfectly
  axis-aligned edge the n×n regular grid only resolves n+1 of its n²+1 coverage
  levels, which a rotated grid would fix for the same ray count; it costs the
  "lands on a finer raster's pixel centers" identity that currently pins the
  offsets, so it needs a different check first.
- Adaptive supersampling (march the center ray, then only refine pixels whose
  neighbors disagree). It would recover most of the edge quality at a fraction
  of the rays, but the work per pixel stops being uniform — which is fine for
  the CPU tiles and bad for a warp.
- **Deep / opacity shadow maps.** The correct answer for translucent media, and
  the natural upgrade path — but it cannot be built on the existing depth latch:
  it needs a new *output* from the marcher (an accumulated-alpha-versus-t
  profile per light ray, not one latched scalar), which is a real change to
  `render_ray`'s inner loop and to `volren_raycast_kernel` on both backends,
  plus a K-slice per-ray buffer and a compression policy for the profile.
  `shadow_view` / `shadow_settings` are deliberately shaped so it is later a
  payload-and-lookup swap behind the same API.
- **PCF for shadows.** Average the binary test over a 2×2 or 3×3 texel
  neighbourhood in `shadow_visibility`, mirrored into both backends, plus a
  `shadows.pcf_radius` key. Explicitly deferred until the single-tap version is
  correct and measured, which it now is.
- **Device-resident shadow maps.** At 512² × 4 B = 1 MB each way that is ~0.2 ms
  on this PCIe link against a ~13 ms pass — noise. Same refactor as the resident
  volume cache when it becomes worth it.
- **Casting the volume onto cvcGL scene GEOMETRY.** Two independent blockers,
  both in the *node*, not in `cvc::volren`: `VolRenNode::applyFrame` re-poses
  its quad across the *viewer* camera's frustum every frame (and
  `applyTransformToVTK()` is deliberately a no-op), so from a light's viewpoint
  that quad is an arbitrarily-oriented billboard whose fragment depth was
  generated for the wrong camera; and `SceneGraph::setShadowsEnabled` builds
  `StridedShadowBaker -> vtkShadowMapPass -> vtkTranslucentPass`, where the
  baker and the shadow pass are **opaque-only** while the VolRenNode quad
  renders in the *translucent* pass, after shadows are resolved. Doing it
  properly means a light-view raycast built from `vtkShadowMapBakerPass::
  GetLightCameras()` (VTK's own baked camera, not our fit), converted with
  `depth_to_window_z()`, min-combined into `(*GetShadowMaps())[i]` by a baker
  subclass so every existing receiver shader picks the volume up unchanged.
  A cheaper honest interim exists and needs no VTK surgery: `shadow_view::project`
  is public precisely so a demo can compute a ground-plane decal texture from
  `shadow_map_view(0)` + `shadow_map_depth(0)` — a physically derived volume
  shadow on the one receiver that matters, entirely demo-side.
- `VolRenNode::invalidateVolumeData()` is all-or-nothing: it re-uploads every
  registered volume rather than the one that changed, because the node's
  snapshot model gives the caller no stable per-volume identity to name (the
  raycaster's indices are rebuilt every frame). One announced in-place write to
  a nine-bunny scene therefore costs nine uploads.
