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
  byte-identical regardless of thread count. Every lighting term added since
  keeps that: fixed tap grids, fixed cone offsets, no jitter anywhere.
- **New knobs are neutral at their defaults.** Anti-aliasing, volumetric
  shadows, deep shadow maps, soft shadows, ambient occlusion, the sky/ground
  ambient and the specular reflectance all default to the value that reproduces
  the previous image **byte for byte**, on both backends, and each is pinned
  that way by a `memcmp` test rather than a tolerance.

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

struct hemisphere_ambient {  // shapes `ambient` by the sample's own normal -- OFF by default
  bool enabled = false;
  std::array<float, 3> sky{1.f, 1.f, 1.f};     // toward +up
  std::array<float, 3> ground{1.f, 1.f, 1.f};  // toward -up
  std::array<double, 3> up{0.0, 0.0, 1.0};     // cvc scenes are Z-up
};

struct ao_settings {       // ambient occlusion on distance-field volumes -- OFF by default
  float  strength = 0.0f;  // 0 skips the cone entirely
  double radius   = 0.0;   // cone length, in the volume's LOCAL units; 0 is off
  int    samples  = 5;     // taps along the cone -- the quality/cost dial
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
  bool distance_field = false;   // the scalars ARE a signed distance, positive outside
                                 // (a claim about the DATA; only ao_settings reads it)
  std::vector<isosurface> isosurfaces;  // implies the legacy ISO_SURFACE bit when non-empty
};

struct render_settings {   // scene level
  std::array<float, 3> background{0.f, 0.f, 0.f};
  std::vector<light> lights;             // no lights + ambient 0 => shaded samples are black (legacy semantics)
  std::vector<cut_plane> cut_planes;     // legacy declared these but never implemented; live here
  shadow_settings shadows;               // volumetric shadows -- OFF by default, HARD when on (see below)
  bool two_sided_lighting = false;       // legacy light_both
  float ambient = 0.0f;                  // legacy zeroed ambient; now a real knob
  hemisphere_ambient ambient_hemisphere; // sky/ground tint on that constant -- OFF
  ao_settings ao;                        // occlusion on that constant -- OFF
  float shading_gain = 0.9f;             // the legacy output damping, now a knob
  float specular = 1.0f;                 // scene-level specular reflectance; 1.0 is the legacy
  int steps = defaults::steps;           // samples along the scene-bbox diagonal
  float opacity_cutoff = defaults::opacity_cutoff;        // early ray termination (0.95)
  float depth_alpha_threshold = defaults::depth_alpha_threshold; // where the depth map latches
  unsigned threads = 0;                  // 0 => pool default; 1 => serial
  int supersample = 1;                   // sub-samples per pixel EDGE: n => n x n rays
};
```

The shading expression, with every term named:

```
L = ambient * mix(ground, sky, ½ + ½ N·up) * ao * base                      // ambient
  + Σ_lights vis_i * (base * (N·L_i) * color_i + color_i * specular * (N·H_i)^shininess)
L = min(shading_gain * L, 1)                                            // per channel
```

`vis_i` is the shadow map's answer (soft-filtered when `pcf_radius > 0`), `ao`
is the occlusion cone's. **They attenuate different terms** — see "Energy: what
attenuates what" below, which is where the no-double-darkening claim is stated
and pinned.

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

**Isosurface DDA occlusion cutoff.** Isosurface hits are *collected* by a
per-cell DDA that runs to completion before the first sample composites, so the
`opacity_cutoff` in step 4 — which bounds *visible* work in the march loop —
cannot bound the DDA. Without a second cutoff a ray walks every cell of every
volume even when the first surface it crossed is fully opaque, and on a
nine-bunny scene that is ~40% of the whole frame spent on cells behind an
opaque surface.

The hit buffer is therefore kept **sorted by `t` as it is built** (insertion in
stable order, which is exactly what the collect-then-`stable_sort` it replaced
produced), and each volume's walk carries a forward scan over the hits strictly
nearer than the current cell, accumulating alpha with the same recurrence
`composite()` uses. When that reaches `opacity_cutoff` the walk stops: every
remaining hit it could find is one `composite_hits_up_to` would refuse, because
accumulated alpha only ever grows (later volumes and transfer-function samples
add to it), so a prefix that saturates now still saturates then. The device
path additionally stops once the buffer is full and the cell starts at or past
the farthest kept hit, which `push_hit` would discard anyway.

This is **pure work elimination — output is byte-identical**, verified by
`memcmp` against a pre-change render at 1/2/4/9 volumes on both backends.

**Isosurface-only scenes skip the march loop.** When no volume is `shaded` or
`unshaded` there is nothing for step 2–3's loop to composite: its body would
run the window test, `to_local_point` and `cell_index` at every one of `steps`
samples and take neither transfer-function branch. Both backends resolve one
`any_tf` predicate up front and, when it is false, go straight to the trailing
`composite_hits_up_to(t1)`. The hits composite in the same order against the
same limit, so this too is **byte-identical**; it is worth ~25% of an
isosurface-only frame on the device.

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
enum class shadow_mode { hard = 0, deep = 1 };

struct shadow_settings {
  bool enabled = false;      // OFF by default: every existing scene is byte-identical
  std::vector<int> lights;   // indices into render_settings::lights; EMPTY == all cast
  int   resolution = 512;    // light-view raster EDGE, square, clamped to [64, max_raster_dim]
  float strength = 1.0f;     // 0 = byte-identical no-op, 1 = the light contributes nothing
  float bias_scale = 1.0f;   // constant bias, in latch quanta (see below)
  float slope_scale = 1.0f;  // slope bias, in texel_world * tan(theta)
  float min_occluder_opacity = 0.5f;  // hard mode only: opacity floor for an isosurface to cast
  shadow_mode mode = shadow_mode::hard;   // DEFAULT hard: no existing scene moves
  int depth_slices = 16;     // deep mode only; clamped to [1, 64]
  float pcf_radius = 0.0f;   // soft shadows: filter half-width in light-map TEXELS.
                             // 0 is the single-tap comparison, byte for byte.
  int   pcf_taps = 3;        // taps per EDGE, so taps^2 reads; clamped to [1, 7]
};

struct shadow_view {          // the light-view frame of one built map; PUBLIC on purpose
  std::array<double,3> eye, right, up, forward;   // forward == -normalize(light.direction)
  double parallel_scale, texel_world;             // half-height == half-width; 2*ps/height
  int width, height, light_index;
  bool project(const std::array<double,3> &p, int &ix, int &iy, double &depth) const;
  // deep maps only (slices == 0 for a hard map): the profile's knot grid, in
  // light-space depth.  Knot j sits at depth_min + j * slice_dz.
  int slices; double depth_min, slice_dz; double depth_max() const;
};
```

`raycaster` exposes the maps the last `render()` built:

```cpp
std::size_t      shadow_map_count() const;
cvc::image       shadow_map_depth(std::size_t i) const;   // GRAY f32, +inf where the light missed
cvc::image       shadow_map_profile(std::size_t i) const; // deep payload; EMPTY for a hard map
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

Everything down to "Deep shadow maps" below describes `shadow_mode::hard` — the
default, the cheaper of the two, and the one whose behaviour every pre-existing
scene keeps byte for byte. `shadow_mode::deep` keeps *all* of this geometry, the
same bias and the same caching, and changes only the per-texel payload and the
lookup; it adds one output to the light pass and nothing to the main march.

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
filter, the nearest-of-n resolve would dilate every occluder by half a pixel of
its silhouette, and a deep profile has no defined resolve at all), and `shadows`
is cleared (no recursion). Everything else — `steps`, `cut_planes`,
`opacity_cutoff`, `depth_alpha_threshold`, `threads` — is inherited, which is
why a cut plane that removes an occluder removes its shadow too.

In `shadow_mode::deep` there is a fourth difference: the per-volume isosurface
list is **not** filtered by `min_occluder_opacity`, because a deep map
represents a low-opacity surface rather than needing it thresholded away.

### Caching

The maps are **camera-independent**, and they are cached on a fingerprint of
the light pass's own inputs: lights, volumes (buffer identity + content
generation + geometry), model transforms, per-volume settings, cut planes,
`steps`, scene bounds and the shadow settings. Moving the camera does **not**
rebuild. `strength`, `bias_scale` and `slope_scale` are consumed by the main
march, not by the light pass, so they are deliberately absent from the key.

`mode` and `depth_slices` enter the key through the resulting **slice count**:
the payload's shape is part of the map's identity, so flipping the mode rebuilds
rather than reinterpreting bytes laid out for the other. `min_occluder_opacity`
enters it only in hard mode — deep mode does not read it, and hashing it there
would rebuild for a knob the light pass ignored.

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

### Deep shadow maps (`shadow_mode::deep`)

`shadow_mode::hard` — the default, and everything above — stores **one scalar
per light-view texel**: the depth where the light stopped. That makes every
occluder fully opaque and every shadow binary. `shadow_mode::deep` stores the
light ray's **accumulated opacity as a function of depth**, so a translucent
occluder casts a partial shadow and a sample deep inside a medium is attenuated
by exactly what lies between it and the light.

Both modes share this file's geometry, the same `strength`, the same bias
formula and the same caching. They differ in the per-texel **payload** and in
the **lookup**.

#### The payload, and why it has two channels

The renderer's occluders come in exactly two kinds, and each gets the
representation it needs:

- An **isosurface** is a *step* in accumulated alpha at an exactly known depth.
  Uniform slices cannot represent a step — they smear it over one slice width,
  which on the flagship content (an opaque SDF isosurface) would turn every
  contact shadow into a soft onset several cells long.
- A **transfer-function medium** accumulates *gradually*, one contribution per
  cell entered, so a piecewise-linear reconstruction over a handful of knots is
  genuinely accurate for it — and it is the case a hard map cannot represent at
  all.

So each texel carries `depth_slices + 1` floats:

| plane | contents |
|---|---|
| 0 | the **exact** light-space depth at which this ray's accumulated alpha first reached `opacity_cutoff`, or `+inf` |
| `1 + k` | accumulated alpha at knot `k+1` (light-space depth `depth_min + (k+1)·slice_dz`), counting only contributions **strictly in front of** that terminal depth |

Contributions at or beyond the terminal are deliberately excluded from the
slices, so a receiver *in front of* an opaque occluder is never dimmed by the
interpolation ramping early into the step.

**Uniform in `t`, not in accumulated alpha, and not a compressed
piecewise-linear curve.** Uniform-in-alpha (store the depth at which each of K
opacity quantiles is crossed) and Lokovic–Veach-style compressed node lists both
represent a step exactly, which is the attraction — but both make the lookup a
**search** over up to K entries, divergent across a warp, where uniform-in-`t`
is `floor((z - z0)/dz)` and two loads. The lookup runs once per shaded
*contribution* per casting light — for a marched medium that is once per cell
entered, hundreds per ray — so O(1) is the property worth buying, and the exact
terminal channel recovers the one thing uniform slices cannot express. The knot
grid is fitted to the scene's **own** light-depth extent (`fit_light_camera`
already computes it exactly, from the same box corners it fits the ortho window
to), so no knot is spent outside the marched region.

**Layout: PLANE-major**, `slices + 1` planes of the light-view raster stacked
into one GRAY f32 image (`width` × `height·(slices+1)`). Both layouts were built
and measured; on this box they are within noise of each other on both backends,
for the lookup and for the light pass's writes alike. Plane-major is kept
because every plane is then a plain `width × height` raster — plane 0 in
particular has the exact shape and meaning of a *hard* map's depth raster — and
because it is the coalescing-friendly layout on the device.

#### The lookup

```
if (depth > profile[texel] + bias)          -> 1 - strength      // terminal channel
u = (depth - bias - depth_min) / slice_dz                        // knot coordinate
alpha = lerp of the two bracketing knots (knot 0 is implicitly 0, held past the last)
                                            -> 1 - strength * alpha
```

- The **terminal** comparison is written as `depth > stored + bias`, *not* the
  algebraically equal `depth - bias > stored`. That is what makes an opaque
  occluder reproduce the hard lookup **bit for bit** rather than merely closely.
- **Interpolation space: linear in accumulated alpha** (equivalently in
  transmittance, its affine image), against light-space **depth**. Not optical
  depth / log-transmittance, which is the reflex when a quantity is called
  transmittance: this renderer composites with the discrete front-to-back alpha
  recurrence, not Beer–Lambert, so there is no `tau` for a reconstruction to be
  linear in — and log space cannot represent the transmittance 0 that opaque
  media reach constantly. Linear in alpha also makes the value *at* a knot exact
  (it is the accumulation a co-located sample would have produced) and keeps the
  reconstruction monotone for free.
- The **texel** lookup is nearest by default, exactly as in hard mode. Lateral
  filtering IS available in both modes through `shadows.pcf_radius` /
  `shadows.pcf_taps` (see "Soft shadows"); at the default `pcf_radius` 0 the
  filter collapses to that single nearest tap.
- **Bias**: unchanged formula, unchanged knobs. It shifts the *query* toward the
  light so a receiver is not attenuated by its own contribution — the same job
  it does in hard mode, where it is added to the stored depth instead.
- The result `1 - strength · alpha` collapses to hard's
  `shadowed ? 1 - strength : 1` whenever alpha is 0 or 1, which is the identity
  `ShadowMap.DeepVisibilityChannelsAndInterpolation` asserts directly.

#### Producing the profile

The transmittance-vs-depth curve is a **new output of the marcher**, and the
constraint is that the ordinary render path must not pay for it.

- It is **not** a `render_settings` field. It is meaningful only for the
  orthographic light-view pass, whose ray parameter `t` *is* the light-space
  depth (`origin` on the eye plane, `direction == forward`, so
  `dot(p - eye, forward) == t`). `ensure_shadow_maps()` requests it by writing a
  private `profile_request` on the temporary light-pass raycaster and reads
  `_profile_out` back; `render_settings` and `frame` are untouched.
- Contributions reach `composite()` in non-decreasing `t`, so the profile
  streams out with a **single forward cursor** — no per-ray slice array on
  either backend, each knot written exactly once, straight to the output buffer.
- The buffer is **pre-filled** (`+inf` terminal, zero alphas) so the rays that
  return early — missed the scene box, reached no volume — are correct for free.
  On the CPU that pre-fill happens *after* the backend split, because the
  capture kernel writes every slot of its own pixel and would throw a 17.8 MB
  host memset away (7 ms at 64 slices).
- On the device, capture is a **kernel template parameter**
  (`volren_raycast_kernel<CAPTURE>`), not a runtime `if (q.prof_out)`. The
  ordinary render launches a separate instantiation with no capture state in
  registers and no extra predicate in the compositing recurrence.

**Measured cost to the ordinary path: none.** `ptxas` for sm_75 reports the
non-capture instantiation at **3232 bytes stack frame, 12 B spill stores, 8 B
spill loads, 255 registers** — the same numbers as before deep shadows existed.
Only `cmem[0]` moves, 1912 → 2072 bytes, which is the four shadow-map frames
gaining their knot-grid fields in the parameter block. Wall clock, the same
library built twice with the CPU capture branches statically removed, 8
interleaved rounds of min-of-9 at 640×360:

| scene | backend | capture removed | capture present | delta |
|---|---|---|---|---|
| opaque isosurface, 96³ | CPU | 30.55 ms | 30.15 ms | −1.3% |
| opaque isosurface, 96³ | CUDA | 11.96 ms | 11.93 ms | −0.2% |
| shaded TF haze (one contribution per cell, hundreds per ray) | CPU | 307.4 ms | 289.9 ms | −5.7% |
| shaded TF haze | CUDA | 152.7 ms | 153.2 ms | +0.3% |

Every delta is inside this box's run-to-run spread, and the densest case comes
out *faster* with the code present — which is what "no cost" looks like on a
noisy machine, not a speedup.

#### Memory

Per casting light, on the host **and** on the device:

```
resolution² · (depth_slices + 1) · 4 bytes
```

| resolution | slices | per light |
|---|---|---|
| 512² (default) | 16 (default) | **17.8 MB** |
| 512² | 32 | 34.6 MB |
| 512² | 64 (`limits::max_shadow_depth_slices`) | 68.2 MB |
| 256² | 16 | 4.5 MB |

Against `cuda_limits::default_cache_bytes` (512 MB) and one 512³ `UShort` volume
(256 MB) that is small, but it is three orders of magnitude more than a hard
map's 1 MB, which changed two decisions:

- **Shadow rasters are now device-RESIDENT**, in the same cache as the voxels
  (keyed on host pointer + byte length, co-owned through a pin, validated
  against a generation — a `cvc::image` is a refcounted copy-on-write buffer, so
  the argument is identical to `cuda_volume::pin`'s). They used to be re-staged
  every render, which was defensible at 1 MB; at 17.8 MB it measured **+6.7 ms
  on a 6 ms frame**, and it was paying per frame for data the map cache
  deliberately rebuilds only per scene change. Cost after the fix: **+0.5 ms**.
  `raycast_cuda_cache_bytes()` and friends now report voxels *and* shadow
  rasters, sharing one budget under one LRU.
- **Nothing per-ray scales with `depth_slices`** — neither backend keeps a slice
  array — so the cap is a memory bound, not an occupancy or register one.

One consequence of residency worth knowing: a rebuild allocates *fresh* images,
so it lands on a new cache key and the previous map's blocks linger until the
LRU evicts them — and because the cache co-owns the host `cvc::image`, they
linger on the host too. That is the volume cache's pre-existing behaviour for a
mutated volume, but a deep map amplifies it: dragging a light rebuilds every
frame, so the budget fills with superseded maps rather than with one. It is
bounded (`raycast_cuda_set_cache_budget()`, 512 MB by default) and correct, but
a caller that drags a light through a 4-light scene at 64 slices should expect
the cache to sit at its budget.

#### How many slices

Two regimes, and they want very different things.

When the occluder and the receiver are **separated in light depth** — the
ordinary "A shadows B" case — the receiver only ever queries the profile's flat
tail, and **2 slices already reproduce a 256-slice map exactly** (0 pixels off,
measured on the plate-and-ball scene for both a smooth TF occluder and a
translucent isosurface).

When the receiver sits **inside** the medium — self-shadowing, which is the case
deep maps exist for — the knot resolution is what the reconstruction rides on.
A 64³ translucent ball with an embedded translucent shell, 128² frame, against
its own 512-slice reference:

| slices | mean abs ΔL | max abs ΔL | pixels off by >1 LSB |
|---|---|---|---|
| 1 | 9.46 | 60 | 66.9% |
| 2 | 21.02 | 59 | 77.3% |
| 4 | 8.88 | 45 | 71.5% |
| 8 | 5.11 | 26 | 57.2% |
| **16 (default)** | **1.60** | **9** | **40.4%** |
| 32 | 0.56 | 4 | 7.9% |
| 64 | 0.00 | 0 | 0.0% |

16 is the default because it is where the error drops under ~1 LSB on average
on the *hardest* case while the map is still 17.8 MB and the rebuild still
interactive; 32 halves the worst pixel again for double the memory. (The
non-monotone 1 → 2 → 4 rows are real and expected: with one or two knots the
whole reconstruction is one or two ramps, and where those ramps land relative to
the medium is arbitrary.)

#### Cost of the deep payload

`volren_bunny`'s scene (64³ SDF bunnies, `steps` 384, 640×360, map 512²), median
of 7. "Steady state" is every frame that reuses the cached map — all camera
motion; "rebuild" is a frame that pays the light pass.

| | steady state, hard | steady state, deep | rebuild delta, hard | rebuild delta, deep |
|---|---|---|---|---|
| CUDA, 1 bunny | +0.01 ms | +0.14 ms | +16.7 ms | +41.7 ms |
| CUDA, 4 bunnies | +0.24 ms | +1.02 ms | +22.0 ms | +38.2 ms |
| CUDA, 9 bunnies | +0.46 ms | +0.74 ms | +34.5 ms | +49.6 ms |
| CPU, 9 bunnies | +0.78 ms | +2.02 ms | +58.6 ms | +64.0 ms |

On that scene the deep render is byte-identical to the hard one (opaque
occluders), so the steady-state column is the pure price of the payload. The
denser test is a self-shadowing translucent medium, where the lookup fires once
per cell entered — 96³ haze plus a translucent shell, 640×360, `steps` 512, min
of 3 rounds × 7:

| | shadows off | hard | deep |
|---|---|---|---|
| CUDA | 83.56 ms | 85.31 ms (+2.1%) | 86.02 ms (+2.9%) |
| CPU | 205.96 ms | 198.06 ms | 204.66 ms |

Deep costs **+0.7 ms over hard** on an 86 ms frame at the densest lookup rate
this renderer can produce. The CPU numbers are all inside the noise floor.

The **rebuild** premium is the honest cost, and it is transfer, not compute: the
light pass writes the profile into device memory, copies it *out* to the host
image (the map is host data both backends share), and the main pass copies it
back *in*. That is 2 × 17.8 MB per rebuild at 16 slices, and it scales linearly
with the slice count (measured at 1/4/16/64 slices, before the residency fix:
+1.8/+2.4/+6.7/+23.3 ms — exactly PCIe bandwidth). See "Future work".

#### Determinism

Unchanged, and it now covers the map: the same scene rendered at
`threads` 1, 2, 4 and "pool default" produces a byte-identical frame **and a
byte-identical 4.2 MB profile**. The capture is a per-ray forward cursor writing
disjoint slots — no atomics, no ordering dependence.

### Soft shadows (percentage-closer filtering)

`pcf_radius > 0` replaces the single depth comparison with the box average of
`pcf_taps²` comparisons on a regular grid spread over ±`pcf_radius` **light-map
texels**. It applies to both payloads — a hard map's binary test and a deep
map's reconstructed transmittance — and it is the cheapest large perceptual
change available here, because a hard-edged shadow is the single most obviously
synthetic thing in the image.

**The two knobs are orthogonal, and that is the design.** `pcf_radius` sets the
penumbra's WIDTH; `pcf_taps` sets how many levels that band resolves into.
Measured on a ball floating 8 units over a plate, 128³, map 512², where one
texel is 4.0 screen pixels:

| radius (texels) | penumbra width | levels at 3 taps | at 5 | at 7 |
|---|---|---|---|---|
| 0 | 0 px (2 levels total) | — | — | — |
| 1 | 4 px | 4 | 4 | 4 |
| 2 | 8 px | 7 | 8 | 8 |
| 4 | 16 px | 7 | 15 | 16 |
| 8 | 32 px | 8 | 17 | 27 |
| 16 | 64 px | 8 | 23 | 38 |

Exactly 4 px of penumbra per texel of radius, independent of the tap count; the
tap count buys levels inside that band and never widens it. At a wide radius
3 taps is a visible staircase and 7 is a gradient — which is what makes the pair
a quality/cost dial rather than one knob with two names.

**Structure.** The light camera is orthographic, so a receiver's light-space
depth does not depend on which texel it is compared against: the filter does
**one** projection (three dot products) and then offsets the map index. A
perspective light would have to re-project per tap.

**The taps are fixed, unjittered and unrotated.** A per-pixel rotated Poisson
disk resolves a wide penumbra into more levels for the same tap count and is the
standard answer — and it is refused here, because determinism (byte-identical
across runs and thread counts) is a documented contract of this renderer and a
per-pixel rotation is exactly what breaks it. The regular grid also keeps the
two backends comparable tap for tap; they measure **byte-identical** on every
soft-shadow scene in `VolrenCudaTest.SoftShadowParity`.

**The bias widens with the filter, by `1 + 2·pcf_radius`.** The slope term
covers the depth error across the LATERAL footprint of what a receiver is
compared against, and PCF is precisely the operation that grows that footprint:
without the widening the outer taps self-shadow and the soft edge arrives with a
ring of acne inside it. At radius 0 the factor is exactly `1.0`, which is what
keeps an unfiltered render byte-identical rather than merely close
(`ShadowMap.PcfWidensTheSlopeBiasByExactlyTheFootprint`). The cost is the usual
one: a wide filter peter-pans contact shadows, so `pcf_radius` and `bias_scale`
are adjusted together.

**Cost** (interleaved A/B in one process — same cache state, alternating, so
drift on this box cancels — `volren_bunny` scene, map 512², min of the rounds):

| taps | reads/sample | CUDA, 1 bunny | CUDA, 9 bunnies | CPU serial, 1 bunny |
|---|---|---|---|---|
| 1 (off) | 1 | 5.81 ms | 16.38 ms | 24.66 ms |
| 3 | 9 | +0.39 ms (+6.7%) | +1.74 ms (+10.6%) | +0.14 ms (+0.6%) |
| 5 | 25 | +0.98 ms (+16.8%) | +3.56 ms (+21.7%) | +0.25 ms (+1.0%) |
| 7 | 49 | +1.92 ms (+33.0%) | +6.21 ms (+37.9%) | +1.18 ms (+4.8%) |

Roughly linear in the tap count, as it should be: the filter adds texel reads
and nothing else, and it is paid per shaded contribution per casting light. In
deep mode each tap is two loads rather than one.

The CPU column is an order of magnitude cheaper in RELATIVE terms and that is
not noise — the CPU march is memory-bound elsewhere, so a few more cached reads
hide in the stall, while the device kernel is issue-bound and pays for every
one. (The CPU figures are taken at `threads = 1` and 320×200: with the pool
running on this box, the whole sweep sits inside the scheduling noise and comes
out non-monotone, which is a measurement artifact and not a result.)

### Deliberate limits

Hard mode only:

- **The occluder test is binary**, scaled by `strength`. `pcf_radius` softens
  the RESULT of that test across the map's own texel grid, which is a penumbra
  in the image but not a physically sized one: a real penumbra grows with the
  occluder-to-receiver distance and this one is a constant number of texels
  everywhere. Contact-hardening would need a blocker-search pass — see
  "Future work".
- **One occluder layer per light ray** — the latch is a single scalar, so a
  point behind two thin sheets is exactly as dark as behind one. Deep mode
  composes layers correctly; `DeepShadowsStackOccluderLayersThatHardShadowsCannot`
  pins both halves of that sentence.
- **Translucency is ignored on the caster side.** The latch fires on the first
  isosurface hit whatever its opacity, which is what `min_occluder_opacity`
  exists to bound: without it the `volren_bunny --shell` decorative shell
  (opacity 0.16, four world units out) becomes the occluder for the body it
  wraps. Transfer-function volumes still cast through the alpha latch and are
  not filtered.
- **A genuinely thick translucent medium self-shadows at grazing incidence**,
  and no bias fixes it: the map records the *light* direction's
  50%-transmittance surface, which really is a different surface from the view
  direction's, and the gap grows with thickness and obliquity. `bias_scale` is
  the knob in hard mode; deep mode is the fix.

Deep mode only:

- **`min_occluder_opacity` is IGNORED.** It exists only because a hard latch
  cannot represent a low-opacity surface at all, so the surface is deleted from
  the light pass instead. A deep map represents it — the `--shell` case now dims
  what it wraps by its true 16% — and filtering there would delete an occluder
  the payload can express exactly. Pinned by `DeepShadowsIgnoreMinOccluderOpacity`.
- **Accumulation past `opacity_cutoff` collapses to fully blocked**, discarding
  the residual `1 - opacity_cutoff` (5% at the default). That is the same
  early-ray-termination approximation the main march already makes, and it is
  what lets an opaque occluder be exact; it is pinned as a *contract* by the
  saturating half of `DeepShadowsStackOccluderLayersThatHardShadowsCannot`.
- **A translucent isosurface's step is smeared over one slice.** Only the
  terminal channel is exact in depth, and it fires on saturation, so a
  0.6-opacity surface in open space is represented as a `slice_dz`-wide ramp
  rather than a step. `depth_slices` is the knob; `slice_dz` is the scene's
  light-depth extent over it.
- **The profile is 17.8 MB per light at the defaults**, host and device, and the
  rebuild pays a host round trip for it. Neither matters while the map is cached
  (all camera motion), both matter when a light or a transform is being dragged.

Both modes:

- **`volume_settings::unshaded` samples are NOT shadowed.** That mode is defined
  as "TF readout with no lighting model"; there is no light term to attenuate,
  and darkening it would invent a lighting model for the one mode whose
  contract is that it has none. Pinned by
  `UnshadedSamplesAreNotShadowed`.
- **A degenerate light direction casts nothing** rather than throwing —
  consistent with `blinn_phong`, where such a light already contributes nothing.
- **The PCF taps do not interpolate.** Each is a nearest-texel read, so a
  `pcf_radius` under `(pcf_taps-1)/2` collapses several taps onto one texel and
  the filter stops widening. A filter narrower than a texel cannot blur, and
  making it pretend to would cost a bilinear read per tap for no visible gain.
  The rounding to a texel is **away from zero at a half**, not up, so the grid
  is exactly its own mirror image: the filter is a box average *centered* on the
  receiver, and `floor(x + 0.5)` breaks that at every tap whose exact position
  is a half texel — 12.5% of legal radii at 3 taps, 20.9% at 7, worst case a
  footprint whose centroid sat a third of a texel off the sample (5 taps at
  radius 5 gave `{-5, -2, 0, 3, 5}`). That slides the whole penumbra, and it
  slides it *by a different amount* as the radius crosses each half-integer, so
  a caller dragging the slider would watch the shadow creep.
  `ShadowMap.PcfGridIsInertUnlessBothKnobsAskForIt` asserts the symmetry
  identity over the knob's whole legal range rather than the rounding rule that
  happens to deliver it. The measured penumbra table above is unchanged by this
  (its radii are all integers).
- At most `limits::max_shadow_maps` (4) casting lights; `render()` throws
  `cvc::volren_error` above that rather than silently dropping a light, for
  **both** backends (`cuda_limits::max_shadow_maps` is `static_assert`ed equal,
  so the CPU path can never accept a scene the kernel could not represent).
- **The volume does not cast onto cvcGL scene GEOMETRY** — see the note in the
  CUDA/cvcGL section below.

Those last two together decide what the feature actually *shows* in a scene,
and `volren_bunny` is the worked example (`--soft-shadows R`, `--ao S` and `--rig`
drive the lighting terms from the CLI, and the panel's shadow and ambient
sections expose every knob live). With nothing but volumes receiving,
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

### Cost of the light pass (both modes)

Measured on this box (GTX 1650, CUDA 12.0), `volren_bunny`'s scene: 64³ SDF
bunnies, `steps` 384, 640×360 rays, shadow map 512², median of 9. Hard mode;
the deep payload's own premium is tabulated above.

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

## Ambient occlusion (`ao_settings`, `volume_settings::distance_field`)

Ambient was a **flat constant added to every sample whatever way it faces**,
which is why a crevice read exactly as bright as an exposed dome. `ao_settings`
replaces that constant, on volumes that declare themselves distance fields, with
a locally measured one.

**Why a distance field makes this cheap enough to ship.** The honest cost of
ambient occlusion is a hemisphere integral — many rays, each marched. A signed
distance field collapses that to a handful of point samples, because `f(q)` is
the distance from `q` to the nearest surface **in any direction**: one fetch
certifies that a whole SPHERE of radius `f(q)` around `q` is empty. Walking a
few points out along the normal and asking, at each, "is the free sphere here as
big as the distance I have travelled?" is therefore a genuine neighbourhood
measurement, not a single-ray one — which is what makes five taps a usable
answer where five shadow rays would be noise.

```
occ = Σᵢ wᵢ · clamp((hᵢ - (f(p + n·hᵢ) - isovalue)) / hᵢ, 0, 1) / Σᵢ wᵢ
      hᵢ = radius · i / samples,   wᵢ = 1/i,   i = 1 … samples
ambient *= 1 - strength · occ
```

Exact in both limiting cases: an isolated convex surface reads 0 (`f(p+n·h) == h`
at every tap, so every term vanishes) and a sealed point reads 1.

**Decisions, and the reasons.**

- **It attenuates AMBIENT only.** Direct light already carries an exact
  per-light visibility term (the shadow maps); folding a local occlusion
  estimate into it would darken the same occluder twice. The ambient term is the
  one that pretends every direction is equally visible, and therefore the one an
  occlusion estimate is *about*. The consequence is worth stating plainly:
  **with `ambient` at 0 — the default — AO changes no pixel**, and the renderer
  skips the cone outright rather than running it and multiplying by zero.
- **Isosurface hits only.** A shaded transfer-function sample fires once per
  CELL ENTERED — hundreds of times along a ray, against one or two isosurface
  hits — so the same cone would cost two orders of magnitude more there. And
  "distance to the surface" has no meaning for a medium with no surface. The
  hemisphere tint still applies to TF samples; the cone does not.
- **`distance_field` is a claim about the DATA, not a mode.** Nothing else in
  the renderer reads it. A volume without it contributes no occlusion, which is
  the honest answer for a field where `f - isovalue` means nothing — rather than
  a plausible-looking number derived from an assumption the caller never made.
- **The cone measures ONE volume: the one whose isosurface was hit.** A hit on
  volume *A* is occluded by *A*'s own geometry and by nothing else, so two
  overlapping distance fields do not shade each other's creases and a volume
  resting on another gets no contact darkening from it. That is a deliberate
  scope, not an oversight: the estimator's whole cheapness rests on `f(q)`
  being *the* distance to the nearest surface, and the min over several
  volumes' fields is only a distance field again when they share a frame and a
  level set — which nothing enforces and the 3×3 bunny grid does not have (each
  volume carries its own `model_transform`). Doing it properly means either an
  N-volume min per tap, which multiplies the cost by the volume count for a
  term worth a fraction of `ambient`, or a fused scene SDF, which is a
  preprocessing pass and a different feature. Shadows already cover
  volume-to-volume darkening exactly, and they do it on the direct term where
  the energy actually is. Contact AO between volumes is listed under
  "Future work".
- **The isovalue is subtracted**, so an offset surface works identically: the
  `--shell` decorative surface at distance 4 is the level set of an SDF too.
- **The cone marches in the volume's LOCAL frame**, along the gradient divided
  by the voxel span. The shading normal deliberately skips that divide (it is
  what the legacy renderer did, and correcting it would move every pixel of
  every anisotropic scene) but the cone cannot: it compares the field against a
  distance travelled, and a direction wrong by the span ratio makes that
  comparison meaningless rather than merely off-axis. On the cubic grids a
  distance field is normally sampled on, the two agree exactly.
- **Falloff `1/i`, not `1/2^(i-1)`.** Geometric falloff is the textbook choice
  and it is wrong for a knob-driven radius: at the 16-sample ceiling it gives the
  outer HALF of the cone 0.006% of the answer, so `radius` silently stops
  meaning anything past a few taps. `1/i` still puts the near taps in charge —
  the first outweighs the last by `samples` to one — while leaving the far end
  able to register a wall (`Occlusion.FalloffIsNearWeightedAndTheRadiusKeepsMeaning`).
- **A tap that leaves the grid counts as UNOCCLUDED at full weight.** Dropping
  it would renormalize the cone onto the taps that remain, so a surface near the
  volume's boundary would darken as its cone ran out of data — precisely
  backwards.

**What it does to the image.** On two overlapping balls lit by ambient alone
(so every pixel *is* the occlusion factor), 48³, strength 1, radius 0.3 against
a 0.4 ball radius: the seam loses 22 of 204 levels while the open cap loses **at
most 1**, and every pixel that loses more than 4 levels lies within 12 px of the
seam. On the flagship bunny the difference map lights up the ear cups, the
crease behind the ear, the neck/shoulder and leg/haunch junctions and the base
contact, and nothing else. Its magnitude is bounded by `ambient`: at the
demo's 0.25 the deepest crease can lose at most a quarter of its shading, which
reads as depth rather than as dirt — and it is why `--rig` raises the ambient
share as well as turning AO on.

**Cost** (interleaved A/B, CUDA, `volren_bunny`, radius 8 world units):

| samples | CUDA, 1 bunny | CUDA, 9 bunnies | CPU serial, 1 bunny |
|---|---|---|---|
| 0 (off) | 5.51 ms | 16.19 ms | 22.55 ms |
| 1 | +7.5% | +10.1% | +1.2% |
| 3 | +15.4% | +21.5% | +4.1% |
| 5 (default) | +28.3% | +32.6% | +2.8% |
| 8 | +42.9% | +49.5% | +4.5% |
| 16 | +80.2% | +89.8% | +10.6% |

Linear in the tap count on the device, and **this is the honest number**: the
cone is one trilinear fetch (8 voxels) per tap per shaded isosurface hit, and on
this scene almost every ray has such a hit. The same CPU/GPU asymmetry as the
soft-shadow filter, and for the same reason — memory-bound versus issue-bound.
It is why `samples` is a knob and not a constant, and why the default is 5
rather than the 16 that would look best: at 5 the device pays a third of a
frame for it, at 16 it pays a whole one.

The sample count is a smoothness dial, not a correctness one: **1 sample is a
hard threshold at exactly one distance** and produces the strongest, sharpest
crease band (worst darkening 59 levels of 204 on the two-ball probe); 16
integrates the cone and softens it (worst 36). More samples is *less* extreme,
not more accurate.

## The light rig, and energy

Three things were wrong with the shading model as an *appearance* model, all of
them measurable rather than aesthetic.

**1. Ambient carried no information about the surface.** `hemisphere_ambient`
tints it by the sample's own normal — sky overhead, bounce underfoot — for one
dot product and three lerps, measured at **+3.7% on one bunny, +0.4% on
nine (CUDA) and +0.8% (CPU serial)**, i.e. free at the scale it matters. With
both colours white the mix is exactly `{1,1,1}` for every normal (`a₀ == a₁`
makes `a₀ + (a₁-a₀)·f` exact), which is why turning it on with the defaults is a
byte-identical no-op and not a rounding-sized change.

**2. Coloured key/fill already worked; nothing surfaced it.** Multiple lights
accumulate (the legacy overwrite bug was fixed in the port) and each carries its
own colour into both its diffuse and its specular term, so a warm key plus a
cool fill needs no new API — only a demo that shows it, which `--rig` now is.
The one thing worth knowing is that an empty `shadows.lights` means **every**
light casts, so a two-light rig silently pays two light passes; `--rig` sets
`shadows.lights = {0}` so only the key casts, which is also the right picture (a
fill exists to open up what the key left dark).

**3. The specular had no material term at all, and it clamps.** The legacy
expression adds the highlight at the light's FULL colour on top of a diffuse
term that already reaches the material colour, so a fully lit sample under a
white key reaches 1.85 before the clamp. Measured on the flagship bunny at its
shipped settings (`ambient` 0.25, one white key, gain 0.9), by the black-box
test that halving the gain must halve every channel:

| configuration | object pixels that clamp | channel clamps | worst overshoot |
|---|---|---|---|
| flagship (ambient 0.25, white key 1.0, gain 0.9) | **15.84%** | 32380 | 223/255 |
| ambient 0.45 | 35.13% | 67408 | 255/255 |
| gain 1.0 | 25.84% | 50651 | 255/255 |
| **specular 0.35**, everything else flagship | 9.28% | 16215 | 73/255 |
| **specular 0**, everything else flagship | **0.00%** | **0** | **0** |
| `--rig` (ambient 0.45, warm key 0.75 + cool fill, gain 1.0, specular 0.35) | 5.36% | 5484 | 39/255 |

The last two rows are the finding: **the specular term is the entire cause.**
With it removed the same scene at the same exposure does not clamp a single
channel — so the sixteen percent was not "too much light", it was a highlight
added with no material reflectance on top of a diffuse term that already reached
the material colour. That plateau is what makes the "before" bunny's haunch read
as a cut-out rather than as a surface.

`render_settings::specular` is the fix and it is one multiply; 1.0 is the legacy
expression exactly, and stays the default. It is a scene knob rather than a
per-surface one because the per-surface list has a fixed 6-field state encoding
and this does not need to break it.

Note also that ambient and gain make it *worse*, not better: both raise the
whole sum against a fixed ceiling, so "turn the lighting up" and "stop damping
the output" are exactly the two things a caller reaches for and exactly the two
that push more of the object into the plateau. That is why the gain became a
knob in the same change as the reflectance rather than on its own.

`shading_gain` becomes a knob for the adjacent reason: 0.9 is a legacy damping
applied to the **ambient term as well**, so at 0.9 the renderer cannot reproduce
its own material colour (a surface with `ambient` 1 and no lights renders at
`0.9 × base`). The default is unchanged.

### Energy: what attenuates what

Shadows scale DIFFUSE and SPECULAR; occlusion scales AMBIENT. Nothing is
attenuated twice, and that is checkable rather than asserted: shading is
`L = A·ao + D·vis`, affine in `(ao, vis)`, so

```
L(ao, vis) + L(1, 1) == L(ao, 1) + L(1, vis)      exactly
```

If either factor ever multiplied the other's term the product breaks the
identity by exactly the amount of the double attenuation.
`ShadowsAndOcclusionDoNotDoubleDarken` renders all four corners of a scene where
both effects are real (AO worth >8 levels, the shadow worth >20) and holds the
identity to **≤2 levels**, which is the four independent byte roundings and
nothing else.

The remaining energy hazard is the clamp, and it is deliberately left alone: it
is per channel, so a highlight that saturates only some channels desaturates
toward white — which is what a real specular highlight does — and any
alternative (a tone curve, a luminance-preserving clamp) changes every existing
image for no knob's default. `specular` and `shading_gain` are the two knobs
that let a caller stay out of it.

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
| `setShadowConfig(s)` / `shadowConfig()` | the whole `shadow_settings` | resolution, strength, bias — and every field the named accessors below do not cover |
| `setShadowMode(m)` / `shadowMode()` | `shadows.mode` | `hard` (one depth) vs `deep` (a transmittance profile) |
| `setDeepShadowSlices(n)` / `deepShadowSlices()` | `shadows.depth_slices` | clamped on the state read; inert in `hard` |
| `setSoftShadows(r, taps)` / `softShadowRadius()`, `softShadowTaps()` | `shadows.pcf_radius`, `.pcf_taps` | set as a **pair**: a tap count with no radius does nothing and a radius with one tap is not a filter |
| `setAmbientLevel(a)` / `ambientLevel()` | `render_settings::ambient` | **not** `setAmbient` — `GeometryNode` owns that name for the quad's VTK material |
| `setHemisphereConfig(h)` / `hemisphereConfig()` | `ambient_hemisphere` | neutral while both colours are white |
| `setOcclusionConfig(ao)` / `occlusionConfig()` | `ao_settings` | strength 0 skips the cone entirely |
| `setShadingGain(g)` / `shadingGain()` | `shading_gain` | not range-checked, like the state binding |
| `setSpecularLevel(s)` / `specularLevel()` | `specular` | **not** `setSpecular`, same collision as `ambient` |
| `setRenderConfig(rs)` / `renderConfig()` | the whole `render_settings` | what every accessor above is a read-modify-write of |
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

**`converged()` is not optional for a capture, and the demo proves it.** The
raycast runs on a worker, so the picture on screen at an arbitrary tick was
produced by whatever settings were live when *that* raycast started — for the
first ticks of a run, the pre-configuration ones. `volren_bunny --png` used to
write on the very first loop iteration, and the result was that `--rig`,
`--ao 1.0` and `--soft-shadows 6` each produced a PNG **byte-identical to the
default one**: every screenshot of this feature was a screenshot of the scene
before it. The demo now writes only once `converged()` is true (bounded by a
tick count so a capture cannot hang), and `--frames N` alongside `--png` is a
floor rather than a deadline. Nothing about the renderer changed; the lesson is
that "no new frame this tick" is ambiguous between *settled* and *still
working*, which is exactly what `converged()` exists to disambiguate.

**The demo panel is grouped by COST MODEL, not by feature.** `volren_bunny`'s
"Volume raycast" window is a fixed readout — backend, milliseconds, Mray/s,
raycast count, then the raster and the ray budget it resolves to — over four
collapsing groups: **Scene** (every control invalidates the shadow maps and pays
a rebuild frame), **Sampling** (priced in rays per frame), **Shadows** (a
light-view pass whose cost is camera-*independent*), **Lighting** (nearly free,
except the AO cone, which has its own dial). Knobs share a row only where they
are one decision — a filter's radius and its tap count, an output gain and the
specular it compensates for, a cone's reach and its sample count — which is also
what keeps all four groups inside a 720-tall viewport without a scrollbar. The
mode-specific pair sits side by side with each half disabled in the mode that
ignores it: in `deep`, `depth slices` is live and `min caster` is greyed, which
is the clearest statement the UI can make about what the representation
actually changes.

**A still camera makes the Mray/s readout a cold number, so the demo has
`--continuous`.** With nothing moving, the node raycasts twice and stops, and
`lastRenderSeconds()` then reports a raycast taken on a GPU that has barely
woken up. Measured at 1280×720: one bunny reads **26.1 ms still / 19.3 ms
warm**, nine read **67–82 still / 61.5 warm**. That is not merely imprecise — it
inverts an ordering: still-camera timings made nine bunnies *with* shadows look
faster (59.9 ms) than without (67.3 ms), which the warm numbers correctly
reverse (61.5 off → 63.5 hard → 64.0 deep → 82.2 deep+soft). `--continuous`
mirrors the panel's own "re-raycast every frame" checkbox and is off by default.

**Naming: why the two lighting knobs are not `setAmbient`/`setSpecular`.**
`GeometryNode` already owns both names, for the **quad's** VTK material — the
constructor sets ambient 1 / specular 0 precisely so the raycast image shows
unlit. Overloading them on the derived class would give one call site two
meanings, and the `double` vs `float` overload resolution that decides which is
not something a reader should have to work out. `ambientLevel()` and
`specularLevel()` belong to the raycaster's shading model; `setAmbient()` and
`setSpecular()` belong to the polygon the frame is pasted on.

**Teardown with a raycast in flight was a null dereference, now fixed.** The
worker's completion callback read the backend back through the node's own
`m_worker` unique_ptr. `~VolRenNode` calls `m_worker.reset()`, and `reset()`
stores nullptr into the pointer **before** running the deleter — i.e. before
`~worker()` joins the render thread. A frame still marching at that moment
therefore returned into the callback with `m_worker` already null. The callback
now holds a raw `worker*` captured at construction, which is valid for exactly
the same reason the reach-back was not: the callback runs inside
`worker::loop()`, which is what the join is waiting on, so the worker *object*
outlives every call even though the unique_ptr no longer names it. It only
surfaces when a raycast genuinely outlasts the destructor, so it hid behind
fast scenes: SIGSEGV 3/3 on `volren_bunny --bunnies 9 --volume-shadows
--deep-shadows --shell --soft-shadows 4 --ao 0.7 --rig --continuous` (220 ms a
frame), clean 5/5 after. `cvcgl_volren_node` now pins it with a deliberately
expensive node — 64³ volume, 4×4 rays per pixel, 768 steps, serial — ticked
once and dropped: 5/5 SIGSEGV without the fix, 3/3 clean with it, and the check
reports whether the frame really was in flight rather than assuming it.

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
flattened `shadow_view` plus HOST pointers to its f32 depth raster and (in deep
mode) its transmittance profile, and the host puts both through the **resident
block cache** — the same one the voxels use, keyed and pinned the same way — so
a steady-state frame moves no shadow bytes at all. The frames ride in the
parameter block (~160 bytes each, taking it to 2072 bytes of `cmem[0]`); a
host-resolved `light_map[]` means the kernel never scans the maps looking for a
light. `ptxas` for sm_75 measures **3232 bytes stack frame / 255 registers**,
against 3184 / 254 before shadows and unchanged by deep maps — no occupancy
change in either step.

Deep-shadow **capture** — the light pass's second output — is a separate
instantiation, `volren_raycast_kernel<1>`, so the ordinary render launches
literally different code rather than a predicated version of the same code (see
"Producing the profile"). The capture instantiation measures 3248 bytes / 24 B
spill stores; nothing else launches it.

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
on top of the spline cache and the hit buffer. The measured sm_75 frame is
**3232 bytes per thread, 255 registers, 12 bytes of spill**.

Doubling the cap would double that tail, but — contrary to what this section
used to say — that is *not* why the cap is 16. The kernel is issue-bound, not
occupancy-bound: removing the spline cache's 576-byte `deriv` tensor (frame
3232 → 2656) moves a nine-volume frame by 1.006x, and pushing occupancy the
other way is actively harmful (`-maxrregcount=64` spills 5284 bytes and runs
**4.9x slower**). Do not add `__launch_bounds__`. Raising the cap would be
nearly free; 16 stands because it covers the scenes this renderer is driven
with, with headroom.

A thread still carries ONE spline-gradient cache keyed on
*(volume, cell)* rather than the CPU's one-per-volume: it is pure memoization,
so the values are identical, and 16 caches would be 13 KB per thread.

**Parity of the lighting terms.** The soft-shadow tap loop, the occlusion cone
and the hemisphere tint are each a new arithmetic path in the kernel, and each
is transcribed from the same header the host uses. They measure
`worst_channel_diff = 0` — literally byte-identical images, not "inside the
budget" — on every configuration in `SoftShadowParity` (both payloads × 3 tap
counts), `AmbientOcclusionParity` (3 sample counts × 2 radii) and
`AmbientRigParity` (isosurface and shaded-TF shading sites). That is expected
rather than lucky: none of the three calls a fast-math-routed transcendental,
which is where the pre-existing residual differences live.

**Resident device block cache.** Voxel blocks — and, since deep shadow maps,
**shadow-map rasters** — stay on the device between renders and across
`raycaster` instances, so a camera-only re-render, or cvcGL's `VolRenNode`
rebuilding its raycaster's volume list every frame, launches with no
host-to-device traffic at all. The invalidation rule:

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

5. **Shadow rasters** (a hard map's depth raster; a deep map's depth raster and
   its transmittance profile) go through the identical machinery. The pin is a
   held `cvc::image` copy, which is a refcounted copy-on-write buffer, so rules
   1 and 2 carry over verbatim; the generation is the raycaster's shadow-map
   fingerprint, so an entry is invalidated by content as well as by address. A
   rebuild always allocates fresh images, so it lands on a different key anyway
   — the generation is belt and braces at zero cost. Rasters and voxels share
   one budget under one LRU.

`raycast_cuda_cache_bytes()` reports what is resident,
`raycast_cuda_cache_upload_bytes()` the bytes ever pushed H2D (the counter a
"no upload on a camera move" assertion reads), and `raycast_cuda_clear_cache()`
frees everything not in flight.

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
| `ambient_hemisphere.enabled` | int 0/1 | sky/ground tint on the ambient constant |
| `ambient_hemisphere.sky` / `.ground` | "r,g,b" floats | both `1,1,1` is a byte-identical no-op |
| `ambient_hemisphere.up` | "x,y,z" | normalized on use; a degenerate vector is a flat 50/50 blend |
| `ao.strength` | double | read RAW; `render()` throws outside [0,1] (the `shadows.strength` rule -- above 1 would drive ambient negative) |
| `ao.radius` | double | cone length in the volume's LOCAL units; `<= 0` is off, so there is no range to violate |
| `ao.samples` | int | taps along the cone; CLAMPED on read (the `resolution` convention) |
| `shading_gain` | double | the legacy 0.9 output damping; NOT range-checked, for the same reason `ambient` is not |
| `specular` | double | scene-level specular reflectance; 1.0 is the legacy expression |
| `threads` | int | |
| `lights` | flat CSV | `r,g,b,dx,dy,dz` per light |
| `cut_planes` | flat CSV | `px,py,pz,nx,ny,nz` per plane |
| `shadows.enabled` | int 0/1 | master switch |
| `shadows.lights` | flat CSV ints | casting light indices; `""` = every light casts. A separate index list rather than extra fields on `lights`, because 42 values would be ambiguous between 7 lights of 6 fields and 6 of 7 |
| `shadows.resolution` | int | light-view raster edge; CLAMPED on read (the `threads` convention) |
| `shadows.strength` | double | 0 = no-op, 1 = full |
| `shadows.bias_scale` | double | constant bias in latch quanta |
| `shadows.slope_scale` | double | slope bias in `texel_world * tan(theta)` |
| `shadows.min_occluder_opacity` | double | isosurface opacity floor to cast; ignored in deep mode |
| `shadows.mode` | int 0/1 | 0 hard, 1 deep. An ENUM, so an out-of-domain value is malformed state and leaves the object alone (the `shadows.lights` discipline) rather than being clamped into a mode nobody asked for |
| `shadows.depth_slices` | int | deep profile knots; CLAMPED on read (the `resolution` convention -- it bounds the map's memory, so every value has a defensible nearest meaning) |
| `shadows.pcf_radius` | double | soft-shadow filter half-width in light-map texels; CLAMPED to `[0, max_pcf_radius]` on read, so a negative value round-trips into "unfiltered" -- which is what `render()` does with it too |
| `shadows.pcf_taps` | int | taps per filter EDGE; CLAMPED to `[min_pcf_taps, max_pcf_taps]`. An EVEN value rounds DOWN to the odd grid below it: a grid with no center tap would displace the shadow by half a tap spacing |
| `volumes.count` | int | number of bound volume-settings blocks |
| `volumes.<n>.shaded`, `.unshaded`, `.tf_auto_domain` | int 0/1 | |
| `volumes.<n>.distance_field` | int 0/1 | the scalars are a signed distance, positive outside. Read TOLERANTLY, unlike its siblings: a per-volume key exists only once something seeded it, so a tree written by an older build simply has no node here, and rejecting the whole snapshot over one missing field would discard the fifteen that parsed |
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
- Blinn-Phong diffuse+specular with the 0.9 output gain — now
  `render_settings::shading_gain`, defaulting to that 0.9, and the per-channel
  clamp above it is unchanged.

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
13. Deep shadow maps (new capability — see "Deep shadow maps"). Off by default
    twice over: shadows are off, and when they are on the mode is `hard`, which
    is the pre-existing single-scalar map. An **opaque** occluder renders
    byte-identically in either mode by construction, so switching to `deep` on
    the renderer's flagship content changes nothing at all.
14. Soft shadows (new capability — see "Soft shadows"). Off by default from
    either knob: `pcf_radius` 0 and `pcf_taps` 1 both take the historical
    single-tap comparison, and both are pinned as byte-identical.
15. Ambient occlusion (new capability — see "Ambient occlusion"). Off by default
    four times over — `ao.strength` 0, `ao.radius` 0, `ambient` 0, and
    `volume_settings::distance_field` false — each of which independently
    reproduces the pre-AO image byte for byte.
16. The ambient constant can be a sky/ground HEMISPHERE
    (`ambient_hemisphere`), which is off by default and additionally a no-op
    when its two colours match. The legacy flat constant is the `sky == ground`
    case exactly.
17. The specular lobe has a REFLECTANCE (`render_settings::specular`). The
    legacy expression has none — the highlight is added at the light's full
    colour — which puts 15.84% of the flagship bunny's pixels into the clamp at
    its shipped settings. 1.0 is that expression exactly, and stays the default.

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

- **Contact-hardening soft shadows (PCSS).** `pcf_radius` is a constant number
  of texels everywhere, so a shadow is as soft at the point of contact as it is
  metres away — the one thing that still reads as synthetic once the hard edge
  is gone. The standard fix is a blocker-search pass whose average blocker depth
  drives a per-sample radius, which is one more filtered lookup per shaded
  contribution (measured cost of the existing 3×3: +6.7% to +10.6%, so a search
  plus a filter is roughly double). Worth doing only after the deep map's host
  round trip below, since both touch the same lookup.
- **Ambient occlusion for transfer-function media.** Deliberately out of scope
  now: a shaded TF sample fires once per cell entered, so the same 5-tap cone
  would cost two orders of magnitude more than it does on an isosurface hit, and
  "distance to the surface" is undefined for a medium anyway. The technique that
  fits a medium is a low-resolution precomputed occlusion volume, which is a
  different data structure with its own invalidation story — not a knob on this
  one.
- **Contact ambient occlusion BETWEEN volumes.** The cone measures the hit's own
  volume only (the reasoning is in the AO section), so a bunny resting on a
  plate gets no darkening from the plate. The cheap version — take the min of
  every distance-field volume's `f - isovalue` at each tap, after mapping the
  tap through each volume's inverse `model_transform` — multiplies the tap cost
  by the volume count and is only a distance field again when the volumes are
  scaled alike, which nothing enforces. The version that is actually right is a
  fused scene SDF rasterized once per scene change, i.e. a new resident
  structure with the resident voxel cache's invalidation story. Neither is
  worth it before the empty-cell skipping below, which is a larger win on the
  same frames.
- **A first-order distance estimate for non-SDF fields.** `(f - isovalue) / |∇f|`
  is the distance to the level set to first order, which would let the same cone
  run on an arbitrary scalar field. It needs a gradient at each tap, and the
  gradient here is a 4³ spline (64 fetches against the cone's 8), so it is ~8×
  the cost of the SDF path — plausible only if the tap count drops with it.
  Left out rather than shipped slow.

- **Empty-cell skipping for the isosurface DDA.** After the occlusion cutoff
  above, the DDA walk is still ~63% of a nine-volume device frame, and almost
  all of it is cells the corner min/max test rejects: a ray crossing a 64³
  bunny walks ~100 cells and brackets an isovalue in ~2. A per-volume min/max
  brick table (an 8³ brick grid over a 64³ volume is 512 entries) would let the
  walk skip whole bricks, and because it can only ever reject cells that
  bracket nothing it is byte-identical by construction. The work is a host-side
  build, a device upload with the same invalidation rule as the resident voxel
  cache, and a mirror in `raycaster.cpp`. This is the single largest remaining
  win and should be measured before anything else on this list.

- **Per-cell marching cubes in float — measured at 1.22x, and rejected.**
  `extract_contour` / `intersect_triangle` / `in_triangle` are ~1400 FP64 ops,
  10.7% of the kernel's static instruction count but ~98% of its FP issue slots
  at sm_75's 1/32 double rate. Running the solve in float, on a frame rebased
  so both the cell corner and the ray origin sit next to the cell, takes a
  nine-volume frame from 17.0 ms to 13.9 ms and is **byte-identical on the
  bunny scene at 1/2/4/9 volumes** — but it breaks
  `VolrenCudaTest.IsosurfaceSphereParity` and `SelfShadowParity`, whose
  orthographic camera is exactly axis-aligned with the grid.

  The mechanism is not precision loss, and more precision does not fix it: the
  same rebase carried out entirely in *double* fails identically (one ray loses
  its front-face hit and latches the back surface a full chord later). Rebasing
  makes the solve *more* accurate, and that is the problem — `in_triangle`'s
  legacy `u1 == 0.0` and `denom == 0.0` degeneracy guards rely on absolute
  coordinates rounding a near-degenerate triangle to exactly zero. Shrink the
  operands and the degeneracy becomes representable, the guard stops firing,
  and a different branch is taken. This is the "bit-faithful to libiso"
  contract in `detail/cell_intersect.h` doing its job. Any future attempt has
  to replace those guards with scale-aware ones *first*, as a separate change
  justified on its own terms, and re-baseline the isosurface images.

  (For the record, two cheaper variants were also measured and dropped:
  dropping the mathematically-redundant `normalized()` on the triangle normal —
  the scale cancels in `fz/fm` — is worth 6% and passes all 70 tests, but it
  perturbs `t` at the ulp level for no margin worth having; and rebasing to the
  cell corner *without* also rebasing the ray keeps a ~500-unit lever arm and
  moves 0.017% of pixels with silhouette flips up to 165 levels.)

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
- **The deep map's host round trip.** A CUDA rebuild writes the profile into
  device memory, copies it out to the host `cvc::image`, and copies it straight
  back in for the main pass — 2 × 17.8 MB at the defaults, and the whole of the
  deep mode's rebuild premium (measured above; the steady state is already
  free). Keeping the light pass's output device-resident and handing the main
  pass the same allocation would remove both copies, but it breaks the property
  that a shadow map is *host data both backends share* — today a CPU-built map
  and a CUDA-built one are interchangeable, which is what
  `DeepShadowProfileProducedOnEitherBackendAgrees` checks and what lets the
  light pass pick its own backend. Worth doing as a device-side *fast path* that
  falls back to the host image, not as a replacement for it.
- **Per-texel knot grids for deep maps.** The grid is currently uniform over the
  whole scene's light-depth extent and shared by every texel, so a ray that
  crosses one thin shell spends most of its knots on empty space. Fitting
  `[z_first, z_last]` per texel costs two more floats per texel and one extra
  load in the lookup, and would buy most of what raising `depth_slices` buys.
  The reason it is not here: the separated-occluder regime — the common one —
  already reproduces a 256-slice map with **2** slices, so the win is confined
  to thick self-shadowing media, and it should be measured against simply
  raising `depth_slices` on those before the payload grows a second per-texel
  affine.
- **Exact steps for partially-opaque isosurfaces.** The terminal channel is
  exact but fires only on saturation, so a 0.6-opacity surface is a
  `slice_dz`-wide ramp. A second exact-depth slot (the first isosurface hit and
  its alpha) would cover it for one surface per ray; more than one needs the
  variable-length node list this design deliberately rejected for lookup cost.
- ~~**PCF for shadows.**~~ SHIPPED. `shadow_visibility` /
  `shadow_visibility_deep` box-average the test over a `pcf_taps`-per-edge grid
  spread across `pcf_radius` texels, mirrored into both backends. The deep
  lookup is 2 loads per tap, so a 3×3 PCF there is 18. What remains open is a
  *rotated* or Poisson tap pattern: the regular grid banded on some scenes,
  which is why the tap count is exposed rather than fixed.
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
