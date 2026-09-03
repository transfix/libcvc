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
  bool two_sided_lighting = false;       // legacy light_both
  float ambient = 0.0f;                  // legacy zeroed ambient; now a real knob
  int steps = defaults::steps;           // samples along the scene-bbox diagonal
  float opacity_cutoff = defaults::opacity_cutoff;        // early ray termination (0.95)
  float depth_alpha_threshold = defaults::depth_alpha_threshold; // where the depth map latches
  unsigned threads = 0;                  // 0 => pool default; 1 => serial
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

  camera &view();
  render_settings &settings();

  void set_thread_pool(cvc::thread_pool *pool);  // borrowed; default: a private pool
                                                 // (cvc::thread_pool allows one in-flight
                                                 // parallel_for, so the shared computePool
                                                 // is unsafe to borrow blindly)

  frame render();          // interruption-point + threadProgress aware
  cvc::bounding_box scene_bounds() const;        // union of volume boxes ("metavolume")
};
```

Algorithm (faithful to volren where it defines the look, fixed where it was
broken — every deviation listed in "Fidelity" below):

1. One ray per pixel; slab-method AABB entry/exit against the union of the
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

The gradient is the legacy quadratic-B-spline (de Boor) gradient over the
4×4×4 neighborhood, with the per-ray cache carried in a stack
`detail::spline_gradient_cache` (the state that used to force `vrCopyEnv`).

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
| `two_sided_lighting` | int 0/1 | |
| `ambient` | double | |
| `threads` | int | |
| `lights` | flat CSV | `r,g,b,dx,dy,dz` per light |
| `cut_planes` | flat CSV | `px,py,pz,nx,ny,nz` per plane |
| `volumes.count` | int | number of bound volume-settings blocks |
| `volumes.<n>.shaded`, `.unshaded`, `.tf_auto_domain` | int 0/1 | |
| `volumes.<n>.matrix` | 16 CSV doubles | row-major model matrix (cvcGL `matrix` encoding) |
| `volumes.<n>.transfer_function.color` | flat CSV | `value,r,g,b` per point (VolumeNode encoding) |
| `volumes.<n>.transfer_function.opacity` | flat CSV | `value,a` per point (VolumeNode encoding) |
| `volumes.<n>.window` | "min,max" or "" | density window |
| `volumes.<n>.gradient_ramp` | "r0,r1,r2" or "" | |
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

- CUDA path: `src/cvc/volren/raycast.cu` twin following the
  `voxels_kernels.cu` precedent (per-source `--use_fast_math`, `CVC_USING_CUDA`
  guards, CPU/GPU parity tests on shared fixtures).
- cvcGL drop-in node: a `GraphicsNode` that renders the raycast frame as a
  textured quad with `setTexture(frame.color)` + a depth texture bound via
  `setShaderTexture` and a `//VTK::Depth::Impl` fragment replacement using
  `depth_to_window_z` — the integration recipe is in the port notes; the
  camera interop (`camera::from_pose` ⇄ `CameraController::getPose`) ships
  now.
- Per-sample (opacity-corrected) integration mode alongside the per-cell one.
