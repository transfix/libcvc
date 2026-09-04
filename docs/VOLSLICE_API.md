# cvc::volslice — the VolumeRover2 slice renderer in libcvc

Port of volumerover2's `OpenGLVolumeRendering` "VolumeLibrary" (Thane/Bajaj,
UT Austin 2002–2003, arand patches through 2012): the classic back-to-front
view-aligned-slice volume compositor, now the **third** volume-rendering path
in the cvcGL scene graph beside `VolumeNode` (VTK's own GPU raycaster) and
`VolRenNode` (the `cvc::volren` software raycaster).

## Design rules

The rules the `cvc::volren` port established, applied unchanged:

- **No globals, no singletons.** Everything hangs off an injected `cvc::app`.
- **State tree is the source of truth** for every tunable, under
  `<node>.volslice.*`.
- **The legacy sampling model is kept exactly** — the plane sweep, the
  256-case clip table, the arand 2011 slice-count formula — and deviations
  are opt-in and documented here.
- **cvc objects at the boundaries**: `cvc::volume` in, VTK scene out.

## Architecture

| layer | what it is | GL? |
|---|---|---|
| `inc/cvc/volslice/slicer.h` | the slicing engine: local→clip matrix + box in, back-to-front triangle-fan slices with 3D texcoords out | no — pure geometry, unit-tested |
| `inc/cvc/volslice/settings.h` | `render_settings`: the full tunable surface | no |
| `inc/cvc/volslice/state_settings.h` | the state-tree binding (volren's pattern: seed-suppressed ctor, all-or-nothing re-read, one apply callback) | no |
| `inc/cvc/gl/VolSliceNode.h` | the scene node: per-tick slice recompute, R8 3D texture + 256×1 LUT texture, fragment-shader dependent lookup | yes |

The legacy `Renderer`/impl zoo (paletted 3D/2D textures, SGI color table,
NV/ARB fragment programs, Cg) collapses to one GLSL path: on any driver since
~2010 only `FragmentProgramARBImpl` ever ran, and its whole fragment program
was "sample the 3D texture, look the value up in a 256-entry table" — which
is `kFragmentSampleImpl` in `VolSliceNode.cpp`.

### What the legacy code read from GL state is now an explicit input

`RendererBase::getViewPlane()` read the modelview/projection matrices off the
GL stack at render time. `compute_slices()` takes the local→clip matrix as an
argument; the node builds it from the live `vtkCamera` composite matrix and
the node's composed scene-graph world transform, so the volume follows its
node like any other scene citizen.

### Space conventions

The legacy sliced an origin-centered, aspect-normalized unit cube and left
placement to the caller's translate/scale. A real volume's local bounding box
is exactly that cube under a uniform scale plus a translation, and both
preserve plane parallelism and relative spacing — so the engine runs the
legacy algorithm verbatim in ratio space and maps out. Slice counts match
legacy for any box shape.

## The state-key map (`<node>.volslice.*`)

| key | type | legacy origin |
|---|---|---|
| `quality` | double [0,1], raw | the VolumeRover2 quality slider; N = 2·(10 + max_planes·q³) |
| `max_planes` | int, clamped [1,10000] | `setMaxPlanes` (default 1000); also caps planes/frame at 10× |
| `near_plane` | double [0,1], raw | `setNearPlane`: fraction of the diagonal peeled from the viewer side |
| `interpolation` | int 0=linear 1=nearest, rejected outside | legacy hardcoded `GL_LINEAR` |
| `opacity_correction` | int 0/1 | **new, opt-in deviation** (below) |
| `tf_auto_domain` | int 0/1 | — |
| `window` | `""` or `"min,max"` | the UChar-coercion range VolumeViewer applied before upload |
| `transfer_function.color` | `"value,r,g,b"×N | the shared VolumeNode/volren encoding |
| `transfer_function.opacity` | `"value,a"×N | one TF editor drives all three renderers |

`quality`/`near_plane` are stored raw (the renderer clamps; out-of-range
round-trips rather than being silently rewritten), `max_planes` is clamped on
read (an implementation resource), enums reject unknown values and keep the
last good settings — the volren conventions, key for key.

## Order-dependent blending: the scene-wide effect

Slice compositing is sequential by construction. **VTK 9's default
translucent pass is order-independent** (`vtkRenderer::UseOIT`, weighted
accumulation with `rgb=(ONE,ONE)` — verified by reading the GL blend state
during the pass), under which a slice stack *averages* into an X-ray look
instead of compositing. `VolSliceNode::addToRenderer()` therefore switches
its renderer to sequential translucency (`UseOITOff`), which VTK renders with
exactly the legacy blend state (`SRC_ALPHA/ONE_MINUS_SRC_ALPHA`, depth-write
off, depth-test on). Other translucent actors then depth-sort as props,
pre-VTK9 style. The toggle is not restored on node removal.

Two rendering facts found the hard way, kept as regression tests:

- **`ForceTranslucentOn()` is mandatory**: with no `vtkTexture` and opacity
  1.0, VTK classifies the actor opaque — depth writes and no blending.
- **`DISABLE_SHIFT_SCALE` on the mapper is mandatory**: the vertex shader
  derives the volume texcoord from `vertexMC`, and VTK's AUTO shift-scale
  silently re-centers VBO coordinates for boxes offset from the origin
  (compensating in `MCDCMatrix`). The demo bunny — box z∈[0,100] — rendered
  zero pixels while the origin-centered unit test ball passed.

## Fidelity

**Preserved exactly**
- The 256-case plane/cube clip table and its fan ordering (`LookupTables.h`
  transcribed verbatim), the back-to-front sweep, the aspect-ratio
  normalization.
- Slice count `N = 2·(10 + max_planes·quality³)`, the 10·max_planes cap, the
  near-plane fraction (arand 6-14-2011 semantics).
- The UChar density coercion (R8 texture; also the pragmatic choice — WebGL2
  cannot LINEAR-filter float textures without an extension).
- The 256-entry RGBA LUT with texel-center indexing (byte i ↔ entry i, the
  `coldentbl` layout), dependent lookup per fragment.
- Unshaded rendering (the only path that ran on non-Cg builds).

**Fixed**
- The copyable facade's shallow-copied raw impl pointers (a latent
  double-delete) — gone with the impl zoo.
- Per-frame ARB program regeneration (a leak) — one GLSL program, compiled
  once by VTK.
- The pow2 padding + `setTextureSubCube` crop dance — NPOT textures are
  core; the sub-cube survives in `slice_params` for callers that still crop.
- GL-state-derived view plane — explicit matrix input.

**Dropped**
- The renderer implementation zoo and its `glewIsSupported` fallback chains;
  `VolumeRendererFactory` (already dead in volumerover2).
- The Cg-only shaded path (precomputed 127-biased normals texture +
  headlight) — shading is future work, planned as on-the-fly gradients.
- The RGBA direct-color path (`SimpleRGBAImpl`) — colormapped only for now.
- `blend_mode`: the legacy shipped exactly one blend function; additive
  existed only as commented-out dead code, and `dst + src·a` is not
  expressible in-shader under the sequential pass's blend state. Not a
  silent knob — it does not exist.
- Out-of-core hooks (`setDataSubVolume`/`setHintDimensions`) — dead in the
  legacy tree too.

**Deviation (opt-in): `opacity_correction`**
The legacy renderer's apparent density changed with the quality slider (more
slices × the same per-slice alpha). With `opacity_correction` on, each
slice's alpha is rescaled for the actual plane spacing
(`α' = 1−(1−α)^(spacing/reference)`, reference = the default-quality spacing
for that box), so quality changes sharpness instead of density. Default OFF —
the faithful behavior. Measured limit: correction cannot be exact through an
8-bit framebuffer (at q=0.9 the corrected per-slice alpha here is ~0.007, and
~1500 sequential blend roundings lose brightness); the legacy had the same
characteristic, one reason its TFs used chunky alphas.

## The node in one page

```cpp
auto node = sg.getGraphicsRoot()->addGraphicsChild<cvc::gl::VolSliceNode>("vol");
node->setVolume(density_volume);          // one volume per node (VolumeNode's model)
cvc::volslice::render_settings s;
s.window_min = -6; s.window_max = 6;      // the value window -> byte texture
s.tf.add({-6.0, 1.f, .55f, .18f, .92f}); // TF over the RAW value domain
s.tf.add({ 1.0, 0.f, 0.f, 0.f, 0.f});
node->setConfig(s);                       // or write the state keys directly
// per frame, before SceneRenderer::render():
node->tick();                             // recomputes slices on camera/transform/settings change
node->planesRendered();                   // the legacy getNumberOfPlanesRendered()
```

One volume per node — `VolumeNode`'s model, deliberately not `VolRenNode`'s
embedded multi-volume list: the slice renderer is a true scene citizen and
the scene graph composes multiples (VTK depth-sorts the actors).

## Future work

- **Shaded path**: gradient lighting in the fragment shader (on-the-fly
  central differences — modern GPUs make the legacy's precomputed normals
  texture unnecessary), lit by the scene's `lights` settings.
- **RGBA volumes** (`SimpleRGBAImpl`'s job): an RGBA8 3D texture path,
  bypassing the LUT.
- **Coexistence with OIT**: a custom `vtkVolumeMapper` rendering in the
  volumetric pass would restore order-independent translucency for the rest
  of the scene while keeping sequential compositing for the slices.
- **wasm**: nothing here is desktop-only — 3D textures are WebGL2-core and
  the demo builds in the wasm gallery.
