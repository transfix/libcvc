# External Ocean Water & Volumetric Terrain — Feasibility Companion

**Status:** feasibility notes, revision 1 — companion to `WATER-RENDERING-ROADMAP.md`, `LSYSTEM-LABORATORY-ROADMAP.md`, and `VISIBILITY-AND-LOD-ROADMAP.md`. Not authoritative over them; where this note and a roadmap disagree, the roadmap wins and this note is the thing to correct.
**Scope:** two questions raised together — (A) can we integrate the *ABYSSAL* WebGL FFT-ocean (`github.com/Token-Gremlin/natural-disasters`) into cvcGL, "via WASM"? (B) should terrain and onshore water become tetrahedral meshes, can an L-system build terrain tets, and can terrain be deformable under the new LOD?
**Provenance markers:** **[R]** cited from an in-tree roadmap with section/line. **[C]** read directly from cvcGL/libcvc source with file:line. **[X]** read from the external ABYSSAL source (via summarizer — *not* byte-exact; verify before porting). **[P]** projection/derivation, not an independent cvcGL measurement. **[E]** estimate.
**Method:** assembled from two adversarially-verified subagent sweeps plus direct source reads. Known evidence gaps are listed in §7; read them before treating any single number as settled.

---

## 0. Direct answers

1. **ABYSSAL water via WASM — yes, but "via WASM" is the wrong axis.** cvcGL *already is* a single-threaded WASM/WebGL2 target that deploys to `transfix.github.io` **[R]**. WASM neither blocks nor enables this. The work is re-hosting ABYSSAL's GLSL + multi-pass GPU pipeline onto cvcGL's VTK/Emscripten stack, and the decision is **where** it belongs.
2. **Where it belongs: the open sea and the disaster field — not inland water.** ABYSSAL is a fetch-limited FFT ocean; the water roadmap already rejects that class of technique for ponds/lakes/streams **[R]**, while the *sea* is a crude 4-wave slab that is explicitly out of the inland-water scope **[R]** — exactly the thing an FFT ocean upgrades.
3. **Tets for terrain / onshore water — no.** Terrain is a load-bearing 2.5-D analytic heightfield; onshore water was already analyzed and parked as the opt-in, native-only, no-LOD **W5** tier. Tets *fight* the LOD system.
4. **L-system → tet meshes — yes mechanically; the generation pieces already exist in-tree** — but it earns its keep only for genuinely volumetric features (caves/arches/overhangs), and even there the right *render* output is usually a triangle isosurface, not tets. The tet/unstructured **renderer does not exist yet**.
5. **Deformable terrain × LOD — yes for cheap heightfield deformation, which is essentially already the architecture; no for volumetric/tet deformation.** Dig-anywhere volumetric editing wants SDF/voxel + dual-contouring + clipmap LOD, not tets, and that runtime path does not exist today.

---

# Part A — ABYSSAL ocean water in cvcGL

## A.1 What ABYSSAL is

A browser ocean/disaster simulator: Three.js `^0.169.0` + Vite, **WebGL2 / GLSL ES 3.0**, zero external assets, all procedural **[X]**. License **MIT** (`Copyright (c) 2026 Davi (Token-Gremlin)`) **[X]**.

- **Water = multi-cascade FFT ocean.** 3 cascades (swell/wind/ripple), N=256², JONSWAP/TMA spectrum with Pierson–Moskowitz saturation and Donelan–Banner `cos^2s` directional spreading, wind/fetch-driven (`α = 0.076·(u²/Fg)^0.22`), cascade tiles **[4099, 389, 41.3] m** **[X]**. The IFFT is a **fragment-shader butterfly ping-pong** (no compute shaders): per frame `1 spectrum + 16 butterfly (2 dirs × log₂256=8) + 1 assemble + 1 copy = 19 passes/cascade × 3 = 57 full-screen GPU passes/frame` **[X][P]**.
- **Disasters = analytic vertical event-height, *not* a volume raymarch.** `vortexField` (Rankine), `solitonHeight` (sech²), `rogueGroup` (3-mode Gerstner), `hurricaneField`, unified in `oceanEventHeight()` and added *separably* to the FFT displacement **[X]**.
- **Shading:** GGX + water Fresnel + SSS + Jacobian foam. **Hard dependency** on an external sky/atmosphere system: equirect `uEnvMap` + `uTransmittanceLUT` + sky-view/aerial LUTs, plus startup-baked procedural textures (foam 2048², ripple 1024², weather 1024², cloud 128³+32³) **[X]**.

## A.2 Where it fits, and where it must not go

| Target | Verdict | Reason |
|---|---|---|
| **Open sea (near+far field)** | **Strong fit** | Current sea is a `72×72×20` camera-following `VolumeNode` slab (−40→+8 m) with *"four crested travelling waves"* + a far-field flat mesh at z=0 **[R, Lab §4.6 L465-468]**. ABYSSAL's km-scale fetch-limited spectrum is physically appropriate here. |
| **Disaster events (tsunami / rogue / hurricane)** | **Strongest fit** | Analytic vertical displacement, not a second volume pass, so it dodges the *~30 fps two-volume ceiling* **[R]**. High scenario value for disaster/C-UAS work. |
| **Inland ponds / lakes / streams** | **No fit** | The water roadmap already rejects FFT ocean: *"Phillips is a fetch-limited wind-sea spectrum; a pond has zero fetch and a stream is advective, not dispersive."* **[R, WATER §12 L1265]**. A pond has ~zero fetch; the spectrum degenerates. Use the W1–W4 surface+column path. |

The sea is explicitly outside the inland-water design: *"the sea (already volumetric and staying that way)"* **[R, WATER §2 L146/L155]**.

## A.3 WASM feasibility — the technique is already WASM-shaped

The FFT is already a fragment-shader ping-pong, so nothing about it needs compute shaders. Gates, in priority order:

1. **⚠️ cvcGL multi-pass hosting — the linchpin.** cvcGL already subclasses `vtkRenderPass` (`StridedShadowBaker : vtkShadowMapBakerPass`, `SceneGraph.cpp:916`), runs a `vtkRenderPassCollection` chain, does off-screen render-to-texture (shadow bake), and uses `vtkOpenGLFramebufferObject` (`ImGuiOverlay.cpp:34`) **[C]** — so the FBO/render-to-texture primitives exist. But it has **never driven a tight 57-pass full-screen ping-pong loop** (no `vtkOpenGLQuadHelper` usage found) **[C]**. VTK's pass-chain abstraction is not built for that pattern; drive it directly via `vtkTextureObject` + `vtkOpenGLFramebufferObject` + `vtkOpenGLQuadHelper`. **Prove this first.**
2. **Frame budget.** Lab sits at *~21 ms → ~47 fps* against a ≥45 fps target with only ~2 fps headroom; R12 flags 45 fps as *"the number I am least sure of"* **[R]**. 57 full-screen passes is a large addition. Levers: N→128 (cost ∝ N²), run the IFFT every Nth frame, keep far-field coarse. *Mitigant:* `lsystem_forest` is **CPU-bound with ~32 ms idle GPU/frame** **[R, WATER §1.1]**, and FFT cost lands on that idle GPU.
3. **Float render targets.** Working RTs RGBA32F, outputs RGBA16F **[X]** — requires `EXT_color_buffer_float` advertised on the deploy targets. One-line spike to confirm.
4. **Heap is *not* the worry.** FFT float textures total **~25–37 MiB** and live in **GPU texture memory, not the wasm linear heap** (2 GiB ceiling, ~110 MB resident, ~18× margin) **[X][R]**. Watch instead ABYSSAL's startup-baked 128³/32³ 3-D textures if its sky is pulled in **[X]**.

## A.4 Porting mechanics — re-host, not drop-in

You cannot drop in Three.js (C++/VTK/Emscripten engine). Each Three.js piece has a VTK analogue: `WebGLRenderTarget`/MRT → `vtkOpenGLFramebufferObject` with multiple color attachments; ping-pong → two swapped `vtkTextureObject`s; DataTexture uploads (Gaussian noise + 8×256 butterfly LUT) → one-time float `vtkTextureObject`s; the pass schedule → the driver loop (`world::pump`/`deriver::step`). ~85–90% of the GLSL ports **[X]**, but through **shader-replacement anchors**, where two documented traps bite:

- **`CVC_FS_NORMAL` (mandatory).** At `//VTK::Normal::Impl` the writable local is `normalVCVSOutput` on desktop but `normalizedNormalVCVSOutput` under Emscripten; the wrong name is an ESSL error that fails the *whole* program and hides the actor **[R, Lab §8.11 L1929]**. ABYSSAL sums cascade derivatives `d0+d1+d2` for normals **[X]** — squarely in the hot path.
- **`ReplaceShaderTCoord` early-returns with no bound texture** → the projected-grid vertex-displacement shader must inject its own `//VTK::PositionVC::Dec` or bind a 1×1 dummy; displacement itself goes at `//VTK::PositionVC::Impl` via `GeometryNode::addVertexShaderReplacement` (`GeometryNode.h:94`, already used in `lsystem_forest.cpp:159,362`) **[C][R]**. Ship an `cvcgl_ocean_shader` compile+link smoke test on **both** backends, mirroring `cvcgl_sway_shader`.

**Largest non-shader item:** ABYSSAL's ocean fragment shader reflects an equirect env map + atmosphere LUTs. That plumbing must exist in cvcGL or be stubbed, or the water has nothing to reflect **[X]**.

## A.5 Licensing

MIT → LGPL-2.1-only is clean: the combined work ships LGPL-2.1, but the ported GLSL/logic **must retain the MIT notice** (source headers + a `NOTICE`/third-party file). Same third-party-notice discipline libcvc already carries; do not strip or relicense on asserted authority.

## A.6 Phased plan (spike-first) and the trial demo

0. **Spike (throwaway, native):** one RGBA32F ping-pong FBO + one butterfly pass in a cvcGL demo. Retires the two hardest unknowns — can cvcGL drive full-screen ping-pong at all, and does the float RT render.
1. **OceanFFT re-host:** the 3-cascade N=256 pipeline as FBO passes; validate height/normal output offline, no shading.
2. **OceanNode surface:** projected-grid mesh + displacement/normal/foam as shader replacements (obey both traps), wired to cvcGL env/atmosphere (stub the sky if absent) — replaces the §4.6 slab.
3. **Disaster field:** `oceanEventHeight()` as separable analytic vertex displacement — high value, low cost.
4. **WASM parity + budget gate:** build via `_wasm_demos` (`src/cvcGL/examples/CMakeLists.txt:41`) + `build-wasm-demo.sh` on the on-disk emsdk (`wasm-deps/emsdk`); measure against the ~21 ms line; only then append to `deploy-pages.yml` `DEMOS:` (line 38) for post-merge deploy on catx-03 **[R]**.

**Trial demo recommendation:** stand this up as a **new native desktop demo** (e.g. `lsystem_coast`), *not* by editing `lsystem_forest`, which is the untouched performance control **[R, Lab §1.3]**. See §A.7 prognosis.

## A.7 Prognosis for a trial demo — favorable, with three framings

- **Do it native first.** Desktop cvcGL has full VTK, float FBOs and threads; the FFT ping-pong is far easier to stand up and debug there. WASM parity is a *later* gate, not the first test.
- **New demo, not a fork of the control.** `lsystem_forest` is the designated CPU-bound performance control **[R]**; clone its island/sky scaffolding into a new demo that swaps the sea. An island demo is the natural host — it already *has* a sea to replace.
- **Budget is favorable for a demo.** The control is CPU-bound with ~32 ms idle GPU **[R]**; FFT fill lands on the idle resource. Scope down for the trial: N=128, fewer cascades, sky-gradient reflection stub, no disaster field yet.
- **Set the visual expectation:** until the env-map/atmosphere reflection is wired, the surface will look flatter than ABYSSAL's screenshots — the geometry/foam will be right before the shading is.
- **What a green demo proves:** (1) cvcGL can host the multi-pass IFFT (the linchpin), (2) float RTs behave, (3) the frame cost with the FFT is measured, not projected. That is exactly the de-risking Phases 0–2 exist for, at demo (throwaway) risk.

## A.8 Phase-0 result — PROVEN, and the trial scaffold

**The linchpin is verified on real hardware, not projected.** A standalone probe and the in-tree `OceanFFT` class both ran an 8-pass RGBA32F full-screen ping-pong and read back **bit-exact** on this dev box:

```
OpenGL renderer: Intel(R) Iris(R) Xe Graphics   (OpenGL 4.5, offscreen, hardware-accelerated)
expected (8.250 16.500 24.750 33.000)
got      (8.250 16.500 24.750 33.000)
PASS: RGBA32F float-RT ping-pong, 8 passes at 256x256
OceanFFT::selfTest -> PASS ; readbackDisplacement -> 262144 floats, height range [-1.000, 1.000]
```

So on the cvcGL/VTK stack: offscreen GL works, RGBA32F render targets allocate and render, `vtkOpenGLQuadHelper` + `vtkOpenGLFramebufferObject` + `vtkTextureObject` drive multi-pass ping-pong, and float readback (`vtkTextureObject::Download`) is exact. **The whole FFT-ocean pipeline shape is buildable here.**

**One real gotcha surfaced, now documented in code:** blend and depth-test **must be explicitly disabled per pass** (`vtkOpenGLState::vtkglDisable`) or VTK's prior additive-blend state makes passes **accumulate** (a first run produced runaway `7.6e8` values). Two more VTK-9.5 specifics the OceanNode inherits: the quad VS attribute is `ndCoordIn` (own the VS + read source texels with `texelFetch(gl_FragCoord)` to avoid any varying mismatch — VTK's internal quad varying is neither `tcoordVC` nor `texCoordVC`), and `vtk_glew.h` is **not** exported to consumers (use `vtkOpenGLState` + `vtkTextureObject::Download`, or GL-enum literals, instead of raw GL).

**Scaffold landed (native):**
- `src/cvcGL/examples/OceanFFT.{h,cpp}` — the GPU core: RGBA32F RTs, `selfTest()` (the proof above), `step(t)` (placeholder travelling-wave field; **Phase 1** swaps in the JONSWAP spectrum + butterfly IFFT), `displacement()` / `readbackDisplacement()`. VTK-only, so it is unit-testable headless (verified) and drops into any cvcGL `SceneRenderer`.
- `src/cvcGL/examples/lsystem_coast.cpp` — a separate island+ocean demo (the forest control is untouched). Runs `selfTest()` at startup, then CPU-displaces an ocean grid each frame from `readbackDisplacement()` via `GeometryNode::updateVertices` (**Phase 2a**). Falls back to a flat sea if float RTs are absent.
- `src/cvcGL/examples/CMakeLists.txt` — `lsystem_coast` target added (native; **not** in `_wasm_demos` until Phase 4).

**Built natively and rendered (2026-08-29).** `lsystem_coast` compiled and linked against a from-source cvcGL and rendered a real frame: an island ringed by the GPU-`OceanFFT`-driven sea (`OceanFFT Phase-0 self-test: PASS` inside the demo), with per-frame `updateColors` giving foam-capped crests. The sea is visibly wavy; it is uniform/spiky because `step()` is still the placeholder 2-sinusoid field (Phase 1 fixes that), and the near field is flat-lit because Phase 2a keeps normals bind-pose (Phase 2b fixes that).

Build recipe (see [[cvcgl-examples-windows-build]] memory for the full trap writeup): direct CMake against the `deps` prefix — `-DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON`, CUDA/tests/CLI off, Ninja `-j6`. **Two traps hit and fixed:** (1) msys2's `pkgconf` leaked libpng from `C:\msys64\mingw64`, poisoning the MSVC compile with MinGW CRT headers — fix: `PKG_CONFIG_LIBDIR=<prefix>/lib/pkgconfig` + strip msys64 from PATH; (2) `vtkTextureObject` reports `IF=0` (float RTs "unsupported") if the RTs are allocated **before the first `Render()`** — the context's float-support flags are only set after `OpenGLInit`, so the demo draws one frame before `OceanFFT::init()`.

**Phase 1 — DONE (2026-08-29).** `OceanFFT::step()` now runs the real spectral pipeline: a one-time JONSWAP+conjugate-symmetry `h0` pass, per-frame time evolution → two packed complex buffers, an 8-stage Cooley–Tukey **butterfly IFFT** (bit-reversal + twiddle LUT generated CPU-side, 2 directions per buffer), and an assemble pass (permutation, choppy displacement, Jacobian foam). Verified numerically in the harness (**height mean = +0.000, Jacobian mean = +1.000, zero NaNs, evolves in time** — the signatures of a correct FFT) and visually: natural multi-scale swell with foam breaking on the crests. Raw output is already in metres.

**World-clock sync — DONE.** The waves advance on **`cvc::world_clock`** (world time, drift-free `t()`), so they move at correct speed regardless of frame rate, honour scale/pause, and are deterministic in a fixed-frame capture. `cvc::state.coast.clock.t` is published each frame.

**All water levers are in the state tree (live-modifiable).** `knob()` reads `cvc::state` each frame; a `--set coast.<key>=<value>` CLI override demonstrated calm→storm by respectralizing live. Schema:
| key | drives | live |
|---|---|---|
| `coast.water.wind_speed` / `.fetch` / `.tile_size` / `.depth` / `.wind_dir_x` / `.wind_dir_z` | JONSWAP spectrum (`rebuildSpectrum()` on change) | ✓ (respectralize) |
| `coast.water.chop` | horizontal-displacement scale (assemble `uLambda`) | ✓ per-frame |
| `coast.water.wave_amp` / `.choppiness` / `.foam_bias` | demo displacement + foam coloring | ✓ per-frame |
| `coast.clock.scale` / `.paused` | `world_clock` rate / pause | ✓ per-frame |
| `coast.clock.t` | published world time (observability) | — |

**Phase 2b — lit water DONE (2026-08-29).** The sea now has real form (light on crests, shade in troughs) via **smooth per-vertex normals** computed from the FFT height gradient (central differences on the tiled field) and fed through a new **`GeometryNode::updateNormals(xyz)`** — the normal twin of `updateVertices`/`updateColors` (in-place normal buffer overwrite, no mesh rebuild; requires a mesh that already carries normals, which `setGeometry`'s `ensureNormals()` provides). VTK's lighting (sun + wide fill + specular) then shades and glints the surface.

**Tests in place (ctest-green):**
- `src/cvcGL/test/cvcgl_ocean_fft.cpp` — 10 FFT-correctness checks (zero-mean height, unit-mean Jacobian, finiteness, determinism, time evolution, live wind-knob response); compiles `OceanFFT.cpp`, runs against an offscreen GL context, SKIPs (rc 0) without float RTs.
- `src/cvcGL/test/cvcgl_update_normals.cpp` — render-based check that `updateNormals` toward/away from the light changes brightness, plus the no-op-on-mismatch guard.
- Both use explicit `if`-return, **not `assert`** (NDEBUG no-op under Release — see `cvcgl_volume_range.cpp`), and are registered unconditionally beside the other `cvcgl_*` tests.

**Phase 2c — Fresnel BRDF + cascades DONE (2026-08-29).**
- **Multi-cascade FFT** — the sea is now **3 independent `OceanFFT` cascades** (non-harmonic tiles 251 / 83 / 29 m, weighted 1.0 / 0.55 / 0.32; each cascade's wind = `coast.water.wind_speed × factor`) summed at every vertex → swell + wind waves + ripples. Global spectrum knobs rebuild all three on change; `coast.water.tile_size` is retired (tiles are per-cascade in code).
- **Full CPU water BRDF** — VTK's Phong can't express Fresnel or reflection, so the sea is CPU-shaded (VTK lighting off; the BRDF is baked into the vertex colours): a deep-water body (lambert), **Fresnel** (`F0≈0.02`) mix toward a **sky-gradient reflection** of the reflected view ray, a sharp **sun glint** (`(R·sun)^200`, sun matched to the key light), and Jacobian foam. The camera position is read live from `renderer()->GetActiveCamera()` so Fresnel/glint stay correct as the view moves.

The result reads as a real, sunlit, multi-scale sea — dark saturated body, sky-lightened toward the horizon, sun-glint sparkling on the wave faces.

**GPU no-readback path — DONE (2026-08-29), now the default.** The sea is a **static** grid; its shader does everything on the GPU: the vertex stage samples the 3 OceanFFT displacement textures (bound live) and displaces a mutable copy of the position; the fragment stage runs the full Fresnel BRDF. **No per-frame CPU readback, no `updateVertices`/`updateColors`** — only the FFT `step()` (GPU) and a handful of uniforms. `--cpu` selects the old readback path for comparison.
- New cvcGL API on `GeometryNode`: **`setShaderTexture(name, vtkTextureObject*)`** (bind a live render-to-texture field to a `sampler2D`) + **`setShaderUniformf/i/3f`**, both applied every draw via the mapper's `UpdateShaderEvent`, guarded by `IsUniformUsed` so the depth-only shadow shader is skipped. This is the general "sample a live GPU field in a mesh shader" capability.
- **Two VTK traps hit and fixed:** with `disableCoordinateShiftScale`, `vertexMC` is an immutable `in` — so the position impl must be **fully replaced**, displacing a `vec4 dispMC = vertexMC;` copy and driving `gl_Position`/`vertexVCVSOutput` from it (not `vertexMC.xy += …`, which the roadmap's sway note implies works but does **not** with shift-scale off). And the water BRDF **consumes** `//VTK::Light::Impl`, writing `gl_FragData[0]` directly (no `CVC_FS_NORMAL` needed — it doesn't touch `normalVCVSOutput`). `OceanFFT`'s output texture is set Linear + Repeat (the FFT field is exactly periodic) for seamless tiled sampling.

**WASM wiring done.** `lsystem_coast` added to `_wasm_demos` (`src/cvcGL/examples/CMakeLists.txt`) and to `DEMOS` in `.github/workflows/deploy-pages.yml`, so it builds + deploys to `transfix.github.io` post-merge on catx-03. The GPU path is what makes this viable: the CPU readback (`vtkTextureObject::Download`, glGetTexImage-style) is not a WebGL2 operation.

**Remaining:** a local/CI wasm build verification; one cosmetic (`setGridVisible(false)` doesn't fully suppress the world-bounds box). `GeometryNode::updateNormals` (Phase 2b) remains the fast path for VTK-lit deformed meshes.

## A.9 Phase-1 port reference — ABYSSAL FFT internals (MIT)

Captured byte-exact from `src/ocean/OceanFFT.js` (MIT, © 2026 Davi / Token-Gremlin) as the concrete target for replacing the `OceanFFT::step()` placeholder. `g = 9.80665`.

**Spectrum / h0 init** (JONSWAP + TMA + Donelan–Banner `cos²ˢ`):
```glsl
omega = sqrt(g * k * tanh(min(k*depth, 20.0)));            // dispersion
// jonswap(omega, sb=(alpha,peak,gamma)):
sigma = omega<=peak ? 0.07 : 0.09;
r     = exp(-(omega-peak)^2 / (2*sigma^2*peak^2));
S     = tmaCorrection(omega)*alpha*g*g*omega^-5 * exp(-1.25*(peak/omega)^4) * gamma^r;
// tmaCorrection(omega): oh=omega*sqrt(depth/g); oh<=1 ->0.5*oh^2; oh<2 ->1-0.5*(2-oh)^2; else 1
// spread power: omega>peak ? 9.77*(omega/peak)^-2.5 : 6.97*(omega/peak)^5 ; cos2s + normFactor(s) piecewise
h0  = gaussNoise.xy * sqrt(2.0 * max(S,0) * dk * dk);
oH0 = vec4(h0k, h0mk.x, -h0mk.y);                          // pack h0(k) and conj h0(-k), m=(N-p)
```
**Time evolution → MRT** (complex packed as `f + i·g` across 2 attachments):
```glsl
phase = omega*uTime; e=vec2(cos,sin); ec=vec2(e.x,-e.y);
h  = cmul(h0.xy,e) + cmul(h0.zw,ec);  ih = vec2(-h.y,h.x);
Dx=ih*kn.x; Dz=ih*kn.y; Dy=h; DyDx=ih*k.x; DyDz=ih*k.y; DxDx=-h*k.x*kn.x; DzDz=-h*k.y*kn.y; DxDz=-h*k.y*kn.x;
oBuf0 = vec4(Dx.x-Dz.y,   Dx.y+Dz.x,   Dy.x-DyDx.y, Dy.y+DyDx.x);   // mrtA[0]
oBuf1 = vec4(DyDz.x-DxDx.y, DyDz.y+DxDx.x, DzDz.x-DxDz.y, DzDz.y+DxDz.x); // mrtA[1]
```
**Butterfly IFFT** — LUT texture `(log2N, N)`, `bf.xy`=twiddle(cos,sin), `bf.zw`=wing indices; per stage, 2 directions, ping-pong mrtA↔mrtB:
```glsl
o0 = vec4(pa.rg + cmul(bf.xy, pb.rg), pa.ba + cmul(bf.xy, pb.ba));
```
**Assemble** — permute `perm=((x+y)%2==0)?1:-1`, extract the 8 reals, then:
```glsl
jac = (1+lambda*DxDx)*(1+lambda*DzDz) - (lambda*DxDz)^2;   // folding => foam
oDisp  = vec4(lambda*Dx, Dy, lambda*Dz, jac);
oDeriv = vec4(DyDx, DyDz, lambda*DxDx, lambda*DzDz);
foam   = prev.r*exp(-dt*decay) + smoothstep(bias, bias-0.30, jac)*mul*dt;
```
**Cascades:** 3 non-harmonic tiles **4099 / 389 / 41.3 m**, wavenumber bands `[[1e-4,b1],[b1,b2],[b2,9999]]` with `bi = 2π/Li·4`; each cascade → `uOceanDisp/Deriv/Turb i`, materials composite all three.

**Port order:** (1) CPU-generate the Gaussian-noise + butterfly LUT textures once (already stubbed as `m_noise`); (2) spectrum→h0 pass; (3) time pass → MRT pair; (4) `log2(256)=8`-stage butterfly ×2 dirs (the ping-pong `selfTest` already proves); (5) assemble → `displacement()`; (6) add cascades 2–3. Verify each stage against a known IFFT of an impulse before wiring shading.

---

# Part B — Tet / volumetric terrain, L-system tets, deformable × LOD

## B.1 Tets for terrain — no

Terrain is a strictly 2.5-D analytic, resolution-independent heightfield: *"a pure function of world (x, y) plus a stored delta grid"* **[R, Lab §4.4 L423]**, single-valued in z (one float/column, L1308). That single-valued property is load-bearing in three independent places:

- **Single-`grid_spec` invariant** — one `raster(grid_spec)` emits class/risk/hard/occupancy/height together, so misalignment is *"structurally unrepresentable"* **[R, §0.6 L19]**, *"one of the two or three best properties of this design"* **[R, L2761]**.
- **Picking** — the brush ray-marches `heightfield::sample(x,y)`, never the render mesh; at T3 a pick lands *"up to 8 m from the click"* **[R, L108/L2088]**.
- **Export fidelity** — material/nav export runs from the analytic surface, byte-identical regardless of render rung **[R, L104]**.

Tetrahedralizing a single-valued surface tetrahedralizes solid rock nobody sees, and it gets **zero relief** from the LOD apparatus (octave band-limiting L439; 128 m chunks T0–T4 at 2/4/8/16/32 m, 65²→5² verts L1716-1726; 2.5-D XZ-quadtree + Y-interval **[R, VIS L475]**; a horizon occluder that **requires a heightfield** **[R, VIS L508]**). The reference tet renderer is *"insensitive to viewport and camera distance… no fill relief, no distance-LOD relief, no culling relief"* **[R, WATER §1.2 L23]**.

> **[P] caveat:** 0.43 µs/tet, 100 % CPU is a **projection against VTK 9.5's stock `vtkProjectedTetrahedra` mapper for an as-yet-unbuilt `UnstructuredVolumeNode`**, not a shipped cvcGL measurement, and is a property of the PT pipeline specifically (WATER §1.3 flags an FBO-off confound; D-W6 re-measure pending). The structural conclusion (no LOD relief) holds within the design's own scope.

## B.2 Tets for onshore water — no (already settled as W5)

Homogeneous water is a closed surface: *"the tetrahedra contribute exactly zero information. This is arithmetic, not taste"* (Beer–Lambert) **[R, WATER §1.2 L24]**. W5 is opt-in (`--water-tier=volume`), **native-only (compiled out of wasm)**, hard-capped `max_tets=4800`/one body, refusing above the cap **[R, §6.3 L853-860]**; one 28×28×3 pond eats the whole budget (~2.1 ms); *"W5 has no LOD, and that is a first-class disqualifier"* **[R, L975]**. Onshore ponds/lakes render as closed surfaces. Reserve tets for genuine FEM/CFD interior fields meeting all four WATER §6.1 conditions.

## B.3 L-system → tet meshes — yes mechanically; renderer is the missing piece

The generation half is real, native and default-on:

- `inc/cvc/utility/algorithm.h` exposes `cvc::sdf()`, `iso()`, `tetrahedralize()`, `hexahedralize()`, `tetrahedralize2()` (incl. an interval overload meshing *between two isosurfaces*) **[C]**; `CVC_ENABLE_MESHER`/`CVC_ENABLE_SDF` are `ON` by default (`CMakeLists.txt:241-242`) **[C]**. Engine is the in-tree **LBIE octree mesher**; `cvc::geometry` stores `tet_t/hex_t/tets_t/hexs_t` (`geometry.h:70-85`) **[C]**.
- The L-system already emits real 3-D solid triangle meshes via `mesh_emitter` into `cvc::geometry` **[R, L728]** (trees, rocks, buildings; `boulder_fracture` ~1280-tri closed hulls). `SDF_LIBRARY.md` shows the idiom: `cvc::sdf(mesh)` → `cvc::tetrahedralize2(inv_sdf, -0.05, 0.05)`.

**Natural pipeline:** grammar grows a 3-D network → `cvc::sdf()` field → `cvc::iso()` surface *or* `tetrahedralize()` volume → mesh. Every stage except the renderer is real code today.

**Compelling only where terrain is genuinely volumetric** — caves, arches, overhangs, karst, tunnels (multiple z per column) — which the heightfield physically cannot express. **A mismatch for base terrain:** TR1 archipelago is *"composition, not a grammar"* **[R, L859]**; surface grammars (rivers/trails/mudflats) only paint classes, adding no z **[R, L860-862]**. Today, arch/overhang geometry exists only as placed archetype meshes (hoodoo K3; `champagnat_cane` arch — *"the arch is the model"* **[R, L823/L846]**).

**New work even for the compelling case:** the **renderer** — no `UnstructuredVolumeNode`/`vtkProjectedTetrahedra` in `src/cvcGL`; `VolumeNode` renders *structured* grids only; the planned unstructured node (~250 LoC) is *"the first unstructured path cvcGL will have"* **[R, WATER L57]**, unbuilt. And usually you want an **isosurface (triangle surface)** for hollow terrain, not tets; tets are for scientific interior fields.

> **caveats:** a from-scratch **Windows build of the mesher path is not re-verified** (memory records pycvc/material Windows builds needed non-trivial infra fixes); and whether `tetrahedralize()` populates `.tets()` natively vs. a triangle round-trip (there are `encode/decode_tets_from_triangles` helpers; `lbie_mesher_test` asserts TETRA via `numtris%4==0`) is **unconfirmed** — spike before relying on the tet container. `cvc::sdf()`'s real signature needs an app context + dimension/bbox and defaults to `SDF_V1`.

## B.4 Deformable terrain × LOD — heightfield yes, volumetric/tet no

**Cheap heightfield deformation is essentially already the architecture.** Edits/erosion apply as a delta: `h(x,y) = analytic + delta.sample(x,y)` **[R, L429]** — one 2048² f32 raster (16 MB) holding erosion + authored edits **[R, L450]**. Deform = write the delta, re-derive the affected chunk, which re-enters the same `select_rung` ladder; staying single-valued preserves picking / single-`grid_spec` / export. Latency is tiered: **Tier-2 debounced re-mesh** (180 ms debounce, double-buffered commit that never blanks the viewport, ≤6 s convergence) **[R, L2137/L2144]** for interactive edits; **Tier-3 region-scoped erosion** for bakes **[R, L450]**.

**The one missing piece:** no height sculpt/dig **brush** — the named brushes paint material *class*, not height. A live raise/lower/dig brush is a small new UI writing into the existing delta store; its tier (continuous vs. debounced) is unspecified.

**Volumetric/tet deformation is the opposite:** tets get no LOD relief and any topology change *"must redo preprocessing"* **[R, WATER L803]** — cannot be cheaply re-meshed or LOD'd, so it cannot leverage the LOD at all.

## B.5 If dig-anywhere volumetric deformation is genuinely wanted

The correct primitive is **SDF/voxel field + dual-contouring (or LBIE isosurface) → triangle surface, chunked under a clipmap/chunk LOD — not tets.** The output is a triangle surface, so it drops into everything the LOD/cull system already assumes (*"1,024× the triangles costs +0.20 ms"* **[R, L136]**); a dig writes a bounded field region and re-contours only dirty chunks, mirroring the delta-grid locality (*"an edit inside one leaf invalidates that leaf and its ancestors, not the world"* **[R, L1282]**). **Present vs. new:** the field/extraction primitives are real and default-on (`cvc::sdf` v1/v2, `cvc::iso`, LBIE, offline dual contouring); the **runtime** loop (live editable sparse voxel/SDF store, per-frame dirty-region re-contour, clipmap over the field, seam handling) is **research-grade and absent** — the in-tree SDF/dual-contour is offline/bake only, and it departs from the single-valued invariants. Do it only if the volumetric requirement is real.

## B.6 The decision that flips everything

**What is the volumetric terrain actually *for*?**

| Goal | Right answer | Tets? |
|---|---|---|
| Visual overhangs / caves / arches | Triangle surfaces — archetype meshes now, or grammar → SDF → `iso` | No |
| A scientific *interior field* (subsurface geoscience: stratigraphy, pore pressure, seismic velocity, groundwater; or FEM/CFD) that lives natively on an unstructured mesh and must not be resampled | Tet/hex volume + `UnstructuredVolumeNode` | **Yes — the one legit case**, and it is FEM/CFD viz, not "terrain" |

Sub-questions that size the work: is "deformable" **edit-time** (≈ today's architecture) or **per-frame runtime** (the large new subsystem)? Must terrain stay **single-valued** so GRL-SNAM's 2-D nav/occupancy export and picking still work (a volumetric rep has no defined export to that consumer today)?

---

## 7. Evidence gaps (read before relying on a number)

- **cvcGL internals for Part A** were partly reconstructed after one subagent under-returned; backfilled by direct `src/cvcGL` reads (§A.3 citations are `[C]`, verified). The multi-pass-ping-pong linchpin is confirmed *primitives-exist / pattern-new*, not *proven-buildable*.
- **ABYSSAL source** was read via a summarizer, not byte-exact `[X]`; the MRT complex-field packing and N=256 default call-site were inferred. Byte-exact read advised before porting the FFT packing.
- **Tet cost numbers** are `[P]` projections vs. stock VTK PT for an unbuilt node (§B.1).
- **LOD/vis system** is *"design, not yet implemented"* **[R, VIS L4]**; only `cvc::lod::select` + a brute-force `cvc::vis` oracle are coded. The tet-vs-LOD conclusion is unaffected (no relief from a designed *or* built LOD).
- **Mesher/SDF Windows build** and the `.tets()` population path are unverified (§B.3).
