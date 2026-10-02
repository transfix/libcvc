# glsync: Firefox WebGL sync census for cvcGL wasm apps

`glsync.js` wraps every WebGL2 method on the page and sorts each call by what Firefox 156 does with it:

- **S**: a synchronous round trip to the GPU process
- **C**: answered in the content process
- **A**: queued
- **F**: flushed without waiting

It also models Firefox's async-present flush budget. Once per report window it prints one `GLSYNC` line to the console, and appends a short version to the page's `#profhud` element (a HUD) when it has one. Without `?glsync` in the URL it does nothing; the only cost is one URL check.

The model is Firefox 156's. In any other browser, every line it prints says `model is Firefox-only`.

It is a developer tool, not part of the cvcGL SDK. It is how the WebGL state shim (`../webgl_state_shadow.js`, linked by `cvcgl_wasm_app()`) was found and is checked: with the shim on, the census should show no `ACTIVE_TEXTURE`, `SCISSOR_BOX` or `BLEND_*` sync calls. Compare against `?glshim=0`.

## Getting the script into the page

The cvcGL dev server injects it:

```
python3 src/cvcGL/examples/wasm/serve.py -d build-wasm-mt/gallery --glsync [--js-profiling] [--prof-param prof] 8822
```

- `--glsync[=PATH]` inserts `<script src="/glsync.js"></script>` before the first `<script>` of every served `index.html`, so the census wraps WebGL before anything else does: before the page's own scripts and before the module's `--pre-js` state shim. `/glsync.js` is served from PATH whatever `-d` says. The default is this file in the source tree, or `devtools/glsync.js` next to an installed `serve.py`. It is read on every request: after editing it, reload the page; there is no need to restart the server.
- `--prof-param NAME` also injects `window.__glsyncProfParam = "NAME"`, and the census then adds `?NAME` to the URL. Use it for an app that prints `PROF n=` lines only with a URL switch, such as `?prof`.
- `--js-profiling` sends `Document-Policy: js-profiling`, which Chromium requires before it exposes the JS Self-Profiling API (`new Profiler(...)`). That API samples wasm frames too, so a build linked with `--profiling-funcs` yields a named, function-level CPU profile.

All three are off by default. With none of them, `serve.py` is the plain cross-origin-isolated server that the `cvcgl-examples-web` launcher runs.

## URLs

- `http://localhost:8822/nav_city_drive/?glsync`: census on.
- `...?glsync=seq,stacks`: also prints the worst task's sync calls in order, and the stack of each slow sync call.
- `...?glsync&glshim=0`: without the state shim, for the A/B.
- `...?glsync&frameyield=vtk`: with VTK's in-render yield back on (see [Frames and tasks](#frames-and-tasks)). An app linked with `-sASYNCIFY_IGNORE_INDIRECT=1` locks `FrameYield::App` and refuses this.

Options go after `?glsync=` and are comma-separated:

| option | meaning |
|---|---|
| `1`, `on`, `true` (or nothing) | on, with the defaults |
| `thr=<ms>` | list keys at or above this many ms per frame (default 0.1; S keys are always listed). A bare number other than `1` also works: `0.5`, `2`, `1.0` |
| `shmem=<bytes>` | the command shmem size to model. Default 100000, Firefox's default for `webgl.out-of-process.shmem-size`. See the A/B below |
| `clock=auto\|prof\|raf` | the frame clock: when a report window ends and what counts as a frame. Default `auto`. See [Frame clock](#frame-clock) |
| `every=<N>` | paints per window on the `raf` clock (default 60) |
| `prof` | add `?prof` to the URL, unless the URL already has `prof`, or `profhud` or `glcount` (other switches with which an app may already print PROF lines). The same as `serve.py --prof-param prof` |
| `seq` | adds a `GLSYNC seq` line with the worst JS task's sync calls in order (a task is a whole frame only with `FrameYield::App`). `*` marks a call that waited for queued commands (the drain) |
| `stacks` | adds a `GLSYNC stack` line with the JS/wasm caller of each key's first call of 2 ms or more |
| `lite` | plain queued calls are counted but not timed, which cuts the overhead. The gl ms figure is then tagged `(A untimed)`. That tag is approximate: `lite` still times texture uploads (`texImage*`, `texSubImage*`) and the queued slots of `useProgram`, `uniformBlockBinding`, `readPixels` into a PACK buffer, and `getExtension(enable)` |
| `all` | list every key |
| `nohud` | console only |
| `0`, `off`, `false` | disabled |

The startup line `glsync: census on ...` repeats the settings in use: the shmem value, the threshold, the calibrated per-call overhead (`ovh~`, approximate), the paint clock and the frame clock.

Console helpers:
- `__glsync.table()`: per-key totals since load.
- `__glsync.last()`: the last `GLSYNC` line.
- `__glsync.lastWindow()`: the raw flush figures behind it.
- `__glsync.model()`: the flush model's state right now.
- `__glsync.clock()`: the frame clock reporting now (`raf` or `prof`).

## Frame clock

- **`prof`**: a report right after each `PROF n=<frame>` console line, for an app that prints its own frame-timing line (anything may follow the frame number). `f` is the frame delta since the previous one. An app that pauses prints `PROF paused n=<frame>` when it pauses and `PROF resumed n=<frame>` when it resumes. A report made while paused is tagged `paused`, and `resumed` restarts the window at that frame, so what was drawn while paused is not divided into the next report's frames.
- **`raf`**: a report every `every=N` paints of a drawn canvas (default 60). `f` is the number of JS tasks in the window that drew to the default framebuffer (the canvas). This is the clock for pages whose output never reaches `console.log`. The cvcGL gallery page sends `Module.print` to a DOM node, and the gallery demos print no PROF lines anyway. The line says `clock=raf`.
- **`auto`** (default): `raf` until the first `PROF n=` line, then `prof` from that line on. The first `prof` window starts `warming`.

## Reading the GLSYNC line

Format example from the Node harness. **This is a mock frame, so the numbers mean nothing:**

```
GLSYNC n=20 f=10 | gl 2.7k/f 3.8ms | sync 13.6x 2.8ms (drain~2.7 rt~11us) | presents 10/10f flush~10.4/11.4 max 12/13 >10 20%/100% (lo 2 / hi 10) [4.0 chunk + 5.4 sync + 0.0 auto + 1 present; hi +1.0 race] shmem=100000 | ovh~0.31ms | top: getParameter(ACTIVE_TEXTURE)[S] 1.0x 2.6ms max 2.6 | ...
```

- **`n=20 f=10`**: the frame number, and the frames in this window. On the `prof` clock these are PROF frames; on the `raf` clock they are tasks that drew to the canvas (`clock=raf` follows). Counts and ms are per frame unless noted.
- **`gl 2.7k/f 3.8ms`**: WebGL calls per frame and their summed time.
- **`sync 13.6x 2.8ms`**: synchronous calls per frame and their time.
  - `drain~` is the part spent waiting for queued commands to run in the GPU process.
  - `rt~` is the median bare round trip.
- **`presents 10/10f`**: canvas presents in the window, against frames. Firefox presents only in a refresh-driver paint, and the census uses `requestAnimationFrame` as that paint clock. If presents are fewer than frames, paints were skipped or late, and each present then carries several frames' flushes. Presents can also outnumber frames: with `FrameYield::Vtk`, a paint can land between the tasks of one frame, so that frame is presented twice.
- **`flush~lo/hi`** (per present): flushes since the previous present, which is the counter Firefox checks. Above 10, the present is synchronous and waits for the GPU process.
  - **lo** is the expected count. It assumes the paint runs before the last frame's deferred auto-flush and cancels it.
  - **hi** is the bound. It assumes that auto-flush ran first.
- **`max`**: the worst present, as lo/hi.
- **`>10 20%/100% (lo 2 / hi 10)`**: presents over the budget, as a share and then as a count. `lo` is the expected-case count, from the lo flush figure. `hi` is the bound: it also counts presents that only the race pushes over. These are the presents that lost async present. A present is not a frame: one present can carry several frames, and with `FrameYield::Vtk` one frame can have two presents.
- **`[... chunk + ... sync + ... auto + ... F + 1 present; hi +r race]`**: per-present causes. They add up to lo:
  - `chunk`: shmem overflows (command volume).
  - `sync`: sync calls with commands queued.
  - `auto`: deferred auto-flushes that certainly ran.
  - `F`: `gl.flush()` calls and DOM-source uploads.
  - `present`: the present's own flush.
  - Adding `race` gives hi.
- **`ping20 n` / `syncping n`** appear only when the counter reached 20 flushes (Firefox sends an async Ping, so the >10 verdict for that present is uncertain) or passed 70 (Firefox sends a synchronous SyncPing).
- **`ovh~`**: roughly what the census itself adds per frame: calls per wrapper kind times a per-call cost that is calibrated on no-op calls at load. It is an approximation, not a floor or a bound. In Node, a wrapped call costs about 100 ns, and the calibration lands close to that. A `lite` queued call costs about 2 ns, but calibrates at about 20 ns, so `lite`'s `ovh~` runs high.
- **`top:`**: keys by time, as `key[class] calls/f ms/f max`. `[S]` keys are always listed. `!n` marks a non-sync key with n calls of 1 ms or more, an unexpected stall.

`presents 0/10f (no paint of a drawn canvas)` means nothing was drawn to the canvas in that window, or no paint has happened yet.

## The shmem A/B (webgl.out-of-process.shmem-size)

The `chunk` part of flush~ is command volume divided by the shmem size. To test a bigger shmem:

1. **Baseline.** Load `...?glsync`, let it run, and note `chunk`, `flush~`, the `>10` share, and the app's frame rate.
2. **Change the pref.** In `about:config`, set `webgl.out-of-process.shmem-size` to, for example, `1000000`.
3. **Reload the page.** Firefox reads the pref when the WebGL context is created, so the running page keeps the old size.
4. **Tell the census.** Load `...?glsync=shmem=1000000`. The census cannot read about:config, so without this it keeps modelling 100000-byte chunks. The value shows in the startup line and in every GLSYNC line (`shmem=`).
5. **Compare.** `chunk` should fall. If the `>10` share falls with it, and the frame rate rises, command volume was costing async present.
6. **Reset the pref** afterwards.

`webgl.out-of-process.async-present` must be `true` (the default) for the budget to matter at all.

## What it does not tell you

The Node tests run a synthetic heavy-overlay frame with hand-picked call volumes. The figures it produces, like the example line above, are a **harness self-test only**: they check the census's bookkeeping against hand-computed values. They are not evidence about any real app. Only a live `?glsync` run in Firefox measures that.

Known gaps (none occurs per frame in the cvcGL demos):
- Client-side validation errors schedule an auto-flush.
- DOM uploads that miss the GPU-copy fast path upload inline instead.
- A canvas resize queues a command and dirties the canvas.
- `getFramebufferAttachmentParameter(DEPTH_STENCIL_ATTACHMENT)` syncs twice.
- Byte sizes are estimates, about 32 B per command plus payload.
- It models one context.
- It treats a task's WebGL calls as one burst, which ends at the task's first microtask checkpoint. A task that spreads its calls over several microtasks would count its own pending auto-flush as already run. The cvcGL apps make all of a task's calls in one run.

Extension-dependent classes (`getExtension`, `DEPTH_CLAMP`, `PROVOKING_VERTEX_WEBGL`, `MAX_VIEWS_OVR`, `UNMASKED_*`, `OBJECT_NAME`) and `drawingBufferWidth/Height` follow the page's actual state. `drawingBufferWidth/Height` is S only after a canvas resize with no read and no paint since, because Firefox refetches the size in the first paint and in the first paint after each resize. The header of `glsync.js` has the details and the Firefox source references.

### Frames and tasks

The census sees JS tasks and paints, not frames. A frame is one task only in an app that yields once per frame. cvcGL's `FrameYield::App` (`SceneRenderer::setFrameYield`) is that mode, and every cvcGL gallery demo uses it. With `FrameYield::Vtk` (VTK's own behaviour, or `?frameyield=vtk`), `vtkWebAssemblyOpenGLRenderWindow::Frame()` in VTK 9.5 also calls `emscripten_sleep(0)` inside every render, so each frame is two or more tasks. In that mode:
- The `seq` line shows the worst task, not the worst frame.
- On the `raf` clock, `f` counts every task that drew, which can be more than one per frame.
- A paint can land between the tasks of one frame, so a frame can be presented twice and `presents` can exceed `f`.

The flush figures stay right either way: the paint clock and the auto-flush bookkeeping follow the real task and paint boundaries.

## Tests

```
NODE=$CVC_EMSDK_DIR/node/*/bin/node
$NODE tests/test_glsync.js [--show]   # mock WebGL2 + fake rAF; --show prints the census output
python3 tests/test_serve.py           # serve.py's flags, on ephemeral 127.0.0.1 ports
```

`src/cvcGL/wasm/run-js-tests.sh` runs these and the state shim's test with the emsdk's node. CI runs that script. ctest registers the node tests as `cvcgl_glsync` and `cvcgl_webgl_state_shadow` (label `js`).
