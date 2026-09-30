# Ariadne AppRuntime — lifting the app-host bundling into `cvc::ariadne`

*Design note for splitting `cvc::gl::ariadne::AriRuntime` into a backend-neutral host
(`cvc::ariadne::AppRuntime`) + a pluggable, optional `SceneAdapter`, leaving only the
GL/VTK/ImGui concretions in cvcGL.*

## Why

`AriRuntime` bundles the wiring `ariadne_hello.cpp` did by hand — load a `.ari`, run
its `init:`/`on_*` scripts, register host verbs, realize the scene, and drive the
per-frame `drain → visibility → tick` loop. That bundling is **backend-neutral**: it
is the same sequence for an ImGui-over-VTK host and for a terminal (FTXUI) host. But
it currently lives in cvcGL and hard-codes the GL backend, which undercuts the whole
point of the `Backend` abstraction (`ImGuiBackend` vs FTXUI): a terminal host would
have to reimplement the identical bundling.

The one genuinely GL-bound part is **scene realization** — turning the parsed
`cvc::ariadne::Scene` into a live `cvc::gl::SceneGraph` (`realize_scene`) and servicing
it per frame (`tick_scene`, which needs a `vtkRenderer`). Everything else the class
does is pure libcvc.

## The split

Two neutral types move into `cvc::ariadne` (the `cvc` core library); the GL host shrinks
to a thin composer.

### `cvc::ariadne::SceneAdapter` (new, abstract — mirrors `Backend`)

The scene is delegated exactly the way the widget surface is delegated to `Backend`:
an interface `cvc::ariadne` calls and a concrete layer implements.

```cpp
namespace cvc::ariadne {
class SceneAdapter {
public:
  virtual ~SceneAdapter() = default;
  // Realize the parsed scene under `prefix` (and frame the view's camera to it, if any).
  virtual void realize(const Scene &scene, const std::string &prefix,
                       std::vector<std::string> *warns) = 0;
  // Mirror bound `visible:` state onto scene nodes. No-op until a scene is realized.
  virtual void sync_visibility(cvc::app &app) = 0;
  // Per-frame scene service (volume tickers, …). No-op until a scene is realized.
  virtual void tick() = 0;
};
}
```

`SceneVisibilityBinding` (what `sync_scene_visibility` consumes) is already a neutral
`cvc::ariadne` type, so the interface takes only neutral types — no GL leakage.

### `cvc::ariadne::AppRuntime` (new, neutral bundling)

Owns the `Runtime`, the loader/init/verb wiring, and the neutral per-frame core. Holds
a `SceneAdapter*` that **may be null** — that is the load-bearing design point: a host
with no way to render a scene graph (a terminal SDK backend today; an ASCII rasterizer
later) constructs `AppRuntime` with `scene == nullptr` and everything except scene
realize/sync/tick works unchanged; loading a scene-bearing `.ari` on such a host emits
a warning and drops the scene rather than failing.

```cpp
namespace cvc::ariadne {
class AppRuntime {
public:
  AppRuntime(cvc::app &app, std::string prefix, SceneAdapter *scene = nullptr);
  ~AppRuntime();

  void set_backend(Backend &backend);              // parameterize over the widget backend
  Runtime &runtime();                              // escape hatch: backend attach / advanced hosts

  void add_component_path(const std::string &dir);
  void on(const std::string &event, std::function<void()> handler);
  void register_verb(const std::string &name, std::function<void()> fn);
  void register_async_verb(const std::string &name, const std::string &done_channel,
                           std::function<std::string()> work);

  std::vector<std::string> load(const std::string &path);
  void set_root(Widget root);

  void drain();          // NEUTRAL per-frame core: actions + scheduler slice + scene sync/tick
  bool should_close() const;
  void request_close();

  void post_key(bool down, const std::string &key, int mods = 0, bool repeat = false);
  void post_pointer(int kind, double x, double y, double dx, double dy, int button, int clicks);

  std::vector<std::string> take_warnings();
  bool reload_if_changed();
  static bool have_yaml();
};
}
```

`drain()` is the full neutral per-frame step:

```cpp
rt_.drain();                                   // queued actions on this thread + a scheduler slice
if (scene_) { scene_->sync_visibility(app_); scene_->tick(); }
(void)rt_.take_reactive_warnings();
```

Input pumping, camera integration, and the draw are **not** here — they are backend/host
concrete (VTK `processUIEvents` + `cam.update` + `view.render()` for GL; terminal I/O for
FTXUI). The neutral core needs no `dt` (only the GL camera does).

### `cvc::gl::ariadne::AriRuntime` (thin GL host — public API unchanged)

Keeps its exact signature so `pycvc_ari.i`, `demo.ari`, and the CI render test do not
change. It now composes:

- an `ImGuiBackend` (as before),
- a `GlSceneAdapter` implementing `SceneAdapter` over the `SceneRenderer` +
  `CameraController` (`realize_scene` / `sync_scene_visibility` / `tick_scene`), and
- an `AppRuntime(app, prefix, &sceneAdapter)`, to which every setup/document/diagnostic
  method forwards.

Its `frame(dt)` stays the GL frame driver — `view.processUIEvents(); cam.update(dt);
app_rt.drain();` — and `render()` stays `view.render()`. Member order: `backend_` and
`scene_` are declared **before** `app_rt_`, so the `Runtime` (which points at the backend)
and the `AppRuntime` (which points at the adapter) are torn down first; `~AriRuntime`
still clears the overlay draw callback before any member dies.

## Lifetime & ordering (unchanged guarantees)

- `Backend` outlives `Runtime`: `backend_` declared before `app_rt_` (which owns `rt_`).
- `SceneAdapter` outlives `AppRuntime`'s use: `scene_` declared before `app_rt_`.
- The overlay draw callback (captures `app_rt_.runtime()`) is cleared first in
  `~AriRuntime`, closing the Python-GC-stray-render window exactly as before.

## Blast radius

- New: `inc/cvc/ariadne/scene_adapter.h`, `inc/cvc/ariadne/app_runtime.h`,
  `src/cvc/ariadne/app_runtime.cpp` (added to the `cvc` target's ariadne source list).
- Rewritten (thin): `inc/cvc/gl/ariadne/AriRuntime.h`, `src/cvcGL/ariadne/AriRuntime.cpp`
  (cvcGL globs `ariadne/*.cpp`, so no cvcGL CMake edit).
- Unchanged: `pycvc_ari.i`, `demo.ari`, `test_pycvc_ari.py`, all callers — the GL
  `AriRuntime` API is byte-for-byte the same.

## Follow-ups enabled (not in this PR)

- A terminal (`FtxuiBackend`) host built directly on `AppRuntime` with `scene == nullptr`
  — the concrete motivation for the split.
- The document-level `on_key`/`on_pointer` VTK producer (still a TODO) now lands cleanly
  in the GL host, feeding `app_rt.post_key`/`post_pointer`.
