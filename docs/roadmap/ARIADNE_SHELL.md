# Ariadne shell: one document that hosts other documents

*Draft design, 2026-11-22, with the decisions of 2026-11-23 in section 6. Nothing in it is
built. The section numbers in [ARIADNE.md](../ARIADNE.md) are cited as §N.*

## 1. Goal

A standard entry document, `cvc://components/shell.ari`, that runs other `.ari` documents as
applications:

- it lists a catalog of applications (first, the examples' `gallery.json`) in its main menu, each
  with its name and its icon, loads one only when the user picks it, runs several at once, and
  closes them to free their memory and compute until they are picked again. It replaces arigl's
  gallery mode, and later the browser gallery;
- it opens any other document too, local or from the network, as a browser opens any page:
  sandboxed by default, with settings that narrow what it accepts (4.12, 4.13);
- they share its menu bar, tray and notification bell;
- each application's scene is drawn either in a window of its own (a viewport) or in the main
  view, chosen per application and switchable while it runs; one application at a time, the
  *front* application, may have the main view;
- an application that asks may take over what only the outermost document gets today (keys,
  the pointer, signals) and add to the shell (its menus, tray, right-click menus and Settings
  window), and may even change the shell's settings, but only what the shell grants it;
- a Processes window lists the DSL processes running in the shell and in each application;
- a link to an `.ari` document, clicked in an application such as nicotina's feed, opens that
  document from the network as an application of its own, once the user has seen what it is and
  what it asks for (4.12).

The first user is one arigl window that can open every libcvc example document. The goal beyond
it is 4.12's golden: applications that travel as links in posts.

**The design follows two rules.**

- **A document runs the same mounted as it does on its own.** Under its mount's sub-prefix (§7.3)
  it finds the keys it would find at its own prefix: its scene's nodes at
  `graphics.root.children.<node>`, its camera at `viewers.main.camera`, and its own state. Most
  of the gaps in section 3 are places where that is not true today.
- **A mounted document gets nothing beyond its own scope unless it asks and the document that
  loads it grants it.** That is how `link:` holes and `functions:` work today (§7.4, §6.6), and
  4.5 extends it to input, signals and the shell's UI.

## 2. What exists

| Piece | Where | What it gives the shell |
|---|---|---|
| `load:`, isolated mounts | §6.3, §7.3, §7.4; `expand_load` in [loader.cpp](../../src/cvc/ariadne/loader.cpp) | A document under `includes.<as>`, default-deny state, `link:` holes, granted functions, its own `init:` and `on_tick:` |
| Loaded documents as applications | §5.25, transfix/libcvc#1241 | A mount with its own `menubar:` is one menu of the outer bar, with Restart and Quit; Windows > Applications ticks it on and off; its tray icons join the outer tray |
| One notification feed per app | §5.24, `cvc/core/notify.h` | Every mount posts to the feed the outer bell shows, named by its `meta.name` |
| `repeat:` on a `load:` | §5.25 | Mount instances counted each frame, but all of one URI |
| arigl's gallery mode | §9, [src/arigl/gallery.h](../../src/arigl/gallery.h), `AppRuntime::set_host_menus` | A Demos menu over a JSON catalog. Picking a document *replaces* the running one: `load()` clears the last scene first |
| Scenes of their own | §5.18, `realize_scenes` in [AriRuntime.cpp](../../src/cvcGL/ariadne/AriRuntime.cpp) | One SceneGraph per `scenes:` entry at `<prefix>.scenes.<id>`, with its own hidden viewport and physics key; viewports mirror it |
| `(ps)`, `(inspect)`, `(kill)` | [STATE_EXEC_DEVELOPER_GUIDE.md](../STATE_EXEC_DEVELOPER_GUIDE.md), `intrinsic_ps` | The scheduler's process rows |
| Quit by scope | `Runtime::Impl::quit_app` in [ariadne.cpp](../../src/cvc/ariadne/ariadne.cpp) | The Runtime already knows each action's scope (`live_actions_`) and each mount's tick |
| `requires:` and `(has? ...)` | §5.7, [requires.h](../../inc/cvc/ariadne/requires.h) | A document lists what it needs, each entry required or `optional: true`, and checks at run time with `(has? NAME)` |
| `link:` holes, `needs:` | §7.4 | Read-only or read-write access to a parent's subtree, granted per name, torn down and planted again at every reconcile |
| The `settings:` widget | §4, `settings_emit.h` | One widget edits every key under one or more state subtrees, each key's editor chosen from its metadata |

## 3. Gaps

1. **`load:` is fixed when the document is parsed.** Its URI is a literal string, and the loader
   expands it before the document runs. There is no way to mount a document at run time, and no
   way to start one quit. A shell that listed every demo as a `load:` would load all of them at
   start and run every `init:`.
2. **A loaded document loses its scene.** `parse_document` (loader.cpp), which parses a mounted
   fragment, reads only `menubar:`, `windows:`, `overlays:` and `root:`. A document's `scene:`,
   `scenes:`, `data:`, `assets:` and `keys:` are read for the outermost document alone
   (`r.scene = parse_scene(...)` in the top-level load). A fragment's are dropped without a
   warning. §11 lists this as "sub-scene mounting not yet parsed". Most example documents are
   scenes, so mounted they show only their windows.
3. **A loaded document gets no input, no signals and no command line.** Its `on_key:`,
   `on_pointer:`, `on_signal:` and `args:` are ignored with a warning (§6.3), and its `keys:` is
   not read. The reason holds for input: a module that saw every key could read what is typed
   into the parent's fields. But there is no way for a parent to hand any of it over, even to a
   document it trusts.
4. **A loaded document cannot add to its parent.** It has its own menu in the bar and its tray
   icons join the tray (§5.25), and that is all. It cannot put an item in the parent's File
   menu, a page in its Settings window, or an item in a right-click menu, and it cannot change
   the parent's settings except through a `link:` hole the parent wrote for it by name. Ariadne
   had no right-click menus at all: ImGui's were used only inside two native widgets (the text
   view's Copy, the transfer function editor). Text has had one since transfix/libcvc#1476
   (Cut, Copy, Paste, ARIADNE.md §5.28), and SH8 lets a document add its own (§5.30).
5. **One main view, one world clock.** An `AriRuntime` has one main view and one `SceneAdapter`,
   and `AppRuntime::load()` clears the last document's scene before realizing the next. A scene's
   world follows `cvc::app::world_clock()`, one per app (`AppRuntime::restart_world` resets it).
6. **A `(ps)` row does not say whose process it is.** It holds `pid`, `name`, `status`,
   `priority`, `uid` and `steps`. Nothing in it names the document or mount the process runs
   for, though the Runtime knows each action's scope.
7. **Views are drawn under the UI, `volren` only in the main view, and volumes barely restyle**
   (§5.18). A window behind a view's window still draws over the view. A `volren` node is left
   out of every view but the main one. A view can scale a `type: volume` node's opacity, at the
   cost of uploading the volume again, and cannot restyle a `volslice` node at all. Scene windows
   need all three fixed (4.4).
8. **The network, its credentials and the end-to-end keys are the host's, not each
   document's**, and a mounted document may call every host verb. Harmless while every document
   is the launcher's own; not once one comes from a link (4.12).
9. **Mounts share their document's frame budget.** In group 0 that is the app's; under
   `isolate_documents` it is the document's group's 200000 steps and 0.1 s (§2.5). Every open
   application competes for it.

## 4. Design

### 4.1 The shell document

`src/cvc/ariadne/components/shell.ari`, installed with the other components and addressed as
`cvc://components/shell.ari`. It takes `--catalog <uri>` (`args:`, §5.6), a catalog in the format
arigl's gallery already reads (`{"demos": [{slug, title, desc, doc}]}`). `arigl --shell` passes
the installed examples catalog, `share/libcvc-examples/gallery.json`.

```yaml
meta: { name: Shell }
args:
  catalog: { type: string, required: true, help: The catalog of documents (gallery.json) }
data:
  catalog: { uri: { arg: catalog }, as: json }   # §5.17; a uri: from an arg is new (SH1)
front: shell.front                               # 4.4: which application owns the main view
menubar:
  - menu: Apps                                   # Open <title>, one item per catalog entry
    items: [...]                                 # the Windows menu, tray and clock come as usual
windows:
  - window: Launcher                             # the catalog as a table, Open buttons
    children: [...]
root:
  - load: cvc://components/processes.ari         # 4.7
    as: processes
  - mounts: shell.apps                           # 4.2: one isolated mount per entry
    allow: { bind: shell.allow }                 # empty: any file or link (4.2, 4.13)
```

The shell is one document. The applications are mounts in it, not documents of their own: they
share its Runtime, its scheduler and its owner.

### 4.2 On-demand mounts: `mounts:`

A new widget, `mounts: <state key>`, holds one mount per child of that key:

```
shell.apps.<as>.uri          the document to mount
shell.apps.<as>.args.<name>  its args: substitutions, as load:'s args: are
shell.apps.<as>.argv         its command line, parsed by its own args: block (4.6)
shell.apps.<as>.grants       what it may take over or add to (4.5)
shell.apps.<as>.view         window (the default) or main: where its scene is drawn (4.4)
shell.apps.<as>.background   run (the default) or hold (4.4)
shell.apps.<as>.budget.*     its budget, when the user changed the default (4.11)
```

- **Mounting.** When a child appears, the Runtime asks its host's loader for the fragment. The
  loader resolves and parses it as `expand_load` does: the same URI resolver, the cycle, depth
  (32) and total (256) guards, the component paths. The fragment is mounted at `includes.<as>`,
  exactly as a static `load:` with `as: <as>` would be: its `init:` run, its `on_tick:` started.
  `AppRuntime` gives the Runtime that loader as a callback, so the loader stays free of state.
- **Closing.** When the child goes, the mount is quit (`quit_app`), its widgets, scene, world,
  datasets and caches come down, and it leaves the tree, so a closed application costs no memory
  or compute beyond its state. Its state stays under `includes.<as>` until the shell drops it, so
  reopening the same id resumes it, as Windows > Applications does today.
- **Caching.** A parsed fragment is kept per URI, args and file stamp. A reload
  (`AriRuntime::reload_if_changed`) parses it again, and a mount whose program did not change
  keeps its resident, as a static `load:` does (§7.4).
- **What may be mounted.** By default, any local file or URL, as a browser opens any page: what
  is not in the catalog runs sandboxed as its own principal (4.12). `allow:` narrows it to URI
  prefixes, literally or from a state key (`allow: { bind: <key> }`), and the shell's *Open: only
  catalog applications* setting (4.13) narrows it to the catalog. A URI outside them is refused,
  and the refusal is posted to the feed. The fragment's `requires:` goes through the scan
  `check_host_requirements` runs over a static load's modules. Where that scan warns, a `mounts:`
  entry is refused instead, with the need named (`gl.scene` under `ari`, say).
- **Grants.** A `mounts:` application gets no `link:` holes and no functions of the shell's.
  What it may reach beyond its own scope is what it asks for and the shell grants (4.5): the
  shell's input, signals, menus, right-click menus and Settings window, and the settings the
  shell shares (4.8). A later version may take `link:` and `functions:` per entry too.
- **Verbs.** `(app-open URI [AS [ARGS]])` writes the entry and returns its id. `(app-close AS)`
  removes it. `(app-view AS window|main)` moves its scene; `main` writes the `front:` key. They
  act on the document that calls them: a mounted application cannot open others unless the shell
  grants it the functions.
- **As built in SH1** (ARIADNE.md §6.8). `Kind::Mounts` reads `mounts:`, `allow:` and a third
  key, `catalog:` (a list, or `{ bind: <key> }`), which names the documents that run as `local`:
  the runtime takes the principal from where the bytes came (4.12), and the slot is where it
  learns which local files are the shell's catalog. An entry's own `catalog` flag does the same
  for that entry: `shell.ari` sets it on what it opens from its catalog, and `catalog:` covers the
  catalog's documents opened any other way (both, at Joe's choice, 2026-11-27). `load_mount` (loader.h) runs `expand_load`
  in a fresh context, stateless; `AppRuntime` hands the Runtime a loader that adds its component
  paths and the `requires:` check (`Runtime::set_mount_loader`). The Runtime keeps each mount in
  a slot by sub-prefix, reconciles the slots at the top of each frame (open, close, parse again)
  and plans the Windows menu over them, so a mount's menu bar, windows and tray icons are an
  application's (4.9: one without a bar gets a menu of Restart, Quit and Close). A mount's
  `state:` is written, its `init:` runs and its `on_tick:` starts as it opens, not when first
  drawn. Closing quits it,
  kills every process under its scope (forks and spawns too) and frees the slot; `ran_inits`
  keeps it, so reopening resumes. The `requires:` check refuses what the static scan would fail
  or warn on: a required entry, a function a program calls, `gl.scene` for a `scene:` on a host
  without one; an optional entry stays a warning. The entry gets `status`, `error`, `principal`
  and `name` back. `(app-open)` without an id takes the URI's file name and returns an entry
  already open on that URI; the verbs read the slot's key when called (it is set with the tree,
  so a resident started before the first frame finds it). The shell (`components/shell.ari`)
  takes `--open` besides `--catalog`, to open entries at start; its Apps menu lists the catalog
  by title through a `menu_item:` label that is an expression. `background` is read since SH3
  (4.4); `view`, `grants` and `argv` are not read yet (SH6, SH7).

### 4.3 Scenes of mounted documents

A mounted document keeps its scene, for `mounts:` and for a static `load:` alike. This closes
the §11 caveat.

- **Loader.** `expand_load` parses the fragment's `scene:`, `scenes:`, `data:`, `assets:` and
  `keys:` and carries them on the mount, as it carries `init:` and `on_tick:` now.
- **Realizing.** `SceneAdapter` gets `realize_mount(id, scene, scenes, sub_prefix)` and
  `clear_mount(id)`. Each mount's scene is a SceneGraph of its own at the mount's sub-prefix, so
  its nodes are at `<sub>.graphics.root.children.<node>` and its physics key is beside them. Its
  `scenes:` are at `<sub>.scenes.<id>`. Its `data:` loads under the sub-prefix. Every relative
  bind in the fragment, widget or scene, resolves as it did when the document ran alone.
- **Drawing.** Each mount's scene has an anchor viewport of its own, as a `scenes:` entry has
  today: the scene's own renderer, with its lights, background and shadow chain. Its scene window
  is that renderer drawing into the window's texture, and the main view, when the application is
  in front, is that renderer drawing into the window's frame (4.4). A `viewport` widget with
  `scene: { mount: <as> }` is a further view of it.
- **As built in SH3** (ARIADNE.md §5.18, §6.3, §6.8). `parse_scene_blocks` (loader.cpp) parses a
  document's and a fragment's scene blocks alike; `expand_load` keeps the fragment's on the mount
  as `Widget::mount_scene` (`MountScene`, mount_scene.h: its `scene`, `scenes`, `data`, `keys`,
  the load:'s `view` and `in_background`), and adds its scene window to the fragment's tree: a
  window `ariadne_scene` titled by its name, with one `viewport` (`scene_window: true`,
  `fill: true`), shown while `<sub>.ariadne.scene` is `1`. The Runtime binds the mount's
  datasets at its sub-prefix before its `init:` (a `load:` as it is first drawn, a `mounts:`
  entry as it opens) and lists the mounts with a scene, datasets or keys
  (`Runtime::scene_mounts`: every `load:` not under `repeat:` and every open entry, less the
  quit applications, each with its `hold`). `AppRuntime` realizes each one's scene from that
  list once a frame (`SceneAdapter::realize_mount(id, scene, scenes, sub_prefix)`, the id its
  sub-prefix), takes it down when it leaves the list or is parsed again (`clear_mount`), and
  tells the adapter which worlds hold (`hold_mount`). cvcGL's adapter gives each a SceneGraph at
  the sub-prefix on a viewport of its own, `mount:<sub>`, whose camera is
  `<sub>.viewers.main.camera` (`ViewportManager::addSceneViewport`'s new `viewer`), framed on the
  scene as the main view is; its `scenes:` get theirs (`mount:<sub>|scene.<id>`, state at
  `<sub>.scenes.<id>`). `SceneViews` resolves a viewport's scene by the scope it is drawn in: a
  mounted document's viewports find their mount's scene, `scene: { mount: <as> }` the scene at
  `includes.<as>`, and the scene window's view is the scene's own viewport drawn into the
  window's texture (no mirror). Not built: a mounted scene's `world:` block (vehicles), its keys
  (SH7), the main view (SH6), a repeated `load:`'s scene, and a world for a mounted scene on a
  host without a scene adapter (`ari`: a `mounts:` entry with a scene is refused there, naming
  `gl.scene`, and a `load:`'s is warned about).

### 4.4 Where an application's scene is drawn: its window or the main view

Both are supported. An application with a scene opens in a window, and can be promoted to the
main view. Its view is `window` (the default) or `main` (`shell.apps.<as>.view`; `view:` on a
static `load:`), and the user can switch it while it runs: its menu has *Show in the main view*
or *Show in a window*, and `(app-view AS window|main)` does the same from the shell's programs.
At most one application is in the main view at a time, the *front* application; the outermost
document's top-level `front: <state key>` names it. Putting another application in the main view
sends the one there back to its window.

**An application may ask for the main view**, as a desktop application or a game asks to take
over the display. It asks for the token `request.main_view` (4.5), and `(app-request-view
"main")` from its own programs puts the request to the shell. The shell grants it, or puts it to
the user (*FE Lab asks to take over the main view: Allow / Not now*), as its `grants:` and the
user's settings say; a refusal leaves it in its window. The user can always send it back, and
Escape held for a second does so whatever the application binds, so no application can keep the
main view against the user. With `ui.fullscreen` granted as well, `(fullscreen #t)` (§10.7) then
makes the main view cover the screen.

**The same scene, the same camera.** Either way the application's scene is drawn from
`<sub>.viewers.main.camera`, so a switch keeps its view, and a program that steers the camera
(`viewers.main.camera.track.*`, say) works in both. A switch moves where the scene is drawn and
nothing else: the SceneGraph, its world and its bodies stay as they are.

**In a window** (the default):

- The shell gives the application a window titled by its name. The window shows the application's
  scene drawn by that scene's own renderer, from `<sub>.viewers.main.camera`, into a texture the
  window draws as its picture. It is the scene's own view, not a mirror of another one, so it
  draws whatever the main view can: volumes of every kind, shadows focused on its own camera, its
  HUD. The application's own windows float as usual. Maximize (§5.9) fills the screen under the
  menu bar, which looks like the main view without taking it.
- Windows stack over it as they do over any window: one in front covers it, one behind does not
  (*Views drawn into textures*, below).
- Several applications' scenes show side by side, and the main view keeps the shell's own scene
  (or none).
- The camera modes are the main view's (orbit, fly, map, track). Fly's move keys reach the
  application only with `delegate.keys` and the keyboard focus in its window (4.6).
- A `viewport` widget with `scene: { mount: <as> }` is another view of the same scene: the
  Launcher's live thumbnails, or a second angle.

**In the main view:**

- The main view's camera reads and writes the front application's `<sub>.viewers.main.camera`.
  Its background, lights, shadows, chrome and HUD are the front scene's.
- One scene in the main view, never two: the outermost document's `scene:` is not drawn under
  the front application's. When `front:` is empty or absent, the main view draws the outermost
  document's own `scene:`, as today.
- **How to draw it** is for a spike in SH6. Since a scene window is its scene's own renderer,
  the natural way is the same here: the main view draws the front scene's SceneGraph from its own
  renderer, into the window's frame instead of a texture, with the main camera controller pointed
  at the front scene's camera keys. The other way is for the main viewport to mirror the front
  scene's anchor viewport (`ViewportManager::addMirrorViewport`), which would bring back the
  mirror limits below.

**Either way:**

- arigl's Settings items (shadows, grid, Controls...) act on the scene that has the focus: the
  main view's, or the scene window's. Their `visible_when:` reads that mount's `ariadne.scene`.
- **Background applications run, each at its own rate.** An application's world keeps stepping
  while its scene is drawn nowhere, at its scene's own rate (`scene: clock: hz`), or at the
  shell's rate when it names none. That needs a world clock per scene: today an app has one
  (`cvc::app::world_clock()`, gap 5), so each mount's world gets a clock of its own, which the
  shell's frame drives and which follows the shell's clock for its rate when the scene sets none
  (SH3, with the world-clock API's owners: WORLD-CLOCK-SYNC-ROADMAP.md).
- **Holding them is the user's choice.** The shell's Settings have *Hold background
  applications' worlds* (4.13), off by default, and an application's *Permissions...* can hold
  just that one (`shell.apps.<as>.background: hold`). A held world stops stepping while its scene
  is drawn nowhere (not in the main view, and its window closed or collapsed) and goes on when it
  is drawn again. Its `on_tick:` and actions keep running; Quit is what stops them.
- **As built in SH3.** The world clocks are the app's, by state prefix
  (`cvc/core/scene_clocks.h`: `add_scene_clock`, `remove_scene_clock`, and
  `scene_clock(app, prefix)`, the clock of the scene at a prefix or the nearest one above it,
  else `app.world_clock()`). A mounted scene's clock is made before its world, with its
  `clock: hz` quantum or the quantum of the clock above it, and its scene drives it each frame
  (`RealizedScene::scene_world_clock`, a `SceneClockDriver` and `SceneWorld::build` given that
  clock), so its physics, animations and `<sub>.sim.*` keys are its own; its `scenes:` read it and
  do not drive it. Node types look the clock up by their bind prefix (cvcGL's water, weather,
  L-system, world, swarm and vehicle nodes do); a `nav_swarm` node's navigator
  (`scene_swarm.cpp`) still reads `app.world_clock()`. The shell's setting is the slot's
  `in_background:` (`shell.ari` binds it to `shell.settings.hold_background`, with no page for it
  until SH14); an entry's `background` wins over it, and a `load:` takes `in_background:` (its
  `background:` is the backdrop every widget takes). Drawn nowhere means no view of the scene was
  drawn in the last frame's walk (`SceneViews::scene_drawn`): its window closed, collapsed or
  hidden, and no further view drawn. A held world's clock runs no time (`SceneClockDriver::hold`),
  so its bodies and animations stand still.

**Views drawn into textures (SH2).** Today a view (§5.18) is drawn into the window's frame under
all of the UI, and its window gets a hole cut in its background so the view shows through
(`ImGuiBackend::cut_window_background`). ImGui draws every window after the scene, so any window
over the view's rect covers it, whichever of the two is in front: §5.18's "another window behind
the viewport's window still draws over the view". With application windows that is not
acceptable, and the fix is for every view, not only the shell's:

- Each view renders into a texture of its own: a `vtkRenderer` drawing into an FBO-attached
  `vtkTextureObject`, the pattern the ocean's `vtkOpenGLFramebufferObject` passes use, and the
  GPU tier [CVCGL-UI-DSL-ROADMAP.md](CVCGL-UI-DSL-ROADMAP.md) §9.9.13 (b) planned for
  render-to-texture. The texture is the size of the widget's rect.
- The widget draws the texture as an image in its window, so the window stacks, clips and
  scrolls the view as it does any of its content. No hole is cut.
- That also ends two more of §5.18's limits: the view lying under the whole UI, and its opaque
  first frame (the hole came a frame late).
- Input is unchanged: the widget's rect takes the drags, the wheel and the picks.
- The main view is still drawn into the window's own frame, under everything, which is where it
  belongs.

**Volumes in every view (SH4).** Of cvcGL's three volume renderers, two already draw in views:

- `type: volume` (`VolumeNode`, VTK's own GPU raycaster) draws in any renderer, and a view can
  restyle its opacity and move it.
- `volslice` (`VolSliceNode`) sweeps slices of its own for each view, facing that view's camera
  (`VolSliceNode::viewProp(renderer)`). A view can move it but not restyle it.
- `volren` (`VolRenNode`, the `cvc::volren` CPU or CUDA raycaster) does not. It raycasts one
  picture and depth map for one renderer's camera (`m_renderer->GetActiveCamera()` in
  [VolRenNode.cpp](../../src/cvcGL/VolRenNode.cpp)) and draws them on a quad posed across that
  camera's frustum. From another camera the quad would show skewed, so the mirror filter leaves
  the node out (`VolumeKind::raycast` in
  [scene_views.cpp](../../src/cvcGL/ariadne/scene_views.cpp)).
  Nothing in the GPU or in VTK forces this: the node was written for one camera.

Drawing views into textures does not fix `volren` by itself, since its picture belongs to one
camera. Two changes do:

- A scene window is its scene's own renderer, so a `volren` node in an application's scene draws
  there exactly as it draws in the main view today.
- For the other views of a scene (a viewport widget, a thumbnail, a second angle), `VolRenNode`
  raycasts once per renderer that shows it, as `VolSliceNode` sweeps once per view: each with its
  own camera, picture, depth map and quad, and its own job in flight. Each raycasts again only
  when its camera, the node's transform or the settings change, as the one raycast does now.
  `volren.resolution_scale` and the view's `refresh_hz` bound the cost, and on CUDA the views'
  jobs share the device. Its quad's depth test works in a view's texture as it does in the main
  view's frame.

**Restyling volumes in views (SH5).** A view's `override:` (§5.18) restyles a mesh in that view
only. For volumes it does much less today: a `type: volume` node takes `opacity` and `transform`,
a `volslice` node `transform` alone, and a `volren` node is not in the view at all. With several
applications' scenes, thumbnails and second angles on one volume, a view must be able to show it
its own way. An override entry that covers a volume node, of any of the three kinds, takes:

| Field | On a volume |
|---|---|
| `transfer_function` | the view's own transfer function: `preset:` and the `cvc::tf::model` keys, as a volume node's `transfer_function:` takes them (§5.4), or `{ bind: <path> }` to a model in state, so a `tf_editor` can drive one view's look |
| `color` | every colour point replaced by this colour, the opacity kept: the volume in one hue |
| `opacity` | every opacity point scaled, as `type: volume` does today |
| `transform` | as today |

They apply in that order: the view's transfer function (the node's when the entry sets none),
then `color`, then `opacity`. `drawstyle`, `line_width` and `point_size` mean nothing to a
volume, and a view warns once and ignores them. The main view and every other view keep the
node's own look. How each kind does it:

- **`volren`.** Each view already has its own raycast (above), so its job takes the node's
  `volume_settings` with the view's transfer function, colour and opacity applied, and raycasts
  again when the override changes. The volume is the node's; nothing is copied.
- **`volslice`.** A view's actor (`viewProp`) binds a transfer-function texture of its own, built
  from the view's look, and keeps sharing the node's volume texture, the large one. The uniforms
  that differ per view become the view's: the alpha exponent, and the eye and view direction a
  sample shader reads, which a view prop takes from the node's own renderer today
  (`VolSliceNode.h`). So a custom sample shader that lights the volume lights it right in every
  view, restyled or not.
- **`type: volume`.** Today a restyled view gets a second `vtkVolume` with a second
  `vtkSmartVolumeMapper`, which uploads the whole volume to the GPU again, because a GPU volume
  mapper keeps one set of transfer-function tables, for the property it last drew. With several
  restyled views of a large volume that is not acceptable either. The fix is in VTK, patched in
  cvcpkg's recipe, upstream-ready and opt-in: the GPU raycast mapper keeps its
  transfer-function tables per volume property, so one mapper, and one upload of the volume,
  draws every view's property. `color` and `transfer_function` then come with no more cost than
  `opacity`.

### 4.5 Asks and grants

A mounted document asks for what lies beyond its scope in its `requires:` (§5.7), with the tokens
below. The document that loads it grants them with `grants:`. A token holds in the mounted
document only when it asked and was granted, and `(has? NAME)` tells its programs which ones
hold. `check_host_requirements` answers these tokens from the mount's grants, where it answers
the others from the host.

```yaml
# fe_lab.ari asks
requires:
  - delegate.keys                                 # do not mount me without it
  - {name: extend.context_menu, optional: true}   # use it if granted
  - {name: access.settings, optional: true}

# the document that loads it grants
root:
  - load: fe_lab.ari
    grants: [delegate.keys, extend.context_menu]
```

| Token | Granted, the document | Not granted |
|---|---|---|
| `delegate.keys` | has its `on_key:` and `keys:` run for the keys it has focus for (4.6) | they are ignored with a warning, as today |
| `delegate.pointer` | has its `on_pointer:` run for the pointer over the main view while it is in front, and over its own windows | ignored with a warning |
| `delegate.signals` | has its `on_signal:` run for the signals the parent passes on (4.6); a list (`{delegate.signals: [SIGTERM, SIGHUP]}`) limits which | ignored with a warning |
| `extend.menubar` | may add items to the parent's menus (4.7) | its menus stay in its own application menu (§5.25) |
| `extend.tray` | has its tray icons join the parent's tray | its tray icons are not shown |
| `extend.context_menu` | may add items to the right-click menus the parent opens to it (4.7) | its items are not shown |
| `extend.settings` | adds its settings as a page of the parent's Settings window (4.7) | its page is not shown |
| `access.settings` | reads and writes the settings the parent shares (4.8); `access.settings.read` reads only | they are not reachable |
| `request.main_view` | may ask to be shown in the main view (4.4) | it stays in its window |
| `request.open` | may ask its shell to open a document as the shell's own application (4.12) | it cannot; nicotina copies the link instead |

- **Asking.** A required token that is not granted refuses the mount, naming it, as a missing
  `requires:` entry fails a load. An optional one warns and leaves `(has? ...)` false. A token
  the document did not ask for is never given, whatever the parent grants.
- **Alone.** The outermost document holds every `delegate.*` and `extend.*` token: what it would
  delegate or extend is its own. So a document that asks for them runs alone as before, its
  contributions going to its own menus, tray, right-click menus and Settings window. It holds no
  `access.*` token, since no one shares settings with it, so `(has? "access.settings")` is false.
- **Granting.** `grants:` goes on a `load:` (a list of tokens) and on a `mounts:` slot, where it
  is the most any of the slot's applications may get. An entry's own `shell.apps.<as>.grants`
  picks from that. A mount grants its own mounts only tokens it holds itself, so a grant never
  widens down the tree.
- **The defaults.** With no `grants:`, a mount gets `[extend.tray, extend.settings]`: its tray
  icons, as §5.25 gives a loaded application today, and its own page in the Settings window,
  which edits only its own keys. Its own application menu and its notifications need no token:
  both are its own and named as its own (§5.24, §5.25). The loading document opts out with
  `deny: [extend.tray]` beside or instead of `grants:`, and `grants: []` gives nothing beyond
  the menu and the notifications.
- **Asking the user.** On a `mounts:` slot, `grants: { ask: [...] }` names tokens the shell puts
  to the user when an application that asks for them opens: a dialog that names the application
  and each token in words ("receive keys while it is in front", "change the shell's settings").
  The answer is kept for the run under `shell.grants.<principal>`, and saved with the shell's
  settings when the user ticks *Remember* (4.13).
- **Withdrawing.** Windows > Applications > *<name>* > *Permissions...* lists the tokens the
  application asked for and the ones it holds, and the user can withdraw one. A withdrawn token
  comes off at the next frame: input stops reaching the application, its contributions leave the
  shell's menus and windows, and its settings holes are not planted at the next reconcile, which
  plants every hole afresh (§7.4).

### 4.6 Input, signals and the command line

- **Keys.** The parent's `keys:` bindings match first, and a key one of them binds never reaches
  a mounted document. The parent's `on_key:` sees every key, as today. Then a key goes to at most
  one mounted document holding `delegate.keys`: the front application while the main view has
  focus, or the application whose window (its scene window or another of its own) has focus.
  Ctrl+Tab, a shell binding, moves the focus from one application to the next. No mounted
  document gets a key while a text field outside it has the keyboard, which keeps the reason
  §6.3 gives.
- **An application's key bindings** live at `<sub>.keys.<id>`, its own. The Controls window lists
  them under the application's name, after the parent's, and marks a key the parent's shadows.
- **Pointer.** Events over the main view go to the front application's `on_pointer:`, and events
  over an application's window to its own, when it holds `delegate.pointer`. The parent's
  `on_pointer:` sees them first.
- **Signals.** The outermost document's `on_signal:` still takes a signal over (§5.3). Without
  one, the shell passes signals on to the applications holding `delegate.signals`: an interrupt
  (Ctrl-C, SIGINT) to the application that has the focus, and a request to end (SIGTERM, SIGHUP)
  to every one, so each can save, before the shell quits. With one, the shell's handler passes a
  signal on itself with `(app-signal AS NAME)`. The terminal loop's force-quit on three Ctrl-Cs
  within two seconds (§5.3) stays, so no application can trap the user there; a GL window's close
  button stays the shell's.
- **The command line** needs no token, since the parent chooses what to pass. A `load:` takes
  `argv: [...]`, a `mounts:` entry `shell.apps.<as>.argv`. The mounted document's `args:` is
  parsed against it as a top-level document's is (§5.6), and `argv` and `args.*` are published
  under the sub-prefix before its `init:` runs. With no `argv`, its defaults are published. A
  parse error, or a missing `required:` argument, refuses the mount and posts the usage to the
  feed, where a top-level document would print it and exit 2.

### 4.7 Adding to the shell: menus, right-click menus, Settings

A document says what it adds to the document that loads it in a new top-level block,
`contributes:`. Each part needs its token. Alone, the document adds the same things to its own
menus and windows.

```yaml
contributes:
  menus:                                # extend.menubar
    File:
      - { item: Export STL..., on: (export-stl) }
  context_menus:                        # extend.context_menu
    scene:                              # the right-click menu over an application's scene
      - item: Smooth this mesh
        on: (smooth-mesh (get-attr event "node"))
  settings:                             # extend.settings
    - { page: FE Lab, root: settings }  # its own subtree, as a settings: widget's root
```

- **Whose code runs.** A contributed item runs in the contributing document's scope, with its
  functions, as an application's menu does today (§5.25). It never runs in the parent's.
- **Who added it.** A contributed item carries the application's name: in a menu it goes after
  the parent's own items, under a separator and the name; in the Settings window its page is
  titled by it. An application cannot pass its item off as one of the shell's.
- **Menus** are matched by title. An item for a menu the parent has not got goes in the
  application's own menu.
- **Tray icons** stay where they are written today, as `tray_icon:` entries in the document's
  `menubar:` (§5.22). They join the parent's tray only with `extend.tray`.
- **Settings pages.** The parent's Settings window is a `settings:` widget (§4). Each granted
  page adds its `root:`, resolved in the contributing document's scope, as one more group titled
  by the page. The keys are the application's own, so a page grants nothing over the parent's
  state.

**Right-click menus** do not exist yet (gap 4), so they come first, as an Ariadne feature of
their own:

- `context_menu:` on a window, a group, a `viewport`, or the document's top level (the main view)
  holds items as a menu's `items:` do. A right-click opens it, and so do a long press on a touch
  screen and the Menu key or Shift+F10, which is how the terminal backends open it.
- Its items get an `event` dict: where it was opened and the widget's id, and over a scene view
  the fields a `viewport` with `input: pick` writes (§5.18: `node`, `hit`, `x`, `y`, `z`, a
  slice's voxel), plus the node's `tags`, which a pick does not carry today. An item's
  `visible_when:` may read `event`, so an item shows only over the nodes it acts on.
- `context_menu: { id: <name>, items: [...] }` opens the menu to granted documents, which add to
  it with `contributes: context_menus: <name>`. A menu without an `id` takes items from no one.
  The shell names two: `scene`, over an application's scene wherever it is drawn (the main view
  or its window), and `app`, an application's row in the Launcher.
  An item added to `scene` gets the pick as values in `event`: it learns which node was clicked,
  and gains no access to the parent's state.
- A new `requires:` token, `ui.context_menu`, says whether the backend opens them. ImGui opens
  them on a right-click (`BeginPopupContextItem`, as the text view's Copy uses
  `BeginPopupContextWindow`), FTXUI and tvision on the key.
- Built in SH8 as ARIADNE.md §5.30, in the one context menu transfix/libcvc#1476 gave every
  backend (§5.28): a widget's own entries (a field's Cut, Copy, Paste) first, then under a rule
  the innermost `context_menu:` around it, whose items alone are the menu over anything without
  entries of its own. In ImGui a right-drag over such a thing (a pan in a view, the main view's
  camera) opens nothing; tvision shows the menu as a popup of its own, so a short window does not
  cut it off. `contributes:` (SH9) is not built: the `id` is kept.

### 4.8 Changing the shell's settings: `shares:` and `access.settings`

The shell names the settings an application may change in a new top-level block, `shares:`:

```yaml
shares:
  settings:
    theme: shell.settings.theme
    bell: ariadne.tray.bell
    menu_collapse: shell.settings.menu_collapse
```

- An application holding `access.settings` finds each at `host.settings.<name>` in its own scope:
  a read-write `link:` hole the runtime plants, exactly like one a parent writes by hand (§7.4).
  With `access.settings.read` the holes are read-only.
- A grant may narrow the names: `grants: [{access.settings: [theme]}]`.
- The shell's keys outside `shares:` stay out of reach. A hole grants a subtree (§7.4), so the
  shell shares leaves or small subtrees, never its whole state.
- The shell's Settings window edits the same keys, so a change an application makes shows there
  at once, and the user can put it back.

### 4.9 The application list

The shell adds to §5.25's applications:

- every `mounts:` entry is an application, with or without a `menubar:` of its own (§5.25 makes
  one only of a mount with a bar). One without a bar gets a menu with its commands alone;
- the menu of the application that has the focus first among the applications, marked;
- *Show in the main view* or *Show in a window* (4.4), *Permissions...* (4.5) and, for a
  `mounts:` application, *Close* beside Quit and Restart;
- a mirror of the application list in the state tree, under the outermost document's prefix:
  `ariadne.apps.<as>.{name, scope, running, front, asked, grants}`, as the feed mirrors itself to
  `notify.*`. The Launcher, the Permissions window and the Processes view read it from there.

**The Apps menu** replaces arigl's Demos menu (gallery mode, §9) and lists two kinds of
application, each with its icon and its name:

- **The catalog's**, the shell's own list (4.1): every entry is listed from the start, and none is
  loaded until the user picks it. Picking a running one brings it to the front. Its menu has
  *Close*, which frees it (4.2), and the entry stays in the list, ready to load again.
- **Opened from elsewhere**: a link from a nicotina post, a file the user opened, a document
  another application asked for (`request.open`). They are listed under a separator while they
  run, and they load and close the same way. They are not in the catalog and need not be, and
  closing one takes it off the menu.

**Application icons.** A document names its icon at several sizes in `meta.icons:`, as a web
page names its favicons:

```yaml
meta:
  name: FE Lab
  icons:
    - { src: icons/fe-16.png, size: 16 }
    - { src: icons/fe-32.png, size: 32 }
    - { src: icons/fe-128.png, size: 128 }
```

- `src` is relative to the document, and a pack (4.12) carries the files, so the shell reads an
  application's icons without running any of it, from the catalog's documents and through
  `(app-info URI)` for the rest.
- The document hosting it chooses how to show an icon and at what size. The shell takes the
  smallest icon at least as large as it needs (the menu's line height times the display's scale,
  or a Launcher tile) and scales it down to fit; when none is that large, it scales up the
  largest. A document with no icons gets one drawn from its name's first letter.
- A menu item takes `icon:` for this (new; a button takes one today, §5.27). A backend that
  draws no pictures (`button.icon` false: the terminals without kitty or Sixel) shows the name
  alone.

### 4.10 The Processes view

- **Rows say whose process it is.** `(ps)` and `(inspect)` rows get `scope`: the process's chroot
  relative to its owning document's prefix. It is `""` for the document's own programs and
  `includes.fe` for a mount's. Runtime-started processes already carry their scope
  (`live_actions_`, the mounts' ticks). `process_info` gets it from the process's context. The
  threaded plan's published rows (state-exec-perf S2-Q9) carry it too.
- **`components/processes.ari`** is a window with a table: application (its `scope` looked up in
  `ariadne.apps`), pid, name, status, steps, what it waits on, memory, and a Kill button. Its
  `on_tick:` polls `(ps)` twice a second into its own state, with a cap on the rows. The outermost
  document's rows are named by its `meta.name`.
- **What it sees.** It runs on its document's scheduler, so it lists that document's processes,
  every mount's included. Since the threaded plan's S2b a process sees only its owner's, in group
  0 too; the shell's mounts share the shell's owner, so the view keeps them. Another document's
  processes in another execution group come in with S2c, which merges the owner's rows across
  groups.
- **As built in SH0.** `execute_options::owner_root` and `process::owner_root` hold the owner's
  root; `process_scope` writes `scope` from it and the chroot, which the Runtime now records in
  `process::root_path` for every process it starts. Spawned and forked children keep both, the
  cluster's process rows carry `scope`, and `state_value_codec` carries the owner root. The mirror
  has `name`, `scope` and `running` (`front`, `asked` and `grants` come with SH6 and SH7); its key
  is the scope without its `includes` segments, joined by `_` (`fe`, a repeated mount's `cam_0`),
  and the document's own name is mirrored to `ariadne.name`, from the root widget's `app_name`,
  which the loader sets to `meta.name`. `processes.ari` is a component like the others in
  ARIADNE.md §6.5: a document imports it, includes the window and calls `(processes-step ROWS)`
  from its `on_tick:`.

### 4.11 Budgets

Every application gets a budget of its own from the first mounts (SH1), so a runaway one cannot
stall the shell, local or not:

- **What it bounds:** the steps and the time its programs may use in a frame, and the memory they
  may hold, as the document's lanes bound one document today (`ariadne.limits.*`), counted over
  every process whose scope is under the mount.
- **Defaults by principal**: generous for the catalog's applications, tighter for a document
  opened from a file or the network (4.12). The numbers are settled in SH1 against the examples:
  none of them may hit its default.
- **The user's control:** each application's *Permissions...* has its budget, kept in
  `shell.apps.<as>.budget.*` and saved with the shell's settings (4.13).
- **When an application reaches it,** its processes wait for the next frame, as a document's do
  when they use up its share, and it is not killed. When it stays at its limit for several
  seconds, the shell posts *FE Lab is using all the time it is allowed*, with *Allow more*,
  *Quit* and *Restart*. The Processes view shows each application's share.

A mount in an execution group of its own comes later; it needs processes that can move between
groups (threaded execution Stage 3).

**As built in SH1** (ARIADNE.md §6.8). The scheduler holds a mount to its budget:
`cvc::state_exec::frame_account` (process.h) is a share of each frame that the processes given it
draw on together (`execute_options::account`; `spawn` and `fork` keep the parent's). The async
scheduler charges each step's steps and seconds to it, ends a run at its limit, and parks a
process whose account is spent as `(await)` parks it, at its next pick or right after the step that
spent it; `wake_awaiting()`, once a frame, starts a new frame on every account it has seen. The
Runtime gives every process whose scope is under a `mounts:` entry that entry's account
(`make_action_context`, so its actions and its `on_tick:` resident). Memory: state_exec's
per-process state-write tracker is not fed by the state verbs, so a mount's memory is its
processes' measured heap shares, which the scheduler now measures for a process whose account
has a memory cap (`process::measures_heap`), without a limit of its own; the Runtime sums them
four times a second into the account, and past the cap the account is spent. The defaults
(`Runtime::default_mount_budget`): `local` 100000 steps, 0.025 s and 256 MB a frame, `file:` and
`remote:` 20000 steps, 0.005 s and 64 MB. Settled against the shipped examples
(`AriadneMounts.TheShippedExamplesKeepWithinTheLocalBudget` mounts every one as a catalog
application with a stub scene adapter): the busiest uses 108 steps and well under a millisecond
in a frame, so the defaults leave room for documents far heavier than the examples while a
runaway mount takes at most a quarter of the pump's 200000 steps and 0.1 s. The notice comes
after 3 seconds held; its click opens a Budgets window (Allow more, which doubles the entry's
`budget.*`, Quit, Restart), since a feed entry has one action. The mirror carries
`ariadne.apps.<id>.budget.*` and `principal`, and `processes.ari` lists each application's
share. A static `load:` still shares its document's budget.

### 4.12 Documents from the network: the nicotina golden

**The golden.** cy-pca/nicotina, the TX.WTF client written as `.ari` documents, runs as an
application in the shell. Someone posts a link to an `.ari` document. A reader clicks it in
nicotina's feed. The shell fetches the document over https and runs it as an application of its
own beside nicotina, in a scene window, as a browser opens a page: at once, in a sandbox, with a
badge saying where it comes from and whether it is pinned and signed. It asks the reader only
when the document wants more than the sandbox gives. Applications that travel as links are the
start of a software ecosystem, so the shell treats every such document as code from a stranger,
and the user can narrow what it accepts (4.13). That scenario, run end to end, is this section's
acceptance test (SH15).

**What stands in the way today** (checked in the code):

- **The network, its credentials and the end-to-end keys belong to the host, not to a
  document** (`AppRuntime::enable_net`, §10.3). A credential is found by name, so a document
  fetched from a post and mounted beside nicotina could send
  `(http-request "POST" "https://tx.wtf/..." (dict "auth" "txwtf" ...))` and post as the reader.
  The allowlist is shared the same way: it reaches every origin nicotina may.
- A mounted document may call every host verb and function the host registered (§11: the
  provider registry is process-global). Its `data:` paths and scene sources resolve on the local
  disk, and a `python:` block runs CPython (arix, ariglx).
- Only arigl-web fetches a document set from a URL (`doc_fetch.h`, §9.4). A native host has no
  fetcher for one, and nothing checks what is fetched against a hash or a signature.
- nicotina copies any link it does not know (`open-link` in `nicotina.ari`) and never opens one.

**Principals.** Every mount runs as a principal, and every check below keys on it:

- `local`: a document the shell's launcher or catalog named from the local disk or `cvc://`,
  trusted as documents are today;
- `file:<directory>`: a local file the user opened that is not in the catalog, sandboxed as a
  remote document is, with its own directory in place of an origin;
- `remote:<origin>`: a document fetched from a URL, such as `remote:https://apps.example.org`,
  with its publisher when it is signed (below).

The principal is fixed at mount time from where the bytes came; a document cannot name its own.
The shell says which local files are its catalog's: the slot's `catalog:` lists them, or an
entry's `catalog` flag marks it (as built in SH1, 4.2); neither makes a remote document `local`.
What it loads inherits it: a remote document that `load:`s a file from its own origin runs that
file as the same principal and cannot raise it. Below, a *sandboxed* document is one whose
principal is `file:` or `remote:`.

**What a sandboxed document can reach.** Default-deny, as the relay already runs its clients'
programs (`room-run`: the read lane's builtins and a list of allowed names, `server_document.h`)
and as `LaneProcess::allow` does for any program from outside a document:

| Resource | A sandboxed document by default | With a grant (asked for on the sheet) |
|---|---|---|
| Its own state, widgets, windows and scene | yes, under its mount (§7.3) | |
| Another document's state, the shell's | never | the settings the shell shares (`access.settings`, 4.8) |
| Host verbs and functions | only those marked safe for remote principals: the DSL's own, the state verbs, `notify`, the scene's | each other verb by name (`clipboard-set`, `open-url`, `file-pick`, `http-upload`, ...) |
| The network | its own origin, for its files and its API | more origins it names in `requires:` (`{name: net.origin, origins: [...]}`) |
| Credentials | its own, under its principal: its `txwtf` is not nicotina's | never another principal's |
| End-to-end keys, OIDC sign-in | none | its own, under its principal |
| Local files | a `file:` document its own directory; a `remote:` one none, its relative paths resolving in its fetched set; `file://` refused | a file the user picks (`file-pick`) |
| Persistent storage | its own, per principal, with a quota | |
| `python:` blocks, native extensions | refused: the mount fails | never |
| WebAssembly guests (`wasm:`) | yes, sandboxed, under its budget | |
| CPU and memory | a budget of its own per frame and a memory cap (4.11) | |
| Input, menus, tray, right-click menus, Settings | as 4.5 to 4.7 | the tokens of 4.5 |

**Network, credentials and keys per principal (SH11).** The host checks each request against
the principal of the program that makes it; the program's scope tells the runtime which mount it
runs for. The allowlist is per principal: the launcher's (`--net-allow`, `--net-policy`) for
`local` documents, as today; for a `remote:` one its own origin plus what it was granted; for a
`file:` one only what it was granted.
Credentials are kept as (principal, name, origin, token), and a lookup by name finds only the
caller's principal's; the credential file's lines gain the principal. The end-to-end keys and
OIDC sign-ins are split the same way. A local application can also have a policy and a store of
its own: `shell.apps.<as>.net_policy` and `.credential_store`, as `bin/nicotina` passes
`--net-policy fleet.policy --credential-store nicotina` today. This split is needed as soon as the
shell hosts nicotina beside any other application, remote or not.

*As built in SH11:*

- **Who a program runs as** ([`principal.h`](../../inc/cvc/ariadne/principal.h)): a table per
  app of scope to principal. `open_mount` records a mount's principal there (SH1's
  `mount_principal`, with what the shell's entry gives it) before any of its programs is built.
  `close_mount` drops it once the mount's processes are gone, and so does the Runtime's teardown;
  an entry reopened as a new principal (its catalog flag) is recorded again. A scope under several
  runs as the innermost; a `local` one inside a sandboxed one runs as the sandboxed one. Unrecorded scopes, the shell itself among them, are
  `local`. A remote principal is kept in one spelling, `remote:` and the origin as a policy
  writes it.
- **The verbs** (`enable_net`, [`net_host.h`](../../inc/cvc/ariadne/net_host.h)): each program's
  http verbs, `fetch`, `http-upload`, `ws-*`, `net-*`, `oidc-*`, `e2ee-*` and `open-url` are bound,
  as the program is built, to its principal's network (`net_host_access(app, scope)`, from
  `intrinsics_context::root_path`). `local` gets the launcher's, as before. `remote:<origin>` gets
  its origin and `ScopePrincipal::net_origins`; `file:<dir>` only `net_origins`. That list is
  the input from SH7's `net.origin` grants; nothing writes it yet.
- **Credentials** ([`credentials.h`](../../inc/cvc/net/credentials.h)): one store, one file, one
  lock, seen through one view per principal (`CredentialStore::for_principal`). Each view has its
  own entries, bindings (the launcher's policy binds only `local`'s names) and renewers. The file's
  lines are `<principal> <name> <origin> <token>`, and a three-word line loads as `local`.
- **End-to-end keys**: `local` uses the launcher's store, a sandboxed principal one of its own
  in memory. **Sign-ins**: kept per owner, with its policy and store; another principal's are not
  posted to the global `oidc#NAME` channel. **WebSockets**: a handle is its opener's.
  **Pictures, video and Save** with `auth:`: fetched as the widget's principal
  (`ImageRequest::scope`, `PlayerSource::scope`, `MediaSave::scope`).
- **A local application's own network**: `shell.apps.<as>.net_policy`, `.credential_store` and
  `.e2ee_store`, read as the mount opens, for a `local` mount only. A sandboxed mount that names one
  is refused. A policy alone gets an in-memory store, so its `credential` lines do not rebind the
  launcher's names. A policy that does not load, or binds an origin it does not allow, refuses the
  mount (`set_principal_check`). `shell.ari` takes them from a catalog entry's `net_policy`,
  `credential_store` and `e2ee_store` (paths relative to the catalog), and opens such an entry
  under its slug with them and its catalog flag written first, so it opens as `local`.
- **Left for later**: `data:` entries still read over the launcher's allowlist into the one shared
  store (with SH12/SH13's fetching). Keeping a sandboxed program off the other global `#` channels
  is SH12's.

**Untrusted bytes in native readers.** A sandboxed document's datasets, scene sources, pictures
and videos are a stranger's bytes handed to native readers: VTK's, the mesh and image decoders,
the media player. Those are not memory-safe. So a sandboxed document may load only the formats
the shipped documents already load, and anything else is refused with a notification:

- documents: `.ari` and `.aria` (below);
- pictures: PNG and JPEG;
- models: glTF (`.glb`) and OFF;
- volumes: RawIV and MRC;
- contours (`.contour`, `.ser`, `.cvcc`) and VolumeGridRover point classes (`.vgr`);
- JSON, a heightfield's `terrain.json` among it, and PDB molecules;
- video: MP4;
- cvcpkg data packages, which the registry's SHA-256 checks (§5.17).

These readers are fuzzed in the order of their exposure: the media player (FFmpeg) first, then
the image decoders, the glTF and mesh readers, the volume readers, the YAML parser and the zip
reader. Running each sandboxed principal out of the process is the strong answer (SH17). The
browser host, arigl-web, has the browser's sandbox around all of it already.

**Links and fetching (SH13).**

- **Recognising an Ariadne link.** The media type the server sends decides first:
  `application/vnd.cvc.ari+yaml` for a document and `application/vnd.cvc.aria+zip` for an
  archive (the names are settled in SH13, section 6). When the type is generic
  (`application/octet-stream`, `text/plain`, or none), the path's extension decides: `.ari` or
  `.aria`. `(app-open URL)` makes the decision itself, so the application that asks need not
  know (below). `http://` is accepted for `localhost` only, for tests. Other sources come through
  the same fetcher later: `cvcpkg://` (a registry package, checked against the registry's SHA-256
  as a `data:` package is, §5.17) and a relay's world pack (OW5k).
- **A post's application attachment.** Ideally a post says what it links to: an attachment with
  the URL, the pin and the publisher, and the shell's badge and sheet show them. That is a TX.WTF
  change (its post model and API). A naked link in a post's text works too: it opens with a
  warning that it is unpinned, or unsigned.
- **A pin.** A fragment pins the content: `https://apps.example.org/orrery/main.ari#sha256=<hex>`,
  the hash of an `.ari` file, or of an `.aria`'s manifest. A fragment never reaches the server,
  so the link works in any client, and the shell refuses bytes that do not match.
- **Fetching a single `.ari`.** The shell fetches it with `doc_fetch` (§9.4), which arigl-web uses
  today: the document and every file it names relative to itself, from its own origin only. The
  files go in a cache directory per principal and content hash and run from there, as an
  installed set does. The native fetcher is new; the browser keeps its own. A pin on the main
  document does not cover the files it names; an archive does.
- **`.aria` archives.** A zip archive of the original `.ari`, the documents and data it refers to,
  and a manifest, `aria.manifest`, that describes them: the format's version, the entry
  document, the entry's `meta` (name, version, icons), and every file's path, size and SHA-256
  (the fields of OW5k's world-pack index, transfix/libcvc#1470, so one checker serves both). A
  pin and a signature cover the manifest, and the manifest covers every file, so compressing the
  archive again changes neither. `ari pack DIR` writes one. Zip readers are a classic place for
  attacks, so the shell's is strict:
  - the archive holds exactly the manifest's files, the manifest and the signature, nothing else;
  - no absolute path, `..` segment, link, duplicate name, or entry outside the central directory;
  - each file's size is checked before it is inflated and its hash after;
  - the total inflated size and the number of files are capped.
- **Signatures.** An `.aria` may carry `aria.sig`: an Ed25519 signature over the manifest's bytes
  and the id of the key that made it. The key is a signing key of the publisher's own, published
  on their TX.WTF account beside the end-to-end identity key that accounts already publish
  (`identity_key_ed25519` in `GET /e2ee/keys/{username}`, §10.8), never that key itself, so the
  two uses stay apart. The shell shows a verified publisher only when the key is bound to the
  account. A signature by a key the account has rotated out opens with a warning label, *signed
  with a retired key*; the rest of what rotation means is still to be agreed (section 6). An
  unsigned document opens, marked *unsigned*, unless the user turned on *Refuse unsigned
  documents* (4.13).
- **Updates.** A pinned link always runs the same bytes. An unpinned one is fetched again when
  it is opened; if it changed and asks for more than before, the sheet asks again.
- **The cache** is content-addressed, so a pack posted twice is fetched once. It has a size cap,
  and the Launcher lists what it holds.

**Opening, and the sheet (SH14).** By default the shell opens a document from a file or the
network as a browser opens a page: at once, sandboxed as above, with its badge, without asking.
It shows a sheet first, before the document runs any program, its `init:` included, only:

- when the document asks for more than the sandbox and the defaults give: a token, a host verb,
  another network origin;
- when the user turned on *Ask before opening documents from files or the network* (4.13).

A pinned link whose bytes do not match is refused outright, with no sheet to answer. The sheet is
the shell's own window, drawn at the top of the stack (SH2 makes that hold), so no application can
draw over it or imitate it. It shows:

- the document's `meta.name` and description, marked as the document's own words;
- where it comes from: the origin, and whether the link pinned it;
- who published it: the verified publisher, or *unsigned*;
- who asked to open it: "posted by @user, opened from nicotina", from the request (below);
- what it asks for, in words: each token beyond the default, each host verb, each network origin,
  and its size;
- *Allow*; *Open without them*, when everything it asks for beyond the defaults is optional;
  *Cancel*; and for a signed document *Always trust this publisher*, which keeps the answer with
  the shell's settings (4.13), so that the sheet comes back only when the asks grow.

A running remote application carries a badge the shell draws in its window's title and in its
menu: its origin and how far it is trusted (pinned, signed, unsigned). *Permissions...* (4.5)
lists what it holds. Close ends it. Its state and storage stay under its principal until the user
clears them.

**Opening from inside an application: `request.open` (SH14).** nicotina must not mount the
document itself: a mount of nicotina's would be its child, inside its scope. It asks the shell:

- A new token, `request.open`, lets a document ask its shell to open a document as the shell's
  own application. With it, `(app-open URI (dict "from" TEXT))` from a mounted document is not a
  mount. It is a request to the nearest enclosing document with a `mounts:` slot, which applies
  its own `allow:` and settings and opens the document as above, naming the requester on its
  badge and on any sheet ("opened from nicotina"). The new application is the requester's
  sibling: neither reaches the other's state.
- `app-open` is an async call (§10): it holds the calling program until the shell knows what the
  URI is (its media type, then its extension), and returns `{opened kind error}`. `kind` is
  `ari`, `aria` or `other`; for `other` nothing is opened, so the requester treats the link as it
  would any other. That way nicotina need not guess from the URL.
- Without the token, or with no enclosing `mounts:` slot (nicotina run on its own),
  `(has? "request.open")` is false, and nicotina copies the link as it does today. So nicotina
  runs the same in the shell and outside it.
- The shell takes a request only within a second of the requester's own input (a click or a key
  in its window, or a link it drew), as a browser allows a pop-up, and drops it otherwise. The
  second counts to when `app-open` is called, not to when it returns.
- `(app-info URI)`, under the same token, has the shell fetch a document's metadata without
  running anything: `{kind name description icons origin pinned publisher size}`, or why it
  could not.

**nicotina's part** (cy-pca/nicotina, once SH14 is in):

- `open-link` sends a link it does not follow itself (not a profile, hashtag or post of its own
  site) to `(app-open href (dict "from" <the post's URL and author>))` when
  `(has? "request.open")`, and copies it when the answer's `kind` is `other`, or without the
  token, as now.
- `nicotina.ari` asks for `{name: request.open, optional: true}`. The shell's entry for nicotina
  grants it, with nicotina's `fleet.policy` and credential store as its own (SH11).
- A post's application attachment (above), once TX.WTF has one, shows as a chip: the
  application's icon, name and publisher, and whether it is pinned and signed. A plain link may
  show one too, from `(app-info href)`.

**The golden test (SH15)**, one form in each repository:

- **libcvc**, `shell_remote_golden`: a local https server, with a test certificate, serves two
  signed `.aria` archives (a document with a scene, an import and a dataset; one asks for nothing
  beyond the defaults, the other for `delegate.keys`). A fixture feed document stands in for
  nicotina: posts whose links' `on_link` calls `(app-open ...)`. The shell runs the feed with
  `request.open` granted. It checks that:
  - a click on the first opens it at once, with no sheet, as the feed's sibling, principal
    `remote:https://localhost:<port>`, its scene window drawn (an image golden) and its badge
    saying *pinned, signed*; the second shows the sheet first, which the test answers;
  - it cannot read the feed's state or the shell's; its `(http-get ... (dict "auth" "txwtf"))`
    finds no credential while the feed's still works; a request to any origin but its own is
    refused;
  - a busy loop in it keeps to its budget, the shell's frame time holds, and the shell posts its
    notice;
  - an archive served as `application/octet-stream` is still recognised by its `.aria` extension,
    and a link to a plain page comes back as `kind` `other`;
  - each of these is refused: a tampered file under a pinned link; an archive with a `..` path, a
    duplicate entry, or more inflated bytes than its manifest says; a `python:` block; a format
    off the list (with a notification); an unsigned archive while *Refuse unsigned documents* is
    on;
  - an unsigned archive opens marked *unsigned* while that setting is off, and one signed with a
    retired key opens with its warning label;
  - a `request.open` with no click before it is dropped;
  - Close takes it away, and a second click opens it from the cache.
- **nicotina**, `tests/shell.sh`: nicotina runs inside `shell.ari` against `stub_txwtf.py`, with
  a post linking to the same pack, and the test checks the same opening and the same credential
  isolation through nicotina's own card.

### 4.13 The shell's settings, and where they are kept

The shell's Settings window has a page of its own:

| Setting | Default | What it does |
|---|---|---|
| *Hold background applications' worlds* | off | a world stops stepping while its scene is drawn nowhere (4.4) |
| *Open* | any file or link | or *only catalog applications* (4.2, 4.12) |
| *Ask before opening documents from files or the network* | off | the sheet comes before every sandboxed document, not only one that asks for more (4.12) |
| *Refuse unsigned documents* | off | an unsigned document from a file or the network is not opened (4.12) |
| *Applications may ask for the main view* | ask me | or always, or never (4.4) |
| Trusted publishers | none | the publishers the user chose to always trust, each removable (4.12) |
| Each application's *Permissions...* | its defaults | its grants, its budget, and holding its world (4.4, 4.5, 4.11) |

The settings live in the shell's state (`shell.settings.*`) for the run. Saving them across runs
(SH16):

- **Natively**, a file in the user's config directory beside the credential store (§10.3):
  `$XDG_CONFIG_HOME/cvc/shell/<shell's name>.settings`, or `~/.config/cvc/shell/...`, and
  `%LOCALAPPDATA%\cvc\shell\...` on Windows. It holds trust decisions, so it is written as the
  credential file is: owner-only, replaced whole, and refused when another user owns it or others
  can write it.
- **In the browser**, the wasm file system lives in memory and goes with the page, so the file is
  kept in the browser's storage for the page's origin. Cookies would persist too, but a cookie
  rides along with every request to the origin and holds 4 KB, so the shell uses IndexedDB
  (Emscripten's IDBFS mounts it as a directory) or `localStorage`. Either lasts as long as the
  user keeps the site's data.
- An application's own storage (per principal, 4.12) is kept the same way, under its principal.

## 5. Phases

Each phase is one PR with tests. SH0, SH2, SH4 and SH8 do not depend on the others and can go
first or in parallel; SH5 builds on SH4 for `volren`. SH3's scene windows need SH2: they do not
ship drawn under the UI.

| Phase | Content | Tests |
|---|---|---|
| SH0 | `scope` on `(ps)` and `(inspect)` rows; the `ariadne.apps.*` mirror; `components/processes.ari` | runtime: a mount's resident is listed with its scope; Kill from the view; `ari test` on the component |
| SH1 | `mounts:` and the verbs, `allow:`, the `requires:` check at mount; a `data:` entry's `uri: { arg }`; a budget per mount with defaults by principal and the shell's notice (4.11); `shell.ari` with a catalog of the example documents without scenes (`tray_demo`, `json_feed`, `apps/counter`) | runtime and loader: open, close (its memory freed), reopen with state kept, hot reload, refused URI, unmet `requires:`; a busy mount held to its budget while the others run; a tmux run on FTXUI |
| SH2 | Views drawn into textures (4.4): every `viewport` renders into a texture of its own and draws it as an image in its window; no background hole. A cvcGL change every view gains from | goldens of a window over a view and a view's window over another window, each stacking right; a scrolled and a clipped view; the first frame drawn; picks still land |
| SH3 | Scenes of mounted documents, for `load:` and `mounts:`: `realize_mount`, `clear_mount`, `viewport scene: { mount }`; the scene window (`view: window`), its scene's own renderer into a texture; a world clock per scene, at its `clock: hz` or the shell's rate; background worlds running, held by the setting or per application | loader: a fragment's scene is parsed; AriRuntime: two mounts' scenes realized apart, binds resolve under the sub-prefix; two worlds side by side step at their own rates, and one without a rate at the shell's; a held world is not stepped while its window is closed; a golden of two applications' scene windows overlapping, one with a `volren` node |
| SH4 | `volren` in every view (4.4): a raycast per renderer that shows the node, each with its own camera, picture, depth map and quad | `cvcgl_ariadne` goldens of one `volren` node in the main view and two viewports from other cameras, depth-tested against geometry in each; a view that does not change does not raycast again |
| SH5 | Restyling volumes in views (4.4): `transfer_function`, `color` and `opacity` overrides on `volren`, `volslice` and `type: volume` nodes; a `volslice` view prop's own transfer-function texture and per-view uniforms; the VTK patch (cvcpkg recipe, opt-in) that keeps a GPU volume mapper's transfer-function tables per property | goldens of each kind in the main view and in three views (another preset, one colour, half opacity), the main view unchanged; a `type: volume` node restyled in two views uploads its volume once; a `volslice` sample shader lit right in a view |
| SH6 | `view: main` and `front:`: the main view draws the front application's scene; promotion and demotion; `request.main_view` and `(app-request-view "main")`, granted, asked or refused per the setting; Escape held sends it back; Settings follow the focused scene | switch to the main view and back, camera and world kept; the one in the main view goes back to its window when another takes it; a request without the token, or refused, leaves it in its window; golden |
| SH7 | Asks and grants (4.5): the tokens in `requires:`, `grants:` on `load:` and `mounts:`, the default; keys, pointer and signals to granted applications; `argv:` (4.6) | loader and runtime: a required token not granted refuses the mount; a key reaches only the focused granted application, and none while a parent's field has the keyboard; SIGTERM reaches every granted application; `argv` parsed, a bad one refused with the usage posted; one document's `ari test` alone and mounted |
| SH8 | Right-click menus (4.7): `context_menu:`, its `id:`, the `event` dict and the pick over a view, `ui.context_menu`; ImGui, FTXUI and tvision | runtime: an item runs with its `event`; an ImGui golden of an open menu; a tmux run on FTXUI |
| SH9 | `contributes:` (menus, right-click menus, Settings pages) with their tokens; `shares:` and `access.settings`; `grants: { ask }` and Permissions... | an item added to File runs in the application's scope; a withdrawn token takes its items away at the next frame; `access.settings` writes the shell's key and `access.settings.read` cannot; a document alone adds the same items to its own menus |
| SH10 | The shell replaces arigl's gallery mode: `arigl --shell [catalog]`, and `CVC_ARIGL_GALLERY` opening the shell on its catalog; the Apps menu, catalog entries loaded when picked, applications opened from elsewhere under a separator; `meta.icons` and a menu item's `icon:` (4.9). The browser gallery follows | an arigl smoke test that opens three examples from the Apps menu, closes one and opens it again; a golden of the menu with icons; FTXUI shows names alone |
| SH11 | The network, credentials, end-to-end keys and OIDC sign-ins per principal (4.12); an application's own `net_policy` and `credential_store` | a credential captured by one mount is not found from another; each mount's allowlist holds; the credential file's new lines; nicotina in the shell still signs in and posts |
| SH12 | The sandbox for `file:` and `remote:` principals (4.12): host verbs default-deny, with a list per principal; `python:` and local files outside its own refused; the formats the shipped documents load and no others; tighter budget defaults and a memory cap (4.11); the readers' fuzzing started in the order of 4.12 | runtime: each refusal, named; a busy sandboxed mount keeps to its budget and the shell's frame holds |
| SH13 | Fetching into the shell (4.12): recognising a link by media type, then extension; `doc_fetch` natively into a cache per principal and hash; `#sha256=` pins; `.aria` archives (`aria.manifest` with the world-pack index's fields, the strict zip reader, `ari pack`); `aria.sig` signatures with a signing key of the publisher's own | a pinned set verifies and a tampered file is refused; each zip rule refuses its case; a signature verifies, or is reported missing, wrong or by a retired key; a second open comes from the cache |
| SH14 | Opening (4.12): at once in the sandbox, the sheet only for asks beyond it or when the setting asks; the badge; remembered publishers; `request.open` and `app-open`'s `kind`, taken only after the requester's own input; the shell's settings page (4.13), kept for the run | a document that asks for nothing opens with no sheet; one that asks shows it before any program runs; Cancel runs nothing; *Refuse unsigned* and *only catalog applications* refuse; a request with no input before it is dropped; the opened application is the requester's sibling |
| SH15 | The golden (4.12): `shell_remote_golden` in libcvc; `open-link` and `tests/shell.sh` in cy-pca/nicotina | the checks 4.12 lists, in both repositories |
| SH16 | The shell's settings saved across runs (4.13): the owner-only file natively, IndexedDB or `localStorage` in the browser; trusted publishers, remembered grants, budgets | settings survive a restart; a file another user can write is refused; the browser keeps them across a reload |
| SH17 | Sandboxed principals out of the process: each `remote:` (and `file:`) principal in a process of its own, its DSL processes moved there with state_exec's migration (pause, serialize, send, resume; STATE_EXEC_DEVELOPER_GUIDE.md section 9) and the arisrv state_exec work under way, talking to the shell over a state transport. Needs a design of its own | to be set by its design |

SH11 comes before SH12 to SH15, and before the shell hosts nicotina beside any other
application. SH13 needs SH1; SH14 needs SH7 and SH13; SH15 needs all of them and SH3. SH16 can
follow SH14 at any time. SH17 is on the roadmap: the in-process sandbox is enough to ship the
golden natively (decision 13).

The closed `.ari` schema (transfix/libcvc#1464, open) learns the new keys (`mounts:`, `front:`,
`grants:`, `argv:`, `contributes:`, `shares:`, `context_menu:`, and the `net.origin` form of a
`requires:` entry) in the phase that adds each.

## 6. Decisions and open questions

**Decided (Joe, 2026-11-23):**

1. Background applications' worlds run by default, each at its own rate, or at the shell's when
   it names none. A setting holds them (4.4, 4.13).
2. The shell replaces arigl's gallery mode, and later the browser gallery. The catalog is listed
   in the main menu with icons and loaded when picked; applications opened from elsewhere load
   the same way without being in the catalog. Documents name icons at several sizes (4.9).
3. Any local or remote document may be opened by default, sandboxed, as a browser opens a page;
   settings narrow it to the catalog or ask first (4.2, 4.12, 4.13).
4. A scene opens in a window by default. An application may ask for the main view, and the user
   or the outer document grants it or not (4.4).
5. Every application gets a budget from the start, with defaults by principal and a setting per
   application (4.11).
6. Sensible default grants (the tray, its own Settings page) with an opt-out for the outer
   document (4.5).
7. Grant answers are kept for the run, and saved with the shell's settings: a file natively,
   browser storage in wasm (4.13).
8. Contributed menu items go under the application's name, after a separator (4.7).
9. A link is recognised by its media type, then its extension. The archive is `.aria`: a zip of
   the `.ari`, what it refers to, and a manifest. TX.WTF posts should carry an application
   attachment with the pin and the publisher; a naked link opens with a warning (4.12).
10. Signing keys are kept apart from the end-to-end identity keys. A package signed by a
    rotated-out key opens with a warning label (4.12).
11. Unsigned documents open, marked *unsigned*; a setting refuses them (4.12, 4.13).
12. Sandboxed documents may load the formats the shipped documents already load (4.12).
13. The in-process sandbox is enough to ship the golden natively. Out-of-process principals are on
    the roadmap, built on state_exec's process migration and the arisrv state_exec work (SH17).

**Still open:**

1. Key rotation: what a rotation means for packs already signed and posted, how a publisher
   retires a key, and what *signed with a retired key* lets the user do.
2. The media types' names (`application/vnd.cvc.ari+yaml`, `application/vnd.cvc.aria+zip`), and
   whether to register them.
3. The shape of TX.WTF's application attachment, and where its server change is made.
4. ~~The default budget numbers per principal~~: settled in SH1 (4.11, *As built in SH1*).
5. In the browser, IndexedDB or `localStorage` for the saved settings (4.13).

## 7. Not in scope

- Several outermost documents, each its own Runtime, in one window. The shell is one document
  with mounts.
- Relay and conference documents as applications. They need a `relay:` block and the host verbs
  of an `AppRuntime` of their own.
- Scenes in the terminal hosts. `ari`'s backends draw no scene, so there the shell runs documents
  without scenes; `arigl --terminal` draws scenes (§9.2).
- Out-of-process principals in the first version. They are on the roadmap as SH17, with a design
  of their own; the in-process sandbox and the format list come first.
