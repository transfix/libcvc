# Ariadne shell: one document that hosts other documents

*Draft design, 2026-11-22. Nothing in it is built. The section numbers in
[ARIADNE.md](../ARIADNE.md) are cited as §N.*

## 1. Goal

A standard entry document, `cvc://components/shell.ari`, that runs other `.ari` documents as
applications:

- it opens them on demand from a catalog (first, the examples' `gallery.json`), runs several at
  once, and closes them;
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
   has no right-click menus at all yet: ImGui's are used only inside two native widgets (the
   text view's Copy, the transfer function editor).
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
    allow: { bind: shell.allow }                 # cvc:// and the catalog's directory, set by init:
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
shell.apps.<as>.background   hold (the default) or run (4.4)
```

- **Mounting.** When a child appears, the Runtime asks its host's loader for the fragment. The
  loader resolves and parses it as `expand_load` does: the same URI resolver, the cycle, depth
  (32) and total (256) guards, the component paths. The fragment is mounted at `includes.<as>`,
  exactly as a static `load:` with `as: <as>` would be: its `init:` run, its `on_tick:` started.
  `AppRuntime` gives the Runtime that loader as a callback, so the loader stays free of state.
- **Closing.** When the child goes, the mount is quit (`quit_app`), its widgets and scene come
  down, and it leaves the tree. Its state stays under `includes.<as>` until the shell drops it,
  so reopening the same id resumes it, as Windows > Applications does today.
- **Caching.** A parsed fragment is kept per URI, args and file stamp. A reload
  (`AriRuntime::reload_if_changed`) parses it again, and a mount whose program did not change
  keeps its resident, as a static `load:` does (§7.4).
- **What may be mounted.** `allow:` lists the URI prefixes the slot accepts, literally or from a
  state key (`allow: { bind: <key> }`). A URI outside them is refused, and the refusal is posted
  to the feed. The fragment's `requires:` goes through the scan `check_host_requirements` runs
  over a static load's modules. Where that scan warns, a `mounts:` entry is refused instead, with
  the need named (`gl.scene` under `ari`, say). A URL from the network is mounted as a remote
  principal, and only through the consent sheet (4.12).
- **Grants.** A `mounts:` application gets no `link:` holes and no functions of the shell's.
  What it may reach beyond its own scope is what it asks for and the shell grants (4.5): the
  shell's input, signals, menus, right-click menus and Settings window, and the settings the
  shell shares (4.8). A later version may take `link:` and `functions:` per entry too.
- **Verbs.** `(app-open URI [AS [ARGS]])` writes the entry and returns its id. `(app-close AS)`
  removes it. `(app-view AS window|main)` moves its scene; `main` writes the `front:` key. They
  act on the document that calls them: a mounted application cannot open others unless the shell
  grants it the functions.

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

### 4.4 Where an application's scene is drawn: its window or the main view

Both are supported. An application with a scene has a view, `window` or `main`
(`shell.apps.<as>.view`; `view:` on a static `load:`), and the user can switch it while it runs:
its menu has *Show in the main view* or *Show in a window*, and `(app-view AS window|main)` does
the same from a program. At most one application is in the main view at a time, the *front*
application; the outermost document's top-level `front: <state key>` names it. Putting another
application in the main view sends the one there back to its window.

**The same scene, the same camera.** Either way the application's scene is drawn from
`<sub>.viewers.main.camera`, so a switch keeps its view, and a program that steers the camera
(`viewers.main.camera.track.*`, say) works in both. A switch moves where the scene is drawn and
nothing else: the SceneGraph, its world and its bodies stay as they are.

**In a window** (the proposed default):

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
- **Background applications.** An entry's `background:` is `hold` (the default) or `run`. A held
  application's world is not stepped while its scene is drawn nowhere: not in the main view, and
  its window closed or collapsed. It stops following the app's world clock until it is drawn
  again. Its `on_tick:` and actions keep running; Quit is what stops them. `run` keeps its world
  following the same clock as the others, in lockstep, since an app has one world clock (gap 5).
  Rates of their own (a scene's `clock: hz`) for applications running side by side would need a
  world clock per scene. That is a world-clock API change and is left open (section 6).

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
- **The default.** With no `grants:`, a mount gets `[extend.tray]`, which is what §5.25 gives a
  loaded application today. Its own application menu and its notifications need no token: both
  are its own and named as its own (§5.24, §5.25). `grants: []` turns the tray off too.
- **Asking the user.** On a `mounts:` slot, `grants: { ask: [...] }` names tokens the shell puts
  to the user when an application that asks for them opens: a dialog that names the application
  and each token in words ("receive keys while it is in front", "change the shell's settings").
  The answer is kept for the run under `shell.grants.<uri>`.
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
  every mount's included. Today a document in group 0 also sees the other documents' processes.
  The threaded plan's S2b limits a process to its owner's; the shell's mounts share the shell's
  owner, so the view keeps them. Another document's processes in another execution group come in
  with S2c, which merges the owner's rows across groups. S2 is still at S2a (transfix/libcvc#1404,
  draft).

### 4.11 Budgets

Local applications may share one budget per document at first: the applications share the
shell's, and the Processes view shows each application's share of the steps. A document from the
network (4.12) may not: it gets a budget of its own per frame and a memory cap from the start
(SH12), so a stranger's busy loop cannot stall the shell. A mount in an execution group of its own
comes later; it needs processes that can move between groups (threaded execution Stage 3).

### 4.12 Documents from the network: the nicotina golden

**The golden.** cy-pca/nicotina, the TX.WTF client written as `.ari` documents, runs as an
application in the shell. Someone posts a link to an `.ari` document. A reader clicks it in
nicotina's feed. The shell fetches the document over https, shows the reader what it is, where it
comes from, who published it and what it asks for, and on the reader's yes runs it as an
application of its own beside nicotina, in a scene window. Applications that travel as links are
the start of a software ecosystem, so the shell treats every such document as code from a
stranger. That scenario, run end to end, is this section's acceptance test (SH15).

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
- `remote:<origin>`: a document fetched from a URL, such as `remote:https://apps.example.org`,
  with its publisher when it is signed (below).

The principal is fixed at mount time from where the bytes came; a document cannot name its own.
What it loads inherits it: a remote document that `load:`s a file from its own origin runs that
file as the same principal and cannot raise it.

**What a remote document can reach.** Default-deny, as the relay already runs its clients'
programs (`room-run`: the read lane's builtins and a list of allowed names, `server_document.h`)
and as `LaneProcess::allow` does for any program from outside a document:

| Resource | A remote document by default | With a grant (on the consent sheet) |
|---|---|---|
| Its own state, widgets, windows and scene | yes, under its mount (§7.3) | |
| Another document's state, the shell's | never | the settings the shell shares (`access.settings`, 4.8) |
| Host verbs and functions | only those marked safe for remote principals: the DSL's own, the state verbs, `notify`, the scene's | each other verb by name (`clipboard-set`, `open-url`, `file-pick`, `http-upload`, ...) |
| The network | its own origin, for its files and its API | more origins it names in `requires:` (`{name: net.origin, origins: [...]}`) |
| Credentials | its own, under its principal: its `txwtf` is not nicotina's | never another principal's |
| End-to-end keys, OIDC sign-in | none | its own, under its principal |
| Local files | none: relative paths resolve in its fetched set, and `file://` is refused | a file the user picks (`file-pick`) |
| Persistent storage | its own, per principal, with a quota | |
| `python:` blocks, native extensions | refused: the mount fails | never |
| WebAssembly guests (`wasm:`) | yes, sandboxed, under its budget | |
| CPU and memory | a budget of its own per frame and a memory cap (4.11) | |
| Input, menus, tray, right-click menus, Settings | as 4.5 to 4.7 | the tokens of 4.5 |

**Network, credentials and keys per principal (SH11).** The host checks each request against
the principal of the program that makes it; the program's scope tells the runtime which mount it
runs for. The allowlist is per principal: the launcher's (`--net-allow`, `--net-policy`) for
`local` documents, as today, and for a remote one its own origin plus what it was granted.
Credentials are kept as (principal, name, origin, token), and a lookup by name finds only the
caller's principal's; the credential file's lines gain the principal. The end-to-end keys and
OIDC sign-ins are split the same way. A local application can also have a policy and a store of
its own: `shell.apps.<as>.net_policy` and `.credential_store`, as `bin/nicotina` passes
`--net-policy fleet.policy --credential-store nicotina` today. This split is needed as soon as the
shell hosts nicotina beside any other application, remote or not.

**Untrusted bytes in native readers.** A remote document's datasets, scene sources, pictures and
videos are a stranger's bytes handed to native readers: VTK's, the mesh and image decoders, the
media player. Those are not memory-safe. Until they are fuzzed, or moved out of the process, a
remote principal may load only a vetted list of formats (settled in SH12; glTF, PNG and JPEG are
candidates), and anything else is refused with a notification. Isolation out of the process is
the strong answer (section 7). The browser host, arigl-web, has the browser's sandbox around all
of it already.

**Links and fetching (SH13).**

- **An `.ari` link** is an `https://` URL whose path ends in `.ari` (a single document, or a
  set's main document) or `.aripack` (a pack, below). `http://` is accepted for `localhost` only,
  for tests. Other sources come through the same fetcher later: `cvcpkg://` (a registry package,
  checked against the registry's SHA-256 as a `data:` package is, §5.17) and a relay's world pack
  (OW5k).
- **A pin.** A fragment pins the content: `https://apps.example.org/orrery/main.ari#sha256=<hex>`.
  A fragment never reaches the server, so the link works in any client, and the shell refuses
  bytes that do not match.
- **Fetching.** The shell fetches with `doc_fetch` (§9.4), which arigl-web uses today: the
  document and every file it names relative to itself, from its own origin only. The files go in
  a cache directory per principal and content hash and run from there, as an installed set does.
  The native fetcher is new; the browser keeps its own.
- **Packs.** A pin on a set's main document does not cover its imports. A pack is a set with an
  index of every file's SHA-256 and size: the `world-pack.index` format OW5k brings for world
  packs (transfix/libcvc#1470, `world_pack.h`), made general for any document set. A link pins the
  index's hash, and the shell checks every file against the index. `ari pack DIR` writes one.
- **Signatures.** A pack may carry a signature over its index by a publisher's key. The shell
  shows a verified publisher only when the key is bound to an identity it can check: a TX.WTF
  account that publishes it. Accounts already publish an Ed25519 identity key for end-to-end DMs
  (`identity_key_ed25519` in `GET /e2ee/keys/{username}`, §10.8), so the place to publish a key
  exists. Whether packs are signed with that key or with a signing key of their own published
  beside it (keeping the two uses apart), and how keys are rotated, is settled in SH13
  (section 6). An unsigned document still opens, marked unsigned.
- **Updates.** A pinned link always runs the same bytes. An unpinned one is fetched again when
  it is opened; if it changed and asks for more than before, the sheet asks again.
- **The cache** is content-addressed, so a pack posted twice is fetched once. It has a size cap,
  and the Launcher lists what it holds.

**The consent sheet (SH14).** Before a remote document runs any program, its `init:` included,
the shell shows a sheet. It is the shell's own window, drawn at the top of the stack (SH2 makes
that hold), so no application can draw over it or imitate it. It shows:

- the document's `meta.name` and description, marked as the document's own words;
- where it comes from: the origin, and whether the link pinned it;
- who published it: the verified publisher, or *unsigned*;
- who asked to open it: "posted by @user, opened from nicotina", from the request (below);
- what it asks for, in words: each token beyond the default, each host verb, each network origin,
  and its size;
- *Open*, *Cancel*, and for a signed document *Always trust this publisher*, which keeps the
  answer in the shell's settings, so that the sheet comes back only when the asks grow.

A running remote application carries a badge the shell draws in its window's title and in its
menu: its origin and how far it is trusted (pinned, signed, unsigned). *Permissions...* (4.5)
lists what it holds. Close ends it. Its state and storage stay under its principal until the user
clears them.

**Opening from inside an application: `request.open` (SH14).** nicotina must not mount the
document itself: a mount of nicotina's would be its child, inside its scope. It asks the shell:

- A new token, `request.open`, lets a document ask its shell to open a document as the shell's
  own application. With it, `(app-open URI (dict "from" TEXT))` from a mounted document is not a
  mount. It is a request to the nearest enclosing document with a `mounts:` slot, which applies
  its own `allow:` and principals and shows the sheet, naming the requester ("nicotina asks to
  open ..."). The new application is the requester's sibling: neither reaches the other's state.
- Without the token, or with no enclosing `mounts:` slot (nicotina run on its own),
  `(has? "request.open")` is false, and nicotina copies the link as it does today. So nicotina
  runs the same in the shell and outside it.
- The shell takes a request only within a second of the requester's own input (a click or a key
  in its window, or a link it drew), as a browser allows a pop-up, and drops it otherwise.
- `(app-info URI)`, under the same token, has the shell fetch a document's metadata without
  running anything: `{name description origin pinned publisher size}`, or why it could not.

**nicotina's part** (cy-pca/nicotina, once SH14 is in):

- `open-link` sends a link whose path ends in `.ari` or `.aripack` to
  `(app-open href (dict "from" <the post's URL and author>))` when `(has? "request.open")`, and
  copies it otherwise, as now.
- `nicotina.ari` asks for `{name: request.open, optional: true}`. The shell's entry for nicotina
  grants it, with nicotina's `fleet.policy` and credential store as its own (SH11).
- A card may show an `.ari` link as an application chip: its name, origin and publisher, from
  `(app-info href)`. Later, TX.WTF posts could carry the pin and the
  publisher as fields of their own.

**The golden test (SH15)**, one form in each repository:

- **libcvc**, `shell_remote_golden`: a local https server, with a test certificate, serves a
  small pack (a document with a scene, an import and a dataset). A fixture feed document stands in
  for nicotina: a post whose link's `on_link` calls `(app-open ...)`. The shell runs the feed with
  `request.open` granted, and the test answers the sheet. It checks that:
  - the click opens the pack as the feed's sibling, principal `remote:https://localhost:<port>`,
    its scene window drawn (an image golden) and its badge saying *pinned*;
  - it cannot read the feed's state or the shell's; its `(http-get ... (dict "auth" "txwtf"))`
    finds no credential while the feed's still works; a request to any origin but its own is
    refused;
  - a busy loop in it keeps to its budget, and the shell's frame time holds;
  - a tampered file under a pinned link, a `python:` block, and a format off the vetted list are
    each refused, the last with a notification;
  - a `request.open` with no click before it is dropped;
  - Close takes it away, and a second click opens it from the cache.
- **nicotina**, `tests/shell.sh`: nicotina runs inside `shell.ari` against `stub_txwtf.py`, with
  a post linking to the same pack, and the test checks the same opening and the same credential
  isolation through nicotina's own card.

## 5. Phases

Each phase is one PR with tests. SH0, SH2, SH4 and SH8 do not depend on the others and can go
first or in parallel; SH5 builds on SH4 for `volren`. SH3's scene windows need SH2: they do not
ship drawn under the UI.

| Phase | Content | Tests |
|---|---|---|
| SH0 | `scope` on `(ps)` and `(inspect)` rows; the `ariadne.apps.*` mirror; `components/processes.ari` | runtime: a mount's resident is listed with its scope; Kill from the view; `ari test` on the component |
| SH1 | `mounts:` and the verbs, `allow:`, the `requires:` check at mount; a `data:` entry's `uri: { arg }`; `shell.ari` with a catalog of the example documents without scenes (`tray_demo`, `json_feed`, `apps/counter`) | runtime and loader: open, close, reopen with state kept, hot reload, refused URI, unmet `requires:`; a tmux run on FTXUI |
| SH2 | Views drawn into textures (4.4): every `viewport` renders into a texture of its own and draws it as an image in its window; no background hole. A cvcGL change every view gains from | goldens of a window over a view and a view's window over another window, each stacking right; a scrolled and a clipped view; the first frame drawn; picks still land |
| SH3 | Scenes of mounted documents, for `load:` and `mounts:`: `realize_mount`, `clear_mount`, `viewport scene: { mount }`; the scene window (`view: window`), its scene's own renderer into a texture; `background: hold \| run` | loader: a fragment's scene is parsed; AriRuntime: two mounts' scenes realized apart, binds resolve under the sub-prefix; a held world is not stepped while its window is closed; a golden of two applications' scene windows overlapping, one with a `volren` node |
| SH4 | `volren` in every view (4.4): a raycast per renderer that shows the node, each with its own camera, picture, depth map and quad | `cvcgl_ariadne` goldens of one `volren` node in the main view and two viewports from other cameras, depth-tested against geometry in each; a view that does not change does not raycast again |
| SH5 | Restyling volumes in views (4.4): `transfer_function`, `color` and `opacity` overrides on `volren`, `volslice` and `type: volume` nodes; a `volslice` view prop's own transfer-function texture and per-view uniforms; the VTK patch (cvcpkg recipe, opt-in) that keeps a GPU volume mapper's transfer-function tables per property | goldens of each kind in the main view and in three views (another preset, one colour, half opacity), the main view unchanged; a `type: volume` node restyled in two views uploads its volume once; a `volslice` sample shader lit right in a view |
| SH6 | `view: main` and `front:`: the main view draws the front application's scene; switching between window and main view; Settings follow the focused scene | switch to the main view and back, camera and world kept; the one in the main view goes back to its window when another takes it; golden |
| SH7 | Asks and grants (4.5): the tokens in `requires:`, `grants:` on `load:` and `mounts:`, the default; keys, pointer and signals to granted applications; `argv:` (4.6) | loader and runtime: a required token not granted refuses the mount; a key reaches only the focused granted application, and none while a parent's field has the keyboard; SIGTERM reaches every granted application; `argv` parsed, a bad one refused with the usage posted; one document's `ari test` alone and mounted |
| SH8 | Right-click menus (4.7): `context_menu:`, its `id:`, the `event` dict and the pick over a view, `ui.context_menu`; ImGui, FTXUI and tvision | runtime: an item runs with its `event`; an ImGui golden of an open menu; a tmux run on FTXUI |
| SH9 | `contributes:` (menus, right-click menus, Settings pages) with their tokens; `shares:` and `access.settings`; `grants: { ask }` and Permissions... | an item added to File runs in the application's scope; a withdrawn token takes its items away at the next frame; `access.settings` writes the shell's key and `access.settings.read` cannot; a document alone adds the same items to its own menus |
| SH10 | `arigl --shell [catalog]` opens `shell.ari` on the examples catalog | an arigl smoke test that opens three examples and switches between them |
| SH11 | The network, credentials, end-to-end keys and OIDC sign-ins per principal (4.12); an application's own `net_policy` and `credential_store` | a credential captured by one mount is not found from another; each mount's allowlist holds; the credential file's new lines; nicotina in the shell still signs in and posts |
| SH12 | The remote principal's sandbox (4.12): host verbs default-deny, with a list per principal; `python:` and local files refused; the vetted formats; a budget per mount and a memory cap (4.11) | runtime: each refusal, named; a busy remote mount keeps to its budget and the shell's frame holds |
| SH13 | Fetching into the shell (4.12): `doc_fetch` natively into a cache per principal and hash; `#sha256=` pins; packs (the world-pack index made general, `ari pack`); signatures | a pinned set verifies and a tampered file is refused; a pack's index checks every file; a signature verifies, or is reported missing or wrong; a second open comes from the cache |
| SH14 | The consent sheet, the badge and remembered publishers; `request.open`, taken only after the requester's own input (4.12) | the sheet comes before any program runs; Cancel runs nothing; a request with no input before it is dropped; the opened application is the requester's sibling |
| SH15 | The golden (4.12): `shell_remote_golden` in libcvc; `open-link` and `tests/shell.sh` in cy-pca/nicotina | the checks 4.12 lists, in both repositories |

SH11 comes before SH12 to SH15, and before the shell hosts nicotina beside any other
application. SH13 needs SH1; SH14 needs SH7 and SH13; SH15 needs all of them and SH3.

The closed `.ari` schema (transfix/libcvc#1464, open) learns the new keys (`mounts:`, `front:`,
`grants:`, `argv:`, `contributes:`, `shares:`, `context_menu:`, and the `net.origin` form of a
`requires:` entry) in the phase that adds each.

## 6. Open questions

1. Should background applications' worlds be held (proposed) or run by default? Is a world clock
   per scene wanted, so applications side by side keep rates of their own?
2. Does the shell replace arigl's gallery mode (`CVC_ARIGL_GALLERY`), and later the browser
   gallery, or sit beside them?
3. Which URIs may the shell mount without the sheet: catalog entries only (proposed), or also any
   local file the user picks? (Links from the network always go through the sheet, 4.12.)
4. Should an application's scene open in a window (proposed) or in the main view by default?
5. A budget per mount is required for remote documents (SH12). Should local applications get one
   at the same time, or later (proposed)?
6. With no `grants:`, should a mount get `[extend.tray]`, today's behaviour (proposed), or
   nothing?
7. Should the user's answers to `grants: { ask }` be kept for the run (proposed) or saved across
   runs? Saving needs a place for the shell's settings to persist.
8. Contributed menu items under the application's name, after a separator (proposed), or among
   the parent's own items with a mark?
9. How is an `.ari` link recognised: by its path's extension, `.ari` or `.aripack` (proposed), by
   a media type the server sends, or by a scheme of its own? Should TX.WTF posts carry an
   application attachment with the pin and the publisher?
10. Signing: Ed25519 over the pack index (proposed), with the account's end-to-end identity key or
    with a signing key of its own published beside it (proposed, to keep the uses apart)? How are
    keys rotated, and what does a pack signed by a rotated-out key show?
11. Unsigned remote documents: open them through the sheet, marked *unsigned* (proposed), or
    refuse them unless the user turns them on?
12. The first list of formats a remote document may load (4.12).
13. Is the in-process sandbox enough to ship the golden natively, or should it ship in the browser
    host first, under the browser's sandbox, until remote principals run out of the process?

## 7. Not in scope

- Several outermost documents, each its own Runtime, in one window. The shell is one document
  with mounts.
- Relay and conference documents as applications. They need a `relay:` block and the host verbs
  of an `AppRuntime` of their own.
- Scenes in the terminal hosts. `ari`'s backends draw no scene, so there the shell runs documents
  without scenes; `arigl --terminal` draws scenes (§9.2).
- Each remote principal in a process of its own, talking to the shell over a state transport.
  It is the strong answer to a stranger's bytes in native readers (4.12) and needs a design of
  its own. The in-process sandbox and the vetted formats come first.
