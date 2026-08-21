# cvc::nav demos — the reactive swarm, rendered through cvcGL

Runnable C++ programs that show the [GRL-SNAM](https://github.com/CVC-Lab/GRL-SNAM)
navigation scenarios natively: the exact `cvc::nav` runtime (`sim_world` / `sim_thread`,
**no Python, no libtorch**) driving agents inside the cvcGL scene graph — the payoff of the
torch-free nav port. Same brain as the Python demos, rendered in real 3-D.

Every demo opens an **interactive window by default** (Tab = orbit/fly camera,
WASD + mouse to fly); `--capture orbit|fly` renders offscreen PNG frames instead.
Intent, history, sensing, and belief are all **drawn** — goals, routes, trails,
sensor rings, fog tiers, captions — the same visual vocabulary as the original
2-D matplotlib demos, in 3-D.

| demo | what it shows |
|------|---------------|
| **`nav_city_swarm`** | 800 vehicles cross a varied-height procedural city under **fog** (default `--belief grouped`, M≈12 planes): every agent's **goal pyramid** is drawn (it sinks when reached), breadcrumb **trails** accumulate the street-flow, the ground is the **fleet's fog coverage**, agents recolour by state (white = wall-follow escape, green = arrived), and a HUD counts arrivals. `--no-fog --agents 1500` is the static-map scale-benchmark path. |
| **`nav_fog_ghost`** | the fog-of-war "ghost" story, told on a top-down **map view by default** (`--view 3d` for perspective): the ground shows the agent's honest epistemics — never-seen near-black, remembered dim, in-view lit, belief as an LED grid (WALL red, GHOST amber) — while the phantom stands as a **translucent amber 3-D wall that erodes cell-by-cell** as the sensor clears it. Blue PLAN line (the live carrot) vs yellow TRACK trail, GOAL/start markers, a 4-beat caption arc, 0.5× story pacing. |
| **`nav_finale`** | the flagship: 8 vehicles that already **hold the city map** converge on a drawn **staging line** (Act 1), then split into hue-matched **pursuit packs** chasing 4 hovering labelled targets (Act 2) — per-vehicle **A\* route spines** on the ground bend in place as routes retarget, engagement lines link pursuer to target, act cards narrate, and the PiP minimap draws true-position dots + route lines. Auto-probes `$CVC_NAV_BUNDLE` / `~/scenes/austin_south` for the real Austin bundle; the synthetic fallback says so on the HUD. `--view map` for a top-down ortho answer. |

## How the GRL-SNAM nav model works

All three demos run the **same reactive brain** — the torch-free `cvc::nav` port of the
GRL-SNAM navigator. There is no neural net *in the loop*; the "learning" is baked into three
scalar coefficients. Per agent, per tick:

1. **Sense** (only with `--fog`) — the agent ray-casts the *truth* map into its own
   **belief**: an M-plane **log-odds occupancy** grid (`sim_world` holds M planes; agent `n`
   reads/writes plane `map_id[n]`). Free cells accrue negative log-odds, hits positive
   (clamped ±8). When a cell's occupied bit flips, the plane's version bumps.
2. **Rebuild the field** — for any plane whose belief changed, the **signed-distance field**
   φ (clearance to the nearest *believed* wall) is recomputed by an exact EDT. *The rebuild
   is the replan:* the agent plans on φ, so a corrected belief instantly reshapes the route.
3. **Coefficients** — a tiny **CoefMLP** (5→64→64→3 SiLU, shipped as `coef_mlp.cvcnav`) maps
   local features `[φ, goal_dist, goal_dir·2, goal_dir·wall_normal]` to three gains
   **(α, β, γ) = wall-barrier / goal-spring / damping**. The default weights give the constant
   `(1, 3, 4)` basin; the trained weights come from GRL-SNAM's self-supervised
   differentiable-rollout surrogate (CoefEnergyNet — no labels).
4. **Drive** — an IPC-style barrier on the clearance `d = φ − r`: a wall force `−α·b′(d)·n̂`
   pushes off obstacles, a spring `−β·(pos − goal)` pulls to the goal, `−γ·v` damps — turned
   into throttle + pure-pursuit **steer toward a carrot** by a kinematic bicycle with a
   corner/stopping governor.
5. **Carrot FSM** — the carrot normally sits ahead on the goal bearing (*seek*). If the agent
   stalls in a concavity (a potential-field dead-end) for ~70 ticks it switches to
   **wall-follow**, walking the carrot along the obstacle tangent until it rounds the corner —
   the reactive escape a pure potential field can't do alone.

**Belief modes** choose M and who shares a plane: `shared` = one plane for everyone (the
thousands-of-agents path); `grouped` = K planes from a k-means-lite on start positions
(K = max(2, agents/64)); `private` = one plane per agent (M = N). **They only differ in
*behaviour* under `--fog`** — with sensing off, every plane stays equal to the known map, so
`grouped`/`private` merely recolour the same trajectories. `nav_finale` adds a **global A\*
spine** (`grid_nav::astar` + line-of-sight string-pull) *on top of* this reactive local
control: A\* picks the coarse route, the coef drive does the real wall-hugging in between.

### The Finale + the Austin bundle

`nav_finale` loads a scene bundle at runtime and rasterizes its `buildings.glb` into the
nav occupancy grid in C++ (`occupancy_from_model` — the C++ analog of GRL-SNAM's
`building_occupancy`, CPU scan-conversion of the mesh footprint). The bundle is **never
committed** — pass it with `--bundle`:

```sh
nav_finale --bundle /path/to/scenes/austin_south --capture fly --offscreen \
    --frames 600 --out /tmp/finale
ffmpeg -framerate 30 -i /tmp/finale/frame_%05d.png -c:v libx264 -pix_fmt yuv420p finale.mp4
```

A bundle is a directory with `terrain.json` (world bounds) + `buildings.glb`. With no
`--bundle`, the Finale falls back to a synthetic city so the scenario still runs. The
rasterized occupancy is internally consistent with the rendered mesh and the sim (all use
the same world→cell mapping); it does not bit-match Python's cached `.occ.npy` because that
is a pixel-grid GL render while ours is `sim_world`'s grid-node convention.

Shared helpers are in `nav_common.{h,cpp}`: `occupancy_to_walls` (grid → blocky buildings),
`AgentGlyphs` (**one** merged arrow mesh for all agents, streamed via
`GeometryNode::updateVertices` — the path that scales; per-node actors die at ~63 nodes),
and `orbit_camera` (the scripted capture orbit).

## Build

Build with the rest of cvcGL (`-DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON`):

```sh
cmake --build build --target nav_city_swarm
```

## Run

No arguments needed — every demo opens an interactive window:

```sh
./build/bin/nav_fog_ghost                 # the story demo (map view, captions)
./build/bin/nav_city_swarm                # 800 agents, fog, grouped belief
./build/bin/nav_finale                    # 2-act pursuit (auto-probes for the Austin bundle)
```

Offscreen capture → PNG frames → mp4 (capture is explicit, prints where frames go,
and always terminates — default 600 frames):

```sh
./build/bin/nav_city_swarm --capture orbit --frames 300 --out /tmp/city
ffmpeg -framerate 30 -i /tmp/city/frame_%05d.png -c:v libx264 -pix_fmt yuv420p city.mp4
```

Notable options: `--belief shared|grouped|private`, `--no-fog` (static known map — the
scale-benchmark path), `--agents N`, `--view map|3d` (fog_ghost defaults to map, finale
to 3d), `--speed X` (fog_ghost story pace, default 0.5× real time via a fixed-dt pacer —
world speed is display-rate independent), plus the shared capture flags (`--capture
none|orbit|fly`, `--frames`, `--fps`, `--out`, `--png`, `--width`, `--height`,
`--no-shadows`).

The swarm runs on a `sim_thread` off the render thread; each frame the loop reads its
latest lock-free snapshot and streams poses (updateVertices) and state colours
(updateColors) into the merged agent mesh.

### Installing / running

Preferred: install into a cvcpkg prefix and run straight from `bin/` — the binaries carry
`$ORIGIN`-relative RPATHs, so there is no `LD_LIBRARY_PATH` to set:

```sh
cvcpkg install cvcgl-examples          # drops lsystem_forest + the three nav demos into <prefix>/bin
<prefix>/bin/nav_city_swarm --agents 1000 --belief grouped
```

From a local build tree, run the binaries directly — the build sets an RPATH with the
fresh build's lib dir first, so no wrapper or `LD_LIBRARY_PATH` is needed. Like every
cvcGL example (`lsystem_forest` included), the demos render on the **GPU** by default.

### GPU / GLX troubleshooting

cvcGL renders through VTK/OpenGL and needs a working GL context. If a run dies with
`X Error: BadValue … Request Major 152 (GLX)`, check the GPU driver first — this is almost
always a host issue, not the demo:

```sh
nvidia-smi   # "Failed to initialize NVML: Driver/library version mismatch" == the smoking gun
cat /proc/driver/nvidia/version                     # loaded kernel module version
ls /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.*     # userspace lib version
```

If the loaded kernel module and the userspace libs disagree (a driver package updated but the
machine hasn't rebooted), GLX is down **system-wide** — for `glxinfo`, `lsystem_forest`, and
these demos alike. **Reboot** to load the matching kernel module and the GPU renders again with
no special env.

As a last-resort stopgap while the GPU is down (e.g. capturing an offscreen preview before
a reboot), `LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa ./build/bin/<demo> …`
forces mesa llvmpipe (software, slow — fine for a quick check, not for deliverable mp4s).
