# cvc::nav demos — the reactive swarm, rendered through cvcGL

Runnable C++ programs that show the [GRL-SNAM](https://github.com/CVC-Lab/GRL-SNAM)
navigation scenarios natively: the exact `cvc::nav` runtime (`sim_world` / `sim_thread`,
**no Python, no libtorch**) driving agents inside the cvcGL scene graph — the payoff of the
torch-free nav port. Same brain as the Python demos, rendered in real 3-D.

| demo | what it shows |
|------|---------------|
| **`nav_city_swarm`** | N vehicles reactively navigate a procedural "city" (the same `city_scene` the trainer uses); agents are coloured by belief group. The scalable hero — one merged glyph mesh streamed per frame, smooth into the thousands. |
| **`nav_fog_ghost`** | the fog-of-war "ghost" story: one vehicle carries a stale belief map with a phantom wall reality lacks; it detours around the ghost, senses the space is actually clear, and drives through. The ground heatmap is the agent's **SDF clearance field φ** (red = near a believed wall, blue = open) — watch the ghost trough dissolve as it senses. |
| **`nav_finale`** | the flagship: 8 vehicles that already **hold the city map** drive in from the west edge and rendezvous (Act 1), then split into pursuit packs chasing 4 moving targets (Act 2), on the **real Austin** bundle. Global A\* spine + reactive local control; 3-D only, with a live 2-D picture-in-picture minimap (agent positions + the line to each one's current target). |

Both `nav_city_swarm` and `nav_fog_ghost` render **either** the default 3-D perspective
**or** a top-down 2-D orthographic "matplotlib" map with `--ortho` — one codebase, both
looks. `nav_finale` is 3-D only (the 2-D view is the PiP minimap).

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

Interactive window (Z-up; `Tab` = orbit/fly, `WASD`+mouse to fly):

```sh
./build/bin/nav_city_swarm --agents 1000 --belief grouped
```

Offscreen capture → PNG frames → mp4:

```sh
./build/bin/nav_city_swarm --agents 1500 --belief private \
    --capture orbit --offscreen --frames 300 --fps 30 --out /tmp/city
ffmpeg -framerate 30 -i /tmp/city/frame_%05d.png -c:v libx264 -pix_fmt yuv420p city.mp4
```

`nav_city_swarm` options: `--agents N`, `--grid G` (city resolution), `--belief
shared|grouped|private` (agents are coloured by belief group), `--fog` (enable sensing so
belief diverges; default is a static known map — the thousands-of-agents path), `--hz`
(sim tick rate), plus the shared capture flags (`--capture none|orbit|fly`, `--offscreen`,
`--frames`, `--fps`, `--out`, `--png`, `--width`, `--height`, `--no-shadows`).

The swarm runs on a `sim_thread` off the render thread; each frame the loop reads its
latest lock-free snapshot and streams the poses into the one agent mesh.

### Installing / running

Preferred: install into a cvcpkg prefix and run straight from `bin/` — the binaries carry
`$ORIGIN`-relative RPATHs, so there is no `LD_LIBRARY_PATH` to set:

```sh
cvcpkg install cvcgl-examples          # drops lsystem_forest + the three nav demos into <prefix>/bin
<prefix>/bin/nav_city_swarm --agents 1000 --belief grouped
```

From a local build tree, `build-demos/run-demo.sh <demo> …` sets `LD_LIBRARY_PATH` so the
fresh libs win over any stale installed libcvc — nothing else. Like every cvcGL example
(`lsystem_forest` included), the demos render on the **GPU** by default.

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

As a last-resort stopgap while the GPU is down (e.g. capturing an offscreen preview before a
reboot), `NAV_SOFTWARE=1 build-demos/run-demo.sh …` forces mesa llvmpipe (software, slow —
fine for a quick check, not for deliverable mp4s).
