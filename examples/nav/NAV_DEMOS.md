# cvc::nav examples — the reactive navigator, drawn by Ariadne

The [GRL-SNAM](https://github.com/CVC-Lab/GRL-SNAM) navigation scenarios on libcvc's own
`cvc::nav` runtime (`sim_world` / `sim_thread`, **no Python, no libtorch**): the same brain as the
Python demos, drawn in 3-D by Ariadne documents, plus three headless CLIs.

The C++ programs that once drew the swarm through cvcGL by hand (`nav_city_swarm`,
`nav_city_drive`, `nav_fog_ghost`, `nav_finale` and their `nav_common`) are retired. Each loaded
and fitted its vehicle model its own way. The documents draw their vehicles through Ariadne's one
vehicle look ([docs/ARIADNE.md](../../docs/ARIADNE.md), "A model in parts"), so the same Humvee
model, in parts where the vehicle-humvee package has them, is drawn everywhere.

## The rendered demos

| demo | what it shows |
|------|---------------|
| [`nav_city_swarm.ari`](../ariadne/documents/nav_city_swarm.ari) | 800 vehicles cross a city (the Austin bundle, or `--bundle DIR`) under fog with grouped belief: goal flags, street-flow trails, the fleet's fog coverage, a minimap. `--no-fog --agents 1500` is the scale-benchmark path. |
| [`nav_city_drive.ari`](../ariadne/documents/nav_city_drive.ari) | the legible cousin: 60 Humvees on Jolt in colour-coded convoys thread downtown Austin to six rally points; chase one, drag a rally point, switch belief. |
| [`nav_scrub`](../ariadne/nav_scrub) | watch a policy learn: cvc::nav's coef_trainer trains on a walled square of city blocks, every checkpoint is deployed on the Jolt Humvee and on nav's kinematic bicycle, and the document shows checkpoint k beside step 0 ([docs/NAV_SCRUB.md](../../docs/NAV_SCRUB.md)). In the gallery it trains and deploys in the page. |
| [`nav_convoy`](../ariadne/nav_convoy) | a navigation demo whose whole UI is the reusable component library (`cvc://components/*.ari`); its C++ host adds only a streamed agents node, a few verbs (`sim.restart`, `sim.step`, `camera.recenter`) and its readouts. |
| [`world_lab.ari`](../ariadne/documents/world_lab.ari) | one scene with three worlds and the vehicles you pick, set up and spawn onto spawn points placed on a minimap (ARIADNE_WORLDS.md section 14). |

The city documents are Ariadne documents that `arigl` runs with no program of their own. The city
is the document's `data:` block (the scene-austin-south package, or `--bundle DIR`), drawn with its
aerial image on the ground and on the buildings. The swarm is a `nav_swarm` scene node, and each
thing it draws (the vehicles, flags, goals, trails, belief walls, the minimap's markers) a
`swarm_layer` node under it with its own `visible:` and `tags:`. The panels are the shipped
`swarm_controls` and `swarm_minimap` components, and the chase camera, the 2-D map and the capture
cameras are the document's `on_tick` ([ARIADNE.md §5.4, "Swarms"](../../docs/ARIADNE.md)).

```sh
arigl --net-allow https://cvcpkg.org nav_city_swarm.ari        # Austin from cvcpkg.org, then the cache
arigl nav_city_swarm.ari --bundle /path/to/austin_south          # a local bundle
arigl nav_city_drive.ari --bundle /path/to/austin_south --follow 0
arigl snapshot --mp4 city.mp4 --seconds 20 --width 1280 --height 720 nav_city_swarm.ari --capture orbit --no-ui
```

Their command lines take the old programs' options: `--agents`, `--belief`, `--no-fog`, `--plant`,
`--seed`, `--hz`, `--follow`, `--capture orbit|fly|follow`, `--map`, `--no-shadows`, `--no-ui`,
`--no-pip`, and `--bench` (the frame rate on stderr every 120 frames).

They drew the same scenes faster than the programs did. On a laptop with an RTX 3050 Ti, at
1280 x 720 on the full Austin bundle, the navigator keeping its 60 steps a second in all of them:

| | the program | the document |
|---|---|---|
| swarm: 800 vehicles, fog, grouped belief | 17 fps (19 without the minimap) | 45 fps (50 without) |
| drive: 60 Humvees on Jolt | 29 fps | 55 fps |

Where it comes from: the coarse vehicle rungs are decimated from the model welded at its texture
seams and keep only the points their triangles use (the programs' far rung streamed 262 points a
vehicle, the document's 115, and its middle rungs a third as many); a rung takes only vehicles big
enough on screen to need it, so an overview streams no full-detail Humvees; an empty rung and an
unchanged slot's colours are not sent again; the trails stream every third frame; the minimap
redraws five times a second (and every frame while you drag in it); and the sun's shadow maps follow
the camera.

## Headless CLIs

This directory carries three **headless** CLIs that exercise the same `cvc::nav` runtime without
any GL. They link only `cvc::cvc` (no cvcGL, no VTK), are built and installed with the examples
(`-DCVC_BUILD_CVCGL=ON -DCVC_BUILD_EXAMPLES=ON`), are native-only (the browser gallery is for
rendered scenes), and each runs in the terminal:

| CLI | what it does |
|------|--------------|
| **`nav_swarm_demo`** | the pure-C++ swarm template: a synthetic walled grid drives thousands of vehicles through `sim_world`. |
| **`nav_train_demo`** | self-supervised CoefMLP training CLI: differentiable rollout on a scene's SDF, writes the `.cvcnav` the swarm then drives with ([docs/NAV_TRAINING.md](../../docs/NAV_TRAINING.md)). |
| **`nav_material_demo`** | the material-aware A/B run: the same lane of vehicles with risk/hazard semantics (soft mud, lethal water) off then on, side-by-side stats + PPM trajectories ([docs/NAV_MATERIAL.md](../../docs/NAV_MATERIAL.md)). |

`nav_abi_smoke` (not installed) is a build-time check: it builds a `sim_world` as a swarm does and
fails loudly when the shared libcvc it links is older than the headers it was compiled against.

```sh
cmake --build build --target nav_swarm_demo nav_train_demo nav_material_demo
cvcpkg install libcvc-examples         # the CLIs in <prefix>/bin, the documents in share/libcvc-examples
```

## How the GRL-SNAM nav model works

Every demo runs the **same reactive brain**: the torch-free `cvc::nav` port of the GRL-SNAM
navigator. There is no neural net *in the loop*; the "learning" is baked into three scalar
coefficients. Per agent, per tick:

1. **Sense** (only under fog): the agent ray-casts the *truth* map into its own **belief**, an
   M-plane **log-odds occupancy** grid (`sim_world` holds M planes; agent `n` reads/writes plane
   `map_id[n]`). Free cells accrue negative log-odds, hits positive (clamped ±8). When a cell's
   occupied bit flips, the plane's version bumps.
2. **Rebuild the field**: for any plane whose belief changed, the **signed-distance field** φ
   (clearance to the nearest *believed* wall) is recomputed by an exact EDT. *The rebuild is the
   replan:* the agent plans on φ, so a corrected belief instantly reshapes the route.
3. **Coefficients**: a tiny **CoefMLP** (5→64→64→3 SiLU, shipped as `coef_mlp.cvcnav`) maps local
   features `[φ, goal_dist, goal_dir·2, goal_dir·wall_normal]` to three gains
   **(α, β, γ) = wall-barrier / goal-spring / damping**. The default weights give the constant
   `(1, 3, 4)` basin; the trained weights come from GRL-SNAM's self-supervised
   differentiable-rollout surrogate (CoefEnergyNet, no labels).
4. **Drive**: an IPC-style barrier on the clearance `d = φ − r`: a wall force `−α·b′(d)·n̂` pushes
   off obstacles, a spring `−β·(pos − goal)` pulls to the goal, `−γ·v` damps, turned into throttle
   + pure-pursuit **steer toward a carrot** by a kinematic bicycle with a corner/stopping governor.
5. **Carrot FSM**: the carrot normally sits ahead on the goal bearing (*seek*). If the agent stalls
   in a concavity (a potential-field dead-end) for ~70 ticks it switches to **wall-follow**,
   walking the carrot along the obstacle tangent until it rounds the corner: the reactive escape a
   pure potential field can't do alone.

**Belief modes** choose M and who shares a plane: `shared` = one plane for everyone (the
thousands-of-agents path); `grouped` = K planes from a k-means-lite on start positions
(K = max(2, agents/64)); `private` = one plane per agent (M = N). **They only differ in
*behaviour* under fog**: with sensing off, every plane stays equal to the known map, so
`grouped`/`private` merely recolour the same trajectories.

## GPU / GLX troubleshooting

arigl renders through VTK/OpenGL and needs a working GL context. If a run dies with
`X Error: BadValue … Request Major 152 (GLX)`, check the GPU driver first. This is almost always a
host issue, not the document:

```sh
nvidia-smi   # "Failed to initialize NVML: Driver/library version mismatch" == the smoking gun
cat /proc/driver/nvidia/version                     # loaded kernel module version
ls /usr/lib/x86_64-linux-gnu/libGLX_nvidia.so.*     # userspace lib version
```

If the loaded kernel module and the userspace libs disagree (a driver package updated but the
machine hasn't rebooted), GLX is down **system-wide**, for `glxinfo` and arigl alike. **Reboot** to
load the matching kernel module and the GPU renders again with no special env.

As a last-resort stopgap while the GPU is down (e.g. capturing an offscreen preview before a
reboot), `LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa arigl …` forces mesa llvmpipe
(software, slow: fine for a quick check, not for deliverable mp4s).
