# cvc::nav demos — the reactive swarm, rendered through cvcGL

Runnable C++ programs that show the [GRL-SNAM](https://github.com/CVC-Lab/GRL-SNAM)
navigation scenarios natively: the exact `cvc::nav` runtime (`sim_world` / `sim_thread`,
**no Python, no libtorch**) driving agents inside the cvcGL scene graph — the payoff of the
torch-free nav port. Same brain as the Python demos, rendered in real 3-D.

| demo | what it shows |
|------|---------------|
| **`nav_city_swarm`** | N vehicles reactively navigate a procedural "city" (the same `city_scene` the trainer uses); agents are coloured by belief group. The scalable hero — one merged glyph mesh streamed per frame, smooth into the thousands. |
| **`nav_fog_ghost`** | the fog-of-war "ghost" story: one vehicle carries a stale belief map with a phantom wall reality lacks; it sets off detouring, senses the space is clear, and drives through. The belief is a live ground heatmap (red = believed wall, blue = open) — watch the ghost wall dissolve. |
| **`nav_finale`** | the flagship: 8 vehicles enter blind from the west and rendezvous (Act 1), then split into pursuit packs chasing 4 moving targets (Act 2), on the **real Austin** bundle. 3-D only, with a live 2-D picture-in-picture minimap (agent positions + the line to each one's current target). |

Both `nav_city_swarm` and `nav_fog_ghost` render **either** the default 3-D perspective
**or** a top-down 2-D orthographic "matplotlib" map with `--ortho` — one codebase, both
looks. `nav_finale` is 3-D only (the 2-D view is the PiP minimap).

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

### Headless rendering

cvcGL renders through VTK/OpenGL, which needs a GL context. On a box with a broken/absent
GLX (offscreen fails with `X Error: BadValue … GLX`), force the mesa software GL:

```sh
DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa \
    ./build/bin/nav_city_swarm --capture orbit --offscreen --frames 300 --out /tmp/city
```

Software rendering is slow — fine for verification; render the deliverable mp4s on a
GPU/display machine.
