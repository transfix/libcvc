# Boats in Parts: Outboards That Turn, Propellers That Spin, Boat Fleets: Roadmap

**Status:** revision 1 (2026-11-26), planned. Nothing is built.
- The scope is decided: P1–P4 (§2, decision B3).
- The new model is being made (B1).
- B2, the package and the preset, is open.

The plan follows the parted vehicle look: transfix/libcvc#1471, and cy-pca/graphics-assets#21,
which published `vehicle-humvee` 1.0.0+cvc.5. It also follows `OCEAN-AND-WEATHER-ROADMAP.md`
§7 (Phase D: boats; D5 and D6 drew the boat) and `FLEETS_AND_NIGHT_ROADMAP.md` (the entity
layer, FL2). Where this document and the code disagree about what is built, the code wins.

**Goal.**
- An Ariadne boat is drawn with its outboards (or its rudder) turning as the helm turns them,
  and its propellers spinning with the throttle.
- This holds both for the `boat` scene node and for world vehicles on `dynamicboat`.
- A fleet of boats is drawn at scale by the entity layer, as a fleet of cars is.

**Provenance markers.**
- **[C]** read from source, file:line on master `327a791f4`.
- **[P]** derived or projected here, not measured.
- **[E]** an estimate.

---

## 0. Direct answers

1. **Can TacticalBoat.glb be split the way the Humvee's wheels were?** No.
   - `vehicle-tactical-boat`'s model is one welded Meshy AI surface of 29 612 triangles.
   - The Humvee's wheels were connected pieces of their own.
   - A new model, generated with separate pieces, is the plan (§3).
2. **What does libcvc need?** A parted *boat* look:
   - a hull, a steering part (an outboard or a rudder), a propeller, and a rig;
   - posed from what the physics already reports (§1). Only the propeller's spin is new.
   - The posing is shared by the `boat` node (P2) and world vehicles (P3).
   - Boat fleets need the entity layer to take boats, which it never does today (P4, §6).
3. **How does it ship?** As the Humvee's parts do:
   - a standard-library splitter in graphics-assets (`tools/split_boat.py`, beside
     `tools/split_wheels.py`) runs at publish time over the staged source;
   - the parts and a `rig.json` are installed under `share/cvc-scenes/shared/<name>/` (§8).

## 1. What exists today

- **The `boat` node draws one piece** [C `src/cvcGL/ariadne/boat_node.cpp:100-173`].
  - It draws its `model:` (or the tactical preset's TacticalBoat), fitted to
    `dyn_boat_desc::length_overall` with its keel on the hull's bottom (`boat_fit_of`, `:78`).
  - Otherwise it draws its hull (`add_hull_node`).
  - It is bound to the hull body in a scene tick (`:153-171`).
  - The helm keys `throttle` and `rudder` drive only the physics
    [C `inc/cvc/ariadne/scene_boat.h:36-41`].
- **The physics reports what a drawing needs** [C `inc/cvc/physics/boat.h:120-148`].
  `boat_state` has, after the actuators' first-order lag:
  - `outboard_angle` (rad, + turning left);
  - `rudder_angle` (rad, + trailing edge to port);
  - `throttle` ([−1, 1], negative astern).
  - The boat node reads it through `BoatRuntime::state()`; world vehicles through
    `dyn_boat_plant::boat_state(i)` [C `inc/cvc/physics/dyn_boat_plant.h:222`].
  - Nothing reports a shaft speed.
- **The sign.** A positive rudder control turns the boat left [C `boat.h:28-29`].
  - An outboard turns it by "pushing the stern right to turn left" [C `boat.h:39-41`], so its
    propeller swings to port.
  - A rudder's trailing edge goes to port [C `boat.h:46`].
  - In both cases the drawn part turns by **−angle about +z** at its pivot [P].
- **The tactical preset** has twin outboards, as one thrust at their propellers
  [C `src/cvc/physics/dyn_boat_plant.cpp:284-302`]:
  - 0.5 m aft of the transom, at (−3.6, 0, −0.357);
  - swinging 0.3 rad;
  - their legs one 0.2 m² rudder at (−3.6, 0, −0.42) that swings with them;
  - `length_overall` 7.8.
  - Its hull is baked from TacticalBoat.glb (`kTacticalLoft`; `tools/boat_hulls/README.md`).
- **World vehicles** [C `src/cvcGL/ariadne/vehicle_view.cpp:1193-1327`]:
  - A boat with a box look draws its hull (`:1255`).
  - A boat with a model look goes down the car path: `vehicle_desc_of`'s stand-in shape, and
    `base_z = ground_z − z` (`:1283`).
  - A parted look there would be posed as a car's wheels.
  - `pose_vehicle_wheels` (`:1329`) poses only wheels.
- **The entity layer never takes a boat:**
  - [C `src/cvcGL/ariadne/entity_view.cpp:805-806`] "a boat: its hull, its own node";
  - [C `src/cvcGL/ariadne/room_view.cpp:486-487`] the same on relay clients.
  - Every boat is a scene node of its own, so a fleet of N boats is N nodes.
- **Relay clients** draw a parted look at rest (`attach_vehicle_model` with no wheels list).
  - They pose no wheels, so the parted Humvee's wheels do not roll there either.
  - The pose stream carries a pose and `steer`. For a boat, `steer` is nav's bicycle angle, not
    its rudder ([C `inc/cvc/physics/dyn_boat_plant.h:174-178`]), and there is no throttle.
- **No water inside the hull.** The water node passes the four boats nearest the camera to its
  shader, boat nodes and boat world vehicles alike (`OCEAN-AND-WEATHER-ROADMAP.md` §7, "As built
  (D5, D6)").
  - It reads them from the world, not from the drawn nodes.

## 2. Decisions

- **B1. The model: DECIDED (2026-11-26).** A new Meshy boat, generated with separate pieces,
  is being made (§3).
- **B2. The package and the preset: OPEN.** A new model has a new hull shape, so the physics'
  hull is baked again from it.
  - **(a), assumed here:** a new package beside `vehicle-tactical-boat`, which stays as it is.
    - libcvc bakes `tactical` again from the new boat and makes its parts the preset's default
      look.
    - An older libcvc keeps drawing TacticalBoat.glb over the hull it was baked for.
  - **(b):** a new preset (`patrol`, say), with `tactical` left as it is.
- **B3. How far: DECIDED (2026-11-26): P1–P4,** so boat fleets are in scope.
  - P2b (simple parts for launch and jonboat) and P5 (moving parts on relay clients) stay
    optional.

## 3. The model

| Option | Cost | Verdict |
|---|---|---|
| Cut the motors out of TacticalBoat.glb past the transom and cap the holes (bake_loft's sections narrow to the outboards' legs past it) | 1–2 days of tooling for a one-off [E]. Caps on a welded AI surface look patched. The two motors may be fused through their bracket. The propeller cannot be cut from its leg, so **it cannot spin**. | Fallback only, if the new model's pieces come back fused |
| **A new Meshy boat in separate pieces** | A splitter, a rig and a hull re-bake: about 1½ days [E] (P1, P1b) | **Chosen (B1)**: the only option with a turning motor and a spinning propeller under a textured model |
| A simple rudder or outboard built in code under the stern | About ½ day once P2's posing exists [E] | Not for `tactical`, whose own motors would stay frozen in its mesh. Good for `launch` and `jonboat`, which draw a plain hull with no motor (P2b) |

**What the export needs.**
- **The same kind of boat as `tactical`:** about 7.8 m overall, twin outboards on brackets past
  the transom. The physics then stays close to what was measured (8 kN ahead, turning circles of
  19–32 m).
  - A single outboard works too: the rig gives one pivot.
  - A different boat is decision B2 (b).
- **Separate pieces, each a named node or mesh:**
  - the hull, with everything that does not move (console, arch, rails);
  - each outboard (cowl, leg, skeg) **without its propeller**;
  - each propeller.
  - One outboard and one propeller are enough: the rig places copies, as the Humvee's one
    wheel is placed four times.
- **Pose:** the motors straight ahead, not trimmed up; the propeller shafts along the boat's
  length.
- **Materials:** one material and one texture atlas across the pieces, which the splitter keeps.
- **Format:** real-world metres, glTF's +Y up, about 30 000 triangles in all.
- **Licence:** CC-BY-4.0 with Meshy attribution, as `vehicle-tactical-boat` has.

**Checked on arrival:**
- each named piece is its own connected set of triangles;
- the propeller's hub and axis can be read from its bounds;
- each motor has a clear swivel point at the front of its clamp bracket.

## 4. The parted boat look

**The look**, in a document's `model:` or an archetype's `look:`:

```yaml
model: { data: boats, chassis: patrol/hull.glb, motor: patrol/outboard.glb,
         prop: patrol/prop.glb, rig: patrol/rig.json }
```

- `vehicle_look` gains `motor` and `prop` beside `chassis` and `wheel`
  [C `inc/cvc/ariadne/world_rules.h:144-161`].
- `parted()` stays true, and a new `boat_parted()` is true when `motor` is set.
- Keeping `chassis` for the hull reuses the parts' parsing, their cache, the data-store wait
  and the fallback to installed files by name (`parted()`, [C `vehicle_view.cpp:536-629`]).
- `node_model_look` learns these keys as well as `{ data, file }`.
- **Models load only through vehicle_view's looks.** Nothing here reads a .glb any other way.

**The rig** uses the frame the boat is fitted in:
- metres; x toward the bow, y left, z up;
- centred along x and across, the keel's lowest point at z 0.
- This is `fit_vehicle_geometry`'s frame and `bake_loft.py`'s, so the physics' propeller and
  rudder points can be checked against it. A sketch:

```json
{ "kind": "boat",
  "length_overall": 7.8,
  "steer": { "pivots": [[-3.42, 0.46, 0.62], [-3.42, -0.46, 0.62]],
             "axis": [0, 0, 1], "max_angle": 0.3, "part": "outboard" },
  "prop":  { "hub": [-0.30, 0.0, -0.98], "axis": [1, 0, 0], "blades": 3, "rps": 8 },
  "up": "z", "source": { } }
```

- **`steer.pivots`:** one per motor or rudder.
  - `axis` allows a raked rudder stock or a trimmed motor; the default is +z.
  - `part` is `outboard` (the default) or `rudder`, and picks which angle drives it.
- **`prop.hub`:** relative to its motor's pivot, so the propeller turns with its motor.
  - A shaft boat's propeller is not on its rudder: `prop.on: hull` gives an absolute hub.
- **The angle:** the physics' own, in radians (`outboard_angle` or `rudder_angle`), drawn as
  −angle about the axis (§1). No mapping is needed.
  - `max_angle` is used only where the helm arrives as a fraction in [−1, 1] (pose entities,
    P5). There the angle is the fraction times the preset's `outboard_max_angle` or
    `rudder.max_angle`, else the rig's.
- **The spin:** spin += 2π · throttle · rps · dt, on world time, so it stops when the world
  pauses. It runs backward astern.
  - `rps` is the view's own, because the physics models no shaft speed.
  - A real propeller turns at 30–90 rev/s, which strobes at 60 fps. About 8 rev/s on three
    blades reads as fast without the wagon-wheel effect [P].
- **The fit: one uniform scale,** `desc.length_overall / rig.length_overall`, with the keel on
  the hull's bottom, as the one-piece model is fitted today.
  - The parted car look fits each axis by wheelbase and track; a boat has neither.
  - The pivots scale with the boat.
- **Its parser:** `parse_boat_rig` beside `parse_vehicle_rig` (`world_rules.h`), or
  `vehicle_rig` with `kind: boat`.

**The drawing**, in `vehicle_view.h`/`.cpp`, beside the parted car's functions:

| Function | What it does |
|---|---|
| `attach_boat_parts(...)` | Puts `<id>_hull` under the group, and each motor as `<id>_motor<k>` (a group at its pivot) holding its mesh and `<id>_motor<k>_prop`. The parts keep their texture. Returns the drawn motors, as `attach_parts` returns `drawn_wheel`s. |
| `boat_part_poses(drawn, helm)` | Pure, like `parted_wheel_poses`. Each motor gets T(pivot) · R(axis, −angle), each propeller T(hub) · Rx(spin). |

- `helm` is `{ angle, throttle, spin }`.
- `boat_fit_of` moves from boat_node.cpp into vehicle_view, so the node, world vehicles and the
  entity layer fit a boat alike.

## 5. Where the helm comes from

- **P2: the `boat` node.**
  - The scene tick that binds the hull already finds `scene.world->boat(id)`. Each tick it
    reads `state()`, integrates the spin from the step's dt, and sets the parts' pose
    matrices. These are change-detected, as the wheels' are.
  - The `tactical` preset's default look becomes its parts where they are installed
    (`find_vehicle_model("<name>/rig.json")`); else the one-piece model; else its hull.
- **P3: world vehicles on `dynamicboat`.**
  - A new `WorldVehicles::helm(id)` returns the angle, the throttle and the propeller's spin,
    from `dyn_boat_plant::boat_state(i)`.
    - The spin is integrated each physics step inside WorldVehicles, as a wheel's roll is
      (`entity_wheel::spin`, [C `inc/cvc/ariadne/world_vehicles.h:252-266`]).
    - It is then deterministic, and the same on every holon.
  - A `pose` entity takes its angle from `pose.steer` and spins its propeller at its
    `pose.speed`.
  - `sync_vehicle_markers`' boat branch takes a `boat_parted()` look, the hull bound to the
    chassis body, with the boat fit in place of the car path's `base_z`.
  - `pose_vehicle_wheels` poses the boats' parts each frame.

## 6. Boat fleets in the entity layer (P4)

A fleet's plants all float or none do, and a fleet holds up to 4096 vehicles
(`world_rules.h:290-292`, `kMaxFleetCount`). Today a fleet of boats is that many nodes, and with
twin-outboard parts five draws each.

**P4a needs neither the model nor P2.** It lets the layer take boats:
- **The skips.** Remove the two "a boat: its own node" skips (§1). A boat goes to the first layer
  whose `of:` takes it, as a car does.
- **The fit.** `entity_draw_info` gets `fit_length = length_overall` and `base_z` = the keel,
  from `boat_fit_of`.
  - The layer is posed by the chassis body, which for a boat is its hull body: the origin the
    boat node draws about.
  - `room_entities` already sends `fit_length` [C `src/cvc/ariadne/conference/room_entities.cpp:282-286`].
    Its `base_z` for a boat becomes the keel too.
- **A box-look boat's template is its hull.** `vehicle_look_mesh_of` gets a hull branch: the
  `hull_mesh` flat-shaded as `add_hull_node` draws it, in the layer's colour roles:
  - topsides in role 0 (the entity's colour);
  - the stripe in role 1 (its second colour);
  - the wet bottom in role 2 (its own dark red).
  - These are the colours `sync_vehicle_markers` gives the hull today.
  - A box hull keeps its open cockpit and teak sole.
  - Without this branch the layer would draw a boat as the car's boxes.
- **Promotion already works.** `sync_vehicle_markers` skips an entity that is not
  `entity_drawn_as_node` before its boat branch, so a promoted boat gets its node, with its
  parts moving after P3.
- **The water.** The water node's exclusion reads the nearest four boats from the world, not
  from the nodes (§1), so drawing in the layer does not change it.
  - A fleet's farther boats have no exclusion today. A box hull's open cockpit may show the
    water at a distance. This is a limit of D5's four slots, not of P4.

**P4b, after P2: a parted boat's template at rest.** `vehicle_look_mesh_of` gets a
`boat_parted()` branch:
- the hull, each motor at its pivot straight ahead, and each propeller on its motor;
- all in role 3 (the model's) on the textured finest rung.
- The coarser rungs already turn role 3 into the entity's colour
  [C `entity_view.cpp:295`].
- **The propellers are dropped from the coarse rungs:** they are a few pixels at most. The layer
  decimates each colour role on its own [C `entity_view.cpp:238-239`], which is the likely cause
  of the parted Humvee's overhead cost (about 4 ms at 240 vehicles, with the ranges
  overlapping). So the motors stay in the hull's role there.
- **What shows:** far copies have their motors straight and their propellers still; promoted
  boats move. The parted Humvee's far copies keep their wheels at rest the same way.

**The bench**, modelled on the parted Humvee's:
- a water world with a fleet of 60 and of 240 on `dynamicboat:tactical`;
- chase and overhead views;
- three cases: own nodes (today), the layer with one piece, the layer with parts;
- six interleaved runs each, medians with their ranges;
- timed with `--fps 1000` as T(800 frames) − T(200 frames), because `arigl snapshot`'s fixed
  clock never writes `frame_ms`;
- **the physics step timed on its own.** 240 boats on `dyn_boat_plant` may cost more than
  drawing them [P], and then the layer alone will not make a fleet smooth.

## 7. Optional

- **P2b: simple parts for `launch` and `jonboat`.** The rig comes from the `boat_desc` itself:
  - a pivot at `rudder.position` (launch's shaft and rudder) or at `propeller.position`
    (jonboat's outboard);
  - a blade, or a small cowl and leg, built in code as `add_hull_node` builds the hull;
  - posed by the same `boat_part_poses`.
  - It doubles as the test fixture that needs no package.
- **P5: the helm on relay clients.**
  - The pose record gains the angle and the throttle. This is a protocol change.
  - Relay clients would then pose boats' parts and, with the same change, the parted Humvee's
    wheels. Do both or neither.

## 8. Packaging (cy-pca/graphics-assets)

- **The source.** It is staged on catx-03 under `/opt/cvc-wasm/scenes/shared/`, beside
  TacticalBoat.glb and Humvee.glb. `publish.yml` copies from there; the .glb sources are not in
  git.
- **`tools/split_boat.py SRC OUTDIR --nose=±x [--hull N --motor N --prop N]`** follows
  `split_wheels.py`:
  - **Standard library only,** reproducible byte for byte, with `--self-test` on a synthetic
    boat in `validate.yml`.
  - **Picking the pieces:** by their node or mesh names. The flags override the names; with no
    names it falls back to connected pieces aft of the transom.
  - **Output:** every part in the rig's frame (§4), z up.
  - **Measuring:** each motor's pivot (the front of its clamp bracket, at its top) and the
    propeller's hub and axis come from bounds. `--pivot x,y,z` overrides.
  - **Provenance:** `rig.json` records the source's sha256, title, author and licence, as the
    Humvee's does.
  - **Shared code:** the .glb reader and writer can move into a `tools/glbio.py` both splitters
    import. `publish.yml` runs plain `python3`, so the script's directory is on the path.
- **The package** (B2 (a)): a new recipe, `vehicle-<name>-boat` 1.0.0+cvc.1, any/noarch:
  - `share/cvc-scenes/shared/<Name>.glb`, the assembled one-piece model, for a libcvc without
    parted boats and for `bake_loft.py`;
  - `share/cvc-scenes/shared/<name>/hull.glb`, `outboard.glb`, `prop.glb` and `rig.json`;
  - a `build.sh` that checks the five files, as `vehicle-humvee`'s does;
  - an ASCII recipe;
  - a `publish.yml` case: copy the source, run the splitter.
- **In libcvc once it is published (P1b):**
  - bake the hull again from the assembled model (`tools/boat_hulls/README.md`, "A new boat");
  - replace `kTacticalLoft` (or add the B2 (b) preset);
  - set `propeller.position` and `rudder.position` from `rig.json`'s hub and pivots;
  - measure again (`PresetsDoWhatTheySay`, `TheTacticalBoatIsItsModelsHull`).

## 9. Order of work

| Phase | What | Where | Cost [E] |
|---|---|---|---|
| P0 | The model in pieces (§3) | — | in hand |
| P1 | `split_boat.py`, the recipe, the self-test; published on a go-ahead | graphics-assets | ~1 day |
| P1b | The hull baked again; the propeller and rudder points; the preset measured again | libcvc | ~½ day |
| P2 | The parted boat look on the `boat` node: the look and rig, the parts, `boat_part_poses`, the spin, tests, docs | libcvc | ~1½–2 days |
| P3 | World vehicles on `dynamicboat`: `helm(id)`, the spin in WorldVehicles, the markers and their poses | libcvc | ~1 day |
| P4a | Boats in the entity layer: the skips, the boat fit, the hull template, room_view, the bench | libcvc | ~1½ days + ½ day bench |
| P4b | A parted boat's template at rest, propellers off the coarse rungs | libcvc | ~½ day |
| P2b | Simple parts for launch and jonboat (optional) | libcvc | ~½ day |
| P5 | The helm on the relay's pose stream (optional) | libcvc | ~2 days |

- P1–P4 total about 5½–6½ days, not counting the model.
- **P4a can start now:** it speeds up fleets of today's one-piece TacticalBoat and hull-look
  boats, in a PR of its own.
- P1 and P2 can run side by side: P2's posing can be built and tested on P2b's simple parts
  before the package exists.
- P1b waits for the model.
- P2 and P3 share `attach_boat_parts` and `boat_part_poses`, and can be one PR.
- P4b follows P2.

**Tests:**
- `world_rules_test`: a rig parses, and bad ones are refused.
- A pure test of `boat_part_poses`: the sign, the pivot, a propeller on its turned motor.
- The boat node with the parts staged locally.
- The entity view's tests with boats in a layer.
- **Evidence,** as the parted Humvee's was gathered:
  - the motors at 0 and at ±0.3 rad;
  - a propeller with one blade painted, on a test copy only;
  - a short side-view video.

**Docs:**
- `ARIADNE.md` "A model in parts" gains a boat paragraph.
- `scene_boat.h`'s and `world_rules.h`'s look comments.
- `entity_view.h`'s comment that boats keep their own node.

## 10. Risks and open questions

- **Meshy's pieces.** The export's format and names are unknown until the model is in hand.
  Fused pieces fall back to the cut (§3).
- **The pivot** is measured from bounds. A trimmed or raked motor needs `axis` set by hand.
- **The hull re-bake** changes the `tactical` preset's measured numbers (top speed, turning
  circles) and the tests that pin them (B2).
- **`entity_info::ground_z` for a boat** is used by the car path's `base_z`
  (`vehicle_view.cpp:1283`). P3 and P4 use the boat fit instead and should not depend on it.
- **Physics cost at fleet scale** is unmeasured (§6, the bench).
