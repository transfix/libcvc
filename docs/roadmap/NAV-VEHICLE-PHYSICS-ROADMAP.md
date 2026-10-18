# Nav Vehicle Physics Roadmap — Jolt as the plant, one clock, and a differentiable vehicle

> **Goal:** the vehicles `cvc::nav` drives run on real vehicle physics when a simulation asks for
> it. They steer through the wheels, drive with an accelerator and a brake, and account for their
> energy. World physics (grade, grip, slip) feeds the navigator's energy. Everything runs on one
> `cvc::world_clock`. Training moves off the point-mass surrogate onto a differentiable vehicle
> model, while Jolt stays the authoritative plant for evaluation and deployment.

API reference: [`../PHYSICS_API.md`](../PHYSICS_API.md). Related: [`WORLD-CLOCK-SYNC-ROADMAP.md`](WORLD-CLOCK-SYNC-ROADMAP.md)
(the clock), [`NAV_UNITS_ROADMAP.md`](NAV_UNITS_ROADMAP.md) and [`../WORLD_UNITS_API.md`](../WORLD_UNITS_API.md)
(the SI ↔ normalized-frame boundary).

## Architecture

```
app world_clock        fixed_dt = 0.01 s (100 Hz)
  │ quanta
  ├─ every k = 6 quanta (0.06 s, the navigator's trained dt) ─ cvc::nav sim_world::step
  │     sense → fields → carrot → coefficients → forces → controller: (δ_cmd, a_cmd)
  │        ├─ plant: kinematic → the fused bicycle (unchanged, byte-identical)
  │        └─ plant: jolt      → vehicle_plant: driver model (pedals, gear)
  │                              + steering actuator (rate limit, per-wheel Ackermann)
  └─ every quantum ─ cvc::physics::world::step (Jolt, 2 collision steps)
        height field with per-cell materials, building meshes, vehicles
        → read back pose, speed, yaw rate across the SI ↔ normalized boundary (world_units)
```

## Design rules

- **The engine is the plant; the kinematic bicycle stays.** A simulation picks its plant. The
  kinematic arm is byte-identical to today, so every golden and parity test keeps its meaning.
- **SI, z up, one boundary.** `cvc::physics` is SI with gravity along −z. The navigator stays in its
  normalized frame. The two meet at the plant adapter, through `world_units::to_normalized` /
  `from_normalized` and `nav::frame_of(cfg)`, and nowhere else.
- **A floating origin.** The facade keeps positions in double precision and hands the engine float
  offsets from an origin it can rebase. That resolves about 0.25 mm across a 3 km city.
- **One μ for every plant.** Every shape and height-field cell carries a material from one table.
  Friction is combined from that table, so the tire, a box and the navigator's friction field read
  the same value.
- **No engine in a public header.** The Jolt backend is its own target with Jolt linked PRIVATE.
  `CVC_WITH_JOLT` is a private definition, and a program asks `available_backends()` at run time.
- **Deterministic by contract.** A run is a function of the initial state, the order of calls, the
  config and the backend build. `state_hash()` digests the whole simulation state. A determinism
  mode a build cannot deliver is refused, never downgraded. The physics translation units are
  compiled without FMA contraction, like nav's.
- **Thread-safe, and no singletons.** A world locks itself on every call. A binding belongs to one
  app and one world. Jolt's own process globals (allocator, factory, type registry) are scoped to
  the worlds that use them: the first world sets them up and the last one tears them down.
- **Configured from the state tree.** Everything configurable about a world lives under one prefix
  (`physics.*`): the build settings, the solver settings, gravity, the combine rules and the
  material table. A `world_binding` reads the tree into the world and publishes the world's
  telemetry back.

## Phases

| Phase | What | Gate |
|---|---|---|
| **P0 — One clock** (done) | `world_clock` lockstep and batch, followers, the `sim.*` binding; nav ticks every k quanta | Kinematic goldens unchanged; one trajectory under every pacing mode |
| **P1 — `cvc::physics` + Jolt** (done, 3.6.0) | The facade, null and Jolt backends, height fields with per-cell materials, triangle meshes, sorted contact events, save/restore, `state_hash`, the floating origin, the state-tree binding | (1) A box dropped on a height field rests at the sampled height ±1 cm. (2) A contact reports the cell's material and μ, and μ decides stick or slide. (3) The same trace twice gives the same hash. (4) 1 thread vs N gives the same hash. (5) Save, restore and re-run give the same hash. |
| **P2 — The vehicle** (done) | `vehicle_desc` with the Humvee and RoboRacer presets (`inc/cvc/physics/vehicle.h`, `src/cvc/physics/vehicle.cpp`); per-wheel Ackermann from a rate-limited steering actuator (a `WheeledVehicleController` subclass); a driver model from `a_cmd` to throttle, brake and gear; aero (against a flow field's wind) and rolling forces (C_rr from the ground material); wheel-state readback; wheels in a medium field sink to the Bekker depth, ride on the compacted surface and feel its compaction resistance and traction | Envelopes, not curves, all met in `physics_vehicle_test`: skidpad lateral ≤ μg + 10% (6.93 m/s² peak at μ = 0.7, μg = 6.86); stopping distance within ±10% of v²/(2μg) (26.25 m from 20 m/s at μ = 0.8, against 25.52 m); coastdown against the drag and rolling model (14.813 m/s against 14.813 after 15 s); the Ackermann relation within 1e-4 rad; steer rate never above its limit; no throttle/brake overlap. In 0.4 m of snow (k = 10⁶ N/m³) a wheel sinks 0.0936 m, the Bekker depth for its load, and the chassis stands 0.3057 m higher than on bare ground (depth − z = 0.3064); coasting in it decelerates 2.39 m/s² against the model's 2.47. The same drive twice, and a save / restore / re-run, give the same hash |
| **P3 — The energy ledger** | Supplied, brake, tire, aero and rolling energy, ΔKE + ΔPE, and the residual, with a battery or fuel source | On a closed drive cycle, \|residual\| ≤ 2% of supplied energy |
| **P4 — The plant seam in `cvc::nav`** | `vehicle_plant`, `plant_kind {kinematic, jolt}`, readback across the boundary, the drift tooling below | Every nav golden and parity test unchanged; below 0.3 g the Jolt plant tracks the kinematic trajectory within 0.5 m over 200 ticks; the adapter test leaves a near-zero residual; the per-tick drift is published |
| **P5 — Physics in the navigator's energy** | Grade cost and tip-over margin (H1), gravity in the governors (H2), slope look-ahead features (H3), measured slip and load as modulators (H4), an energy route cost (H5) | Parity goldens; energy per km drops on a hilly route with path-length growth under a fixed bound; zero entries into infeasible slope |
| **P6 — Bindings and harnesses** | pycvc for the physics world, the vehicle and plant selection | Python reads the same values as C++ |
| **P6a — The scene binding** (started) | `world::states`, `pose_tracker` (the last two states, interpolated at the clock's fraction), cvcGL `physics_scene_binding` (nodes and merged-mesh groups); then the city bundle loader in the library (one source for the scene's and the world's static geometry) and Ariadne `physics:` / `body:` specs | A bound node's transform is exactly the interpolated pose; off-thread presents reach the owner thread; removed bodies and destroyed nodes unbind; one writer per actor |
| **P7 — wasm-mt replay** | The libcvc recipe declares Jolt for wasm-mt; a native trace replayed under node | The same `state_hash` native and in wasm |
| **P8 — The differentiable vehicle** | See below | See below |

The Jolt `wasm-mt` column is published (`joltphysics` 5.6.0+cvc.5). Jolt's own smoke test already
reproduces the native state hash under node, with a thread pool, bit for bit.

## Training: off the point mass, onto a differentiable vehicle (P8)

What trains today:

- `coef_train` can backpropagate through the point-mass surrogate (the default) or through the full
  deployed kinematic bicycle with its controller (`rollout_kind::bicycle`, `bike_step` /
  `bike_step_bwd` in `detail/diff_rollout.h`, CPU and CUDA, gradient-checked).
- `material_train` and `geom_rollout`'s L_multi are point-mass only.
- Nothing differentiable yet models tires, slip, load transfer, actuator lag or longitudinal
  forces. Jolt is never differentiated.

The plan, in order:

1. **P8b, first item:** a kinematic-bicycle rollout for `material_train`, which needs no new plant.
2. **P8a:** `dyn_bike_step`, a planar single-track model with its adjoint, beside `bike_step`.
   - State (x, y, ψ, v_x, v_y, r), plus steering and drive-force actuator states.
   - Inputs are the controller's (δ_cmd, a_cmd), so the controller adjoint is reused.
   - Brush or Pacejka tires, a smooth friction circle, quasi-static load transfer, a low-speed blend
     to the kinematic model, and drag, rolling resistance and grade.
   - SI spec-sheet parameters converted by length exponent.
   - Gates: a gradient check, parity goldens, the kinematic limit against `bike_step`, the skid
     modes, and the stopping envelope.
3. **P8b, the rest:** `rollout_kind::dynamic` in both trainers. Gate: on the Jolt plant, a policy
   trained on the dynamic model beats the point-mass one over paired seeds.
4. **P8c:** a differentiable power model and energy in the loss, calibrated to the P3 ledger.

How the physics channels enter training:

- **Per unit mass.** Each channel is defined per unit mass, for example grade as g·∇z converted with
  `to_normalized(…, acceleration, frame_of(cfg))`. The model's mass of 1 stays valid and the
  existing coefficients need no re-dimensioning.
- **Work per channel:** a forward term and its adjoint, a finite-difference gradient check, and the
  same force in the deployed drive, off by default and byte-identical when off.
- **Measured plant state (H4)** trains only from states recorded in Jolt runs, or acts as a fixed
  modulator. It is never trained on zeros and then deployed on live values.

### Obstacle distance in training: the SDF, not discs (proposed)

The deployed drive and `coef_train` read obstacle distance from the SDF (`sdf_sample`,
`field_stack`, the `CoefMLP` features). The material stack does not: `coef_energy_net` puts one
coefficient on each obstacle disc, over `obs_feats` = [cx, cy, R_eff, W, goal_x − cx,
goal_y − cy]. `integrate_surrogate_material`, `integrate_bicycle_material` and L_multi integrate
against those discs, and the governor uses the nearest one.

Discs fit round things and fit buildings badly:

- A long wall or an L-shaped block becomes either one disc of its circumradius, which fills free
  space, or a row of discs, whose gaps overstate the clearance.
- Either way the clearance the policy trains on is not the clearance it drives on.

Proposal:

- Static geometry reaches training through the SDF, interpolated C¹ (bicubic, or the exact
  polygon distance). A bilinear SDF's gradient jumps at cell edges, which the adjoint feels.
- Discs or capsules stay for dynamic obstacles (other agents), where one coefficient per obstacle
  is natural.

This changes the material model and needs retraining (GRL-SNAM), so it belongs with P8a/P8b. Not
scheduled.

## Drift between the kinematic and Jolt plants

There will be drift, and it is bounded. The navigator reads the vehicle's state back every tick, so
errors do not pile up as they would when replaying commands blind. The sources:

- **Actuator lag:** the kinematic drive applies δ and acceleration instantly.
- **Governors that assume an instant response:** `v_stop` and `a_long ≤ (v_lim − sp)/dt`. With
  lag, the speed overshoots the stopping speed. This is the safety-relevant one.
- **Tire slip and understeer**, which grow with lateral acceleration.
- **Grade, drag and saturation.**
- **The integrator step:** one 0.06 s step against 0.01 s quanta, about half a step length apart.
- **The readback point:** the rear axle, the body yaw, and the signed forward speed there.

P4 adds three things to measure and contain the drift:

1. **An adapter test.** Jolt is set up to behave almost kinematically (stiff tires, no lag, flat
   ground) and compared against the kinematic model at `nsub = k`.
2. **A per-tick drift residual,** binned by lateral g and published next to the energy residual.
3. **Lag-aware governors in the Jolt arm only.**

## Status

- **P0** is merged; see the clock roadmap.
- **P1** has landed in this phase: `inc/cvc/physics/` (types, shapes, bodies, backends, the world
  and the state-tree binding) and `src/cvc/physics/` (the facade, the null backend and the Jolt
  backend). It is tested by `physics_world_test`, all five gates included.
  - Measured on Windows (MSVC, Jolt 5.6.0+cvc.5): the gate tests pass, one thread and three give
    the same hash, and save/restore into a fresh world reproduces the run.
- **P2** is done (above): `physics_vehicle_test`, every gate met on Windows.
  - Each wheel hangs on its own spring-damper suspension (Jolt's vehicle model), with an anti-roll bar per axle.
  - The wheels probe the ground with a ray of their own (`medium_tester` in the Jolt backend), which also finds the ground under a medium.
  - A medium answers a wheel's load over 0.1 s: without that lag, sinkage computed from the last step's raw load made a vehicle bounce on the snow.
  - **Hills** (follow-up): the driver holds a vehicle with no command on the full brake below 0.3 m/s. On a 15% grade (μ = 0.9) it drifts 0.0000 m in 10 s, and the hand brake alone 0.6 mm. Released in neutral it rolls 5.652 m in 3 s against the model's g (sin θ − C_rr cos θ) 5.630 m. A hill start reaches 4.66 m/s uphill in 6 s and rolls back at most 4 mm. Any change of input wakes a sleeping vehicle; before that, a released vehicle stayed asleep on the grade.
  - **The whole state, in real time** (follow-up):
    - `world::snapshot()` copies every body, field, vehicle (description and state, wheels
      included) and contact at one tick, under one lock: 0.18 ms for 1001 moving bodies
      against a 2.1 ms step.
    - `world_binding` publishes it as `physics.snapshot` (data) every `snapshot_every` steps.
    - Named bodies, vehicles and fields get keys of their own: telemetry written from that
      snapshot; vehicle commands, pedals and tuning (`world::set_vehicle`) read from the tree;
      field drag, damping and traction knobs.
    - `set_field` keeps a medium's ruts.
    - The snapshot as JSON (`to_json`, `physics.snapshot_json` every `snapshot_json_every`
      steps) for Python and remote readers. The static bodies are a separate document,
      rewritten only when they change.
    - Per-body settings: `world::get_body` / `set_body` (material, mass, damping, sleeping)
      and `bodies.<name>.*` knobs.
    - Publishing cost:
      - `publish_thread` moves every key write and the JSON off the step;
      - `pull()` skips the tree when nothing under the prefix changed (about 2 µs);
      - static bodies are shared between snapshots;
      - named actors alone take a partial snapshot.
      - Measured medians at 1000 moving bodies on a loaded machine (step about 3 ms): JSON
        costs about 4 ms on the step, and with the publish thread everything costs about
        0.2 ms.
    - The binding no longer took a material's own keys (`friction`, ...) for four more
      materials on every pull. That cost about 0.45 ms a pull.
  - **Ruts** (follow-up): a medium with `ruts = true` keeps a rut map that the wheels press their contact patches into. A Humvee in 0.4 m of snow leaves tracks 0.0945 m deep (it sinks 0.0952 m) and 0.30 m wide (tire 0.31 m), with nothing between them. A second Humvee in those tracks sinks to the same depth and meets 0 N of compaction resistance, against 2690 N for the leader's front axle; the leader's rear axle, in its own front wheels' ruts, meets 97 N. The rut maps are saved, hashed and replayed.
- **P3–P8** are not started.
