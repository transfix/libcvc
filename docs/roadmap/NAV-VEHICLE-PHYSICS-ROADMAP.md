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
| **P2 — The vehicle** (done) | `vehicle_desc` with the Humvee and RoboRacer presets (`inc/cvc/physics/vehicle.h`, `src/cvc/physics/vehicle.cpp`); per-wheel Ackermann from a rate-limited steering actuator (a `WheeledVehicleController` subclass); a driver model from `a_cmd` to throttle, brake and gear; aero (against a flow field's wind) and rolling forces (C_rr from the ground material); wheel-state readback; wheels in a medium field sink to the Bekker depth, ride on the compacted surface and feel its compaction resistance and traction | Envelopes, not curves, all met in `physics_vehicle_test`: skidpad lateral ≤ μg + 10% (6.85 m/s² peak at μ = 0.7, μg = 6.86); stopping distance within ±10% of v²/(2μg) (25.90 m from 20 m/s at μ = 0.8, against 25.50 m); coastdown against the drag and rolling model (15.304 m/s against 15.304); the Ackermann relation within 1e-4 rad; steer rate never above its limit; no throttle/brake overlap. In 0.4 m of snow (k = 10⁶ N/m³) a wheel sinks 0.1057 m, the Bekker depth for its load, and the chassis stands 0.2986 m higher than on bare ground (depth − z = 0.2993); coasting in it decelerates 2.50 m/s² against the model's 2.57 (the field presets' figures). The same drive twice, and a save / restore / re-run, give the same hash |
| **P3 — The energy ledger** (done) | `vehicle_energy` per vehicle per engine sub-step: supplied, idle, brake, tire, aero, rolling, medium, drivetrain, suspension, chassis damping, field work, stored (kinetic, potential, spring), the drivetrain step's split error, and the residual; a battery (regen with a cap) or fuel source; Wh/km; in the snapshot, its JSON and the `vehicles.<name>.energy.*` keys. Measured by copies of Jolt's PostCollide and friction solve that change no bit of the run | All met in `physics_energy_test`: a closed cycle over rolling hills, residual + drivetrain split +4.4 kJ of 640 kJ supplied at 8 collision steps (0.69%); coasting supplies 0 and its stored energy falls by what it dissipates (residual +52 J of 211 kJ); coasting downhill gains -dPE - dissipated (residual +46 J of 62 kJ); regeneration returns at most 70% of the brakes' own energy, and never above its cap |
| **P4 — The plant seam in `cvc::nav`** | `vehicle_plant`, `plant_kind {kinematic, jolt}`, readback across the boundary, the drift tooling below | Every nav golden and parity test unchanged; below 0.3 g the Jolt plant tracks the kinematic trajectory within 0.5 m over 200 ticks; the adapter test leaves a near-zero residual; the per-tick drift is published |
| **P5 — Physics in the navigator's energy** | Grade cost and tip-over margin (H1), gravity in the governors (H2), slope look-ahead features (H3), measured slip and load as modulators (H4), an energy route cost (H5) | Parity goldens; energy per km drops on a hilly route with path-length growth under a fixed bound; zero entries into infeasible slope |
| **P6 — Bindings and harnesses** (libcvc part done) | pycvc for the physics world, the vehicle and plant selection; then GRL-SNAM, grl-snam-dbg and cvcdbg on the plant | Python reads the same values as C++ |
| **P6a — The scene binding** (started) | `world::states`, `pose_tracker` (the last two states, interpolated at the clock's fraction), cvcGL `physics_scene_binding` (nodes and merged-mesh groups); then the city bundle loader in the library (one source for the scene's and the world's static geometry) and Ariadne `physics:` / `body:` specs | A bound node's transform is exactly the interpolated pose; off-thread presents reach the owner thread; removed bodies and destroyed nodes unbind; one writer per actor |
| **P7 — wasm-mt replay** | The libcvc recipe declares Jolt for wasm-mt; a native trace replayed under node | The same `state_hash` native and in wasm |
| **P8 — The differentiable vehicle** | See below | See below |
| **P9 — The plant by default** | The plant plans with each vehicle's own limits and size; a `plant_host` picks the plant by name, in code or the state tree; the demos and cvcdbg drive Humvees on Jolt by default | A sim_world sized with nav's defaults drives within the vehicle's envelope (done); the demos' Humvees reach their goals in Austin, which waits on vehicle-aware routes and P8 |

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
4. **The external-force port:** the differentiable rollout takes a per-agent force, as the
   deployed `drive_step_ext` does, for cvcdbg's comm-network trainer, which replaces its
   point-mass surrogate with this rollout (agreed with the DBG session, 2026-10-19).
   - `bike_control` sums an optional F_ext (drive frame) into the force accumulator and, when
     `steer`, into the steering bias, exactly like `ext_force`. Off is byte-identical.
   - `bike_control_bwd` returns dL/dF_ext. Two forms: a per-step value with the caller's 2x2
     dF_ext/d(o), and a per-substep callback with a callback VJP.
   - A trajectory helper records each step's state; its reverse takes dL/dstate_t at every step.
     Coefficient gradients are optional.
   - The same port on the dynamic plant's path; a torch twin in GRL-SNAM.
   - Done: the bicycle and P8a's `dyn_bike_step` (#728), and P8d's four-wheel `dyn4_bike_step`
     with `dyn4_rollout_ext` (#780). The torch twin covers the first two; its four-wheel path
     has its fixture (`tests/data/dyn4_ext_rollout_golden.json`).
5. **P8d: a two-track (four-wheel) dynamic plant** with its adjoint, beside `dyn_plant` (Joe,
   2026-10-19: "add the four-wheel plant to the roadmap, you build it"). It is full Ackermann for
   the trainers and for cvcdbg. Done: `diff::dyn4_plant` (#731, docs/NAV_TRAINING.md), and its
   external-force port (#780).
   - Wheels: four contact points from the body's pose and the vehicle's geometry, Ackermann
     angles on the front pair from the controller's δ_cmd, and the drive and brake split per
     wheel.
   - The ground per wheel: μ from the material grip field and the terrain height sampled under
     each wheel.
   - Suspension: a spring-damper corner per wheel feeding that tire's normal load, with the
     body's heave, roll and pitch. This replaces P8a's quasi-static longitudinal transfer.
   - Tires: Pacejka per tire within a combined-slip friction ellipse. The stiff lateral solve
     reuses P8a's per-tire secant linearization.
   - Inputs and adjoint: the same (δ_cmd, a_cmd), so `bike_control` and its adjoint are reused,
     plus the external-force port. The VJP is on forward-mode duals, like `dyn_plant`.
   - Parameters from `vehicle_desc`: track, wheel positions, suspension rates and the centre of
     gravity's height, as `dyn_bike_si_of` does for P8a.
   - Gates: a gradient check; the single-track limit (zero track, rigid corners) against
     `dyn_plant`; steady load transfer against the quasi-static formula; μ-split braking yaws
     toward the high-μ side; a bump under one wheel moves its load; the P2 envelopes (skidpad,
     stopping, Ackermann) against Jolt's wheeled vehicle; goldens for a GRL-SNAM torch twin.
   - CPU first. CUDA comes later, as for `dyn_plant`.
6. **P8c:** a differentiable power model and energy in the loss, calibrated to the P3 ledger
   (after P8d, Joe 2026-10-19).

### P8a's vehicle fitted to Jolt (2026-10-20)

`nav_plant::dyn_bike_si_of` now gives the dynamic model the values the Jolt plant's one-tick
residual asks for. Jolt runs of the Humvee and the RoboRacer were fitted on held-out seeds, which
made the twist residual about 11x lower.

- **Tires.** Jolt's lateral friction ramps linearly to its peak at `tire_peak_slip_angle`: a
  cornering stiffness of 1 / alpha_peak per unit load. A Pacejka tire peaking at the same angle
  starts 4.5x steeper. B is now set so that B C = 1.12 / alpha_peak (`kJoltTireStiffness`).
- **Front and rear.** They are alike now (`front_stiffness` 1, was 0.85), since Jolt's tires have no
  load sensitivity.
- **A direct drive** (one gear, no torque converter: the RoboRacer) gets no drive lag and no drag or
  rolling resistance. The driver model's integral trim tracks the commanded acceleration, so the
  command is the achieved acceleration.
- **A geared drive through a converter** (the Humvee) keeps its 0.1 s lag and its resistances.

The P8b gate rerun on Jolt RoboRacers (city, 20 held-out seeds, paired bootstrap; Linux), before
and after:

| training | dynamic-trained, before / after | dynamic - untrained seed, before | after |
|---|---|---|---|
| lr 5e-5, 150 steps | 50.8% / 51.6% | +0.2 [-1.8, +2.2] | +0.9 [-1.0, +3.0] |
| lr 1e-4, 300 steps | 48.5% / 52.1% | -2.1 [-4.2, +0.1] | +1.5 [-1.4, +4.1] |
| lr 2e-4, 300 steps | 43.1% / 53.8% | -7.5 [-10.0, -5.3] | +3.1 [+0.7, +5.5] |
| checkpoints selected on 32 validation seeds | 52.2% / 53.2% | +1.6 [-0.4, +3.6] | +2.6 [+0.7, +4.6] |

- **The point-mass arm and the untrained seed are unchanged:** 51.4%, 49.9%, 22.2%, 52.2% for the
  point mass, and 50.6% for the seed.
- **Training through the dynamic model now helps on Jolt** rather than only hurting less:
  - its validation reach rises steadily over training (0.497 -> 0.526 at step 210), where before it
    fell to 0.41;
  - the trained policy beats the untrained one outside the CI at lr 2e-4 and with checkpoint
    selection.
- **At the default learning rate the change is within the noise:** +0.7 [-1.4, +2.9].

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

### The dynamic model's residual (P8a against Jolt)

`cvc::physics::one_tick_residuals` (`cvc/physics/plant_log.h`) measures the drift the same way for
both models. Each tick, it steps P8a's `diff::dyn_plant` and the kinematic bicycle from Jolt's own
state, with the command nav held over the tick. The models use the tick's substeps (`nsub = k`) and
are compared with where Jolt was at the tick's end. The bins are the lateral acceleration Jolt
measured over the tick.

Ticks are left out when they have a static contact, a wheel off the ground, an impact (above
1.5 g), or the vehicle at rest. `cvc plant-log` runs it and prints the bins.

Below: nav's 0.06 s tick (6 substeps), 4 seeds, 60 s each, mu 0.9, measured 2026-10-20. Each cell
is the position rms in mm, three ways:

- the dynamic model with the spec-sheet values it had before the fit (#753);
- the dynamic model with `dyn_bike_si_of`'s values now, fitted to Jolt;
- the kinematic bicycle.

The RoboRacer's city runs plan with 0.4 of its acceleration and braking: at its full 6 m/s^2 its
chassis box pitches onto the ground.

| lateral g | Humvee slalom | Humvee city | RoboRacer slalom | RoboRacer city |
|---|---|---|---|---|
| 0.0-0.1 | 2.0 / 1.2 / 3.9 | 2.0 / 1.6 / 3.4 | 0.9 / 0.3 / 0.7 | 1.9 / 1.3 / 1.9 |
| 0.1-0.2 | 5.1 / 1.4 / 11.6 | 4.8 / 1.6 / 11.1 | 3.3 / 0.4 / 5.6 | 3.4 / 0.9 / 5.2 |
| 0.2-0.3 | 7.1 / 1.3 / 17.9 | 7.0 / 2.0 / 18.1 | 5.0 / 0.6 / 10.7 | 4.8 / 1.0 / 8.6 |
| 0.3-0.5 | 8.1 / 1.2 / 27.5 | 8.1 / 2.8 / 26.2 | 6.9 / 0.8 / 15.8 | 6.5 / 1.8 / 11.7 |
| >= 0.5 | 5.2 / 4.2 / 94.0 | 4.5 / 3.8 / 52.0 | 6.5 / 0.7 / 25.1 | 12.6 / 11.5 / 30.4 |

- **Position and heading:** the dynamic model is closer than the kinematic bicycle wherever the tires
  carry load. Below 0.5 g the fitted values bring it 1.3-9x closer again.
- **Speed:** below 0.5 g the RoboRacer's speed residual now matches the kinematic bicycle's,
  0.001-0.035 m/s rms (it was 0.018-0.090). Two spec-sheet values had been off:
  - its 0.1 s drive lag;
  - the drag and rolling resistance it subtracted. The RoboRacer's driver model tracks the commanded
    acceleration with an integral trim, so it makes up for them.

  The Humvee's speed residual is unchanged, and below the kinematic bicycle's.
- **What is left:** the RoboRacer in the city above 0.5 g (11.5 mm), and its yaw rate there,
  0.04-0.34 rad/s rms.

**Fitted to Jolt** (`cvc::physics::fit_dyn_bike`, `cvc plant-log --fit-dyn`): a coordinate descent on
the one-tick twist residual over the knobs a spec sheet does not give. The knobs are the drive and
steering lags, factors on the tires' B, the yaw inertia, and drag + rolling resistance. Each fit uses
seeds 100-102 and is judged on seed 103.

The first fits ran against the spec sheet: tire B x0.25-0.40, and for the RoboRacer no lag and no
resistance. #753 adopted them as `dyn_bike_si_of`'s values. Refitted against those values:

| run (50 Hz) | drive lag | tire B front / rear | yaw inertia | held-out twist residual, (m/s)^2: default -> fit | before #753 |
|---|---|---|---|---|---|
| Humvee slalom | 0.20 s | x1.45 / x1.20 | x1.00 | 0.00046 -> 0.00049 | 0.00287 -> 0.00045 |
| Humvee city | 0.04 s | x1.00 / x1.00 | x1.00 | 0.00065 -> 0.00061 | 0.00280 -> 0.00060 |
| RoboRacer slalom | 0 | x1.00 / x1.00 | x0.85 | 0.00001 -> 0.00001 | 0.00035 -> 0.00001 |
| RoboRacer city | 0 | x1.00 / x1.20 | x1.75 | 0.00015 -> 0.00018 | 0.00229 -> 0.00018 |

- **Nothing left to fit on flat ground:** on the held-out seed the defaults do as well as a fit. A
  refit gains at most 6% (Humvee city) and loses up to 20% (RoboRacer city). The tire factors come back
  at x1, except the Humvee slalom's, whose fit does worse held out. No steering lag is found, and drag
  and rolling stay as they are.
- **Weakly identified:** the Humvee's drive lag (0.04-0.20 s) and either vehicle's yaw inertia move
  between scenes.
- **The city-bundle fits (Austin) are not trustworthy:** they land on the grids' edges (B x0.3-0.7, yaw
  inertia x1.75-2.1), absorbing what the planar model leaves out on rough terrain.
- **Held out, at nav's 0.06 s tick:** position rms in mm, as fit / default now / spec sheet before #753
  / kinematic:
  - Humvee slalom, above 0.5 g: 2.7 / 3.4 / 6.4 / 55.2;
  - RoboRacer city, 0.3-0.5 g: 1.1 / 1.1 / 6.1 / 11.2.
- **The LieSPHGP datasets do not move:** Jolt drives the vehicles, and `dyn_bike_si_of` enters only
  the residuals. The rerun's flights and pickles are byte-identical.

These are the targets a plant fitted to Jolt has to beat, bin by bin.

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
  - **Hills** (follow-up): the driver holds a vehicle with no command on the full brake below 0.3 m/s. On a 15% grade (μ = 0.9) it drifts 0.0000 m in 10 s, and the hand brake alone 0.6 mm. Released in neutral it rolls 5.685 m in 3 s against the model's g (sin θ − C_rr cos θ) 5.662 m. A hill start reaches 5.57 m/s uphill in 6 s and rolls back at most 49 mm (the field presets' figures). Any change of input wakes a sleeping vehicle; before that, a released vehicle stayed asleep on the grade.
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
  - **Ruts** (follow-up): a medium with `ruts = true` keeps a rut map that the wheels press their contact patches into. A Humvee in 0.4 m of snow leaves tracks 0.1062 m deep (it sinks 0.1060 m) and 0.30 m wide (tire 0.32 m), with nothing between them. A second Humvee in those tracks sinks to the same depth and meets 0 N of compaction resistance, against 3391 N for the leader's front axle; the leader's rear axle, in its own front wheels' ruts, meets 127 N (the field presets' figures). The rut maps are saved, hashed and replayed.
- **P3** is done (above): `physics_energy_test`, every gate met on Windows.
  - The vehicle includes its engine (D8): `supplied` is the engine torque's work, and the idle
    speed's clamp is its own input. So coasting with no input supplies exactly nothing.
  - Jolt's drivetrain step is implicit and split from the friction solve: it solves engine,
    clutch and wheels with an estimate of the brake and ground torques, then undoes the
    estimate. Every flow is charged at the step's end speed, so clutch slip is never negative,
    and the energy the split makes (dL tau / I a wheel) is its own term, `drivetrain_split`.
    It is proportional to the collision step. The gate is on residual + split at the step a
    twin runs at: 0.69% for the Humvee at 8 steps (the residual alone is 0.01%).
  - **The presets against the field** (follow-up): every value in `vehicle.cpp` names its
    source (AM General's sheet, the TM, the Army's dynamometer tests of the 6.5 L, the lab's
    configuration).
    - New powertrain terms:
      - a net torque curve;
      - a motoring-friction curve that acts only with the throttle shut;
      - a torque converter (its coupling, its capacity and its torque multiplication);
      - the driveline's efficiency;
      - a throttle-dependent shift schedule on the gearbox input's speed.
    - The driver model gained an integral trim.
    - Fuel burns on the dynamometer's Willans line, and none on the overrun.
    - The M1097A2 at GVW makes 0-30 / 0-50 mph in 8.9 / 23.6 s, against the Army's 9.4 /
      26.1 s.
    - Its range comes out 596-600 km against the TM's 442 km (bias-ply; about 490 km on
      radials). This gap is open, and needs a coast-down and fuel-flow measurement.
    - The drivetrain's share of the engine's work at cruise fell from about half to 20%.
    - The RoboRacer, with the lab's 15.15:1 gearing, needs 32 collision steps for a 1% energy
      error.
  - `nav_stats` gains `energy_J` / `energy_per_km` with P4's plant seam, where the navigator
    first reads a physics plant.
- **P4** is done: `nav_plant_test`, every gate met on Windows (PHYSICS_API.md, "cvc::nav's
  plant").
  - `cvc/nav/vehicle_plant.h`: `vehicle_plant`, `plant_kind {kinematic, jolt}`, and
    `kinematic_step`, nav's bicycle as the reference.
  - `sim_world::set_plant`. The drive's plant arm (`veh_params::plant`) computes the command
    without integrating it, with D14's plant-aware governors. The kinematic arm is untouched.
  - `cvc::physics::nav_plant`:
    - the rear-axle readback across the SI boundary;
    - the command through the driver model;
    - static contacts and energy for `nav_stats` (`energy_J`, `energy_per_km`,
      `building_contacts`);
    - the drift, binned by lateral g and published beside `vehicles.<name>.*`.
  - A configure-time check keeps `cvc/physics` out of nav's sources.
  - Measured:
    - a curving route's path within 0.03 m of the bicycle's;
    - the adapter's residual at most 7 mm a tick;
    - at a wall, the speed at most 0.04 m/s over the latency-aware `v_stop`, with no contact;
    - about 0.1 ms a Humvee a tick.
  - What P4 found in the plant, fixed in the presets' commit:
    - Jolt's idle clamp crept the Humvee forward through its brakes; a torque converter
      (capacity and multiplication) fixes it.
    - The driver model gained an integral trim.
  - Open:
    - a step change of goal or a swerve at 0.25 g leaves the Humvee 0.6-1.2 m off the
      bicycle's path, from its 0.35 rad/s steering. That is reported, not gated;
    - the lag-aware governors could also take the steering rate into the corner speed.
- **P5** is done: `nav_terrain_test` (kinematic) and `nav_plant_test` (Jolt), every gate met on
  Windows (PHYSICS_API.md, "Terrain in the navigator's energy").
  - `cvc/nav/terrain.h`: the slope, the grade cost and the infeasible cells from a height raster
    and the vehicle's limits; `sim_world::set_terrain`; `nav_plant::terrain_config_of`.
  - H1: the grade cost through the material pipeline, a soft force and a hard barrier; the
    infeasible cells plan as obstacles.
  - H2: gravity along the path in the stopping governor (never counting on its help uphill)
    and across it in the corner cap; on the Jolt plant the driver model feeds the contact
    normals' grade forward.
  - H3: the slope look-ahead feature and coefficient-net format v3.
  - H4: `plant_state` gains tire use, load transfer, pitch and roll; `plant_modulation` scales
    gamma and the hard barriers; the gains are the plant's.
  - H5: `astar`'s climb cost, a metre of rise weighed as 1 / C_rr metres of flat.
  - Measured on the Jolt plant:
    - over a 12 m hill, 7826 kJ/km against 15403 for a path 1.44 × as long;
    - 0 ticks on a 1.2 ridge;
    - round a 30 m circle on μ 0.7, mean tire use 0.79 against 1.36 with the modulator.
  - The state tree:
    - `sim_world` binds its own settings (`sim_world::pull` / `publish`) under `nav.*`: sensing,
      belief, the crowd, the vehicle's scalars in metres, the material weights and the terrain.
    - `nav_plant` binds the modulation knobs (`nav_plant.modulation.*`) and each vehicle's measured
      state.
    - `nav_stats` counts entries onto infeasible slope.
    - Key metadata is one shared form, `cvc/state/state_key_meta.h`.
  - Open: GRL-SNAM's Python twin of H1/H5; a net retrained on the slope feature; H3 in the CUDA
    drive.
- **P6, the libcvc part, is done:** pycvc for the physics world, the vehicle and plant
  selection (PHYSICS_API.md, "Python: pycvc").
  - `bindings/pycvc/pycvc_physics.i`: `physics_world`, `vehicle_desc`, `physics_binding`
    (world_binding), `nav_plant`, and the sim_world's plant, terrain and tree functions.
    pycvc's state helpers read a key's comment and properties.
  - The gate, in `test_pycvc_physics.py` against `physics_parity_test`:
    - Python rebuilds the cross-platform golden scene and gets `70b2cc0bd34ea4be`;
    - a Humvee through the driver model and an agent on the Jolt plant match their C++ twins
      bit for bit (3 state hashes, 22 values);
    - a vehicle's state, the snapshot JSON and the tree agree.
  - Left of P6, downstream once a libcvc release carries P4–P6:
    - GRL-SNAM `Squad(plant=…)` and the `nav_native` plant option;
    - grl-snam-dbg `gen-episodes` / `eval` on Jolt, a `KinematicsRecord` schema bump and
      energy in the scorecards;
    - cvcdbg `--plant` / `--vehicle` / `--clock` and the HUD.
- **P7 done.** A native trace gives the same `state_hash` at every step in wasm-mt under node.
  - The trace is 1240 steps: the golden scene, a Humvee, and a RoboRacer in mud and by water
    (`physics_replay_test`, CI job `physics-wasm`).
  - The wasm-mt publish now installs `joltphysics` (the recipe already declared it), and fails if
    the build loses Jolt.
  - Getting there took two fixes. The first: Jolt's wheels hashed uninitialized contact fields
    until they first landed. The second: libm's tan, atan, tanh, expm1, pow, sin, cos, atan2 and
    asin, which differ by an ULP between libms, were replaced by `det_math.h` in everything that
    feeds the hash.
- **P8** is in progress.
  - P8b's first item is merged.
  - P8a, the dynamic vehicle, is done: `dyn_bike_step` and `bike_control` (docs/NAV_TRAINING.md).
    It passes its gates: gradient checks, the kinematic limit against `bike_step`, the skid
    modes and the stopping envelope. GRL-SNAM's torch twin matches its goldens to 1e-9
    (CVC-Lab/GRL-SNAM#121).
  - P8b, in progress:
    - `coef_train`'s `rollout_kind::dynamic` (CPU): gradient-checked; training through it is
      stable and lowers the loss (docs/NAV_TRAINING.md).
    - `material_train`'s `material_integrator::dynamic` (`integrate_dynamic_material`): its
      rollout VJP, `L_multi` and the full loss-to-weights chain are gradient-checked.
    - The gate on the Jolt plant, over 20 paired seeds of RoboRacers in the city:
      - Trained alike at lr 2e-4 for 300 steps, the dynamic-trained policy reaches 44.0% and the
        point-mass-trained one 35.0%: +9.0 points, 95% CI [+6.7, +11.4].
      - But both fall below the untrained seed's 51.7%. At the stable 5e-5 they tie (+0.4, CI
        [-1.8, +2.8]).
      - So the dynamic model makes training hurt less on the real vehicle; it does not yet make it
        help.
    - Checkpoint selection on Jolt reach (`coef_trainer::train(scene, verbose, fitness)`,
      `cvc::physics::reach_on_plant`):
      - Each training ends on its checkpoint with the best reach on 8 validation seeds, the
        untrained policy included.
      - Held out, the dynamic-trained policy reaches 52.7% and the point-mass-trained one 50.1%:
        +2.6 points, CI [+0.3, +4.7]. The seed reaches 51.7%.
      - So both now stay near the seed, and the gate's wording holds there.
      - The dynamic one leads the seed by +1.0, CI [-1.2, +3.4], so not significantly.
      - Jolt reach is deterministic but chaotic: a 4e-9 change of scale moves a seed's reach by up
        to 0.1, so validation needs 8 or more seeds.
    - Lower peak rates with denser checkpoints (1e-4 every 10 steps, 5e-5 every 20, and 2e-4
      every 5; 31 checkpoints each, seeds now scored in parallel):
      - The best, 1e-4 every 10, leads the seed by +1.9 [-0.8, +4.4]. No setting beats it
        outside the CI.
      - At the lower rates validation reach stays flat (0.445 to 0.523 against the untrained
        0.479), and the extra checkpoints mostly add selection noise.
      - The dynamic training beats the point-mass one in all four settings (+1.1 to +2.6),
        outside the CI in two.
    - 32 validation seeds (now the default): at 1e-4 every 10, the held-out leads shrink to
      +0.6 [-1.3, +2.7] over the seed and +0.3 [-2.2, +3.0] over the point-mass policy; the
      8-seed +1.9 was mostly selection luck. At 2e-4 every 30 the same checkpoints win.
      Training's real effect on Jolt reach is about a point at most.
    - Next: a trained policy that beats the seed outside the CI. Neither the step size nor the
      selection noise is the limit; the training signal is. Lever: Jolt reach as the training
      objective (a zeroth-order fine-tune of the selected policy on `reach_on_plant`), or a loss
      closer to it.
    - Zeroth-order fine-tuning on Jolt reach (`cvc::physics::es_finetune_on_plant`):
      - ES over every weight, and over the basin alone, found nothing. The trained policy sits
        near a local top, and ES steps move reach less than the plant's chaos.
      - A grid over the constant basin did: (alpha, beta, gamma) = (1, 9, 4) reaches 55.4% held
        out against the seed's 51.7%, +3.8 [+1.9, +5.7]. It is one grid step from a cliff where
        a goal spring too strong for the damping collapses reach to about 20%.
      - So the policy's leverage on Jolt is mostly its basin. A learned, state-dependent policy
        that beats the seed is still open.
  - P8d, the four-wheel plant, is done: `diff::dyn4_plant` passes its gates (#731,
    docs/NAV_TRAINING.md), and takes the external force (#780). Open there: the anti-roll bars,
    the unsprung masses and wheel spin.
  - The external-force port is done on every plant: the bicycle and P8a's (#728), and the four
    wheels (#780). Next (Joe 2026-10-19): P8c.
- **P9, the plant by default (2026-10-18, Joe: "can't we use the actual jolt vehicle dimensions
  and physics?").**
  - What moved the agents: no default path used Jolt. Every default was the kinematic bicycle
    or a point mass, sized at the trained metric's numbers: at Austin's 0.05 per metre, a 3 m
    disc, 18 m/s and 30 m/s^2.
  - In libcvc:
    - `vehicle_desc::max_speed`, which the driver model holds (Humvee 24.6 m/s, RoboRacer
      10 m/s);
    - `nav_plant` gives `sim_world` each vehicle's acceleration, top speed, cornering limit and
      body (`plan_with_vehicle`, on by default);
    - `plant_host` chooses the plant by name;
    - `nav_plant` backs up from rest when nav asks, rather than braking against its springs'
      jitter;
    - the drive sends a plant only what its own integration would apply (`allow_reverse`,
      `vmax`);
    - the cvcGL nav demos take `--plant`.
  - **The demos' default stays the bicycle, for now.** The Austin finale's Humvees drive their
    routes, then wedge in a corner after act 2 replans them: upright, at full lock, unable to
    turn a 6.7 m circle out of it. That happens with the plan off too.
  - `nav_material_demo` shows the same gap. The material layer keeps the bicycle out of the lake,
    but the Humvee enters it 65 times.
  - Vehicle routes (R1-R3):
    - `plan_vehicle_route`, a Hybrid A* over the vehicle's turning circle, body and reverse
      gear, with multi-point turns;
    - `route_follower` drives them: the trained drive on the last forward leg, pure pursuit
      on manoeuvres;
    - `nav_finale` uses them on a plant. Austin's 8 Humvees drive 5387 m in 900 frames (1903 m
      on A* routes) and none wedges (7 of 8 did).
  - Next, before the default flips: training through the vehicle (P8), and the material layer
    on the vehicle.
  - Training stays differentiable: its surrogate's parameters should come from the
    `vehicle_desc` until P8a's `dyn_bike_step` exists.
