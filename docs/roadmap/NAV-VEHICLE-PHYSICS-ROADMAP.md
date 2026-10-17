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
| **P1 — `cvc::physics` + Jolt** (this phase) | The facade, null and Jolt backends, height fields with per-cell materials, triangle meshes, sorted contact events, save/restore, `state_hash`, the floating origin, the state-tree binding | (1) A box dropped on a height field rests at the sampled height ±1 cm. (2) A contact reports the cell's material and μ, and μ decides stick or slide. (3) The same trace twice gives the same hash. (4) 1 thread vs N gives the same hash. (5) Save, restore and re-run give the same hash. |
| **P2 — The vehicle** | `vehicle_desc` with presets; per-wheel Ackermann with a steering actuator; a driver model from `a_cmd` to throttle, brake and gear; aero and rolling forces; wheel-state readback | Envelopes, not curves: skidpad lateral ≤ μg + 10%; stopping distance within ±10% of v²/(2μg); coastdown against the drag and rolling model; the Ackermann relation within 1e-4 rad; steer rate never above its limit; no throttle/brake overlap |
| **P3 — The energy ledger** | Supplied, brake, tire, aero and rolling energy, ΔKE + ΔPE, and the residual, with a battery or fuel source | On a closed drive cycle, \|residual\| ≤ 2% of supplied energy |
| **P4 — The plant seam in `cvc::nav`** | `vehicle_plant`, `plant_kind {kinematic, jolt}`, readback across the boundary, the drift tooling below | Every nav golden and parity test unchanged; below 0.3 g the Jolt plant tracks the kinematic trajectory within 0.5 m over 200 ticks; the adapter test leaves a near-zero residual; the per-tick drift is published |
| **P5 — Physics in the navigator's energy** | Grade cost and tip-over margin (H1), gravity in the governors (H2), slope look-ahead features (H3), measured slip and load as modulators (H4), an energy route cost (H5) | Parity goldens; energy per km drops on a hilly route with path-length growth under a fixed bound; zero entries into infeasible slope |
| **P6 — Bindings and harnesses** | pycvc for the physics world, the vehicle and plant selection | Python reads the same values as C++ |
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
- **P2–P8** are not started.
