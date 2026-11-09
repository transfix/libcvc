# Nav CoPH Roadmap: cooperative stagewise port-Hamiltonian learning on Ariadne and in `cvc::nav`

> **Goal:** the cooperative agents of CoPH-Stagewise
> ([SetasAditya/CoPH-Sense](https://github.com/SetasAditya/CoPH-Sense), `CoPH-Stagewise/`) run
> in mainline GRL-SNAM and appear on Ariadne in two forms: first in their own worlds, then driving
> the cars template's Ackermann cars. Afterwards they get a C++ twin in `cvc::nav` that makes the
> same fits, the same commit decisions and the same packets as the Python. The method then runs
> without torch in pycvc-gl, both on the desktop and in the browser, and over cvcdbg's radios.

**Status:** spec, 2026-11-09. Nothing is implemented. The decisions in
[§10](#10-decisions-settled) were settled on 2026-11-09: Joe took the recommended column for all
twelve ("Yes, take them all"), which is what the spec assumes. The Python reference is CoPH-Sense at
`c1f1ff3` (2026-11-07). That is five commits old and still moving, so the vendoring step pins a
commit (§2).

Related:
- [`NAV-BELIEF-MATERIAL-CONTROLLER-ROADMAP.md`](NAV-BELIEF-MATERIAL-CONTROLLER-ROADMAP.md): the
  twin pattern this spec follows. The Python is the reference, GRL-SNAM writes the goldens, and the
  C++ is pinned to them. Its `nav_visible_obstacles` builder is reused in §3.3.
- [`NAV-VEHICLE-PHYSICS-ROADMAP.md`](NAV-VEHICLE-PHYSICS-ROADMAP.md): the plants the cars drive.
- [`../CARS.md`](../CARS.md): the cars template, `controller(s, p)`, `cost()` and `coeffs`.
- Headers: `inc/cvc/nav/sim_world.h` (`override_command`), `inc/cvc/nav/route_follower.h`
  (another driver that steers through `override_command`), `inc/cvc/nav/plant_ledger.h` and
  `inc/cvc/nav/nav_stats.h`.
- Upstream docs: `docs/COOPERATIVE_STAGEWISE_PH_FORMULATION.md` and `docs/RESEARCH_STATUS.md`,
  plus `CoPH-Stagewise/CROSSING_DEMO.md` and `HAZARD_DEMO.md`.

## Why

CoPH-Stagewise learns a navigation energy online, one stage of the route at a time. It keeps the
last few stages and revises them as new evidence arrives. It commits a revision only when the
revision passes validation, and it books every energy jump a revision causes apart from the
physical work. Agents share what they learned as small packets: field anchors with gradients, a
covariance and provenance. A receiver fuses a packet into its own fit only under its own commit
gate.

The reference is a torch program. It drives a 2-D point in worlds of disks and slabs about 12 m
across. Our stack already has what it takes to show the method at vehicle scale:
- the cars template: N Ackermann cars, a Python controller called every tick, and Austin;
- `sim_world::override_command`;
- the plant energy ledgers;
- belief planes with fog;
- cvcdbg's radio network.

Five things are missing:

1. the method in a maintained, tested, released package (GRL-SNAM);
2. a bridge from a fully actuated point to a car: the force CoPH asks for has to become a steer
   and an acceleration;
3. a world bridge from occupancy and the material raster to CoPH's obstacles and risk readings;
4. a message layer in libcvc, with no dependency on cvcdbg;
5. an implementation without torch, for the browser and for cvcdbg.

## 1. The reference in one page

All values are SI, float64, unit mass.

| Piece | Python (`coph_stagewise/`) | Defaults |
|---|---|---|
| Plant | `model.flow`: `q' = v`, `v' = -d v + u`, an exact zero-order hold | `dt` 0.12 s |
| Energy balance | `model.balance`: work, heat, ΔKE, residual = ΔKE − work + heat | residual about 1e-15 per step |
| Risk field | `model.risk`: `r(q) = softplus(Φ(q) w) / 4`, with Φ a 4 × 4 grid of Gaussian RBFs over the stage, plus an optional compact Wendland residual | RBF width 0.8 m |
| Potential | `model.potential`: `½|q − g|² + π₀ r(q) + π₁ Σ_k softplus(6 (0.35 − c_k))² / 36`, where `c_k` is the distance to obstacle k minus its radius and the 0.12 m body; mode `progress` scales the shaping by a bounded α | `π` starts at (1, 2), box [0, 10]² |
| Command | `model.command`: `u = −∇V − 1.4 v`, scaled down to `|u| ≤ 3` | force limit 3 |
| Stages | `learner.advance`: 3 × 4 m stages with 30 % overlap (a step of 2.1 m); the current stage plus the previous 2 are kept | `previous` 2 |
| Field fit | `learner._field_fit`: ridge toward a prior over the observations of the kept stages, with continuity rows in the overlaps | ridge 0.02 |
| Damping fit | `learner._fit_damping`: Newton steps on the plant loss, held within a trust region and bounds | 8 steps, ±0.15, [0.15, 1.5] |
| Policy update | `learner.replay` and `update`: replay from stored stage entries, a gradient proposal projected onto the box and the trust region, then halving | 8 replay steps, ±0.25, 8 halvings |
| Commit | `learner.update`: accept only if validation worsens by less than the slack, else roll back | slack 0.003; every 8 ticks |
| Sensing | point readings of the risk field and obstacles within range | radius 1.25 m |
| Packets | `cooperative.Packet`: pose, energy, version, goal, 4 anchors `(q, value, ∇)`, their joint covariance from an 8-member block bootstrap, sources and stage; its size is its compact JSON | ttl 2 s (300 s in the demos) |
| Receiver | `cooperative.assimilate` checks and stages a packet. The next `update` adds one Gauss–Newton step, whitened by the packet's covariance, to the ridge normal equations, under the receiver's own gate | cap 8 received |
| Channel | in-process; `delay_ticks`, no range model | delay 0 |

The two demos use one configuration profile:

```python
Config(controller_mode='routed', shaping_mode='progress', replay_objective='mission',
       replay_domain='retained', replay_horizon_mode='supported_boundary', replay_update_cap=1024)
CommunicationConfig(byte_budget=4000000, ttl=300.)
```

The C++ twin implements this profile and refuses the others (decision D7).

**What the reference establishes**, per its own documents:
- **Crossing.** In `crossing_demo`, sharing what an agent has seen saves the other agent's route.
  All 16 missions (2 seeds × 4 arms) reach their goals with no collisions. The reciprocal saving is
  27.1 m, against 28.0 m predicted in advance.
- **Hazard.** In `hazard_demo`, a shared graded field lets the receiver apply its own risk
  preference. The carrier's worst-case excess cost is 4.6 against 68.1 with no communication.
  Raw readings tie it at more bytes, and so does the arm with no receiver fitting.
- **Ledger.** The energy ledger closes.

**Not established:**
- any benefit from replay;
- any benefit from receiver field fitting (the hazard tie above);
- any benefit from adaptive-rate sharing.

The demos and the docs built from this roadmap headline only the established results (§8).

## 2. Pipeline

```
 sense (risk readings, obstacles in range) ──► stage ledger: current + 2 previous stages
                                                   │ every 8 ticks
            received packets ──► assimilate ──► field fit (ridge + Gauss-Newton) ──► damping fit (Newton)
                                                   │
                                     replay ──► policy proposal ──► project, halve ──► validate
                                                   │ accept: commit, book the shaping jump
                                                   │ reject: roll back
 command  u = -∇V - 1.4 v, |u| <= 3 ──► allocate (car: steer, accel; booked residual) ──► plant
                                                   │
                                export packets ──► channel (ideal | cvcdbg radio) ──► peers
```

## 3. GRL-SNAM: `grl_snam.coph`, the demos and the goldens

### 3.1 Vendoring (T1)

- **What.** Copy the method's core and the two demos' scenarios from a pinned upstream commit into
  `grl_snam/coph/`, along with the tests that cover them. That is about 15 modules and 2.3k lines:
  `model`, `records`, `learner`, `cooperative`, `navigation`, `stage_prediction`,
  `replay_budget`, `route_field`, `world`, `scenarios`, `shapes`, the maze geometry, and the
  crossing and hazard demos. The demos' full static import closure is 65 modules and 10.4k lines,
  because their report sections pull in the reporting and campaign suites.
- **License.** CoPH-Sense is MIT, like GRL-SNAM. Each file keeps its copyright line.
  `grl_snam/coph/VENDORED.md` records the commit and every local change.
- **Untangling.** Two couplings get cut, and both changes are recorded upstream as suggestions:
  - `CooperativeLearner` subclasses `diagnostics.InstrumentedLearner`. Its hooks become an optional
    mixin, so the core does not carry the diagnostics suite.
  - `route_field` imports `terrain_warning` when it loads. That import becomes lazy.
- **Trimming.** Reporting, campaign and figure modules that the two demos import only for their
  report sections stay out. The CLI writes the demos' `results.json` without them.
- **Dependencies.** Torch, numpy and matplotlib, all of which GRL-SNAM already has.
- **CLI.** `grl-snam coph crossing|hazard --out DIR [--seeds N] [--steps N]`.
- **Gate.** At the pinned commit, the vendored demos reproduce upstream's per-arm results
  (`results/*/results.json`):
  - exactly, on the same OS and torch version;
  - otherwise, the same goal and collision counts, a reciprocal saving within 0.5 m and a
    worst-case excess within 0.5.

### 3.2 `grl-snam ariadne coph`: CoPH's worlds in 3-D (T3, demo 2a)

This demo runs the vendored crossing or hazard episode live, or replays one from a log, in the
Ariadne viewer. It uses the host that `ariadne cars` and `fleet` already use. Because the physics
is CoPH's own, the demo shows only what the reference establishes.

- **The world.** A flat world, scaled for display:
  - slabs as boxes, and the doors as gaps that appear once an agent senses them;
  - disks as cylinders;
  - the risk field draped as a heat map, toggled between the truth and the selected agent's
    current `r`;
  - each agent's three stage rectangles, coloured by age;
  - its sensor disc.
- **Agents.** Each is drawn as a RoboRacer mesh at the point's position, heading along its
  velocity. The mesh is for display only.
- **Packets.** Each packet is an arc from sender to receiver, drawn at delivery: green if the
  receiver's gate accepted the update that used it, red if not. The arc's width shows its bytes.
- **Panels**, per agent (`.ari` panels like `cars_panels.ari`):
  - the ledger: work, heat, ΔKE, shaping jumps, residual;
  - version;
  - `π₀`, `π₁`, `d`;
  - accepts and rejects with their reasons;
  - bytes sent and received.
- **Arms.** Communication off and CoPH run side by side (two panes), with the same seed.
- **Headless.** `--log run.npz`, in the cars template's log shape plus the ledger columns.
- **Gate.** The per-arm outcomes equal T1's. It is the same code, and the viewer only reads it.

### 3.3 CoPH drives the cars (T4, demo 2b)

`grl_snam/ariadne/coph_cars.py` is a cars-template controller. Each car carries one
`CooperativeLearner`. Each tick it:
- reads `s` (positions, heading, speed, steer);
- maps the state into CoPH's frame (§3.3.1);
- runs CoPH at CoPH's own cadence (D6);
- allocates the planar force to a steer and an acceleration (§3.3.2);
- returns `{"coeffs": {"pi_risk", "pi_barrier", "damping", "version"}}`.

`cost()` draws the selected car's `V`, or its `r`, as the raster.

#### 3.3.1 Scale

Froude scaling (D4) with a factor `s`: lengths × `s`, times × `√s`, speeds × `√s`, accelerations
unchanged. CoPH's force limit (3 m/s²) then stays on the order of a vehicle's `a_max`.

| | `s` | stage | sensor | speed (about 1 m/s in CoPH) | CoPH step |
|---|---|---|---|---|---|
| RoboRacer, flat or generated world (first, D3) | 1 | 3 × 4 m | 1.25 m | 1 m/s | 0.12 s |
| Humvee, Austin | 25 | 75 × 100 m | 31 m | 5 m/s | 0.6 s |

#### 3.3.2 The Ackermann allocation

CoPH commands a planar force `u`. A car accepts an acceleration along its heading `ĥ`, plus a
steer that sets the lateral acceleration `v² tan δ / L`. With `n̂` the left normal:

```
a      = clamp(u·ĥ, −a_max, a_max)
κ*     = (u·n̂) / max(v², v_min²)
δ      = clamp(atan(L κ*), −δ_max, δ_max)
u_used = a ĥ + (v² tan δ / L) n̂
e      = u − u_used                      (the unallocated force)
```

- **What the learner sees.** It observes `u_used`, the force the car actually took (D5), just as
  it observes the clipped force today.
- **The ledger.** It gains one term, the unallocated power `e · v` and its integral, next to the
  existing clipping. At low speed almost all of the lateral force is unallocated, and the ledger
  shows it.
- **CoPH's state.** CoPH's velocity state is `v ĥ`. The heading and steer angle are hidden from
  it, and so is the steer rate (`s.steer` lags the command, per `CARS.md`).
- **Damping.** The damping fit absorbs whatever part of the car's longitudinal dynamics a linear
  drag explains.

The formulation treats a fully actuated point, so this allocation is new research, not plumbing.
2b reports what happens and claims nothing until it is measured.

#### 3.3.3 The world adapter

- **Obstacles.** `pycvc.nav_visible_obstacles` (the belief controller's builder) gives each car the
  disks it sees within its sensor radius: centres, radii and weights. Scaled, these become CoPH's
  disk list, capped at `obstacle_cap` (256). A building becomes many disks.
- **Risk readings.** A car reads the material raster's risk channel at its position: the belief
  plane `r~` when the world has fog, the truth otherwise. This is CoPH's hazard reading.
  Unseen material patches are the hidden gates and risk patches of CoPH's scenarios.
- **Goal.** CoPH's goal well is `½|q − g|²`. Far from the goal, that term dominates every other.
  On long routes the goal is therefore clipped to a local goal 30 m along the straight line,
  the same rule as decision E2 of the belief controller's roadmap (D12).

#### 3.3.4 Arms and logs

The arms are communication off, CoPH packets, and raw readings. The run is headless (`--headless
--log`) over paired seeds. The summary reports:
- arrivals, path length, collisions;
- excess risk cost against the best arm;
- bytes;
- the ledger, including the unallocated power.

### 3.4 Goldens (T5)

`python -m grl_snam.tools.coph_golden --out DIR` writes `coph_golden.json`. Every double is
written as its raw bits (a uint64, in hex), so the file is the same bytes on every run and OS. The
file is committed in libcvc `src/cvc/tests/data/`. A GRL-SNAM test regenerates it and compares the
bytes with `tests/fixtures/`, as `belief_material_controller_golden` does. The cases are in §5.

## 4. The C++ twin (`inc/cvc/nav/coph*.h`)

Every PR is additive and default-off. Everything is double precision, as in the Python. Where a
tier says BIT, the TU builds with `-ffp-contract=off`.

### 4.1 Records and the stage ledger (C1)

- **`coph_config`.** The fields of `Config` and `CommunicationConfig` that the profile uses, with
  the same defaults. Validation mirrors `__post_init__`, with the same refusals. A field or mode
  outside the profile throws (D7).
- **`coph_stage`.** It holds:
  - id, centre, extent and the RBF centres;
  - observations, transitions and anchors with their caps (240, 120, 12);
  - the entry state;
  - the field weights and the local residual.
- **`coph_ledger`.** A ring of the current stage and the previous `B` (2). The advance rule steps
  `width × (1 − overlap)` and builds the overlap rows.

### 4.2 Plant, potential, command and energy (C2)

- **`coph_flow` and `coph_balance`.** Closed forms with `std::expm1`, which handle a small `d`
  exactly as `torch.expm1` does.
- **`coph_potential<T>` and `coph_grad_potential<T>`.** Analytic `∇V`: the goal term, the RBF and
  Wendland gradients through softplus, the disk barrier, and `progress` shaping with its α bound.
  They are templated on the scalar, so the same code runs on `double`, on `dual<double, 2>` for
  replay (§4.3) and on a hyper-dual for the damping Newton step. This replaces torch autograd.
- **`coph_command`.** `−∇V − 1.4 v`, scaled to the limit. It returns both the applied force and
  the proposed one, as the Python does.
- **`coph_energy`.** Per agent: work, heat, ΔKE, the shaping jumps (`V_new(x) − V_old(x)` at each
  commit), the allocation residual (§3.3.2) and the balance residual. Its items join
  `plant_ledger` and apply only when CoPH is attached (D11).

### 4.3 The learner (C3)

- **Field fit.** Build the design matrix over the kept stages, with the overlap continuity rows,
  and solve the ridge normal equations with Eigen's LLT. The Python uses an LU solve, so the tier
  is FLOAT.
- **Damping.** Newton steps on the 1-D plant loss. The first and second derivatives come from
  hyper-duals. The step is clipped to the trust region and the bounds.
- **Replay.** Run 8 steps of command + flow on `dual<double, 2>`, one direction per policy gain.
  That yields the cost and its gradient in one pass. The `mission` objective, the collision and
  goal ends, the retained-stage domain and the `supported_boundary` horizon with its update cap
  follow `stage_prediction.retained_replay` and `replay_budget`. Eligibility reasons
  (`unsupported_entry`, `domain_exit`, ...) are BIT.
- **Update.** The proposal, the projection onto the box and the trust region, up to 8 halvings,
  the validation and feasibility checks, the commit with its shaping jump, or the rollback.
- **Routed controller.** `controller_mode='routed'` executes the route field's attraction. The C++
  takes the route field's anchors as input from C1's records. `route_field`'s anchor mode is
  ported; the `additive_routing` and `localized_receiver` paths it imports are not (D7).

### 4.4 Packets and the channel (C4)

- **`coph_packet`.** The Python fields, in the same order.
- **Byte count.** `bytes()` is the length of the compact, key-sorted JSON. Floats are written with
  Python's `repr` rule: the shortest round-trip digits from `std::to_chars`, scientific when the
  exponent is below −4 or at least 16, and `1e-05` style exponents. Byte counts drive the budgets
  and rate decisions, so they are BIT (D9).
- **Bootstrap.** The 8-member block bootstrap over update batches. Production uses libcvc's
  seeded RNG; the goldens inject the resample indices (D8).
- **Assimilate.** Schema and expiry checks, the received cap, the staging, and one Gauss–Newton
  step:
  - the residual is `[value, ∇value]` at each anchor;
  - its Jacobian with respect to `w` is analytic (softplus over the RBF features);
  - it is whitened by the LLT factor of the covariance, plus the covariance floor;
  - it is added to the local normal equations.
- **`coph_channel`.** A transport interface: `send(from, to, bytes, at_s)` and
  `poll(agent, now_s)`. libcvc ships `ideal_channel`, with seeded delay, drops, duplicates and an
  optional range. cvcdbg adapts its radio network (`Network::send(agent, bits, tag, at_s)` →
  `MessageReception`) on its own side, so libcvc gains no dependency on cvcdbg.

### 4.5 The fleet driver (C5)

`coph_fleet` lives outside `sim_world`, as `route_follower` does (D10). Before each `step()`, it:
- reads each attached agent's pose, heading and speed;
- scales them (§3.3.1);
- steps CoPH at CoPH's own cadence, holding `u` in between (D6);
- allocates (the twin of §3.3.2, pinned to its golden);
- calls `override_command(i, steer, accel)`.

Updates run every 8 CoPH steps, and packets go through the channel. Without a `coph_fleet`, every
run is byte-unchanged.

### 4.6 pycvc and a native controller (C6)

- **Bindings.** `NavCophAgent` and `NavCophFleet`, plus the kernels for the golden tests (`flow`,
  `balance`, `potential`, `grad_potential`, `command`, `allocate`, `packet_bytes`).
- **Native controller.** The cars template gains `--controller coph`, which runs the fleet
  natively. pycvc-gl's wasm build then runs the CoPH demo in the browser, without torch.
- **Live parity.** GRL-SNAM's parity test can check the native fleet against the torch one.

## 5. Golden cases and tiers

FLOAT64 means `rtol` 1e-12 for closed forms and 1e-9 after a linear solve.

| # | What | Content | Tier |
|---|---|---|---|
| G1 | config | the profile, partial overrides, every refusal in `__post_init__`, a mode outside the profile | BIT (accept or refuse; values) |
| G2 | flow, balance | 40 `(x, u, d, dt)` cases, including `d` down to 1e-8 and the bounds | FLOAT64 1e-12 |
| G3 | potential, `∇V`, command | 50 points on the crossing and hazard scenes, `legacy` and `progress` shaping, on both sides of the clip | FLOAT64 1e-12 |
| G4 | stages | advance points, centres, retention and overlap rows along a scripted path | BIT |
| G5 | field fit | design matrix, targets and weights at 3 update snapshots of a scripted crossing run | FLOAT64 1e-9 |
| G6 | damping | Newton iterates and the clipped result at the same snapshots | FLOAT64 1e-9 |
| G7 | replay | cost and `∂cost/∂π` for 6 policies per snapshot, plus the eligibility reasons | reasons BIT; values 1e-9 |
| G8 | update | proposal, halving index, accept or reject and its reason, rollback; cases kept away from the 0.003 slack | BIT (decisions) |
| G9 | packets | export at 3 snapshots with injected bootstrap indices: anchors, covariance, `bytes()` | values 1e-9; bytes BIT |
| G10 | assimilate | packets, plus expired, over-cap and malformed cases: the receiver's weights after one update, and acceptance | acceptance BIT; weights 1e-9 |
| G11 | allocation | 60 `(u, ψ, v, L, δ_max, a_max)` cases, including `v` = 0, reverse and the lock: steer, accel, `e` | FLOAT64 1e-12 |
| G12 | episode | 2 crossing agents, 60 ticks, communication on: per-update versions and accept bits, positions | decisions BIT; positions 1e-6 |

**C++-only tests:** byte-unchanged runs without a fleet; the channel's delay, drop and duplicate
statistics; refusal of out-of-profile configs; and the ledger's residual staying at round-off over
a 2,000-tick run.

## 6. pycvc probes

GRL-SNAM probes each new symbol with a `HAS_` flag, as it does for the belief kernels. Refusals
raise `ValueError`.

## 7. Work breakdown

**GRL-SNAM**
- **T1.** Vendor, untangle, port the tests, and pass the numbers gate (§3.1).
- **T2.** The `grl-snam coph` CLI (§3.1).
- **T3.** `grl-snam ariadne coph`, demo 2a (§3.2).
- **T4.** The cars controller, the allocation and the world adapter, demo 2b (§3.3).
- **T5.** The golden tool and its byte-compare test (§3.4).

**libcvc**, each PR additive and default-off:
- **C1.** Records and the stage ledger; G1 and G4.
- **C2.** Plant, potential, command and energy; G2, G3 and G11 (the allocation lands here so T4
  and C5 share it).
- **C3.** The learner; G5–G8.
- **C4.** Packets, assimilation and `ideal_channel`; G9 and G10.
- **C5.** `coph_fleet`; G12 and the C++-only tests.
- **C6.** pycvc, `--controller coph` and the wasm demo.

**Order:** T1 → T3 (the first demo) alongside T5 → C1 → C2 → C3 → C4 → T4 → C5 → C6.

**cvcdbg** (separate, after C4 and C5): the radio channel adapter and the convoy harness. See the
cvcdbg workstream.

## 8. What the demos claim

- **2a** headlines the crossing result (shared geometry saves the other agent's route) and the
  hazard result (a shared graded field). Both are labelled with the reference's own numbers and
  seed counts.
- **Unestablished pieces stay labelled.** Replay, receiver fitting and adaptive-rate sharing run
  in the demos and show on the panels as unestablished.
- **2b** is labelled exploratory until paired-seed runs measure it. Its first report is the
  allocation residual and the ledger, not a navigation gain.

## 9. Questions for the CoPH-Sense authors

- Is `CoPH-Stagewise` the package to integrate, and which commit should T1 pin?
- Which scenario should headline the demo?
- What scale and plant should the cars use first?
- How should the underactuated case be formulated, if they have a view (§3.3.2)?
- How should upstream and `grl_snam.coph` stay in sync: co-owned PRs, and the goldens as the
  contract?

## 10. Decisions (settled)

**Settled 2026-11-09:** Joe took the middle column for all twelve. The last column records the
alternative each one rejected and why.

| # | Decision | Settled | Rejected alternative and its cost |
|---|---|---|---|
| D1 | Demo order | 2a first, then 2b | 2b first: the vehicle story sooner, but with no established result to show |
| D2 | Vendor or depend | Vendor the core and the two demos into `grl_snam.coph` at a pinned commit, kept in sync by the goldens | Depend on upstream: no copy, but there is no released package, the API changes daily, and the demos' imports pull in 10.4k of its 18.6k lines |
| D3 | First plant and scale | RoboRacer, `s` = 1, on flat and generated worlds | Humvees on Austin first: the better picture, but scale, allocation and the world adapter all change at once |
| D4 | Scaling law | Froude: accelerations unchanged, times × `√s` | Lengths only: Humvees would crawl at 1 m/s |
| D5 | What the learner sees on a car | The force the car took, `u_used`; the unallocated power is its own ledger term | The commanded `u`: the plant model sees forces the car never produced, and the damping fit absorbs the error silently |
| D6 | Clocks | CoPH steps at its own `dt` (0.12 `√s`), holding `u` over the 0.05 s ticks | Re-time CoPH to 0.05 s: `update_every` and the replay horizons are counted in steps, so every learning constant shifts |
| D7 | C++ scope | The demos' profile only; other modes are refused | Every mode: several times the code, and most modes are unestablished experiments |
| D8 | Bootstrap RNG parity | The goldens inject resample indices; production uses libcvc's RNG | Port numpy's PCG64 and its bounded-integer draw, so seeded runs match end to end |
| D9 | Packet size | Count the compact JSON bytes, as the Python does | A binary wire format: smaller on the radios, but budgets and rate decisions stop matching the Python; it can be added later as a channel option |
| D10 | Where the driver lives | `coph_fleet` outside `sim_world`, through `override_command` | A drive mode inside `sim_world::step`: more invasive, for no gain in this spec |
| D11 | Energy items | New `plant_ledger` items that apply only with CoPH attached | A separate CoPH JSON: two ledgers to read |
| D12 | Long routes | A local goal 30 m along the line, as the belief controller's E2 | The raw goal: the quadratic well swamps the risk and barrier terms |
