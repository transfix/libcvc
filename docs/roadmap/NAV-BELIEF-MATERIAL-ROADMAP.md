# Nav Belief Material Roadmap — material maps from what the sensor has seen

> **Goal:** a vehicle that only senses within a fixed range navigates on the material it has
> observed, not on the whole map. The material planes (risk and hard hazards), the witness gate and
> the material trainer all read a belief plane's *observed* material: the truth rasters where that
> plane's sensor has seen, and fixed values everywhere else.

**Status:** implemented 2026-10-20, with §9's recommended column: B1 (the kernels and golden 1,
#742), B2 (`sim_world::set_material_belief`, #744) and B3 (the 3-channel net, `.cvcnm` version 2,
the patch crop and the gate in training, #746). The torch side in GRL-SNAM is built against the
same goldens. The decisions in [§9](#9-open-decisions) can still be revisited; the settled ones are
in [§10](#10-settled).

Related: [`../NAV_MATERIAL.md`](../NAV_MATERIAL.md) (the material field, the gate, the fidelity
tiers), [`../NAV_TRAINING.md`](../NAV_TRAINING.md), `inc/cvc/nav/belief_occupancy.h` (the
precedent), `inc/cvc/nav/grid_nav.h` (`sense_batch`), `inc/cvc/nav/material.h`,
`inc/cvc/nav/material_train.h`, `inc/cvc/nav/coef_energy_net.h`. Torch side: GRL-SNAM's
`grl_snam/belief.py`, `grl_snam/material.py`, `grl_snam/material_nav.py`; CVC-Lab/GRL-SNAM#124 (the
material ext channel) and CVC-Lab/GRL-SNAM#125 (the torch material trainer).

## Why

Material today is oracle world state. `sim_world::set_material` derives the planes from the whole
map once, and the gate's feasibility surface is `hard | truth`. A controller whose sensor reaches
only a fixed range would then:

- bend away from mud it has never seen, because `grad r~` is built from the whole map;
- feel the hard barrier of a pond behind a building;
- have the witness gate certify a detour through terrain nobody has observed;
- train its `lam_soft` / `lam_hard` heads on patches that carry information it will not have when
  deployed.

Belief material closes all four. It is what the occupancy belief already does for geometry.

## The precedent and the design rule

`belief_occupancy.h` ported log-odds → occupancy as one new BIT surface between the raw belief and
the field, bit-identical with numpy, with its float32 sigmoid and comparison pinned. Belief material
follows the same rule, with as few new BIT surfaces as possible:

- the seen mask comes from the sense ray-cast `sense_batch` already runs (BIT with
  `BeliefGrid.sense`);
- the belief inputs are a per-cell select;
- `material_build` and `witness_gate` are reused **unchanged**. Only their inputs change.

So the planes, the gate surface and the gate decisions are BIT end to end, from the truth rasters
and the agents' poses. The only FLOAT-tier piece is the trainer, which is already FLOAT.

## Pipeline

```
truth material (risk_raw, hard)          sense ray-cast (sense_batch, unchanged rays)
           │                                        │  seen_m = material mask of plane m (§1)
           └──────────── belief_material_inputs ────┘
                         risk_b = seen ? risk_raw : unknown_risk
                         hard_b = seen ? hard     : unknown_hard
                                    │
                         material_build (unchanged, BIT) ──► 6 planes: r~, phi_m, grad r~, grad phi
                                    │
        gate surface:  gate_hard_b = hard_b | occ_m | (unknown_gate_hard & !seen)
                       clear_m_b   = f32(sqrt(edt2(gate_hard_b)) * cell_w)
                                    │
             witness_gate (unchanged, BIT)          drive_step_material (unchanged)
                                    │
     trainer batches: risk_patch = [r~|unk, hard|unk, seen] (3 ch), rollout_patch = the 6 belief planes
```

`occ_m` is plane m's planning raster `belief_occ(m)` inside `sim_world`, and a caller-supplied
raster in the standalone kernels (§2.4, decision D5).

## 1. The seen mask

### 1.1 The rule: the sense ray-cast's field of view

A cell is **seen** by belief plane m once a ray from any agent sensing into plane m has touched it.
At the default material range this is exactly `ever_seen(m)`, the raster `sense_batch` already keeps.
The rule is pinned to `raycast_agent` (`src/cvc/nav/grid_nav.cpp`) and `BeliefGrid.sense`
(`grl_snam/belief.py`):

- **Grid.** Corner samples: `cell_w = (max_x - min_x) / (cols - 1)`, `cell_h` likewise; sample
  (r, c) sits at `(min_x + c*cell_w, min_y + r*cell_h)`.
- **Start cell.** `(r0, c0)` is the agent's continuous cell rounded with `rint` (round-half-even). An
  agent off the grid senses nothing that tick.
- **Rays.** Ray j of n has angle `a_j = heading + (j / max(n, 1) - 0.5) * fov` (float64, libm
  `cos`/`sin`).
- **March.** `sr = sin(a) * (cell_w / cell_h)` and `sc = cos(a)`, both divided by
  `max(|sr|, |sc|, 1e-9)`: one cell per step along the dominant axis. Steps run
  `s = 0 .. int(range_m / cell_w)`, visiting cell `(rint(r0 + sr*s), rint(c0 + sc*s))`.
- **Range.** A ray stops at the first cell that is off the grid or farther than `range_m`, measured
  as `hypot((r - r0)*cell_h, (c - c0)*cell_w)`. That cell is not seen.
- **Hit-cell rule.** A ray stops at the first occluding cell, **and that cell is seen**: it is the
  face the sensor sees. Nothing beyond it on that ray is seen. The agent's own cell (s = 0) is always
  seen when it is on the grid.
- **Occluders.** The `truth` raster handed to `sense_batch`, plus the peer and mover boxes when the
  caller passes them. The material layer adds no occluders of its own:
  - a hard hazard that is not in the sense truth (open water, canopy) does not block sight, so it is
    seen across;
  - one that is (a caller that rasterizes deep water into occupancy, as `world_planes::rasterize`
    does) occludes like a wall.
- **Memory.** A monotonic OR across sense ticks. The mask never decays within an episode and resets
  only with its plane.
- **Grouping.** Per belief plane (shared, clustered or private): the agents of a plane share what any
  of them saw, and no plane sees another's cells.

### 1.2 Range

`belief_material_params::material_range_m`, where 0 (the default) means the sense range. A smaller
value keeps the same rays and occluders and only shortens the cutoff. The material mask is then the
cells the sense cast touched whose distance from the agent's cell (the `hypot` above) is at most
`material_range_m`. This is one march, folded into a second mask, with no second cast. It requires
`0 < material_range_m <= range_m`; anything else throws.

Keep the material range well above `d_hat_m` (12 m by default). With `unknown_hard = 0` a hazard has
no barrier until it is seen (§2.2), so the barrier's whole ramp should be built from seen cells
before the vehicle reaches it.

### 1.3 Rays and coverage

The material mask uses the sense's own `n_rays` and `fov_rad`; there is no separate count. A fan
leaves unseen gaps between adjacent rays wherever the arc between them is wider than a cell. As a
sizing rule (not a contract), n rays over `fov` cover the rim without gaps out to about
`n * cell / fov`:

| rays over 360° | gap-free out to | on 0.6 m cells | on 2 m cells | on 5 m cells |
|---|---|---|---|---|
| 72 | 11.5 cells | 6.9 m | 22.9 m | 57.3 m |
| 240 (`sim_world` default) | 38.2 cells | 22.9 m | 76.4 m | 191 m |

A gap fills in as the vehicle moves, but until then it is an unknown cell, and with
`unknown_gate_hard` an obstacle to the gate. The spec does not densify rays automatically. A caller
that wants a gap-free rim sets `n_rays >= fov * range / min(cell_w, cell_h)` (decision D2).

### 1.4 The range disc (an option, not the default)

`seen_rule::disc` marks every corner sample within `material_range_m` of the agent's position, with
no occlusion. It is gap-free, but it sees through walls: the material behind a building enters the
belief. It exists so that a controller trained on a disc mask can be reproduced. It is not the
default (decision D1). Its arithmetic is pinned to numpy:

- the sample x of column c is `min_x + c * ((max_x - min_x) / (cols - 1))`, with the last column
  exactly `max_x` (`np.linspace`); y likewise;
- seen iff `(x_c - px)*(x_c - px) + (y_r - py)*(y_r - py) <= R*R` in float64, with `(px, py)` the
  agent's unsnapped world position;
- applied per agent on sense ticks, ORed into the plane's material mask.

## 2. Belief inputs to `material_build`

### 2.1 The pinned unseen values

```cpp
// inc/cvc/nav/belief_material.h (new)
enum class seen_rule { raycast, disc };

struct belief_material_params {
  float unknown_risk = 0.0f;      // risk_raw on an unseen cell, in [0, 1]
  std::uint8_t unknown_hard = 0;  // hard on an unseen cell, 0 or 1
  bool unknown_gate_hard = true;  // an unseen cell blocks the witness gate's rays
  seen_rule rule = seen_rule::raycast;
  double material_range_m = 0.0;  // 0 => the sense range
};
```

Per cell: `risk_b = seen ? risk_raw : unknown_risk` and `hard_b = seen ? (hard != 0) : unknown_hard`.
Then `material_build(risk_b, hard_b, rows, cols, cell_w, scale, sigma)` runs unchanged.

### 2.2 What the defaults do

`unknown_risk = 0` and `unknown_hard = 0` are *neutral*. An unseen cell exerts no force of its own,
so the force response reveals nothing about hidden terrain. These are also the values the existing
range-limited controller was trained with. Three consequences:

- **The blur crosses the frontier.** `material_build` blurs the composed raster: radius
  `int(4*sigma + 0.5)` cells, 4 at `sigma = 1`. Seen risk bleeds into unknown space, and the unknown
  zeros pull down the seen cells at the frontier. At a seen risky cell next to the frontier, `r~` is
  lower than its truth value and `grad r~` points into the seen area. The soft force therefore pushes
  toward unseen ground. This is optimistic, in the sense of `unknown_policy::optimistic` for
  occupancy.
- **Only seen hazards have a barrier.** `phi_m` is the distance to *seen* hard cells only, so an
  unseen pond has no barrier until a ray reaches it. That is why §1.2 keeps the range above `d_hat_m`.
- **`unknown_hard = 1` is the pessimistic setting.** Every frontier becomes a hazard boundary, and
  the barrier pushes away from unknown space. It is available, not the default.

Both values are validated: `unknown_risk` must be finite and in [0, 1], and `unknown_hard` 0 or 1.
Otherwise the call throws.

### 2.3 Fidelity

The select is BIT. `material_build` on its output is BIT, as before (same TU and
`-ffp-contract=off`). The numpy twin is
`MaterialGrid(np.where(seen, risk_raw, np.float32(unknown_risk)), np.where(seen, hard, bool(unknown_hard)), ...)`.

### 2.4 Standalone kernels

These are for batch builders, tests and the torch twin. Each has a pycvc binding
(`nav_belief_material_inputs`, `nav_belief_gate_surface`, `nav_seen_disc`); all are new symbols.

```cpp
// risk_out [rows*cols] f32, hard_out [rows*cols] u8 (0/1)
void belief_material_inputs(const float *risk_raw, const std::uint8_t *hard,
                            const std::uint8_t *seen, int rows, int cols,
                            const belief_material_params &p, float *risk_out,
                            std::uint8_t *hard_out);

// gate_hard_out = hard_b | occ | (p.unknown_gate_hard & !seen);
// clear_m_out = (float)(sqrt(edt2_squared(gate_hard_out)) * cell_w)  (f64 chain, one f32 store)
void belief_gate_surface(const std::uint8_t *hard_b, const std::uint8_t *occ,
                         const std::uint8_t *seen, int rows, int cols, double cell_w,
                         const belief_material_params &p, std::uint8_t *gate_hard_out,
                         float *clear_m_out);

// seen_rule::disc: OR one agent's disc (world position px, py; radius R) into seen
void seen_disc(double px, double py, double R, int rows, int cols, double min_x,
               double min_y, double max_x, double max_y, std::uint8_t *seen);
```

The clearance chain is the one `sim_world::set_material` and `MaterialRuntime._recompute` already
use. The Python twin of the gate surface is `MaterialRuntime.update_occ(occ | ~seen)` on the
belief-input `MaterialGrid`.

`sense_batch` gains optional outputs. When they are null its bytes are unchanged:

- `belief_planes::material_seen` (`[K*rows*cols]`), the material-range mask of §1.2;
- `belief_planes::material_changed` (`[K]`), set to 1 when that mask grew;
- `sense_agents::material_range_m` (`[n]`).

## 3. `sim_world` wiring

```cpp
// Truth material is ONE [rows, cols] world plane; one belief material plane is kept per belief plane.
void set_material_belief(const float *risk_raw, const std::uint8_t *hard,
                         const material_config &mc, const belief_material_params &bp);
const std::uint8_t *material_seen(int m) const; // == ever_seen(m) at the default range and rule
```

- **Default off.** `set_material` (the oracle path) and the no-material path are byte-unchanged.
  `set_material_belief` replaces `set_material`; it does not stack on it.
- **Throws** under `freeze_sense == true`, where nothing would ever be seen, and when the raster
  sizes do not match.
- **Rebuild rule.** Plane m's six material planes and its contiguous risk view are re-derived on a
  sense tick only when its material mask grew. Its gate surface (`gate_hard_b`, `clear_m_b`) is
  re-derived when the mask grew or `belief_occ(m)` changed. Before the first sense tick every cell
  is unseen. `material_every` (in sense ticks, default 1) caps the rebuild rate (decision D4).
- **Gate occupancy.** `belief_occ(m)`, the plane's own planning raster: what the plane believes is
  blocked, dynamic layer included. Not truth (decision D5).
- **The drive is unchanged.** Agent i samples material plane `map_id[i]`.
- **`raster_version(m)`** also bumps when plane m's material mask changed.
- **Memory.** About 36 bytes per cell per belief plane: the 6-plane stack, the risk view and
  `clear_m` in f32, plus `gate_hard`, the material mask and `hard_b` in u8. That is about 36 MB per
  plane on a 1024² raster. Shared and clustered belief are fine. Private belief over many agents is
  not (100 agents ≈ 3.6 GB), and the header says so.
- **Mid-episode truth changes are out of scope.** Calling `set_material_belief` again re-derives
  from the new truth on every remembered cell, which reveals a change the agents did not see. Updating
  only what is in view (needing the tick's union field of view, which `sense_batch` does not output)
  is deferred.

## 4. The witness gate on belief planes

The kernel is unchanged. Its inputs are the belief `r~` (channel 0), `gate_hard_b` and `clear_m_b`
(§2.4).

With `unknown_gate_hard` on (the default):

- A ray is feasible only while it stays on seen ground, at least `hard_margin_m` from the frontier
  (2 cells by default). So the gate can switch the soft force on only to steer toward a seen,
  lower-risk detour.
- The straight-to-goal nominal risk is still the mean over the samples taken before its ray broke
  (the pinned order).

With it off, unknown cells are ordinary cells whose risk is about `unknown_risk`. A ray through
unseen ground then scores as nearly risk-free and certifies detours into the unknown: the leak the
default prevents.

`lam_hard` is never gated, as before.

## 5. The trainer on belief planes

No truth enters a batch except through what was seen.

- **Model patch:** 3 channels, `[B*3*P*P]`.
  - ch0 = `seen ? r~_b : unknown_risk`;
  - ch1 = `seen ? hard_b : unknown_hard`;
  - ch2 = `seen` (0/1).

  Masking ch0 removes the blur's bleed into unknown cells, so the net reads exactly the pinned value
  where nothing was seen. The seen bit tells "unknown" from "known clear" (decision D6).
- **Rollout patch:** the 6 belief planes, **unmasked**. These are the planes the deployed drive
  samples, so the rollout samples them too.
- **Cost:** `cum_risk` and `hard_count` in `J` are scored on the rollout patch, that is on belief.
  This makes the rollout a twin of the deployed field (decision D7).
- **Gate in training (new, optional).** `material_batch::gate_active` (`[B]` u8). Null means
  ungated, which is today's trainer, byte for byte. When given, each agent's `lam_soft` is
  multiplied by its bit in all three integrators and their VJPs. The bit is computed once per sample
  by `witness_gate_batch` on the belief planes at the sample's start, and it is data (stop-gradient),
  as at deployment (decision D8).
- **Crop.** The model patch spans `patch_extent_m`, centred at the agent, on a P×P grid at
  `np.linspace(-extent/2, extent/2, P)` offsets:
  - ch0 is sampled bilinearly (align-corners) and then masked by the seen bit;
  - ch1 and ch2 take the nearest cell by `rint` (round-half-even, as `world_to_cell`);
  - a sample off the map is unseen: ch2 = 0, and ch0 and ch1 take the pinned values.

  `material_patch_belief` implements this crop, with a numpy twin. The rollout patch's crop does not
  change; only its source planes do.
- **Strides.** `material_batch::risk_patch` is `[B*C*P*P]` with `C = model.risk_channels()`
  (§6). The trainer validates the batch against the model.

## 6. `.cvcnm` input changes

Today `risk_enc.net.0.weight` is `[16, 2, 3, 3]` and the forward hardcodes 2 input channels. The
loader checks no tensor shape. A 3-channel file read by today's libcvc (3.7.0 and earlier) would be
read with 2-channel strides: wrong coefficients and no error. Hence:

- **Format version 2.** It is written only for nets that need it (`risk_channels == 3`). A 2-channel
  net is still written as version 1, byte-identical to today. Readers accept 1 and 2. A pre-spec
  reader refuses version 2 with "unsupported .cvcnm version" instead of computing garbage
  (decision D9).
- **The v2 header** is v1's through `eps`, then a belief block, then `n_tensors` as before:

  | field | type | meaning |
  |---|---|---|
  | `risk_channels` | u32 | 2 = `[r~, hard]`, every cell seen (v1's meaning); 3 = `[r~, hard, seen]` |
  | `seen_rule` | u32 | 0 raycast, 1 disc |
  | `unknown_risk` | f32 | the pinned unseen risk the net was trained with |
  | `unknown_hard` | u32 | 0 or 1 |
  | `unknown_gate_hard` | u32 | 0 or 1 |
  | `material_range_m` | f32 | 0 = the sense range |
  | `patch_extent_m` | f32 | world metres the P×P model patch spans; 0 = not recorded |

- **Loader hardening (both versions).** Every known tensor's shape is checked against the
  hyperparams, and `risk_enc.net.0.weight` must be `[16, risk_channels, 3, 3]`. A mismatch throws.
  `risk_channels()` and `belief_header()` are exposed.
- **`arch_hash`** (FNV-1a over the sorted (name, shape) descriptors) changes with the channel count
  on its own; the computation does not change.
- **Meta round-trip.** The `meta_len` + UTF-8 trailer is kept on load and written back by
  `serialize()`. Today libcvc writes `meta_len = 0`, so a load → save drops any meta GRL-SNAM wrote.
- **Inputs must match the training settings.** A caller running a v2 net on belief planes compares
  the header's `unknown_risk`, `unknown_hard` and `seen_rule` with its `belief_material_params` and
  refuses a mismatch: a net trained with unknown 0 sees inputs it never trained on at unknown 0.5.
  `coef_energy_net` is not run inside `sim_world` today. Wiring it in is outside this spec, but
  whoever does it uses this check and the §5 crop.
- **Torch side.** `RiskPatchEncoder(in_ch=…)` in `grl_snam/material_nav.py`, and v2 in
  `matnet_export.write_matnet` / `read_matnet`. The numpy reference `_risk_enc` already follows the
  weight's shape.

## 7. Goldens and tests

1. **`belief_material_golden.json` (BIT, committed, about 50 KB).**
   - **Scene:** a 32×32 synthetic raster on square cells: two buildings (in the sense truth, so
     they occlude), a hard pond and hard canopy outside the sense truth (seen across), and a
     risk-0.6 patch.
   - **Agents:** 3 agents on 2 planes (agents 0 and 1 on plane 0, agent 2 on plane 1). Their poses
     are scripted, not driven, so no FLOAT drive enters a BIT golden. 3 sense ticks.
   - **Cases:** (a) the defaults; (b) `material_range_m < range_m`; (c) `seen_rule::disc`;
     (d) unknown risk 0.5, unknown hard 1, `unknown_gate_hard` off.
   - **Content:** per case, tick and plane, the SHA-256 of the little-endian bytes of the material
     mask, `risk_b`, `hard_b`, each of the 6 planes, `gate_hard_b` and `clear_m_b`, plus the
     `witness_gate_batch` outputs for scripted goals. Case (a)'s last tick also ships full arrays,
     for diffing a mismatch.
   - **Python reproduction:** `BeliefGrid.sense`, then `np.where`, `MaterialGrid`, `MaterialRuntime`
     and `witness_gate_batch`.
2. **`sim_world` (C++, behavioural).**
   - With every cell seen and the gate occupancy set to truth, `set_material_belief`'s planes equal
     `set_material`'s, byte for byte.
   - No material and `set_material` are unchanged.
   - `freeze_sense` throws.
   - A plane's material ignores another plane's sensing.
   - The rebuild trigger fires exactly when the mask grows.
3. **The belief trainer golden (FLOAT).** The 3-channel sibling of the queued small trainer golden,
   sharing its builder:
   - narrow net: d_tok 16, nhead 2, 1 layer, d_risk 16;
   - B 4, P 9;
   - patches cropped by `material_patch_belief` from golden 1's last default planes;
   - `gate_active` set for 2 of the 4 agents;
   - contents: w0, grad0, w3 and the 4 steps' terms;
   - the tolerances of the 2-channel trainer golden.
4. **`.cvcnm`.**
   - v1 2-channel files load and re-save byte-identically, meta included.
   - v2 round-trips.
   - A wrong-shape tensor throws.
   - A v2 file's version field is 2.

## 8. Work breakdown

**libcvc**, each PR additive and default-off:

- **B1 — kernels (BIT).** `belief_material.h`, the `sense_batch` material-mask outputs, pycvc, and
  golden 1.
- **B2 — `sim_world`.** `set_material_belief`, the rebuild rule, the read surface, the
  `nav.material` live knobs unchanged, and test 2.
- **B3 — net and trainer.**
  - `.cvcnm` v2 and the loader hardening;
  - C-channel strides in the CPU and CUDA forward, backward and trainer;
  - `material_batch::gate_active`;
  - `material_patch_belief`;
  - goldens 3 and 4.

  B3 follows the queued small trainer golden and #734, whose `material_batch` it extends.

**GRL-SNAM (torch)**, built against the goldens:

- **T1.** The material-range and disc masks on `BeliefGrid`, the belief `MaterialGrid` /
  `MaterialRuntime` composition, and parity with golden 1.
- **T2.** `RiskPatchEncoder(in_ch)`, `matnet_export` v2, `gate_active` in the torch trainer (#125),
  the crop twin, and parity with golden 3.
- **T3.** The range-limited controller on `CoefEnergyNetMaterial` with the 3-channel patch (§10).

## 9. Open decisions

For the library maintainer and the sensor-controller owner. Each row's recommendation is what the
spec above assumes.

| # | Decision | Recommended | Alternative and its cost |
|---|---|---|---|
| D1 | Default seen rule for the controller's port | Ray-cast (§1.1): no see-through-walls leak, and it is already BIT | Disc (§1.4): reproduces the existing controller's training, but material behind buildings leaks in |
| D2 | Ray count on the material mask | One count, raised to coverage (`n_rays >= fov * range / cell`) | Keep 72: gaps past 11.5 cells (23 m on 2 m cells) read as unknown, and gate walls with `unknown_gate_hard` on. Or a separate material ray count: a second cast, more cost |
| D3 | Unseen values | `unknown_risk 0`, `unknown_hard 0`, `unknown_gate_hard on`: neutral, and the existing controller's values | A prior (e.g. risk 0.5) or pessimistic hard (frontier = hazard) |
| D4 | Material rebuild cadence | Every sense tick on which the mask grew (`material_every 1`) | Every k sense ticks (the existing controller queries every 10 ticks): cheaper, staler |
| D5 | Gate occupancy | `belief_occ(m)`, the plane's own planning raster | `truth & seen` (the existing controller): no log-odds lag, but it does not see dynamic obstacles |
| D6 | Masking after derivation | The model patch only (§5) | Also zero the 6 force planes off-seen (the existing controller does). `phi_m = 0` there reads as "inside a hazard" to `hard_count`, and the zeroed gradients put a step in the field at the frontier |
| D7 | Training cost | On belief: the deployed twin | On truth (a privileged cost): teaches caution near the unknown, needs a second, truth-scored patch in the batch |
| D8 | Gate in training | Optional, off by default (today's trainer, and the existing controller's default) | On by default: lam_soft gradients only where the gate fires, as at deployment |
| D9 | `.cvcnm` for 3-channel nets | Version 2, so pre-spec readers fail loudly | Version 1 with the channel count read from the weight's shape: old readers silently mis-stride |
| D10 | Model patch for the port | P 25 over 100 m (the existing controller's), recorded in `patch_extent_m` | P 32, the `.cvcnm` default; the extent must be chosen and recorded either way |

## 10. Settled

- **2026-10-19:** the split. libcvc writes this spec, the kernels and the goldens; the torch side
  in GRL-SNAM is built against them.
- **2026-10-20:** the target is the shared stack: `sim_world`, a `.cvcnm` `CoefEnergyNetMaterial`
  with the 3-channel patch, and the native and torch trainers. The existing range-limited controller
  moves onto it rather than keeping a separate net. Where its current conventions do not leak truth
  (the unseen values, unknown-blocks-gate), they are the defaults here.
