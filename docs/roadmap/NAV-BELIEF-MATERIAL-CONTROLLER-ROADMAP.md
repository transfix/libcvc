# Nav Belief Material Controller Roadmap — GRL-SNAM's range-limited controller in C++

> **Goal:** a `.cvcnm` version-2 net trained by `grl-snam austin-material-train` drives in C++ and
> makes the same decisions as GRL-SNAM's `BeliefMaterialController`. Given the same belief state,
> pose and goal, the C++ side builds the same model inputs and refuses the same checkpoints. It
> predicts the same six coefficients (within the FLOAT tier) and gates `lam_soft` the same way (bit
> for bit).

**Status:** C1 (the contract and the inputs, `belief_material_controller.h`) implemented, with
goldens G1–G5 and its pycvc bindings; C2–C4 to come. The decisions in
[§9](#9-decisions-settled) were settled on 2026-11-08: Joe took the recommended column for all ten.
The spec was merged as transfix/libcvc#1053. GRL-SNAM's side of E5 and E6 is CVC-Lab/GRL-SNAM#163,
and its golden generator (T-a) is CVC-Lab/GRL-SNAM#164, stacked on it.

Related: [`NAV-BELIEF-MATERIAL-ROADMAP.md`](NAV-BELIEF-MATERIAL-ROADMAP.md) (the belief planes,
the 3-channel patch, `.cvcnm` v2: this spec builds on its §5 crop and §6 header check),
[`../NAV_MATERIAL.md`](../NAV_MATERIAL.md), `inc/cvc/nav/coef_energy_net.h`,
`inc/cvc/nav/belief_material.h`, `inc/cvc/nav/sim_world.h`, `inc/cvc/nav/material.h`
(`integrate_surrogate_material`, `material_drive`). Python reference: GRL-SNAM
`grl_snam/belief_material_controller.py` (CVC-Lab/GRL-SNAM#141), `docs/MATERIAL_NAV.md` "A
range-limited belief-material controller", and the Austin tools
`grl_snam/tools/austin_material_{training,evaluation}.py`.

## Why

libcvc already has every kernel the Python controller calls:

| Python (`belief_material_controller.py`) | libcvc today |
|---|---|
| `material_patch_belief` | `belief_material.h` `material_patch_belief` (BIT) |
| `belief_material_runtime`, gate surface | `belief_material_inputs`, `belief_gate_surface`, `sim_world::set_material_belief` |
| `MaterialRuntime.gate` | `witness_gate` / `witness_gate_batch` (BIT) |
| `CoefEnergyNetMaterial(risk_channels=3)` forward | `coef_energy_net::forward_one` / `forward_batch` (FLOAT, rtol 1e-4) |
| `.cvcnm` v2 belief block | `cvcnm_belief_header`, `belief_header()` |
| `integrate_surrogate_material` (`rollout`) | `integrate_surrogate_material` (FLOAT) |

What it lacks is the controller layer #141 adds on top:

1. the `meta` contract (`grl_snam.belief_material`) and the refusals built on it;
2. the robot-relative obstacle and goal tokens;
3. `decide`: the gate → `lam_soft` switch;
4. a way to run `coef_energy_net` per agent inside `sim_world`.

Without (1), a C++ host that loads a belief `.cvcnm` reads `meta()` as an opaque string. It would
then feed tokens in absolute metres, the frame `train_coef_energy` uses, and run the heads under
`material_config`'s defaults (k 1.25/m, d_hat 12 m, trigger 0.45) instead of the contract's (5/m,
3 m, 0.30). No error is raised, and the coefficients are wrong.

## Pipeline

```
 .cvcnm v2 ── header check (§1.2) ── meta contract (§1.1) ── runtime check (§1.3)
                                                                   │
 belief plane m (set_material_belief): r~_b, hard_b, seen_m, gate_hard_b, clear_m_b
                                                                   │
 pose p, goal g (world m), obstacles (C, R, W) ── tokens (§2) ── obs_feats [N*6], goal_feats [4]
                                         └── material_patch_belief(p) ── risk_patch [3*P*P]
                                                                   │
                                  coef_energy_net::forward_one ── alphas, beta, gamma,
                                                                   │  lam_soft, lam_hard, mu_lat
                                  witness_gate(p, g) ── active ──► lam_soft_eff (§3)
                                                                   │
                     sim_world drive (§4)   or   integrate_surrogate_material (§5)
```

## 1. The contract

### 1.1 The `meta` record

A belief-material `.cvcnm` records what it was trained with as JSON in its `meta` trailer, under
the key `grl_snam.belief_material` (Python `BeliefMaterialContract.to_meta`, `sort_keys=True`):

| field | type | default | rule |
|---|---|---|---|
| `feature_frame` | string | `"robot_relative"` | must equal `"robot_relative"` |
| `feature_scale_m` | number | 20.0 | finite, > 0 |
| `k_sharp` | number | 5.0 | finite, > 0 (1/m) |
| `d_hat_sdf_m` | number | 3.0 | finite, > 0 (m) |
| `material_gate_trigger` | number | 0.30 | finite, in [0, 1] |
| `geometry_d_hat_m` | number | 3.0 | finite, > 0 (m) |

The parse mirrors `BeliefMaterialContract.from_meta`:

- An empty `meta`, invalid JSON, a top level that is not an object, or no object under the key:
  throw ("no 'grl_snam.belief_material' record ... feature frame and material constants are
  unknown").
- Any field not in the table throws ("unknown belief-material contract fields [...]").
- A **missing** field takes its default, as the dataclass does.
- The value rules above are checked after the defaults are filled in.
- Other top-level keys in `meta` are ignored.

Value types (decision E6): the C++ parse accepts only JSON numbers for the numeric fields and only
a string for `feature_frame`. Python used to run `float(value)`, which accepted the string `"5"`
and `true` as numbers; CVC-Lab/GRL-SNAM#163 tightens it to match. Both sides store a JSON integer
as a double, so `to_meta` writes `20.0` for a record that said `20`. `json.loads` also accepts the
`NaN`, `Infinity` and `-Infinity` literals and keeps the last of duplicate keys, so the C++ parse
does both: a record with `NaN` under some other key still loads.

Numbers are read with `std::strtod` from the JSON text, rewritten as `<digits>e<exponent>` with
no radix character, so the C locale's decimal point does not matter. Python writes them with
`repr`, which round-trips, so the parsed doubles are exact. The parser is a small one in the
controller's TU,
scoped to this record: a JSON object of string keys whose values are numbers, strings, booleans,
null, arrays or objects, so other keys can be skipped. It adds no dependency.

### 1.2 The header check

This is the twin of `BeliefMaterialController.from_cvcnm`. The net must be version 2 with
`risk_channels() == 3`. Then every field below is compared, and **all** mismatches are reported in
one `std::invalid_argument` ("incompatible belief checkpoint: name=actual (expected x), ..."):

| header | compared with | how |
|---|---|---|
| `risk_channels` | 3 | equal |
| `seen_rule` | `belief_material_params::rule` (raycast 0, disc 1) | equal |
| `unknown_risk` | `bp.unknown_risk` | abs 1e-6 |
| `unknown_hard` | `bp.unknown_hard` | equal (as bool) |
| `unknown_gate_hard` | `bp.unknown_gate_hard` | equal (as bool) |
| `material_range_m` | `bp.material_range_m` | abs 1e-6 |
| `patch_extent_m` | the controller's `patch_extent_m` (default 100) | abs 1e-6 |
| `patch_size()` | the controller's `patch_size` (default 25) | equal |

The controller also requires `bp.rule == seen_rule::raycast`, as Python does
(`BeliefMaterialController` refuses `seen_rule='disc'`). The header check runs first and the meta
parse second, in Python's order, so the same broken file produces the same first error.

### 1.3 The runtime check

This is the twin of `check_runtime`. A `material_config` whose constants differ from the contract
is refused, with all mismatches in one message:

| `material_config` | contract field | tolerance |
|---|---|---|
| `k_sharp` | `k_sharp` | abs 1e-6 |
| `d_hat_m` | `d_hat_sdf_m` | abs 1e-6 |
| `gate.material_trigger` | `material_gate_trigger` | abs 1e-6 |

`material_config_for(contract, lam_soft, lam_hard, cell_w)` returns the `material_config` that is
the twin of `contract.material_params(...)` on a grid of cell width `cell_w`. A host builds its
belief material from it, so the check passes by construction. It sets:

- `k_sharp`, `d_hat_m` and `gate.material_trigger` from the contract;
- `gate.horizon_cells = max(1, (int)rint(25.0 / cell_w))`. Python's `GateParams` sets the ray
  length in metres (`horizon_m` 25) and `MaterialRuntime` rounds it to cells (`round` is
  half-even, as `rint` is). libcvc's `gate_params` default is 12 cells, which matches only on a grid
  of about 2.1 m. On the Austin 15 m grid Python's horizon is 2 cells.
- `gate.hard_margin_m = 0`, which `sim_world` resolves to 2 cells: Python's `None`;
- `sigma` 1 and the other gate fields at defaults that already equal Python's (16 primitives,
  improvement margin 0.05, progress slack 0.5 cells).

The contract itself pins neither `sigma` nor the gate's horizon and margins. Both sides use the
values above, and the goldens pin them (decision E10). §1.3's check does not cover them, because
Python's `check_runtime` does not.

### 1.4 API

New header `inc/cvc/nav/belief_material_controller.h`, compiled with `-ffp-contract=off` like
`belief_material.cpp`:

```cpp
namespace cvc::nav {

inline constexpr const char *kBeliefMaterialMetaKey = "grl_snam.belief_material";

struct belief_material_contract {
  std::string feature_frame = "robot_relative";
  double feature_scale_m = 20.0;
  double k_sharp = 5.0;
  double d_hat_sdf_m = 3.0;
  double material_gate_trigger = 0.30;
  double geometry_d_hat_m = 3.0;

  static belief_material_contract from_meta(const std::string &meta,
                                            const std::string &source = "checkpoint");
  std::string to_meta() const; // byte-identical to Python's json.dumps(..., sort_keys=True)
  void validate() const;       // the §1.1 value rules
};

material_config material_config_for(const belief_material_contract &c, float lam_soft,
                                    float lam_hard, double cell_w);

class belief_material_controller {
public:
  // Takes ownership of the net. Runs §1.2 and §1.1 (throws std::invalid_argument).
  belief_material_controller(coef_energy_net net, const belief_material_params &bp = {},
                             int patch_size = 25, double patch_extent_m = 100.0,
                             const std::string &source = "checkpoint");
  static belief_material_controller from_cvcnm(const std::string &path,
                                               const belief_material_params &bp = {},
                                               int patch_size = 25, double patch_extent_m = 100.0);

  void check_runtime(const material_config &mc) const; // §1.3
  const belief_material_contract &contract() const;
  const coef_energy_net &net() const;
  ...
};

} // namespace cvc::nav
```

`to_meta` lets a C++-trained net (`coef_energy_net::set_meta`) carry a contract GRL-SNAM can read.
Its output must equal Python's `to_meta` byte for byte, which the goldens check. That means
Python's `json.dumps` layout (`{"grl_snam.belief_material": {"d_hat_sdf_m": 3.0, ...}}`, keys
sorted, `", "` and `": "` separators) and Python's float `repr`. `repr` is the shortest
round-trip digits, as `std::to_chars` gives, but formatted Python's way: `20.0` and not `20`,
scientific below 1e-4 or from 1e16 up (`1e-05`, `1e+16`).

## 2. The tokens

### 2.1 The builder

This is the twin of `build_belief_material_controller_inputs` for one agent. The caller supplies
the agent's world position `p`, its goal `g`, and its already range-limited obstacles: centres
`C[N*2]`, radii `R[N]` and weights `W[N]`, all in world metres. With `s = (float)feature_scale_m`,
everything below is float32 arithmetic in this order, one operation at a time:

```
pf = (float)p, gf = (float)g                       // np.asarray(..., float32) first
d  = (gf - pf) / s                                 // goal_delta, per component
goal_feats = [d.x, d.y, dist(d), 1.0f]
for k in 0..N-1:
  l = ((float)C[k] - pf) / s                       // local centre
  obs_feats[k] = [l.x, l.y, (float)R[k] / s, (float)W[k], d.x - l.x, d.y - l.y]
obs_mask[k] = 1                                    // every supplied obstacle is a real token
risk_patch = material_patch_belief(r~_b, hard_b, seen, rows, cols, bounds,
                                   (double)pf.x, (double)pf.y, patch_extent_m, P, bp)
```

The token layout is the one `coef_energy_net.h` documents:
`[cx, cy, R_eff, W, goal_x - cx, goal_y - cy]` and `[dx, dy, dist, 1]`. Only the frame differs from
`train_coef_energy`: robot-relative and divided by `s`, where `train_coef_energy` uses absolute
metres.

Two details are easy to miss:

- **The crop is centred on the float32 position.** Python passes the float32 `position` array to
  `material_patch_belief`, which then works in float64. So the C++ side passes
  `(double)(float)p`, not `p`. Using the double position moves the patch by up to half a float32
  ulp. That is enough to change a nearest-cell `rint` at a cell boundary.
- **`dist` is `np.linalg.norm` of a float32 2-vector.** numpy computes it as `sqrt(dot(d, d))`, and
  the dot may go through BLAS `sdot`, whose kernels can fuse a multiply-add. The C++ side computes
  `sqrtf(d.x*d.x + d.y*d.y)` in float32 without contraction. The tier for `dist` is 1 ulp; every
  other token value is BIT.

`N` may be 0: `obs_feats` is then empty and the forward runs on the goal token and the patch.
Shapes are validated as Python does (`C` is `N x 2`, and `R` and `W` have `N` entries); `p` and `g`
must be finite and `s` finite and > 0. The `seen` raster must be the belief planes' size.

```cpp
struct belief_material_controller_inputs {
  std::vector<float> obs_feats;       // [N*6]
  std::vector<std::uint8_t> obs_mask; // [N], all 1
  float goal_feats[4];
  std::vector<float> risk_patch;      // [3*P*P]
};
// The belief planes the crop reads: channel 0 (r~) of the 6-plane stack, hard_b and the seen mask.
struct belief_material_view {
  const float *risk;
  const std::uint8_t *hard_b, *seen;
  int rows, cols;
  double min_x, min_y, max_x, max_y;
};
void build_belief_material_controller_inputs(double px, double py, double gx, double gy,
                                             const float *C, const float *R, const float *W, int N,
                                             const belief_material_view &planes,
                                             double patch_extent_m, int P, double feature_scale_m,
                                             const belief_material_params &bp,
                                             belief_material_controller_inputs &out);
```

### 2.2 The obstacle source

The controller does not choose its obstacles: `decide` takes them as arguments. Both Austin tools,
training and evaluation, choose them the same way (`scan_visible_geometry`), and a trained net has
only ever seen that set. A C++ host that wants the same decisions needs the same set, so libcvc
gains a twin, `visible_obstacles`:

- the candidates are the cells with `last_visible & truth`: occupied cells a ray touched on this
  tick;
- a cell's centre is `(min_x + c / (cols - 1) * (max_x - min_x), min_y + r / (rows - 1) * (max_y
  - min_y))`, computed in float64 and stored as float32;
- each candidate's distance to the position is `sqrt(dx*dx + dy*dy)` in float64, from the float32
  centre and the float64 position (numpy's `norm(axis=1)` on that mixed array is a plain f64
  reduce with no BLAS);
- the nearest `max_obstacles` are kept (36 in both tools), nearest first;
- every radius is `0.5 * min(cell_w, cell_h)` and every weight is 1.

The tie order is the one open BIT question. `np.argsort` defaults to an unstable introsort, and on
a grid equal distances are common. A different tie order permutes the tokens. The transformer has
no positional encoding over obstacles, so the coefficients are unchanged up to reduction order,
but a tie at the cutoff picks a different set. So both sides sort stably on (distance, row-major
index) (decision E5): `kind="stable"` on argwhere's row-major order, CVC-Lab/GRL-SNAM#163. The
golden's 9 x 9 scene puts rings of equal distances at the cutoff.

```cpp
// Writes up to max_obstacles centres [k*2], radii [k], weights [k]; returns k.
int visible_obstacles(const std::uint8_t *last_visible, const std::uint8_t *truth, int rows,
                      int cols, double min_x, double min_y, double max_x, double max_y, double px,
                      double py, int max_obstacles, float *C, float *R, float *W);
```

`scan_visible_geometry`'s `ray_hits` output feeds only the waypoint chooser
(`choose_material_local_waypoint`), which is not part of this spec (see E2).

## 3. `decide`: the gate → `lam_soft` switch

This is the twin of `BeliefMaterialController.decide` for one agent:

```
check_runtime(mc)                                        // §1.3, every call (it is cheap)
inputs    = build_belief_material_controller_inputs(p, g, C, R, W, planes, ...)   // §2.1
predicted = net.forward_one(inputs)                      // alphas[N], beta, gamma,
                                                         // lam_soft, lam_hard, mu_lat
if mc.gate_enabled:
    gate     = witness_gate(r~_b, gate_hard_b, clear_m_b, rows, cols,
                            world_to_cell(p), world_to_cell(g), gate_params(mc))
    lam_soft = gate.active ? predicted.lam_soft : 0
else:                                                    // fail-open, as material_config documents
    gate     = none
    lam_soft = predicted.lam_soft
effective = predicted with lam_soft replaced             // lam_hard is never gated
```

- **One goal for both.** The goal token and the gate use the same `g`. Python's tools pass the
  local goal there: the teacher's 30 m lookahead in training, and the waypoint chooser's output in
  evaluation.
- **The gate's inputs** are the belief plane's `r~_b` (channel 0), `gate_hard_b` and `clear_m_b`:
  the planes `MaterialRuntime.gate` reads. `world_to_cell` is `MaterialRuntime._world_to_cell_f`:
  `cc = (x - min_x) / (max_x - min_x) * (cols - 1)`, `cr` likewise, in float64. Its horizon and
  margin are the ones §1.3 derives.
- **The gate takes the position as given; the crop takes it rounded to float32.** Python's `decide`
  passes `position_world` to `runtime.gate` unchanged (float64 in the Austin evaluation), but the
  token builder casts it to float32 first (§2.1). The C++ side does the same. Inside `sim_world`
  the position is already float32, so the two agree there.
- **Both the predicted and the effective coefficients are returned**, with the gate decision, as
  in `BeliefMaterialControlCommand`. The Austin evaluation's `fixed_lambda` and `waypoint` arms
  rebuild `effective` from `predicted` and fixed lambdas. Keeping `predicted` lets a C++ host run
  those arms too.
- **Parity.** The gate bit is BIT: same planes, same cell coordinates, same kernel. The zero in
  `lam_soft_eff` when the gate is off is therefore exact. Every predicted value is FLOAT (the
  forward's tier).

```cpp
struct belief_material_coefficients {
  std::vector<float> alphas; // [N]
  float beta, gamma, lam_soft, lam_hard, mu_lat;
};
struct belief_material_command {
  belief_material_coefficients predicted, effective;
  bool gated = false;        // false when mc.gate_enabled is off
  gate_decision gate;        // valid when gated
};
// on belief_material_controller:
belief_material_command decide(double px, double py, double gx, double gy, const float *C,
                               const float *R, const float *W, int N,
                               const belief_material_view &planes, const std::uint8_t *gate_hard_b,
                               const float *clear_m_b, const material_config &mc) const;
```

A batched `decide_batch` over agents with ragged obstacle lists wraps `forward_batch` and
`witness_gate_batch` with the same `obs_offsets` layout as `forward_batch`. That is what
`sim_world` calls.

## 4. `sim_world`: the net per agent

`coef_energy_net` is not run inside `sim_world` today. The belief roadmap's §6 left that wiring to
whoever does it, with its header check and the §5 crop. This section is that wiring.

### 4.1 Attaching

```cpp
// After set_material_belief. Throws std::invalid_argument if there is no belief material, if the
// controller's §1.2 check fails against the belief_material_params set_material_belief took, if §1.3
// fails against its material_config, or if the coef_mlp is a lam-head net (E8).
void set_material_controller(std::shared_ptr<const belief_material_controller> c,
                             const material_controller_options &opt = {});
void clear_material_controller();
bool has_material_controller() const;

struct material_controller_options {
  enum class mode { lam, full } drive = mode::lam; // §4.3 / §4.4; full is C4, refused until then
  int decide_every = 1;        // ticks between decisions; the last command is held between (E4)
  int max_obstacles = 36;      // §2.2
  double local_goal_m = 30.0;  // §4.2 goal; 0 = the agent's goal itself (E2)
};
```

- **Default off.** Without a controller, every run is byte-unchanged, `set_material_belief` runs
  included.
- **It needs belief material.** On the oracle `set_material` path there is no seen mask, so a
  3-channel net would read truth as "seen everywhere". That is input it never trained on, so
  attaching throws.
- **It is detached** by `set_material`, `set_material_belief` and `clear_material`. Re-attach
  after a material change, so the §1.3 check always sees the live `material_config`.
- **`set_material_lam` has no effect** while a controller is attached: the net's lambdas replace
  the fixed ones, as with a lam-head `coef_mlp`. **`set_material_lam_scale` still applies** to the
  net's lambdas, after the gate (`lam_soft_eff * lam_soft_scale`, `lam_hard * lam_hard_scale`).
  At the default scale 1 this is exactly Python's `effective`. This keeps demo3's force-bias
  slider working.

### 4.2 Each tick

The controller replaces the fixed-lambda columns in step()'s material block, right after the gate.
For each agent i on belief plane m = `map_id[i]`, on a tick where `tick % decide_every == 0`:

- **Position.** `o_` converted to world metres in float32, exactly as the gate block does today
  (`o / scale + c`). The gate and the crop then agree on where the agent is.
- **Goal.** The agent's goal in world metres, clipped to `local_goal_m` along the straight line
  from the agent when it is farther than that. With `local_goal_m = 0` the goal is used as is.
  The gate block uses this same point, so the token and the gate agree (§3). This changes the gate
  block's goal from `goal_` only while a controller is attached. A caller override column
  (`set_local_goal(i, x, y)`) takes precedence when set. It is the hook for a ported waypoint
  chooser (E2).
- **Obstacles.** `visible_obstacles` on `last_visible(m) & truth()` (§2.2, E3).
- **Planes.** Plane m's `material_plane_data(m)` channel 0, its `hard_b`, `material_seen(m)`,
  `material_gate_hard(m)` and `material_clear_m(m)`.
- **Decision.** `decide_batch` over all agents, fanned across the thread pool like
  `witness_gate_batch`.

Then `mat_lam_soft_[i] = effective.lam_soft * lam_soft_scale`,
`mat_lam_hard_[i] = effective.lam_hard * lam_hard_scale`, and `mat_gate_active_[i]` is the gate
bit (1 when the gate is disabled, as today). Between decisions the last columns are held.

**Read surface:** `material_controller_coefficients()` returns the last predicted and effective
columns per agent ([n] each, with alphas ragged and `obs_offsets`). The existing
`material_gate_active()` and the drive telemetry's `lam_soft` carry the rest.

### 4.3 Mode `lam`: next to the coef_mlp (the default, C3)

`coef_mlp` keeps producing alpha, beta and gamma for the SDF drive (`drive_step_material`). The
energy net supplies only the two lambdas. The net's per-obstacle alphas, beta and gamma are
computed and reported but not executed.

This is the twin of `live_material_kwargs`, which hands GRL-SNAM's live rollouts (`sdf_rollout`,
`bicycle_rollout`) only the material field, `lam_soft`, `lam_hard`, `k_sharp` and `d_hat`. Its
docstring says the per-obstacle alphas "remain the responsibility of the base geometry adapter
instead of being collapsed into one SDF coefficient". `sim_world`'s drive is that base adapter. The
`material_drive` it builds already takes `k_sharp` and `d_hat_m` from `material_config`, which
§1.3 has pinned to the contract.

A lam-head `coef_mlp` would overwrite these lambdas inside `drive_step_material`. Attaching
refuses one (E8).

### 4.4 Mode `full`: instead of the coef_mlp (deferred, C4)

The net's alphas, beta and gamma would drive too, through the disc-based force law
`bicycle_rollout_material` already has: per-obstacle IPC barriers on the visible discs with reach
`geometry_d_hat_m`, the goal spring and damping, and the material forces. This is closest to the
Austin evaluation, which executes all six coefficients, but on the bicycle rather than the point
mass. It changes what `sim_world` executes far more than mode `lam` does, needs its own
behavioural acceptance, and is not needed for "same decisions". It is specified only far enough
here to keep the option and the enum value; its own short spec comes with C4.

### 4.5 Cost and units

- **Cost.** One forward per agent per decision: a transformer over at most 37 tokens and a CNN
  over a 3x25x25 patch, plus one crop and one gate. The C3 PR measures it on the golden net size
  and on the Austin size (d_tok 64, 2 layers) and documents the per-agent time. `decide_every`
  exists for large swarms.
- **Units.** Tokens, the crop and the gate work in world metres. `sim_world`'s normalized frame
  (`o_`, `goal_`) is converted once per agent, as the gate block already does. Python's
  `feature_scale_m` is not `sim_world`'s `scale`; the two are unrelated.
- **Plant.** "Same decisions" means the same coefficients and gate bit from the same state. The
  trajectories still differ, because `sim_world` executes them on its own plant and Python's
  evaluation on a 1 s point mass capped at 3 m/s.

## 5. The standalone rollout and the live arguments

- **`rollout`** is the twin of `BeliefMaterialController.rollout`. It runs
  `integrate_surrogate_material` with the command's effective coefficients and
  `d_hat = geometry_d_hat_m`. It sets `surrogate_material_params::d_hat_sdf` and `k_sharp` from the
  `material_config` §1.3 checked, and `robot_radius`, `margin_factor` and `mass` from arguments
  (defaults 0, 0.5, 1). The rollout patch is the caller's unmasked 6-plane patch, as in Python.
  The same shape checks apply: the alphas match N, and the patch is 6 x H x W with H, W >= 2.
  `mu_lat` is carried in the command and not used.
- **`live_material_kwargs`** becomes `material_drive_for(command, stack, mc)`: a `material_drive`
  with the stack, one-agent `lam_soft`/`lam_hard` pointers into the command, and the checked
  `k_sharp`/`d_hat_m`. This is what a host driving its own vehicle with `drive_step_material`
  needs.

Both are thin compositions of existing FLOAT kernels. They add no new numerical surface.

## 6. Goldens from the Python reference

The Python controller is the reference, so this time the golden is generated in GRL-SNAM and
committed in libcvc. The belief roadmap went the other way: libcvc's kernels wrote golden 1 and
the torch side matched it.

- **Generator:** `python -m grl_snam.tools.belief_material_controller_golden --out DIR` (T-a, a
  new GRL-SNAM module). It writes `belief_material_controller_golden.json` and
  `belief_material_controller_golden.cvcnm`, the same bytes on every run and OS. Every float32 is
  written as its raw bits (a uint32). C1's version writes G1–G5; C2 extends it with G6–G7.
- **Committed** in libcvc `src/cvc/tests/data/`, next to `belief_material_golden.json`. A GRL-SNAM
  test regenerates both into a temp directory and compares bytes with a copy in
  `tests/fixtures/`. So a change to the Python controller fails GRL-SNAM's CI until the golden is
  regenerated and re-committed on both sides.
- **The scene** is the 32x32 synthetic scene of `belief_material_golden.json` (two occluding
  buildings, a hard pond and canopy outside the sense truth, a risk-0.6 patch). Python already
  reproduces it bit for bit. The agents' poses are scripted, so no drive enters the golden, and
  their goals are the scene's own (on the grid, so C2's gate sees them). The tokens are built on
  the planes of three of its cases (the defaults, the 7 m material range, the pessimistic pins),
  which the JSON carries. A second, 9 x 9 scene puts equal distances at the cutoff (§2.2).
- **The net** is a narrow `CoefEnergyNetMaterial(risk_channels=3)`, as in golden 3 of the belief
  roadmap: d_tok 16, nhead 2, 1 layer, d_risk 16, P 9 over a 36 m extent. Its weights come from
  numpy's PCG64 stream (uniform over `±1/sqrt(fan_in)`), not torch's initialisers, so the file
  does not depend on the torch version. The lambda heads' last biases are 0, so both lambdas sit
  mid-range, which keeps the gate's effect visible. It is saved as v2 with the default contract in
  `meta`. The encoder and head widths are fixed by the architecture, so the file is 241 KB; the
  forward is the same code at any size.

**Cases and tiers:**

| # | what | content | tier |
|---|---|---|---|
| G1 | contract parse | 28 `meta` strings: valid, partial (defaults), integers, `repr` edge cases (`1e+16`, `5e-324`, `0.0001`), other keys holding `NaN`, a duplicate key, whitespace; then empty, not JSON, trailing text, no key, a non-object record, an unknown field, a wrong or non-string frame, a zero / negative / infinite (`1e999`) scale, a `NaN`, a trigger outside [0, 1], and `"5"` / `true` / `null` numbers. An accepted one records its values and `to_meta()` | BIT (values and strings); for a refusal, only that both sides refuse |
| G2 | header check | the golden `.cvcnm` against 12 `belief_material_params` / patch settings: each field broken alone, one case with two (both reported), two within the 1e-6 tolerance | BIT (which fields) |
| G3 | runtime check | 6 `material_config`s: the contract's own, k_sharp, d_hat and trigger each off by 1e-3, all three, and all three within the tolerance. Plus `material_config_for` on 8 cell widths | BIT |
| G4 | tokens | 3 belief cases x 3 ticks x 3 agents (the B1 golden's poses and goals), plus a 7.5 m scale and an empty obstacle set: `obs_feats`, `goal_feats` and the full `risk_patch` (raw float32 bits) | BIT, `dist` 1 ulp |
| G5 | visible obstacles | each agent's `last_visible` after its own sense tick: centres, radii and weights (float32 bits) with `max_obstacles` 36 and 5; plus the 9 x 9 tie scene | BIT |
| G6 | decide | the same agents and ticks, gate on and off: predicted alphas/beta/gamma/lam_soft/lam_hard/mu_lat, the gate bit and its `nominal_risk`/`best_risk`/`feasible_count`, effective lam_soft/lam_hard | gate BIT; zero pattern of `lam_soft_eff` BIT; values FLOAT (rtol 1e-4, the forward's tier) |
| G7 | rollout | one decided command per agent through `rollout` (5 steps, dt 0.5, the Austin constants) | FLOAT (rtol 1e-4 / atol 1e-5, the existing material rollout tier) |

**C++-only tests** (behavioural, no Python):

- **`sim_world` wiring.** With private belief and one agent per plane, the inputs and lambdas the
  wiring produces on each tick equal standalone `decide` on the same planes, byte for byte.
- **Defaults.** No controller leaves every run byte-unchanged, `set_material_belief` included.
- **Refusals.** Attaching throws on the oracle path, on a lam-head `coef_mlp`, on a §1.2 mismatch
  and on a §1.3 mismatch; material changes detach the controller.
- **Cadence.** `decide_every` holds the lambdas between decisions.
- **Scale.** `set_material_lam_scale` multiplies the net's lambdas after the gate.

There is **no trained-checkpoint golden** yet. `austin-material-train` needs a published Stage-1
geometry checkpoint, and GRL-SNAM's `MATERIAL_NAV.md` says none exists. When the first
reproducible Austin run lands, its `material.cvcnm` becomes a smoke test: it loads, passes §1.2
and §1.3 with `material_config_for(contract)`, and decides on the golden scene.

## 7. pycvc

New bindings, each a new symbol GRL-SNAM can probe (a `HAS_` flag on its side, as for the belief
kernels). Refusals raise `ValueError`. C1's, checked against the golden by
`bindings/pycvc/test_pycvc_nav_belief_controller.py`:

- `nav_belief_material_contract(meta)`, returning a dict of the fields plus `meta` (its
  `to_meta()`);
- `nav_belief_material_contract_to_meta(feature_scale_m, k_sharp, d_hat_sdf_m,
  material_gate_trigger, geometry_d_hat_m)`;
- `nav_belief_material_check_cvcnm(path, unknown_risk, unknown_hard, unknown_gate_hard,
  material_range_m, patch_size, patch_extent_m)`: the §1.2 header check and the contract parse,
  returning the contract dict;
- `nav_visible_obstacles(last_visible, truth, min_x, min_y, max_x, max_y, px, py,
  max_obstacles)`, returning `(centers, radii, weights)`;
- `nav_belief_material_controller_inputs(px, py, gx, gy, centers, radii, weights, risk, hard_b,
  seen, min_x, min_y, max_x, max_y, patch_extent_m, P, feature_scale_m, unknown_risk,
  unknown_hard)`, returning `(obs_feats, obs_mask, goal_feats, risk_patch)`.

Later:
- C2: `NavBeliefMaterialController(path, ...)`, with `decide`, `check_runtime` and `contract`;
- C3: `sim_world.set_material_controller(...)`.

GRL-SNAM's parity test can then check the native controller live against the torch one, as
`test_material_train_torch.py` does for the trainer.

## 8. Work breakdown

**libcvc**, each PR additive and default-off:

- **C1 — contract and inputs (BIT).** `belief_material_controller.h`: the contract parse, `to_meta`,
  the header and runtime checks, `material_config_for`, the token builder, `visible_obstacles`, and
  goldens G1–G5 with their pycvc bindings.
- **C2 — decide (FLOAT forward, BIT gate).** `belief_material_controller` with `decide`,
  `decide_batch`, `rollout`, `material_drive_for`, and goldens G6–G7.
- **C3 — `sim_world`, mode `lam`.** `set_material_controller`, the tick wiring, the local goal, the
  read surface, the C++-only tests, `NAV_MATERIAL.md`, and the cost measurement.
- **C4 — mode `full` (later, optional).** Its own short spec first (§4.4).

**GRL-SNAM**, before or alongside C1:

- **T-a.** The golden generator and its byte-compare test (§6), stacked on T-b.
- **T-b.** `scan_visible_geometry` sorts stably (E5), and the contract refuses non-number values
  (E6): CVC-Lab/GRL-SNAM#163. Neither changes a committed result: there is no trained Austin
  checkpoint, and the route corpus does not depend on the obstacle order.

**cvcdbg** (separate, after C3): see the note in the libcvc workstream. demo3,
`dbg_arrival_check3` and `dbg_drive` use only the truth-plane `set_material` today.

## 9. Decisions (settled)

**Settled 2026-11-08:** Joe took the middle column for all ten ("Yes, start C1"), which is what
the spec above assumes. The last column records the alternative each one rejected and why.

| # | Decision | Settled | Rejected alternative and its cost |
|---|---|---|---|
| E1 | What the net drives in `sim_world` | Mode `lam`: the net's lambdas next to `coef_mlp`'s alpha/beta/gamma, the twin of `live_material_kwargs` | Mode `full`: all six coefficients through disc barriers on the bicycle. Closer to the Austin evaluation, but a different executed force law in `sim_world`, so it needs its own acceptance; deferred to C4 |
| E2 | The goal for the token and the gate in `sim_world` | The agent's goal clipped to `local_goal_m` (30 m, training's `teacher_lookahead_m`) along the straight line, plus a caller override column | The raw goal: `dist` far outside what the net saw on long routes (1.5 in training, up to 3 for the evaluation's 60 m waypoints). The carrot: a few metres away and deflected around obstacles, also off-distribution. Porting `choose_material_local_waypoint`: the exact evaluation twin, but its own spec and BIT surface; it plugs into the override column later |
| E3 | Obstacle source in `sim_world` | `last_visible(m) & truth()` of the agent's plane: exact for private belief or one agent per plane | Per-agent visibility: exact for shared planes too, but `sense_batch` keeps no per-agent visible raster, so it needs a new output |
| E4 | Decision cadence | Every tick (`decide_every` 1): the Austin evaluation decides every step | Every k ticks: k times cheaper, with lambdas up to k ticks stale |
| E5 | Obstacle tie order | Stable on (distance, row-major index) on both sides, a one-line GRL-SNAM change | Leave numpy's unstable sort: the goldens must avoid ties at the cutoff, and live parity can differ on tied scenes |
| E6 | Contract value types | JSON numbers only in C++, and Python tightened to match | Mirror Python's `float()` coercion (`"5"` and `true` accepted): more code, and it keeps a loose format |
| E7 | Header and runtime tolerances | abs 1e-6, Python's | Exact: the header's float32 fields would then have to be compared after a float32 round trip on both sides |
| E8 | A lam-head `coef_mlp` with a controller attached | Refuse at attach | Let the controller win silently: two producers of the same columns, one of them ignored |
| E9 | Effective lambdas and `set_material_lam_scale` | The scale multiplies after the gate; scale 1 is Python's `effective` | Ignore the scale while a controller is attached: demo3's force-bias slider would stop working on a learned policy |
| E10 | Constants the contract does not pin (`sigma`, gate horizon and margins) | Python's values, derived by `material_config_for` (the 25 m horizon in cells, the 2-cell margin, sigma 1) and pinned by the goldens. Propose optional `material_sigma_cells` and `gate_horizon_m` fields in a later contract version; missing fields take defaults, so old files still load | Pin them now: a contract change in GRL-SNAM first, and every existing checkpoint lacks them anyway. Or leave libcvc's 12-cell default: the gate then looks 180 m ahead on the Austin 15 m grid where training looked 30 m |
