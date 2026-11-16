# Convoy formations as a port-Hamiltonian interconnection: measured

Status: measured on 2026-11-14, on Phase 1 of `cvc::ph` composition ([PH_COMPOSE.md](../PH_COMPOSE.md),
[PH-COMPOSE-ROADMAP.md](PH-COMPOSE-ROADMAP.md)). Nothing is integrated. `ph_convoy_test`
reproduces every number here (see [Reproducing](#reproducing)).

## The question, and the answer

A convoy formation is a linkage: each follower's `formation_parent` and its slot behind that
parent (`cvc/nav/nav_stats.h`). Followers usually chase their slots. The question was whether the
formation holds better if the linkage is instead a power-preserving interconnection of
port-Hamiltonian blocks: vehicles as masses, links as storage, the coupling doing no net work.

**No.** On point masses on a straight road, the case most favourable to it:

- **Against slot chasing.** The pH formation holds the slot at speed: it ends on the slot, while
  slot chasing ends 20 m behind it at 8 m/s. It does worse on everything else:
  - it settles more slowly (24 s against 14.5 s on the column's speed step) and overshoots (38%
    against 0%);
  - in an emergency stop it comes closer (12.4 m against 21.9 m) and backs up afterwards, at
    4.4 m/s; slot chasing never backs up.
- **What holds the slot is constant spacing and the relative speed, not the pairing.** The same
  link laws pulling only the follower (`pd`) beat the pH formation on peak error, settling and
  closest approach, in all four scenarios. They also do so with the lead's speed given to every
  follower (`pd_lead` against `ph_lead`).
- **What the pairing buys is robustness.** Disturbances do not grow down the column (spacing-error
  ratio 0.83, against 1.18 for `pd`), and errors stay bounded at any column length and any gains.
  `pd` collides at 20 vehicles, and with little damping at 5. The price is settling that grows with
  the square of the column length: 24 s, 102 s and 385 s at 5, 10 and 20 vehicles. It also puts
  the whole column's load on the link off the lead.
- **No energy saving.** The pH formation spends more on traction and braking than slot chasing in
  both scenarios.
- **The books are informative but virtual.** Per link, they show where the energy of a manoeuvre
  goes: 47% of it into the link off the lead. That energy is the controller's, though. Each engine
  delivers only the net of its links' forces, so it is not fuel.

So the convoy is no reason for Phase 5's state-dependent port maps (the
[verdict](#verdict-phase-5-and-a-full-simulation-trial) is below).

## The model

**The plant.**

- Unit-mass point masses, so energies are J/kg (kJ per tonne).
- Linear drag of 0.02 /s, which each engine cancels from its own speed.
- Net acceleration limits of +2 / -4 m/s² along the road and ±2.5 across it.
- Steps of 10 ms. Slots are 22 m behind the parent.

**The lead.** Its speed loop is `v' = clip((v_ref - v) / 0.8)`, the same in every run. It does
not feel the column, so it is a velocity source.

**The scenarios.**

- A **speed step**: `v_ref` from 4 to 8 m/s at t = 2 s.
- An **emergency stop**: `v_ref` from 8 to 0 m/s, so the lead brakes at its 4 m/s² limit.
- Each runs for 90 s.
- Each runs on a **column** of 5 (parent i-1) and on a **wedge** tree: 1 and 2 off the lead,
  3 behind 1, 4 behind 2, every slot 22 m back and 22 m to the side.
- Every follower starts at its law's steady state at the starting speed.

**The follow laws.**

| law | what it is |
| --- | --- |
| `chase` | Slot chasing, as a trail-pursuit follower does it: `v_w = clamp(0.4 s, 0, 10)`, `a = clamp((v_w - v) / 0.8)`. No reaction on the parent. It settles 2.5 v behind its slot (a 2.5 s time headway) and never backs up. |
| `pd` | A virtual link pulling the follower only. The link is a slot spring k = 0.5 /s² in parallel with a Maxwell branch (spring 8c in series with a dashpot c = 1.25 /s), so below 8 rad/s it adds a dashpot c on the relative speed. Constant spacing. |
| `ph` | The same link laws as cvc::ph storage blocks, wired by the linkage: each link pulls its child and holds its parent back. |
| `ph_twin` | `chase`'s gains made power-preserving: pure spring links (0.4 / 0.8) and the follower's 1 / 0.8 speed damping, with reactions. |
| `maxwell` | The microgrid line law taken literally: one elongation state, `H = 1/2 k e²`, R the link damping (k = 0.5, R = 1 / 1.25). |
| `pd_lead`, `ph_lead` | `pd` and `ph`, plus a dashpot (c = 1.25) from every follower to the lead's broadcast speed. |

`pd` against `ph`, and `pd_lead` against `ph_lead`, use the same information and the same link
laws. The only difference within each pair is the reaction on the parent, which is what makes
the coupling power-preserving.

### How it maps onto cvc::ph

- **A follower** is a `canonical_law` with n = 2: q is the position, p the momentum, the damping
  its drag (plus `ph_twin`'s speed damping). It has a coupling port and an input on its momentum.
- **A link** is a `linear_law`, one elongation per branch and axis, `H = 1/2 sum_b k_b |e_b|²`,
  `R_b = relax_b / k_b`. Its port's effort is the link force.
- **The wiring** is `connect_incidence` over the linkage, followers as buses and links as lines.
  This is the DC-microgrid structure with the roles swapped: vehicle velocity plays bus voltage,
  link force plays line current.
- **The same law objects** are wired as the column and as the wedge. In both, `check_balance_random`
  puts the pairing's power below 1e-14.

The literal line law cannot hold a slot. Its spring and dashpot are in series, so under any
sustained force the link flows: it has no rest length. It ends the column's speed step 12.8 m off
the slot, and the stop 25.6 m off, which puts the follower past its parent (`maxwell`'s 0 m gap).
The link used everywhere else keeps a pure slot spring (R = 0) and adds the dashpot as a Maxwell
branch in parallel. That makes a two-branch `linear_law`, still a storage block.

**The composite is written in the lead's frame.**

- A follower's q is its position less the lead's, and p its momentum less the lead's speed. The
  lead's acceleration is an inertial input on each follower.
- The obvious alternative, the lead's speed fed to the links off it as a flow input, does not work
  with the split step. The lead's speed reaches those links in the local step and the follower's
  speed in the coupling half-steps. Within every substep the link therefore holds a spurious
  elongation of ±v d/2, which its stiff branch turns into force.
- The slot then creeps at constant speed: 9.45 cm in 90 s at 8 m/s with 2 substeps, and 2.36 cm
  with 4 (second order). In the lead's frame it does not creep at all
  (`PhConvoy.AFlowInputOnALinkCreepsUnderTheSplitStep`).
- This holds for any external flow source driving a line in `cvc::ph`. The source should be a
  bus, or the system should be written in its frame.

**Other details.**

- The engines' drag cancelling, the speed damping and the acceleration limits are inputs, held
  over each step.
- An independent RK4 of the same equations, in the road's frame, matches the composite to 4 mm
  over 90 s, for every pH law, both wirings and both scenarios.
- The composite's books close to 3e-3 J/kg over a run (the worst step is 2e-5).

### The measures

| measure | definition |
| --- | --- |
| peak, end | Slot error s: distance behind the slot (m). `peak` is max \|s\|, `end` is s at 90 s. |
| settle | Seconds after the lead's command until \|s - s_end\| stays under 1 m. |
| oversh | Excursion past s_end, opposite the largest excursion, as a fraction of that excursion. |
| gap min | Closest parent-child distance (here also the closest pair). |
| γ ratio | String stability. γ_i = max \|s_i - h v_i\| is the spacing-policy error, with h = 2.5 s for `chase`, the subtree's headway for `ph_twin`, 0 for the rest. The ratio is the largest γ_child / γ_parent over follower parents; above 1, disturbances grow down the column. |
| a max, a min, v min | Followers' largest and smallest net acceleration, and lowest speed (< 0: backing up). |
| E+, E- | Followers' engine work, J/kg: traction (including the drag) and braking. The lead's work is the same in every run. |

All tables report the worst follower.

## Results: 5 vehicles

**Column, speed step.**

| law | peak | end | settle | oversh | gap min | γ ratio | a max | E+ | E- |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chase | 20.03 | 20.00 | 14.5 | 0% | 32.00 | 0.88 | 0.94 | 518.9 | 0.0 |
| pd | 2.87 | 0.00 | 7.5 | 16% | 21.53 | 1.18 | 2.00 | 574.0 | 27.7 |
| ph | 4.79 | 0.01 | 24.0 | 38% | 20.35 | 0.83 | 1.38 | 566.2 | 17.8 |
| ph_twin | 79.54 | 79.54 | 64.9 | 0% | 32.00 | 0.79 | 0.74 | 460.9 | 0.0 |
| maxwell | 13.00 | 12.80 | 10.9 | 3% | 22.00 | 0.75 | 1.17 | 519.3 | 0.0 |
| pd_lead | 1.18 | 0.00 | 4.0 | 0% | 22.00 | 0.81 | 2.00 | 544.8 | 0.0 |
| ph_lead | 1.55 | 0.03 | 9.4 | 0% | 22.00 | 0.58 | 1.96 | 544.8 | 0.0 |

**Column, emergency stop.**

| law | peak | end | settle | oversh | gap min | γ ratio | a min | v min | E+ | E- |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chase | 20.00 | 0.07 | 15.8 | 0% | 21.93 | 0.88 | -1.88 | 0.00 | 21.4 | 108.4 |
| pd | 5.73 | 0.00 | 8.0 | 16% | 16.27 | 1.18 | -4.00 | -5.14 | 36.2 | 141.4 |
| ph | 9.58 | 0.02 | 35.5 | 38% | 12.42 | 0.83 | -2.76 | -4.35 | 41.3 | 138.6 |
| ph_twin | 80.00 | 0.91 | 73.3 | 0% | 22.32 | 0.79 | -1.49 | 0.05 | 22.1 | 84.0 |
| maxwell | 25.99 | 25.60 | 12.0 | 3% | 0.00 | 0.75 | -2.35 | -0.14 | 20.1 | 110.6 |
| pd_lead | 2.37 | 0.00 | 8.8 | 0% | 19.63 | 0.81 | -4.00 | -0.43 | 10.9 | 122.4 |
| ph_lead | 3.09 | 0.06 | 21.9 | 0% | 18.91 | 0.58 | -3.93 | -0.22 | 10.7 | 122.0 |

**Wedge, speed step.** The parent-child gap at the slot is √2 × 22 = 31.1 m.

| law | peak | end | settle | oversh | gap min | γ ratio | a max | E+ | E- |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chase | 20.03 | 20.00 | 8.4 | 0% | 38.83 | 0.72 | 0.94 | 528.8 | 0.0 |
| pd | 2.13 | 0.00 | 5.8 | 3% | 31.06 | 1.11 | 2.00 | 550.2 | 5.0 |
| ph | 3.37 | 0.00 | 7.8 | 13% | 30.85 | 0.54 | 1.54 | 550.7 | 5.1 |
| ph_twin | 40.00 | 40.00 | 17.9 | 0% | 38.83 | 0.64 | 0.74 | 517.1 | 0.0 |
| maxwell | 8.48 | 6.40 | 9.6 | 50% | 31.11 | 0.57 | 1.23 | 540.1 | 3.0 |
| pd_lead | 1.18 | 0.00 | 4.0 | 0% | 31.11 | 0.66 | 2.00 | 544.7 | 0.0 |
| ph_lead | 1.48 | 0.00 | 7.0 | 0% | 31.11 | 0.42 | 1.99 | 544.7 | 0.0 |

**Wedge, emergency stop.**

| law | peak | end | settle | oversh | gap min | γ ratio | a min | v min | E+ | E- |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| chase | 20.00 | 0.07 | 9.4 | 0% | 31.06 | 0.72 | -1.88 | 0.00 | 14.4 | 112.9 |
| pd | 4.26 | 0.00 | 6.7 | 3% | 28.26 | 1.11 | -4.00 | -2.17 | 17.0 | 126.7 |
| ph | 6.73 | 0.00 | 8.7 | 13% | 26.78 | 0.54 | -3.07 | -2.39 | 19.9 | 127.7 |
| ph_twin | 40.00 | 0.00 | 21.8 | 0% | 31.11 | 0.64 | -1.49 | 0.00 | 14.4 | 108.5 |
| maxwell | 16.97 | 12.80 | 10.4 | 50% | 22.57 | 0.57 | -2.46 | -1.54 | 18.6 | 119.9 |
| pd_lead | 2.37 | 0.00 | 8.4 | 0% | 29.49 | 0.66 | -4.00 | -0.43 | 10.8 | 122.8 |
| ph_lead | 2.96 | 0.00 | 12.7 | 0% | 29.10 | 0.42 | -3.98 | -0.36 | 10.7 | 122.6 |

**Reading the tables.**

- **The reactions cost.** At equal information, they cost about 1.6x the peak error (`ph` / `pd`)
  and 1.25-1.3x (`ph_lead` / `pd_lead`), a longer settling, and a closer approach in the stop. The test
  pins these comparisons for every scenario.
- **The bidirectional chain's front link tows the column.** On the column, `ph`'s error is largest
  at the front and falls toward the back: 4.79, 3.98, 2.87, 1.51 m. `pd`'s grows toward the back:
  1.92, 2.13, 2.44, 2.87 m.
- **A shallower tree helps every law.** The wedge's depth is two, so its gap between `pd` and `ph`
  is smaller than the column's.
- **Slot chasing's 2.5 s headway is a tradeoff.** It gives up the slot at speed in exchange for no
  overshoot, no reversing, and never closing inside its standoff. With h = 2.5 s at least twice
  its 0.8 s lag, it is already string-stable (γ ratio 0.88).
- **Energy.** `chase`'s lower E+ on the step is drag it does not pay: its column ends 40 m
  longer, so between them its followers cover 100 m less. Its E- is 0. On the stop, the
  constant-spacing laws brake harder, because they start 20 m closer.
- **`ph_twin` piles up.** With reactions, each link carries the speed damping of every follower
  behind it. The link off the lead stretches by the subtree's headway: 4 × 2.5 × 8 = 80 m on the
  column. Slot chasing's headway cannot be made power-preserving without this.

## Column length

The speed step and the stop, over a 400 s run. The `k2c5` rows use the stiffest, most damped links
from the gain study below (k = 2, c = 5).

**Speed step: peak (m) / settle (s).**

| law | 5 | 10 | 20 |
| --- | --- | --- | --- |
| chase | 20.03 / 14.5 | 20.03 / 28.9 | 20.03 / 56.4 |
| pd | 2.87 / 7.5 | 5.38 / 16.4 | 246 / 84.6, collides |
| ph | 4.79 / 24.0 | 5.57 / 102 | 5.66 / 385 |
| pd k2c5 | 0.87 / 0 | 1.18 / 5.2 | 1.69 / 8.8 |
| ph k2c5 | 1.73 / 5.9 | 2.52 / 12.4 | 2.80 / 74.4 |
| pd_lead | 1.18 / 4.0 | 1.18 / 4.0 | 1.18 / 4.0 |
| ph_lead | 1.55 / 9.8 | 1.55 / 10.2 | 1.55 / 8.5 (0.12 m left at 400 s) |

**Emergency stop: peak / settle / gap min / v min.**

| law | 5 | 10 | 20 |
| --- | --- | --- | --- |
| chase | 20.0 / 15.8 / 21.9 / 0 | 20.0 / 30.7 / 21.9 / 0 | 20.0 / 59.0 / 21.9 / 0 |
| pd | 5.7 / 8.0 / 16.3 / -5.1 | 35.5 / 28.8 / 11.2 / -15.0 | diverges |
| ph | 9.6 / 35.6 / 12.4 / -4.4 | 11.1 / 154 / 10.9 / -6.6 | 11.3 / 391 / 10.7 / -7.8 |
| pd k2c5 | 1.8 / 4.3 / 20.3 / -1.8 | 2.4 / 6.0 / 19.6 / -5.3 | 19.6 / 28.2 / 18.0 / -14.6 |
| ph k2c5 | 3.5 / 7.2 / 18.6 / -2.2 | 5.1 / 25.5 / 17.0 / -4.6 | 5.6 / 105 / 16.4 / -6.7 |
| pd_lead | 2.4 / 8.8 / 19.6 / -0.4 | 2.4 / 8.8 / 19.6 / -0.4 | 2.4 / 8.8 / 19.6 / -0.4 |
| ph_lead | 3.1 / 23.0 / 18.9 / -0.2 | 3.1 / 33.5 / 18.9 / -0.2 | 3.1 / 23.9 / 18.9 / -0.2 (0.23 m left) |

What this shows:

- **`pd` amplifies.** Its error grows geometrically down the column, as constant-spacing predecessor
  following does. Stiffer, more damped links slow the growth but do not stop it.
- **`ph` stays bounded but slows down.** Its error is bounded at every length. Its slowest mode,
  though, is the column's lowest elastic mode, whose frequency falls as 1/N.
- **The lead's speed changes the picture.**
  - `pd_lead` is independent of length: every follower hears the lead, so its error does not depend
    on its place in the column.
  - `ph_lead` keeps its peak, but the chain springs still leave an overdamped shape mode. Its rate
    is k λ₁ / c, with λ₁ ∝ 1/N², so 20 vehicles are still relaxing at 400 s.
- **Slot chasing stays bounded too.** It settles in a time linear in N: each follower adds about
  3 s of lag.

## Link gains

`pd` and `ph` on the column of 5, with k in {0.25, 0.5, 1, 2} /s² and c in {0.5, 1.25, 2.5, 5} /s
(`PhConvoy.LinkGains`).

- **Well damped (c ≥ 2.5).** `pd` is better at every gain.
  - Its best runs (c = 5): 0.8 m on the step, never leaving the 1 m band; on the stop, 1.5-1.8 m,
    4-12 s and a 20.3-20.5 m gap.
  - `ph`'s best (k = 2, c = 5): 1.73 m and 5.9 s on the step; 3.45 m, 7.2 s and an 18.6 m gap on
    the stop.
- **Lightly damped.** `ph` wins, and that is the passivity guarantee at work:
  - With c = 0.5, `pd` peaks at 6-107 m. It closes to 0 m (collides) on the stop at k ≥ 1, and
    backs up at up to 18 m/s.
  - `ph` stays within 15.2 m and never collides (its closest is 6.8 m).
  - At k = 2, c = 1.25 on the stop, `pd` peaks at 32 m and `ph` at 5.2 m.

So the pH linkage is the safer law to hand an untuned or learned link, and the worse one to tune.

## The energy books

The composite keeps the formation's own books, in the lead's frame: the followers' kinetic energy
relative to the lead, plus the links' stored energy. The lead's manoeuvre does work on the
followers through the inertial input, and that work ends up stored or dissipated in the links (each
follower's drag and its engine's cancelling of it balance). Numbers are J/kg, over the whole run.

| run | the lead's manoeuvre puts in | links dissipate, front to back | peak stored |
| --- | --- | --- | --- |
| `ph`, column step | 25.4 (4.2, 6.3, 7.3, 7.6 per follower) | 11.8, 8.1, 4.3, 1.2: 47% in the link off the lead | 5.7 J/kg in link 0-1 |
| `ph`, column stop | 101.5 | 47.3, 32.2, 17.2, 4.8: 47% in link 0-1 | 23.0 in link 0-1 |
| `ph_lead`, column step | 12.8 | chain links 1.7, lead dampers 11.1 (87%) | 0.6 |
| `ph_lead`, column stop | 51.1 | chain links 6.8, lead dampers 44.3 (87%) | 2.4 |

`ph_twin`'s books are different: its links are pure springs, and its speed damping is fed by an
input of its own. At the end of the column's step, the link off the lead still holds 1582 J/kg
(its 79.5 m stretch).

What the books show:

- **Where the disturbance goes.** They locate what is wrong with the bidirectional chain: the link
  off the lead does half of the column's work.
- **The pairing is not doing the work.** With the lead's speed available, the links to the lead
  absorb 87% of the disturbance, and the chain's reactions do very little.
- **The energy is virtual.** A link's "dissipated" energy is not a separate loss. It is throttle
  and brake work at the vehicles on its two ends, which their engines deliver as the net of their
  links' forces. The engine work is the E+ / E- of the tables.

## What the structure guarantees, and what it does not

**It guarantees passivity.** For any linkage and any positive gains, the pH formation cannot
inject energy into its own motion. The measurements show this in two places: `pd` collides at 20
vehicles and when lightly damped, and `ph` never does. Slot chasing gets the same boundedness from
its time headway, without the pairing.

**It allows rewiring without retraining.** The same law objects are wired as a column and as a
wedge, and the pairing stays power-free for both. But slot chasing also rewires by changing
`formation_parent`. The guarantee would matter for learned link laws: any `H >= 0`, `R >= 0`
stays passive in any wiring. That case was not measured here.

**It does not cover:**

- **Delays and losses.** A bidirectional link needs each parent to know its child's state. An
  interconnection over a delayed or lossy channel is no longer power-preserving unless it is
  passivated (wave variables, scattering).
- **Heading.** Real vehicles need heading-dependent attachment, which needs Phase 5.
- **Obstacles, and following a trail around corners.**

## Verdict: Phase 5, and a full-simulation trial

The convoy is not a reason to build Phase 5's state-dependent port maps, and a trial of the pH
linkage in the full convoy simulation is not worth running.

- **Point masses were the easy case.** Port maps are constant there and the attachment is exact,
  yet the power-preserving linkage already loses on every formation-holding measure:
  - to the same links pulling one way, with the same information;
  - to slot chasing, on settling, overshoot, closest approach in an emergency stop, and backing up.
- **Heading makes it harder.** Heading-dependent attachment on SE(2) vehicles adds the nonholonomic
  constraint, which slot chasing handles by pure pursuit along the parent's trail.
- **The one gain is already there.** Errors bounded for any column length and any gains, slot
  chasing gets from its time headway.
- **The cost grows with the convoy.** Settling grows as N², and the front link carries the column.
- **The real channel removes the guarantee.** Once the links run over a delayed, lossy channel,
  the guarantee is gone too.

Phase 5 should be justified by its own cases, the planar arm and legged blocks. If a full-simulation
trial is run, the law to try is not pH. Constant spacing on the predecessor's relative speed, plus
the lead's broadcast speed (`pd_lead`):

- holds the slot to within 1.2 m on the step and 2.4 m on the stop;
- settles in 4-9 s whatever the column's length;
- does not grow down the column.

What it trades: it closes to 19.6 m rather than 21.9 m in the stop, and it brakes harder.

## Reproducing

```
ctest -R PhConvoy                                     # the cross-check, the creep, the 5-vehicle matrix (about 20 s)
CVC_PH_CONVOY_EXPERIMENTS=1 bin/ph_convoy_test --gtest_filter='PhConvoy.ColumnLength:PhConvoy.LinkGains'   # about 4 min
```

The tests print every table above, prefixed `[ph-convoy]`, together with the per-follower
breakdowns and the full books.

- `PhConvoy.TheCompositeIsTheFormationItStandsFor` checks the composite against the RK4 and checks
  the pairing.
- `PhConvoy.AFlowInputOnALinkCreepsUnderTheSplitStep` pins the creep.
- `PhConvoy.FollowLawsOnTheStepAndTheStop` pins the comparisons made here.
