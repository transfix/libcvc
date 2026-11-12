/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.

  libcvc is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// ewave.h — waves that bodies make, on a square window of the sea
// (docs/roadmap/OCEAN-AND-WEATHER-ROADMAP.md, C1, C3 and C5): Tessendorf's eWave on the CPU.
//
//   cvc::ocean::ewave_params p;   // n = 256 cells of dx = 0.5 m, deep water
//   cvc::ocean::ewave w(p);
//   w.set_mask(mask);             // 1 open water, 0 land and obstacles, between: absorbing beach
//   w.add_height(x, y, -0.2, 1.0); // a dropped crate's dent
//   w.step(1.0 / 30.0);
//   w.at(x, y);                   // the wave height there (m, above the still level)
//
// The state is the surface height h and the surface velocity potential phi on n x n cells (n a
// power of two) of side dx, cell (i, j) at origin + (i dx, j dx), stored row-major (row j).
//
// Linear water waves are Hamiltonian: H = 1/2 (g h.h + phi.K.phi), h_t = K phi, phi_t = -g h,
// with K the Dirichlet-to-Neumann operator, kappa(k) = |k| tanh(|k| d) per wavenumber in one
// depth d (omega = sqrt(g kappa)). A step of dt takes one of two paths:
//
// Without ghost cells (no walls, walls = zero, or ghost_cells = 0): the exact step (O8a).
//   - h and phi pack into one complex field z = h + i phi, which one forward FFT takes to
//     wavenumbers (fft.h, sign -1);
//   - each wavenumber k and its mirror -k are separated into H(k) and PHI(k) through Hermitian
//     symmetry and moved on exactly:
//       H'   = H cos(omega dt) + (kappa / omega) PHI sin(omega dt)
//       PHI' = PHI cos(omega dt) - (g / omega) H sin(omega dt)
//     then damped by exp(-(damping_k2 |k|^2 + damping) dt); the inverse FFT (sign +1) brings them
//     back. The update is exact for any dt: there is no CFL limit. The per-mode coefficients are
//     computed for a dt and kept while dt is unchanged.
//   - the real-space pass below.
//
// With ghost cells (walls = no_flux, the default, and walls near water): the symmetric step
// (O8c), a no-flux wall that conserves energy by construction. The state lives on the water cells
// (mask above 0) only; wall cells hold h = phi = 0.
//   - R takes the water to the extended field over the whole window: water cells copy, each
//     ghost cell (a wall cell whose mirror image across the nearest shore lands in water,
//     ewave_wall_ghosts) takes that image, the weighted sum of its corners, and every other wall
//     cell 0. R^T folds a field back: each water cell takes its own value plus, for each ghost
//     that reads it, that ghost's weight on it times the field at the ghost.
//   - S = R^T G R, G the periodic operator kappa~(k) on the window (FFT, multiply, inverse FFT),
//     is symmetric positive semidefinite for any set of ghosts and weights. M, the lumped mass,
//     is the diagonal of the row sums of R^T R (ewave_lumped_mass): 1 + the weights of the
//     ghosts that read water cell i, each times its ghost's total weight. The water moves by
//     h_t = M^-1 S phi, phi_t = -g h, whose energy H = 1/2 (g h.M.h + phi.S.phi) is conserved.
//     Where a straight wall is mirrored cell for cell (weights 1), M = 2 and M^-1 S is exactly
//     the even extension's operator: the exact Neumann wall. Elsewhere (a staircase, a curve, an
//     island, a thin wall, an image cut short) it is first order in where it reflects, but it is
//     never unstable, as overwriting the ghosts with the image (O8c's first build) was.
//   - Stormer-Verlet, symplectic, in s equal substeps of dt_s = dt / s:
//       phi_half = phi - (g dt_s / 2) h;  h' = h + dt_s M^-1 S phi_half;
//       phi'     = phi_half - (g dt_s / 2) h'
//     s is the fewest that keep omega_max dt_s <= 1.8 (ewave_max_phase), omega_max = omega at
//     the window's corner wavenumber sqrt(2) pi / dx (ewave_substeps): one substep at 1/30 s for
//     cells down to about 0.2 m. With nonnegative weights S <= max(G) R^T R <= max(G) M, so the
//     generalised eigenvalues of (S, M) are at most G's largest and every mode is inside
//     Stormer-Verlet's limit omega dt_s < 2: any dt is stable. The step conserves the modified
//     energy 1/2 (g h.M.h - (g dt_s / 2)^2 h.S.h + phi.S.phi) exactly (a quadratic invariant of
//     the linear map, for any symmetric S and positive M), which is positive definite under that
//     limit, so nothing can grow. H itself wobbles below it: a mode started from h alone keeps
//     H within [(x cot x)^2, 1] of its start, x = omega dt_s / 2 (0.99 at omega dt_s = 0.2, 0.51
//     at the limit, in the shortest waves only).
//   - kappa~ = 4 sin^2(omega dt_s / 2) / (g dt_s^2), not kappa: Stormer-Verlet's discrete
//     frequency then satisfies cos(omega~ dt_s) = 1 - g kappa~ dt_s^2 / 2 = cos(omega dt_s), so
//     open water keeps eWave's exact dispersion at any dt, and a straight wall's modes their
//     exact frequencies. (With kappa itself a ring's phase drifts: ocean_ewave_test measured a
//     straight wall's reflection 55 % off its even image at dt 0.1 s, against round-off with
//     kappa~.) kappa~ <= kappa is still a nonnegative multiplier, so the argument above holds. h
//     is exact at each step for a mode started from h alone; phi's amplitude is x cot x of the
//     exact (within 1 % for omega dt_s < 0.35).
//   - Then, once a step: damping_k2 as h <- M^-1 R^T F R h with F = exp(-2 damping_k2 |k|^2 dt)
//     (one more FFT pair, only when damping_k2 > 0): self-adjoint under M with its eigenvalues in
//     [0, 1], so it only takes potential energy, and twice the rate on h alone takes a mode's
//     energy at the exact step's mean rate. Then the real-space pass below, the beach and the
//     sponge on h alone at twice their rate (a diagonal factor under the diagonal M only takes
//     potential energy; one on phi need not, as S is not diagonal), the global damping on both
//     (a uniform factor scales H). Every one of these only removes energy.
//   - The extended field must not step down to 0 where the image is cut, nor jump between two
//     shores' images: the fold carries G's response to that step into the water, and waves
//     longer than the cut's distance see it more than the wall. So the default band reaches as
//     far as the image does (ghost_cells = 4096: every wall cell whose mirror point lands in
//     water), and near the cuts that remain (a mirror point out of the window or among walls, a
//     ghost dropped by the readers' limit) and the seams (neighbouring ghosts mirroring more than
//     2.5 cells apart: the two faces of a thin wall, a wall's medial axis) the weights taper to 0
//     over ewave_taper_cells, as sin^2. Measured on a straight wall (ocean_ewave_test): the
//     whole image matches the even image to 2.9e-14 without the sponge and 2.2 % with it (the
//     sponge's h-only form); a band of 96 cells to 12 %, of 16 cells to 23 %.
//   - Ghost limits keep the step symmetric: a ghost dropped is a row of R dropped, never one
//     corner of a ghost from the fold but not the fill. No water cell is read by more than
//     ewave_max_readers ghosts; where more would (a small convex pond compresses its image),
//     whole ghosts are dropped, farthest from the shore first (ewave_wall_ghosts), on the CPU and
//     the GPU alike.
//
// The real-space pass: h and phi are damped by a rate per cell, as exp(-rate dt), so the damping
// per second does not depend on the step size. A beach value m of the mask (0 < m < 1) adds
// beach_rate (1 - m); the sponge adds a rate that ramps up as r^2 over the last sponge_cells cells
// toward the window's edge (r from 0 to 1), to sponge_rate at the edge. The sponge absorbs
// outgoing waves and hides the transform's wrap-around: the window is periodic. Walls (m = 0) are
// set to h = phi = 0 every step. (On the symmetric step, h alone at twice the rate: see above.)
//
// Walls: a mask value of exactly 0 is a wall (land, a cliff, an obstacle; beaches, 0 < m < 1, only
// damp as above). ewave_params::walls picks what a wall does:
//   no_flux (the default): a real vertical wall, d(phi)/dn = 0, about which h and phi are even,
//     so a crest reflects as a crest and the wall is an antinode: the symmetric step above.
//     ghost_cells (default 4096, as far as the image reaches) caps how far into a wall its image
//     goes: it sets accuracy, never stability, and costs only set_mask time (39 ms for a 256^2
//     coast, 3 ms with a band of 16) and fill and fold work (a 256^2 step 4.1 ms with 22400
//     ghosts, 2.9 ms without walls). A wall thinner than about two tapers is mostly tapered, so
//     it reflects partly as a zero wall would and leaks (behind a 3 m band 10.8 % of the incident
//     amplitude, 6 m 5.7 %, 12 m 3.0 %; zero walls 4.6 % and 2.8 %).
//   zero: h = phi = 0 on walls, a Dirichlet wall (a string's fixed end): a crest reflects as a
//     trough. O8a's behaviour, bit for bit: the exact step.
//
// Sources (add_height, add_heights) go straight into h between steps, times the mask, so water
// never appears on land. add_heights takes C2's whole-field -(zeta(t) - zeta(t - dt)).
//
// k = 0 (the exact step): the mean height has kappa = 0 and stays (but for the global damping).
// Exactly, the mean potential would drift by -g mean(h) dt a step, but a uniform potential moves
// no water, and the mask would turn that drift into a step of phi at every shore, so the mean
// potential is held at 0 instead (a choice of gauge: the heights are those of the exact update).
// The symmetric step needs no gauge: a uniform potential on the water is not uniform on the
// window unless every wall cell is a ghost, and S sees it. (Where every wall cell is a ghost, a
// mean height makes the mean potential drift linearly, as it would exactly, and S ignores it.)
//
// Energy (energy()): without ghosts, the water's, over the cells whose mask is above 0:
// potential 0.5 rho g sum h^2 dA, kinetic 0.5 rho sum over k of kappa |PHI(k)|^2 dA / n^2 (walls
// hold phi = 0 after a step). With ghosts, the symmetric step's H in joules,
// 0.5 rho (g h.M.h + phi.S.phi) dA with S built on kappa (not kappa~, so it does not depend on
// dt): the water and its mirror image (beside a straight wall, the water's energy counted twice).
// Without damping, beach or sponge, the exact step conserves it to round-off, the symmetric step
// up to the wobble above.
//
// Determinism: the coefficients and the sponge use cvc::algo::det math, the FFT is fft.h's, and
// the module is built without FMA contraction, so a window gives the same bits on every platform
// for the same inputs and steps; digest() hashes the parameters, the mask and the state. A window
// may be read from several threads while nothing writes to it.
//
// Standard library only, like the rest of cvc::ocean.

#ifndef CVC_OCEAN_EWAVE_H
#define CVC_OCEAN_EWAVE_H

#include <cstdint>
#include <cvc/ocean/fft.h>
#include <vector>

namespace cvc {
namespace ocean {

// What a wall (a mask value of exactly 0) does: see the opening comment.
enum class ewave_walls : int {
  no_flux = 0, // a real wall: a crest reflects as a crest (the symmetric step on ghost cells)
  zero = 1     // h = phi = 0: a crest reflects as a trough (O8a)
};

struct ewave_params {
  int n = 256;                // cells a side (a power of two, 4..4096)
  double dx = 0.5;            // cell size (m)
  double origin_x = 0.0;      // cell (0, 0)'s position (m, world x and y)
  double origin_y = 0.0;      //
  double depth = 1000.0;      // the window's one water depth (m; +infinity is deep water)
  double gravity = 9.80665;   // m/s^2
  double density = 1025.0;    // kg/m^3, for energy()
  double damping_k2 = 0.0;    // gamma: modes decay as exp(-gamma |k|^2 dt) (m^2/s)
  double damping = 0.0;       // and all of them as exp(-damping dt) (1/s)
  double sponge_cells = 16.0; // the edge band's width (cells, at most n / 2; 0: no sponge)
  double sponge_rate = 2.0;   // its damping rate at the edge (1/s)
  double beach_rate = 10.0;   // a mask value m in (0, 1) damps at beach_rate (1 - m) (1/s)
  ewave_walls walls = ewave_walls::no_flux; // crest for crest (see above)
  int ghost_cells = 4096; // no_flux: the wall cells within this many cells of the shore
                          // hold the water's mirror image (0..4096, at most n is used; the
                          // default: as far as the image reaches; 0 makes walls zero)
};

// The most ghosts that may read one water cell (a nonzero weight on it): EWave (cvcGL) gathers
// them per water texel. ewave_wall_ghosts drops whole ghosts beyond it.
constexpr int ewave_max_readers = 12;

// How far (cells) the image tapers to 0 before a cut or a seam (see ewave_wall_ghosts).
constexpr int ewave_taper_cells = 8;

// The largest omega dt a symmetric substep takes (Stormer-Verlet is stable below 2).
constexpr double ewave_max_phase = 1.8;

// One wall cell's ghost: its mirror point across the nearest shore, as the bilinear corners
// around it. corner is the lower corner (i0, j0) as a row-major index; w weighs (i0, j0),
// (i0 + 1, j0), (i0, j0 + 1) and (i0 + 1, j0 + 1), is 0 on wall corners and sums to 1, or to the
// taper's factor (in (0, 1)) near a cut or a seam.
struct ewave_ghost {
  std::uint32_t cell;
  std::uint32_t corner;
  double w[4];
};

// The ghosts of an n x n mask (row-major, a wall where the value is exactly 0), for the wall
// cells within ghost_cells of the shore, in row-major order of their cells. For a wall cell p,
// q is the nearest non-wall cell centre (no wrap round the window), at |q - p| below
// ghost_cells + 1/2 cells; the shore lies half a cell from q, so with n = (q - p) / |q - p| and
// s = |q - p| - 1/2 the mirror point is p' = p + 2 s n. When several cells are nearest, p' is
// the mean of their mirror points, over those whose direction is within 90 degrees of the first
// found (the first wall layer of a 45-degree staircase then mirrors exactly across its shore
// line; the far side of a thin wall is left to its own cells). A cell gets no ghost when p' is
// outside the window or all four corners are walls. Then, while some water cell is read (a
// nonzero weight) by more than max_readers ghosts (0: no limit), whole ghosts are dropped,
// farthest from the shore first (ties: the higher cell index first), each only if a cell it
// reads is still over. Last, the taper: a ghost within ewave_taper_cells T of a cut (a wall cell
// left without a ghost) or a seam (a ghost whose mirror point is more than 2.5 cells from a
// 4-neighbour's, a water neighbour's being the neighbour itself) has its weights scaled by
// sin^2(pi/2 D / T), D the distance to the nearest cut cell's centre, or to the nearest seam
// cell's plus half a cell (the smaller factor of the two). Deterministic: integer search, then
// IEEE arithmetic and det math only. A straight wall mirrors exactly: with water up to cell -1
// and wall from cell 0 on, wall cell m mirrors onto cell -1 - m with weight 1 (when nothing is
// cut within T). EWave (cvcGL) uploads this table.
std::vector<ewave_ghost> ewave_wall_ghosts(int n, const std::vector<double> &mask, int ghost_cells,
                                           int max_readers = ewave_max_readers);

// The lumped mass M of the symmetric step for an n x n mask and its ghosts: per water cell (mask
// above 0) the row sum of R^T R, 1 + for each ghost that reads it (a nonzero weight) its weight
// on it times the ghost's total weight; 0 on walls. ewave and EWave (cvcGL) both use this.
std::vector<double> ewave_lumped_mass(int n, const std::vector<double> &mask,
                                      const std::vector<ewave_ghost> &ghosts);

// The substeps the symmetric step takes for a step of dt (s, above 0 and finite): the fewest
// equal ones that keep omega_max dt / s <= ewave_max_phase, omega_max = omega at the window's
// corner wavenumber sqrt(2) pi / dx. Throws std::invalid_argument past 10^6.
int ewave_substeps(const ewave_params &p, double dt);

class ewave {
public:
  // Throws std::invalid_argument for parameters out of range.
  explicit ewave(const ewave_params &p);

  const ewave_params &params() const noexcept { return _p; }
  int size() const noexcept { return _p.n; }

  // The mask, n x n values in [0, 1], row-major. Throws std::invalid_argument for a wrong size or
  // a value out of range. It applies from the next step (and to sources added after it). Under
  // no_flux walls it also works out the ghosts (ewave_wall_ghosts) and the lumped mass.
  void set_mask(const std::vector<double> &mask);
  const std::vector<double> &mask() const noexcept { return _mask; }
  // The wall cells' ghosts (empty under zero walls, or without walls near water): R's rows.
  const std::vector<ewave_ghost> &ghosts() const noexcept { return _ghosts; }
  // The lumped mass M per cell (ewave_lumped_mass: 1 + the weights of the ghosts that read it,
  // on water; 0 on walls; empty without ghosts).
  const std::vector<double> &mass() const noexcept { return _mass; }

  // Replaces the state: n x n heights (m) and potentials (m^2/s), row-major. Not masked; with
  // ghosts the next step zeroes the walls (they hold no state).
  void set_state(const std::vector<double> &h, const std::vector<double> &phi);
  // Adds amount (m) at (x, y) in world coordinates, spread over a smooth bump of the given radius
  // (m): amount (1 - r^2 / radius^2)^3 within it, times the mask. Throws std::invalid_argument
  // for a radius not above 0 or a value that is not finite.
  void add_height(double x, double y, double amount, double radius);
  // Adds n x n heights (m), row-major, times the mask.
  void add_heights(const std::vector<double> &dh);

  // Moves the waves on by dt (s, above 0 and finite).
  void step(double dt);
  // The substeps step(dt) takes: 1 without ghosts (the exact step), else ewave_substeps.
  int substeps(double dt) const;

  // The state, n x n row-major (0 on walls after a step).
  const std::vector<double> &height() const noexcept { return _h; }
  const std::vector<double> &phi() const noexcept { return _phi; }
  // The height at (x, y) in world coordinates, bilinear between the cells; 0 outside the window.
  double at(double x, double y) const;
  // Cell (i, j)'s position.
  double x_of(int i) const noexcept { return _p.origin_x + double(i) * _p.dx; }
  double y_of(int j) const noexcept { return _p.origin_y + double(j) * _p.dx; }
  // The time stepped so far (s).
  double time() const noexcept { return _time; }

  // The wave energy (J), potential and kinetic, as in the opening comment (with ghosts: the
  // symmetric step's H, the water and its image).
  double potential_energy() const;
  double kinetic_energy() const;
  double energy() const { return potential_energy() + kinetic_energy(); }

  // A 64-bit FNV-1a digest of the parameters, the mask and the state. Under zero walls the walls
  // and ghost_cells are left out, so O8a's digests stand.
  std::uint64_t digest() const;

  // omega(k) for this window's depth and gravity (rad/s, k in rad/m).
  double omega(double k) const;

private:
  void prepare(double dt);           // the exact step's per-mode coefficients for dt
  void prepare_symmetric(double dt); // the symmetric step's substeps and multipliers for dt
  void update_real(double dt);       // the real-space factor for dt
  void exact_step(double dt);
  void symmetric_step(double dt);
  // z's real (and, if both, imaginary) parts at the ghost cells from their corners in a (and b).
  void fill(double *z, const double *a, const double *b) const;
  // acc = R^T of z's real part (acc2: of its imaginary part, if not null), on the water.
  void fold(const double *z, double *acc, double *acc2) const;

  ewave_params _p;
  fft_plan _fft;
  std::vector<double> _h, _phi, _mask;
  bool _masked = false;            // some mask value is below 1
  std::vector<double> _k2, _kappa; // per mode: |k|^2 and kappa
  std::vector<double> _z;          // the complex work field, (re, im) interleaved
  double _dt = 0.0;                // the dt the coefficients below were computed for (0: none)
  std::vector<double> _cf;         // per mode: H' = cf[0] H + cf[1] PHI, PHI' = cf[2] H + cf[3] PHI
  std::vector<double> _real;       // per cell: the mask's and the sponge's factor for _real_dt
  double _real_dt = 0.0;           // the dt _real was computed for (0: none)
  bool _real_ones = true;          // every _real value is 1: the real-space pass is skipped
  bool _real_stale = true;         // the mask changed since _real was computed
  // The symmetric step (no_flux walls with ghosts).
  std::vector<ewave_ghost> _ghosts;     // R's ghost rows
  std::vector<double> _mass, _inv_mass; // per cell: M and 1 / M on water, 0 on walls
  double _sym_dt = 0.0;                 // the dt below were computed for (0: none)
  int _sym_n = 1;                       // substeps
  double _sym_dts = 0.0;                // the substep
  double _sym_damp = 1.0;               // exp(-damping dt)
  std::vector<double> _gk;              // per mode: kappa~ / n^2 for the substep
  std::vector<double> _fk;              // per mode: exp(-damping_k2 |k|^2 dt) / n^2
  std::vector<double> _acc, _acc2;      // per cell: folds
  double _time = 0.0;
};

} // namespace ocean
} // namespace cvc

#endif // CVC_OCEAN_EWAVE_H
